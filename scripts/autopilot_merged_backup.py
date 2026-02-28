"""
autopilot.py — Autonomous Rover with PID Heading + Slalom Proportional Dodge.

Merges the PID heading-corrected cruise from autopilot_pid.py with the
proportional differential cone-dodging strategy from slalom.py for smarter,
smoother obstacle avoidance.

4-State Finite State Machine
─────────────────────────────
  FORWARD_CRUISE       Drive forward with gyro-based PID heading correction.
  OBSTACLE_STEERING    Proportional differential dodge — slalom-style
                       TURN_AGGRESSION left/right speed split combined with
                       IR hard-swerve for close-range side obstacles.
  OBSTACLE_FALLBACK    Last-resort: stop → reverse → spin.  Uses random spin
                       direction when both IR sensors are blocked (slalom
                       trap-escape), directed spin otherwise.
  EMERGENCY_STOP       Cliff detected or manual kill — all motors locked.

Smart Steering (Slalom-Integrated)
──────────────────────────────────
  When front sonar detects an obstacle within STEER_DETECT_CM:
  1. Speed is proportionally reduced based on distance.
  2. IR sensors choose dodge direction (away from blocked side).
  3. Proportional differential: avoid_strength = (STEER_DETECT_CM - dist) * TURN_AGGRESSION
     applied as left/right speed split (from slalom.py's cone-dodge algorithm).
  4. IR hard swerve (60/20 split) overrides proportional dodge when a single IR fires.
  5. MPU6050 gyro tracks heading offset for course-return after clearing.
  6. If distance drops below STEER_FALLBACK_CM or times out, falls back to
     stop → reverse → spin sequence.

  Trap Escape (from slalom.py):
  - Both IR sensors triggered OR sonar < TRAP_ESCAPE_CM → immediate fallback
    with random spin direction (breaks corner-trapping loops).

Hardware
────────
  Motors (L298N)    IN1=17, IN2=27, ENA=12 (PWM0)
                    IN3=22, IN4=23, ENB=13 (PWM1)  @ 1000 Hz
  MPU6050           I2C Bus 1, Address 0x68 — Gyro Z-axis yaw rate
  Front Sonar       Trig=25, Echo=24
  Rear Sonar        Trig=20, Echo=16
  IR Edge           Left=GPIO5, Right=GPIO6  (Active LOW = obstacle)

PID Logic
─────────
  Target yaw rate = 0 °/s (straight line).
  Gyro_Z drifting left  → increase left motor speed.
  Gyro_Z drifting right → increase right motor speed.

Wraps the existing CarSystem (motor.py) and SensorSystem (sensors.py)
through MotorDriver and Sonar helper classes for clean abstraction.
"""

import time
import random
import threading
from collections import deque
from enum import Enum


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    FORWARD_CRUISE      = "FORWARD_CRUISE"
    OBSTACLE_STEERING   = "OBSTACLE_STEERING"
    OBSTACLE_FALLBACK   = "OBSTACLE_FALLBACK"
    EMERGENCY_STOP      = "EMERGENCY_STOP"


# ── PID Controller ─────────────────────────────────────

class PIDController:
    """Discrete PID controller with anti-windup clamping."""

    def __init__(self, kp=2.0, ki=0.0, kd=0.5, output_limit=30.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error, dt):
        """Compute PID output given current error and timestep (seconds)."""
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        max_integral = self.output_limit / max(self.ki, 0.001)
        self._integral = max(-max_integral, min(max_integral, self._integral))
        i = self.ki * self._integral

        # Derivative
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Sum and clamp
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        """Zero all internal state."""
        self._integral = 0.0
        self._prev_error = 0.0


# ── MPU6050 Sensor ─────────────────────────────────────

class MPU6050Sensor:
    """
    Reads Gyro Z-axis yaw rate from the MPU6050 over I2C.

    Tries the high-level `mpu6050` pip library first; falls back to raw
    smbus register reads if the library is unavailable.

    I2C Address: 0x68 (default)
    Register Map:
        0x6B  PWR_MGMT_1  — write 0x00 to wake from sleep
        0x47  GYRO_ZOUT_H — high byte of Z-axis gyro
        0x48  GYRO_ZOUT_L — low byte of Z-axis gyro
        0x75  WHO_AM_I    — should read 0x68
    """

    # Sensitivity for ±250 °/s range (default after reset)
    _GYRO_SCALE = 131.0  # LSB per °/s

    def __init__(self, address=0x68, bus_number=1):
        self._address = address
        self._bus_number = bus_number
        self._offset_z = 0.0
        self._available = False
        self._use_lib = False
        self._sensor = None
        self._bus = None

        # ── Try high-level library first ──
        try:
            from mpu6050 import mpu6050 as MPU6050Lib
            self._sensor = MPU6050Lib(self._address)
            # Quick read to verify hardware is present
            self._sensor.get_gyro_data()
            self._use_lib = True
            self._available = True
            print("🧭 MPU6050: Connected (mpu6050 library)")
            return
        except Exception:
            pass

        # ── Fallback: raw smbus ──
        try:
            import smbus2 as smbus_mod
        except ImportError:
            try:
                import smbus as smbus_mod
            except ImportError:
                print("⚠️  MPU6050: No I2C library (install smbus2 or mpu6050)")
                return

        try:
            self._bus = smbus_mod.SMBus(self._bus_number)
            # Wake the sensor (clear sleep bit in PWR_MGMT_1)
            self._bus.write_byte_data(self._address, 0x6B, 0x00)
            time.sleep(0.05)
            # Verify identity
            who = self._bus.read_byte_data(self._address, 0x75)
            if who not in (0x68, 0x72):  # MPU6050 or MPU6052
                print(f"⚠️  MPU6050: Unexpected WHO_AM_I=0x{who:02X}")
            self._available = True
            print("🧭 MPU6050: Connected (raw smbus)")
        except Exception as e:
            print(f"⚠️  MPU6050: I2C init failed — {e}")

    # ── Raw register helpers ───────────────────

    def _read_raw_gyro_z(self):
        """Read signed 16-bit Gyro-Z from registers 0x47-0x48."""
        high = self._bus.read_byte_data(self._address, 0x47)
        low = self._bus.read_byte_data(self._address, 0x48)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value / self._GYRO_SCALE

    # ── Public API ─────────────────────────────

    @property
    def available(self):
        return self._available

    def read_gyro_z(self):
        """
        Return drift-corrected Z-axis yaw rate in °/s.

        Positive = rotating clockwise (drifting right).
        Negative = rotating counter-clockwise (drifting left).
        Returns 0.0 if sensor is unavailable or on I2C error.
        """
        if not self._available:
            return 0.0
        try:
            if self._use_lib:
                data = self._sensor.get_gyro_data()
                raw = data['z']
            else:
                raw = self._read_raw_gyro_z()
            return raw - self._offset_z
        except Exception:
            return 0.0

    def calibrate(self, duration=2.0):
        """
        Measure gyro Z drift while the robot is stationary.

        Reads at ~100 Hz for *duration* seconds, computes the mean bias,
        and stores it as the zero offset.  The robot MUST be still.
        """
        if not self._available:
            print("🧭 MPU6050: Unavailable — skipping calibration (heading correction disabled)")
            return

        print(f"🧭 [CALIBRATING] Stand still for {duration:.0f}s — measuring gyro drift...")
        samples = []
        start = time.time()
        while time.time() - start < duration:
            try:
                if self._use_lib:
                    data = self._sensor.get_gyro_data()
                    samples.append(data['z'])
                else:
                    samples.append(self._read_raw_gyro_z())
            except Exception:
                pass
            time.sleep(0.01)  # ~100 Hz

        if samples:
            self._offset_z = sum(samples) / len(samples)
            print(f"🧭 [CALIBRATED] {len(samples)} samples — drift offset = {self._offset_z:+.3f} °/s")
        else:
            self._offset_z = 0.0
            print("🧭 [CALIBRATED] No samples collected — offset = 0.0")


# ── Sonar Wrapper ──────────────────────────────────────

class Sonar:
    """
    Wraps a sonar read callable with timeout protection and median filtering.

    The underlying callable (from SensorSystem) already has its own
    0.1 s per-pulse timeout.  This class adds a hard 0.15 s thread-level
    timeout as a safety net, plus a 2-sample rolling median filter to
    reject single-reading noise spikes with minimal lag.
    """

    def __init__(self, get_distance_fn, name="sonar"):
        self._read = get_distance_fn
        self.name = name
        self._history = deque(maxlen=2)

    def read(self):
        """
        Return filtered distance in cm.

        -1  → timeout / sensor error
        >0  → valid distance

        The read is executed inside a daemon thread with a 0.15 s hard
        timeout so a hung I/O bus can never freeze the main loop.
        """
        if self._read is None:
            return -1

        result = [None]

        def _worker():
            try:
                result[0] = self._read()
            except Exception:
                result[0] = -1

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        t.join(timeout=0.15)

        raw = result[0] if result[0] is not None else -1

        # Feed valid readings into the median filter
        if raw >= 0:
            self._history.append(raw)

        if len(self._history) == 0:
            return raw

        # Return median of buffered readings
        buf = sorted(self._history)
        return buf[len(buf) // 2]


# ── Motor Driver Wrapper ──────────────────────────────

class MotorDriver:
    """
    High-level motor commands with PID differential support.

    Wraps CarSystem and adds the ability to apply an asymmetric
    correction to left/right wheels for straight-line heading control.
    """

    def __init__(self, car, min_correction=15.0, noise_gate=1.0):
        """
        Parameters
        ----------
        car : motor.CarSystem
            The existing low-level motor controller.
        min_correction : float
            Minimum PWM differential when PID requests a correction.
            Ensures motors overcome static friction / weight.
        noise_gate : float
            PID output below this is treated as zero (gyro noise filter).
        """
        self._car = car
        self._min_correction = min_correction
        self._noise_gate = noise_gate

    def forward(self, base_speed, correction=0.0):
        """
        Drive forward with differential PID heading correction.

        correction > 0 → drifting right → boost left, slow right.
        correction < 0 → drifting left  → boost right, slow left.

        Parameters
        ----------
        base_speed : float
            Base PWM % (0-100).
        correction : float
            PID output added/subtracted to left/right wheels.
        """
        # Apply minimum correction deadband to overcome motor friction/weight.
        # Noise gate: ignore tiny PID outputs (gyro noise).
        # If PID wants a real correction, enforce at least MIN_CORRECTION_PWM differential.
        if abs(correction) < self._noise_gate:
            correction = 0.0
        elif 0 < correction < self._min_correction:
            correction = self._min_correction
        elif -self._min_correction < correction < 0:
            correction = -self._min_correction

        left_speed = max(0, min(100, base_speed + correction))
        right_speed = max(0, min(100, base_speed - correction))
        self._car._set_raw_motors(left_speed, right_speed, True, True)
        self._car._current_speed = base_speed

    def forward_differential(self, left_speed, right_speed):
        """
        Drive forward with explicit left/right wheel speeds.

        Maps slalom.py's ``set_motors(l_speed, r_speed, direction=1)``
        to the CarSystem abstraction.  Speeds are clamped to 0-100 %.

        Parameters
        ----------
        left_speed : float
            Left-side PWM % (0-100).
        right_speed : float
            Right-side PWM % (0-100).
        """
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        self._car._set_raw_motors(left_speed, right_speed, True, True)
        self._car._current_speed = (left_speed + right_speed) / 2.0

    def reverse(self, speed, duration=0.8):
        """Reverse at *speed* for *duration* seconds, then stop.

        Calls reverse() in a 50 Hz loop so the PWM ramp in
        _set_raw_motors (MAX_PWM_DELTA_PER_TICK = 5 %/call) actually
        reaches the target speed.  A single call would only ramp
        from 0 % to 5 %.
        """
        end = time.time() + duration
        while time.time() < end:
            self._car.reverse(speed)
            time.sleep(0.02)          # 50 Hz — matches physics loop
        self._car.stop()

    def spin_right(self, speed=50, duration=0.4):
        """Tank-turn right (left fwd / right back) for *duration* s.

        Loops at 50 Hz so the PWM ramp reaches the target (same
        reason as reverse — single call caps at +5 %).
        """
        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("right", speed)
            time.sleep(0.02)          # 50 Hz
        self._car.stop()

    def spin_left(self, speed=50, duration=0.4):
        """Tank-turn left (right fwd / left back) for *duration* s.

        Mirror of spin_right for slalom trap-escape random direction.
        """
        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("left", speed)
            time.sleep(0.02)          # 50 Hz
        self._car.stop()

    def stop(self):
        """Gentle coast stop (all pins LOW, PWM 0)."""
        self._car.stop()

    def forward_steer(self, speed, angle):
        """
        Drive forward with a steering angle (like a car turn).

        Uses differential speed: inner wheel slows, outer stays at full.

        Parameters
        ----------
        speed : float
            Forward PWM % (0-100).
        angle : float
            Steering angle. Negative = left, positive = right.
            Magnitude 0-90; deadzone ±5° treated as straight.
        """
        turn_factor = abs(angle) / 90.0
        inner = speed * (1.0 - turn_factor * 0.9)  # drops to 10% at ±90°

        left_speed = speed
        right_speed = speed

        if angle < -5:      # turning LEFT → slow left wheel
            left_speed = inner
        elif angle > 5:     # turning RIGHT → slow right wheel
            right_speed = inner

        self._car._set_raw_motors(
            max(0, min(100, left_speed)),
            max(0, min(100, right_speed)),
            True, True
        )
        self._car._current_speed = speed

    def brake(self):
        """Magnetic lock brake (H-bridge short-circuit)."""
        self._car.brake()


# ── Autonomous Rover (AutoPilot) ──────────────────────

class AutoPilot:
    """
    4-State Autonomous Rover with slalom-integrated smart steering.

    Combines PID heading cruise with slalom.py's proportional differential
    dodge (TURN_AGGRESSION) and trap-escape logic for robust obstacle avoidance.

    States
    ------
    FORWARD_CRUISE      Drive forward; PID keeps heading straight.
    OBSTACLE_STEERING   Slalom proportional dodge — TURN_AGGRESSION-based
                        left/right speed differential.  IR hard swerve
                        (60/20 split) overrides for close side obstacles.
    OBSTACLE_FALLBACK   Stop → reverse → spin.  Random spin when both IRs
                        blocked (slalom trap-escape); directed spin otherwise.
    EMERGENCY_STOP      Cliff/edge detected or manual kill.

    Parameters
    ----------
    car : motor.CarSystem
        Low-level motor controller.
    get_sonar : callable → float
        Returns front sonar distance in cm.
    get_ir : callable → (bool, bool)
        Returns (left_obstacle, right_obstacle).  True = obstacle detected.
    get_rear_sonar : callable → float | None
        Returns rear sonar distance in cm (optional).
    get_camera_distance : callable → float | None
        Accepted for API compatibility; not used by the rover FSM.
    """

    # ── Tuning constants (exposed to UI via TUNING_KEYS) ──

    # ── Cruise tuning ──
    BASE_SPEED       = 50     # Forward cruise PWM %
    REAR_CLEAR_CM    = 20     # Rear must be > this to reverse (cm)
    PID_KP           = 5.0    # PID proportional gain (aggressive to overcome friction)
    PID_KI           = 0.0    # PID integral gain
    PID_KD           = 0.5    # PID derivative gain
    PID_LIMIT        = 30.0   # Max PID correction (PWM %)
    STATUS_INTERVAL  = 0.5    # Status print interval (seconds)
    CALIBRATION_TIME = 2.0    # Gyro calibration duration (seconds)

    # ── Minimum correction deadband ──
    MIN_CORRECTION_PWM   = 15.0   # Min differential when PID says "correct" (overcomes friction)
    CORRECTION_NOISE_GATE = 1.0   # Ignore PID output below this (gyro noise filter)

    # ── Smart steering avoidance ──
    STEER_DETECT_CM  = 90     # Begin slowing + steering at this distance (cm)
    STEER_ACTIVE_CM  = 60     # Steering becomes more aggressive below this (cm)
    STEER_FALLBACK_CM = 25    # Give up steering, fall back to stop-reverse (cm)
    STEER_CLEAR_CM   = 100    # Path is clear above this distance (cm)
    STEER_ANGLE_MIN  = 15     # Min avoidance steering angle (degrees)
    STEER_ANGLE_MAX  = 55     # Max avoidance steering angle (degrees)
    STEER_SPEED_MIN  = 25     # Min forward speed during avoidance (PWM %)
    STEER_TIMEOUT    = 5.0    # Max seconds in OBSTACLE_STEERING before fallback
    HEADING_KP       = 1.5    # Proportional gain for heading return after clearing
    HEADING_TOLERANCE = 3.0   # Degrees — heading considered restored below this
    EMERGENCY_STOP_CM = 10    # Hard-stop safety net — immediate brake below this (cm)

    # ── Slalom proportional dodge (from slalom.py) ──
    TURN_AGGRESSION       = 1.8    # Multiplier: avoid_strength = (detect_cm - dist) * this
    IR_HARD_SWERVE_FAST   = 60     # Fast-side PWM % for IR hard swerve
    IR_HARD_SWERVE_SLOW   = 20     # Slow-side PWM % for IR hard swerve
    TRAP_ESCAPE_CM        = 15     # Both-IR or sonar < this → immediate fallback
    RANDOM_SPIN_ON_BOTH_IR = True  # Use random spin direction when both IRs blocked

    # ── Fallback (stop-reverse-spin) ──
    FRONT_DANGER_CM  = 35     # Legacy threshold (used for fallback only)
    REVERSE_SPEED    = 70     # Reverse PWM % (strong push-back)
    REVERSE_DURATION = 1.2    # Reverse time (seconds)
    SPIN_SPEED       = 60     # Spin/pivot PWM %
    SPIN_DURATION    = 0.6    # Spin-right time (seconds)
    STOP_PAUSE       = 0.5    # Pause before fallback maneuver (seconds)

    TUNING_KEYS = [
        "BASE_SPEED", "REAR_CLEAR_CM",
        "PID_KP", "PID_KI", "PID_KD", "PID_LIMIT",
        "STATUS_INTERVAL", "CALIBRATION_TIME",
        "MIN_CORRECTION_PWM", "CORRECTION_NOISE_GATE",
        # Smart steering
        "STEER_DETECT_CM", "STEER_ACTIVE_CM", "STEER_FALLBACK_CM",
        "STEER_CLEAR_CM", "STEER_ANGLE_MIN", "STEER_ANGLE_MAX",
        "STEER_SPEED_MIN", "STEER_TIMEOUT", "HEADING_KP", "HEADING_TOLERANCE",
        "EMERGENCY_STOP_CM",
        # Slalom proportional dodge
        "TURN_AGGRESSION", "IR_HARD_SWERVE_FAST", "IR_HARD_SWERVE_SLOW",
        "TRAP_ESCAPE_CM", "RANDOM_SPIN_ON_BOTH_IR",
        # Fallback
        "FRONT_DANGER_CM", "REVERSE_SPEED", "REVERSE_DURATION",
        "SPIN_SPEED", "SPIN_DURATION", "STOP_PAUSE",
    ]

    def __init__(self, car, get_sonar, get_ir,
                 get_rear_sonar=None, get_camera_distance=None):
        # Wrap low-level motor controller (with minimum correction deadband)
        self._motor = MotorDriver(car, min_correction=self.MIN_CORRECTION_PWM,
                                  noise_gate=self.CORRECTION_NOISE_GATE)
        self._car = car  # Keep direct ref for speed telemetry

        # Wrap sonar callables with timeout + median filter
        self._front_sonar = Sonar(get_sonar, "front")
        self._rear_sonar = Sonar(get_rear_sonar, "rear")

        # IR callable: returns (left_cliff, right_cliff)
        self._get_ir = get_ir

        # Camera distance callable — fused with sonar for better obstacle detection
        self._get_camera_distance = get_camera_distance

        # MPU6050 gyroscope
        self._gyro = MPU6050Sensor()

        # PID controller for heading correction
        self._pid = PIDController(
            kp=self.PID_KP,
            ki=self.PID_KI,
            kd=self.PID_KD,
            output_limit=self.PID_LIMIT,
        )

        # State
        self._state = State.FORWARD_CRUISE
        self._active = False
        self._turn_direction = ""
        self._last_status_time = 0.0
        self._last_pid_time = 0.0

        # Smart steering state
        self._steer_direction = ""        # "left" or "right" during OBSTACLE_STEERING
        self._steer_entry_time = 0.0      # Timestamp when steering avoidance started
        self._heading_offset = 0.0        # Accumulated yaw drift in degrees
        self._returning_to_heading = False # True while correcting back to original heading

        # Slalom dodge state (from slalom.py)
        self._last_steer_dir = "right"    # Remembers last dodge direction for alternation
        self._both_ir_at_fallback = False  # True when fallback entered due to both IRs blocked

        # Telemetry (exposed for UI)
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        self._gyro_calibrated = False

    # ── Properties (read-only for main.py) ─────

    @property
    def state(self):
        return self._state

    @property
    def is_active(self):
        return self._active

    @property
    def turn_direction(self):
        return self._turn_direction

    @property
    def gyro_z(self):
        """Last read Z-axis yaw rate in °/s (for telemetry)."""
        return self._last_gyro_z

    @property
    def pid_correction(self):
        """Last PID heading correction output (for telemetry)."""
        return self._last_pid_correction

    @property
    def gyro_available(self):
        """Whether the MPU6050 hardware is connected."""
        return self._gyro.available

    @property
    def gyro_calibrated(self):
        """Whether the gyro has been calibrated."""
        return self._gyro_calibrated

    # ── Tuning API ─────────────────────────────

    @classmethod
    def get_default_tuning(cls):
        """Return compile-time default values for all tuning keys."""
        return {key: getattr(cls, key) for key in cls.TUNING_KEYS}

    def get_tuning(self):
        """Return current instance values for all tuning keys."""
        return {key: getattr(self, key) for key in self.TUNING_KEYS}

    # ── Lifecycle ──────────────────────────────

    def start(self):
        """Calibrate gyro, reset PID, begin FORWARD_CRUISE."""
        self._gyro.calibrate(duration=self.CALIBRATION_TIME)
        self._gyro_calibrated = self._gyro.available
        self._pid = PIDController(
            kp=self.PID_KP,
            ki=self.PID_KI,
            kd=self.PID_KD,
            output_limit=self.PID_LIMIT,
        )
        self._state = State.FORWARD_CRUISE
        self._turn_direction = ""
        self._steer_direction = ""
        self._steer_entry_time = 0.0
        self._heading_offset = 0.0
        self._returning_to_heading = False
        self._both_ir_at_fallback = False
        self._last_status_time = time.time()
        self._last_pid_time = time.time()
        self._active = True
        print("🚀 [ROVER] Autonomous navigation STARTED (slalom-integrated)")

    def stop(self):
        """Halt all motors and deactivate FSM."""
        self._active = False
        self._motor.brake()
        self._pid.reset()
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        self._heading_offset = 0.0
        self._returning_to_heading = False
        self._steer_direction = ""
        self._both_ir_at_fallback = False
        print("🛑 [ROVER] Autonomous navigation STOPPED")

    # ── Sensor helpers ─────────────────────────

    def _read_ir(self):
        """Read IR sensors. Returns (left_obstacle, right_obstacle)."""
        try:
            return self._get_ir()
        except Exception:
            return (False, False)

    def _choose_steer_direction(self):
        """
        Pick left or right for obstacle avoidance using IR proximity.

        IR active-LOW: True = obstacle/proximity detected on that side.
        Steer AWAY from the blocked side.

        Returns
        -------
        str
            "left" or "right"
        """
        left_blocked, right_blocked = self._read_ir()
        if left_blocked and not right_blocked:
            return "right"   # obstacle on left → steer right
        if right_blocked and not left_blocked:
            return "left"    # obstacle on right → steer left
        # Both clear or both blocked → alternate left/right to avoid corner-trapping
        self._last_steer_dir = 'left' if self._last_steer_dir == 'right' else 'right'
        return self._last_steer_dir

    def _interpolate(self, dist, far_cm, near_cm, far_val, near_val):
        """
        Linear interpolation between two distance thresholds.

        Returns far_val when dist >= far_cm, near_val when dist <= near_cm,
        and linearly interpolated in between.
        """
        if dist >= far_cm:
            return far_val
        if dist <= near_cm:
            return near_val
        t = (far_cm - dist) / max(far_cm - near_cm, 0.01)
        return far_val + t * (near_val - far_val)

    def get_rear_distance(self):
        """Public accessor for rear sonar (used by telemetry if needed)."""
        return self._rear_sonar.read()

    # ── Status printer ─────────────────────────

    def _maybe_print_status(self, front_dist, gyro_z, correction, extra=""):
        """Print status line at STATUS_INTERVAL frequency."""
        now = time.time()
        if now - self._last_status_time >= self.STATUS_INTERVAL:
            self._last_status_time = now
            state_name = self._state.value
            heading_str = f" | Heading: {self._heading_offset:+.1f}°" if self._heading_offset != 0 else ""
            extra_str = f" | {extra}" if extra else ""
            print(f"📡 [{state_name}] Front: {front_dist:.0f}cm | "
                  f"Gyro_Z: {gyro_z:+.2f}°/s | "
                  f"PID: {correction:+.1f}{heading_str}{extra_str}")

    # ── FSM Core ───────────────────────────────

    def update(self):
        """
        Single tick of the 4-state FSM.

        Called by the drive_autonomous thread at ~20 Hz.

        OBSTACLE_STEERING is non-blocking (runs per-tick for smooth
        proportional differential dodge).
        OBSTACLE_FALLBACK is a blocking sequence (~1.7 s) which is
        acceptable since it runs in a dedicated daemon thread.
        """
        if not self._active:
            return

        # ── STATE: EMERGENCY_STOP ──
        if self._state == State.EMERGENCY_STOP:
            # Stay locked.  Only start() can exit this state.
            self._motor.brake()
            return

        # ── Read gyro for all active states ──
        now = time.time()
        dt = now - self._last_pid_time
        self._last_pid_time = now
        gyro_z = self._gyro.read_gyro_z()
        self._last_gyro_z = gyro_z

        # ── Track heading offset (integrate yaw rate) ──
        if dt > 0 and dt < 0.5:  # ignore huge gaps (startup, pause)
            self._heading_offset += gyro_z * dt

        # ── Read front sonar (shared across all active states) ──
        front_dist = self._front_sonar.read()

        # ── Fuse camera distance with sonar ──
        # Use min(sonar, camera) — conservative: always trust the closer reading
        if self._get_camera_distance is not None:
            try:
                cam_dist = self._get_camera_distance()
                if cam_dist is not None and cam_dist > 5:  # filter noise/invalid
                    if front_dist < 0:
                        front_dist = cam_dist  # sonar failed, use camera
                    else:
                        front_dist = min(front_dist, cam_dist)
            except Exception:
                pass  # camera error — use sonar only

        # ── Read IR sensors (shared across states) ──
        left_ir, right_ir = self._read_ir()

        # ── EMERGENCY BRAKE: safety net across ALL states ──
        if 0 < front_dist < self.EMERGENCY_STOP_CM:
            self._motor.brake()
            print(f"\n🚨 [EMERGENCY BRAKE] Obstacle at {front_dist:.0f}cm "
                  f"(< {self.EMERGENCY_STOP_CM}cm) — hard stop!")
            self._state = State.OBSTACLE_FALLBACK
            self._both_ir_at_fallback = (left_ir and right_ir)
            # Fall through to fallback handler this tick

        # ── STATE: FORWARD_CRUISE ──
        if self._state == State.FORWARD_CRUISE:
            self._turn_direction = ""

            # ── SLALOM TRAP TRIGGER: both IRs OR sonar < TRAP_ESCAPE_CM ──
            # From slalom.py Priority 1: immediate fallback for trapped situations
            if (left_ir and right_ir) or (0 < front_dist < self.TRAP_ESCAPE_CM):
                reason = "both IRs blocked" if (left_ir and right_ir) else f"sonar {front_dist:.0f}cm < {self.TRAP_ESCAPE_CM}cm"
                print(f"\n🚨 [CRUISE→FALLBACK] Trap detected ({reason}) — escaping!")
                self._both_ir_at_fallback = (left_ir and right_ir)
                self._state = State.OBSTACLE_FALLBACK
                # Fall through to fallback handler below

            # ── Graduated obstacle response ──
            elif 0 < front_dist < self.STEER_FALLBACK_CM:
                # Too close — go straight to fallback (stop-reverse-spin)
                print(f"\n🚨 [CRUISE→FALLBACK] Obstacle dangerously close "
                      f"at {front_dist:.0f}cm (< {self.STEER_FALLBACK_CM}cm)")
                self._both_ir_at_fallback = (left_ir and right_ir)
                self._state = State.OBSTACLE_FALLBACK
                # Fall through to fallback handler below

            elif 0 < front_dist < self.STEER_DETECT_CM:
                # Obstacle in steering range — begin slalom proportional dodge
                self._steer_direction = self._choose_steer_direction()
                self._steer_entry_time = now
                self._returning_to_heading = False
                print(f"\n🔀 [CRUISE→STEER] Obstacle at {front_dist:.0f}cm "
                      f"— slalom dodge {self._steer_direction} "
                      f"(threshold: {self.STEER_DETECT_CM}cm)")
                self._state = State.OBSTACLE_STEERING
                # Fall through to steering handler below

            # ── Single-IR hard swerve while cruising ──
            # From slalom.py Priority 2: side obstacle detected by IR only
            elif left_ir and not right_ir:
                self._turn_direction = "right"
                self._motor.forward_differential(
                    self.IR_HARD_SWERVE_FAST, self.IR_HARD_SWERVE_SLOW)
                self._last_steer_dir = "right"
                self._maybe_print_status(
                    front_dist if front_dist >= 0 else 999,
                    gyro_z, 0, "IR SWERVE RIGHT (left blocked)")
                return

            elif right_ir and not left_ir:
                self._turn_direction = "left"
                self._motor.forward_differential(
                    self.IR_HARD_SWERVE_SLOW, self.IR_HARD_SWERVE_FAST)
                self._last_steer_dir = "left"
                self._maybe_print_status(
                    front_dist if front_dist >= 0 else 999,
                    gyro_z, 0, "IR SWERVE LEFT (right blocked)")
                return

            else:
                # ── Normal PID cruise (with heading return if needed) ──
                correction = self._pid.update(gyro_z, dt)

                # Heading return: blend in a correction to steer back on course
                if self._returning_to_heading:
                    heading_correction = -self.HEADING_KP * self._heading_offset
                    heading_correction = max(-self.PID_LIMIT, min(self.PID_LIMIT, heading_correction))
                    correction += heading_correction
                    correction = max(-self.PID_LIMIT * 2, min(self.PID_LIMIT * 2, correction))

                    if abs(self._heading_offset) < self.HEADING_TOLERANCE:
                        self._heading_offset = 0.0
                        self._returning_to_heading = False
                        print("    🧭 Heading restored — resuming pure PID cruise")

                # Store for telemetry
                self._last_pid_correction = correction

                # Drive forward with differential correction
                self._motor.forward(self.BASE_SPEED, correction)

                # Periodic status
                extra = ""
                if self._returning_to_heading:
                    extra = f"Returning to heading ({self._heading_offset:+.1f}°)"
                self._maybe_print_status(
                    front_dist if front_dist >= 0 else 999,
                    gyro_z, correction, extra)
                return

        # ── STATE: OBSTACLE_STEERING (non-blocking, per-tick) ──
        # Slalom proportional differential dodge with IR hard-swerve override
        if self._state == State.OBSTACLE_STEERING:
            self._turn_direction = self._steer_direction

            display_dist = front_dist if front_dist >= 0 else 999

            # ── Exit conditions ──
            elapsed = now - self._steer_entry_time

            # Both-IR trap → immediate fallback
            if left_ir and right_ir:
                print(f"    🚨 [STEER→FALLBACK] Both IRs blocked — trap escape!")
                self._both_ir_at_fallback = True
                self._state = State.OBSTACLE_FALLBACK
                # Fall through to fallback handler below

            elif front_dist > self.STEER_CLEAR_CM or front_dist < 0:
                # Path is clear — resume cruise with heading return
                self._pid.reset()
                self._returning_to_heading = (abs(self._heading_offset) > self.HEADING_TOLERANCE)
                self._turn_direction = ""
                self._steer_direction = ""
                self._state = State.FORWARD_CRUISE
                print(f"    ✅ Path clear at {display_dist:.0f}cm "
                      f"— resuming cruise (heading offset: {self._heading_offset:+.1f}°)\n")
                return

            elif 0 < front_dist < self.STEER_FALLBACK_CM:
                # Too close despite steering — fall back to stop-reverse
                print(f"    🚨 [STEER→FALLBACK] Still too close "
                      f"at {front_dist:.0f}cm — stopping")
                self._both_ir_at_fallback = (left_ir and right_ir)
                self._state = State.OBSTACLE_FALLBACK
                # Fall through to fallback handler below

            elif elapsed > self.STEER_TIMEOUT:
                # Timed out — steering alone couldn't clear, fall back
                print(f"    ⏱️  [STEER→FALLBACK] Timeout after "
                      f"{elapsed:.1f}s — obstacle still at {display_dist:.0f}cm")
                self._both_ir_at_fallback = (left_ir and right_ir)
                self._state = State.OBSTACLE_FALLBACK
                # Fall through to fallback handler

            else:
                # ── IR hard swerve override (slalom.py Priority 2) ──
                # Single IR triggered during steering → immediate hard swerve
                if left_ir and not right_ir:
                    self._steer_direction = "right"
                    self._turn_direction = "right"
                    self._last_steer_dir = "right"
                    self._motor.forward_differential(
                        self.IR_HARD_SWERVE_FAST, self.IR_HARD_SWERVE_SLOW)
                    self._last_pid_correction = (self.IR_HARD_SWERVE_FAST - self.IR_HARD_SWERVE_SLOW) / 2.0
                    self._maybe_print_status(
                        display_dist, gyro_z, self._last_pid_correction,
                        f"IR HARD SWERVE RIGHT | {elapsed:.1f}s")
                    return

                elif right_ir and not left_ir:
                    self._steer_direction = "left"
                    self._turn_direction = "left"
                    self._last_steer_dir = "left"
                    self._motor.forward_differential(
                        self.IR_HARD_SWERVE_SLOW, self.IR_HARD_SWERVE_FAST)
                    self._last_pid_correction = -(self.IR_HARD_SWERVE_FAST - self.IR_HARD_SWERVE_SLOW) / 2.0
                    self._maybe_print_status(
                        display_dist, gyro_z, self._last_pid_correction,
                        f"IR HARD SWERVE LEFT | {elapsed:.1f}s")
                    return

                else:
                    # ── Slalom proportional differential dodge ──
                    # From slalom.py Priority 3: avoid_strength = (detect - dist) * TURN_AGGRESSION
                    # Proportional speed: slow down as we get closer
                    speed = self._interpolate(
                        front_dist,
                        self.STEER_DETECT_CM, self.STEER_FALLBACK_CM,
                        self.BASE_SPEED, self.STEER_SPEED_MIN
                    )

                    # Slalom proportional avoid strength
                    avoid_strength = (self.STEER_DETECT_CM - front_dist) * self.TURN_AGGRESSION

                    # Apply as differential left/right speed split
                    if self._steer_direction == "right":
                        left_speed = speed + avoid_strength
                        right_speed = speed - avoid_strength
                    else:  # "left"
                        left_speed = speed - avoid_strength
                        right_speed = speed + avoid_strength

                    # Re-evaluate direction mid-steer if IR changes
                    new_dir = self._choose_steer_direction()
                    if new_dir != self._steer_direction:
                        self._steer_direction = new_dir
                        self._turn_direction = new_dir
                        print(f"    🔄 Direction change → now dodging {new_dir}")

                    # Drive with differential
                    self._motor.forward_differential(left_speed, right_speed)

                    # Telemetry — report the differential as pseudo-correction
                    self._last_pid_correction = avoid_strength if self._steer_direction == "right" else -avoid_strength

                    # Periodic status
                    self._maybe_print_status(
                        display_dist, gyro_z, self._last_pid_correction,
                        f"Slalom {self._steer_direction} | Spd: {speed:.0f}% | "
                        f"Avoid: {avoid_strength:.0f} | L:{left_speed:.0f}/R:{right_speed:.0f} | "
                        f"{elapsed:.1f}s")
                    return

        # ── STATE: OBSTACLE_FALLBACK (blocking stop-reverse-spin) ──
        # Enhanced with slalom.py trap-escape: random spin when both IRs blocked
        if self._state == State.OBSTACLE_FALLBACK:
            self._turn_direction = self._steer_direction or "right"

            # Phase 1: Stop
            self._motor.stop()
            time.sleep(self.STOP_PAUSE)

            if not self._active:
                return

            # Phase 2: Check rear and reverse if clear
            rear_dist = self._rear_sonar.read()
            if rear_dist < 0 or rear_dist > self.REAR_CLEAR_CM:
                # Rear is clear (or no rear sensor) — reverse
                print(f"    ↩️  Reversing for {self.REVERSE_DURATION:.1f}s "
                      f"(rear: {rear_dist:.0f}cm)")
                self._motor.reverse(self.REVERSE_SPEED, self.REVERSE_DURATION)
            else:
                print(f"    ⛔ Rear blocked ({rear_dist:.0f}cm) — skipping reverse")

            if not self._active:
                return

            # Phase 3: Brief stop before spin
            self._motor.stop()
            time.sleep(0.1)

            if not self._active:
                return

            # Phase 4: Spin to change direction
            # Slalom trap-escape: random spin when both IRs were blocked
            if self._both_ir_at_fallback and self.RANDOM_SPIN_ON_BOTH_IR:
                spin_dir = random.choice(["left", "right"])
                print(f"    🎲 Both IRs blocked — random spin {spin_dir} "
                      f"for {self.SPIN_DURATION:.1f}s (slalom trap-escape)")
            else:
                # Directed spin: use IR-based steer direction
                spin_dir = self._steer_direction or self._choose_steer_direction()
                print(f"    🔄 Spinning {spin_dir} for {self.SPIN_DURATION:.1f}s")

            if spin_dir == "left":
                self._motor.spin_left(self.SPIN_SPEED, self.SPIN_DURATION)
            else:
                self._motor.spin_right(self.SPIN_SPEED, self.SPIN_DURATION)

            if not self._active:
                return

            # Phase 5: Reset PID and resume cruising
            self._pid.reset()
            self._last_pid_time = time.time()
            self._turn_direction = ""
            self._steer_direction = ""
            self._heading_offset = 0.0  # Reset heading after major maneuver
            self._returning_to_heading = False
            self._both_ir_at_fallback = False
            self._state = State.FORWARD_CRUISE
            print("    ✅ Fallback avoidance complete — resuming FORWARD_CRUISE\n")
            return


# ── Standalone test ────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  AUTONOMOUS ROVER — Standalone Sensor Test")
    print("  (Slalom-Integrated Autopilot)")
    print("=" * 60)

    # Test MPU6050
    gyro = MPU6050Sensor()
    if gyro.available:
        gyro.calibrate(2.0)
        print("\nReading Gyro Z for 5 seconds:")
        for _ in range(50):
            z = gyro.read_gyro_z()
            print(f"  Gyro_Z: {z:+.2f} °/s")
            time.sleep(0.1)
    else:
        print("\nMPU6050 not available — skipping gyro test")

    print("\nDone.")
