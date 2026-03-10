"""
autopilot_pid.py — Laser-Scanner Autonomous Rover

4-State Finite State Machine
─────────────────────────────
  CRUISING         Servo centred, drive forward with PID heading correction.
  SCANNING         Obstacle < OBSTACLE_CM ahead → stop, sweep -30° to +30°.
  TURNING          Analyse sweep → pivot toward clearest direction.
  EMERGENCY_STOP   Manual kill — all motors locked.

Sensor Stack (via Pico UART Bridge)
────────────
  VL53L0X + Pan/Tilt  Pan/tilt gimbal on Pico GP2/GP3, VL53L0X on Pico I2C → UART bridge
  MPU6050          Pico I2C 0x68 → UART bridge — gyro Z-axis yaw rate for PID
  LM393 Encoder    Pico GPIO — wheel RPM
  ADS1115 ADC      Pico I2C 0x48 — battery voltage & current sensing

Scan-then-Drive Logic
─────────────────────
  Default:  Servo at 0° (centre).  Drive Forward.
  Trigger:  Centre distance < OBSTACLE_CM (50 cm).
  Reaction: Stop → sweep (-30 to +30) → pick best direction → pivot turn → resume.
  Sweep:    7 readings × 0.05 s ≈ 0.35 s.
  Fallback: Both sectors blocked → reverse + spin.

Wraps CarSystem (motor.py) and SensorSystem (sensors.py).
"""

import time
import threading
from collections import deque
from enum import Enum


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    CRUISING        = "CRUISING"
    SCANNING        = "SCANNING"
    TURNING         = "TURNING"
    EMERGENCY_STOP  = "EMERGENCY_STOP"


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
    Reads Gyro Z-axis yaw rate from the Pico sensor bridge (UART).

    The MPU6050 is now physically connected to the Pico which streams
    data at 50 Hz over UART. This wrapper reads from pico_sensor_reader
    and provides the same API as the original I2C-direct implementation.
    """

    def __init__(self, address=0x68, bus_number=1):
        self._offset_z = 0.0
        self._available = False

        # Try to connect via Pico sensor bridge
        try:
            from pico_sensor_reader import get_gyro_z, get_sensor_packet, _global_reader
            self._pico_get_gyro_z = get_gyro_z
            self._pico_get_packet = get_sensor_packet

            # Check if Pico reader is initialized and has data
            if _global_reader is not None and _global_reader.is_connected():
                self._available = True
                print("🧭 MPU6050: Connected (via Pico bridge)")
            else:
                # Reader exists but no data yet — wait briefly
                import time
                time.sleep(0.5)
                if _global_reader is not None and _global_reader.is_connected():
                    self._available = True
                    print("🧭 MPU6050: Connected (via Pico bridge)")
                else:
                    # Mark available anyway — data may arrive later
                    self._available = True
                    print("🧭 MPU6050: Pico bridge initialized (awaiting first packet)")
        except ImportError:
            print("⚠️  MPU6050: pico_sensor_reader not available")
        except Exception as e:
            print(f"⚠️  MPU6050: Pico bridge init failed — {e}")

    # ── Public API ─────────────────────────────

    @property
    def available(self):
        return self._available

    def read_gyro_z(self):
        """
        Return drift-corrected Z-axis yaw rate in °/s from Pico bridge.

        Positive = rotating clockwise (drifting right).
        Negative = rotating counter-clockwise (drifting left).
        Returns 0.0 if sensor is unavailable or on error.
        """
        if not self._available:
            return 0.0
        try:
            raw = self._pico_get_gyro_z()
            return raw - self._offset_z
        except Exception:
            return 0.0

    def calibrate(self, duration=2.0):
        """
        Measure gyro Z drift while the robot is stationary.

        Reads from Pico bridge at ~50 Hz for *duration* seconds,
        computes the mean bias, and stores it as the zero offset.
        The robot MUST be still.
        """
        if not self._available:
            print("🧭 MPU6050: Unavailable — skipping calibration (heading correction disabled)")
            return

        import time
        print(f"🧭 [CALIBRATING] Stand still for {duration:.0f}s — measuring gyro drift...")
        samples = []
        start = time.time()
        while time.time() - start < duration:
            try:
                gz = self._pico_get_gyro_z()
                if gz != 0.0 or len(samples) > 0:  # skip initial zeros before first packet
                    samples.append(gz)
            except Exception:
                pass
            time.sleep(0.02)  # ~50 Hz (match Pico output rate)

        if samples:
            self._offset_z = sum(samples) / len(samples)
            print(f"🧭 [CALIBRATED] {len(samples)} samples — drift offset = {self._offset_z:+.3f} °/s")
        else:
            self._offset_z = 0.0
            print("🧭 [CALIBRATED] No samples collected — offset = 0.0")


# ── Distance Sensor Wrapper ────────────────────────────

class DistanceSensor:
    """
    Wraps a distance-read callable with timeout protection and median filtering.

    Works with any callable that returns distance in cm (VL53L0X laser, etc.).
    Adds a 0.15 s thread-level timeout as a safety net, plus a 2-sample rolling
    median filter to reject noise spikes.
    """

    def __init__(self, get_distance_fn, name="laser"):
        self._read = get_distance_fn
        self.name = name
        self._history = deque(maxlen=2)

    def read(self):
        """Return filtered distance in cm.  -1 → error, >0 → valid."""
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

        if raw >= 0:
            self._history.append(raw)

        if len(self._history) == 0:
            return raw

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
    Laser-Scanner Autonomous Rover — 4-State FSM.

    States
    ------
    CRUISING       Servo centred, drive forward with PID heading correction.
    SCANNING       Obstacle < OBSTACLE_CM → stop, sweep ±30° to map surroundings.
    TURNING        Analyse sweep data → pivot toward clearest sector, then resume.
    EMERGENCY_STOP Manual kill — brake.  Only start() exits.

    Parameters
    ----------
    car : motor.CarSystem
        Low-level motor controller.
    get_forward_distance : callable → float
        Returns forward laser distance in cm (servo already centred).
    sensor_system : sensors.SensorSystem
        Used for scan_sweep() during SCANNING state.
    get_rear_distance : callable → float | None
        Rear distance in cm (optional; -1 when absent).
    """

    # ── Tuning constants ──────────────────────────────

    # Cruise
    BASE_SPEED       = 50     # Forward PWM %
    PID_KP           = 5.0
    PID_KI           = 0.0
    PID_KD           = 0.5
    PID_LIMIT        = 30.0
    STATUS_INTERVAL  = 0.5
    CALIBRATION_TIME = 2.0
    MIN_CORRECTION_PWM   = 15.0
    CORRECTION_NOISE_GATE = 1.0

    # Laser scanner
    OBSTACLE_CM      = 50     # Trigger scan when forward < this
    ALL_BLOCKED_CM   = 30     # Both sectors blocked → fallback reverse
    SCAN_STEP_DEG    = 10
    SCAN_RANGE_DEG   = 30     # Sweep ±30°
    SCAN_SETTLE_MS   = 50     # Already baked into SensorSystem

    # Turning
    TURN_SPEED       = 50     # Pivot PWM %
    TURN_MIN_ANGLE   = 25     # Minimum pivot angle (degrees)
    TURN_MAX_ANGLE   = 70     # Maximum pivot angle (degrees)

    # Fallback (reverse + spin)
    REAR_CLEAR_CM    = 20
    REVERSE_SPEED    = 70
    REVERSE_DURATION = 1.2
    SPIN_SPEED       = 60
    SPIN_DURATION    = 0.6
    STOP_PAUSE       = 0.3

    # Emergency
    EMERGENCY_STOP_CM = 10

    TUNING_KEYS = [
        "BASE_SPEED",
        "PID_KP", "PID_KI", "PID_KD", "PID_LIMIT",
        "STATUS_INTERVAL", "CALIBRATION_TIME",
        "MIN_CORRECTION_PWM", "CORRECTION_NOISE_GATE",
        # Laser scanner
        "OBSTACLE_CM", "ALL_BLOCKED_CM",
        "SCAN_STEP_DEG", "SCAN_RANGE_DEG",
        # Turning
        "TURN_SPEED", "TURN_MIN_ANGLE", "TURN_MAX_ANGLE",
        # Fallback
        "REAR_CLEAR_CM", "REVERSE_SPEED", "REVERSE_DURATION",
        "SPIN_SPEED", "SPIN_DURATION", "STOP_PAUSE",
        # Emergency
        "EMERGENCY_STOP_CM",
    ]

    def __init__(self, car, get_forward_distance,
                 sensor_system=None, get_rear_distance=None):
        # Motor wrapper
        self._motor = MotorDriver(car, min_correction=self.MIN_CORRECTION_PWM,
                                  noise_gate=self.CORRECTION_NOISE_GATE)
        self._car = car

        # Distance sensor (threaded + median)
        self._front_sensor = DistanceSensor(get_forward_distance, "front_laser")

        # Scan capability (needs the full SensorSystem for sweep)
        self._sensor_system = sensor_system

        # Rear distance (optional)
        self._get_rear_distance = get_rear_distance

        # Gyroscope
        self._gyro = MPU6050Sensor()

        # PID
        self._pid = PIDController(
            kp=self.PID_KP, ki=self.PID_KI,
            kd=self.PID_KD, output_limit=self.PID_LIMIT,
        )

        # FSM state
        self._state = State.CRUISING
        self._active = False
        self._turn_direction = ""
        self._last_status_time = 0.0
        self._last_pid_time = 0.0

        # Heading tracking (integrated from gyro)
        self._heading = 0.0            # integrated yaw (degrees)
        self._target_yaw = 0.0         # desired heading after turn
        self._dodge_direction = 0      # -1 left, 0 none, +1 right

        # Turn state
        self._turn_target_angle = 0.0
        self._turn_start_heading = 0.0

        # Telemetry
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        self._gyro_calibrated = False
        self._last_scan_data = []      # most recent sweep results
        self._scan_data_fresh = False  # True when new scan available

    # ── Properties ─────────────────────────────────

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
        return self._last_gyro_z

    @property
    def pid_correction(self):
        return self._last_pid_correction

    @property
    def gyro_available(self):
        return self._gyro.available

    @property
    def gyro_calibrated(self):
        return self._gyro_calibrated

    @property
    def target_yaw(self):
        return self._target_yaw

    @property
    def current_heading(self):
        return self._heading

    @property
    def slalom_sign(self):
        return self._dodge_direction

    @property
    def last_scan_data(self):
        """Latest sweep results: [(angle_deg, dist_cm), …]."""
        return self._last_scan_data

    @property
    def scan_data_fresh(self):
        """True once after a new sweep; auto-clears on read."""
        if self._scan_data_fresh:
            self._scan_data_fresh = False
            return True
        return False

    # ── Tuning API ─────────────────────────────────

    @classmethod
    def get_default_tuning(cls):
        return {key: getattr(cls, key) for key in cls.TUNING_KEYS}

    def get_tuning(self):
        return {key: getattr(self, key) for key in self.TUNING_KEYS}

    # ── Lifecycle ──────────────────────────────────

    def start(self):
        """Calibrate gyro, centre servo, reset PID, begin CRUISING."""
        self._gyro.calibrate(duration=self.CALIBRATION_TIME)
        self._gyro_calibrated = self._gyro.available
        self._pid = PIDController(
            kp=self.PID_KP, ki=self.PID_KI,
            kd=self.PID_KD, output_limit=self.PID_LIMIT,
        )
        self._state = State.CRUISING
        self._turn_direction = ""
        self._dodge_direction = 0
        self._heading = 0.0
        self._target_yaw = 0.0
        self._last_status_time = time.time()
        self._last_pid_time = time.time()
        self._last_scan_data = []
        self._scan_data_fresh = False

        # Centre the laser scanner servo
        if self._sensor_system:
            self._sensor_system.center_servo()

        self._active = True
        print("🚀 [ROVER] Laser-scanner autonomous navigation STARTED")

    def stop(self):
        """Halt motors, centre servo, deactivate FSM."""
        self._active = False
        self._motor.brake()
        self._pid.reset()
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        self._heading = 0.0
        self._target_yaw = 0.0
        self._dodge_direction = 0
        self._turn_direction = ""
        if self._sensor_system:
            self._sensor_system.center_servo()
        print("🛑 [ROVER] Laser-scanner autonomous navigation STOPPED")

    # ── Sensor helpers ─────────────────────────────

    def _read_rear(self):
        if self._get_rear_distance is None:
            return -1
        try:
            return self._get_rear_distance()
        except Exception:
            return -1

    def get_rear_distance(self):
        """Public accessor (for telemetry / compat)."""
        return self._read_rear()

    # ── Scan analysis ──────────────────────────────

    @staticmethod
    def _analyse_scan(scan_data, blocked_cm=30):
        """Analyse sweep data and choose the best turn direction.

        Returns
        -------
        dict with keys:
            direction : str   "left" | "right" | "blocked"
            angle     : float  Suggested pivot angle (positive degrees)
            left_avg  : float  Average distance in left sector (-60…-10)
            right_avg : float  Average distance in right sector (+10…+60)
            best_angle: int    Angle (user-space) of the farthest reading
            best_dist : float  Distance at best_angle
        """
        left_dists = []
        right_dists = []
        best_angle = 0
        best_dist = 0.0

        for angle, dist in scan_data:
            if dist < 0:
                dist = 0  # treat errors as blocked
            if dist > best_dist:
                best_dist = dist
                best_angle = angle
            if angle <= -10:
                left_dists.append(dist)
            elif angle >= 10:
                right_dists.append(dist)

        left_avg = sum(left_dists) / len(left_dists) if left_dists else 0
        right_avg = sum(right_dists) / len(right_dists) if right_dists else 0

        # Both sectors blocked?
        if left_avg < blocked_cm and right_avg < blocked_cm:
            direction = "blocked"
            angle = 0.0
        elif left_avg >= right_avg:
            direction = "left"
            # Pivot angle proportional to how far left the best reading is
            angle = max(25, min(70, abs(best_angle))) if best_angle < 0 else 45.0
        else:
            direction = "right"
            angle = max(25, min(70, abs(best_angle))) if best_angle > 0 else 45.0

        return {
            "direction": direction,
            "angle": angle,
            "left_avg": round(left_avg, 1),
            "right_avg": round(right_avg, 1),
            "best_angle": best_angle,
            "best_dist": round(best_dist, 1),
        }

    # ── Status printer ─────────────────────────────

    def _maybe_print_status(self, front_dist, gyro_z, correction, extra=""):
        now = time.time()
        if now - self._last_status_time >= self.STATUS_INTERVAL:
            self._last_status_time = now
            state_name = self._state.value
            extra_str = f" | {extra}" if extra else ""
            print(f"📡 [{state_name}] Front: {front_dist:.0f}cm | "
                  f"Gyro_Z: {gyro_z:+.2f}°/s | "
                  f"PID: {correction:+.1f}{extra_str}")

    # ── FSM Core ───────────────────────────────────

    def update(self):
        """Single tick of the 4-state FSM (called at ~20 Hz)."""
        if not self._active:
            return

        # ── EMERGENCY_STOP — stays locked ──
        if self._state == State.EMERGENCY_STOP:
            self._motor.brake()
            return

        # ── Gyro read (shared by all active states) ──
        now = time.time()
        dt = now - self._last_pid_time
        self._last_pid_time = now
        gyro_z = self._gyro.read_gyro_z()
        self._last_gyro_z = gyro_z

        # Integrate heading
        if 0 < dt < 0.5:
            self._heading += gyro_z * dt

        # ── STATE: CRUISING ──────────────────────────
        if self._state == State.CRUISING:
            self._turn_direction = ""
            self._dodge_direction = 0

            # Read forward distance (servo should already be centred)
            front_dist = self._front_sensor.read()
            display_dist = front_dist if front_dist >= 0 else 999

            # Emergency hard stop
            if 0 < front_dist < self.EMERGENCY_STOP_CM:
                self._motor.brake()
                print(f"\n🚨 [EMERGENCY BRAKE] Obstacle at {front_dist:.0f}cm!")
                self._state = State.SCANNING
                return

            # Obstacle threshold → scan
            if 0 < front_dist < self.OBSTACLE_CM:
                self._motor.stop()
                print(f"\n🔍 [CRUISE→SCAN] Obstacle at {front_dist:.0f}cm "
                      f"(< {self.OBSTACLE_CM}cm) — stopping to scan")
                self._state = State.SCANNING
                return

            # Normal PID cruise
            correction = self._pid.update(gyro_z, dt)
            self._last_pid_correction = correction
            self._motor.forward(self.BASE_SPEED, correction)
            self._maybe_print_status(display_dist, gyro_z, correction)
            return

        # ── STATE: SCANNING (blocking sweep, ~0.6 s) ─
        if self._state == State.SCANNING:
            self._turn_direction = ""
            self._motor.stop()

            if self._sensor_system is None:
                # No scanner available — go to fallback reverse+spin
                print("⚠️  No sensor_system for scanning — fallback")
                self._do_fallback()
                return

            # Perform full sweep
            print("🔄 Sweeping -30° to +30° …")
            scan_data = self._sensor_system.scan_sweep(
                start_deg=-self.SCAN_RANGE_DEG,
                end_deg=self.SCAN_RANGE_DEG,
                step_deg=self.SCAN_STEP_DEG,
            )
            self._last_scan_data = scan_data
            self._scan_data_fresh = True

            # Log sweep results
            readings_str = "  ".join(
                f"{a:+3d}°:{d:.0f}cm" for a, d in scan_data
            )
            print(f"📊 Scan: {readings_str}")

            # Analyse
            analysis = self._analyse_scan(scan_data, self.ALL_BLOCKED_CM)
            print(f"📊 Left avg: {analysis['left_avg']}cm | "
                  f"Right avg: {analysis['right_avg']}cm | "
                  f"Best: {analysis['best_angle']:+d}° @ {analysis['best_dist']}cm | "
                  f"Decision: {analysis['direction']}")

            if analysis["direction"] == "blocked":
                print("⛔ Both sectors blocked — fallback reverse+spin")
                self._do_fallback()
                return

            # Set up turn
            self._turn_direction = analysis["direction"]
            self._dodge_direction = -1 if analysis["direction"] == "left" else 1
            self._turn_target_angle = analysis["angle"]
            self._turn_start_heading = self._heading
            self._state = State.TURNING
            print(f"🔀 [SCAN→TURN] Pivoting {analysis['direction']} "
                  f"~{analysis['angle']:.0f}°")
            return

        # ── STATE: TURNING (pivot until heading delta met) ──
        if self._state == State.TURNING:
            target_delta = self._turn_target_angle
            actual_delta = abs(self._heading - self._turn_start_heading)

            if actual_delta >= target_delta:
                # Turn complete
                self._motor.stop()
                time.sleep(0.1)
                self._pid.reset()
                self._last_pid_time = time.time()
                self._target_yaw = self._heading  # new straight heading
                self._turn_direction = ""
                self._dodge_direction = 0
                self._state = State.CRUISING
                print(f"    ✅ Turn complete ({actual_delta:.1f}° of "
                      f"{target_delta:.0f}°) — resuming CRUISING\n")
                return

            # Continue pivoting
            if self._turn_direction == "left":
                self._car.pivot_turn("left", self.TURN_SPEED)
            else:
                self._car.pivot_turn("right", self.TURN_SPEED)

            self._maybe_print_status(
                999, gyro_z, 0,
                f"Pivot {self._turn_direction} | "
                f"{actual_delta:.1f}°/{target_delta:.0f}°")
            return

    # ── Fallback manoeuvre (reverse + spin) ────────

    def _do_fallback(self):
        """Blocking: stop → reverse → random spin → resume cruise."""
        self._motor.stop()
        time.sleep(self.STOP_PAUSE)
        if not self._active:
            return

        # Reverse if rear clear
        rear_dist = self._read_rear()
        if rear_dist < 0 or rear_dist > self.REAR_CLEAR_CM:
            print(f"    ↩️  Reversing {self.REVERSE_DURATION:.1f}s "
                  f"(rear: {rear_dist:.0f}cm)")
            self._motor.reverse(self.REVERSE_SPEED, self.REVERSE_DURATION)
        else:
            print(f"    ⛔ Rear blocked ({rear_dist:.0f}cm) — skip reverse")

        if not self._active:
            return
        self._motor.stop()
        time.sleep(0.1)
        if not self._active:
            return

        # Spin (alternate direction)
        import random
        spin_dir = random.choice(["left", "right"])
        print(f"    🔄 Spinning {spin_dir} {self.SPIN_DURATION:.1f}s")
        if spin_dir == "left":
            self._car.pivot_turn("left", self.SPIN_SPEED)
        else:
            self._car.pivot_turn("right", self.SPIN_SPEED)
        time.sleep(self.SPIN_DURATION)
        self._car.stop()

        if not self._active:
            return

        # Reset and resume
        self._pid.reset()
        self._last_pid_time = time.time()
        self._heading = 0.0
        self._target_yaw = 0.0
        self._turn_direction = ""
        self._dodge_direction = 0
        self._state = State.CRUISING
        print("    ✅ Fallback complete — resuming CRUISING\n")


# ── Standalone test ────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  AUTONOMOUS ROVER — Standalone Sensor Test")
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
