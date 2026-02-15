"""
autopilot.py — Autonomous Rover with PID Heading Correction.

3-State Finite State Machine
─────────────────────────────
  FORWARD_CRUISE       Drive forward with gyro-based PID heading correction.
  OBSTACLE_AVOIDANCE   Stop → reverse → spin right → resume.
  EMERGENCY_STOP       Cliff detected or manual kill — all motors locked.

Hardware
────────
  Motors (L298N)    IN1=17, IN2=27, ENA=12 (PWM0)
                    IN3=22, IN4=23, ENB=13 (PWM1)  @ 1000 Hz
  MPU6050           I2C Bus 1, Address 0x68 — Gyro Z-axis yaw rate
  Front Sonar       Trig=25, Echo=24
  Rear Sonar        Trig=20, Echo=16
  IR Edge           Left=GPIO5, Right=GPIO6  (Active LOW = cliff)

PID Logic
─────────
  Target yaw rate = 0 °/s (straight line).
  Gyro_Z drifting left  → increase left motor speed.
  Gyro_Z drifting right → increase right motor speed.

Wraps the existing CarSystem (motor.py) and SensorSystem (sensors.py)
through MotorDriver and Sonar helper classes for clean abstraction.
"""

import time
import threading
from collections import deque
from enum import Enum


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    FORWARD_CRUISE      = "FORWARD_CRUISE"
    OBSTACLE_AVOIDANCE  = "OBSTACLE_AVOIDANCE"
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
    timeout as a safety net, plus a 3-sample rolling median filter to
    reject single-reading noise spikes.
    """

    def __init__(self, get_distance_fn, name="sonar"):
        self._read = get_distance_fn
        self.name = name
        self._history = deque(maxlen=3)

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

    def __init__(self, car):
        """
        Parameters
        ----------
        car : motor.CarSystem
            The existing low-level motor controller.
        """
        self._car = car

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
        left_speed = max(0, min(100, base_speed + correction))
        right_speed = max(0, min(100, base_speed - correction))
        self._car._set_raw_motors(left_speed, right_speed, True, True)
        self._car._current_speed = base_speed

    def reverse(self, speed, duration=0.8):
        """Reverse at *speed* for *duration* seconds, then stop."""
        self._car.reverse(speed)
        time.sleep(duration)
        self._car.stop()

    def spin_right(self, speed=50, duration=0.4):
        """Tank-turn right (left fwd / right back) for *duration* s."""
        self._car.pivot_turn("right", speed)
        time.sleep(duration)
        self._car.stop()

    def stop(self):
        """Gentle coast stop (all pins LOW, PWM 0)."""
        self._car.stop()

    def brake(self):
        """Magnetic lock brake (H-bridge short-circuit)."""
        self._car.brake()


# ── Autonomous Rover (AutoPilot) ──────────────────────

class AutoPilot:
    """
    3-State Autonomous Rover with PID heading correction.

    States
    ------
    FORWARD_CRUISE      Drive forward; PID keeps heading straight.
    OBSTACLE_AVOIDANCE  Stop → check rear → reverse → spin right → resume.
    EMERGENCY_STOP      Cliff/edge detected or manual kill.

    Parameters
    ----------
    car : motor.CarSystem
        Low-level motor controller.
    get_sonar : callable → float
        Returns front sonar distance in cm.
    get_ir : callable → (bool, bool)
        Returns (left_cliff, right_cliff).  True = cliff/edge detected.
    get_rear_sonar : callable → float | None
        Returns rear sonar distance in cm (optional).
    get_camera_distance : callable → float | None
        Accepted for API compatibility; not used by the rover FSM.
    """

    # ── Tuning constants (exposed to UI via TUNING_KEYS) ──

    BASE_SPEED       = 50     # Forward cruise PWM %
    FRONT_DANGER_CM  = 25     # Front obstacle threshold (cm)
    REAR_CLEAR_CM    = 20     # Rear must be > this to reverse (cm)
    REVERSE_SPEED    = 50     # Reverse PWM %
    REVERSE_DURATION = 0.8    # Reverse time (seconds)
    SPIN_SPEED       = 50     # Spin/pivot PWM %
    SPIN_DURATION    = 0.4    # Spin-right time (seconds)
    STOP_PAUSE       = 0.5    # Pause before avoidance maneuver (seconds)
    PID_KP           = 2.0    # PID proportional gain
    PID_KI           = 0.0    # PID integral gain
    PID_KD           = 0.5    # PID derivative gain
    PID_LIMIT        = 30.0   # Max PID correction (PWM %)
    STATUS_INTERVAL  = 0.5    # Status print interval (seconds)
    CALIBRATION_TIME = 2.0    # Gyro calibration duration (seconds)

    TUNING_KEYS = [
        "BASE_SPEED", "FRONT_DANGER_CM", "REAR_CLEAR_CM",
        "REVERSE_SPEED", "REVERSE_DURATION",
        "SPIN_SPEED", "SPIN_DURATION", "STOP_PAUSE",
        "PID_KP", "PID_KI", "PID_KD", "PID_LIMIT",
        "STATUS_INTERVAL", "CALIBRATION_TIME",
    ]

    def __init__(self, car, get_sonar, get_ir,
                 get_rear_sonar=None, get_camera_distance=None):
        # Wrap low-level motor controller
        self._motor = MotorDriver(car)
        self._car = car  # Keep direct ref for speed telemetry

        # Wrap sonar callables with timeout + median filter
        self._front_sonar = Sonar(get_sonar, "front")
        self._rear_sonar = Sonar(get_rear_sonar, "rear")

        # IR callable: returns (left_cliff, right_cliff)
        self._get_ir = get_ir

        # Camera distance callable (kept for API compat, unused in rover FSM)
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
        self._last_status_time = time.time()
        self._last_pid_time = time.time()
        self._active = True
        print("🚀 [ROVER] Autonomous navigation STARTED")

    def stop(self):
        """Halt all motors and deactivate FSM."""
        self._active = False
        self._motor.brake()
        self._pid.reset()
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        print("🛑 [ROVER] Autonomous navigation STOPPED")

    # ── Sensor helpers ─────────────────────────

    def _read_ir(self):
        """Read IR cliff sensors. Returns (left_cliff, right_cliff)."""
        try:
            return self._get_ir()
        except Exception:
            return (False, False)

    def get_rear_distance(self):
        """Public accessor for rear sonar (used by telemetry if needed)."""
        return self._rear_sonar.read()

    # ── Status printer ─────────────────────────

    def _maybe_print_status(self, front_dist, gyro_z, correction):
        """Print status line at STATUS_INTERVAL frequency."""
        now = time.time()
        if now - self._last_status_time >= self.STATUS_INTERVAL:
            self._last_status_time = now
            state_name = self._state.value
            print(f"📡 [{state_name}] Front: {front_dist:.0f}cm | "
                  f"Gyro_Z: {gyro_z:+.2f}°/s | "
                  f"PID: {correction:+.1f} | "
                  f"Speed: {self.BASE_SPEED}")

    # ── FSM Core ───────────────────────────────

    def update(self):
        """
        Single tick of the 3-state FSM.

        Called by the drive_autonomous thread at ~20 Hz.
        OBSTACLE_AVOIDANCE is a blocking sequence (~1.7 s) which is
        acceptable since it runs in a dedicated daemon thread.
        """
        if not self._active:
            return

        # ── STATE: EMERGENCY_STOP ──
        if self._state == State.EMERGENCY_STOP:
            # Stay locked.  Only start() can exit this state.
            self._motor.brake()
            return

        # ── Check IR cliff sensors (applies in any non-emergency state) ──
        left_cliff, right_cliff = self._read_ir()
        if left_cliff or right_cliff:
            self._state = State.EMERGENCY_STOP
            self._motor.brake()
            side = "LEFT" if left_cliff else "RIGHT"
            if left_cliff and right_cliff:
                side = "BOTH"
            print(f"\n⚠️  EMERGENCY STOP — Cliff detected ({side})!")
            print(f"    IR Left: {'CLIFF' if left_cliff else 'OK'} | "
                  f"IR Right: {'CLIFF' if right_cliff else 'OK'}")
            return

        # ── STATE: FORWARD_CRUISE ──
        if self._state == State.FORWARD_CRUISE:
            self._turn_direction = ""

            # Read front sonar
            front_dist = self._front_sonar.read()

            # Obstacle check
            if 0 < front_dist < self.FRONT_DANGER_CM:
                print(f"\n🚧 [CRUISE→AVOID] Obstacle at {front_dist:.0f}cm "
                      f"(threshold: {self.FRONT_DANGER_CM}cm)")
                self._state = State.OBSTACLE_AVOIDANCE
                # Fall through to avoidance handler below
            else:
                # PID heading correction
                now = time.time()
                dt = now - self._last_pid_time
                self._last_pid_time = now

                gyro_z = self._gyro.read_gyro_z()

                # Error = deviation from target yaw rate of 0
                # Positive gyro_z = drifting right → need positive correction
                # (boost left wheel, slow right wheel)
                correction = self._pid.update(gyro_z, dt)

                # Store for telemetry
                self._last_gyro_z = gyro_z
                self._last_pid_correction = correction

                # Drive forward with differential correction
                self._motor.forward(self.BASE_SPEED, correction)

                # Periodic status
                self._maybe_print_status(
                    front_dist if front_dist >= 0 else 999,
                    gyro_z, correction)
                return

        # ── STATE: OBSTACLE_AVOIDANCE ──
        if self._state == State.OBSTACLE_AVOIDANCE:
            self._turn_direction = "right"

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

            # Phase 4: Spin right to change direction
            print(f"    🔄 Spinning right for {self.SPIN_DURATION:.1f}s")
            self._motor.spin_right(self.SPIN_SPEED, self.SPIN_DURATION)

            if not self._active:
                return

            # Phase 5: Reset PID and resume cruising
            self._pid.reset()
            self._last_pid_time = time.time()
            self._turn_direction = ""
            self._state = State.FORWARD_CRUISE
            print("    ✅ Avoidance complete — resuming FORWARD_CRUISE\n")
            return


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
