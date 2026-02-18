"""
autopilot.py — Slalom-Style Autonomous Rover (Gyro-Based Yaw Tracking).

Ported from slalom.py's 3-priority gyro-based navigation algorithm,
wrapped in the AutoPilot class interface expected by main.py.
Uses ONLY sonar, IR, and MPU6050 — no camera.

Priority System (from slalom.py)
────────────────────────────────
  P1  ESCAPE        IR stuck OR sonar < CRITICAL_DIST
                    → stop → reverse → random spin → reset yaw
  P2  SLALOM DODGE  sonar < WARN_DIST
                    → accumulate target_yaw with alternating slalom_sign
                      using dynamic increments based on proximity
  P3  CRUISE        Clear path → straight, slowly return target_yaw to center

Execution (PID-like)
────────────────────
  error = target_yaw - heading
  correction = error * GYRO_KP
  l_speed = BASE_SPEED - correction
  r_speed = BASE_SPEED + correction
  → forward differential drive with per-wheel trims

Hardware
────────
  Motors (L298N)    Controlled via CarSystem (motor.py)
  MPU6050           I2C Bus 1, Address 0x68 — Gyro Z-axis yaw tracking
  Front Sonar       Via SensorSystem
  Rear Sonar        Via SensorSystem (checked before reversing)
  IR Edge           Left=GPIO5, Right=GPIO6  (Active LOW = obstacle)
"""

import time
import random
import threading
from collections import deque
from enum import Enum


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    CRUISING        = "CRUISING"
    DODGING         = "DODGING"
    ESCAPING        = "ESCAPING"
    EMERGENCY_STOP  = "EMERGENCY_STOP"


# ── MPU6050 Sensor ─────────────────────────────────────

class MPU6050Sensor:
    """
    Reads Gyro Z-axis from the MPU6050 over I2C for yaw tracking.
    Integrates gyro Z rate into a heading angle (degrees).
    """

    _GYRO_SCALE = 131.0  # LSB per °/s (±250 °/s range)

    def __init__(self, address=0x68, bus_number=1):
        self._address = address
        self._bus_number = bus_number
        self._offset_z = 0.0
        self._available = False
        self._use_lib = False
        self._sensor = None
        self._bus = None

        # Yaw integration state
        self._current_yaw = 0.0
        self._last_time = time.time()

        # Try high-level library first
        try:
            from mpu6050 import mpu6050 as MPU6050Lib
            self._sensor = MPU6050Lib(self._address)
            self._sensor.get_gyro_data()
            self._use_lib = True
            self._available = True
            print("🧭 MPU6050: Connected (mpu6050 library)")
            return
        except Exception:
            pass

        # Fallback: raw smbus
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
            self._bus.write_byte_data(self._address, 0x6B, 0x00)
            time.sleep(0.05)
            who = self._bus.read_byte_data(self._address, 0x75)
            if who not in (0x68, 0x72):
                print(f"⚠️  MPU6050: Unexpected WHO_AM_I=0x{who:02X}")
            self._available = True
            print("🧭 MPU6050: Connected (raw smbus)")
        except Exception as e:
            print(f"⚠️  MPU6050: I2C init failed — {e}")

    def _read_raw_gyro_z(self):
        high = self._bus.read_byte_data(self._address, 0x47)
        low = self._bus.read_byte_data(self._address, 0x48)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value / self._GYRO_SCALE

    @property
    def available(self):
        return self._available

    def read_gyro_z(self):
        """Read raw gyro Z rate (deg/s), offset-corrected."""
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

    def get_yaw(self):
        """Integrate gyro Z into cumulative yaw heading (degrees).
        Must be called frequently (~50Hz) for accurate tracking."""
        if not self._available:
            return 0.0
        gyro_z = self.read_gyro_z()
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        self._current_yaw += gyro_z * dt
        return self._current_yaw

    def reset_yaw(self):
        """Reset integrated yaw to zero."""
        self._current_yaw = 0.0
        self._last_time = time.time()

    @property
    def current_yaw(self):
        return self._current_yaw

    def calibrate(self, duration=2.0):
        if not self._available:
            print("🧭 MPU6050: Unavailable — skipping calibration")
            return
        print(f"🧭 [CALIBRATING] Stand still for {duration:.0f}s …")
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
            time.sleep(0.01)
        if samples:
            self._offset_z = sum(samples) / len(samples)
            print(f"🧭 [CALIBRATED] {len(samples)} samples — offset = {self._offset_z:+.3f} °/s")
        else:
            self._offset_z = 0.0
            print("🧭 [CALIBRATED] No samples — offset = 0.0")
        # Reset yaw after calibration
        self.reset_yaw()


# ── Sonar Wrapper ──────────────────────────────────────

class Sonar:
    """Wraps a sonar callable with timeout protection + 2-sample median filter."""

    def __init__(self, get_distance_fn, name="sonar"):
        self._read = get_distance_fn
        self.name = name
        self._history = deque(maxlen=2)

    def read(self):
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
    High-level motor commands wrapping CarSystem.
    Maps slalom.py's set_motors() calls to the CarSystem abstraction
    with per-wheel trim factors.
    """

    def __init__(self, car):
        self._car = car

    def forward_differential(self, left_speed, right_speed,
                             fl_trim=1.0, fr_trim=1.0,
                             rl_trim=1.0, rr_trim=1.0):
        """Drive forward with explicit left/right wheel speeds (0-100 %)
        and per-wheel trim factors applied."""
        left_speed = max(0, min(100, abs(left_speed)))
        right_speed = max(0, min(100, abs(right_speed)))

        # Apply trims: front wheels get their trim, rear wheels get theirs
        fl = left_speed * fl_trim
        fr = right_speed * fr_trim
        rl = left_speed * rl_trim
        rr = right_speed * rr_trim

        # Clamp after trim
        fl = max(0, min(100, fl))
        fr = max(0, min(100, fr))
        rl = max(0, min(100, rl))
        rr = max(0, min(100, rr))

        # Direct per-wheel PWM for trim support
        self._car.pwm_fl.ChangeDutyCycle(int(fl))
        self._car.pwm_fr.ChangeDutyCycle(int(fr))
        self._car.pwm_rl.ChangeDutyCycle(int(rl))
        self._car.pwm_rr.ChangeDutyCycle(int(rr))

        # Set direction pins: all forward
        from motor import GPIO
        GPIO.output([self._car.FL_IN1, self._car.RL_IN1,
                     self._car.FR_IN3, self._car.RR_IN3], True)
        GPIO.output([self._car.FL_IN2, self._car.RL_IN2,
                     self._car.FR_IN4, self._car.RR_IN4], False)

        self._car._current_speed = (left_speed + right_speed) / 2.0
        self._car._last_l_fwd = True
        self._car._last_r_fwd = True

    def reverse(self, speed, duration=0.8, rear_sonar=None,
                rear_clear_cm=20):
        """Reverse at *speed* for *duration* seconds, then stop.

        If *rear_sonar* (a Sonar instance) is provided, the rear distance
        is checked every iteration and the car stops immediately when an
        obstacle is closer than *rear_clear_cm*.
        """
        end = time.time() + duration
        while time.time() < end:
            # Continuous rear-obstacle check while reversing
            if rear_sonar is not None:
                rd = rear_sonar.read()
                if 0 < rd < rear_clear_cm:
                    print(f"    ⛔ Rear obstacle at {rd:.0f}cm — stopping reverse")
                    break
            self._car.reverse(speed)
            time.sleep(0.02)
        self._car.stop()

    def spin_right(self, speed=50, duration=0.4):
        """Tank-turn right for *duration* s."""
        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("right", speed)
            time.sleep(0.02)
        self._car.stop()

    def spin_left(self, speed=50, duration=0.4):
        """Tank-turn left for *duration* s."""
        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("left", speed)
            time.sleep(0.02)
        self._car.stop()

    def stop(self):
        self._car.stop()

    def brake(self):
        self._car.brake()


# ── Autonomous Rover (AutoPilot) ──────────────────────

class AutoPilot:
    """
    Slalom-style autonomous rover — gyro-based yaw-tracking navigation.
    Ported from slalom.py's 3-priority algorithm.

    Priority 1: ESCAPE       (IR stuck OR dist < CRITICAL_DIST)
                              → stop → reverse → random spin → reset yaw
    Priority 2: SLALOM DODGE (dist < WARN_DIST)
                              → accumulate target_yaw with slalom memory
    Priority 3: CRUISE       (clear path → straight, center yaw)

    Uses ONLY sonar, IR, and MPU6050 — no camera.
    """

    # ── Navigation tuning (from slalom.py) ──
    BASE_SPEED       = 50     # Cruising PWM %
    ESCAPE_SPEED     = 65     # Reverse / spin PWM % during escape
    GYRO_KP          = 1.5    # Gyro proportional gain for yaw correction

    # ── Distance thresholds ──
    CRITICAL_DIST    = 20     # P1: IR stuck or sonar < this → escape (cm)
    WARN_DIST        = 100    # P2: sonar < this → slalom dodge (cm)

    # ── Slalom dodge parameters ──
    SLALOM_BASE_DEG  = 15     # Base dodge increment (degrees)
    SLALOM_PROXIMITY = 0.30   # Proximity multiplier for dynamic increment
    SLALOM_COOLDOWN  = 0.5    # Min time between dodge increments (seconds)
    SLALOM_HEADING_THRESH = 10  # Heading error below which new dodge is added (degrees)
    SLALOM_CLEAR_TIME = 1.0   # Time after last dodge before resetting direction (seconds)
    YAW_CENTER_RATE  = 0.2    # Rate at which target_yaw returns to center (deg/tick)
    YAW_CENTER_DEAD  = 2.0    # Dead zone for yaw centering (degrees)

    # ── Per-wheel trim factors ──
    FL_TRIM          = 0.6    # Front-Left motor trim
    FR_TRIM          = 0.6    # Front-Right motor trim
    RL_TRIM          = 1.0    # Rear-Left motor trim
    RR_TRIM          = 1.0    # Rear-Right motor trim

    # ── Escape timing ──
    ESCAPE_STOP_TIME     = 0.1    # Pause before reversing (seconds)
    ESCAPE_REVERSE_TIME  = 0.8    # Reverse duration (seconds)
    ESCAPE_SPIN_TIME     = 0.5    # Spin duration (seconds)
    ESCAPE_RESUME_TIME   = 0.2    # Pause after spin before resuming (seconds)

    # ── Rear sonar ──
    REAR_CLEAR_CM    = 20     # Rear must be > this to reverse (cm)

    # ── Gyro calibration ──
    CALIBRATION_TIME = 2.0    # Gyro calibration duration (seconds)
    STATUS_INTERVAL  = 0.5    # Status print interval (seconds)

    TUNING_KEYS = [
        # Navigation
        "BASE_SPEED", "ESCAPE_SPEED", "GYRO_KP",
        # Distances
        "CRITICAL_DIST", "WARN_DIST",
        # Slalom
        "SLALOM_BASE_DEG", "SLALOM_PROXIMITY", "SLALOM_COOLDOWN",
        "SLALOM_HEADING_THRESH", "SLALOM_CLEAR_TIME",
        "YAW_CENTER_RATE", "YAW_CENTER_DEAD",
        # Motor trims
        "FL_TRIM", "FR_TRIM", "RL_TRIM", "RR_TRIM",
        # Escape timing
        "ESCAPE_STOP_TIME", "ESCAPE_REVERSE_TIME",
        "ESCAPE_SPIN_TIME", "ESCAPE_RESUME_TIME",
        # Rear sonar
        "REAR_CLEAR_CM",
        # Gyro
        "CALIBRATION_TIME", "STATUS_INTERVAL",
    ]

    def __init__(self, car, get_sonar, get_ir, get_rear_sonar=None,
                 sensor_system=None, get_rear_distance=None):
        self._motor = MotorDriver(car)
        self._car = car

        self._front_sonar = Sonar(get_sonar, "front")
        # Accept rear sonar callable from either kwarg name
        rear_fn = get_rear_sonar or get_rear_distance
        self._rear_sonar = Sonar(rear_fn, "rear")
        self._get_ir = get_ir
        self._sensor_system = sensor_system

        # Gyro — central to navigation (yaw tracking)
        self._gyro = MPU6050Sensor()

        # State
        self._state = State.CRUISING
        self._active = False
        self._turn_direction = ""
        self._last_status_time = 0.0

        # Slalom state (from slalom.py)
        self._target_yaw = 0.0
        self._dodge_direction = 0    # 0=none, 1=right, -1=left
        self._last_dodge_time = 0.0

        # Telemetry
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
        return self._gyro.current_yaw

    @property
    def slalom_sign(self):
        return self._dodge_direction

    # ── Tuning API ─────────────────────────────

    @classmethod
    def get_default_tuning(cls):
        return {key: getattr(cls, key) for key in cls.TUNING_KEYS}

    def get_tuning(self):
        return {key: getattr(self, key) for key in self.TUNING_KEYS}

    # ── Lifecycle ──────────────────────────────

    def start(self):
        """Calibrate gyro and begin autonomous driving."""
        self._gyro.calibrate(duration=self.CALIBRATION_TIME)
        self._gyro_calibrated = self._gyro.available
        self._state = State.CRUISING
        self._turn_direction = ""
        self._target_yaw = 0.0
        self._dodge_direction = 0
        self._last_dodge_time = 0.0
        self._last_status_time = time.time()
        self._active = True
        print("🚀 [ROVER] Autonomous navigation STARTED (slalom yaw-tracking mode)")

    def stop(self):
        """Halt all motors and deactivate."""
        self._active = False
        self._motor.brake()
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        print("🛑 [ROVER] Autonomous navigation STOPPED")

    # ── Helpers ────────────────────────────────

    def _read_ir(self):
        try:
            return self._get_ir()
        except Exception:
            return (False, False)

    def get_rear_distance(self):
        return self._rear_sonar.read()

    def _maybe_print_status(self, dist, msg):
        now = time.time()
        if now - self._last_status_time >= self.STATUS_INTERVAL:
            self._last_status_time = now
            state_name = self._state.value
            heading = self._gyro.current_yaw
            target = self._target_yaw
            gyro_str = f" | Yaw: {heading:+.1f}° → {target:+.1f}°" if self._gyro.available else ""
            print(f"📡 [{state_name}] Front: {dist:.0f}cm{gyro_str} | {msg}")

    # ── Escape (slalom.py P1) ──────────────────

    def _escape(self, left_stuck, right_stuck):
        """
        P1 escape from slalom.py:
        stop → reverse → spin (IR-directed or random) → reset yaw.
        """
        self._state = State.ESCAPING
        self._turn_direction = "escaping"

        # 1. STOP
        self._motor.stop()
        time.sleep(self.ESCAPE_STOP_TIME)
        if not self._active:
            return

        # 2. REVERSE (check rear sonar before AND during reverse)
        rear_dist = self._rear_sonar.read()
        if rear_dist < 0 or rear_dist > self.REAR_CLEAR_CM:
            print(f"    ↩️  Reversing {self.ESCAPE_REVERSE_TIME:.1f}s "
                  f"(rear: {rear_dist:.0f}cm)")
            self._motor.reverse(
                self.ESCAPE_SPEED, self.ESCAPE_REVERSE_TIME,
                rear_sonar=self._rear_sonar,
                rear_clear_cm=self.REAR_CLEAR_CM)
        else:
            print(f"    ⛔ Rear blocked ({rear_dist:.0f}cm) — skip reverse")

        if not self._active:
            return

        # 3. SPIN (IR-directed or Random — exactly like slalom.py)
        if left_stuck:
            spin_mode = "right"
        elif right_stuck:
            spin_mode = "left"
        else:
            spin_mode = "right" if random.choice([True, False]) else "left"

        if spin_mode == "right":
            print(f"    🔄 Spin RIGHT {self.ESCAPE_SPIN_TIME:.1f}s")
            self._motor.spin_right(self.ESCAPE_SPEED, self.ESCAPE_SPIN_TIME)
        else:
            print(f"    🔄 Spin LEFT {self.ESCAPE_SPIN_TIME:.1f}s")
            self._motor.spin_left(self.ESCAPE_SPEED, self.ESCAPE_SPIN_TIME)

        if not self._active:
            return

        # 4. Reset headings (exactly like slalom.py)
        self._motor.stop()
        time.sleep(self.ESCAPE_RESUME_TIME)
        self._gyro.reset_yaw()
        self._target_yaw = 0.0
        self._dodge_direction = 0

        self._turn_direction = ""
        self._state = State.CRUISING
        print("    ✅ Escape complete — yaw reset, resuming cruise\n")

    # ── FSM Core ───────────────────────────────

    def update(self):
        """
        Single tick of the slalom autopilot.

        Mirrors slalom.py's auto_pilot_thread() priorities exactly:
          P1: Escape       (IR stuck OR dist < CRITICAL_DIST)
          P2: Slalom dodge (dist < WARN_DIST → accumulate target_yaw)
          P3: Cruise       (clear path → center yaw)

        Then executes PID-like yaw correction for motor output.
        """
        if not self._active:
            return

        if self._state == State.EMERGENCY_STOP:
            self._motor.brake()
            return

        # ── 1. READ SENSORS ──
        dist = self._front_sonar.read()
        left_stuck, right_stuck = self._read_ir()

        # Read gyro Z rate for telemetry
        self._last_gyro_z = self._gyro.read_gyro_z()

        # ─────────────────────────────────────────────
        # 🚨 PRIORITY 1: ESCAPE
        #    IR stuck OR sonar < CRITICAL_DIST
        #    (slalom.py: left_stuck or right_stuck or dist < CRITICAL_DIST)
        # ─────────────────────────────────────────────
        if left_stuck or right_stuck or (0 < dist < self.CRITICAL_DIST):
            reason = ("IR stuck" if (left_stuck or right_stuck)
                      else f"{dist:.0f}cm < {self.CRITICAL_DIST}cm")
            print(f"\n🚨 ESCAPE ({reason})")
            self._escape(left_stuck, right_stuck)
            return

        # ─────────────────────────────────────────────
        # 🧠 PRIORITY 2: SLALOM DODGE
        #    sonar < WARN_DIST
        #    → accumulate target_yaw with slalom memory
        # ─────────────────────────────────────────────
        elif 0 < dist < self.WARN_DIST:
            self._state = State.DODGING

            # Initialize dodge direction if not set
            if self._dodge_direction == 0:
                self._dodge_direction = 1  # Default right

            # Dynamic increment: base + proximity factor
            dynamic_increment = self.SLALOM_BASE_DEG + int(
                (self.WARN_DIST - dist) * self.SLALOM_PROXIMITY)

            heading_error = abs(self._target_yaw - self._gyro.current_yaw)

            if heading_error < self.SLALOM_HEADING_THRESH:
                # Car has achieved current target — add more dodge
                if time.time() - self._last_dodge_time > self.SLALOM_COOLDOWN:
                    self._target_yaw += (dynamic_increment * self._dodge_direction)
                    self._last_dodge_time = time.time()
                    self._turn_direction = "right" if self._dodge_direction > 0 else "left"
                    self._maybe_print_status(
                        dist, f"SLALOM +{dynamic_increment}° "
                              f"(target: {self._target_yaw:.0f}°)")
            else:
                self._turn_direction = "right" if self._dodge_direction > 0 else "left"
                self._maybe_print_status(
                    dist, f"TURNING {dynamic_increment}° "
                          f"(err: {heading_error:.0f}°)")

        # ─────────────────────────────────────────────
        # 🚀 PRIORITY 3: CRUISE (Clear Path)
        # ─────────────────────────────────────────────
        else:
            self._state = State.CRUISING
            self._turn_direction = ""

            # Reset dodge direction after clear time
            if time.time() - self._last_dodge_time > self.SLALOM_CLEAR_TIME:
                self._dodge_direction = 0

            # Slowly return target_yaw to center
            if self._target_yaw > self.YAW_CENTER_DEAD:
                self._target_yaw -= self.YAW_CENTER_RATE
            elif self._target_yaw < -self.YAW_CENTER_DEAD:
                self._target_yaw += self.YAW_CENTER_RATE

            self._maybe_print_status(
                dist if dist >= 0 else 300, "CRUISING")

        # ─────────────────────────────────────────────
        # 🏎️ EXECUTION (PID-like yaw correction)
        #    Exactly like slalom.py's execution block
        # ─────────────────────────────────────────────
        heading = self._gyro.get_yaw()
        error = self._target_yaw - heading
        correction = error * self.GYRO_KP

        l_speed = self.BASE_SPEED - correction
        r_speed = self.BASE_SPEED + correction

        self._last_pid_correction = correction

        self._motor.forward_differential(
            l_speed, r_speed,
            self.FL_TRIM, self.FR_TRIM,
            self.RL_TRIM, self.RR_TRIM)


# ── Standalone test ────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  AUTONOMOUS ROVER — Slalom Yaw-Tracking Autopilot")
    print("=" * 60)

    gyro = MPU6050Sensor()
    if gyro.available:
        gyro.calibrate(2.0)
        print("\nReading Gyro Z for 5 seconds:")
        for _ in range(50):
            z = gyro.read_gyro_z()
            yaw = gyro.get_yaw()
            print(f"  Gyro_Z: {z:+.2f} °/s | Yaw: {yaw:+.2f}°")
            time.sleep(0.1)
    else:
        print("\nMPU6050 not available — skipping gyro test")

    print("\nDone.")
