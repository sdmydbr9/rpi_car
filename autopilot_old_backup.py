"""
autopilot.py — Slalom-Style Autonomous Rover.

Exact replica of slalom.py's 4-priority obstacle avoidance logic,
wrapped in the AutoPilot class interface expected by main.py.
All distances and speeds are tunable via the web UI tuning panel.

Priority System (from slalom.py)
────────────────────────────────
  P1  TRAP ESCAPE      Both IRs triggered OR sonar < TRAP_ESCAPE_CM
                        → stop → reverse → random spin → resume
  P2  SIDE OBSTACLE    Single IR triggered
                        → hard swerve (IR_SWERVE_FAST / IR_SWERVE_SLOW)
  P3  PROPORTIONAL     Sonar < DODGE_DETECT_CM
      DODGE            → avoid_strength = (DODGE_DETECT_CM - dist) * TURN_AGGRESSION
                          applied as left/right speed differential
  P4  CRUISE           Clear path → straight at BASE_SPEED

Hardware
────────
  Motors (L298N)    Controlled via CarSystem (motor.py)
  MPU6050           I2C Bus 1, Address 0x68 — Gyro Z-axis (telemetry only)
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
    FORWARD_CRUISE      = "FORWARD_CRUISE"
    OBSTACLE_STEERING   = "OBSTACLE_STEERING"
    OBSTACLE_FALLBACK   = "OBSTACLE_FALLBACK"
    EMERGENCY_STOP      = "EMERGENCY_STOP"


# ── PID Controller (kept for interface compatibility) ──

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
        if dt <= 0:
            return 0.0
        p = self.kp * error
        self._integral += error * dt
        max_integral = self.output_limit / max(self.ki, 0.001)
        self._integral = max(-max_integral, min(max_integral, self._integral))
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


# ── MPU6050 Sensor ─────────────────────────────────────

class MPU6050Sensor:
    """
    Reads Gyro Z-axis yaw rate from the MPU6050 over I2C.
    Used for telemetry display; the slalom driving logic does not
    depend on it.
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
    Maps slalom.py's set_motors() calls to the CarSystem abstraction.
    """

    def __init__(self, car):
        self._car = car

    def forward_differential(self, left_speed, right_speed):
        """Drive forward with explicit left/right wheel speeds (0-100 %)."""
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        self._car._set_raw_motors(left_speed, right_speed, True, True)
        self._car._current_speed = (left_speed + right_speed) / 2.0

    def reverse(self, speed, duration=0.8):
        """Reverse at *speed* for *duration* seconds, then stop."""
        end = time.time() + duration
        while time.time() < end:
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
    Slalom-style autonomous rover — exact replica of slalom.py's
    4-priority logic, with tunable distances and speeds.

    Priority 1: TRAP ESCAPE     (both IRs OR dist < TRAP_ESCAPE_CM)
    Priority 2: SIDE OBSTACLE   (single IR → hard swerve)
    Priority 3: PROPORTIONAL    (dist < DODGE_DETECT_CM → differential dodge)
    Priority 4: CRUISE          (clear path → straight ahead)
    """

    # ── Speed tuning ──
    BASE_SPEED       = 50     # Cruising PWM %
    ESCAPE_SPEED     = 70     # Reverse / spin PWM % during trap escape
    TURN_AGGRESSION  = 1.8    # Proportional dodge multiplier

    # ── IR swerve speeds ──
    IR_SWERVE_FAST   = 60     # Fast-side PWM % for IR hard swerve
    IR_SWERVE_SLOW   = 20     # Slow-side PWM % for IR hard swerve

    # ── Distance thresholds (tunable) ──
    TRAP_ESCAPE_CM   = 15     # P1: both IRs OR sonar < this → trap escape
    DODGE_DETECT_CM  = 50     # P3: sonar < this → proportional dodge

    # ── Escape timing ──
    ESCAPE_STOP_TIME     = 0.2    # Pause before reversing (seconds)
    ESCAPE_REVERSE_TIME  = 0.8    # Reverse duration (seconds)
    ESCAPE_SPIN_TIME     = 0.5    # Spin duration (seconds)
    ESCAPE_RESUME_TIME   = 0.2    # Pause after spin before resuming (seconds)

    # ── IR swerve timing ──
    IR_SWERVE_TIME   = 0.2    # Duration of IR hard swerve (seconds)

    # ── Rear sonar ──
    REAR_CLEAR_CM    = 20     # Rear must be > this to reverse (cm)

    # ── Gyro / PID (telemetry & calibration) ──
    CALIBRATION_TIME = 2.0    # Gyro calibration duration (seconds)
    STATUS_INTERVAL  = 0.5    # Status print interval (seconds)

    TUNING_KEYS = [
        # Speeds
        "BASE_SPEED", "ESCAPE_SPEED", "TURN_AGGRESSION",
        # IR swerve
        "IR_SWERVE_FAST", "IR_SWERVE_SLOW",
        # Distances
        "TRAP_ESCAPE_CM", "DODGE_DETECT_CM",
        # Escape timing
        "ESCAPE_STOP_TIME", "ESCAPE_REVERSE_TIME",
        "ESCAPE_SPIN_TIME", "ESCAPE_RESUME_TIME",
        # IR swerve timing
        "IR_SWERVE_TIME",
        # Rear sonar
        "REAR_CLEAR_CM",
        # Gyro
        "CALIBRATION_TIME", "STATUS_INTERVAL",
    ]

    def __init__(self, car, get_sonar, get_ir,
                 get_rear_sonar=None, get_camera_distance=None):
        self._motor = MotorDriver(car)
        self._car = car

        self._front_sonar = Sonar(get_sonar, "front")
        self._rear_sonar = Sonar(get_rear_sonar, "rear")
        self._get_ir = get_ir
        self._get_camera_distance = get_camera_distance  # stored for API compat

        # Gyro (telemetry only — slalom logic doesn't depend on it)
        self._gyro = MPU6050Sensor()
        self._pid = PIDController()

        # State
        self._state = State.FORWARD_CRUISE
        self._active = False
        self._turn_direction = ""
        self._last_turn_dir = 1     # 1 = right, -1 = left (matches slalom.py)
        self._last_status_time = 0.0

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
        self._state = State.FORWARD_CRUISE
        self._turn_direction = ""
        self._last_turn_dir = 1
        self._last_status_time = time.time()
        self._active = True
        print("🚀 [ROVER] Autonomous navigation STARTED (slalom mode)")

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
            gyro_str = f" | Gyro_Z: {self._last_gyro_z:+.2f}°/s" if self._gyro.available else ""
            print(f"📡 [{state_name}] Front: {dist:.0f}cm{gyro_str} | {msg}")

    # ── Trap Escape (slalom.py escape_trap) ────

    def _escape_trap(self):
        """
        Exact replica of slalom.py escape_trap():
        stop → reverse → random spin → resume.
        """
        self._state = State.OBSTACLE_FALLBACK
        self._turn_direction = "escaping"

        # 1. STOP
        self._motor.stop()
        time.sleep(self.ESCAPE_STOP_TIME)
        if not self._active:
            return

        # 2. REVERSE (check rear first)
        rear_dist = self._rear_sonar.read()
        if rear_dist < 0 or rear_dist > self.REAR_CLEAR_CM:
            print(f"    ↩️  Reversing {self.ESCAPE_REVERSE_TIME:.1f}s "
                  f"(rear: {rear_dist:.0f}cm)")
            self._motor.reverse(self.ESCAPE_SPEED, self.ESCAPE_REVERSE_TIME)
        else:
            print(f"    ⛔ Rear blocked ({rear_dist:.0f}cm) — skip reverse")

        if not self._active:
            return

        # 3. SPIN (Random Direction — exactly like slalom.py)
        if random.choice([True, False]):
            print(f"    🔄 Random spin RIGHT {self.ESCAPE_SPIN_TIME:.1f}s")
            self._motor.spin_right(self.ESCAPE_SPEED, self.ESCAPE_SPIN_TIME)
        else:
            print(f"    🔄 Random spin LEFT {self.ESCAPE_SPIN_TIME:.1f}s")
            self._motor.spin_left(self.ESCAPE_SPEED, self.ESCAPE_SPIN_TIME)

        if not self._active:
            return

        # 4. Resume
        self._motor.stop()
        time.sleep(self.ESCAPE_RESUME_TIME)

        self._turn_direction = ""
        self._state = State.FORWARD_CRUISE
        print("    ✅ Trap escape complete — resuming cruise\n")

    # ── FSM Core ───────────────────────────────

    def update(self):
        """
        Single tick of the slalom autopilot.

        Mirrors slalom.py's auto_pilot_thread() priorities exactly:
          P1: Trap escape  (both IRs OR dist < TRAP_ESCAPE_CM)
          P2: IR swerve    (single IR → hard swerve)
          P3: Prop. dodge  (dist < DODGE_DETECT_CM → differential)
          P4: Cruise       (clear path → straight)
        """
        if not self._active:
            return

        if self._state == State.EMERGENCY_STOP:
            self._motor.brake()
            return

        # ── Read gyro for telemetry ──
        self._last_gyro_z = self._gyro.read_gyro_z()

        # ── 1. READ SENSORS (exactly like slalom.py) ──
        dist = self._front_sonar.read()
        left_obs, right_obs = self._read_ir()

        # ─────────────────────────────────────────────
        # 🚨 PRIORITY 1: TRAP ESCAPE
        #    Both IRs OR Sonar < TRAP_ESCAPE_CM
        #    (slalom.py: both IRs or dist < 15)
        # ─────────────────────────────────────────────
        if (left_obs and right_obs) or (0 < dist < self.TRAP_ESCAPE_CM):
            reason = "both IRs" if (left_obs and right_obs) else f"{dist:.0f}cm < {self.TRAP_ESCAPE_CM}cm"
            print(f"\n🚨 TRAP ESCAPE ({reason})")
            self._escape_trap()
            return  # Restart loop after escaping

        # ─────────────────────────────────────────────
        # ⚠️ PRIORITY 2: SIDE OBSTACLE (One IR Triggered)
        #    (slalom.py: left_obs → swerve right 60/20,
        #                right_obs → swerve left 20/60)
        # ─────────────────────────────────────────────
        elif left_obs:
            self._state = State.OBSTACLE_STEERING
            self._turn_direction = "right"
            self._last_turn_dir = 1
            self._motor.forward_differential(
                self.IR_SWERVE_FAST, self.IR_SWERVE_SLOW)
            self._last_pid_correction = (self.IR_SWERVE_FAST - self.IR_SWERVE_SLOW) / 2.0
            self._maybe_print_status(
                dist if dist >= 0 else 300, "SWERVE RIGHT (IR)")
            time.sleep(self.IR_SWERVE_TIME)
            return

        elif right_obs:
            self._state = State.OBSTACLE_STEERING
            self._turn_direction = "left"
            self._last_turn_dir = -1
            self._motor.forward_differential(
                self.IR_SWERVE_SLOW, self.IR_SWERVE_FAST)
            self._last_pid_correction = -(self.IR_SWERVE_FAST - self.IR_SWERVE_SLOW) / 2.0
            self._maybe_print_status(
                dist if dist >= 0 else 300, "SWERVE LEFT (IR)")
            time.sleep(self.IR_SWERVE_TIME)
            return

        # ─────────────────────────────────────────────
        # 👀 PRIORITY 3: PROPORTIONAL DODGE
        #    Sonar < DODGE_DETECT_CM
        #    (slalom.py: dist < 50 → TURN_AGGRESSION differential)
        # ─────────────────────────────────────────────
        elif 0 < dist < self.DODGE_DETECT_CM:
            self._state = State.OBSTACLE_STEERING

            # The closer → the sharper the turn
            avoid_strength = (self.DODGE_DETECT_CM - dist) * self.TURN_AGGRESSION

            # Base speed + turn factor (exactly like slalom.py)
            if self._last_turn_dir == 1:  # Turn Right
                l_speed = self.BASE_SPEED + avoid_strength
                r_speed = self.BASE_SPEED - avoid_strength
                self._turn_direction = "right"
            else:  # Turn Left
                l_speed = self.BASE_SPEED - avoid_strength
                r_speed = self.BASE_SPEED + avoid_strength
                self._turn_direction = "left"

            self._motor.forward_differential(l_speed, r_speed)
            self._last_pid_correction = avoid_strength if self._last_turn_dir == 1 else -avoid_strength

            self._maybe_print_status(
                dist,
                f"DODGING ({dist:.0f}cm) | Avoid: {avoid_strength:.0f} | "
                f"L:{l_speed:.0f}/R:{r_speed:.0f}")
            return

        # ─────────────────────────────────────────────
        # 🚀 PRIORITY 4: CRUISE (Clear Path)
        #    (slalom.py: set_motors(BASE_SPEED, BASE_SPEED, 1))
        # ─────────────────────────────────────────────
        else:
            self._state = State.FORWARD_CRUISE
            self._turn_direction = ""
            self._last_pid_correction = 0.0
            self._motor.forward_differential(self.BASE_SPEED, self.BASE_SPEED)
            self._maybe_print_status(
                dist if dist >= 0 else 300, "CRUISING")
            return


# ── Standalone test ────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  AUTONOMOUS ROVER — Slalom Autopilot")
    print("=" * 60)

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
