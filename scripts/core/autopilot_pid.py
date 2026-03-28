"""
autopilot_pid.py — Autonomous Rover with Forward Obstacle Memory + Polar Hazard Map

6-State Finite State Machine
─────────────────────────────
  CRUISE           Drive forward with PID; planner scores steering arcs.
  CAUTION          Obstacle nearby — reduce speed, let planner steer around it.
  PLAN             All easy arcs poor; stop briefly to let planner re-evaluate.
  PROBE            Small cautious pivot to gather new laser data.
  RECOVER          Blocking reverse + spin fallback (last resort).
  EMERGENCY_STOP   Manual kill — all motors locked.

  Legacy states (CRUISING, SCANNING, TURNING) kept as aliases so
  external code referencing them does not break.

Sensor Stack (via Pico UART Bridge)
────────────
  VL53L0X (fixed forward, centre)  Pico I2C → UART bridge — single fixed beam
  MPU6050          Pico I2C 0x68 → UART bridge — gyro Z-axis yaw rate for PID
  ADS1115 ADC      Pico I2C 0x48 — battery voltage & current sensing

Steering: Ackermann (single front servo linkage, rear 2WD)
Odometry: UKF fused (encoders + gyro + magnetometer + steering angle)

Planner Logic
─────────────
  1. Record forward laser into world-frame obstacle memory each tick.
  2. Reproject memory into body frame → polar hazard map.
  3. Score candidate steering arcs (cost = collision + proximity + unknown + smoothness).
  4. Pick lowest-cost arc → drive along it briefly → replan next tick.
  5. If all arcs blocked → PROBE (pivot whole rover) → re-score → RECOVER if still stuck.

Wraps CarSystem (motor.py).  VL53L0X is fixed centre-forward — no scanning servo.
"""

import math
import time
import threading
from collections import deque
from enum import Enum

from obstacle_memory import ForwardObstacleMemory, ARC_BLOCKED_THRESHOLD, PROBE_ANGLE_DEG


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    # New planner-aware states
    CRUISE          = "CRUISE"
    CAUTION         = "CAUTION"
    PLAN            = "PLAN"
    PROBE           = "PROBE"
    RECOVER         = "RECOVER"
    EMERGENCY_STOP  = "EMERGENCY_STOP"

    # Legacy aliases (so external code using State.CRUISING still works)
    CRUISING        = "CRUISE"
    SCANNING        = "PLAN"
    TURNING         = "CAUTION"


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
    High-level motor commands with PID heading correction.

    Wraps CarSystem and applies PID correction as Ackermann steering
    angle adjustment for straight-line heading control.
    """

    def __init__(self, car, min_correction=15.0, noise_gate=1.0):
        """
        Parameters
        ----------
        car : motor.CarSystem
            The existing low-level motor controller.
        min_correction : float
            Minimum PID correction (degrees) when a correction is requested.
            Ensures servo overcomes deadband.
        noise_gate : float
            PID output below this is treated as zero (gyro noise filter).
        """
        self._car = car
        self._min_correction = min_correction
        self._noise_gate = noise_gate

    def forward(self, base_speed, correction=0.0):
        """
        Drive forward with PID heading correction via steering servo.

        correction > 0 → drifting right → steer left (negative angle).
        correction < 0 → drifting left  → steer right (positive angle).

        Parameters
        ----------
        base_speed : float
            Base PWM % (0-100).
        correction : float
            PID output mapped to steering angle adjustment.
        """
        if abs(correction) < self._noise_gate:
            correction = 0.0
        elif 0 < correction < self._min_correction:
            correction = self._min_correction
        elif -self._min_correction < correction < 0:
            correction = -self._min_correction

        # PID correction maps to steering angle (negative = steer left)
        steer_angle = max(-45, min(45, -correction))
        self._car._apply_steering(base_speed, steer_angle, forward=True)
        self._car._current_speed = base_speed

    def reverse(self, speed, duration=0.8, target_dist_m=0.20):
        """Reverse at *speed* until *target_dist_m* reached or *duration* timeout.

        Uses odometry for distance measurement when available, falls back to
        time-based duration if odometry is not active.
        """
        try:
            from odometry_integration import get_position
            sx, sy = get_position()
            use_odom = True
        except Exception:
            use_odom = False

        end = time.time() + duration
        while time.time() < end:
            self._car.reverse(speed)
            time.sleep(0.02)
            if use_odom:
                cx, cy = get_position()
                dx, dy = cx - sx, cy - sy
                if (dx * dx + dy * dy) >= target_dist_m * target_dist_m:
                    break
        self._car.stop()

    def spin_right(self, speed=50, duration=0.4, target_angle_deg=45.0):
        """Full-lock right turn until *target_angle_deg* reached or *duration* timeout.

        Uses odometry heading when available, falls back to time-based.
        """
        try:
            from odometry_integration import get_heading_deg
            start_hdg = get_heading_deg()
            use_odom = True
        except Exception:
            use_odom = False

        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("right", speed)
            time.sleep(0.02)
            if use_odom:
                delta = abs(get_heading_deg() - start_hdg)
                if delta > 180:
                    delta = 360 - delta
                if delta >= target_angle_deg:
                    break
        self._car.stop()

    def spin_left(self, speed=50, duration=0.4, target_angle_deg=45.0):
        """Full-lock left turn until *target_angle_deg* reached or *duration* timeout."""
        try:
            from odometry_integration import get_heading_deg
            start_hdg = get_heading_deg()
            use_odom = True
        except Exception:
            use_odom = False

        end = time.time() + duration
        while time.time() < end:
            self._car.pivot_turn("left", speed)
            time.sleep(0.02)
            if use_odom:
                delta = abs(get_heading_deg() - start_hdg)
                if delta > 180:
                    delta = 360 - delta
                if delta >= target_angle_deg:
                    break
        self._car.stop()

    def stop(self):
        """Gentle coast stop."""
        self._car.stop()

    def forward_steer(self, speed, angle):
        """
        Drive forward with a steering angle via Ackermann servo.

        Parameters
        ----------
        speed : float
            Forward PWM % (0-100).
        angle : float
            Steering angle. Negative = left, positive = right.
            Magnitude 0-90; deadzone ±5° treated as straight.
        """
        if abs(angle) <= 5:
            angle = 0
        self._car._apply_steering(speed, angle, forward=True)
        self._car._current_speed = speed

    def brake(self):
        """Magnetic lock brake (H-bridge short-circuit)."""
        self._car.brake()



# ── Autonomous Rover (AutoPilot) ──────────────────────

class AutoPilot:
    """
    Planner-based Autonomous Rover — 6-State FSM with Ackermann steering.

    States
    ------
    CRUISE         Drive forward with PID heading correction + planner arc scoring.
    CAUTION        Obstacle nearby — reduce speed, planner steers around it.
    PLAN           All arcs poor — brief stop, re-evaluate.
    PROBE          Small cautious pivot (whole rover) to gather new laser data.
    RECOVER        Blocking reverse + spin fallback (last resort).
    EMERGENCY_STOP Manual kill — brake.  Only start() exits.

    Hardware
    --------
    VL53L0X     Fixed centre-forward (no scanning servo).
    Steering    Ackermann servo linkage (±50°).
    Drive       2WD rear, dual independent PID RPM control.
    Odometry    UKF fused (encoders + gyro + magnetometer + steering angle).

    Parameters
    ----------
    car : motor.CarSystem
        Low-level Ackermann motor controller.
    get_forward_distance : callable → float
        Returns forward VL53L0X distance in cm (fixed sensor, no servo).
    get_rear_distance : callable → float | None
        Rear distance in cm (optional; currently unused — no rear sensor).
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
    TURN_MAX_ANGLE   = 50     # Maximum pivot angle (degrees)

    # Fallback (reverse + spin)
    REAR_CLEAR_CM    = 20
    REVERSE_SPEED    = 70
    REVERSE_DURATION = 1.2
    SPIN_SPEED       = 60
    SPIN_DURATION    = 0.6
    STOP_PAUSE       = 0.3

    # Emergency
    EMERGENCY_STOP_CM = 10

    # ── Planner constants ──────────────────────────────
    CAUTION_CM        = 80     # Start planner steering below this range
    CAUTION_SPEED     = 35     # Reduced speed in CAUTION
    PLAN_PAUSE_S      = 0.15   # Brief stop in PLAN before re-scoring
    PROBE_SPEED       = 30     # Slow creep during PROBE
    PROBE_TIMEOUT_S   = 1.0    # Max time in PROBE before RECOVER
    PROBE_ANGLE_DEG   = PROBE_ANGLE_DEG  # from obstacle_memory defaults
    REPLAN_ARC_BLOCKED = ARC_BLOCKED_THRESHOLD  # cost threshold

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
        # Planner
        "CAUTION_CM", "CAUTION_SPEED",
        "PLAN_PAUSE_S", "PROBE_SPEED", "PROBE_TIMEOUT_S",
    ]

    def __init__(self, car, get_forward_distance,
                 sensor_system=None, get_rear_distance=None):
        # Motor wrapper (Ackermann steering + 2WD rear drive)
        self._motor = MotorDriver(car, min_correction=self.MIN_CORRECTION_PWM,
                                  noise_gate=self.CORRECTION_NOISE_GATE)
        self._car = car

        # Distance sensor (threaded + median) — VL53L0X fixed centre-forward
        self._front_sensor = DistanceSensor(get_forward_distance, "front_laser")

        # Legacy: sensor_system accepted but not used for scanning
        # (VL53L0X is fixed; servo only has camera now)
        self._sensor_system = sensor_system

        # Rear distance (optional — currently no rear sensor)
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

        # ── Forward Obstacle Memory + Planner ──────────
        self._planner = ForwardObstacleMemory()
        self._current_steer_deg = 0.0  # last commanded steering angle
        self._plan_enter_time = 0.0    # timestamp when PLAN entered
        self._probe_enter_time = 0.0   # timestamp when PROBE entered
        self._probe_direction = 1      # +1 right, -1 left
        self._last_planner_log_time = 0.0

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

    @property
    def planner_debug(self):
        """Latest PlannerDebug snapshot (dict) for telemetry."""
        return self._planner.debug.to_dict()

    @property
    def planner_memory(self):
        """Forward obstacle memory instance (for external telemetry)."""
        return self._planner

    # ── Tuning API ─────────────────────────────────

    @classmethod
    def get_default_tuning(cls):
        return {key: getattr(cls, key) for key in cls.TUNING_KEYS}

    def get_tuning(self):
        return {key: getattr(self, key) for key in self.TUNING_KEYS}

    # ── Lifecycle ──────────────────────────────────

    def start(self):
        """Calibrate gyro, reset PID, begin CRUISE."""
        self._gyro.calibrate(duration=self.CALIBRATION_TIME)
        self._gyro_calibrated = self._gyro.available
        self._pid = PIDController(
            kp=self.PID_KP, ki=self.PID_KI,
            kd=self.PID_KD, output_limit=self.PID_LIMIT,
        )
        self._state = State.CRUISE
        self._turn_direction = ""
        self._dodge_direction = 0
        self._heading = 0.0
        self._target_yaw = 0.0
        self._last_status_time = time.time()
        self._last_pid_time = time.time()
        self._last_scan_data = []
        self._scan_data_fresh = False
        self._current_steer_deg = 0.0
        self._plan_enter_time = 0.0
        self._probe_enter_time = 0.0
        self._probe_direction = 1

        # Reset planner memory
        self._planner.clear()

        # Centre steering servo (start straight)
        self._car.set_steering(0)

        self._active = True
        print("🚀 [ROVER] Planner-based autonomous navigation STARTED")

    def stop(self):
        """Halt motors, centre steering, deactivate FSM."""
        self._active = False
        self._motor.brake()
        self._pid.reset()
        self._last_gyro_z = 0.0
        self._last_pid_correction = 0.0
        self._heading = 0.0
        self._target_yaw = 0.0
        self._dodge_direction = 0
        self._turn_direction = ""
        self._current_steer_deg = 0.0
        self._planner.clear()
        # Centre steering servo on stop
        self._car.set_steering(0)
        print("🛑 [ROVER] Planner-based autonomous navigation STOPPED")

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

    def _maybe_log_planner(self, transition, front_dist, planner_result):
        """Rate-limited planner state-transition log."""
        now = time.time()
        if now - self._last_planner_log_time < 0.3:
            return
        self._last_planner_log_time = now
        dbg = self._planner.debug
        extra = ""
        if planner_result is not None:
            extra = (f" | best:{planner_result.best_arc_deg:+.0f}° "
                     f"cost:{planner_result.best_arc_cost:.0f} "
                     f"map:[{dbg.hazard_bins_summary}]")
        print(f"🗺️  [{transition}] Front:{front_dist:.0f}cm "
              f"obs:{dbg.obstacle_count} clr:{dbg.clear_corridor_count}{extra}")

    # ── Odometry helpers (lazy import) ──────────────

    @staticmethod
    def _get_odom_pose():
        """Return (x, y, heading_rad) from fused odometry, or None."""
        try:
            from odometry_integration import get_position, get_heading
            x, y = get_position()
            heading = get_heading()  # radians
            return x, y, heading
        except Exception:
            return None

    # ── FSM Core ───────────────────────────────────

    def update(self):
        """Single tick of the 6-state FSM (called at ~20 Hz).

        States: CRUISE → CAUTION → PLAN → PROBE → RECOVER → CRUISE
        EMERGENCY_STOP is always checked first and stays locked.
        """
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

        # ── Read sensors ──────────────────────────────
        front_dist = self._front_sensor.read()   # cm, -1 on error
        display_dist = front_dist if front_dist >= 0 else 999
        front_m = front_dist / 100.0 if front_dist > 0 else 999.0

        # ── Emergency brake (ALL states except EMERGENCY_STOP) ──
        if 0 < front_dist < self.EMERGENCY_STOP_CM:
            self._motor.brake()
            print(f"\n🚨 [EMERGENCY BRAKE] Obstacle at {front_dist:.0f}cm!")
            self._state = State.PLAN
            self._plan_enter_time = now
            return

        # ── Feed planner with pose + laser ────────────
        pose = self._get_odom_pose()
        if pose is not None:
            rx, ry, rh = pose
            planner_result = self._planner.plan(
                rx, ry, rh, front_m, self._current_steer_deg
            )
        else:
            planner_result = None

        # ── STATE: CRUISE ────────────────────────────
        if self._state == State.CRUISE:
            self._turn_direction = ""
            self._dodge_direction = 0

            # Transition to CAUTION when obstacle within caution range
            if 0 < front_dist < self.CAUTION_CM:
                self._state = State.CAUTION
                self._maybe_log_planner("CRUISE→CAUTION", front_dist, planner_result)
                # Fall through to CAUTION handling below
            else:
                # Normal PID cruise — open road
                correction = self._pid.update(gyro_z, dt)
                self._last_pid_correction = correction
                self._motor.forward(self.BASE_SPEED, correction)
                self._maybe_print_status(display_dist, gyro_z, correction)
                return

        # ── STATE: CAUTION ───────────────────────────
        if self._state == State.CAUTION:
            # Use planner arc scoring to steer around obstacles
            if planner_result is not None and not planner_result.all_arcs_blocked:
                steer_deg = planner_result.best_arc_deg
                self._current_steer_deg = steer_deg
                self._dodge_direction = -1 if steer_deg < -3 else (1 if steer_deg > 3 else 0)
                self._turn_direction = (
                    "left" if steer_deg < -3 else "right" if steer_deg > 3 else ""
                )

                # Use reduced speed when near obstacles
                speed = self.CAUTION_SPEED if front_dist < self.OBSTACLE_CM else self.BASE_SPEED
                self._motor.forward_steer(speed, steer_deg)

                # Return to CRUISE if obstacle clears
                if front_dist < 0 or front_dist >= self.CAUTION_CM:
                    self._state = State.CRUISE
                    self._pid.reset()
                    self._last_pid_time = time.time()

                self._maybe_print_status(
                    display_dist, gyro_z, steer_deg,
                    f"Arc:{steer_deg:+.0f}° cost:{planner_result.best_arc_cost:.0f}"
                )
                return

            # All arcs blocked → PLAN
            self._motor.stop()
            self._state = State.PLAN
            self._plan_enter_time = now
            self._maybe_log_planner("CAUTION→PLAN", front_dist, planner_result)
            return

        # ── STATE: PLAN ──────────────────────────────
        if self._state == State.PLAN:
            # Brief pause to let planner re-evaluate (non-blocking on next tick)
            elapsed = now - self._plan_enter_time
            self._motor.stop()

            if elapsed < self.PLAN_PAUSE_S:
                return  # wait one more tick

            # Re-score arcs after pause
            if planner_result is not None and not planner_result.all_arcs_blocked:
                # Found a way out → resume CAUTION
                self._state = State.CAUTION
                self._maybe_log_planner("PLAN→CAUTION", front_dist, planner_result)
                return

            # Still blocked → enter PROBE
            self._state = State.PROBE
            self._probe_enter_time = now
            # Alternate probe direction each time
            self._probe_direction *= -1
            self._maybe_log_planner("PLAN→PROBE", front_dist, planner_result)
            return

        # ── STATE: PROBE ─────────────────────────────
        if self._state == State.PROBE:
            elapsed = now - self._probe_enter_time

            if elapsed > self.PROBE_TIMEOUT_S:
                # Probe timed out → RECOVER (fallback)
                self._motor.stop()
                self._state = State.RECOVER
                print("    ⏳ PROBE timeout → RECOVER")
                return

            # Small cautious steering to gather new observations
            probe_steer = self.PROBE_ANGLE_DEG * self._probe_direction
            self._motor.forward_steer(self.PROBE_SPEED, probe_steer)
            self._current_steer_deg = probe_steer

            # Check if planner found a path after gathering new info
            if planner_result is not None and not planner_result.all_arcs_blocked:
                self._motor.stop()
                self._state = State.CAUTION
                self._maybe_log_planner("PROBE→CAUTION", front_dist, planner_result)
                return

            # Emergency check during probe
            if 0 < front_dist < self.EMERGENCY_STOP_CM:
                self._motor.brake()
                self._state = State.RECOVER
                return

            self._maybe_print_status(
                display_dist, gyro_z, probe_steer,
                f"PROBE {elapsed:.1f}s dir:{'R' if self._probe_direction > 0 else 'L'}"
            )
            return

        # ── STATE: RECOVER ───────────────────────────
        if self._state == State.RECOVER:
            self._do_fallback()
            return

    # ── Fallback manoeuvre (reverse + spin) — odometry-aware ────────

    # Distance/angle targets for odometry-based fallback
    REVERSE_DIST_M   = 0.20   # reverse 20 cm
    SPIN_ANGLE_DEG   = 60.0   # spin 60 degrees

    def _do_fallback(self):
        """Blocking: stop, reverse (distance-based), spin (angle-based), resume.

        When odometry is active, uses distance/angle targets instead of
        fixed durations.  Falls back to time-based if odometry unavailable.
        """
        self._motor.stop()
        time.sleep(self.STOP_PAUSE)
        if not self._active:
            return

        # Reverse if rear clear
        rear_dist = self._read_rear()
        if rear_dist < 0 or rear_dist > self.REAR_CLEAR_CM:
            print(f"    ↩️  Reversing ~{self.REVERSE_DIST_M*100:.0f}cm "
                  f"(rear: {rear_dist:.0f}cm)")
            self._motor.reverse(self.REVERSE_SPEED,
                                duration=self.REVERSE_DURATION,
                                target_dist_m=self.REVERSE_DIST_M)
        else:
            print(f"    ⛔ Rear blocked ({rear_dist:.0f}cm) — skip reverse")

        if not self._active:
            return
        self._motor.stop()
        time.sleep(0.1)
        if not self._active:
            return

        # Spin (alternate direction) — angle-based with time fallback
        import random
        spin_dir = random.choice(["left", "right"])
        print(f"    🔄 Spinning {spin_dir} ~{self.SPIN_ANGLE_DEG:.0f}°")
        if spin_dir == "left":
            self._motor.spin_left(self.SPIN_SPEED,
                                  duration=self.SPIN_DURATION,
                                  target_angle_deg=self.SPIN_ANGLE_DEG)
        else:
            self._motor.spin_right(self.SPIN_SPEED,
                                   duration=self.SPIN_DURATION,
                                   target_angle_deg=self.SPIN_ANGLE_DEG)

        if not self._active:
            return

        # Reset and resume
        self._pid.reset()
        self._last_pid_time = time.time()
        self._heading = 0.0
        self._target_yaw = 0.0
        self._turn_direction = ""
        self._dodge_direction = 0
        self._current_steer_deg = 0.0
        self._state = State.CRUISE
        print("    ✅ Fallback complete — resuming CRUISE\n")


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
