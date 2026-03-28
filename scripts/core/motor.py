"""motor.py — CarSystem: Motor controller for 2WD Ackermann-steering car.

Architecture:
  - Pico W controls 2 rear motors (L298N) + 1 steering servo (Ackermann)
  - Pi sends commands over UART via pico_sensor_reader module
  - Steering is mechanical (servo), not differential (wheel speed)

Provides a clean API for the AutoPilot and manual control:
  - set_speed(0-100)       Forward at given PWM %
  - set_steering(-50..+50) Ackermann servo angle (negative=left, positive=right)
  - reverse(speed)         Straight reverse at given PWM %
  - reverse_steer(speed, angle) Reverse with steering
  - pivot_turn("left"|"right", speed)  Full-lock turn (Ackermann max angle + creep)
  - brake()                Magnetic lock (H-bridge short-circuit braking via Pico)
  - stop()                 Gentle stop (coast via Pico)
  - cleanup()              Center steering and stop motors
"""

import time
import threading


# ──────────────────────────────────────────────
# Voltage-based PWM hard limit  (legacy static fallback)
# ──────────────────────────────────────────────
# Motors are rated 12 V; supply is 3S Li-ion (11.1 V nominal).
# With L298N drop (~2 V), effective motor voltage ≈ 9 V at 100% duty.
# These static values are kept for backward-compatibility with
# standalone scripts that import MAX_PWM_DUTY at module level.
# In the live system, CarSystem.power_limiter computes a *dynamic*
# duty-cycle cap from real-time battery voltage and current sensing.
BATTERY_VOLTAGE   = 11.1   # 3S Li-ion pack (3 × 3.7 V)  — fallback only
MOTOR_MAX_VOLTAGE = 12.0   # 12V rated motors — safe to run at full supply
MAX_PWM_DUTY      = 95     # Allow near full duty with 12V motors on 11.1V supply


# ──────────────────────────────────────────────
# PowerLimiter — adaptive voltage-aware duty-cycle control
# ──────────────────────────────────────────────

class PowerLimiter:
    """Dynamically computes safe PWM duty-cycle limits from live sensor data.

    The algorithm ensures the *effective* motor voltage (after L298N losses)
    stays within a target window, currently 7–8 V for 6 V-rated DC motors.

    L298N voltage-drop model
    ────────────────────────
    The L298N uses BJT H-bridges with significant V_CE(sat) losses that
    increase with load current.  Per the L298N datasheet, the total
    saturation drop (V_CE(sat)H + V_CE(sat)L) is ~2.5 V at 1 A and
    ~3.2 V at 2 A.  We model this as:

        V_drop = V_BASE_DROP + R_ON × I_amps

    where V_BASE_DROP = 1.8 V is the quiescent dual-BJT saturation
    drop and R_ON = 0.6 Ω captures the resistive component.
    The result is clamped to [MIN_DROP, MAX_DROP] for safety.

    Duty-cycle conversion
    ─────────────────────
    The L298N is a physical switch between the battery and the motor.
    The voltage drop occurs only during the ON phase of the PWM signal,
    so the effective voltage available at the driver output is:

        V_effective = V_battery − V_drop

    To deliver a desired motor voltage V_motor, we keep the switch
    open for the right fraction of time:

        duty% = V_motor / (V_battery − V_drop) × 100

    This is capped at 95 % to prevent 100 % duty-cycle anomalies.

    Gear-to-voltage mapping
    ───────────────────────
    Each gear maps to a *voltage window* (min_V, max_V) representing
    the effective voltage at the motor terminals.  These are converted
    to duty-cycle ranges via the formula above, giving gears that adapt
    automatically to battery state and driver losses.
    """

    # Motor voltage safety envelope (effective volts at motor terminals)
    MOTOR_MIN_SAFE_V = 0.0    # Lowest effective motor voltage
    MOTOR_MAX_SAFE_V = 12.0   # 12V rated motors — safe at full supply voltage

    # L298N voltage-drop model parameters  (matched to L298N datasheet)
    V_BASE_DROP = 1.8         # Quiescent V_CE(sat) for two BJTs in series (V)
    R_ON        = 0.6         # Resistive component (Ω) — yields ~2.4 V @ 1 A, ~3.0 V @ 2 A
    MIN_DROP    = 1.8         # Floor on estimated drop (V)
    MAX_DROP    = 3.0         # Ceiling on estimated drop (V)

    # Hard duty-cycle ceiling — prevents 100 % anomalies
    MAX_DUTY_CAP = 95.0       # Never exceed 95 % duty cycle

    # Fallback values when sensor data is unavailable
    FALLBACK_VOLTAGE = 11.1   # Conservative 3S nominal (V)
    FALLBACK_DROP    = 2.4    # Mid-range driver drop estimate (V) @ ~1 A
    # Note: the static MAX_PWM_DUTY (≈63%) is more conservative and is
    # used as the absolute floor for the fallback path.

    # Gear → voltage window (effective motor-terminal voltage)
    #   12V rated motors on 11.1V 3S pack
    GEAR_VOLTAGE_MAP = {
        "N": (0.0, 0.0),       # No drive
        "1": (0.0, 3.5),       # Gentle / precision maneuvering
        "2": (3.5, 6.0),       # Moderate cruising
        "3": (6.0, 9.0),       # Fast driving
        "4": (6.0, 12.0),      # Top gear — full power
        "R": (0.0, 6.0),       # Moderate reverse
    }

    def __init__(self):
        self._lock = threading.Lock()

        # Sensor state
        self._battery_voltage = -1.0   # Last known battery voltage (V)
        self._current_amps    = 0.0    # Last known motor current  (A)

        # Cached computed values
        self._l298n_drop      = self.FALLBACK_DROP
        self._max_safe_duty   = MAX_PWM_DUTY   # Start with static fallback
        self._gear_ranges     = {}             # gear → (min_duty, max_duty)
        self._sensor_valid    = False

        # Pre-compute fallback gear ranges using static values
        self._recompute()

    # ── Public interface ─────────────────────────

    def update(self, battery_voltage: float, current_amps: float):
        """Feed fresh sensor readings and recompute all cached limits.

        Called from the sensor thread (~1 Hz).  Thread-safe.
        """
        with self._lock:
            self._battery_voltage = battery_voltage
            self._current_amps = max(0.0, current_amps) if current_amps >= 0 else 0.0
            self._sensor_valid = battery_voltage > 0
            self._recompute()

    @property
    def max_safe_duty(self) -> float:
        """Current dynamic duty-cycle cap (0-100).  Thread-safe read."""
        with self._lock:
            return self._max_safe_duty

    @property
    def l298n_drop(self) -> float:
        """Estimated L298N voltage drop (V).  Thread-safe read."""
        with self._lock:
            return self._l298n_drop

    @property
    def battery_voltage(self) -> float:
        """Last known battery voltage (V).  -1 if unknown."""
        with self._lock:
            return self._battery_voltage

    @property
    def sensor_valid(self) -> bool:
        """True when live sensor data is being used (not fallback)."""
        with self._lock:
            return self._sensor_valid

    def gear_duty_ranges(self) -> dict:
        """Return a dict of gear → (min_duty, max_duty).  Thread-safe."""
        with self._lock:
            return dict(self._gear_ranges)

    def voltage_to_duty(self, target_motor_v: float) -> float:
        """Convert a desired motor voltage to a PWM duty cycle (%).

        duty% = V_motor / (V_battery − V_drop) × 100

        Uses current battery voltage and L298N drop estimate.
        Must be called with self._lock held, or results may be stale.
        """
        v_batt = self._battery_voltage if self._sensor_valid else self.FALLBACK_VOLTAGE
        v_drop = self._l298n_drop

        # Effective voltage available at the driver output
        v_effective = v_batt - v_drop
        if v_effective <= 0:
            return 0.0

        duty = (target_motor_v / v_effective) * 100.0
        return max(0.0, min(self.MAX_DUTY_CAP, duty))

    def effective_motor_voltage(self, duty_pct: float) -> float:
        """Estimate the motor-terminal voltage for a given duty cycle.

        V_motor = (duty/100) × (V_battery − V_drop)

        The drop only occurs during the ON phase, so the effective
        voltage at the driver output is (V_battery − V_drop), and
        the motor sees that multiplied by the duty fraction.
        """
        with self._lock:
            v_batt = self._battery_voltage if self._sensor_valid else self.FALLBACK_VOLTAGE
            v_drop = self._l298n_drop
        v_motor = (duty_pct / 100.0) * (v_batt - v_drop)
        return max(0.0, v_motor)

    # ── Internal computation ─────────────────────

    def _recompute(self):
        """Recalculate all cached values.  Caller must hold self._lock."""

        if self._sensor_valid:
            v_batt = self._battery_voltage
            # Dynamic L298N drop from current sensor
            raw_drop = self.V_BASE_DROP + self.R_ON * self._current_amps
            self._l298n_drop = max(self.MIN_DROP, min(self.MAX_DROP, raw_drop))
        else:
            v_batt = self.FALLBACK_VOLTAGE
            self._l298n_drop = self.FALLBACK_DROP

        # Maximum safe duty cycle:  duty = V_max / (V_batt − V_drop)
        v_effective = v_batt - self._l298n_drop
        if v_effective > 0:
            raw_duty = (self.MOTOR_MAX_SAFE_V / v_effective) * 100.0
            self._max_safe_duty = max(0.0, min(self.MAX_DUTY_CAP, raw_duty))
        else:
            self._max_safe_duty = 0.0

        # If battery is dead / shorted, fall back to static limit
        if self._max_safe_duty <= 0:
            self._max_safe_duty = float(MAX_PWM_DUTY)

        # When sensor is invalid, cap to the conservative static limit
        if not self._sensor_valid:
            self._max_safe_duty = min(self._max_safe_duty, float(MAX_PWM_DUTY))

        # Gear duty ranges
        for gear, (v_min, v_max) in self.GEAR_VOLTAGE_MAP.items():
            d_min = self.voltage_to_duty(v_min) if v_min > 0 else 0.0
            d_max = self.voltage_to_duty(v_max) if v_max > 0 else 0.0
            # Clamp to overall safety limit
            d_max = min(d_max, self._max_safe_duty)
            d_min = min(d_min, d_max)
            self._gear_ranges[gear] = (round(d_min, 1), round(d_max, 1))

    def __repr__(self):
        with self._lock:
            return (
                f"PowerLimiter(V_batt={self._battery_voltage:.1f}V, "
                f"I={self._current_amps:.1f}A, "
                f"drop={self._l298n_drop:.1f}V, "
                f"max_duty={self._max_safe_duty:.1f}%, "
                f"sensor={'OK' if self._sensor_valid else 'FALLBACK'})"
            )

# ──────────────────────────────────────────────
# CarSystem — Ackermann steering + 2WD rear drive via Pico UART
# ──────────────────────────────────────────────

class CarSystem:
    """
    Motor interface for a 2WD rear-drive + Ackermann-steering car.

    All motor and steering commands are sent to the Pico W over UART.
    The Pico handles PWM ramping, direction changes, and servo control.

    Steering servo limits (from physically tested steering.py):
        CENTER = 1440µs, LEFT = 940µs, RIGHT = 2150µs
        Exceeding these limits risks mechanical damage!

    Steering angle convention:
        -50° = full left (940µs), 0° = straight (1440µs), +50° = full right (2150µs)
    """

    # Steering servo pulse width limits (µs) — LOCKED, do not change!
    STEER_CENTER_PW = 1440
    STEER_LEFT_PW   = 940
    STEER_RIGHT_PW  = 2150

    def __init__(self):
        # Internal bookkeeping
        self._current_speed = 0
        self._steering_angle = 0
        self._steering_pw = self.STEER_CENTER_PW
        self._is_forward = True

        # Adaptive voltage-aware power limiter (still useful for gear-based speed caps)
        self.power_limiter = PowerLimiter()

        # Current gear
        self._current_gear = "1"

        # ── RPM-based PID control (dual independent — matched to rpm_test.py) ──
        # Raw encoder objects (PicoEncoderProxy) for direct step reading
        self._encoder_left = None
        self._encoder_right = None
        # Step tracking for delta computation each PID tick
        self._prev_steps_l = 0
        self._prev_steps_r = 0
        self._rpm_last_time = time.monotonic()
        self._encoder_ppr = 330  # PIO counts direct pulses (1× per cycle)
        # Dual PID state — independent left/right
        self._rpm_integral_l = 0.0
        self._rpm_integral_r = 0.0
        self._rpm_prev_error_l = 0.0
        self._rpm_prev_error_r = 0.0
        self._rpm_pwm_out_l = 0.0    # Last PID output for left wheel
        self._rpm_pwm_out_r = 0.0    # Last PID output for right wheel
        self._rpm_pwm_out = 0.0      # Average of L/R for telemetry
        self._rpm_target = 0.0        # Current target RPM
        self._rpm_actual = 0.0        # Last measured (EMA-filtered) average RPM
        self._rpm_left = 0.0          # Last measured left RPM (EMA-filtered)
        self._rpm_right = 0.0         # Last measured right RPM (EMA-filtered)
        self._rpm_left_raw = 0.0      # Last measured left RPM (raw)
        self._rpm_right_raw = 0.0     # Last measured right RPM (raw)
        # PID gains — matched to proven rpm_test.py tuning
        self._rpm_kp = 0.3
        self._rpm_ki = 0.1
        self._rpm_kd = 0.05
        self._rpm_deadband = 15.0   # Minimum PWM to overcome gear friction
        self._rpm_ff_gain = 0.6     # Feedforward: PWM per RPM
        self._rpm_filter_alpha = 0.3  # EMA smoothing on measured RPM

        # ── Stall detection + dynamic power escalation ──
        # When target RPM > 0 but wheels aren't moving, progressively
        # increase PWM until they break free, then hand back to PID.
        self._stall_ticks_l = 0       # Consecutive stall ticks (left)
        self._stall_ticks_r = 0       # Consecutive stall ticks (right)
        self._stall_boost_l = 0.0     # Extra PWM added to left wheel
        self._stall_boost_r = 0.0     # Extra PWM added to right wheel
        _STALL_RPM_THRESH   = 3.0     # Below this RPM = wheel not moving
        _STALL_DETECT_TICKS = 3       # Ticks (60 ms) before escalation starts
        _STALL_BOOST_STEP   = 3.0     # PWM % added per stall tick
        _STALL_BOOST_MAX    = 40.0    # Maximum stall boost (prevents runaway)
        _STALL_MOVING_RPM   = 5.0     # Above this = wheel is moving, decay boost
        _STALL_BOOST_DECAY  = 2.0     # PWM % removed per tick once moving
        self._stall_cfg = {
            'rpm_thresh': _STALL_RPM_THRESH,
            'detect_ticks': _STALL_DETECT_TICKS,
            'boost_step': _STALL_BOOST_STEP,
            'boost_max': _STALL_BOOST_MAX,
            'moving_rpm': _STALL_MOVING_RPM,
            'boost_decay': _STALL_BOOST_DECAY,
        }

        # IMU accelerometer callback for motion detection
        self._get_accel_fn = None

        # Wheel sync placeholder
        self.wheel_sync = None

    def _angle_to_pw(self, angle):
        """Convert steering angle (-50..+50) to servo pulse width (940..2150µs)."""
        angle = max(-50, min(50, angle))
        if angle < 0:
            pw = self.STEER_CENTER_PW + (angle / 50.0) * (self.STEER_CENTER_PW - self.STEER_LEFT_PW)
        else:
            pw = self.STEER_CENTER_PW + (angle / 50.0) * (self.STEER_RIGHT_PW - self.STEER_CENTER_PW)
        return max(self.STEER_LEFT_PW, min(self.STEER_RIGHT_PW, int(pw)))

    def _send_drive(self, speed, steer_pw, forward=True):
        """Send motor + steering command to Pico via UART.
        Uses ML command for independent L/R control (same speed to both wheels)."""
        from pico_sensor_reader import send_lr_pwm
        duty_cap = self.power_limiter.max_safe_duty
        speed = max(0, min(duty_cap, speed))
        steer_pw = max(self.STEER_LEFT_PW, min(self.STEER_RIGHT_PW, int(steer_pw)))
        send_lr_pwm(int(speed), int(speed), steer_pw, forward=forward)

    def attach_wheel_sync(self, encoder_left, encoder_right):
        """Attach raw encoder objects for direct step-based RPM reading.
        encoder_left, encoder_right: LgpioEncoder instances (or any object with .steps).
        RPM is computed from step deltas inside rpm_pid_tick() each tick,
        exactly like rpm_test.py — no stale data from a separate thread."""
        self._encoder_left = encoder_left
        self._encoder_right = encoder_right
        self._prev_steps_l = encoder_left.steps if encoder_left else 0
        self._prev_steps_r = encoder_right.steps if encoder_right else 0
        self._rpm_last_time = time.monotonic()

    def attach_imu(self, get_accel_fn):
        """Attach IMU accelerometer callback for motion detection.
        get_accel_fn: callable returning (accel_x, accel_y, accel_z)."""
        self._get_accel_fn = get_accel_fn

    def is_car_moving(self, rpm_threshold=5.0, accel_threshold=0.3):
        """Detect if car is physically moving using encoder RPM + IMU accel."""
        # Primary: check encoder RPM (from PID's EMA-filtered actual RPM)
        if self._rpm_actual > rpm_threshold:
            return True
        # Secondary: check IMU lateral acceleration
        if self._get_accel_fn:
            try:
                ax, ay, _az = self._get_accel_fn()
                lateral = (ax ** 2 + ay ** 2) ** 0.5
                if lateral > accel_threshold:
                    return True
            except Exception:
                pass
        return False

    def set_gear(self, gear: str):
        """Update current gear for speed-limit enforcement."""
        if str(gear).upper() == "S":
            gear = "4"
        self._current_gear = gear or "1"

    def get_sync_telemetry(self) -> dict:
        """Return RPM control telemetry for dashboard."""
        return {
            'status': 'ACTIVE' if self._encoder_left else 'OFF',
            'target_rpm': round(self._rpm_target, 1),
            'actual_rpm': round(self._rpm_actual, 1),
            'rpm_left': round(self._rpm_left, 1),
            'rpm_right': round(self._rpm_right, 1),
            'pwm_out': round(self._rpm_pwm_out, 1),
            'pwm_out_l': round(self._rpm_pwm_out_l, 1),
            'pwm_out_r': round(self._rpm_pwm_out_r, 1),
            'gear': self._current_gear,
            'gear_max_rpm': self._get_gear_max_rpm(),
            'stall_boost_l': round(self._stall_boost_l, 1),
            'stall_boost_r': round(self._stall_boost_r, 1),
            'wheels': {},
        }

    def _get_gear_max_rpm(self) -> float:
        """Return max RPM allowed by current gear."""
        from wheel_sync import GEAR_RPM_LIMITS, DEFAULT_MAX_RPM
        _, max_rpm = GEAR_RPM_LIMITS.get(self._current_gear, (0, DEFAULT_MAX_RPM))
        return float(max_rpm)

    def get_target_rpm_cap(self) -> float:
        """Return the current gear's max RPM cap."""
        return self._get_gear_max_rpm()

    def rpm_pid_tick(self, target_rpm: float):
        """Run dual independent PID: target_rpm → (pwm_l, pwm_r).

        Called from physics_loop at 50 Hz.  Reads RPM from the Pico
        (calculated on-device from PIO encoders at 100 Hz), then runs
        independent PID loops for left and right wheels — identical to
        the proven rpm_test.py approach.

        Architecture:
          - get_pico_rpm() returns (rpm_l, rpm_r) from the Pico's latest
            UART packet (fresh at ~50 Hz, computed on Pico at 100 Hz).
          - EMA low-pass filter (alpha=0.3) per wheel
          - Feedforward (deadband + Kf × target) gives a base PWM estimate
          - Independent PID corrections handle per-wheel steady-state error
          - Measures actual dt via time.monotonic() (not a fixed constant)

        Returns (pwm_l, pwm_r) tuple — per-wheel duty cycles.
        """
        from pico_sensor_reader import get_pico_rpm

        self._rpm_target = target_rpm

        now = time.monotonic()
        dt = now - self._rpm_last_time
        self._rpm_last_time = now
        if dt <= 0:
            dt = 0.02  # safety fallback

        # ── Read RPM from Pico (calculated on-device) ──
        rpm_l_raw, rpm_r_raw = get_pico_rpm()
        self._rpm_left_raw = rpm_l_raw
        self._rpm_right_raw = rpm_r_raw

        # EMA low-pass filter per wheel (matched to rpm_test.py ALPHA=0.3)
        alpha = self._rpm_filter_alpha
        self._rpm_left = alpha * rpm_l_raw + (1.0 - alpha) * self._rpm_left
        self._rpm_right = alpha * rpm_r_raw + (1.0 - alpha) * self._rpm_right
        self._rpm_actual = (self._rpm_left + self._rpm_right) / 2.0

        if target_rpm <= 0:
            # Target is zero — reset PID state, keep _rpm_actual live
            # so is_car_moving() can detect coasting wheels
            self._rpm_integral_l = 0.0
            self._rpm_integral_r = 0.0
            self._rpm_prev_error_l = 0.0
            self._rpm_prev_error_r = 0.0
            self._rpm_pwm_out_l = 0.0
            self._rpm_pwm_out_r = 0.0
            self._rpm_pwm_out = 0.0
            # Reset stall detection state
            self._stall_ticks_l = 0
            self._stall_ticks_r = 0
            self._stall_boost_l = 0.0
            self._stall_boost_r = 0.0
            return (0.0, 0.0)

        # ── Feedforward: deadband + linear gain (matched to rpm_test.py) ──
        base_pwm = self._rpm_deadband + (target_rpm * self._rpm_ff_gain)

        duty_cap = self.power_limiter.max_safe_duty

        # ── PID LEFT ──
        error_l = target_rpm - self._rpm_left
        self._rpm_integral_l += error_l * dt
        self._rpm_integral_l = max(-1000, min(1000, self._rpm_integral_l))
        deriv_l = (error_l - self._rpm_prev_error_l) / dt if dt > 0 else 0.0
        self._rpm_prev_error_l = error_l

        pwm_l = base_pwm + (self._rpm_kp * error_l +
                             self._rpm_ki * self._rpm_integral_l +
                             self._rpm_kd * deriv_l)
        floor_l = self._rpm_deadband if target_rpm > 0 else 0.0
        pwm_l = max(floor_l, min(duty_cap, pwm_l))
        # Anti-windup: if clamped at floor, don't let integral wind down
        if pwm_l == floor_l and error_l < 0:
            self._rpm_integral_l -= error_l * dt

        # ── PID RIGHT ──
        error_r = target_rpm - self._rpm_right
        self._rpm_integral_r += error_r * dt
        self._rpm_integral_r = max(-1000, min(1000, self._rpm_integral_r))
        deriv_r = (error_r - self._rpm_prev_error_r) / dt if dt > 0 else 0.0
        self._rpm_prev_error_r = error_r

        pwm_r = base_pwm + (self._rpm_kp * error_r +
                             self._rpm_ki * self._rpm_integral_r +
                             self._rpm_kd * deriv_r)
        floor_r = self._rpm_deadband if target_rpm > 0 else 0.0
        pwm_r = max(floor_r, min(duty_cap, pwm_r))
        # Anti-windup: if clamped at floor, don't let integral wind down
        if pwm_r == floor_r and error_r < 0:
            self._rpm_integral_r -= error_r * dt

        # ── Stall detection + dynamic power escalation ──
        # Continuously check if each wheel is actually moving at the
        # expected RPM.  If a wheel is commanded but stalled (static
        # friction too high for current PWM), progressively add boost
        # PWM until it breaks free.  Once moving, decay the boost so
        # the PID resumes normal control.
        cfg = self._stall_cfg

        # LEFT wheel stall check
        if self._rpm_left < cfg['rpm_thresh']:
            self._stall_ticks_l += 1
            if self._stall_ticks_l >= cfg['detect_ticks']:
                self._stall_boost_l = min(
                    self._stall_boost_l + cfg['boost_step'],
                    cfg['boost_max']
                )
        else:
            self._stall_ticks_l = 0
        # Decay boost once wheel is moving
        if self._rpm_left >= cfg['moving_rpm'] and self._stall_boost_l > 0:
            self._stall_boost_l = max(0.0, self._stall_boost_l - cfg['boost_decay'])

        # RIGHT wheel stall check
        if self._rpm_right < cfg['rpm_thresh']:
            self._stall_ticks_r += 1
            if self._stall_ticks_r >= cfg['detect_ticks']:
                self._stall_boost_r = min(
                    self._stall_boost_r + cfg['boost_step'],
                    cfg['boost_max']
                )
        else:
            self._stall_ticks_r = 0
        # Decay boost once wheel is moving
        if self._rpm_right >= cfg['moving_rpm'] and self._stall_boost_r > 0:
            self._stall_boost_r = max(0.0, self._stall_boost_r - cfg['boost_decay'])

        # Apply stall boost on top of PID output
        pwm_l = min(duty_cap, pwm_l + self._stall_boost_l)
        pwm_r = min(duty_cap, pwm_r + self._stall_boost_r)

        self._rpm_pwm_out_l = pwm_l
        self._rpm_pwm_out_r = pwm_r
        self._rpm_pwm_out = (pwm_l + pwm_r) / 2.0

        return (pwm_l, pwm_r)

    # ── raw motor helper ─────────────────────────

    def _set_raw_motors(self, speed_l, speed_r, l_fwd, r_fwd):
        """Compatibility shim: sends unified speed to Pico (both wheels identical).

        With Ackermann steering, left/right motors run at the same speed.
        Direction is unified: if either side is reverse, we send reverse.
        Steering is handled separately by the servo.
        """
        speed = max(speed_l, speed_r)
        forward = l_fwd and r_fwd
        self._current_speed = speed
        self._is_forward = forward
        self._send_drive(speed, self._steering_pw, forward=forward)

    def _set_target_rpms(self, target_rpm_l, target_rpm_r, l_fwd, r_fwd,
                         speed_l_hint=0.0, speed_r_hint=0.0):
        """Compatibility shim: falls back to open-loop speed control."""
        self._set_raw_motors(speed_l_hint, speed_r_hint, l_fwd, r_fwd)

    # ── steering ────────────────────────────────

    def _apply_steering(self, speed, angle, forward=True, speed_l=None, speed_r=None):
        """Ackermann steering: set servo angle + drive motors.

        If speed_l/speed_r are provided, uses independent per-wheel PWM
        (from dual PID). Otherwise, sends unified speed to both wheels.
        """
        from pico_sensor_reader import send_lr_pwm
        self._steering_pw = self._angle_to_pw(angle)
        duty_cap = self.power_limiter.max_safe_duty
        if speed_l is not None and speed_r is not None:
            pwm_l = max(0, min(duty_cap, speed_l))
            pwm_r = max(0, min(duty_cap, speed_r))
            self._current_speed = (pwm_l + pwm_r) / 2.0
        else:
            speed = max(0, min(duty_cap, speed))
            pwm_l = pwm_r = speed
            self._current_speed = speed
        self._is_forward = forward
        send_lr_pwm(int(pwm_l), int(pwm_r), self._steering_pw, forward=forward)

    # ── public API ──────────────────────────────

    def update_power_state(self, battery_voltage: float, current_amps: float):
        """Feed live sensor data to the power limiter."""
        self.power_limiter.update(battery_voltage, current_amps)

    def set_speed(self, speed):
        """Drive forward at *speed* (0-100) using the current steering angle."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        self._current_speed = speed
        self._is_forward = True
        self._send_drive(speed, self._steering_pw, forward=True)

    def set_steering(self, angle):
        """Set steering angle (-50 to +50). Negative = left, positive = right.
        Immediately re-applies the current speed with the new angle."""
        angle = max(-50, min(50, angle))
        self._steering_angle = angle
        self._steering_pw = self._angle_to_pw(angle)
        if self._current_speed > 0:
            self._send_drive(self._current_speed, self._steering_pw,
                             forward=self._is_forward)

    def reverse(self, speed):
        """Drive both motors in reverse at *speed* (0-100). Steering centred."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        self._current_speed = speed
        self._is_forward = False
        self._steering_pw = self.STEER_CENTER_PW
        self._steering_angle = 0
        self._send_drive(speed, self._steering_pw, forward=False)

    def reverse_steer(self, speed, angle):
        """Reverse with Ackermann steering for angled escape maneuvers."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        angle = max(-50, min(50, angle))
        self._current_speed = speed
        self._steering_angle = angle
        self._steering_pw = self._angle_to_pw(angle)
        self._is_forward = False
        self._send_drive(speed, self._steering_pw, forward=False)

    def pivot_turn(self, direction, speed=50):
        """Ackermann full-lock turn: max steering angle + forward creep.

        True zero-radius tank turns are not possible with Ackermann geometry.
        Instead, we steer to full lock and drive slowly forward.
        *direction*: "left" or "right".
        """
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        if direction == "left":
            self._steering_pw = self.STEER_LEFT_PW
            self._steering_angle = -50
        else:
            self._steering_pw = self.STEER_RIGHT_PW
            self._steering_angle = 50
        self._current_speed = speed
        self._is_forward = True
        self._send_drive(speed, self._steering_pw, forward=True)

    def brake(self):
        """Magnetic brake via Pico (H-bridge short-circuit braking)."""
        from pico_sensor_reader import send_brake
        send_brake()
        self._current_speed = 0

    def stop(self):
        """Gentle stop: coast via Pico (all motor pins LOW)."""
        from pico_sensor_reader import send_stop
        send_stop()
        self._current_speed = 0
        self._steering_angle = 0
        self._steering_pw = self.STEER_CENTER_PW

    def cleanup(self):
        """Center steering and stop motors."""
        self.stop()
