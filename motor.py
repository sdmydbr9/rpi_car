"""motor.py — CarSystem: Low-level motor controller for 4WD (dual L298N).

Two L298N drivers control four independent motors:
  Driver 1 (front): Front-Left + Front-Right
  Driver 2 (rear):  Rear-Left  + Rear-Right

Provides a clean API for the AutoPilot and manual control:
  - set_speed(0-100)       Forward at given PWM %
  - set_steering(-90..+90) Differential arc turn while driving
  - pivot_turn("left"|"right", speed)  True tank turn: opposite wheels
  - reverse(speed)         Straight reverse at given PWM %
  - brake()                Magnetic lock (H-bridge short-circuit braking)
  - stop()                 Gentle stop (PWM → 0, pins LOW)
  - cleanup()              Release GPIO resources
"""

import time
import os
import threading


# ──────────────────────────────────────────────
# Voltage-based PWM hard limit  (legacy static fallback)
# ──────────────────────────────────────────────
# Motors are rated 6 V nominal; supply is 3 × 3.7 V = 11.1 V.
# These static values are kept for backward-compatibility with
# standalone scripts (slalom.py, autonomous_driving_laser.py, etc.)
# that import MAX_PWM_DUTY at module level.
# In the live system, CarSystem.power_limiter computes a *dynamic*
# duty-cycle cap from real-time battery voltage and current sensing.
BATTERY_VOLTAGE   = 11.1   # 3S Li-ion pack (3 × 3.7 V)  — fallback only
MOTOR_MAX_VOLTAGE = 7.0    # Absolute max voltage the motors should see
MAX_PWM_DUTY      = round(MOTOR_MAX_VOLTAGE / BATTERY_VOLTAGE * 100)  # ≈ 63 %


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
    MOTOR_MAX_SAFE_V = 8.0    # Absolute max effective motor voltage

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
    #   Gear   Min V   Max V   Description
    GEAR_VOLTAGE_MAP = {
        "N": (0.0, 0.0),       # No drive
        "1": (0.0, 3.5),       # Gentle / precision maneuvering
        "2": (3.5, 5.5),       # Moderate cruising
        "3": (5.5, 7.0),       # Fast driving
        "S": (5.5, 8.0),       # Sport — max safe power
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
# Motor safety constants
# ──────────────────────────────────────────────
MOTOR_SAFETY = {
    # Max PWM % change per call to _set_raw_motors (called at 50 Hz / 20 ms).
    # 5 % per tick → 0→100 in ~400 ms.  Prevents current spikes from
    # instant duty-cycle jumps that kill motors & H-bridge FETs.
    "MAX_PWM_DELTA_PER_TICK": 5,

    # Mandatory dwell (seconds) at PWM=0 before flipping H-bridge
    # direction pins.  Allows motor back-EMF to decay and avoids
    # shoot-through current in the L298N.
    "DIRECTION_CHANGE_DWELL_S": 0.15,   # 150 ms

    # Time (seconds) to ramp magnetic brake PWM from 0→100 %.
    # Softens the initial current spike of short-circuit braking.
    "BRAKE_RAMP_S": 0.10,               # 100 ms
}


# ──────────────────────────────────────────────
# GPIO abstraction (real RPi.GPIO or mock)
# ──────────────────────────────────────────────

class MockGPIO:
    """Fake GPIO for off-Pi testing."""
    BCM = "BCM"
    IN = "IN"
    OUT = "OUT"

    def __init__(self):
        self.pin_states = {}
        self._mode = None

    def setmode(self, mode): 
        self._mode = mode
        
    def getmode(self):
        return self._mode
        
    def setwarnings(self, val): pass
    def setup(self, pins, mode): pass

    def input(self, pin):
        return self.pin_states.get(pin, 1)

    def output(self, pins, state): pass
    def cleanup(self): pass

    class _MockPWM:
        def __init__(self, pin, freq): 
            self.pin = pin
            self.freq = freq
            
        def start(self, val): pass
        def stop(self): pass
        def ChangeDutyCycle(self, val): pass

    def PWM(self, pin, freq):
        return self._MockPWM(pin, freq)

    def set_pin(self, pin, value):
        self.pin_states[pin] = value


class GPIOWrapper:
    """Thin wrapper: routes to real GPIO when available, mock otherwise."""

    def __init__(self, real_gpio=None):
        self.real_gpio = real_gpio
        self.pin_states = {}
        self._mode = None
        if real_gpio:
            self.BCM = real_gpio.BCM
            self.IN = real_gpio.IN
            self.OUT = real_gpio.OUT
        else:
            self.BCM = "BCM"
            self.IN = "IN"
            self.OUT = "OUT"

    def setmode(self, mode):
        self._mode = mode
        if self.real_gpio:
            self.real_gpio.setmode(mode)
            
    def getmode(self):
        if self.real_gpio:
            return self.real_gpio.getmode()
        return self._mode

    def setwarnings(self, val):
        if self.real_gpio:
            self.real_gpio.setwarnings(val)

    def setup(self, pins, mode):
        if self.real_gpio:
            self.real_gpio.setup(pins, mode)

    def input(self, pin):
        if pin in self.pin_states:
            return self.pin_states[pin]
        if self.real_gpio:
            return self.real_gpio.input(pin)
        return 1

    def output(self, pins, state):
        if self.real_gpio:
            self.real_gpio.output(pins, state)

    def cleanup(self):
        if self.real_gpio:
            self.real_gpio.cleanup()

    def PWM(self, pin, freq):
        if self.real_gpio:
            return self.real_gpio.PWM(pin, freq)
        return MockGPIO._MockPWM(pin, freq)

    def set_pin(self, pin, value):
        self.pin_states[pin] = value


# ──────────────────────────────────────────────
# Detect hardware
# ──────────────────────────────────────────────

GPIO_AVAILABLE = False
real_gpio = None

if os.path.exists('/proc/device-tree/model'):
    try:
        import RPi.GPIO as real_gpio
    except Exception as e:
        print(f"⚠️  GPIO import failed: {e}")

GPIO = GPIOWrapper(real_gpio)

if real_gpio:
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO_AVAILABLE = True
    except Exception as e:
        print(f"⚠️  GPIO initialization failed: {e}")


# ──────────────────────────────────────────────
# CarSystem
# ──────────────────────────────────────────────

class CarSystem:
    """
    Low-level motor interface for a 4WD (dual L298N) differential-drive car.

    Two L298N drivers:
      Driver 1 → Front-Left & Front-Right motors
      Driver 2 → Rear-Left  & Rear-Right  motors

    Left-side  motors (FL + RL) always receive the same command.
    Right-side motors (FR + RR) always receive the same command.

    Direction convention (inverted H-bridge logic):
        Forward  → IN_A=LOW,  IN_B=HIGH
        Reverse  → IN_A=HIGH, IN_B=LOW
        Brake    → IN_A=HIGH, IN_B=HIGH  (+ 100 % PWM = magnetic lock)
        Coast    → IN_A=LOW,  IN_B=LOW   (+ 0 % PWM)
    """

    # ── Pin assignments (BCM) ─── Driver 1: FRONT ───
    FL_IN1 = 17;  FL_IN2 = 27;  FL_ENA = 12   # Front-Left
    FR_IN3 = 23;  FR_IN4 = 22;  FR_ENB = 13   # Front-Right (direction pins swapped)

    # ── Pin assignments (BCM) ─── Driver 2: REAR ────
    RL_IN1 = 10;  RL_IN2 = 7;   RL_ENA = 5    # Rear-Left
    RR_IN3 = 9;   RR_IN4 = 11;  RR_ENB = 6    # Rear-Right  (direction pins swapped)

    LEFT_IR = 5;  RIGHT_IR = 6  # IR obstacle sensors
    PWM_FREQ = 1000        # Hz

    # Convenience lists for bulk GPIO operations
    ALL_DIR_PINS = [FL_IN1, FL_IN2, FR_IN3, FR_IN4,
                    RL_IN1, RL_IN2, RR_IN3, RR_IN4]

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motor direction + PWM enable pins (all 12)
        all_motor_pins = self.ALL_DIR_PINS + [
            self.FL_ENA, self.FR_ENB, self.RL_ENA, self.RR_ENB
        ]
        try:
            GPIO.setup(all_motor_pins, GPIO.OUT)
        except Exception as e:
            print(f"⚠️  Motor pin setup failed: {e}")

        # IR sensor pins
        try:
            GPIO.setup([self.LEFT_IR, self.RIGHT_IR], GPIO.IN)
        except Exception as e:
            print(f"⚠️  IR sensor pin setup failed: {e}")

        # PWM channels — one per motor (4 total)
        try:
            self.pwm_fl = GPIO.PWM(self.FL_ENA, self.PWM_FREQ)
            self.pwm_fr = GPIO.PWM(self.FR_ENB, self.PWM_FREQ)
            self.pwm_rl = GPIO.PWM(self.RL_ENA, self.PWM_FREQ)
            self.pwm_rr = GPIO.PWM(self.RR_ENB, self.PWM_FREQ)
            self.pwm_fl.start(0)
            self.pwm_fr.start(0)
            self.pwm_rl.start(0)
            self.pwm_rr.start(0)
        except Exception as e:
            print(f"⚠️  PWM setup failed: {e}")

            class DummyPWM:
                def start(self, val): pass
                def stop(self): pass
                def ChangeDutyCycle(self, val): pass

            self.pwm_fl = DummyPWM()
            self.pwm_fr = DummyPWM()
            self.pwm_rl = DummyPWM()
            self.pwm_rr = DummyPWM()

        # Internal bookkeeping
        self._current_speed = 0
        self._steering_angle = 0
        self._last_l_fwd = None   # direction tracking for PWM-glitch prevention
        self._last_r_fwd = None

        # PWM ramping state — tracks *actually applied* duty cycle per
        # motor so each call can step toward the target without
        # exceeding MAX_PWM_DELTA_PER_TICK.
        self._applied_duty_fl = 0.0
        self._applied_duty_fr = 0.0
        self._applied_duty_rl = 0.0
        self._applied_duty_rr = 0.0

        # Adaptive voltage-aware power limiter
        self.power_limiter = PowerLimiter()

    # ── raw GPIO helper ─────────────────────────

    def _set_raw_motors(self, speed_l, speed_r, l_fwd, r_fwd):
        """
        Write direction + PWM to all four motors.
        l_fwd / r_fwd: True = forward, False = reverse.

        Left-side  (FL + RL) share speed_l / l_fwd.
        Right-side (FR + RR) share speed_r / r_fwd.

        Safety features:
          - PWM ramping: duty cycle changes are capped at
            MAX_PWM_DELTA_PER_TICK per call (called every 20 ms from
            the physics loop).  This prevents current spikes.
          - Brake dwell: on direction reversal the PWM is zeroed and
            we sleep DIRECTION_CHANGE_DWELL_S before flipping the
            H-bridge direction pins.  This lets back-EMF decay and
            avoids shoot-through current.

        Pin-setting strategy — simultaneous motor activation:
          1. On direction change, zero all 4 PWMs and dwell.
          2. Compute each pin's target value (8 direction pins).
          3. Set all LOW (deactivating) pins in one batch GPIO call.
          4. Set all HIGH (activating) pins in one batch GPIO call.
        """
        max_delta = MOTOR_SAFETY["MAX_PWM_DELTA_PER_TICK"]
        dwell = MOTOR_SAFETY["DIRECTION_CHANGE_DWELL_S"]

        dir_changed = (l_fwd != self._last_l_fwd or r_fwd != self._last_r_fwd)

        if dir_changed:
            # Cut power on all 4 motors while direction pins settle
            self.pwm_fl.ChangeDutyCycle(0)
            self.pwm_fr.ChangeDutyCycle(0)
            self.pwm_rl.ChangeDutyCycle(0)
            self.pwm_rr.ChangeDutyCycle(0)
            self._applied_duty_fl = 0.0
            self._applied_duty_fr = 0.0
            self._applied_duty_rl = 0.0
            self._applied_duty_rr = 0.0
            # Mandatory dwell — let back-EMF decay before flipping direction
            time.sleep(dwell)

        # Determine each pin's target value.
        # Front + Rear left share l_fwd; Front + Rear right share r_fwd.
        pin_vals = [
            # Front-Left
            (self.FL_IN1, l_fwd), (self.FL_IN2, not l_fwd),
            # Front-Right
            (self.FR_IN3, r_fwd), (self.FR_IN4, not r_fwd),
            # Rear-Left
            (self.RL_IN1, l_fwd), (self.RL_IN2, not l_fwd),
            # Rear-Right
            (self.RR_IN3, r_fwd), (self.RR_IN4, not r_fwd),
        ]
        pins_low  = [p for p, v in pin_vals if not v]
        pins_high = [p for p, v in pin_vals if v]

        # Deactivating (LOW) first, then activating (HIGH) — all motors
        # get their trigger pin in the same batch write.
        if pins_low:
            GPIO.output(pins_low, False)
        if pins_high:
            GPIO.output(pins_high, True)

        self._last_l_fwd = l_fwd
        self._last_r_fwd = r_fwd

        # ── PWM ramping ────────────────────────────────────────────
        # Clamp the requested duty cycle and step toward it at most
        # max_delta per tick.  This gives a soft-start / soft-stop
        # at the hardware layer regardless of what the caller asks.
        duty_cap = self.power_limiter.max_safe_duty
        target_l = float(max(0, min(duty_cap, speed_l)))
        target_r = float(max(0, min(duty_cap, speed_r)))

        # Front-Left ramp
        delta_fl = target_l - self._applied_duty_fl
        if abs(delta_fl) > max_delta:
            delta_fl = max_delta if delta_fl > 0 else -max_delta
        self._applied_duty_fl = max(0.0, min(duty_cap, self._applied_duty_fl + delta_fl))

        # Front-Right ramp
        delta_fr = target_r - self._applied_duty_fr
        if abs(delta_fr) > max_delta:
            delta_fr = max_delta if delta_fr > 0 else -max_delta
        self._applied_duty_fr = max(0.0, min(duty_cap, self._applied_duty_fr + delta_fr))

        # Rear-Left ramp
        delta_rl = target_l - self._applied_duty_rl
        if abs(delta_rl) > max_delta:
            delta_rl = max_delta if delta_rl > 0 else -max_delta
        self._applied_duty_rl = max(0.0, min(duty_cap, self._applied_duty_rl + delta_rl))

        # Rear-Right ramp
        delta_rr = target_r - self._applied_duty_rr
        if abs(delta_rr) > max_delta:
            delta_rr = max_delta if delta_rr > 0 else -max_delta
        self._applied_duty_rr = max(0.0, min(duty_cap, self._applied_duty_rr + delta_rr))

        self.pwm_fl.ChangeDutyCycle(int(self._applied_duty_fl))
        self.pwm_fr.ChangeDutyCycle(int(self._applied_duty_fr))
        self.pwm_rl.ChangeDutyCycle(int(self._applied_duty_rl))
        self.pwm_rr.ChangeDutyCycle(int(self._applied_duty_rr))

    # ── steering mixer (differential) ───────────

    def _apply_steering(self, speed, angle, forward=True):
        """
        Compute per-wheel speeds from a base speed + steering angle,
        then write to motors.
        """
        turn_factor = abs(angle) / 90.0
        inner = speed * (1.0 - turn_factor * 0.9)  # drops to 10 % at ±90°

        left_speed = speed
        right_speed = speed

        if angle < -5:     # turning LEFT  → slow left wheel
            left_speed = inner
        elif angle > 5:    # turning RIGHT → slow right wheel
            right_speed = inner

        self._set_raw_motors(left_speed, right_speed, forward, forward)

    # ── public API ──────────────────────────────

    def update_power_state(self, battery_voltage: float, current_amps: float):
        """Feed live sensor data to the power limiter.

        Should be called from the sensor-reading thread (~1 Hz).
        """
        self.power_limiter.update(battery_voltage, current_amps)

    def set_speed(self, speed):
        """Drive forward at *speed* (0-100) using the current steering angle."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        self._current_speed = speed
        self._apply_steering(speed, self._steering_angle, forward=True)

    def set_steering(self, angle):
        """
        Set steering angle (-90 to +90).  Negative = left, positive = right.
        Immediately re-applies the current speed with the new angle.
        """
        angle = max(-90, min(90, angle))
        self._steering_angle = angle
        if self._current_speed > 0:
            self._apply_steering(self._current_speed, angle, forward=True)

    def reverse(self, speed):
        """Drive both motors in reverse at *speed* (0-100). Steering centred."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        self._current_speed = speed
        self._set_raw_motors(speed, speed, False, False)

    def reverse_steer(self, speed, angle):
        """Reverse with differential steering for angled escape maneuvers.
        *angle*: -90 (bias left while reversing) to +90 (bias right)."""
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        angle = max(-90, min(90, angle))
        self._current_speed = speed
        self._apply_steering(speed, angle, forward=False)

    def pivot_turn(self, direction, speed=50):
        """
        True tank turn: one side forward, the other side reverse.
        *direction*: "left" or "right".
        *speed*: PWM % for both motors (default 50).
        """
        speed = max(0, min(self.power_limiter.max_safe_duty, speed))
        if direction == "left":
            # Left motor reverse, Right motor forward → car spins left
            self._set_raw_motors(speed, speed, False, True)
        else:
            # Left motor forward, Right motor reverse → car spins right
            self._set_raw_motors(speed, speed, True, False)

    def brake(self):
        """Magnetic lock: short-circuit all four H-bridges.

        Instead of slamming to 100 % PWM instantly (which causes a
        massive current spike), we ramp the braking PWM from the
        current applied duty cycle up to 100 % over BRAKE_RAMP_S.
        """
        ramp_time = MOTOR_SAFETY["BRAKE_RAMP_S"]
        steps = max(1, int(ramp_time / 0.01))  # 10 ms per step
        start_duty = max(self._applied_duty_fl, self._applied_duty_fr,
                         self._applied_duty_rl, self._applied_duty_rr)

        # Zero all 4 PWMs before changing to brake configuration
        self.pwm_fl.ChangeDutyCycle(0)
        self.pwm_fr.ChangeDutyCycle(0)
        self.pwm_rl.ChangeDutyCycle(0)
        self.pwm_rr.ChangeDutyCycle(0)

        # All 8 direction pins HIGH — all 4 motors brake simultaneously
        GPIO.output(self.ALL_DIR_PINS, True)

        # Ramp braking PWM from current level to the dynamic duty cap
        brake_cap = int(self.power_limiter.max_safe_duty)  # Never exceed voltage limit
        for i in range(1, steps + 1):
            duty = int(start_duty + (brake_cap - start_duty) * (i / steps))
            self.pwm_fl.ChangeDutyCycle(duty)
            self.pwm_fr.ChangeDutyCycle(duty)
            self.pwm_rl.ChangeDutyCycle(duty)
            self.pwm_rr.ChangeDutyCycle(duty)
            time.sleep(0.01)

        self.pwm_fl.ChangeDutyCycle(brake_cap)
        self.pwm_fr.ChangeDutyCycle(brake_cap)
        self.pwm_rl.ChangeDutyCycle(brake_cap)
        self.pwm_rr.ChangeDutyCycle(brake_cap)

        self._current_speed = 0
        self._applied_duty_fl = 0.0
        self._applied_duty_fr = 0.0
        self._applied_duty_rl = 0.0
        self._applied_duty_rr = 0.0
        self._last_l_fwd = None   # brake is neither fwd nor rev
        self._last_r_fwd = None

    def stop(self):
        """Gentle stop: PWM → 0, direction pins LOW (coast)."""
        self.pwm_fl.ChangeDutyCycle(0)
        self.pwm_fr.ChangeDutyCycle(0)
        self.pwm_rl.ChangeDutyCycle(0)
        self.pwm_rr.ChangeDutyCycle(0)
        # All 8 direction pins LOW — all 4 motors coast simultaneously
        GPIO.output(self.ALL_DIR_PINS, False)
        self._current_speed = 0
        self._steering_angle = 0
        self._applied_duty_fl = 0.0
        self._applied_duty_fr = 0.0
        self._applied_duty_rl = 0.0
        self._applied_duty_rr = 0.0
        self._last_l_fwd = None   # reset direction tracking
        self._last_r_fwd = None

    def cleanup(self):
        """Release PWM and GPIO resources."""
        self.stop()
        try:
            self.pwm_fl.stop()
            self.pwm_fr.stop()
            self.pwm_rl.stop()
            self.pwm_rr.stop()
        except Exception:
            pass
        GPIO.cleanup()
