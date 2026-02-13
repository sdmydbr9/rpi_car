"""
motor.py — CarSystem: Low-level motor controller for L298N dual H-bridge.

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
    Low-level motor interface for a 2-motor (L298N) differential-drive car.

    Direction convention (inverted H-bridge logic):
        Forward  → IN_A=LOW,  IN_B=HIGH
        Reverse  → IN_A=HIGH, IN_B=LOW
        Brake    → IN_A=HIGH, IN_B=HIGH  (+ 100 % PWM = magnetic lock)
        Coast    → IN_A=LOW,  IN_B=LOW   (+ 0 % PWM)
    """

    # Pin assignments (BCM)
    IN1 = 17;  IN2 = 27   # Left motor direction
    IN3 = 22;  IN4 = 23   # Right motor direction
    ENA = 12;  ENB = 13   # Left / Right PWM
    LEFT_IR = 5;  RIGHT_IR = 6  # IR obstacle sensors
    PWM_FREQ = 1000        # Hz

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motor direction pins
        try:
            GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4,
                        self.ENA, self.ENB], GPIO.OUT)
        except Exception as e:
            print(f"⚠️  Motor pin setup failed: {e}")

        # IR sensor pins
        try:
            GPIO.setup([self.LEFT_IR, self.RIGHT_IR], GPIO.IN)
        except Exception as e:
            print(f"⚠️  IR sensor pin setup failed: {e}")

        # PWM channels
        try:
            self.pwm_a = GPIO.PWM(self.ENA, self.PWM_FREQ)
            self.pwm_b = GPIO.PWM(self.ENB, self.PWM_FREQ)
            self.pwm_a.start(0)
            self.pwm_b.start(0)
        except Exception as e:
            print(f"⚠️  PWM setup failed: {e}")

            class DummyPWM:
                def start(self, val): pass
                def stop(self): pass
                def ChangeDutyCycle(self, val): pass

            self.pwm_a = DummyPWM()
            self.pwm_b = DummyPWM()

        # Internal bookkeeping
        self._current_speed = 0
        self._steering_angle = 0
        self._last_l_fwd = None   # direction tracking for PWM-glitch prevention
        self._last_r_fwd = None

    # ── raw GPIO helper ─────────────────────────

    def _set_raw_motors(self, speed_l, speed_r, l_fwd, r_fwd):
        """
        Write direction + PWM to both motors.
        l_fwd / r_fwd: True = forward, False = reverse.

        Pin-setting strategy — simultaneous motor activation:
          1. On direction change, zero both PWMs first.
          2. Compute each pin's target value.
          3. Set all LOW (deactivating) pins in one batch GPIO call.
          4. Set all HIGH (activating) pins in one batch GPIO call.
        Because the L298N drives the motor when it sees one HIGH + one
        LOW on a channel pair, the HIGH pin is the trigger.  By writing
        both motors' HIGH pins in a single GPIO.output() call they
        activate at the same instant — eliminating the timing skew that
        made one side start before the other.
        """
        dir_changed = (l_fwd != self._last_l_fwd or r_fwd != self._last_r_fwd)

        if dir_changed:
            # Cut power while the H-bridge direction pins settle
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)

        # Determine each pin's target: IN1/IN3 = NOT fwd, IN2/IN4 = fwd
        pin_vals = [
            (self.IN1, not l_fwd), (self.IN2, l_fwd),
            (self.IN3, not r_fwd), (self.IN4, r_fwd),
        ]
        pins_low  = [p for p, v in pin_vals if not v]
        pins_high = [p for p, v in pin_vals if v]

        # Deactivating (LOW) first, then activating (HIGH) — both motors
        # get their trigger pin in the same batch write.
        if pins_low:
            GPIO.output(pins_low, False)
        if pins_high:
            GPIO.output(pins_high, True)

        self._last_l_fwd = l_fwd
        self._last_r_fwd = r_fwd

        # Apply PWM — direction pins are already correct
        self.pwm_a.ChangeDutyCycle(int(max(0, min(100, speed_l))))
        self.pwm_b.ChangeDutyCycle(int(max(0, min(100, speed_r))))

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

    def set_speed(self, speed):
        """Drive forward at *speed* (0-100) using the current steering angle."""
        speed = max(0, min(100, speed))
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
        speed = max(0, min(100, speed))
        self._current_speed = speed
        self._set_raw_motors(speed, speed, False, False)

    def reverse_steer(self, speed, angle):
        """Reverse with differential steering for angled escape maneuvers.
        *angle*: -90 (bias left while reversing) to +90 (bias right)."""
        speed = max(0, min(100, speed))
        angle = max(-90, min(90, angle))
        self._current_speed = speed
        self._apply_steering(speed, angle, forward=False)

    def pivot_turn(self, direction, speed=50):
        """
        True tank turn: one side forward, the other side reverse.
        *direction*: "left" or "right".
        *speed*: PWM % for both motors (default 50).
        """
        speed = max(0, min(100, speed))
        if direction == "left":
            # Left motor reverse, Right motor forward → car spins left
            self._set_raw_motors(speed, speed, False, True)
        else:
            # Left motor forward, Right motor reverse → car spins right
            self._set_raw_motors(speed, speed, True, False)

    def brake(self):
        """Magnetic lock: short-circuit both H-bridges at 100 % PWM."""
        # Zero PWM before changing to brake configuration to avoid
        # transient drive pulses while direction pins switch.
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        # All 4 pins HIGH in one call — both motors brake simultaneously
        GPIO.output([self.IN1, self.IN2, self.IN3, self.IN4], True)
        self.pwm_a.ChangeDutyCycle(100)
        self.pwm_b.ChangeDutyCycle(100)
        self._current_speed = 0
        self._last_l_fwd = None   # brake is neither fwd nor rev
        self._last_r_fwd = None

    def stop(self):
        """Gentle stop: PWM → 0, direction pins LOW (coast)."""
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        # All 4 pins LOW in one call — both motors coast simultaneously
        GPIO.output([self.IN1, self.IN2, self.IN3, self.IN4], False)
        self._current_speed = 0
        self._steering_angle = 0
        self._last_l_fwd = None   # reset direction tracking
        self._last_r_fwd = None

    def cleanup(self):
        """Release PWM and GPIO resources."""
        self.stop()
        try:
            self.pwm_a.stop()
            self.pwm_b.stop()
        except Exception:
            pass
        GPIO.cleanup()
