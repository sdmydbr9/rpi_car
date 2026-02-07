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

    def setmode(self, mode): pass
    def setwarnings(self, val): pass
    def setup(self, pins, mode): pass

    def input(self, pin):
        return self.pin_states.get(pin, 1)

    def output(self, pins, state): pass
    def cleanup(self): pass

    class PWM:
        def __init__(self, pin, freq): pass
        def start(self, val): pass
        def stop(self): pass
        def ChangeDutyCycle(self, val): pass

    def PWM(self, pin, freq):
        return self.PWM(pin, freq)

    def set_pin(self, pin, value):
        self.pin_states[pin] = value


class GPIOWrapper:
    """Thin wrapper: routes to real GPIO when available, mock otherwise."""

    def __init__(self, real_gpio=None):
        self.real_gpio = real_gpio
        self.pin_states = {}
        if real_gpio:
            self.BCM = real_gpio.BCM
            self.IN = real_gpio.IN
            self.OUT = real_gpio.OUT
        else:
            self.BCM = "BCM"
            self.IN = "IN"
            self.OUT = "OUT"

    def setmode(self, mode):
        if self.real_gpio:
            self.real_gpio.setmode(mode)

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
        return MockGPIO.PWM(pin, freq)

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
    ENA = 18;  ENB = 19   # Left / Right PWM
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

    # ── raw GPIO helper ─────────────────────────

    def _set_raw_motors(self, speed_l, speed_r, l_fwd, r_fwd):
        """
        Write direction + PWM to both motors.
        l_fwd / r_fwd: True = forward, False = reverse.
        """
        # Left motor direction
        if l_fwd:
            GPIO.output(self.IN1, False)
            GPIO.output(self.IN2, True)
        else:
            GPIO.output(self.IN1, True)
            GPIO.output(self.IN2, False)
        # Right motor direction
        if r_fwd:
            GPIO.output(self.IN3, False)
            GPIO.output(self.IN4, True)
        else:
            GPIO.output(self.IN3, True)
            GPIO.output(self.IN4, False)

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
        GPIO.output(self.IN1, True);  GPIO.output(self.IN2, True)
        GPIO.output(self.IN3, True);  GPIO.output(self.IN4, True)
        self.pwm_a.ChangeDutyCycle(100)
        self.pwm_b.ChangeDutyCycle(100)
        self._current_speed = 0

    def stop(self):
        """Gentle stop: PWM → 0, direction pins LOW (coast)."""
        GPIO.output(self.IN1, False); GPIO.output(self.IN2, False)
        GPIO.output(self.IN3, False); GPIO.output(self.IN4, False)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self._current_speed = 0
        self._steering_angle = 0

    def cleanup(self):
        """Release PWM and GPIO resources."""
        self.stop()
        try:
            self.pwm_a.stop()
            self.pwm_b.stop()
        except Exception:
            pass
        GPIO.cleanup()
