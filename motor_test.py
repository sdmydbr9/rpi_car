#!/usr/bin/env python3
"""
Keyboard motor tester for independent wheel checks.

Key map:
  W/S -> Front-Left  forward/reverse
  E/D -> Front-Right forward/reverse
  R/F -> Rear-Left   forward/reverse
  T/G -> Rear-Right  forward/reverse
  Space or X -> Stop all motors
  + / - -> Increase/decrease test speed
  Q -> Quit
"""

import sys
import termios
import tty

# Try real GPIO first. Fall back to a mock so the script can still run
# off-device for basic key-flow testing.
try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"

        class _MockPWM:
            def __init__(self, pin, freq):
                self.pin = pin
                self.freq = freq

            def start(self, value):
                pass

            def ChangeDutyCycle(self, value):
                pass

            def stop(self):
                pass

        def setmode(self, mode):
            pass

        def setwarnings(self, value):
            pass

        def setup(self, pins, mode):
            pass

        def output(self, pins, state):
            pass

        def cleanup(self):
            pass

        def PWM(self, pin, freq):
            return self._MockPWM(pin, freq)

    GPIO = MockGPIO()
    print("⚠️  RPi.GPIO not available - using mock GPIO")


# Dual L298N, 4 independent wheel channels (BCM numbering).
FL_IN1 = 17
FL_IN2 = 27
FL_ENA = 12

FR_IN3 = 23
FR_IN4 = 22
FR_ENB = 13

RL_IN1 = 10
RL_IN2 = 7
RL_ENA = 19

RR_IN3 = 9
RR_IN4 = 11
RR_ENB = 18

ALL_DIR_PINS = [FL_IN1, FL_IN2, FR_IN3, FR_IN4, RL_IN1, RL_IN2, RR_IN3, RR_IN4]
ALL_EN_PINS = [FL_ENA, FR_ENB, RL_ENA, RR_ENB]

PWM_FREQ = 1000  # Hz

# Voltage-based PWM safety cap: 6V motors on 3S (11.1V nominal) pack.
BATTERY_VOLTAGE = 11.1
MOTOR_MAX_VOLTAGE = 7.0
MAX_PWM_DUTY = round((MOTOR_MAX_VOLTAGE / BATTERY_VOLTAGE) * 100)


DEFAULT_SPEED = min(30, MAX_PWM_DUTY)
SPEED_STEP = 5
MIN_SPEED = 5


# wheel_id -> (pwm_pin, in_a_pin, in_b_pin, human_name)
WHEEL_MAP = {
    "fl": (FL_ENA, FL_IN1, FL_IN2, "Front-Left"),
    "fr": (FR_ENB, FR_IN3, FR_IN4, "Front-Right"),
    "rl": (RL_ENA, RL_IN1, RL_IN2, "Rear-Left"),
    "rr": (RR_ENB, RR_IN3, RR_IN4, "Rear-Right"),
}


# key -> (wheel_id, direction)
COMMAND_MAP = {
    "w": ("fl", "forward"),
    "s": ("fl", "reverse"),
    "e": ("fr", "forward"),
    "d": ("fr", "reverse"),
    "r": ("rl", "forward"),
    "f": ("rl", "reverse"),
    "t": ("rr", "forward"),
    "g": ("rr", "reverse"),
}


def getch():
    """Read one keypress without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def setup_gpio():
    """Initialize motor pins and return PWM objects keyed by wheel id."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ALL_DIR_PINS + ALL_EN_PINS, GPIO.OUT)
    GPIO.output(ALL_DIR_PINS, False)
    GPIO.output(ALL_EN_PINS, False)

    pwms = {}
    for wheel_id, (pwm_pin, _, _, _) in WHEEL_MAP.items():
        pwm = GPIO.PWM(pwm_pin, PWM_FREQ)
        pwm.start(0)
        pwms[wheel_id] = pwm
    return pwms


def stop_all(pwms):
    """Set PWM to zero and coast all motors."""
    for pwm in pwms.values():
        pwm.ChangeDutyCycle(0)
    GPIO.output(ALL_DIR_PINS, False)


def drive_one_wheel(pwms, wheel_id, direction, speed):
    """Stop all wheels, then drive one selected wheel."""
    stop_all(pwms)

    _, in_a, in_b, wheel_name = WHEEL_MAP[wheel_id]
    pwm = pwms[wheel_id]

    # Ensure both direction pins start low before selecting direction.
    GPIO.output([in_a, in_b], 0)

    if direction == "forward":
        GPIO.output(in_a, 1)
        GPIO.output(in_b, 0)
    else:
        GPIO.output(in_a, 0)
        GPIO.output(in_b, 1)

    pwm.ChangeDutyCycle(speed)
    print(f"\r{wheel_name:<11} {direction:<7} @ {int(speed):>2}%   ", end="", flush=True)


def cleanup(pwms):
    """Safe shutdown for PWM and GPIO resources."""
    try:
        stop_all(pwms)
        for pwm in pwms.values():
            pwm.stop()
    finally:
        GPIO.cleanup()


def print_help(speed):
    print("Independent Motor Test")
    print("----------------------")
    print("W/S: Front-Left   fwd/rev")
    print("E/D: Front-Right  fwd/rev")
    print("R/F: Rear-Left    fwd/rev")
    print("T/G: Rear-Right   fwd/rev")
    print("Space or X: Stop all motors")
    print("+/-: Change speed")
    print("Q: Quit")
    print(f"\nStarting speed: {int(speed)}% (max allowed: {MAX_PWM_DUTY}%)")
    print("\nPress a key to test motors...")


def main():
    speed = float(DEFAULT_SPEED)
    pwms = setup_gpio()

    print_help(speed)

    try:
        while True:
            key = getch().lower()

            if key == "q":
                print("\nExiting motor test...")
                break

            if key in (" ", "x"):
                stop_all(pwms)
                print("\rAll motors stopped.                 ", end="", flush=True)
                continue

            if key in ("+", "="):
                speed = min(float(MAX_PWM_DUTY), speed + SPEED_STEP)
                print(f"\rSpeed set to {int(speed)}%          ", end="", flush=True)
                continue

            if key in ("-", "_"):
                speed = max(float(MIN_SPEED), speed - SPEED_STEP)
                print(f"\rSpeed set to {int(speed)}%          ", end="", flush=True)
                continue

            if key in COMMAND_MAP:
                wheel_id, direction = COMMAND_MAP[key]
                drive_one_wheel(pwms, wheel_id, direction, speed)

    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motors...")
    finally:
        cleanup(pwms)
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
