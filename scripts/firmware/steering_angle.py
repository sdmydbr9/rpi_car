from machine import Pin, PWM
import sys
import time


# Steering servo setup from pico_sensor_bridge.py
STEER_PIN = 15
PWM_FREQ_HZ = 50

STEER_CENTER_PW = 1440

# Broad servo pulse bounds for calibration.
# This avoids the previously tested chassis-specific steering limits,
# while still preventing obviously invalid pulse widths.
SERVO_MIN_PW = 500
SERVO_MAX_PW = 2500

# Small movement on each key press.
STEP_US = 10

# Displayed steering angle relative to center.
DISPLAY_MIN_DEG = -50.0
DISPLAY_MAX_DEG = 50.0


steer_servo = PWM(Pin(STEER_PIN))
steer_servo.freq(PWM_FREQ_HZ)

current_pw = STEER_CENTER_PW
recorded_points = []


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def pulse_to_display_deg(pw_us):
    if pw_us == STEER_CENTER_PW:
        return 0.0

    if pw_us < STEER_CENTER_PW:
        span = STEER_CENTER_PW - SERVO_MIN_PW
        ratio = (pw_us - STEER_CENTER_PW) / span
        return round(abs(ratio) * DISPLAY_MIN_DEG, 1)

    span = SERVO_MAX_PW - STEER_CENTER_PW
    ratio = (pw_us - STEER_CENTER_PW) / span
    return round(ratio * DISPLAY_MAX_DEG, 1)


def apply_steering(pw_us):
    global current_pw
    current_pw = clamp(pw_us, SERVO_MIN_PW, SERVO_MAX_PW)
    steer_servo.duty_ns(current_pw * 1000)
    return current_pw


def print_status(prefix="Current"):
    print(
        "{}: {} us, {} deg".format(
            prefix,
            current_pw,
            pulse_to_display_deg(current_pw),
        )
    )


def print_help():
    print("\nSteering calibration controls:")
    print("  l  -> step left by {} us".format(STEP_US))
    print("  r  -> step right by {} us".format(STEP_US))
    print("  c  -> center steering")
    print("  s  -> save current position")
    print("  p  -> print saved positions")
    print("  h  -> show this help")
    print("  q  -> quit and center steering")
    print("\nIf your serial terminal does not send raw key presses,")
    print("type the letter and press Enter.")


def save_current_position():
    entry = {
        "index": len(recorded_points) + 1,
        "pulse_us": current_pw,
        "angle_deg": pulse_to_display_deg(current_pw),
        "offset_us": current_pw - STEER_CENTER_PW,
    }
    recorded_points.append(entry)
    print(
        "Saved #{}: {} us, {} deg, offset {:+d} us".format(
            entry["index"],
            entry["pulse_us"],
            entry["angle_deg"],
            entry["offset_us"],
        )
    )


def print_saved_positions():
    if not recorded_points:
        print("No positions saved yet.")
        return

    print("\nSaved steering checkpoints:")
    for entry in recorded_points:
        print(
            "  #{}: {} us, {} deg, offset {:+d} us".format(
                entry["index"],
                entry["pulse_us"],
                entry["angle_deg"],
                entry["offset_us"],
            )
        )


def read_command():
    if hasattr(sys.stdin, "read"):
        ch = sys.stdin.read(1)
        if ch:
            return ch.strip().lower()
    return ""


print("\n" + "=" * 60)
print("STEERING ANGLE CALIBRATION")
print("Servo pin: GP{} | Center: {} us | Pulse range: {} to {} us".format(
    STEER_PIN,
    STEER_CENTER_PW,
    SERVO_MIN_PW,
    SERVO_MAX_PW,
))
print("=" * 60)

apply_steering(STEER_CENTER_PW)
print_help()
print_status("Start")

try:
    while True:
        command = read_command()
        if not command:
            time.sleep_ms(20)
            continue

        if command == "l":
            previous_pw = current_pw
            apply_steering(current_pw - STEP_US)
            if current_pw == previous_pw:
                print("Already at minimum pulse width.")
            print_status("Left")

        elif command == "r":
            previous_pw = current_pw
            apply_steering(current_pw + STEP_US)
            if current_pw == previous_pw:
                print("Already at maximum pulse width.")
            print_status("Right")

        elif command == "c":
            apply_steering(STEER_CENTER_PW)
            print_status("Centered")

        elif command == "s":
            save_current_position()

        elif command == "p":
            print_saved_positions()

        elif command == "h":
            print_help()
            print_status()

        elif command == "q":
            print("Exiting. Centering steering first.")
            apply_steering(STEER_CENTER_PW)
            print_status("Final")
            break

        else:
            print("Unknown command: {}".format(command))
            print_help()

finally:
    apply_steering(STEER_CENTER_PW)
    time.sleep_ms(300)
    steer_servo.deinit()

    if recorded_points:
        print_saved_positions()
    print("Steering servo released.")