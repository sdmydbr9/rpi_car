#!/usr/bin/env python3
"""
Interactive steering calibration utility for the Pico-driven Ackermann chassis.

The Pico now keeps only a broad servo-safe clamp. The actual left/center/right
steering calibration is owned by the Raspberry Pi and stored in
`.steering_config.json`, so these values can be retuned without reflashing.
"""

import os
import sys
import termios
import tty


_DIAG_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_DIAG_DIR, "..", "core"))

from motor import (  # noqa: E402
    CarSystem,
    STEERING_CONFIG_FILE,
    default_steering_calibration,
    load_steering_calibration,
    save_steering_calibration,
    sanitize_steering_calibration,
)
from pico_sensor_reader import init_pico_reader  # noqa: E402


DEFAULT_STEP_US = 10
MIN_STEP_US = 1
MAX_STEP_US = 100


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def getch():
    """Read a single character without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def calibration_state_text(calibration):
    try:
        sanitize_steering_calibration(calibration)
        return "savable"
    except Exception as exc:
        return f"not savable yet ({exc})"


def approx_angle_text(pulse_width, calibration):
    center = float(calibration.get("center_pw", 1440))
    left = float(calibration.get("left_pw", 940))
    right = float(calibration.get("right_pw", 2150))

    if pulse_width < center:
        span = max(1.0, center - left)
        angle = -50.0 * (center - pulse_width) / span
    else:
        span = max(1.0, right - center)
        angle = 50.0 * (pulse_width - center) / span

    return f"{max(-50.0, min(50.0, angle)):+.1f} deg"


def print_help():
    print("\nSteering calibration controls:")
    print("  a / d  -> move steering left / right by current step")
    print("  [ / ]  -> decrease / increase step size")
    print("  1 2 3  -> move to saved LEFT / CENTER / RIGHT")
    print("  l c r  -> record current pulse as LEFT / CENTER / RIGHT")
    print("  s      -> save current working calibration to disk")
    print("  x      -> reset working calibration to defaults")
    print("  p      -> print current status")
    print("  h      -> show this help")
    print("  q      -> quit and center steering using the saved calibration")


def print_status(current_pw, working, step_us, note=""):
    print("\n" + "=" * 72)
    print("STEERING CALIBRATION")
    print(f"Config file : {STEERING_CONFIG_FILE}")
    print(
        "Working set : "
        f"left={working['left_pw']}us  "
        f"center={working['center_pw']}us  "
        f"right={working['right_pw']}us"
    )
    print(f"Current PW  : {current_pw}us  ({approx_angle_text(current_pw, working)})")
    print(f"Step size   : {step_us}us")
    print(f"State       : {calibration_state_text(working)}")
    if note:
        print(f"Note        : {note}")


def main():
    try:
        reader = init_pico_reader("/dev/ttyS0")
    except Exception as exc:
        print(f"FATAL: could not open Pico UART bridge: {exc}")
        return 1
    if not getattr(reader, "_running", False):
        print("FATAL: Pico UART bridge is not running. Check /dev/ttyS0 and serial permissions.")
        return 1

    saved = load_steering_calibration()
    working = dict(saved)
    CarSystem.apply_steering_calibration(saved)
    car = CarSystem()
    current_pw = saved["center_pw"]
    step_us = DEFAULT_STEP_US

    car.send_steering_only(pulse_width=current_pw)

    print("\n" + "=" * 72)
    print("PI-SIDE STEERING CALIBRATION")
    print("=" * 72)
    print("Move the steering, record left/center/right, then save when happy.")
    print_help()
    print_status(current_pw, working, step_us, "Loaded saved calibration and centered steering.")

    try:
        while True:
            key = getch()
            if not key:
                continue

            if key == "q":
                break

            if key == "a":
                current_pw = clamp(
                    current_pw - step_us,
                    CarSystem.STEER_SERVO_MIN_PW,
                    CarSystem.STEER_SERVO_MAX_PW,
                )
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Moved left.")
                continue

            if key == "d":
                current_pw = clamp(
                    current_pw + step_us,
                    CarSystem.STEER_SERVO_MIN_PW,
                    CarSystem.STEER_SERVO_MAX_PW,
                )
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Moved right.")
                continue

            if key == "[":
                step_us = max(MIN_STEP_US, step_us - 1)
                print_status(current_pw, working, step_us, "Reduced step size.")
                continue

            if key == "]":
                step_us = min(MAX_STEP_US, step_us + 1)
                print_status(current_pw, working, step_us, "Increased step size.")
                continue

            if key == "1":
                current_pw = int(working["left_pw"])
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Moved to saved LEFT.")
                continue

            if key == "2":
                current_pw = int(working["center_pw"])
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Moved to saved CENTER.")
                continue

            if key == "3":
                current_pw = int(working["right_pw"])
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Moved to saved RIGHT.")
                continue

            if key == "l":
                working["left_pw"] = int(current_pw)
                print_status(current_pw, working, step_us, "Recorded current pulse as LEFT.")
                continue

            if key == "c":
                working["center_pw"] = int(current_pw)
                print_status(current_pw, working, step_us, "Recorded current pulse as CENTER.")
                continue

            if key == "r":
                working["right_pw"] = int(current_pw)
                print_status(current_pw, working, step_us, "Recorded current pulse as RIGHT.")
                continue

            if key == "s":
                try:
                    saved = save_steering_calibration(working)
                    working = dict(saved)
                    car.update_steering_calibration(saved, send_now=True)
                    current_pw = clamp(
                        current_pw,
                        working["left_pw"],
                        working["right_pw"],
                    )
                    car.send_steering_only(pulse_width=current_pw)
                    print_status(current_pw, working, step_us, "Saved calibration to disk.")
                except Exception as exc:
                    print_status(current_pw, working, step_us, f"Save failed: {exc}")
                continue

            if key == "x":
                working = default_steering_calibration()
                current_pw = working["center_pw"]
                car.send_steering_only(pulse_width=current_pw)
                print_status(current_pw, working, step_us, "Reset working values to defaults (not saved yet).")
                continue

            if key == "p":
                print_status(current_pw, working, step_us, "Status refresh.")
                continue

            if key == "h":
                print_help()
                print_status(current_pw, working, step_us)
                continue

            print_status(current_pw, working, step_us, f"Ignored key: {repr(key)}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        active = load_steering_calibration()
        CarSystem.apply_steering_calibration(active)
        car.send_steering_only(pulse_width=active["center_pw"])
        print(
            "\nCentered steering using saved calibration: "
            f"left={active['left_pw']}us center={active['center_pw']}us right={active['right_pw']}us"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
