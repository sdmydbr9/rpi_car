#!/usr/bin/env python3
"""
Standalone max-left / max-right steering servo test.

Uses the normal Pi -> Pico UART path, sending ML:0,0,<steer_pw>,F so the
rear motors stay stopped while the Pico updates the Ackermann steering servo.

Controls:
  L - snap steering to configured max left
  R - snap steering to configured max right
  C - snap steering to configured center
  Q - quit and center steering
"""

import json
import os
import sys
import termios
import time
import tty


DIAG_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(DIAG_DIR, "..", ".."))
CORE_DIR = os.path.join(PROJECT_ROOT, "scripts", "core")
STEERING_CONFIG_FILE = os.path.join(PROJECT_ROOT, ".steering_config.json")

sys.path.insert(0, CORE_DIR)

from pico_sensor_reader import init_pico_reader, send_lr_pwm  # noqa: E402
from steering_calibration import (  # noqa: E402
    default_steering_calibration,
    normalize_steering_calibration,
)


def load_steering_calibration():
    """Load persisted steering limits, falling back to shared defaults."""
    defaults = default_steering_calibration()
    try:
        with open(STEERING_CONFIG_FILE, "r", encoding="utf-8") as f:
            return normalize_steering_calibration(json.load(f))
    except FileNotFoundError:
        return defaults
    except Exception as exc:
        print(f"Warning: could not load {STEERING_CONFIG_FILE}: {exc}")
        print("Using default steering calibration.")
        return defaults


def getch():
    """Read one key without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def apply_steering(pulse_us, label):
    """Move steering servo while keeping both drive motors stopped."""
    send_lr_pwm(0, 0, pulse_us, forward=True)
    print(f"\rSteering: {label:<6}  pulse={pulse_us} us   [L/R/C/Q]", end="", flush=True)


def main():
    calibration = load_steering_calibration()
    left_pw = calibration["left_pw"]
    center_pw = calibration["center_pw"]
    right_pw = calibration["right_pw"]

    print("Standalone Steering Test")
    print(f"Using Pico UART on /dev/ttyS0")
    print(f"Limits: left={left_pw} us, center={center_pw} us, right={right_pw} us")
    print("Controls: L=max left, R=max right, C=center, Q=quit")

    init_pico_reader("/dev/ttyS0")
    time.sleep(0.2)
    apply_steering(center_pw, "CENTER")

    try:
        while True:
            key = getch().lower()
            if key == "l":
                apply_steering(left_pw, "LEFT")
            elif key == "r":
                apply_steering(right_pw, "RIGHT")
            elif key == "c":
                apply_steering(center_pw, "CENTER")
            elif key == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        apply_steering(center_pw, "CENTER")
        print("\nSteering centered. Done.")


if __name__ == "__main__":
    main()
