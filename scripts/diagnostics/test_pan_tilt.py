#!/usr/bin/env python3
"""
Interactive pan/tilt servo control over the Pi <-> Pico UART bridge.

This diagnostic exercises the pan/tilt commands currently handled by
`scripts/firmware/pico_sensor_bridge.py`:

    PT:<pan>,<tilt>
    PC

By default it uses the current firmware limits:
    pan  = 0..180 degrees
    tilt = 45..135 degrees

Note: other higher-level scan/tracking code may intentionally use a narrower
application range such as 60..120 degrees. This script talks to the firmware
directly so the full firmware range can be tested.
"""

import argparse
import os
import select
import sys
import termios
import tty
from contextlib import contextmanager

import serial


UART_DEVICES = [
    "/dev/serial0",
    "/dev/ttyAMA0",
    "/dev/ttyS0",
    "/dev/ttyACM0",
    "/dev/ttyUSB0",
]

BAUDRATE = 115200
DEFAULT_STEP_SIZE = 5

PAN_MIN = 0
PAN_MAX = 180
TILT_MIN = 45
TILT_MAX = 135
PAN_CENTER = 90
TILT_CENTER = 90


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, int(value)))


def connect_uart(port):
    uart = serial.Serial(port, BAUDRATE, timeout=0, write_timeout=0.2)
    try:
        uart.reset_input_buffer()
        uart.reset_output_buffer()
    except Exception:
        pass
    return uart


def find_uart(preferred_port=None):
    """Open the requested port or the first available UART device."""
    candidates = [preferred_port] if preferred_port else UART_DEVICES

    for device in candidates:
        if not device:
            continue
        if not os.path.exists(device):
            continue
        try:
            uart = connect_uart(device)
            print(f"Connected to {device} @ {BAUDRATE} baud")
            return uart
        except Exception as exc:
            print(f"Could not open {device}: {exc}")

    if preferred_port:
        raise RuntimeError(f"No usable UART device at {preferred_port}")
    raise RuntimeError(f"No UART device found. Tried: {', '.join(UART_DEVICES)}")


def send_command(uart, command):
    uart.write(f"{command}\n".encode("ascii"))
    uart.flush()


def set_pan_tilt(uart, pan, tilt):
    """Send a PT command, returning the clamped angles."""
    pan = clamp(pan, PAN_MIN, PAN_MAX)
    tilt = clamp(tilt, TILT_MIN, TILT_MAX)
    send_command(uart, f"PT:{pan},{tilt}")
    return pan, tilt


def center_servos(uart):
    """Send a PC command and return the centered angles."""
    send_command(uart, "PC")
    return PAN_CENTER, TILT_CENTER


@contextmanager
def cbreak_mode(stream):
    """Temporarily put stdin into cbreak mode for single-key reads."""
    if not stream.isatty():
        raise RuntimeError("Interactive pan/tilt test must be run from a terminal.")

    fd = stream.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def read_key(timeout=0.05):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1).lower()


def print_status(action, pan, tilt):
    print(f"\r{action:<12} Pan={pan:3d} deg  Tilt={tilt:3d} deg", end="", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive pan/tilt servo UART test")
    parser.add_argument(
        "--port",
        default=None,
        help="UART device to use (default: auto-detect Pi hardware UART first)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=DEFAULT_STEP_SIZE,
        help=f"Angle increment per keypress (default: {DEFAULT_STEP_SIZE})",
    )
    return parser.parse_args()


def interactive_control():
    args = parse_args()
    step_size = max(1, abs(int(args.step)))
    uart = find_uart(args.port)

    pan = PAN_CENTER
    tilt = TILT_CENTER
    set_pan_tilt(uart, pan, tilt)

    print("\n" + "=" * 60)
    print("  INTERACTIVE PAN-TILT SERVO CONTROL")
    print("=" * 60)
    print(f"Starting position: Pan={pan} deg, Tilt={tilt} deg")
    print(f"Pan range:  {PAN_MIN}..{PAN_MAX} deg (step {step_size})")
    print(f"Tilt range: {TILT_MIN}..{TILT_MAX} deg (step {step_size})")
    print("Commands: W=pan left  S=pan right  E=tilt up  D=tilt down  C=center  Q=quit")
    print("Firmware protocol under test: PT:<pan>,<tilt> and PC")

    try:
        with cbreak_mode(sys.stdin):
            print_status("Centered", pan, tilt)
            while True:
                key = read_key()
                if key is None:
                    continue

                if key == "w":
                    pan, tilt = set_pan_tilt(uart, pan - step_size, tilt)
                    print_status("Pan left", pan, tilt)
                elif key == "s":
                    pan, tilt = set_pan_tilt(uart, pan + step_size, tilt)
                    print_status("Pan right", pan, tilt)
                elif key == "e":
                    pan, tilt = set_pan_tilt(uart, pan, tilt - step_size)
                    print_status("Tilt up", pan, tilt)
                elif key == "d":
                    pan, tilt = set_pan_tilt(uart, pan, tilt + step_size)
                    print_status("Tilt down", pan, tilt)
                elif key == "c":
                    pan, tilt = center_servos(uart)
                    print_status("Centered", pan, tilt)
                elif key == "q":
                    print("\nQuitting...")
                    break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        try:
            pan, tilt = center_servos(uart)
            print_status("Centered", pan, tilt)
            print()
        except Exception as exc:
            print(f"\nCould not center servos on exit: {exc}")
        uart.close()
        print("UART closed")


if __name__ == "__main__":
    interactive_control()
