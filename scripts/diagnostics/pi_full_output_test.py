#!/usr/bin/env python3
"""Run an automatic full-output test through the installed Pico firmware.

The current UART protocol controls both motors together, so this test cannot
activate the left and right motors independently. Requested 100% throttle is
mapped by the Pi controller to the Pico firmware's allowed 95% PWM.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time


CORE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "core")
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
STEERING_CONFIG_PATH = os.path.join(PROJECT_ROOT, ".steering_config.json")
sys.path.insert(0, CORE_DIR)

from pico_control import PicoController


START_DELAY_SECONDS = 5
MOTOR_RUN_SECONDS = 1
STEERING_HOLD_SECONDS = 1
PAUSE_SECONDS = 1
COMMAND_REFRESH_SECONDS = 0.1


def hold_drive(
    pico: PicoController,
    direction: str,
    steering: int,
    duration: float,
) -> None:
    """Refresh a full-throttle command so the Pico watchdog stays satisfied."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if not pico.drive(direction, 100, steering):
            raise RuntimeError(pico.last_error or "failed to send drive command")
        time.sleep(COMMAND_REFRESH_SECONDS)


def stop_and_pause(pico: PicoController) -> None:
    if not pico.stop():
        raise RuntimeError(pico.last_error or "failed to stop motors")
    time.sleep(PAUSE_SECONDS)


def apply_saved_steering_calibration(pico: PicoController) -> None:
    if not os.path.exists(STEERING_CONFIG_PATH):
        return
    try:
        with open(STEERING_CONFIG_PATH, encoding="utf-8") as handle:
            pico.set_steering_calibration(json.load(handle))
    except (OSError, TypeError, ValueError) as exc:
        print(
            f"WARNING: could not apply saved steering calibration: {exc}",
            file=sys.stderr,
        )


def run_test(pico: PicoController) -> None:
    pico.stop()
    pico.drive("N", 0, 0)

    print(
        "Lift both drive wheels. The automatic test starts in "
        f"{START_DELAY_SECONDS} seconds."
    )
    time.sleep(START_DELAY_SECONDS)

    print("1/4: both motors forward at the 95% firmware limit")
    hold_drive(pico, "F", 0, MOTOR_RUN_SECONDS)
    stop_and_pause(pico)

    print("2/4: both motors reverse at the 95% firmware limit")
    hold_drive(pico, "R", 0, MOTOR_RUN_SECONDS)
    stop_and_pause(pico)

    print("3/4: steering full left with motors stopped")
    if not pico.drive("N", 0, -50):
        raise RuntimeError(pico.last_error or "failed to steer left")
    time.sleep(STEERING_HOLD_SECONDS)

    print("4/4: steering full right with motors stopped")
    if not pico.drive("N", 0, 50):
        raise RuntimeError(pico.last_error or "failed to steer right")
    time.sleep(STEERING_HOLD_SECONDS)

    print("Test complete: steering centered and motors stopped")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Test both motors and steering through the Pico UART."
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("RC_PICO_PORT", "/dev/ttyS0"),
        help="Pico UART device (default: RC_PICO_PORT or /dev/ttyS0)",
    )
    args = parser.parse_args()

    pico = PicoController(port=args.port)
    if not pico.connected:
        print(f"ERROR: Pico unavailable: {pico.last_error}", file=sys.stderr)
        return 1

    try:
        apply_saved_steering_calibration(pico)
        run_test(pico)
        return 0
    except KeyboardInterrupt:
        print("\nTest interrupted.")
        return 130
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        pico.stop()
        pico.drive("N", 0, 0)
        pico.close()


if __name__ == "__main__":
    raise SystemExit(main())
