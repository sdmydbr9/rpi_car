#!/usr/bin/env python3
"""Interactive, stationary-first smoke test for the manual Pico firmware."""

from __future__ import annotations

import argparse
import os
import sys
import time


CORE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "core")
sys.path.insert(0, CORE_DIR)

from pico_control import PicoController


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=os.environ.get("RC_PICO_PORT", "/dev/ttyS0"))
    parser.add_argument(
        "--battery-ratio",
        type=float,
        default=float(os.environ.get("RC_BATTERY_DIVIDER_RATIO", "5.0")),
    )
    parser.add_argument(
        "--allow-motor-test",
        action="store_true",
        help="Allow a one-second, 15%% PWM forward test.",
    )
    args = parser.parse_args()

    pico = PicoController(port=args.port)
    if not pico.connected:
        print(f"ERROR: Pico unavailable: {pico.last_error}")
        return 1

    try:
        print(f"Pico ping: {'OK' if pico.ping() else 'FAILED'}")
        print(f"Battery: {pico.read_battery_voltage(args.battery_ratio)} V")
        print("Centering steering with motors stopped...")
        pico.drive("N", 0, 0)
        time.sleep(0.5)

        if args.allow_motor_test:
            confirmation = input(
                "Lift the drive wheels clear. Type DRIVE to run the motor test: "
            )
            if confirmation == "DRIVE":
                pico.drive("F", 15, 0)
                started = time.monotonic()
                while time.monotonic() - started < 1.0:
                    pico.drive("F", 15, 0)
                    time.sleep(0.1)
                print("Motor test complete.")
        return 0
    finally:
        pico.stop()
        pico.close()


if __name__ == "__main__":
    raise SystemExit(main())
