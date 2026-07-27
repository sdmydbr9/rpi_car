#!/usr/bin/env python3
"""Direct, guarded servo and motor test for ``basic_rpi_car``.

This script bypasses the gamepad controller. It drives steering from Raspberry
Pi GPIO12 and tests the motors through the Pico UART.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time
from typing import Callable

from main import PiSteeringController, PicoController, load_calibration


COMMAND_REFRESH_S = 0.10
ESTOP_CLEAR_DELAY_S = 0.25
PAUSE_BETWEEN_MOTORS_S = 0.30


def require_wheels_lifted(
    input_fn: Callable[[str], str] = input,
) -> bool:
    answer = input_fn(
        "\nLift BOTH drive wheels clear of the ground and keep clear of the "
        "steering linkage.\nType LIFTED to begin the hardware test: "
    )
    return answer.strip() == "LIFTED"


def send_drive(
    pico: PicoController,
    direction: str,
    throttle: int,
    steering: int,
) -> None:
    if not pico.drive(direction, throttle, steering):
        raise RuntimeError(
            pico.last_error or "failed to write a Pico drive command"
        )


def safe_stop_and_center(
    pico: PicoController | None,
    steering: PiSteeringController | None,
) -> None:
    """Best-effort cleanup that leaves PWM at zero and steering centered."""
    if pico is not None and not pico.stop():
        print(
            f"WARNING: could not send stop: "
            f"{pico.last_error or 'UART unavailable'}",
            file=sys.stderr,
        )
    if pico is not None and not pico.drive("N", 0, 0):
        print(
            f"WARNING: could not send neutral motor command: "
            f"{pico.last_error or 'UART unavailable'}",
            file=sys.stderr,
        )
    if steering is not None and not steering.center():
        print(
            f"WARNING: could not center GPIO12 steering: "
            f"{steering.last_error or 'GPIO12 unavailable'}",
            file=sys.stderr,
        )


def clear_stale_estop(
    pico: PicoController,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Clear the latch left by a safe shutdown of the normal controller."""
    if not pico.stop():
        raise RuntimeError(pico.last_error or "failed to stop the Pico")
    sleep_fn(ESTOP_CLEAR_DELAY_S)
    if not pico.reset_emergency_stop():
        raise RuntimeError(
            pico.last_error or "failed to send the e-stop reset command"
        )
    sleep_fn(0.05)
    send_drive(pico, "N", 0, 0)


def hold_motor(
    pico: PicoController,
    direction: str,
    throttle: int,
    duration: float,
    monotonic_fn: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Refresh motor output frequently enough to satisfy the Pico watchdog."""
    deadline = monotonic_fn() + duration
    while True:
        send_drive(pico, direction, throttle, 0)
        remaining = deadline - monotonic_fn()
        if remaining <= 0:
            break
        sleep_fn(min(COMMAND_REFRESH_S, remaining))


def run_servo_test(
    steering: PiSteeringController,
    steering_angle: int,
    hold_seconds: float,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    positions = (
        ("CENTER", 0),
        ("LEFT", -steering_angle),
        ("CENTER", 0),
        ("RIGHT", steering_angle),
        ("CENTER", 0),
    )
    print("\nServo test: watch the front wheels.")
    for label, angle in positions:
        print(f"  Steering {label} ({angle:+d})")
        if not steering.set_steering(angle):
            raise RuntimeError(
                steering.last_error or "failed to set GPIO12 steering"
            )
        sleep_fn(hold_seconds)


def run_motor_test(
    pico: PicoController,
    throttle: int,
    run_seconds: float,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    print(f"\nMotor test: both motors at {throttle}% requested throttle.")
    print("  FORWARD")
    hold_motor(
        pico,
        "F",
        throttle,
        run_seconds,
        sleep_fn=sleep_fn,
    )
    if not pico.stop():
        raise RuntimeError(pico.last_error or "failed to stop after forward")
    sleep_fn(PAUSE_BETWEEN_MOTORS_S)

    print("  REVERSE")
    hold_motor(
        pico,
        "R",
        throttle,
        run_seconds,
        sleep_fn=sleep_fn,
    )
    if not pico.stop():
        raise RuntimeError(pico.last_error or "failed to stop after reverse")
    sleep_fn(PAUSE_BETWEEN_MOTORS_S)


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Directly test GPIO12 steering and Pico motors with the "
            "wheels lifted."
        )
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("RC_PICO_PORT", "/dev/ttyS0"),
        help="Pico UART device (default: RC_PICO_PORT or /dev/ttyS0)",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(
            os.environ.get(
                "RC_STEERING_CONFIG",
                Path(__file__).with_name(".steering_config.json"),
            )
        ),
        help="optional steering calibration JSON file",
    )
    parser.add_argument(
        "--throttle",
        type=int,
        default=15,
        help="motor test throttle from 1 to 30 percent (default: 15)",
    )
    parser.add_argument(
        "--motor-seconds",
        type=positive_float,
        default=1.0,
        help="duration of each motor direction test (default: 1.0)",
    )
    parser.add_argument(
        "--steering-angle",
        type=int,
        default=30,
        help="steering test magnitude from 1 to 50 (default: 30)",
    )
    parser.add_argument(
        "--steering-seconds",
        type=positive_float,
        default=0.75,
        help="hold time for each steering position (default: 0.75)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--servo-only",
        action="store_true",
        help="run only the steering servo sequence",
    )
    mode.add_argument(
        "--motor-only",
        action="store_true",
        help="run only the forward/reverse motor sequence",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 1 <= args.throttle <= 30:
        print("ERROR: --throttle must be between 1 and 30.", file=sys.stderr)
        return 2
    if not 1 <= args.steering_angle <= 50:
        print(
            "ERROR: --steering-angle must be between 1 and 50.",
            file=sys.stderr,
        )
        return 2

    calibration = load_calibration(args.calibration)
    pico: PicoController | None = None
    steering: PiSteeringController | None = None

    try:
        if not require_wheels_lifted():
            print("Confirmation not received; no movement test was run.")
            return 2

        if not args.motor_only:
            steering = PiSteeringController(calibration=calibration)
            if not steering.connect():
                print(
                    "ERROR: cannot connect to pigpiod for GPIO12 steering: "
                    f"{steering.last_error}",
                    file=sys.stderr,
                )
                return 1
            print("GPIO12 steering: connected")
            run_servo_test(
                steering,
                args.steering_angle,
                args.steering_seconds,
            )

        if not args.servo_only:
            pico = PicoController(port=args.port, calibration=calibration)
            if not pico.connect():
                print(
                    f"ERROR: cannot open Pico UART {args.port}: "
                    f"{pico.last_error}",
                    file=sys.stderr,
                )
                return 1
            print(f"Opened Pico UART: {args.port} at 115200 baud")
            print("Checking for the basic_rpi_car Pico firmware...", flush=True)
            if not pico.ping():
                print(
                    "ERROR: Pico did not answer PING. Check pico firmware, "
                    "TX/RX crossover, common ground, and the UART device. "
                    f"({pico.last_error})",
                    file=sys.stderr,
                )
                return 1
            print("Pico PING: OK")
            print("Clearing the emergency-stop latch left by the controller...")
            clear_stale_estop(pico)
            run_motor_test(
                pico,
                args.throttle,
                args.motor_seconds,
            )

        print("\nHardware test complete; motors stopped and steering centered.")
        return 0
    except KeyboardInterrupt:
        print("\nHardware test interrupted.", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        safe_stop_and_center(pico, steering)
        if pico is not None:
            pico.close()
        if steering is not None:
            steering.close()


if __name__ == "__main__":
    raise SystemExit(main())
