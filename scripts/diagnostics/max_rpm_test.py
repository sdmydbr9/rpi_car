#!/usr/bin/env python3
"""
Max RPM test using the same Pi->Pico motor command style as motor_test.py.

This script:
  - runs on the Raspberry Pi
  - talks to the Pico over UART via pico_sensor_reader
  - keeps re-sending motor commands so the Pico watchdog does not stop the motors
  - drives both rear motors at a chosen PWM with steering centered
  - reports left/right peak RPM and average settled RPM

Use this only with the wheels removed or the chassis safely lifted.
Press Ctrl+C to stop at any time.
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from statistics import mean


_diag_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_diag_dir, "..", "core"))

from pico_sensor_reader import (  # noqa: E402
    get_pico_motor_duty,
    get_pico_rpm,
    get_pico_rpm_raw,
    get_sensor_packet,
    init_pico_reader,
    is_pico_fresh,
    send_brake,
    send_encoder_reset,
    send_motor_command,
    send_reverse_command,
    send_stop,
)


STEER_CENTER_PW = 1440
DEFAULT_PORT = "/dev/ttyS0"
FALLBACK_PORTS = ("/dev/ttyAMA0", "/dev/serial0")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run both motors using motor_test.py style commands and measure max RPM."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"UART device for the Pico bridge. Default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=100,
        help="PWM percentage to command, 0-100. Default: 100",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="How long to run the motors, in seconds. Default: 5.0",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="Ignore samples for this long before computing averages. Default: 1.0",
    )
    parser.add_argument(
        "--sample-time",
        type=float,
        default=0.05,
        help="Seconds between command refreshes and samples. Default: 0.05",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Run the motors in reverse instead of forward.",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="Optional CSV path. Default saves into rover_logs/ with a timestamp.",
    )
    return parser.parse_args()


def resolve_port(requested_port: str) -> str:
    if os.path.exists(requested_port):
        return requested_port

    if requested_port == DEFAULT_PORT:
        for port in FALLBACK_PORTS:
            if os.path.exists(port):
                return port

    raise FileNotFoundError(
        f"UART device not found: {requested_port}. Also checked: {', '.join(FALLBACK_PORTS)}"
    )


def wait_for_fresh_data(timeout_s: float = 8.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if is_pico_fresh() and get_sensor_packet() is not None:
            return
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for fresh Pico telemetry from the Pico.")


def safe_stop() -> None:
    send_stop()
    time.sleep(0.05)
    send_brake()
    time.sleep(0.2)
    send_stop()


def wait_for_spin_down(timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rpm_l, rpm_r = get_pico_rpm()
        if abs(rpm_l) < 5.0 and abs(rpm_r) < 5.0:
            return
        time.sleep(0.05)


def default_csv_path() -> str:
    log_dir = os.path.join(_diag_dir, "..", "..", "rover_logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"max_rpm_test_{timestamp}.csv")


def clamp_speed(speed: int) -> int:
    return max(0, min(100, int(speed)))


def refresh_drive_command(speed: int, reverse: bool) -> None:
    if reverse:
        send_reverse_command(speed, STEER_CENTER_PW)
    else:
        send_motor_command(speed, STEER_CENTER_PW)


def print_summary(
    duration_s: float,
    left_filtered_samples: list[float],
    right_filtered_samples: list[float],
    left_raw_samples: list[float],
    right_raw_samples: list[float],
    left_duty_samples: list[float],
    right_duty_samples: list[float],
) -> None:
    print("\n" + "=" * 84)
    print("Max RPM Summary")
    print("=" * 84)
    print(f"Run duration: {duration_s:.1f}s")
    print()
    print(
        f"{'Motor':<8s} {'Peak filt RPM':>14s} {'Peak raw RPM':>14s} "
        f"{'Avg filt RPM':>14s} {'Avg PWM':>10s}"
    )
    print("-" * 84)
    print(
        f"{'Left':<8s} "
        f"{max(left_filtered_samples) if left_filtered_samples else 0.0:14.1f} "
        f"{max(left_raw_samples) if left_raw_samples else 0.0:14.1f} "
        f"{mean(left_filtered_samples) if left_filtered_samples else 0.0:14.1f} "
        f"{mean(left_duty_samples) if left_duty_samples else 0.0:10.1f}"
    )
    print(
        f"{'Right':<8s} "
        f"{max(right_filtered_samples) if right_filtered_samples else 0.0:14.1f} "
        f"{max(right_raw_samples) if right_raw_samples else 0.0:14.1f} "
        f"{mean(right_filtered_samples) if right_filtered_samples else 0.0:14.1f} "
        f"{mean(right_duty_samples) if right_duty_samples else 0.0:10.1f}"
    )


def main() -> int:
    args = parse_args()
    speed = clamp_speed(args.speed)

    if args.duration <= 0 or args.sample_time <= 0:
        print("Duration and sample time must be positive.")
        return 2
    if args.warmup >= args.duration:
        print("Warmup must be shorter than duration.")
        return 2

    port = resolve_port(args.port)
    csv_path = args.csv or default_csv_path()
    direction = "reverse" if args.reverse else "forward"

    print("\n" + "=" * 72)
    print("Pico Motor Max RPM Test")
    print("=" * 72)
    print(f"UART port   : {port}")
    print(f"Direction   : {direction}")
    print(f"PWM command : {speed}%")
    print(f"Duration    : {args.duration:.1f}s")
    print(f"Warmup      : {args.warmup:.1f}s")
    print(f"Sample time : {args.sample_time:.3f}s")
    print(f"CSV log     : {csv_path}")
    print("\nThis uses the same command method as motor_test.py and refreshes commands continuously.")
    print("Keep the motors unloaded and clear before starting.")

    running = True

    def stop_handler(sig, frame):
        nonlocal running
        running = False
        print("\nStop requested. Stopping motors...")

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    left_filtered_samples: list[float] = []
    right_filtered_samples: list[float] = []
    left_raw_samples: list[float] = []
    right_raw_samples: list[float] = []
    left_duty_samples: list[float] = []
    right_duty_samples: list[float] = []

    try:
        init_pico_reader(port)
        wait_for_fresh_data()

        safe_stop()
        wait_for_spin_down()
        send_encoder_reset()
        time.sleep(0.1)

        with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "elapsed_s",
                    "rpm_left_filtered",
                    "rpm_right_filtered",
                    "rpm_left_raw",
                    "rpm_right_raw",
                    "duty_left",
                    "duty_right",
                    "enc_left_steps",
                    "enc_right_steps",
                ]
            )

            print("\n" + "=" * 72)
            print(f"Running motors {direction} at {speed}%")
            print("=" * 72)
            print(
                f"{'Time':>6s} | {'L_RPM':>8s} {'R_RPM':>8s} | "
                f"{'L_RAW':>8s} {'R_RAW':>8s} | {'L_PWM':>6s} {'R_PWM':>6s}"
            )
            print("-" * 72)

            start = time.monotonic()
            next_print = start

            while running:
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= args.duration:
                    break

                # Critical: refresh commands continuously so the Pico watchdog
                # does not stop the motors after 500 ms.
                refresh_drive_command(speed, args.reverse)

                packet = get_sensor_packet()
                rpm_l, rpm_r = get_pico_rpm()
                raw_l, raw_r = get_pico_rpm_raw()
                duty_l, duty_r = get_pico_motor_duty()

                if packet is not None:
                    writer.writerow(
                        [
                            f"{elapsed:.3f}",
                            f"{rpm_l:.3f}",
                            f"{rpm_r:.3f}",
                            f"{raw_l:.3f}",
                            f"{raw_r:.3f}",
                            f"{duty_l:.3f}",
                            f"{duty_r:.3f}",
                            packet.enc_left_steps,
                            packet.enc_right_steps,
                        ]
                    )

                if elapsed >= args.warmup:
                    left_filtered_samples.append(abs(rpm_l))
                    right_filtered_samples.append(abs(rpm_r))
                    left_raw_samples.append(abs(raw_l))
                    right_raw_samples.append(abs(raw_r))
                    left_duty_samples.append(abs(duty_l))
                    right_duty_samples.append(abs(duty_r))

                if now >= next_print:
                    print(
                        f"{elapsed:6.2f} | {rpm_l:8.1f} {rpm_r:8.1f} | "
                        f"{raw_l:8.1f} {raw_r:8.1f} | {duty_l:6.1f} {duty_r:6.1f}"
                    )
                    next_print = now + 0.25

                time.sleep(args.sample_time)

        print_summary(
            duration_s=args.duration,
            left_filtered_samples=left_filtered_samples,
            right_filtered_samples=right_filtered_samples,
            left_raw_samples=left_raw_samples,
            right_raw_samples=right_raw_samples,
            left_duty_samples=left_duty_samples,
            right_duty_samples=right_duty_samples,
        )
        print(f"\nCSV log saved to: {csv_path}")

    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 1
    finally:
        safe_stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
