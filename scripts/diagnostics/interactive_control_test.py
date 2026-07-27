#!/usr/bin/env python3
"""Interactive, safety-gated manual test console for the running car server.

Run this on the Raspberry Pi while ``scripts/main.py`` is running.  It uses
Socket.IO rather than opening the UART directly, so the normal ownership,
arm/disarm, watchdog, and gamepad-priority rules still apply.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Any

import socketio


DEFAULT_THROTTLE = 15
MAX_TEST_THROTTLE = 20
MAX_TEST_DURATION_S = 2.0


class ManualTestConsole:
    def __init__(self, url: str) -> None:
        self.client = socketio.Client(reconnection=False)
        self.url = url
        self.sequence = 0
        self.owned = False
        self.status: dict[str, Any] = {}
        self.last_error = ""
        self.last_battery: dict[str, Any] = {}
        self._claim_event = threading.Event()
        self._battery_event = threading.Event()

        self.client.on("control_claim_result", self._on_claim)
        self.client.on("control_status", self._on_status)
        self.client.on("control_error", self._on_error)
        self.client.on("battery_result", self._on_battery)

    def _on_claim(self, data: dict[str, Any] | None) -> None:
        self.owned = bool((data or {}).get("owned"))
        self._claim_event.set()

    def _on_status(self, data: dict[str, Any] | None) -> None:
        self.status = dict(data or {})

    def _on_error(self, data: dict[str, Any] | None) -> None:
        self.last_error = str((data or {}).get("message", "request rejected"))

    def _on_battery(self, data: dict[str, Any] | None) -> None:
        self.last_battery = dict(data or {})
        self._battery_event.set()

    def connect(self) -> None:
        self.client.connect(self.url, transports=["polling"], wait_timeout=5)
        time.sleep(0.15)

    def show_status(self) -> None:
        status = self.status
        print(
            "Pico: {pico} | Armed: {armed} | E-stop: {estop} | "
            "Gamepad: {gamepad} | Owner: {owner}".format(
                pico="ready" if status.get("pico_connected") else "unavailable",
                armed=status.get("armed", False),
                estop=status.get("estop", False),
                gamepad=status.get("gamepad_connected", False),
                owner=status.get("remote_owner", False),
            )
        )
        if status.get("pico_error"):
            print("Pico error:", status["pico_error"])

    def claim_and_arm(self) -> None:
        if self.status.get("gamepad_connected"):
            raise RuntimeError("disconnect the Pi gamepad before testing")
        self.last_error = ""
        self._claim_event.clear()
        self.client.emit("control_claim")
        if not self._claim_event.wait(2) or not self.owned:
            raise RuntimeError(self.last_error or "control ownership was not granted")

        self.last_error = ""
        self.client.emit("arm")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if self.last_error:
                raise RuntimeError(self.last_error)
            if self.status.get("armed"):
                return
            time.sleep(0.05)
        raise RuntimeError("arm request timed out")

    def send_drive(self, direction: str, throttle: int, steering: int) -> None:
        self.sequence += 1
        self.client.emit(
            "drive_input",
            {
                "seq": self.sequence,
                "direction": direction,
                "throttle": throttle,
                "steering": steering,
                "brake": False,
            },
        )

    def safe_stop(self) -> None:
        if not self.client.connected:
            return
        try:
            if self.owned:
                self.send_drive("N", 0, 0)
                self.client.emit("disarm")
                time.sleep(0.15)
                self.client.emit("control_release")
                self.owned = False
        except Exception:
            # The server-side timeout and Pico watchdog remain fallbacks.
            pass

    def center_steering(self) -> None:
        self.claim_and_arm()
        try:
            self.send_drive("N", 0, 0)
            time.sleep(1.0)
            print("Center command sent.")
        finally:
            self.safe_stop()

    def sweep_steering(self) -> None:
        self.claim_and_arm()
        try:
            for label, steering in (
                ("left", -50),
                ("center", 0),
                ("right", 50),
                ("center", 0),
            ):
                self.last_error = ""
                self.send_drive("N", 0, steering)
                print(f"Sent {label} steering.")
                time.sleep(1.0)
                if self.last_error:
                    raise RuntimeError(self.last_error)
        finally:
            self.safe_stop()

    def motor_pulse(self) -> None:
        print("WARNING: this test rotates the drive wheels.")
        if input("Type LIFTED after lifting the drive wheels clear: ").strip() != "LIFTED":
            print("Motor test cancelled.")
            return

        direction = input("Direction [F/R] (default F): ").strip().upper() or "F"
        if direction not in {"F", "R"}:
            print("Direction must be F or R.")
            return
        throttle = self._number_input(
            "Throttle 5-20% (default 15): ",
            DEFAULT_THROTTLE,
            5,
            MAX_TEST_THROTTLE,
        )
        duration = self._number_input(
            "Duration 0.2-2.0 seconds (default 1.0): ",
            1.0,
            0.2,
            MAX_TEST_DURATION_S,
        )
        steering = self._number_input(
            "Steering -50 to 50 degrees (default 0): ",
            0,
            -50,
            50,
        )

        self.claim_and_arm()
        try:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.last_error = ""
                self.send_drive(direction, int(throttle), int(steering))
                if self.last_error:
                    raise RuntimeError(self.last_error)
                time.sleep(0.1)
            print("Motor pulse complete.")
        finally:
            self.safe_stop()

    @staticmethod
    def _number_input(
        prompt: str,
        default: float,
        minimum: float,
        maximum: float,
    ) -> float:
        raw = input(prompt).strip()
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print(f"Invalid value; using {default}.")
            return default
        return max(minimum, min(maximum, value))

    def read_battery(self) -> None:
        self.safe_stop()
        self._battery_event.clear()
        self.last_battery = {}
        self.client.emit("battery_read")
        if not self._battery_event.wait(3):
            print("Battery request timed out.")
            return
        if self.last_battery.get("status") == "ok":
            print(f"Battery: {self.last_battery['voltage']:.2f} V")
        else:
            print("Battery read failed:", self.last_battery.get("message", "unknown error"))

    def run(self) -> None:
        print("Manual car test console. Motors start only from option M.")
        while True:
            print("\n[S]tatus  [C]enter  s[W]eep steering  [B]attery  [M]otor pulse  [Q]uit")
            choice = input("> ").strip().lower()
            try:
                if choice == "s":
                    self.show_status()
                elif choice == "c":
                    self.center_steering()
                elif choice == "w":
                    self.sweep_steering()
                elif choice == "b":
                    self.read_battery()
                elif choice == "m":
                    self.motor_pulse()
                elif choice in {"q", "quit", "exit"}:
                    return
                else:
                    print("Choose S, C, W, B, M, or Q.")
            except RuntimeError as exc:
                print("Test rejected:", exc)
                self.safe_stop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    console = ManualTestConsole(f"http://{args.host}:{args.port}")
    try:
        console.connect()
        console.run()
        return 0
    except KeyboardInterrupt:
        print("\nStopping test console.")
        return 130
    except Exception as exc:
        print("Cannot start test console:", exc, file=sys.stderr)
        return 1
    finally:
        console.safe_stop()
        if console.client.connected:
            console.client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
