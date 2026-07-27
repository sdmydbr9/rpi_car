"""Minimal command-only UART transport for the Pico motor controller."""

from __future__ import annotations

import os
import threading
import time
from typing import Callable, Mapping

from steering_calibration import (
    default_steering_calibration,
    normalize_steering_calibration,
    steering_angle_to_pw,
)

try:
    import serial
except ImportError:  # Allows documentation/tests to import without hardware deps.
    serial = None


MAX_PWM_DUTY = 95
VALID_DIRECTIONS = {"F", "N", "R"}


def clamp_throttle(value) -> int:
    try:
        throttle = round(float(value))
    except (TypeError, ValueError):
        throttle = 0
    return max(0, min(100, throttle))


def clamp_steering(value) -> int:
    try:
        steering = round(float(value))
    except (TypeError, ValueError):
        steering = 0
    return max(-50, min(50, steering))


def normalize_direction(value) -> str:
    direction = str(value or "N").strip().upper()
    if direction not in VALID_DIRECTIONS:
        raise ValueError("direction must be F, N, or R")
    return direction


def throttle_to_pwm(throttle) -> int:
    return round(clamp_throttle(throttle) * MAX_PWM_DUTY / 100.0)


def format_drive_command(
    direction,
    throttle,
    steering,
    calibration: Mapping[str, object],
) -> bytes:
    direction_value = normalize_direction(direction)
    if direction_value == "N":
        throttle = 0
    pwm = throttle_to_pwm(throttle)
    pulse = steering_angle_to_pw(clamp_steering(steering), calibration)
    return f"D:{direction_value},{pwm},{pulse}\n".encode("ascii")


class PicoController:
    """Thread-safe, synchronous command transport.

    Drive commands are write-only. The Pico only transmits a response for
    explicit PING and BAT? requests, so no background serial reader is needed.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 115200,
        timeout: float = 0.2,
        serial_factory: Callable[..., object] | None = None,
    ):
        self.port = port or os.environ.get("RC_PICO_PORT", "/dev/ttyS0")
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial_factory = serial_factory
        self._serial = None
        self._lock = threading.RLock()
        self._calibration = default_steering_calibration()
        self.last_error = ""
        self.connect()

    @property
    def connected(self) -> bool:
        return self._serial is not None and bool(
            getattr(self._serial, "is_open", True)
        )

    @property
    def calibration(self) -> dict[str, int]:
        return dict(self._calibration)

    def connect(self) -> bool:
        with self._lock:
            if self.connected:
                return True
            factory = self._serial_factory
            if factory is None:
                if serial is None:
                    self.last_error = "pyserial is not installed"
                    return False
                factory = serial.Serial
            try:
                self._serial = factory(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                calibration_command = (
                    f"SC:{self._calibration['left_pw']},"
                    f"{self._calibration['center_pw']},"
                    f"{self._calibration['right_pw']}\n"
                ).encode("ascii")
                self._serial.write(calibration_command)
                self.last_error = ""
                return True
            except Exception as exc:
                self._serial = None
                self.last_error = str(exc)
                return False

    def _write(self, payload: bytes) -> bool:
        with self._lock:
            if not self.connected and not self.connect():
                return False
            try:
                self._serial.write(payload)
                return True
            except Exception as exc:
                self.last_error = str(exc)
                self.close()
                return False

    def drive(self, direction: str, throttle: int, steering: int) -> bool:
        return self._write(
            format_drive_command(
                direction,
                throttle,
                steering,
                self._calibration,
            )
        )

    def brake(self) -> bool:
        return self._write(b"B\n")

    def stop(self) -> bool:
        return self._write(b"S\n")

    def emergency_stop(self) -> bool:
        return self._write(b"E\n")

    def reset_emergency_stop(self) -> bool:
        return self._write(b"ERST\n")

    def set_steering_calibration(
        self, calibration: Mapping[str, object]
    ) -> dict[str, int]:
        normalized = normalize_steering_calibration(calibration)
        command = (
            f"SC:{normalized['left_pw']},{normalized['center_pw']},"
            f"{normalized['right_pw']}\n"
        ).encode("ascii")
        if not self._write(command):
            raise RuntimeError(self.last_error or "Pico is unavailable")
        self._calibration = normalized
        return dict(normalized)

    def apply_local_calibration(
        self, calibration: Mapping[str, object]
    ) -> dict[str, int]:
        self._calibration = normalize_steering_calibration(calibration)
        return dict(self._calibration)

    def _request(self, command: bytes, prefix: bytes) -> str | None:
        with self._lock:
            if not self.connected and not self.connect():
                return None
            try:
                if hasattr(self._serial, "reset_input_buffer"):
                    self._serial.reset_input_buffer()
                self._serial.write(command)
                deadline = time.monotonic() + self.timeout
                while time.monotonic() < deadline:
                    line = self._serial.readline().strip()
                    if line.startswith(prefix):
                        return line.decode("ascii", "replace")
                self.last_error = f"timeout waiting for {prefix.decode('ascii')}"
            except Exception as exc:
                self.last_error = str(exc)
                self.close()
            return None

    def ping(self) -> bool:
        return self._request(b"PING\n", b"PONG") == "PONG"

    def read_battery_voltage(self, divider_ratio: float = 5.0) -> float | None:
        response = self._request(b"BAT?\n", b"BAT:")
        if not response or response == "BAT:ERR":
            return None
        try:
            adc_mv = float(response.split(":", 1)[1])
            return round((adc_mv / 1000.0) * float(divider_ratio), 2)
        except (TypeError, ValueError):
            self.last_error = f"invalid battery response: {response}"
            return None

    def close(self) -> None:
        with self._lock:
            current = self._serial
            self._serial = None
            if current is not None:
                try:
                    current.close()
                except Exception:
                    pass
