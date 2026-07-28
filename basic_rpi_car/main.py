#!/usr/bin/env python3
"""Standalone Raspberry Pi gamepad controller for the basic RC car."""

from __future__ import annotations

import argparse
from array import array
from dataclasses import dataclass
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Any, Callable, Mapping


UART_BAUD = 115200
DEFAULT_PICO_PORT = "/dev/ttyS0"
STEERING_GPIO = 12
MAX_PWM_DUTY = 95
GAMEPAD_DEADZONE = 0.08
KEEPALIVE_INTERVAL_S = 0.10
STEERING_HEALTH_INTERVAL_S = 1.0
SELECT_DOUBLE_PRESS_S = 0.60
ESTOP_RESET_DELAY_S = 0.20
LOG_MAX_BYTES = 5 * 1024 * 1024
LOG_BACKUP_COUNT = 3

LOGGER = logging.getLogger("basic_rpi_car")
LOGGER.addHandler(logging.NullHandler())

ABSOLUTE_AXIS_CODES = {
    "ABS_Y": 0x01,
    "ABS_Z": 0x02,
    "ABS_RX": 0x03,
}
STEERING_AXIS_CHOICES = ("auto", "ABS_RX", "ABS_Z")

DEFAULT_STEERING_CALIBRATION = {
    "left_pw": 940,
    "center_pw": 1440,
    "right_pw": 2150,
}


@dataclass(frozen=True)
class AxisInfo:
    """Linux absolute-axis metadata used for safe normalization."""

    code: str
    minimum: int
    maximum: int
    current: int
    flat: int = 0

    def __post_init__(self) -> None:
        if self.maximum <= self.minimum:
            raise ValueError(
                f"{self.code} has invalid range "
                f"{self.minimum}..{self.maximum}"
            )

    @property
    def center(self) -> float:
        return (self.minimum + self.maximum) / 2.0

    def normalize(self, value: Any) -> float:
        return normalize_gamepad_axis(
            value,
            self.minimum,
            self.maximum,
            self.flat,
        )

    def is_centered(self) -> bool:
        return self.normalize(self.current) == 0.0

    def describe(self) -> str:
        return (
            f"code={self.code} min={self.minimum} max={self.maximum} "
            f"center={self.center:g} current={self.current} flat={self.flat}"
        )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_gamepad_axis(
    value: Any,
    minimum: int,
    maximum: int,
    flat: int = 0,
) -> float:
    """Normalize a raw Linux axis using the device's reported range."""
    try:
        raw = float(value)
    except (TypeError, ValueError):
        return 0.0

    if maximum <= minimum:
        raise ValueError("axis maximum must be greater than minimum")
    center = (minimum + maximum) / 2.0
    if raw < center:
        span = center - minimum
    else:
        span = maximum - center
    if span <= 0:
        return 0.0

    offset = raw - center
    deadzone = max(float(flat), span * GAMEPAD_DEADZONE)
    if abs(offset) <= deadzone:
        return 0.0

    normalized = offset / span
    normalized = clamp(normalized, -1.0, 1.0)
    return normalized


def _eviocgabs(axis_number: int) -> int:
    """Build Linux's EVIOCGABS ioctl request for ``input_absinfo``."""
    ioc_read = 2
    struct_size = 6 * array("i").itemsize
    return (
        (ioc_read << 30)
        | (struct_size << 16)
        | (ord("E") << 8)
        | (0x40 + axis_number)
    )


def read_axis_info(device_path: str, code: str) -> AxisInfo:
    """Read current/min/max/flat values for one evdev absolute axis."""
    try:
        axis_number = ABSOLUTE_AXIS_CODES[code]
    except KeyError as exc:
        raise ValueError(f"unsupported axis code: {code}") from exc

    values = array("i", [0] * 6)
    with open(device_path, "rb", buffering=0) as device:
        fcntl.ioctl(device.fileno(), _eviocgabs(axis_number), values, True)
    return AxisInfo(
        code=code,
        current=values[0],
        minimum=values[1],
        maximum=values[2],
        flat=max(0, values[4]),
    )


def select_steering_axis(
    axes: Mapping[str, AxisInfo],
    preference: str = "auto",
) -> AxisInfo:
    """Select exactly one steering stick without mistaking a trigger."""
    if preference not in STEERING_AXIS_CHOICES:
        raise ValueError(f"unsupported steering axis preference: {preference}")
    if preference != "auto":
        try:
            return axes[preference]
        except KeyError as exc:
            raise ValueError(
                f"requested steering axis {preference} is unavailable"
            ) from exc

    if "ABS_RX" in axes:
        return axes["ABS_RX"]
    fallback = axes.get("ABS_Z")
    if fallback is not None and fallback.is_centered():
        return fallback
    if fallback is not None:
        raise ValueError(
            "ABS_Z is not centered and is likely a trigger; "
            "use --steering-axis only after verifying the controller mapping"
        )
    raise ValueError("gamepad exposes neither ABS_RX nor a safe ABS_Z stick")


def configure_logging(log_file: Path, debug: bool = False) -> logging.Logger:
    """Configure concise console output and bounded detailed file logging."""
    logger = LOGGER
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
        if not isinstance(handler, logging.NullHandler):
            handler.close()

    formatter = logging.Formatter(
        fmt=(
            "%(asctime)s.%(msecs)03d %(levelname)s "
            "thread=%(threadName)s %(message)s"
        ),
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.DEBUG if debug else logging.INFO)
    console.setFormatter(formatter)
    logger.addHandler(console)

    try:
        file_handler = RotatingFileHandler(
            log_file,
            maxBytes=LOG_MAX_BYTES,
            backupCount=LOG_BACKUP_COUNT,
            encoding="utf-8",
        )
    except OSError as exc:
        logger.warning(
            "event=log_file_unavailable path=%s error=%r "
            "fallback=console_only",
            log_file,
            str(exc),
        )
    else:
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        logger.info("event=log_ready path=%s", log_file)
    return logger


def clamp_throttle(value: Any) -> int:
    try:
        throttle = round(float(value))
    except (TypeError, ValueError):
        throttle = 0
    return max(0, min(100, throttle))


def clamp_steering(value: Any) -> int:
    try:
        steering = round(float(value))
    except (TypeError, ValueError):
        steering = 0
    return max(-50, min(50, steering))


def normalize_calibration(data: Mapping[str, Any]) -> dict[str, int]:
    if not isinstance(data, Mapping):
        raise ValueError("steering calibration must be an object")
    try:
        calibration = {
            "left_pw": int(data["left_pw"]),
            "center_pw": int(data["center_pw"]),
            "right_pw": int(data["right_pw"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "calibration requires integer left_pw, center_pw, and right_pw"
        ) from exc
    if not (
        500
        <= calibration["left_pw"]
        < calibration["center_pw"]
        < calibration["right_pw"]
        <= 2500
    ):
        raise ValueError(
            "calibration must satisfy 500 <= left < center < right <= 2500"
        )
    return calibration


def load_calibration(
    path: Path,
    log: Callable[[str], None] = print,
) -> dict[str, int]:
    if not path.is_file():
        return dict(DEFAULT_STEERING_CALIBRATION)
    try:
        with path.open("r", encoding="utf-8") as handle:
            return normalize_calibration(json.load(handle))
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        log(f"Invalid steering calibration in {path}: {exc}; using defaults.")
        return dict(DEFAULT_STEERING_CALIBRATION)


def steering_to_pulse(
    steering: Any,
    calibration: Mapping[str, Any],
) -> int:
    values = normalize_calibration(calibration)
    angle = clamp_steering(steering)
    if angle < 0:
        pulse = values["center_pw"] + (
            angle
            / 50.0
            * (values["center_pw"] - values["left_pw"])
        )
    else:
        pulse = values["center_pw"] + (
            angle
            / 50.0
            * (values["right_pw"] - values["center_pw"])
        )
    return round(pulse)


def throttle_to_pwm(throttle: Any) -> int:
    return round(clamp_throttle(throttle) * MAX_PWM_DUTY / 100.0)


def format_drive_command(
    direction: str,
    throttle: Any,
    steering: Any,
    calibration: Mapping[str, Any],
) -> bytes:
    normalized_direction = str(direction).strip().upper()
    if normalized_direction not in {"F", "N", "R"}:
        raise ValueError("direction must be F, N, or R")
    if normalized_direction == "N":
        throttle = 0
    return (
        f"D:{normalized_direction},{throttle_to_pwm(throttle)},"
        f"{steering_to_pulse(steering, calibration)}\n"
    ).encode("ascii")


class PicoController:
    """Thread-safe UART connection and command transport."""

    def __init__(
        self,
        port: str = DEFAULT_PICO_PORT,
        calibration: Mapping[str, Any] | None = None,
        timeout: float = 0.2,
        serial_factory: Callable[..., Any] | None = None,
    ):
        self.port = port
        self.baudrate = UART_BAUD
        self.timeout = timeout
        self.calibration = normalize_calibration(
            calibration or DEFAULT_STEERING_CALIBRATION
        )
        self._serial_factory = serial_factory
        self._serial: Any | None = None
        self._lock = threading.RLock()
        self.last_error = ""

    @property
    def connected(self) -> bool:
        return self._serial is not None and bool(
            getattr(self._serial, "is_open", True)
        )

    def _get_serial_factory(self) -> Callable[..., Any] | None:
        if self._serial_factory is not None:
            return self._serial_factory
        try:
            import serial
        except ImportError:
            self.last_error = (
                "pyserial is not installed; run "
                "`python3 -m pip install -r requirements.txt`"
            )
            return None
        self._serial_factory = serial.Serial
        return self._serial_factory

    def connect(self) -> bool:
        with self._lock:
            if self.connected:
                return True
            factory = self._get_serial_factory()
            if factory is None:
                return False
            serial_connection: Any | None = None
            try:
                serial_connection = factory(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                calibration = self.calibration
                serial_connection.write(
                    (
                        f"SC:{calibration['left_pw']},"
                        f"{calibration['center_pw']},"
                        f"{calibration['right_pw']}\n"
                    ).encode("ascii")
                )
                serial_connection.write(b"S\n")
                self._serial = serial_connection
                self.last_error = ""
                return True
            except Exception as exc:
                if serial_connection is not None:
                    try:
                        serial_connection.close()
                    except Exception:
                        pass
                self._serial = None
                self.last_error = str(exc)
                return False

    def _write(self, payload: bytes) -> bool:
        with self._lock:
            if not self.connected and not self.connect():
                return False
            serial_connection = self._serial
            if serial_connection is None:
                self.last_error = "UART connection was not opened"
                return False
            try:
                serial_connection.write(payload)
                self.last_error = ""
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
                self.calibration,
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

    def _request(self, command: bytes, prefix: bytes) -> str | None:
        with self._lock:
            if not self.connected and not self.connect():
                return None
            serial_connection = self._serial
            if serial_connection is None:
                self.last_error = "UART connection was not opened"
                return None
            try:
                if hasattr(serial_connection, "reset_input_buffer"):
                    serial_connection.reset_input_buffer()
                serial_connection.write(command)
                deadline = time.monotonic() + self.timeout
                while time.monotonic() < deadline:
                    response = serial_connection.readline().strip()
                    if response.startswith(prefix):
                        self.last_error = ""
                        return response.decode("ascii", "replace")
                self.last_error = (
                    f"timeout waiting for {prefix.decode('ascii', 'replace')}"
                )
            except Exception as exc:
                self.last_error = str(exc)
                self.close()
            return None

    def ping(self) -> bool:
        return self._request(b"PING\n", b"PONG") == "PONG"

    def read_battery_adc_mv(self) -> float | None:
        response = self._request(b"BAT?\n", b"BAT:")
        if not response or response == "BAT:ERR":
            if response == "BAT:ERR":
                self.last_error = "Pico could not read the stationary battery"
            return None
        try:
            return float(response.split(":", 1)[1])
        except (IndexError, ValueError):
            self.last_error = f"invalid battery response: {response}"
            return None

    def close(self) -> None:
        with self._lock:
            serial_connection = self._serial
            self._serial = None
            if serial_connection is not None:
                try:
                    serial_connection.close()
                except Exception:
                    pass


class PiSteeringController:
    """Drive the steering servo directly from Raspberry Pi GPIO12.

    ``pigpio`` provides stable 50 Hz servo pulses through the already-running
    ``pigpiod`` daemon.  It is imported lazily so UART-only diagnostics remain
    usable on development machines.
    """

    def __init__(
        self,
        calibration: Mapping[str, Any] | None = None,
        gpio: int = STEERING_GPIO,
        pigpio_factory: Callable[[], Any] | None = None,
        logger: logging.Logger | None = None,
    ):
        self.calibration = normalize_calibration(
            calibration or DEFAULT_STEERING_CALIBRATION
        )
        self.gpio = gpio
        self._pigpio_factory = pigpio_factory
        self._client: Any | None = None
        self._lock = threading.RLock()
        self._last_pulse: int | None = None
        self.logger = logger or LOGGER
        self.last_error = ""

    @property
    def connected(self) -> bool:
        return self._client is not None and bool(
            getattr(self._client, "connected", True)
        )

    def _get_factory(self) -> Callable[[], Any] | None:
        if self._pigpio_factory is not None:
            return self._pigpio_factory
        try:
            import pigpio  # type: ignore[import-not-found]
        except ImportError:
            self.last_error = (
                "pigpio is not installed; install it and enable pigpiod"
            )
            return None
        self._pigpio_factory = pigpio.pi
        return self._pigpio_factory

    def connect(self) -> bool:
        with self._lock:
            if self.connected:
                return True
            factory = self._get_factory()
            if factory is None:
                return False
            client: Any | None = None
            try:
                client = factory()
                if not bool(getattr(client, "connected", True)):
                    raise RuntimeError("cannot connect to the pigpiod daemon")
                self._client = client
                self._last_pulse = None
                if not self.set_steering(0):
                    raise RuntimeError(self.last_error or "cannot center steering")
                self.last_error = ""
                return True
            except Exception as exc:
                if client is not None:
                    try:
                        client.stop()
                    except Exception:
                        pass
                self._client = None
                self._last_pulse = None
                self.last_error = str(exc)
                return False

    def _read_pulse_locked(self, client: Any) -> int:
        readback = client.get_servo_pulsewidth(self.gpio)
        if readback is None or int(readback) < 0:
            raise RuntimeError(f"pigpio readback error {readback}")
        return int(readback)

    def _write_verified_locked(
        self,
        client: Any,
        pulse: int,
        sequence: int | None,
        reason: str,
    ) -> bool:
        last_failure = "unknown pigpio failure"
        sequence_value = sequence if sequence is not None else "-"
        for attempt in (1, 2):
            try:
                result = client.set_servo_pulsewidth(self.gpio, pulse)
                if result not in (None, 0):
                    raise RuntimeError(f"pigpio write error {result}")
                readback = self._read_pulse_locked(client)
                self.logger.debug(
                    "event=steering_apply seq=%s reason=%s attempt=%d "
                    "gpio=%d requested_us=%d readback_us=%d",
                    sequence_value,
                    reason,
                    attempt,
                    self.gpio,
                    pulse,
                    readback,
                )
                if readback == pulse:
                    self._last_pulse = pulse
                    self.last_error = ""
                    return True
                last_failure = (
                    f"steering readback mismatch requested={pulse} "
                    f"actual={readback}"
                )
            except Exception as exc:
                last_failure = str(exc)
            self.logger.warning(
                "event=steering_retry seq=%s reason=%s attempt=%d "
                "gpio=%d requested_us=%d error=%r",
                sequence_value,
                reason,
                attempt,
                self.gpio,
                pulse,
                last_failure,
            )

        self._last_pulse = None
        self.last_error = last_failure
        return False

    def set_steering(
        self,
        angle: Any,
        sequence: int | None = None,
    ) -> bool:
        with self._lock:
            if not self.connected and not self.connect():
                return False
            client = self._client
            if client is None:
                self.last_error = "pigpio connection was not opened"
                return False
            pulse = steering_to_pulse(angle, self.calibration)
            if pulse == self._last_pulse:
                return True
            return self._write_verified_locked(
                client,
                pulse,
                sequence,
                "input",
            )

    def verify_steering(self, angle: Any) -> bool:
        """Read back GPIO12 and restore the requested pulse if it changed."""
        with self._lock:
            if not self.connected and not self.connect():
                return False
            client = self._client
            if client is None:
                self.last_error = "pigpio connection was not opened"
                return False
            pulse = steering_to_pulse(angle, self.calibration)
            try:
                readback = self._read_pulse_locked(client)
                self.logger.debug(
                    "event=steering_health gpio=%d requested_us=%d "
                    "readback_us=%d status=%s",
                    self.gpio,
                    pulse,
                    readback,
                    "ok" if readback == pulse else "mismatch",
                )
                if readback == pulse:
                    self._last_pulse = pulse
                    self.last_error = ""
                    return True
                self.logger.warning(
                    "event=steering_health_mismatch gpio=%d "
                    "requested_us=%d readback_us=%d action=correct",
                    self.gpio,
                    pulse,
                    readback,
                )
            except Exception as exc:
                self.logger.warning(
                    "event=steering_health_error gpio=%d requested_us=%d "
                    "error=%r action=correct",
                    self.gpio,
                    pulse,
                    str(exc),
                )
            return self._write_verified_locked(
                client,
                pulse,
                None,
                "health",
            )

    def center(self) -> bool:
        return self.set_steering(0)

    def close(self) -> None:
        with self._lock:
            client = self._client
            self._client = None
            self._last_pulse = None
            if client is None:
                return
            try:
                client.set_servo_pulsewidth(self.gpio, 0)
            except Exception:
                pass
            try:
                client.stop()
            except Exception:
                pass


class CarController:
    """Gamepad state machine, safety rules, and Pico dispatch."""

    def __init__(
        self,
        pico: PicoController,
        pi_steering: PiSteeringController | None = None,
        log: Callable[[str], None] | None = None,
        now_fn: Callable[[], float] = time.monotonic,
        logger: logging.Logger | None = None,
    ):
        self.pico = pico
        self.pi_steering = pi_steering
        self.logger = logger or LOGGER
        self._legacy_log = log
        self._now = now_fn
        self._lock = threading.RLock()
        self.gamepad_connected = False
        self.gamepad_name = ""
        self.gamepad_path = ""
        self.throttle_axis: AxisInfo | None = None
        self.steering_axis: AxisInfo | None = None
        self.axis_configuration_error = (
            "gamepad axis metadata has not been configured"
        )
        self.armed = False
        self.estop = False
        self.direction = "N"
        self.throttle = 0
        self.steering = 0
        self.brake = False
        self._brake_buttons: set[str] = set()
        self._trigger_buttons: set[str] = set()
        self._last_select_at = 0.0
        self._estop_at = 0.0
        self._applied_direction = "N"
        self._applied_throttle = 0
        self._steering_sequence = 0
        self._last_steering_health_at = 0.0
        self._last_logged_drive_state: tuple[str, int, int, bool] | None = None
        self._shutdown = False

    def _emit(self, level: int, message: str) -> None:
        self.logger.log(level, message)
        if self._legacy_log is not None:
            self._legacy_log(message)

    def log(self, message: str) -> None:
        """Compatibility entry point for runtime status messages."""
        self._emit(logging.INFO, message)

    def configure_gamepad(
        self,
        name: str,
        device_path: str,
        throttle_axis: AxisInfo | None,
        steering_axis: AxisInfo | None,
        error: str = "",
    ) -> None:
        """Install one immutable axis profile for this gamepad connection."""
        with self._lock:
            self.gamepad_name = name
            self.gamepad_path = device_path
            self.throttle_axis = throttle_axis
            self.steering_axis = steering_axis
            self.axis_configuration_error = error
            if error:
                self._emit(
                    logging.ERROR,
                    "event=gamepad_axis_error "
                    f"name={name!r} path={device_path!r} error={error!r}",
                )
                return
            if throttle_axis is None or steering_axis is None:
                self.axis_configuration_error = "required axis metadata missing"
                self._emit(
                    logging.ERROR,
                    "event=gamepad_axis_error "
                    f"name={name!r} path={device_path!r} "
                    "error='required axis metadata missing'",
                )
                return

            signed_throttle = -throttle_axis.normalize(
                throttle_axis.current
            )
            if signed_throttle > 0:
                self.direction = "F"
            elif signed_throttle < 0:
                self.direction = "R"
            else:
                self.direction = "N"
            self.throttle = round(abs(signed_throttle) * 100)
            self.steering = round(
                steering_axis.normalize(steering_axis.current) * 50
            )
            self._emit(
                logging.INFO,
                "event=gamepad_axes_ready "
                f"name={name!r} path={device_path!r} "
                f"throttle=({throttle_axis.describe()}) "
                f"steering=({steering_axis.describe()})",
            )

    def _reset_gamepad_state_locked(self) -> None:
        self.direction = "N"
        self.throttle = 0
        self.steering = 0
        self.brake = False
        self._brake_buttons.clear()
        self._trigger_buttons.clear()
        self._last_select_at = 0.0
        self._last_logged_drive_state = None

    def _mark_write_failure_locked(self, action: str) -> None:
        self.armed = False
        self._applied_direction = "N"
        self._applied_throttle = 0
        self._emit(
            logging.ERROR,
            "event=pico_failure action="
            f"{action!r} result=disarmed error="
            f"{(self.pico.last_error or 'UART unavailable')!r}",
        )

    def _apply_pi_steering_locked(self) -> bool:
        if self.pi_steering is None:
            return True
        if self.pi_steering.set_steering(
            self.steering,
            sequence=self._steering_sequence,
        ):
            return True
        self.armed = False
        self._applied_direction = "N"
        self._applied_throttle = 0
        self.pico.stop()
        self._emit(
            logging.ERROR,
            "event=steering_failure action=apply result=disarmed "
            f"seq={self._steering_sequence} angle={self.steering} "
            f"error={(self.pi_steering.last_error or 'GPIO12 unavailable')!r}",
        )
        return False

    def _verify_pi_steering_locked(self) -> bool:
        if self.pi_steering is None:
            return True
        if self.pi_steering.verify_steering(self.steering):
            return True
        self.armed = False
        self._applied_direction = "N"
        self._applied_throttle = 0
        self.pico.stop()
        self._emit(
            logging.ERROR,
            "event=steering_failure action=health_check result=disarmed "
            f"angle={self.steering} "
            f"error={(self.pi_steering.last_error or 'GPIO12 unavailable')!r}",
        )
        return False

    def set_gamepad_connected(self, connected: bool) -> None:
        with self._lock:
            connected = bool(connected)
            if connected == self.gamepad_connected:
                return
            self.armed = False
            self._reset_gamepad_state_locked()
            self._applied_direction = "N"
            self._applied_throttle = 0
            if self.pi_steering is not None and not self.pi_steering.center():
                self._emit(
                    logging.ERROR,
                    "event=steering_center_failure "
                    f"error={(self.pi_steering.last_error or 'GPIO12 unavailable')!r}",
                )
            sent = (
                self.pico.emergency_stop()
                if self.estop
                else self.pico.stop()
            )
            self.gamepad_connected = connected
            if not connected:
                self.gamepad_name = ""
                self.gamepad_path = ""
                self.throttle_axis = None
                self.steering_axis = None
                self.axis_configuration_error = (
                    "gamepad axis metadata has not been configured"
                )
            if not sent:
                self._emit(
                    logging.ERROR,
                    "event=safety_stop_failure watchdog=active "
                    f"error={(self.pico.last_error or 'UART unavailable')!r}",
                )
            self._emit(
                logging.INFO,
                "event=gamepad_connection connected=true result=disarmed"
                if connected
                else (
                    "event=gamepad_connection connected=false "
                    "result=stopped_disarmed"
                ),
            )

    def arm(self) -> bool:
        with self._lock:
            if not self.gamepad_connected:
                self._emit(
                    logging.WARNING,
                    "event=arm_rejected reason=no_gamepad",
                )
                return False
            if self.axis_configuration_error:
                self._emit(
                    logging.ERROR,
                    "event=arm_rejected reason=axis_configuration "
                    f"error={self.axis_configuration_error!r}",
                )
                return False
            if self.estop:
                self._emit(
                    logging.WARNING,
                    "event=arm_rejected reason=emergency_stop",
                )
                return False
            if self.throttle or self.brake:
                self._emit(
                    logging.WARNING,
                    "event=arm_rejected reason=controls_not_neutral "
                    f"throttle={self.throttle} brake={self.brake}",
                )
                return False
            if not self.pico.connect():
                self._mark_write_failure_locked("connection")
                return False
            if not self._apply_pi_steering_locked():
                return False
            self.armed = True
            self._applied_direction = "N"
            self._applied_throttle = 0
            self._last_steering_health_at = self._now()
            if not self.pico.drive("N", 0, self.steering):
                self._mark_write_failure_locked("neutral command")
                return False
            self._emit(
                logging.INFO,
                "event=arm result=armed "
                f"steering={self.steering} throttle={self.throttle}",
            )
            return True

    def disarm(self, reason: str = "Car disarmed.") -> None:
        with self._lock:
            self.armed = False
            self._applied_direction = "N"
            self._applied_throttle = 0
            sent = (
                self.pico.emergency_stop()
                if self.estop
                else self.pico.stop()
            )
            if not sent:
                self._emit(
                    logging.ERROR,
                    "event=stop_failure "
                    f"error={(self.pico.last_error or 'UART unavailable')!r}",
                )
            self._emit(logging.INFO, f"event=disarm reason={reason!r}")

    def emergency_stop(self) -> None:
        with self._lock:
            sent = self.pico.emergency_stop()
            self.estop = True
            self._estop_at = self._now()
            self.armed = False
            self._applied_direction = "N"
            self._applied_throttle = 0
            if sent:
                self._emit(
                    logging.CRITICAL,
                    "event=emergency_stop result=latched",
                )
            else:
                self._emit(
                    logging.CRITICAL,
                    "event=emergency_stop result=uart_failed "
                    f"error={(self.pico.last_error or 'UART unavailable')!r}",
                )

    def reset_emergency_stop(self) -> bool:
        with self._lock:
            if not self.estop:
                return True
            if self.throttle or self.brake:
                self._emit(
                    logging.WARNING,
                    "event=estop_reset_rejected reason=controls_not_neutral",
                )
                return False
            if self._now() - self._estop_at < ESTOP_RESET_DELAY_S:
                self._emit(
                    logging.WARNING,
                    "event=estop_reset_rejected reason=braking_delay",
                )
                return False
            if not self.pico.reset_emergency_stop():
                self._mark_write_failure_locked("e-stop reset")
                return False
            self.estop = False
            self._estop_at = 0.0
            self._emit(
                logging.INFO,
                "event=estop_reset result=reset_disarmed",
            )
            return True

    def _send_current_locked(self) -> bool:
        if not self.armed or self.estop:
            return False
        if not self._apply_pi_steering_locked():
            return False
        if self.brake:
            if not self.pico.brake():
                self._mark_write_failure_locked("brake command")
                return False
            self._applied_throttle = 0
            self._log_drive_state_locked()
            return True

        requested_direction = self.direction
        requested_throttle = (
            0 if requested_direction == "N" else self.throttle
        )
        reversing = (
            self._applied_direction in {"F", "R"}
            and requested_direction in {"F", "R"}
            and self._applied_direction != requested_direction
            and (self._applied_throttle > 0 or requested_throttle > 0)
        )
        if reversing:
            if not self.pico.stop():
                self._mark_write_failure_locked("direction-change stop")
                return False
            self._applied_direction = "N"
            self._applied_throttle = 0
            self.logger.debug(
                "event=drive_direction_dwell requested_direction=%s "
                "requested_throttle=%d",
                requested_direction,
                requested_throttle,
            )
            return True

        if not self.pico.drive(
            requested_direction,
            requested_throttle,
            self.steering,
        ):
            self._mark_write_failure_locked("drive command")
            return False
        self._applied_direction = requested_direction
        self._applied_throttle = requested_throttle
        self._log_drive_state_locked()
        return True

    def _log_drive_state_locked(self) -> None:
        state = (
            self.direction,
            self.throttle,
            self.steering,
            self.brake,
        )
        if state == self._last_logged_drive_state:
            return
        self._last_logged_drive_state = state
        self.logger.debug(
            "event=drive_state direction=%s throttle=%d steering=%d "
            "brake=%s",
            self.direction,
            self.throttle,
            self.steering,
            str(self.brake).lower(),
        )

    def refresh_active_command(self) -> None:
        with self._lock:
            if not self.armed:
                return
            now = self._now()
            if (
                now - self._last_steering_health_at
                >= STEERING_HEALTH_INTERVAL_S
            ):
                self._last_steering_health_at = now
                if not self._verify_pi_steering_locked():
                    return
            if self.throttle or self.brake:
                self._send_current_locked()

    def handle_event(
        self,
        code: str,
        state: Any,
        event_time: float | None = None,
    ) -> None:
        with self._lock:
            if not self.gamepad_connected:
                return
            send_state = False

            if code == "ABS_Y" and self.throttle_axis is not None:
                normalized = self.throttle_axis.normalize(state)
                signed_throttle = -normalized
                if signed_throttle > 0:
                    self.direction = "F"
                elif signed_throttle < 0:
                    self.direction = "R"
                else:
                    self.direction = "N"
                self.throttle = round(abs(signed_throttle) * 100)
                self.logger.debug(
                    "event=throttle_input code=%s raw=%r normalized=%.6f "
                    "direction=%s throttle=%d source_ts=%r",
                    code,
                    state,
                    normalized,
                    self.direction,
                    self.throttle,
                    event_time,
                )
                send_state = True
            elif (
                self.steering_axis is not None
                and code == self.steering_axis.code
            ):
                normalized = self.steering_axis.normalize(state)
                self.steering = round(normalized * 50)
                self._steering_sequence += 1
                pulse = (
                    steering_to_pulse(
                        self.steering,
                        self.pi_steering.calibration,
                    )
                    if self.pi_steering is not None
                    else None
                )
                self.logger.debug(
                    "event=steering_input seq=%d code=%s raw=%r "
                    "normalized=%.6f angle=%d pulse_us=%s source_ts=%r",
                    self._steering_sequence,
                    code,
                    state,
                    normalized,
                    self.steering,
                    pulse if pulse is not None else "-",
                    event_time,
                )
                send_state = True
            elif code in {"ABS_RX", "ABS_Z"}:
                self.logger.debug(
                    "event=axis_ignored code=%s raw=%r selected_steering=%s "
                    "source_ts=%r",
                    code,
                    state,
                    (
                        self.steering_axis.code
                        if self.steering_axis is not None
                        else "unconfigured"
                    ),
                    event_time,
                )
            elif code in {"BTN_THUMBL", "BTN_THUMBR"}:
                if bool(state):
                    self._brake_buttons.add(code)
                    self.throttle = 0
                else:
                    self._brake_buttons.discard(code)
                self.brake = bool(self._brake_buttons)
                send_state = True
            elif code == "BTN_START" and int(state) == 1:
                if self.armed:
                    self.disarm()
                else:
                    self.arm()
                return
            elif code == "BTN_SELECT" and int(state) == 1:
                now = self._now() if event_time is None else event_time
                if (
                    self._last_select_at
                    and now - self._last_select_at
                    <= SELECT_DOUBLE_PRESS_S
                ):
                    self._last_select_at = 0.0
                    self.disarm("Double-Select stop: car disarmed.")
                else:
                    self._last_select_at = now
                return
            elif code in {"BTN_TL2", "BTN_TR2"}:
                if bool(state):
                    self._trigger_buttons.add(code)
                else:
                    self._trigger_buttons.discard(code)
            elif code in {"BTN_WEST", "BTN_X"} and int(state) == 1:
                if self._trigger_buttons:
                    if self.estop:
                        self.reset_emergency_stop()
                    else:
                        self.emergency_stop()
                return

            if send_state:
                self._send_current_locked()

    def shutdown(self) -> None:
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self.armed = False
            self.estop = True
            self.pico.emergency_stop()
            self.pico.close()
            if self.pi_steering is not None:
                self.pi_steering.center()
                self.pi_steering.close()
            self._emit(
                logging.INFO,
                "event=shutdown result=pico_estop steering_closed",
            )


class BasicCarRuntime:
    """Own the gamepad reader and active-command refresh workers."""

    def __init__(
        self,
        controller: CarController,
        stop_event: threading.Event | None = None,
        steering_axis_preference: str = "auto",
    ):
        self.controller = controller
        self.stop_event = stop_event or threading.Event()
        self.steering_axis_preference = steering_axis_preference
        self.exit_code = 0
        self._threads: list[threading.Thread] = []
        self._last_gamepad_error = ""

    def fail(self, message: str) -> None:
        self.controller._emit(logging.ERROR, message)
        self.exit_code = 1
        self.stop_event.set()

    def _configure_connected_gamepad(self, gamepad: Any) -> None:
        name = str(getattr(gamepad, "name", "Unknown gamepad"))
        try:
            device_path = str(gamepad.get_char_device_path())
        except Exception:
            device_path = "unknown"

        axes: dict[str, AxisInfo] = {}
        discovery_errors: dict[str, str] = {}
        if device_path != "unknown":
            for code in ABSOLUTE_AXIS_CODES:
                try:
                    axes[code] = read_axis_info(device_path, code)
                except (OSError, ValueError) as exc:
                    discovery_errors[code] = str(exc)
                    self.controller.logger.debug(
                        "event=axis_unavailable name=%r path=%r "
                        "code=%s error=%r",
                        name,
                        device_path,
                        code,
                        str(exc),
                    )

        throttle_axis = axes.get("ABS_Y")
        steering_axis: AxisInfo | None = None
        error = ""
        if throttle_axis is None:
            error = (
                "ABS_Y throttle metadata unavailable: "
                f"{discovery_errors.get('ABS_Y', 'axis not exposed')}"
            )
        else:
            try:
                steering_axis = select_steering_axis(
                    axes,
                    self.steering_axis_preference,
                )
            except ValueError as exc:
                error = str(exc)

        self.controller.set_gamepad_connected(True)
        self.controller.configure_gamepad(
            name,
            device_path,
            throttle_axis,
            steering_axis,
            error,
        )
        if not error:
            self._last_gamepad_error = ""

    def _gamepad_worker(self) -> None:
        try:
            from inputs import (  # type: ignore[import-not-found]
                UnpluggedError,
                devices,
                get_gamepad,
            )
        except ImportError:
            self.fail(
                "The `inputs` package is not installed; run "
                "`python3 -m pip install -r requirements.txt`."
            )
            return

        while not self.stop_event.is_set():
            try:
                if devices.gamepads and not self.controller.gamepad_connected:
                    self._configure_connected_gamepad(devices.gamepads[0])
                events = get_gamepad()
                if not self.controller.gamepad_connected:
                    self._configure_connected_gamepad(devices.gamepads[0])
                for event in events:
                    self.controller.handle_event(
                        event.code,
                        event.state,
                        event_time=getattr(event, "timestamp", None),
                    )
            except (
                UnpluggedError,
                OSError,
                RuntimeError,
                IndexError,
            ) as exc:
                error = str(exc)
                if error != self._last_gamepad_error:
                    self.controller.logger.warning(
                        "event=gamepad_reader_unavailable error=%r",
                        error,
                    )
                    self._last_gamepad_error = error
                self.controller.set_gamepad_connected(False)
                self.stop_event.wait(1.0)
            except Exception as exc:
                self.controller.set_gamepad_connected(False)
                self.controller._emit(
                    logging.ERROR,
                    f"event=gamepad_reader_error error={str(exc)!r}",
                )
                self.stop_event.wait(1.0)

    def _refresh_worker(self) -> None:
        while not self.stop_event.wait(KEEPALIVE_INTERVAL_S):
            self.controller.refresh_active_command()

    def _pico_reconnect_worker(self) -> None:
        last_error = ""
        while not self.stop_event.is_set():
            if not self.controller.pico.connected:
                if self.controller.pico.connect():
                    self.controller._emit(
                        logging.INFO,
                        "event=pico_connection connected=true "
                        f"port={self.controller.pico.port!r}",
                    )
                    last_error = ""
                else:
                    error = (
                        self.controller.pico.last_error
                        or "UART unavailable"
                    )
                    if error != last_error:
                        self.controller._emit(
                            logging.WARNING,
                            "event=pico_connection connected=false "
                            f"port={self.controller.pico.port!r} "
                            f"error={error!r}",
                        )
                        last_error = error
            self.stop_event.wait(1.0)

    def run(self) -> int:
        self.controller._emit(
            logging.INFO,
            "event=startup status=waiting_for_gamepad "
            "instruction='press Start while controls are neutral'",
        )
        self._threads = [
            threading.Thread(
                target=self._gamepad_worker,
                daemon=True,
                name="gamepad-reader",
            ),
            threading.Thread(
                target=self._refresh_worker,
                daemon=True,
                name="command-refresh",
            ),
            threading.Thread(
                target=self._pico_reconnect_worker,
                daemon=True,
                name="pico-reconnect",
            ),
        ]
        for thread in self._threads:
            thread.start()
        while not self.stop_event.wait(0.25):
            pass
        return self.exit_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Gamepad-only Raspberry Pi RC car controller."
    )
    diagnostics = parser.add_mutually_exclusive_group()
    diagnostics.add_argument(
        "--ping",
        action="store_true",
        help="check the Pico UART connection and exit",
    )
    diagnostics.add_argument(
        "--battery",
        action="store_true",
        help="read the stationary battery voltage and exit",
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("RC_PICO_PORT", DEFAULT_PICO_PORT),
        help=(
            "Pico UART device "
            f"(default: RC_PICO_PORT or {DEFAULT_PICO_PORT})"
        ),
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
        "--steering-axis",
        choices=STEERING_AXIS_CHOICES,
        default="auto",
        help=(
            "steering input axis; auto prefers ABS_RX and safely falls "
            "back to ABS_Z (default: auto)"
        ),
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=Path(__file__).with_name("basic_rpi_car.log"),
        help="rotating detailed log file (default: beside main.py)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="also show detailed input and GPIO diagnostics on the console",
    )
    return parser


def battery_divider_ratio() -> float:
    raw = os.environ.get("RC_BATTERY_DIVIDER_RATIO", "5.0")
    try:
        ratio = float(raw)
    except ValueError as exc:
        raise ValueError(
            "RC_BATTERY_DIVIDER_RATIO must be a positive number"
        ) from exc
    if ratio <= 0:
        raise ValueError(
            "RC_BATTERY_DIVIDER_RATIO must be a positive number"
        )
    return ratio


def run_diagnostic(
    pico: PicoController,
    ping: bool,
    battery: bool,
) -> int:
    if not pico.connect():
        print(f"Pico connection failed: {pico.last_error}")
        return 1
    try:
        if ping:
            if pico.ping():
                print(f"Pico responded on {pico.port}: PONG")
                return 0
            print(f"Pico ping failed: {pico.last_error}")
            return 1
        if battery:
            adc_mv = pico.read_battery_adc_mv()
            if adc_mv is None:
                print(f"Battery read failed: {pico.last_error}")
                return 1
            pack_voltage = adc_mv / 1000.0 * battery_divider_ratio()
            print(
                f"Battery ADC: {adc_mv:.1f} mV; "
                f"estimated pack voltage: {pack_voltage:.2f} V"
            )
            return 0
        return 0
    finally:
        pico.stop()
        pico.close()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logger = configure_logging(args.log_file, args.debug)
    calibration = load_calibration(args.calibration, log=logger.warning)
    pico = PicoController(port=args.port, calibration=calibration)

    if args.ping or args.battery:
        try:
            return run_diagnostic(pico, args.ping, args.battery)
        except ValueError as exc:
            print(f"Configuration error: {exc}")
            return 2

    pi_steering = PiSteeringController(
        calibration=calibration,
        logger=logger,
    )
    if not pi_steering.connect():
        logger.error(
            "event=steering_connection connected=false gpio=%d "
            "physical_pin=32 error=%r",
            STEERING_GPIO,
            pi_steering.last_error,
        )
        return 1

    controller = CarController(
        pico,
        pi_steering=pi_steering,
        logger=logger,
    )
    runtime = BasicCarRuntime(
        controller,
        steering_axis_preference=args.steering_axis,
    )

    def request_shutdown(_signum=None, _frame=None) -> None:
        runtime.stop_event.set()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        return runtime.run()
    except KeyboardInterrupt:
        runtime.stop_event.set()
        return 0
    except Exception as exc:
        logger.exception("event=controller_failure error=%r", str(exc))
        return 1
    finally:
        controller.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
