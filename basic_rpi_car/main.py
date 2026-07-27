#!/usr/bin/env python3
"""Standalone Raspberry Pi gamepad controller for the basic RC car."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any, Callable, Mapping


UART_BAUD = 115200
DEFAULT_PICO_PORT = "/dev/ttyS0"
STEERING_GPIO = 12
MAX_PWM_DUTY = 95
GAMEPAD_DEADZONE = 0.08
KEEPALIVE_INTERVAL_S = 0.10
SELECT_DOUBLE_PRESS_S = 0.60
ESTOP_RESET_DELAY_S = 0.20

DEFAULT_STEERING_CALIBRATION = {
    "left_pw": 940,
    "center_pw": 1440,
    "right_pw": 2150,
}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_gamepad_axis(value: Any) -> float:
    """Normalize the 8-bit or signed 16-bit axes emitted by ``inputs``."""
    try:
        raw = float(value)
    except (TypeError, ValueError):
        return 0.0

    if 0 <= raw <= 255:
        normalized = (raw - 128.0) / 127.0
    else:
        normalized = raw / 32767.0
    normalized = clamp(normalized, -1.0, 1.0)
    return 0.0 if abs(normalized) < GAMEPAD_DEADZONE else normalized


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
    ):
        self.calibration = normalize_calibration(
            calibration or DEFAULT_STEERING_CALIBRATION
        )
        self.gpio = gpio
        self._pigpio_factory = pigpio_factory
        self._client: Any | None = None
        self._lock = threading.RLock()
        self._last_pulse: int | None = None
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
            import pigpio
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

    def set_steering(self, angle: Any) -> bool:
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
            try:
                result = client.set_servo_pulsewidth(self.gpio, pulse)
                if result not in (None, 0):
                    raise RuntimeError(f"pigpio error {result}")
                self._last_pulse = pulse
                self.last_error = ""
                return True
            except Exception as exc:
                self.last_error = str(exc)
                return False

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
        log: Callable[[str], None] = print,
        now_fn: Callable[[], float] = time.monotonic,
    ):
        self.pico = pico
        self.pi_steering = pi_steering
        self.log = log
        self._now = now_fn
        self._lock = threading.RLock()
        self.gamepad_connected = False
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
        self._shutdown = False

    def _reset_gamepad_state_locked(self) -> None:
        self.direction = "N"
        self.throttle = 0
        self.steering = 0
        self.brake = False
        self._brake_buttons.clear()
        self._trigger_buttons.clear()
        self._last_select_at = 0.0

    def _mark_write_failure_locked(self, action: str) -> None:
        self.armed = False
        self._applied_direction = "N"
        self._applied_throttle = 0
        self.log(
            f"Pico {action} failed; car disarmed: "
            f"{self.pico.last_error or 'UART unavailable'}"
        )

    def _apply_pi_steering_locked(self) -> bool:
        if self.pi_steering is None:
            return True
        if self.pi_steering.set_steering(self.steering):
            return True
        self.armed = False
        self._applied_direction = "N"
        self._applied_throttle = 0
        self.pico.stop()
        self.log(
            "Pi steering command failed; car disarmed: "
            f"{self.pi_steering.last_error or 'GPIO12 unavailable'}"
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
                self.log(
                    "Could not center Pi steering: "
                    f"{self.pi_steering.last_error or 'GPIO12 unavailable'}"
                )
            sent = (
                self.pico.emergency_stop()
                if self.estop
                else self.pico.stop()
            )
            self.gamepad_connected = connected
            if not sent:
                self.log(
                    "Safety stop could not be written; the Pico watchdog "
                    f"remains active: {self.pico.last_error or 'UART unavailable'}"
                )
            self.log(
                "Gamepad connected; car is disarmed."
                if connected
                else "Gamepad disconnected; car stopped and disarmed."
            )

    def arm(self) -> bool:
        with self._lock:
            if not self.gamepad_connected:
                self.log("Cannot arm: no gamepad is connected.")
                return False
            if self.estop:
                self.log("Cannot arm: reset the emergency stop first.")
                return False
            if self.throttle or self.brake:
                self.log("Cannot arm: release throttle and brake first.")
                return False
            if not self.pico.connect():
                self._mark_write_failure_locked("connection")
                return False
            if not self._apply_pi_steering_locked():
                return False
            self.armed = True
            self._applied_direction = "N"
            self._applied_throttle = 0
            if not self.pico.drive("N", 0, self.steering):
                self._mark_write_failure_locked("neutral command")
                return False
            self.log("Car armed.")
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
                self.log(
                    f"Stop command failed: "
                    f"{self.pico.last_error or 'UART unavailable'}"
                )
            self.log(reason)

    def emergency_stop(self) -> None:
        with self._lock:
            sent = self.pico.emergency_stop()
            self.estop = True
            self._estop_at = self._now()
            self.armed = False
            self._applied_direction = "N"
            self._applied_throttle = 0
            if sent:
                self.log("EMERGENCY STOP latched.")
            else:
                self.log(
                    "EMERGENCY STOP requested but UART failed: "
                    f"{self.pico.last_error or 'UART unavailable'}"
                )

    def reset_emergency_stop(self) -> bool:
        with self._lock:
            if not self.estop:
                return True
            if self.throttle or self.brake:
                self.log("Release throttle and brake before resetting e-stop.")
                return False
            if self._now() - self._estop_at < ESTOP_RESET_DELAY_S:
                self.log("Wait for emergency braking to finish before reset.")
                return False
            if not self.pico.reset_emergency_stop():
                self._mark_write_failure_locked("e-stop reset")
                return False
            self.estop = False
            self._estop_at = 0.0
            self.log("Emergency stop reset; car remains disarmed.")
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
        return True

    def refresh_active_command(self) -> None:
        with self._lock:
            if self.armed and (self.throttle or self.brake):
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

            if code == "ABS_Y":
                signed_throttle = -normalize_gamepad_axis(state)
                if signed_throttle > 0:
                    self.direction = "F"
                elif signed_throttle < 0:
                    self.direction = "R"
                else:
                    self.direction = "N"
                self.throttle = round(abs(signed_throttle) * 100)
                send_state = True
            elif code in {"ABS_RX", "ABS_Z"}:
                self.steering = round(normalize_gamepad_axis(state) * 50)
                send_state = True
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
            self.log("Controller stopped; emergency stop sent to Pico.")


class BasicCarRuntime:
    """Own the gamepad reader and active-command refresh workers."""

    def __init__(
        self,
        controller: CarController,
        stop_event: threading.Event | None = None,
    ):
        self.controller = controller
        self.stop_event = stop_event or threading.Event()
        self.exit_code = 0
        self._threads: list[threading.Thread] = []

    def fail(self, message: str) -> None:
        self.controller.log(message)
        self.exit_code = 1
        self.stop_event.set()

    def _gamepad_worker(self) -> None:
        try:
            from inputs import UnpluggedError, devices, get_gamepad
        except ImportError:
            self.fail(
                "The `inputs` package is not installed; run "
                "`python3 -m pip install -r requirements.txt`."
            )
            return

        while not self.stop_event.is_set():
            try:
                if devices.gamepads and not self.controller.gamepad_connected:
                    self.controller.set_gamepad_connected(True)
                events = get_gamepad()
                if not self.controller.gamepad_connected:
                    self.controller.set_gamepad_connected(True)
                for event in events:
                    self.controller.handle_event(event.code, event.state)
            except (UnpluggedError, OSError, RuntimeError, IndexError):
                self.controller.set_gamepad_connected(False)
                self.stop_event.wait(1.0)
            except Exception as exc:
                self.controller.set_gamepad_connected(False)
                self.controller.log(f"Gamepad reader error: {exc}")
                self.stop_event.wait(1.0)

    def _refresh_worker(self) -> None:
        while not self.stop_event.wait(KEEPALIVE_INTERVAL_S):
            self.controller.refresh_active_command()

    def _pico_reconnect_worker(self) -> None:
        last_error = ""
        while not self.stop_event.is_set():
            if not self.controller.pico.connected:
                if self.controller.pico.connect():
                    self.controller.log(
                        f"Pico connected on {self.controller.pico.port}."
                    )
                    last_error = ""
                else:
                    error = (
                        self.controller.pico.last_error
                        or "UART unavailable"
                    )
                    if error != last_error:
                        self.controller.log(
                            f"Waiting for Pico on "
                            f"{self.controller.pico.port}: {error}"
                        )
                        last_error = error
            self.stop_event.wait(1.0)

    def run(self) -> int:
        self.controller.log(
            "basic_rpi_car starting; connect the gamepad and press Start "
            "while the controls are neutral."
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
    calibration = load_calibration(args.calibration)
    pico = PicoController(port=args.port, calibration=calibration)

    if args.ping or args.battery:
        try:
            return run_diagnostic(pico, args.ping, args.battery)
        except ValueError as exc:
            print(f"Configuration error: {exc}")
            return 2

    pi_steering = PiSteeringController(calibration=calibration)
    if not pi_steering.connect():
        print(
            "Pi steering unavailable on GPIO12 (physical pin 32): "
            f"{pi_steering.last_error}",
            flush=True,
        )
        return 1

    controller = CarController(
        pico,
        pi_steering=pi_steering,
        log=lambda message: print(message, flush=True),
    )
    runtime = BasicCarRuntime(controller)

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
        print(f"Controller failed: {exc}", flush=True)
        return 1
    finally:
        controller.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
