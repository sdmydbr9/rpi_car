"""Manual control arbitration and safety rules shared by all input sources."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable

from pico_control import (
    PicoController,
    clamp_steering,
    clamp_throttle,
    normalize_direction,
)


COMMAND_TIMEOUT_S = 0.30
KEEPALIVE_INTERVAL_S = 0.10


class ControlRejected(RuntimeError):
    pass


@dataclass(frozen=True)
class DriveInput:
    seq: int
    direction: str
    throttle: int
    steering: int
    brake: bool

    @classmethod
    def from_payload(cls, payload) -> "DriveInput":
        data = payload if isinstance(payload, dict) else {}
        try:
            seq = int(data.get("seq", 0))
        except (TypeError, ValueError):
            raise ControlRejected("seq must be an integer")
        return cls(
            seq=max(0, seq),
            direction=normalize_direction(data.get("direction", "N")),
            throttle=clamp_throttle(data.get("throttle", 0)),
            steering=clamp_steering(data.get("steering", 0)),
            brake=bool(data.get("brake", False)),
        )


class ManualControlArbiter:
    """Owns arming, source priority, dead-man timeout, and Pico dispatch."""

    def __init__(
        self,
        pico: PicoController,
        now_fn: Callable[[], float] = time.monotonic,
    ):
        self.pico = pico
        self._now = now_fn
        self._lock = threading.RLock()
        self._status_callback: Callable[[dict], None] | None = None

        self.remote_owner: str | None = None
        self.gamepad_connected = False
        self.armed = False
        self.estop = False
        self.estop_at = 0.0
        self.direction = "N"
        self.throttle = 0
        self.steering = 0
        self.brake_active = False
        self.last_command_at = 0.0
        self.last_pico_write_at = 0.0
        self._last_seq_by_source: dict[str, int] = {}

    def set_status_callback(self, callback: Callable[[dict], None]) -> None:
        self._status_callback = callback

    def status(self) -> dict:
        with self._lock:
            active_source = (
                "gamepad"
                if self.gamepad_connected
                else ("remote" if self.remote_owner else None)
            )
            return {
                "pico_connected": self.pico.connected,
                "pico_error": self.pico.last_error,
                "armed": self.armed,
                "estop": self.estop,
                "active_source": active_source,
                "remote_owner": bool(self.remote_owner),
                "gamepad_connected": self.gamepad_connected,
            }

    def _notify(self) -> None:
        callback = self._status_callback
        if callback is not None:
            callback(self.status())

    def _source_key(self, source: str, client_id: str | None) -> str:
        return "gamepad" if source == "gamepad" else f"remote:{client_id or ''}"

    def _authorize(
        self,
        source: str,
        client_id: str | None,
        require_armed: bool = False,
    ) -> None:
        if source == "gamepad":
            if not self.gamepad_connected:
                raise ControlRejected("gamepad is not connected")
        else:
            if self.gamepad_connected:
                raise ControlRejected("gamepad has control priority")
            if not client_id or client_id != self.remote_owner:
                raise ControlRejected("this client does not own control")
        if require_armed and not self.armed:
            raise ControlRejected("car is disarmed")

    def claim_remote(self, client_id: str) -> dict:
        with self._lock:
            if self.gamepad_connected:
                raise ControlRejected("gamepad has control priority")
            if self.remote_owner not in (None, client_id):
                raise ControlRejected("another remote client owns control")
            self.remote_owner = client_id
            self._notify()
            return self.status()

    def release_remote(self, client_id: str) -> dict:
        with self._lock:
            if self.remote_owner == client_id:
                self._stop_and_disarm_locked()
                self.remote_owner = None
                self._last_seq_by_source.pop(f"remote:{client_id}", None)
                self._notify()
            return self.status()

    def set_gamepad_connected(self, connected: bool) -> dict:
        with self._lock:
            connected = bool(connected)
            if connected != self.gamepad_connected:
                self._stop_and_disarm_locked()
                self.gamepad_connected = connected
                self.remote_owner = None
                self._last_seq_by_source.clear()
                self._notify()
            return self.status()

    def arm(self, source: str, client_id: str | None = None) -> dict:
        with self._lock:
            self._authorize(source, client_id)
            if self.estop:
                raise ControlRejected("reset emergency stop before arming")
            if not self.pico.connected and not self.pico.connect():
                raise ControlRejected(self.pico.last_error or "Pico is unavailable")
            self.armed = True
            self.last_command_at = self._now()
            self._notify()
            return self.status()

    def disarm(self, source: str, client_id: str | None = None) -> dict:
        with self._lock:
            self._authorize(source, client_id)
            self._stop_and_disarm_locked()
            self._notify()
            return self.status()

    def force_disarm(self) -> dict:
        with self._lock:
            self._stop_and_disarm_locked()
            self._notify()
            return self.status()

    def _stop_and_disarm_locked(self) -> None:
        if self.estop:
            self.pico.emergency_stop()
        else:
            self.pico.stop()
        self._clear_drive_state_locked()
        self.last_pico_write_at = self._now()

    def _clear_drive_state_locked(self) -> None:
        self.armed = False
        self.direction = "N"
        self.throttle = 0
        self.brake_active = False
        self.last_command_at = 0.0

    def emergency_stop(self) -> dict:
        with self._lock:
            self.pico.emergency_stop()
            self.estop = True
            self.estop_at = self._now()
            self.armed = False
            self.direction = "N"
            self.throttle = 0
            self.brake_active = True
            self.last_command_at = 0.0
            self.last_pico_write_at = self._now()
            self._notify()
            return self.status()

    def reset_emergency_stop(
        self, source: str, client_id: str | None = None
    ) -> dict:
        with self._lock:
            self._authorize(source, client_id)
            if self.armed or self.throttle != 0:
                raise ControlRejected("disarm and release throttle before reset")
            if self._now() - self.estop_at < 0.20:
                raise ControlRejected("wait for braking to finish before reset")
            if not self.pico.reset_emergency_stop():
                raise ControlRejected(self.pico.last_error or "Pico is unavailable")
            self.estop = False
            self.estop_at = 0.0
            self.brake_active = False
            self._notify()
            return self.status()

    def accept_drive(
        self,
        source: str,
        drive: DriveInput,
        client_id: str | None = None,
    ) -> None:
        with self._lock:
            self._authorize(source, client_id, require_armed=True)
            if self.estop:
                raise ControlRejected("emergency stop is active")

            source_key = self._source_key(source, client_id)
            previous_seq = self._last_seq_by_source.get(source_key, -1)
            if drive.seq <= previous_seq:
                raise ControlRejected("stale control sequence")
            self._last_seq_by_source[source_key] = drive.seq

            requested_throttle = 0 if drive.direction == "N" else drive.throttle
            if (
                drive.direction != self.direction
                and drive.direction != "N"
                and self.direction != "N"
                and (self.throttle > 0 or requested_throttle > 0)
            ):
                self.pico.stop()
                self.throttle = 0
                self.direction = "N"
                self.last_pico_write_at = self._now()
                raise ControlRejected("release throttle before changing direction")

            self.direction = drive.direction
            self.throttle = requested_throttle
            self.steering = drive.steering
            self.brake_active = drive.brake
            self.last_command_at = self._now()
            if not self._dispatch_locked():
                self._clear_drive_state_locked()
                self._notify()
                raise ControlRejected(
                    self.pico.last_error or "Pico command failed"
                )

    def _dispatch_locked(self) -> bool:
        if self.estop:
            sent = self.pico.emergency_stop()
        elif self.brake_active:
            sent = self.pico.brake()
        else:
            sent = self.pico.drive(
                self.direction,
                self.throttle,
                self.steering,
            )
        self.last_pico_write_at = self._now()
        return sent

    def tick(self) -> None:
        """Run from a tiny monitor thread; never broadcasts telemetry."""
        notify = False
        with self._lock:
            if not self.armed:
                return
            now = self._now()
            if (
                not self.gamepad_connected
                and self.remote_owner
                and (self.throttle > 0 or self.brake_active)
                and now - self.last_command_at > COMMAND_TIMEOUT_S
            ):
                self._stop_and_disarm_locked()
                notify = True
            elif (
                (self.throttle > 0 or self.brake_active)
                and now - self.last_pico_write_at >= KEEPALIVE_INTERVAL_S
            ):
                if not self._dispatch_locked():
                    self._clear_drive_state_locked()
                    notify = True
        if notify:
            self._notify()

    def read_battery(self, divider_ratio: float) -> float:
        with self._lock:
            if self.armed or self.throttle:
                raise ControlRejected("battery can only be read while disarmed")
            voltage = self.pico.read_battery_voltage(divider_ratio)
            if voltage is None:
                raise ControlRejected(
                    self.pico.last_error or "battery sensor is unavailable"
                )
            return voltage
