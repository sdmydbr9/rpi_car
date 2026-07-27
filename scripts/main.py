#!/usr/bin/env python3
"""Manual-only, low-latency Raspberry Pi car control server."""

from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

from flask import Flask, jsonify, make_response, render_template, request, send_from_directory
from flask_cors import CORS
from flask_socketio import SocketIO, emit


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)
CORE_DIR = os.path.join(SCRIPTS_DIR, "core")
DIST_DIR = os.path.join(PROJECT_ROOT, "dist")
if CORE_DIR not in sys.path:
    sys.path.insert(0, CORE_DIR)

from manual_control import ControlRejected, DriveInput, ManualControlArbiter
from network_core import PiCarNetworkManager
from pico_control import PicoController
from steering_calibration import (
    SteeringCalibrationError,
    default_steering_calibration,
    normalize_steering_calibration,
)


STEERING_CONFIG_PATH = os.path.join(PROJECT_ROOT, ".steering_config.json")
BATTERY_DIVIDER_RATIO = float(os.environ.get("RC_BATTERY_DIVIDER_RATIO", "5.0"))


def load_steering_calibration() -> dict[str, int]:
    try:
        with open(STEERING_CONFIG_PATH, "r", encoding="utf-8") as handle:
            return normalize_steering_calibration(json.load(handle))
    except (OSError, ValueError, TypeError, SteeringCalibrationError):
        return default_steering_calibration()


def save_steering_calibration(calibration: dict[str, int]) -> None:
    temporary_path = f"{STEERING_CONFIG_PATH}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as handle:
        json.dump(calibration, handle, indent=2)
        handle.write("\n")
    os.replace(temporary_path, STEERING_CONFIG_PATH)


def get_local_ip() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        probe.close()


def get_hotspot_status() -> bool:
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show", "--active"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
        return any(
            "hotspot" in line.lower() or "carhotspot" in line.lower()
            for line in result.stdout.splitlines()
        )
    except (OSError, subprocess.SubprocessError):
        return False


pico = PicoController()
steering_calibration = load_steering_calibration()
pico.apply_local_calibration(steering_calibration)
if pico.connected:
    try:
        pico.set_steering_calibration(steering_calibration)
    except RuntimeError:
        pass

arbiter = ManualControlArbiter(pico)
network_manager = PiCarNetworkManager()

app = Flask(
    __name__,
    static_folder=os.path.join(DIST_DIR, "assets"),
    static_url_path="/assets",
    template_folder=DIST_DIR,
)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


def broadcast_status(status: dict) -> None:
    socketio.emit("control_status", status)


arbiter.set_status_callback(broadcast_status)


def emit_control_error(message: str) -> None:
    emit("control_error", {"message": message})


@app.route("/")
def index():
    if not os.path.isfile(os.path.join(DIST_DIR, "index.html")):
        return (
            "Web client is not built. Run `npm install && npm run build` "
            "after transferring the project.",
            503,
        )
    response = make_response(render_template("index.html"))
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return response


@app.route("/<path:filename>")
def serve_root_file(filename: str):
    return send_from_directory(DIST_DIR, filename)


@app.route("/api/server-ip")
def api_server_ip():
    hotspot = get_hotspot_status()
    return jsonify(
        {
            "ip": get_local_ip(),
            "port": 5000,
            "hotspot_active": hotspot,
            "transport": "hotspot" if hotspot else "wifi",
        }
    )


@app.route("/api/status")
def api_status():
    return jsonify(arbiter.status())


@app.route("/api/steering/calibration", methods=["GET"])
def api_get_steering_calibration():
    return jsonify({"status": "ok", "calibration": pico.calibration})


@app.route("/api/steering/calibration", methods=["POST"])
def api_set_steering_calibration():
    if arbiter.armed:
        return jsonify({"status": "error", "message": "disarm before calibration"}), 409
    try:
        normalized = normalize_steering_calibration(
            request.get_json(silent=True) or {}
        )
        applied = pico.set_steering_calibration(normalized)
        save_steering_calibration(applied)
        return jsonify({"status": "ok", "calibration": applied})
    except (SteeringCalibrationError, RuntimeError) as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400


@app.route("/system/switch_network_mode", methods=["POST"])
def switch_network_mode():
    if arbiter.armed:
        return jsonify(
            {"status": "error", "message": "disarm before switching networks"}
        ), 409
    mode = str((request.get_json(silent=True) or {}).get("mode", "")).lower()
    arbiter.force_disarm()
    if mode == "hotspot":
        ok = network_manager.enable_hotspot()
    elif mode == "wifi":
        ok = network_manager.enable_wifi()
    else:
        return jsonify({"status": "error", "message": "mode must be wifi or hotspot"}), 400
    return jsonify({"status": "ok" if ok else "error", "mode": mode}), (200 if ok else 500)


@socketio.on("connect")
def on_connect():
    emit("connection_response", {"status": "ok", "message": "manual control ready"})
    emit("control_status", arbiter.status())
    emit("steering_calibration", {"calibration": pico.calibration})


@socketio.on("disconnect")
def on_disconnect():
    arbiter.release_remote(request.sid)


@socketio.on("control_claim")
def on_control_claim(_data=None):
    try:
        arbiter.claim_remote(request.sid)
        emit("control_claim_result", {"owned": True})
    except ControlRejected as exc:
        emit("control_claim_result", {"owned": False})
        emit_control_error(str(exc))


@socketio.on("control_release")
def on_control_release(_data=None):
    arbiter.release_remote(request.sid)
    emit("control_claim_result", {"owned": False})


@socketio.on("arm")
def on_arm(_data=None):
    try:
        arbiter.arm("remote", request.sid)
    except ControlRejected as exc:
        emit_control_error(str(exc))


@socketio.on("disarm")
def on_disarm(_data=None):
    try:
        arbiter.disarm("remote", request.sid)
    except ControlRejected as exc:
        emit_control_error(str(exc))


@socketio.on("drive_input")
def on_drive_input(data):
    try:
        arbiter.accept_drive(
            "remote",
            DriveInput.from_payload(data),
            request.sid,
        )
    except (ControlRejected, ValueError) as exc:
        emit_control_error(str(exc))


@socketio.on("emergency_stop")
def on_emergency_stop(_data=None):
    arbiter.emergency_stop()


@socketio.on("emergency_stop_reset")
def on_emergency_stop_reset(_data=None):
    try:
        arbiter.reset_emergency_stop("remote", request.sid)
    except ControlRejected as exc:
        emit_control_error(str(exc))


@socketio.on("battery_read")
def on_battery_read(_data=None):
    try:
        voltage = arbiter.read_battery(BATTERY_DIVIDER_RATIO)
        emit(
            "battery_result",
            {"status": "ok", "voltage": voltage, "timestamp": time.time()},
        )
    except ControlRejected as exc:
        emit("battery_result", {"status": "error", "message": str(exc)})


@socketio.on("steering_calibration_update")
def on_steering_calibration_update(data):
    if arbiter.armed:
        emit_control_error("disarm before calibration")
        return
    try:
        normalized = normalize_steering_calibration(data or {})
        applied = pico.set_steering_calibration(normalized)
        save_steering_calibration(applied)
        emit("steering_calibration", {"calibration": applied})
    except (SteeringCalibrationError, RuntimeError) as exc:
        emit_control_error(str(exc))


def safety_monitor() -> None:
    while True:
        arbiter.tick()
        time.sleep(0.025)


_gamepad_state_lock = threading.Lock()
_gamepad_state = {
    "direction": "N",
    "throttle": 0,
    "steering": 0,
    "brake": False,
    "seq": 0,
    "lt": False,
    "rt": False,
    "last_select": 0.0,
}


def normalize_gamepad_axis(value) -> float:
    try:
        raw = float(value)
    except (TypeError, ValueError):
        return 0.0
    if 0 <= raw <= 255:
        normalized = (raw - 128.0) / 127.0
    else:
        normalized = raw / 32767.0
    if abs(normalized) < 0.08:
        return 0.0
    return max(-1.0, min(1.0, normalized))


def send_gamepad_state() -> None:
    with _gamepad_state_lock:
        _gamepad_state["seq"] += 1
        drive = DriveInput(
            seq=_gamepad_state["seq"],
            direction=str(_gamepad_state["direction"]),
            throttle=int(_gamepad_state["throttle"]),
            steering=int(_gamepad_state["steering"]),
            brake=bool(_gamepad_state["brake"]),
        )
    if not arbiter.armed:
        return
    try:
        arbiter.accept_drive("gamepad", drive)
    except ControlRejected:
        # A direction reversal intentionally stops first. The active gamepad
        # sender retries after the Pico dwell while the stick remains held.
        pass


def gamepad_active_sender() -> None:
    while True:
        with _gamepad_state_lock:
            active = bool(
                _gamepad_state["throttle"] or _gamepad_state["brake"]
            )
        if arbiter.gamepad_connected and arbiter.armed and active:
            send_gamepad_state()
        time.sleep(0.10)


def reset_gamepad_state() -> None:
    with _gamepad_state_lock:
        _gamepad_state.update(
            {
                "direction": "N",
                "throttle": 0,
                "steering": 0,
                "brake": False,
                "lt": False,
                "rt": False,
            }
        )


def gamepad_reader() -> None:
    try:
        from inputs import UnpluggedError, devices, get_gamepad
    except ImportError:
        print("Gamepad support disabled: install the `inputs` package.")
        return

    if devices.gamepads:
        arbiter.set_gamepad_connected(True)

    while True:
        try:
            events = get_gamepad()
            if not arbiter.gamepad_connected:
                reset_gamepad_state()
                arbiter.set_gamepad_connected(True)

            for event in events:
                code = event.code
                state = event.state

                if code == "ABS_Y":
                    signed_throttle = -normalize_gamepad_axis(state)
                    with _gamepad_state_lock:
                        if signed_throttle > 0:
                            _gamepad_state["direction"] = "F"
                        elif signed_throttle < 0:
                            _gamepad_state["direction"] = "R"
                        else:
                            _gamepad_state["direction"] = "N"
                        _gamepad_state["throttle"] = round(abs(signed_throttle) * 100)
                    send_gamepad_state()
                elif code in ("ABS_Z", "ABS_RX"):
                    with _gamepad_state_lock:
                        _gamepad_state["steering"] = round(
                            normalize_gamepad_axis(state) * 50
                        )
                    send_gamepad_state()
                elif code in ("BTN_THUMBL", "BTN_THUMBR"):
                    with _gamepad_state_lock:
                        _gamepad_state["brake"] = bool(state)
                        if state:
                            _gamepad_state["throttle"] = 0
                    send_gamepad_state()
                elif code == "BTN_START" and state == 1:
                    try:
                        if arbiter.armed:
                            arbiter.disarm("gamepad")
                        else:
                            with _gamepad_state_lock:
                                neutral = (
                                    not _gamepad_state["throttle"]
                                    and not _gamepad_state["brake"]
                                )
                            if not neutral:
                                raise ControlRejected(
                                    "release throttle and brake before arming"
                                )
                            arbiter.arm("gamepad")
                            send_gamepad_state()
                    except ControlRejected:
                        pass
                elif code == "BTN_SELECT" and state == 1:
                    now = time.monotonic()
                    with _gamepad_state_lock:
                        previous = float(_gamepad_state["last_select"])
                        _gamepad_state["last_select"] = now
                    if now - previous <= 0.6:
                        with _gamepad_state_lock:
                            _gamepad_state["last_select"] = 0.0
                        arbiter.force_disarm()
                elif code == "BTN_TL2":
                    with _gamepad_state_lock:
                        _gamepad_state["lt"] = bool(state)
                elif code == "BTN_TR2":
                    with _gamepad_state_lock:
                        _gamepad_state["rt"] = bool(state)
                elif code in ("BTN_WEST", "BTN_X") and state == 1:
                    with _gamepad_state_lock:
                        combo = bool(_gamepad_state["lt"] or _gamepad_state["rt"])
                        neutral = not _gamepad_state["throttle"]
                    if combo:
                        if arbiter.estop and neutral:
                            try:
                                arbiter.reset_emergency_stop("gamepad")
                            except ControlRejected:
                                pass
                        else:
                            arbiter.emergency_stop()
        except (UnpluggedError, OSError, RuntimeError, IndexError):
            if arbiter.gamepad_connected:
                reset_gamepad_state()
                arbiter.set_gamepad_connected(False)
            time.sleep(1)
        except Exception as exc:
            if arbiter.gamepad_connected:
                reset_gamepad_state()
                arbiter.set_gamepad_connected(False)
            print(f"Gamepad reader error: {exc}")
            time.sleep(1)


_background_services_started = False


def start_background_services() -> None:
    global _background_services_started
    if _background_services_started:
        return
    _background_services_started = True
    threading.Thread(
        target=safety_monitor,
        daemon=True,
        name="manual-control-watchdog",
    ).start()
    threading.Thread(
        target=gamepad_reader,
        daemon=True,
        name="gamepad-reader",
    ).start()
    threading.Thread(
        target=gamepad_active_sender,
        daemon=True,
        name="gamepad-sender",
    ).start()


def shutdown(_signum=None, _frame=None) -> None:
    arbiter.emergency_stop()
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown)
    start_background_services()
    try:
        socketio.run(
            app,
            host="0.0.0.0",
            port=5000,
            debug=False,
            allow_unsafe_werkzeug=True,
        )
    except KeyboardInterrupt:
        pass
    finally:
        arbiter.emergency_stop()
        pico.close()
