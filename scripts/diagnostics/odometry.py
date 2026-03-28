#!/usr/bin/env python3
"""
Standalone Odometry Diagnostic Tool
====================================
Single-script diagnostic with web UI on port 5002.
Reads live sensor data from Pico UART, runs fused odometry,
shows an interactive map, raw data logs, and supports gamepad driving.

Usage:
    cd /home/pi/rpi_car/scripts/diagnostics
    python3 odometry.py

Then open http://<pi-ip>:5002 in a browser.
"""

import os
import sys
import math
import json
import time
import threading
import signal
import atexit
from collections import deque
from dataclasses import asdict

# Add core/ to path
_script_dir = os.path.dirname(os.path.abspath(__file__))
_core_dir = os.path.join(_script_dir, '..', 'core')
sys.path.insert(0, _core_dir)

from flask import Flask, Response
from flask_socketio import SocketIO

from pico_sensor_reader import PicoSensorReader, SensorPacket
from odometry_integration import (
    init_odometry, update_odometry, get_position, get_heading_deg,
    get_linear_velocity, get_angular_velocity, get_diagnostics_dict,
    reset_pose,
)


# ── Compass calibration (same logic as main.py) ─────────────────
_COMPASS_CAL_PATH = os.path.join(_script_dir, "compass_cal.json")
_MIN_COMPASS_VECTOR_SQ = 1e-6


def _load_compass_calibration():
    defaults = {"offset_x": 0.0, "offset_y": 0.0, "scale_x": 1.0, "scale_y": 1.0}
    try:
        with open(_COMPASS_CAL_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        cal = {
            "offset_x": float(data.get("offset_x", 0.0)),
            "offset_y": float(data.get("offset_y", 0.0)),
            "scale_x": float(data.get("scale_x", 1.0)) or 1.0,
            "scale_y": float(data.get("scale_y", 1.0)) or 1.0,
        }
        print(f"🧭 Compass calibration loaded: {_COMPASS_CAL_PATH}")
        return cal
    except Exception as exc:
        print(f"⚠️  Using raw heading (no calibration): {exc}")
        return defaults


_COMPASS_CAL = _load_compass_calibration()


def compute_compass_heading(mag_x, mag_y):
    cx = (float(mag_x) - _COMPASS_CAL["offset_x"]) * _COMPASS_CAL["scale_x"]
    cy = (float(mag_y) - _COMPASS_CAL["offset_y"]) * _COMPASS_CAL["scale_y"]
    if (cx * cx + cy * cy) <= _MIN_COMPASS_VECTOR_SQ:
        return None
    heading = -math.degrees(math.atan2(cy, cx)) + 90.0
    return (heading % 360.0 + 360.0) % 360.0


# ── Steering constants (from PicoSensorReader) ──────────────────
STEER_CENTER_PW = 1440
STEER_LEFT_PW = 940
STEER_RIGHT_PW = 2150

# ── Globals ──────────────────────────────────────────────────────
pico: PicoSensorReader = None
trail = deque(maxlen=2000)
raw_log = deque(maxlen=200)  # ring buffer of raw sensor lines for UI
engine_running = False
current_steer_pw = STEER_CENTER_PW  # current steering pulse width

# ── Gamepad state (server-side USB via `inputs` library) ─────────
_GAMEPAD_JOY_CENTER = 128
_GAMEPAD_JOY_DEADZONE = 15
_GAMEPAD_GEAR_MAP = {
    "1":  35.0,
    "2":  60.0,
    "3":  80.0,
    "4": 100.0,
}
gamepad_connected = False
gamepad_throttle = 0.0
gamepad_steering = 0.0
gamepad_gear = "1"
_gp_select_last_press = 0.0
_GP_DOUBLE_PRESS_WINDOW = 0.6

# ── Flask + SocketIO ─────────────────────────────────────────────
app = Flask(__name__)
app.config['SECRET_KEY'] = os.urandom(16).hex()
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


# ── Web UI (single-page, self-contained) ─────────────────────────
@app.route('/')
def index():
    return Response(HTML_PAGE, content_type='text/html')


# ── Socket.IO events ────────────────────────────────────────────
@socketio.on('connect')
def on_connect():
    print("🌐 Client connected")


@socketio.on('reset_odometry')
def on_reset():
    global trail
    reset_pose(0, 0, 0)
    trail.clear()
    print("🔄 Odometry reset")


@socketio.on('engine_toggle')
def on_engine_toggle():
    global engine_running
    engine_running = not engine_running
    state = "ON" if engine_running else "OFF"
    print(f"🏎️ Engine {state}")
    if not engine_running and pico:
        pico.send_stop()


@socketio.on('engine_off')
def on_engine_off():
    global engine_running
    engine_running = False
    if pico:
        pico.send_stop()


@socketio.on('drive')
def on_drive(data):
    """Drive command from web UI or gamepad relay: {throttle: -100..100, steering: -100..100}"""
    global current_steer_pw
    if not engine_running or not pico:
        return
    throttle = max(-100, min(100, float(data.get('throttle', 0))))
    steering = max(-100, min(100, float(data.get('steering', 0))))

    # Map steering % to pulse width
    if steering >= 0:
        steer_pw = STEER_CENTER_PW + (steering / 100.0) * (STEER_RIGHT_PW - STEER_CENTER_PW)
    else:
        steer_pw = STEER_CENTER_PW + (steering / 100.0) * (STEER_CENTER_PW - STEER_LEFT_PW)
    steer_pw = int(max(STEER_LEFT_PW, min(STEER_RIGHT_PW, steer_pw)))
    current_steer_pw = steer_pw

    speed = abs(throttle)
    if speed < 3:
        pico.send_stop()
    elif throttle >= 0:
        pico.send_motor_command(int(speed), steer_pw)
    else:
        pico.send_reverse_command(int(speed), steer_pw)


@socketio.on('brake')
def on_brake():
    if pico:
        pico.send_brake()


@socketio.on('stop')
def on_stop():
    if pico:
        pico.send_stop()


@socketio.on('encoder_reset')
def on_encoder_reset():
    if pico:
        pico.send_encoder_reset()
        print("🔄 Encoder steps reset")


# ── Gamepad reader thread (server-side USB, same as main app) ────
def _gamepad_reader_thread():
    """Read USB gamepad via `inputs` library. Blocks on get_gamepad().
    On any button/stick event the gamepad is considered connected."""
    global gamepad_connected, gamepad_throttle, gamepad_steering
    global gamepad_gear, engine_running, _gp_select_last_press

    try:
        from inputs import get_gamepad as _get_gamepad
    except ImportError:
        print("⚠️  [Gamepad] `inputs` library not installed — gamepad disabled")
        return

    print("🎮 [Gamepad] Reader thread started — waiting for controller…")

    while True:
        try:
            events = _get_gamepad()
            if not gamepad_connected:
                gamepad_connected = True
                print("🎮 [Gamepad] Controller connected!")

            for event in events:
                # Left Stick Y → Throttle
                if event.code == 'ABS_Y':
                    val = event.state
                    if abs(val - _GAMEPAD_JOY_CENTER) < _GAMEPAD_JOY_DEADZONE:
                        gamepad_throttle = 0.0
                    else:
                        max_speed = _GAMEPAD_GEAR_MAP.get(gamepad_gear, 35.0)
                        gamepad_throttle = ((_GAMEPAD_JOY_CENTER - val) / 128.0) * max_speed

                # Right Stick X → Steering (ABS_Z on EvoFox-style pads)
                elif event.code == 'ABS_Z':
                    val = event.state
                    if abs(val - _GAMEPAD_JOY_CENTER) < _GAMEPAD_JOY_DEADZONE:
                        gamepad_steering = 0.0
                    else:
                        gamepad_steering = ((val - _GAMEPAD_JOY_CENTER) / 128.0) * 100.0

                # Face buttons → Gear
                elif event.code in ('BTN_SOUTH', 'BTN_A') and event.state == 1:
                    gamepad_gear = "1"
                    print("🎮 Gear → 1")
                elif event.code in ('BTN_EAST', 'BTN_B') and event.state == 1:
                    gamepad_gear = "2"
                    print("🎮 Gear → 2")
                elif event.code in ('BTN_WEST', 'BTN_X') and event.state == 1:
                    gamepad_gear = "3"
                    print("🎮 Gear → 3")
                elif event.code in ('BTN_NORTH', 'BTN_Y') and event.state == 1:
                    gamepad_gear = "4"
                    print("🎮 Gear → 4 (SPORT)")

                # Start → Engine toggle
                elif event.code == 'BTN_START' and event.state == 1:
                    engine_running = not engine_running
                    state = "ON" if engine_running else "OFF"
                    print(f"🎮 Engine {state}")
                    if not engine_running and pico:
                        pico.send_stop()

                # Select → double-press kill-switch
                elif event.code == 'BTN_SELECT' and event.state == 1:
                    now = time.time()
                    if (now - _gp_select_last_press) < _GP_DOUBLE_PRESS_WINDOW:
                        engine_running = False
                        if pico:
                            pico.send_brake()
                        _gp_select_last_press = 0.0
                        print("🎮 SELECT×2 → ENGINE OFF")
                    else:
                        _gp_select_last_press = now

                # L3/R3 → Brake
                elif event.code in ('BTN_THUMBL', 'BTN_THUMBR') and event.state == 1:
                    if pico:
                        pico.send_brake()
                    print("🎮 Thumb brake")

        except Exception:
            if gamepad_connected:
                gamepad_connected = False
                print("⚠️  [Gamepad] Controller disconnected")
            time.sleep(1)


def _gamepad_drive_thread():
    """50 Hz loop: apply smoothed gamepad inputs to Pico motor commands."""
    global current_steer_pw
    smoothed_throttle = 0.0
    smoothed_steering = 0.0

    while True:
        if engine_running and gamepad_connected:
            smoothed_throttle = 0.4 * gamepad_throttle + 0.6 * smoothed_throttle
            smoothed_steering = gamepad_steering  # no smoothing for instant response

            # Map steering % to pulse width
            if smoothed_steering >= 0:
                steer_pw = STEER_CENTER_PW + (smoothed_steering / 100.0) * (STEER_RIGHT_PW - STEER_CENTER_PW)
            else:
                steer_pw = STEER_CENTER_PW + (smoothed_steering / 100.0) * (STEER_CENTER_PW - STEER_LEFT_PW)
            steer_pw = int(max(STEER_LEFT_PW, min(STEER_RIGHT_PW, steer_pw)))
            current_steer_pw = steer_pw

            speed = abs(smoothed_throttle)
            if speed < 3:
                if pico:
                    pico.send_stop()
            elif smoothed_throttle >= 0:
                if pico:
                    pico.send_motor_command(int(speed), steer_pw)
            else:
                if pico:
                    pico.send_reverse_command(int(speed), steer_pw)
        elif not engine_running:
            smoothed_throttle = 0.0
            smoothed_steering = 0.0

        time.sleep(0.02)  # 50 Hz


# ── Odometry + Telemetry loop (50 Hz) ───────────────────────────
_odom_error_count = 0

def telemetry_loop():
    global _odom_error_count
    while True:
        try:
            packet = pico.get_latest() if pico else None
            if packet is None:
                socketio.emit('telemetry', {
                    'connected': False,
                    'x': 0, 'y': 0, 'heading': 0,
                    'v_linear': 0, 'v_angular': 0,
                    'active': False,
                    'trail': [],
                    'raw_log': list(raw_log),
                })
                time.sleep(0.1)
                continue

            # Compute compass heading
            compass = compute_compass_heading(packet.mag_x, packet.mag_y)

            # Estimate steering angle from pulse width
            steer_frac = (current_steer_pw - STEER_CENTER_PW)
            if steer_frac >= 0:
                steer_angle = (steer_frac / (STEER_RIGHT_PW - STEER_CENTER_PW)) * 30.0
            else:
                steer_angle = (steer_frac / (STEER_CENTER_PW - STEER_LEFT_PW)) * 30.0

            # Update odometry (isolated — UKF can throw LinAlgError)
            odom_active = False
            x_m, y_m, heading_deg, v_lin, v_ang = 0.0, 0.0, 0.0, 0.0, 0.0
            diag = {}
            try:
                odom_state = update_odometry(
                    rpm_left=packet.rpm_left,
                    rpm_right=packet.rpm_right,
                    gyro_z_deg_s=packet.gyro_z,
                    mag_heading_deg=compass,
                    steering_angle_deg=steer_angle,
                    accel_x=packet.accel_x,
                )
                x_m, y_m = get_position()
                heading_deg = get_heading_deg()
                v_lin = get_linear_velocity()
                v_ang = get_angular_velocity()
                odom_active = True
                _odom_error_count = 0
                diag = get_diagnostics_dict()
            except Exception as odom_err:
                _odom_error_count += 1
                if _odom_error_count == 1 or _odom_error_count % 50 == 0:
                    print(f"⚠️  Odometry error (#{_odom_error_count}): {odom_err}")
                # After 10 consecutive failures, re-init the filter
                if _odom_error_count >= 10:
                    try:
                        reset_pose(0, 0, 0)
                        trail.clear()
                        _odom_error_count = 0
                        print("🔄 Odometry filter reset (matrix recovery)")
                    except Exception:
                        pass

            # Trail accumulation (only when odom is active)
            if odom_active and (
                len(trail) == 0 or (
                    (x_m - trail[-1][0])**2 + (y_m - trail[-1][1])**2 > 0.0009
                )
            ):
                trail.append((x_m, y_m))

            # Raw log entry (always — shows sensor data even when odom fails)
            batt_v = (packet.adc_a0 / 1000.0) * 5.0 if packet.adc_a0 > 0 else -1
            raw_entry = {
                't': time.strftime('%H:%M:%S'),
                'rpm_l': round(packet.rpm_left, 1),
                'rpm_r': round(packet.rpm_right, 1),
                'gyro_z': round(packet.gyro_z, 2),
                'mag': f"({packet.mag_x:.0f},{packet.mag_y:.0f},{packet.mag_z:.0f})",
                'compass': round(compass, 1) if compass else '-',
                'laser_mm': packet.laser_mm,
                'enc_l': packet.enc_left_steps,
                'enc_r': packet.enc_right_steps,
                'accel': f"({packet.accel_x:.2f},{packet.accel_y:.2f},{packet.accel_z:.2f})",
                'batt_v': round(batt_v, 2) if batt_v > 0 else '-',
                'duty_l': round(packet.mot_duty_left, 1),
                'duty_r': round(packet.mot_duty_right, 1),
                'temp_c': round(packet.temp_c, 1),
                'x': round(x_m, 4),
                'y': round(y_m, 4),
                'hdg': round(heading_deg, 1),
            }
            raw_log.append(raw_entry)

            # Pico connection stats
            pico_stats = pico.get_stats() if pico else {}

            telemetry = {
                'connected': True,
                'x': round(x_m, 4),
                'y': round(y_m, 4),
                'heading': round(heading_deg, 1),
                'v_linear': round(v_lin, 4),
                'v_angular': round(v_ang, 4),
                'active': odom_active,
                'engine': engine_running,
                'trail': [{'x': p[0], 'y': p[1]} for p in trail],
                'raw_log': list(raw_log)[-50:],  # last 50 entries
                # Sensor raw
                'rpm_left': round(packet.rpm_left, 1),
                'rpm_right': round(packet.rpm_right, 1),
                'gyro_z': round(packet.gyro_z, 2),
                'compass': round(compass, 1) if compass else None,
                'laser_mm': packet.laser_mm,
                'battery_v': round(batt_v, 2) if batt_v > 0 else -1,
                'temp_c': round(packet.temp_c, 1),
                'enc_left': packet.enc_left_steps,
                'enc_right': packet.enc_right_steps,
                'duty_left': round(packet.mot_duty_left, 1),
                'duty_right': round(packet.mot_duty_right, 1),
                # Pico stats
                'pico_packets': pico_stats.get('packets_received', 0),
                'pico_errors': pico_stats.get('errors', 0),
                'pico_health': pico.get_connection_health() if pico else False,
                # Gamepad state
                'gamepad_connected': gamepad_connected,
                'gamepad_gear': gamepad_gear,
                'gamepad_throttle': round(gamepad_throttle, 1),
                'gamepad_steering': round(gamepad_steering, 1),
                # Odometry diagnostics
                'odom_diag': diag,
                'odom_errors': _odom_error_count,
            }
            socketio.emit('telemetry', telemetry)
            time.sleep(0.02)  # 50 Hz

        except Exception as e:
            print(f"❌ Telemetry error: {e}")
            time.sleep(0.05)


# ── Cleanup ──────────────────────────────────────────────────────
def cleanup():
    global engine_running
    engine_running = False
    if pico:
        pico.send_stop()
        pico.close()
    print("\n🛑 Odometry diagnostic stopped")


atexit.register(cleanup)
signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(0)))
signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(0)))


# ── HTML UI ──────────────────────────────────────────────────────
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Odometry Diagnostic</title>
<script src="https://cdn.socket.io/4.7.4/socket.io.min.js"></script>
<style>
  :root {
    --bg: #0a0a0f; --card: #12121a; --border: #1e1e2e;
    --text: #e0e0e8; --muted: #6b6b80; --primary: #22c55e;
    --danger: #ef4444; --amber: #f59e0b; --cyan: #06b6d4;
    --blue: #3b82f6;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Rajdhani','Segoe UI',system-ui,sans-serif;
    background: var(--bg); color: var(--text);
    overflow-x: hidden;
  }
  .header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 8px 16px; background: var(--card); border-bottom: 1px solid var(--border);
  }
  .header h1 { font-size: 16px; font-weight: 700; letter-spacing: 1px; color: var(--primary); }
  .status-dot {
    width: 10px; height: 10px; border-radius: 50%;
    display: inline-block; margin-right: 6px;
  }
  .status-dot.on { background: var(--primary); box-shadow: 0 0 6px var(--primary); }
  .status-dot.off { background: var(--danger); }
  .layout {
    display: grid;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: auto 1fr;
    gap: 8px; padding: 8px;
    height: calc(100vh - 44px);
  }
  .panel {
    background: var(--card); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px; overflow: hidden;
    display: flex; flex-direction: column;
  }
  .panel-title {
    font-size: 11px; font-weight: 700; color: var(--muted);
    letter-spacing: 1.5px; margin-bottom: 6px; text-transform: uppercase;
  }
  /* Map panel */
  .map-panel { grid-column: 1; grid-row: 1 / 3; min-height: 0; }
  .map-container { flex:1; position:relative; min-height:0; overflow:hidden; border-radius:6px; background:#080810; }
  .map-container svg { width:100%; height:100%; touch-action:none; user-select:none; }

  /* Telemetry panel */
  .telem-panel { grid-column: 2; grid-row: 1; }
  .telem-grid {
    display: grid; grid-template-columns: repeat(3, 1fr);
    gap: 6px;
  }
  .telem-cell {
    background: rgba(255,255,255,0.03); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 8px; text-align: center;
  }
  .telem-label { font-size: 9px; color: var(--muted); letter-spacing: 1px; }
  .telem-value { font-size: 16px; font-weight: 700; font-variant-numeric: tabular-nums; }
  .telem-unit { font-size: 9px; color: var(--muted); margin-left: 2px; }
  .v-primary { color: var(--primary); }
  .v-cyan { color: var(--cyan); }
  .v-amber { color: var(--amber); }
  .v-danger { color: var(--danger); }
  .v-blue { color: var(--blue); }

  /* Controls */
  .controls { display:flex; gap:6px; margin-top:8px; flex-wrap:wrap; }
  .btn {
    padding: 6px 14px; border-radius: 6px; border: 1px solid var(--border);
    background: rgba(255,255,255,0.04); color: var(--text);
    font-size: 11px; font-weight: 600; cursor: pointer;
    letter-spacing: 0.5px; transition: all 0.15s;
  }
  .btn:hover { background: rgba(255,255,255,0.08); }
  .btn-primary { border-color: var(--primary); color: var(--primary); }
  .btn-primary:hover { background: rgba(34,197,94,0.15); }
  .btn-danger { border-color: var(--danger); color: var(--danger); }
  .btn-danger:hover { background: rgba(239,68,68,0.15); }
  .btn-active { background: var(--primary) !important; color: #000 !important; border-color: var(--primary); }
  .btn-engine-on { background: var(--primary) !important; color: #000 !important; animation: pulse 1.5s infinite; }
  @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.7; } }

  /* Raw data log */
  .log-panel { grid-column: 2; grid-row: 2; min-height:0; }
  .log-container {
    flex: 1; overflow-y: auto; overflow-x: auto;
    font-family: 'JetBrains Mono','Fira Code',monospace;
    font-size: 10px; line-height: 1.5; min-height: 0;
    scrollbar-width: thin; scrollbar-color: var(--border) transparent;
  }
  .log-container::-webkit-scrollbar { width: 5px; }
  .log-container::-webkit-scrollbar-track { background: transparent; }
  .log-container::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }
  .log-table { border-collapse: collapse; width: 100%; }
  .log-table th {
    text-align: left; padding: 2px 6px; font-size: 8px;
    color: var(--muted); border-bottom: 1px solid var(--border);
    position: sticky; top: 0; background: var(--card); z-index: 1;
  }
  .log-table td {
    padding: 1px 6px; white-space: nowrap; border-bottom: 1px solid rgba(255,255,255,0.02);
  }
  .log-time { color: var(--muted); }
  .log-val { color: var(--cyan); }
  .log-pos { color: var(--primary); }

  /* Gamepad indicator */
  .gamepad-badge {
    display: inline-flex; align-items: center; gap: 4px;
    padding: 3px 8px; border-radius: 4px; font-size: 10px;
    border: 1px solid var(--border);
  }
  .gamepad-badge.connected { border-color: var(--primary); color: var(--primary); }
  .gamepad-badge.disconnected { border-color: var(--muted); color: var(--muted); }

  /* Responsive */
  @media (max-width: 800px) {
    .layout {
      grid-template-columns: 1fr;
      grid-template-rows: 300px auto 250px;
    }
    .map-panel { grid-column: 1; grid-row: 1; }
    .telem-panel { grid-column: 1; grid-row: 2; }
    .log-panel { grid-column: 1; grid-row: 3; }
  }
</style>
</head>
<body>

<div class="header">
  <div style="display:flex;align-items:center;gap:10px">
    <h1>⊕ ODOMETRY DIAGNOSTIC</h1>
    <span class="status-dot" id="connDot"></span>
    <span id="connLabel" style="font-size:11px;color:var(--muted)">Connecting…</span>
  </div>
  <div style="display:flex;align-items:center;gap:10px">
    <span class="gamepad-badge disconnected" id="gpBadge">🎮 <span id="gpLabel">No Gamepad</span></span>
    <span style="font-size:10px;color:var(--muted);margin-left:4px" id="gpGear"></span>
    <span style="font-size:10px;color:var(--muted)" id="picoStats"></span>
  </div>
</div>

<div class="layout">
  <!-- MAP -->
  <div class="panel map-panel">
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:4px">
      <span class="panel-title" style="margin-bottom:0">Odometry Map</span>
      <div style="display:flex;gap:4px;align-items:center">
        <button class="btn btn-primary" onclick="toggleAutoFollow()" id="btnFollow" style="padding:3px 8px;font-size:10px">📍 Follow</button>
        <button class="btn" onclick="handleRecenter()" style="padding:3px 8px;font-size:10px">⊕ Center</button>
        <span style="font-size:9px;color:var(--muted)" id="scaleLabel">grid: 0.5m</span>
      </div>
    </div>
    <div class="map-container" id="mapContainer">
      <svg id="mapSvg" viewBox="0 0 500 500"></svg>
    </div>
  </div>

  <!-- TELEMETRY + CONTROLS -->
  <div class="panel telem-panel">
    <span class="panel-title">Telemetry</span>
    <div class="telem-grid">
      <div class="telem-cell">
        <div class="telem-label">X POS</div>
        <div class="telem-value v-primary" id="valX">0.00<span class="telem-unit">m</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">Y POS</div>
        <div class="telem-value v-primary" id="valY">0.00<span class="telem-unit">m</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">HEADING</div>
        <div class="telem-value v-cyan" id="valHdg">0.0<span class="telem-unit">°</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">SPEED</div>
        <div class="telem-value v-amber" id="valSpeed">0<span class="telem-unit">cm/s</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">DISTANCE</div>
        <div class="telem-value v-blue" id="valDist">0.00<span class="telem-unit">m</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">COMPASS</div>
        <div class="telem-value v-cyan" id="valCompass">-<span class="telem-unit">°</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">RPM L / R</div>
        <div class="telem-value" id="valRpm" style="font-size:13px">0 / 0</div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">LASER</div>
        <div class="telem-value v-amber" id="valLaser">-<span class="telem-unit">mm</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">BATTERY</div>
        <div class="telem-value" id="valBatt" style="font-size:13px">-<span class="telem-unit">V</span></div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">ENC L</div>
        <div class="telem-value" id="valEncL" style="font-size:13px">0</div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">ENC R</div>
        <div class="telem-value" id="valEncR" style="font-size:13px">0</div>
      </div>
      <div class="telem-cell">
        <div class="telem-label">TEMP</div>
        <div class="telem-value" id="valTemp" style="font-size:13px">-<span class="telem-unit">°C</span></div>
      </div>
    </div>
    <div class="controls">
      <button class="btn btn-primary" id="btnEngine" onclick="toggleEngine()">▶ ENGINE</button>
      <button class="btn btn-danger" onclick="emergencyStop()">⛔ STOP</button>
      <button class="btn" onclick="resetOdometry()">🔄 RESET ODOM</button>
      <button class="btn" onclick="resetEncoders()">↺ RESET ENC</button>
    </div>
    <div style="margin-top:6px;font-size:9px;color:var(--muted)">
      🎮 Gamepad: Left stick = Throttle | Right stick = Steering | Start = Engine | Select×2 = Kill
    </div>
  </div>

  <!-- RAW DATA LOG -->
  <div class="panel log-panel">
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:4px">
      <span class="panel-title" style="margin-bottom:0">Raw Sensor Log</span>
      <label style="font-size:9px;color:var(--muted);display:flex;align-items:center;gap:4px">
        <input type="checkbox" id="chkAutoScroll" checked style="accent-color:var(--primary)"> Auto-scroll
      </label>
    </div>
    <div class="log-container" id="logContainer">
      <table class="log-table">
        <thead>
          <tr>
            <th>TIME</th><th>RPM L</th><th>RPM R</th><th>GYRO Z</th>
            <th>COMPASS</th><th>LASER</th><th>ENC L</th><th>ENC R</th>
            <th>DUTY L</th><th>DUTY R</th><th>X</th><th>Y</th><th>HDG</th>
          </tr>
        </thead>
        <tbody id="logBody"></tbody>
      </table>
    </div>
  </div>
</div>

<script>
// ── Socket.IO ──────────────────────────────────────────────────
const socket = io();
let latestData = null;

// ── Map state ──────────────────────────────────────────────────
const MAP_SIZE = 500;
let autoFollow = true;
let mapScale = 75; // px per meter
const MIN_SCALE = 10, MAX_SCALE = 400;
let panOffset = {x:0, y:0};
let isPanning = false, panStart = {x:0,y:0};
let engineOn = false;

// ── Socket events ──────────────────────────────────────────────
socket.on('connect', () => {
  document.getElementById('connDot').className = 'status-dot on';
  document.getElementById('connLabel').textContent = 'Socket Connected';
});
socket.on('disconnect', () => {
  document.getElementById('connDot').className = 'status-dot off';
  document.getElementById('connLabel').textContent = 'Disconnected';
});

socket.on('telemetry', (data) => {
  latestData = data;
  engineOn = data.engine || false;
  updateTelemetry(data);
  updateMap(data);
  updateLog(data);
  updateConnectionStatus(data);
});

// ── Telemetry display ──────────────────────────────────────────
function updateTelemetry(d) {
  document.getElementById('valX').innerHTML = d.x.toFixed(3) + '<span class="telem-unit">m</span>';
  document.getElementById('valY').innerHTML = d.y.toFixed(3) + '<span class="telem-unit">m</span>';
  document.getElementById('valHdg').innerHTML = d.heading.toFixed(1) + '<span class="telem-unit">°</span>';
  document.getElementById('valSpeed').innerHTML = (d.v_linear * 100).toFixed(0) + '<span class="telem-unit">cm/s</span>';
  const dist = Math.sqrt(d.x*d.x + d.y*d.y);
  document.getElementById('valDist').innerHTML = dist.toFixed(3) + '<span class="telem-unit">m</span>';
  document.getElementById('valCompass').innerHTML = (d.compass != null ? d.compass.toFixed(1) : '-') + '<span class="telem-unit">°</span>';
  document.getElementById('valRpm').textContent = d.rpm_left + ' / ' + d.rpm_right;
  document.getElementById('valLaser').innerHTML = d.laser_mm + '<span class="telem-unit">mm</span>';
  const battEl = document.getElementById('valBatt');
  if (d.battery_v > 0) {
    battEl.innerHTML = d.battery_v.toFixed(2) + '<span class="telem-unit">V</span>';
    battEl.className = 'telem-value ' + (d.battery_v < 6.5 ? 'v-danger' : d.battery_v < 7.0 ? 'v-amber' : 'v-primary');
  } else {
    battEl.innerHTML = '-<span class="telem-unit">V</span>';
  }
  document.getElementById('valEncL').textContent = d.enc_left;
  document.getElementById('valEncR').textContent = d.enc_right;
  document.getElementById('valTemp').innerHTML = d.temp_c.toFixed(1) + '<span class="telem-unit">°C</span>';

  // Engine button state
  const btnEngine = document.getElementById('btnEngine');
  if (d.engine) {
    btnEngine.className = 'btn btn-engine-on';
    btnEngine.textContent = '■ ENGINE ON';
  } else {
    btnEngine.className = 'btn btn-primary';
    btnEngine.textContent = '▶ ENGINE';
  }

  // Pico stats
  document.getElementById('picoStats').textContent =
    `Pico: ${d.pico_packets} pkts, ${d.pico_errors} err` + (d.pico_health ? ' ✓' : ' ⚠');

  // Gamepad status (server-side USB)
  const gpBadge = document.getElementById('gpBadge');
  const gpLabel = document.getElementById('gpLabel');
  const gpGear = document.getElementById('gpGear');
  if (d.gamepad_connected) {
    gpBadge.className = 'gamepad-badge connected';
    gpLabel.textContent = `T:${d.gamepad_throttle}% S:${d.gamepad_steering}%`;
    gpGear.textContent = `Gear ${d.gamepad_gear}`;
    gpGear.style.color = 'var(--primary)';
  } else {
    gpBadge.className = 'gamepad-badge disconnected';
    gpLabel.textContent = 'No Gamepad';
    gpGear.textContent = '';
  }
}

function updateConnectionStatus(d) {
  const dot = document.getElementById('connDot');
  const label = document.getElementById('connLabel');
  if (d.connected) {
    dot.className = 'status-dot on';
    label.textContent = 'Pico Connected';
    label.style.color = 'var(--primary)';
  } else {
    dot.className = 'status-dot off';
    label.textContent = 'Pico Disconnected';
    label.style.color = 'var(--danger)';
  }
}

// ── Map rendering ──────────────────────────────────────────────
function updateMap(d) {
  if (!d.active) return;
  const svg = document.getElementById('mapSvg');
  const trail = d.trail || [];
  const cx = autoFollow ? d.x : panOffset.x;
  const cy = autoFollow ? d.y : panOffset.y;

  function w2sx(wx) { return MAP_SIZE/2 + (wx - cx) * mapScale; }
  function w2sy(wy) { return MAP_SIZE/2 - (wy - cy) * mapScale; }

  // Grid
  const gridSpacing = mapScale > 120 ? 0.25 : mapScale > 50 ? 0.5 : 1.0;
  const viewRange = MAP_SIZE / mapScale / 2;
  let gridSvg = '';
  for (let w = Math.floor((cx - viewRange) / gridSpacing) * gridSpacing; w <= cx + viewRange; w += gridSpacing) {
    const sx = w2sx(w);
    const isAxis = Math.abs(w) < 0.001;
    gridSvg += `<line x1="${sx}" y1="0" x2="${sx}" y2="${MAP_SIZE}" stroke="${isAxis ? 'rgba(34,197,94,0.25)' : 'rgba(255,255,255,0.06)'}" stroke-width="${isAxis ? 1 : 0.5}"/>`;
  }
  for (let w = Math.floor((cy - viewRange) / gridSpacing) * gridSpacing; w <= cy + viewRange; w += gridSpacing) {
    const sy = w2sy(w);
    const isAxis = Math.abs(w) < 0.001;
    gridSvg += `<line x1="0" y1="${sy}" x2="${MAP_SIZE}" y2="${sy}" stroke="${isAxis ? 'rgba(34,197,94,0.25)' : 'rgba(255,255,255,0.06)'}" stroke-width="${isAxis ? 1 : 0.5}"/>`;
  }

  // Trail
  let trailPath = '';
  if (trail.length > 0) {
    trailPath = trail.map((p, i) => {
      const sx = w2sx(p.x).toFixed(1), sy = w2sy(p.y).toFixed(1);
      return (i === 0 ? 'M' : 'L') + sx + ',' + sy;
    }).join(' ');
  }

  // Rover arrow
  const rx = w2sx(d.x), ry = w2sy(d.y);
  const hRad = d.heading * Math.PI / 180;
  // Arrow: heading in math convention (0=East), SVG Y-down
  const tipX = rx + 14 * Math.cos(hRad), tipY = ry - 14 * Math.sin(hRad);
  const bLx = rx + 7 * Math.cos(hRad + 2.4), bLy = ry - 7 * Math.sin(hRad + 2.4);
  const bRx = rx + 7 * Math.cos(hRad - 2.4), bRy = ry - 7 * Math.sin(hRad - 2.4);

  // Origin
  const ox = w2sx(0), oy = w2sy(0);

  // Compass rose (heading is math deg: 0=East)
  const compassBearing = ((90 - d.heading) % 360 + 360) % 360;
  const compassR = 36, compassCX = MAP_SIZE - 50, compassCY = 50;
  let compassSvg = `<circle cx="${compassCX}" cy="${compassCY}" r="${compassR}" fill="rgba(0,0,0,0.5)" stroke="rgba(255,255,255,0.15)" stroke-width="1"/>`;
  const dirs = [{l:'N',a:0},{l:'E',a:90},{l:'S',a:180},{l:'W',a:270}];
  dirs.forEach(({l,a}) => {
    const r = (a - compassBearing - 90) * Math.PI / 180;
    const tx = compassCX + Math.cos(r) * (compassR - 10);
    const ty = compassCY + Math.sin(r) * (compassR - 10);
    const tickOuter = compassR - 2, tickInner = compassR - 6;
    compassSvg += `<line x1="${compassCX + Math.cos(r)*tickInner}" y1="${compassCY + Math.sin(r)*tickInner}" x2="${compassCX + Math.cos(r)*tickOuter}" y2="${compassCY + Math.sin(r)*tickOuter}" stroke="${l==='N' ? '#ef4444' : 'rgba(255,255,255,0.5)'}" stroke-width="1.5"/>`;
    compassSvg += `<text x="${tx}" y="${ty}" text-anchor="middle" dominant-baseline="central" fill="${l==='N' ? '#ef4444' : '#22c55e'}" font-size="9" font-weight="700" font-family="Rajdhani,sans-serif">${l}</text>`;
  });
  // Heading indicator
  compassSvg += `<polygon points="${compassCX},${compassCY - compassR + 6} ${compassCX - 4},${compassCY - compassR - 1} ${compassCX + 4},${compassCY - compassR - 1}" fill="var(--primary)"/>`;

  const gridLabel = gridSpacing === 0.25 ? '0.25m' : gridSpacing === 0.5 ? '0.5m' : '1.0m';
  document.getElementById('scaleLabel').textContent = 'grid: ' + gridLabel;

  svg.innerHTML = `
    ${gridSvg}
    <!-- Origin marker -->
    <circle cx="${ox}" cy="${oy}" r="5" fill="none" stroke="rgba(34,197,94,0.5)" stroke-width="1.5"/>
    <circle cx="${ox}" cy="${oy}" r="2" fill="rgba(34,197,94,0.7)"/>
    <text x="${ox+8}" y="${oy-6}" fill="rgba(34,197,94,0.7)" font-size="9" font-family="Rajdhani,sans-serif" font-weight="600">START</text>
    <!-- Trail -->
    ${trailPath ? `
      <path d="${trailPath}" fill="none" stroke="#22c55e" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" opacity="0.8"/>
      <path d="${trailPath}" fill="none" stroke="#22c55e" stroke-width="5" stroke-linecap="round" stroke-linejoin="round" opacity="0.12"/>
    ` : ''}
    <!-- Live segment -->
    ${trail.length > 0 ? `<line x1="${w2sx(trail[trail.length-1].x)}" y1="${w2sy(trail[trail.length-1].y)}" x2="${rx}" y2="${ry}" stroke="#22c55e" stroke-width="2" stroke-dasharray="4 2" opacity="0.9"/>` : ''}
    <!-- Rover pulse -->
    <circle cx="${rx}" cy="${ry}" r="10" fill="none" stroke="#22c55e" stroke-width="1" opacity="0.3">
      <animate attributeName="r" values="8;16;8" dur="2s" repeatCount="indefinite"/>
      <animate attributeName="opacity" values="0.3;0.08;0.3" dur="2s" repeatCount="indefinite"/>
    </circle>
    <!-- Rover arrow -->
    <polygon points="${tipX},${tipY} ${bLx},${bLy} ${bRx},${bRy}" fill="#22c55e" stroke="rgba(255,255,255,0.4)" stroke-width="0.8"/>
    <circle cx="${rx}" cy="${ry}" r="3" fill="white"/>
    <!-- Compass rose -->
    ${compassSvg}
    <!-- Scale -->
    <text x="${MAP_SIZE-8}" y="${MAP_SIZE-8}" text-anchor="end" fill="rgba(255,255,255,0.3)" font-size="8" font-family="Rajdhani,sans-serif">${gridLabel}</text>
  `;
}

// ── Map interactions ───────────────────────────────────────────
const mapContainer = document.getElementById('mapContainer');
mapContainer.addEventListener('pointerdown', (e) => {
  if (autoFollow && latestData) {
    panOffset = {x: latestData.x, y: latestData.y};
    autoFollow = false;
    updateFollowBtn();
  }
  isPanning = true;
  panStart = {x: e.clientX, y: e.clientY};
  mapContainer.setPointerCapture(e.pointerId);
});
mapContainer.addEventListener('pointermove', (e) => {
  if (!isPanning) return;
  const dx = (e.clientX - panStart.x) / mapScale;
  const dy = (e.clientY - panStart.y) / mapScale;
  panOffset.x -= dx;
  panOffset.y += dy;
  panStart = {x: e.clientX, y: e.clientY};
});
mapContainer.addEventListener('pointerup', () => { isPanning = false; });
mapContainer.addEventListener('pointerleave', () => { isPanning = false; });
mapContainer.addEventListener('wheel', (e) => {
  e.preventDefault();
  if (autoFollow && latestData) {
    panOffset = {x: latestData.x, y: latestData.y};
    autoFollow = false;
    updateFollowBtn();
  }
  mapScale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, mapScale * (e.deltaY > 0 ? 0.9 : 1.1)));
}, {passive: false});

function toggleAutoFollow() {
  autoFollow = !autoFollow;
  if (autoFollow) panOffset = {x:0, y:0};
  updateFollowBtn();
}
function handleRecenter() {
  autoFollow = true;
  mapScale = 75;
  panOffset = {x:0, y:0};
  updateFollowBtn();
}
function updateFollowBtn() {
  const btn = document.getElementById('btnFollow');
  btn.className = autoFollow ? 'btn btn-active' : 'btn btn-primary';
}

// ── Raw log ────────────────────────────────────────────────────
function updateLog(d) {
  if (!d.raw_log || d.raw_log.length === 0) return;
  const body = document.getElementById('logBody');
  const container = document.getElementById('logContainer');
  const autoScroll = document.getElementById('chkAutoScroll').checked;

  // Rebuild table
  let html = '';
  d.raw_log.forEach(r => {
    html += `<tr>
      <td class="log-time">${r.t}</td>
      <td class="log-val">${r.rpm_l}</td>
      <td class="log-val">${r.rpm_r}</td>
      <td class="log-val">${r.gyro_z}</td>
      <td class="log-val">${r.compass}</td>
      <td class="log-val">${r.laser_mm}</td>
      <td>${r.enc_l}</td>
      <td>${r.enc_r}</td>
      <td>${r.duty_l}</td>
      <td>${r.duty_r}</td>
      <td class="log-pos">${r.x}</td>
      <td class="log-pos">${r.y}</td>
      <td class="log-pos">${r.hdg}</td>
    </tr>`;
  });
  body.innerHTML = html;
  if (autoScroll) container.scrollTop = container.scrollHeight;
}

// ── Controls ───────────────────────────────────────────────────
function toggleEngine() { socket.emit('engine_toggle'); }
function emergencyStop() { socket.emit('engine_off'); socket.emit('stop'); }
function resetOdometry() { socket.emit('reset_odometry'); }
function resetEncoders() { socket.emit('encoder_reset'); }

// Gamepad is handled server-side via USB `inputs` library.
// No browser Gamepad API needed — the Pi reads the controller directly.

// ── Keyboard controls (arrows for testing) ─────────────────────
const keysDown = {};
document.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  keysDown[e.key] = true;
  if (e.key === 'e' || e.key === 'E') toggleEngine();
  if (e.key === ' ') { e.preventDefault(); emergencyStop(); }
  if (e.key === 'r' || e.key === 'R') resetOdometry();
});
document.addEventListener('keyup', (e) => { delete keysDown[e.key]; });

// Keyboard drive loop
setInterval(() => {
  if (!engineOn) return;
  let t = 0, s = 0;
  if (keysDown['ArrowUp']) t = 40;
  if (keysDown['ArrowDown']) t = -40;
  if (keysDown['ArrowLeft']) s = -50;
  if (keysDown['ArrowRight']) s = 50;
  if (t !== 0 || s !== 0) {
    socket.emit('drive', {throttle: t, steering: s});
  }
}, 50);
</script>
</body>
</html>
"""


# ── Main ─────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("  STANDALONE ODOMETRY DIAGNOSTIC")
    print("  Web UI → http://0.0.0.0:5002")
    print("=" * 60)

    # Init Pico UART reader
    pico = PicoSensorReader(port='/dev/ttyS0', baudrate=115200)

    # Init fused odometry engine
    init_odometry()

    # Start telemetry thread
    telem_thread = threading.Thread(target=telemetry_loop, daemon=True)
    telem_thread.start()

    # Start gamepad threads (server-side USB via `inputs` library)
    gp_reader = threading.Thread(target=_gamepad_reader_thread, daemon=True, name="gamepad-reader")
    gp_reader.start()
    gp_driver = threading.Thread(target=_gamepad_drive_thread, daemon=True, name="gamepad-driver")
    gp_driver.start()

    print("✅ Telemetry loop started (50 Hz)")
    print("🎮 Gamepad: USB controller (server-side via `inputs` library)")
    print("   Start=engine | A/B/X/Y=gear | Left stick=throttle | Right stick=steer")
    print("   Select×2=kill | L3/R3=brake")
    print("⌨️  Keyboard (web): Arrows=drive, E=engine, Space=stop, R=reset")
    print()

    socketio.run(app, host='0.0.0.0', port=5002, allow_unsafe_werkzeug=True)
