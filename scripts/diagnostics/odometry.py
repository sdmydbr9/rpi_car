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
import csv
import logging
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
from gamepad_axis import (
    build_default_axis_profiles,
    describe_axis_profile,
    load_inputs_axis_profiles,
    normalize_centered_axis,
)
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
_GAMEPAD_AXIS_PROFILES = build_default_axis_profiles(
    center=_GAMEPAD_JOY_CENTER,
    deadzone=_GAMEPAD_JOY_DEADZONE,
)
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


def _refresh_gamepad_axis_profiles():
    global _GAMEPAD_AXIS_PROFILES
    _GAMEPAD_AXIS_PROFILES, source_path = load_inputs_axis_profiles(_GAMEPAD_AXIS_PROFILES)
    if source_path:
        summary = ", ".join(
            describe_axis_profile(axis_code, _GAMEPAD_AXIS_PROFILES[axis_code])
            for axis_code in ("ABS_Y", "ABS_Z")
        )
        print(f"🎮 [Gamepad] Axis calibration from {source_path}: {summary}")

# ── RTH (Return-to-Home) state ──────────────────────────────────
rth_active = False
rth_thread: threading.Thread = None
rth_waypoints = []       # list of (x,y) waypoints for return path
rth_current_wp = 0       # index of current waypoint being pursued
rth_total_wp = 0         # total waypoints
rth_status = "idle"      # idle | navigating | obstacle | arrived | cancelled

# RTH navigation constants
_RTH_WAYPOINT_SPACING_M = 0.30  # thin trail to ~30 cm
_RTH_ARRIVE_DIST_M = 0.20       # 20 cm arrival tolerance per waypoint
_RTH_FINAL_ARRIVE_M = 0.10      # 10 cm for origin
_RTH_NAV_SPEED = 50              # PWM % for normal driving
_RTH_SLOW_SPEED = 40             # PWM % for large heading error
_RTH_STEER_GAIN = 1.8
_RTH_MAX_STEER_DEG = 35
_RTH_OBSTACLE_DIST_MM = 200     # brake if laser < this
_RTH_OBSTACLE_CLEAR_MM = 300    # resume when laser > this
_RTH_STUCK_WINDOW_S = 2.0       # seconds to evaluate stuck condition
_RTH_STUCK_DIST_M = 0.015      # must move at least this much in the window
_RTH_MAX_STUCK_COUNT = 3        # skip waypoint after this many stuck events
_RTH_UTURN_THRESHOLD_DEG = 90   # heading error above this → execute U-turn
_RTH_BREAKAWAY_STEP = 5         # PWM % to add per tick during breakaway ramp
_RTH_BREAKAWAY_MAX = 90         # maximum PWM % during breakaway
_RTH_RPM_MOVING = 3.0           # RPM threshold to consider wheels spinning

# Breakaway state — persists across ticks so ramp doesn't restart each tick
_rth_applied_pwm = 0             # actual PWM being sent (may be higher than target)
_rth_wheels_spinning = False     # True once encoders confirm movement


def _wrap_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def _steer_angle_to_pw(angle_deg):
    """Convert steering angle in degrees to servo pulse width."""
    if angle_deg >= 0:
        return int(STEER_CENTER_PW + (angle_deg / 30.0) * (STEER_RIGHT_PW - STEER_CENTER_PW))
    else:
        return int(STEER_CENTER_PW + (angle_deg / 30.0) * (STEER_CENTER_PW - STEER_LEFT_PW))


def _thin_waypoints(points):
    """Reduce dense trail to waypoints spaced ~10 cm apart."""
    if not points:
        return []
    result = [points[0]]
    for x, y in points[1:]:
        lx, ly = result[-1]
        dx, dy = x - lx, y - ly
        if dx * dx + dy * dy >= _RTH_WAYPOINT_SPACING_M ** 2:
            result.append((x, y))
    return result


def _rth_drive(target_pwm, steer_pw, forward=True):
    """Send drive command with encoder-feedback breakaway ramp.

    If wheels aren't spinning (RPM < threshold), ramp PWM up by
    _RTH_BREAKAWAY_STEP each call until they start moving.
    Once wheels are confirmed spinning, settle back to target_pwm.
    """
    global _rth_applied_pwm, _rth_wheels_spinning
    if not pico:
        return 0

    # Read current encoder RPMs
    packet = pico.get_latest()
    avg_rpm = 0.0
    if packet:
        avg_rpm = (abs(packet.rpm_left) + abs(packet.rpm_right)) / 2.0

    if avg_rpm >= _RTH_RPM_MOVING:
        # Wheels are spinning — use target speed (or ease down from breakaway)
        _rth_wheels_spinning = True
        if _rth_applied_pwm > target_pwm:
            _rth_applied_pwm = max(target_pwm, _rth_applied_pwm - 3)
        else:
            _rth_applied_pwm = target_pwm
    else:
        # Wheels NOT spinning — ramp up to break static friction
        _rth_wheels_spinning = False
        if _rth_applied_pwm < target_pwm:
            _rth_applied_pwm = target_pwm  # start at least at target
        _rth_applied_pwm = min(_RTH_BREAKAWAY_MAX,
                               _rth_applied_pwm + _RTH_BREAKAWAY_STEP)

    pwm = int(max(0, min(100, _rth_applied_pwm)))
    pico.send_lr_pwm(pwm, pwm, steer_pw, forward=forward)
    return pwm


def _rth_reset_drive_state():
    """Reset breakaway ramp state (call after brake/stop)."""
    global _rth_applied_pwm, _rth_wheels_spinning
    _rth_applied_pwm = 0
    _rth_wheels_spinning = False


def _rth_thread_func():
    """RTH navigation thread: follow reversed trail back to origin with laser avoidance."""
    global rth_active, rth_waypoints, rth_current_wp, rth_total_wp, rth_status
    global current_steer_pw

    # ── Open CSV log file ────────────────────────────────────────
    _log_dir = os.path.join(_script_dir, '..', '..', 'rover_logs')
    os.makedirs(_log_dir, exist_ok=True)
    _ts = time.strftime('%Y%m%d_%H%M%S')
    _log_path = os.path.join(_log_dir, f'rth_log_{_ts}.csv')
    _csv_file = open(_log_path, 'w', newline='', buffering=1)  # line-buffered
    _csv = csv.writer(_csv_file)
    _csv.writerow([
        'elapsed_s', 'tick', 'event',
        'wp_idx', 'wp_total', 'wp_target_x', 'wp_target_y',
        'pos_x', 'pos_y', 'heading_deg', 'v_linear', 'v_angular',
        'dist_to_wp', 'desired_hdg', 'hdg_error', 'steer_deg', 'steer_pw', 'cmd_speed',
        'rpm_l', 'rpm_r', 'enc_l', 'enc_r', 'duty_l', 'duty_r',
        'gyro_z', 'accel_x', 'accel_y',
        'mag_x', 'mag_y', 'compass_deg',
        'laser_mm', 'temp_c', 'batt_adc',
        'stuck_count', 'moved_cm', 'note',
    ])
    _t0 = time.monotonic()

    def _log(event, wp_i=0, wp_x=0, wp_y=0, dist=0, desired=0, error=0,
             steer=0, steer_p=STEER_CENTER_PW, spd=0, pkt=None,
             stuck_c=0, moved=0, note=''):
        """Write one row to the CSV log."""
        cx, cy = get_position()
        hdg = get_heading_deg()
        vl = get_linear_velocity()
        va = get_angular_velocity()
        r_l = r_r = enc_l = enc_r = d_l = d_r = 0.0
        gz = ax = ay = mx = my = 0.0
        comp = laser = temp = batt = 0
        if pkt:
            r_l, r_r = pkt.rpm_left, pkt.rpm_right
            enc_l, enc_r = pkt.enc_left_steps, pkt.enc_right_steps
            d_l, d_r = pkt.mot_duty_left, pkt.mot_duty_right
            gz = pkt.gyro_z
            ax, ay = pkt.accel_x, pkt.accel_y
            mx, my = pkt.mag_x, pkt.mag_y
            laser = pkt.laser_mm
            temp = pkt.temp_c
            batt = pkt.adc_a0
            comp = compute_compass_heading(mx, my) or 0
        _csv.writerow([
            f'{time.monotonic()-_t0:.3f}', tick_counter[0], event,
            wp_i, rth_total_wp, f'{wp_x:.4f}', f'{wp_y:.4f}',
            f'{cx:.4f}', f'{cy:.4f}', f'{hdg:.1f}', f'{vl:.4f}', f'{va:.4f}',
            f'{dist:.4f}', f'{desired:.1f}', f'{error:.1f}', f'{steer:.1f}', steer_p, spd,
            f'{r_l:.1f}', f'{r_r:.1f}', enc_l, enc_r, f'{d_l:.1f}', f'{d_r:.1f}',
            f'{gz:.2f}', f'{ax:.3f}', f'{ay:.3f}',
            f'{mx:.0f}', f'{my:.0f}', f'{comp:.1f}',
            laser, f'{temp:.1f}', batt,
            stuck_c, f'{moved:.2f}', note,
        ])

    tick_counter = [0]  # mutable for nested scope
    print(f"📝 [RTH] Logging to {_log_path}")

    try:
        # Build waypoint list from current trail
        trail_points = list(trail)
        _log('INIT', note=f'trail_points={len(trail_points)}')

        if len(trail_points) < 2:
            waypoints = [(0.0, 0.0)]
            _log('INIT', note='no_trail, direct_to_origin')
        else:
            trail_points.reverse()
            waypoints = _thin_waypoints(trail_points)
            # Always end at origin
            lx, ly = waypoints[-1]
            if math.sqrt(lx * lx + ly * ly) > _RTH_FINAL_ARRIVE_M:
                waypoints.append((0.0, 0.0))

        rth_waypoints = waypoints
        rth_total_wp = len(waypoints)
        rth_current_wp = 0
        rth_status = "navigating"

        # Log all waypoints for reference
        for wi, (wwx, wwy) in enumerate(waypoints):
            _log('WAYPOINT_PLAN', wp_i=wi, wp_x=wwx, wp_y=wwy,
                 note=f'planned_{wi+1}_of_{rth_total_wp}')

        print(f"🏠 [RTH] Following {rth_total_wp} waypoints back to start")
        _rth_reset_drive_state()

        wp_idx = 0
        while wp_idx < len(waypoints):
            if not rth_active:
                rth_status = "cancelled"
                _log('CANCELLED', wp_i=wp_idx, wp_x=waypoints[wp_idx][0],
                     wp_y=waypoints[wp_idx][1],
                     note=f'user_cancel_at_wp_{wp_idx}')
                print(f"🏠 [RTH] Cancelled at waypoint {wp_idx}/{rth_total_wp}")
                break

            wx, wy = waypoints[wp_idx]
            rth_current_wp = wp_idx
            is_final = (wp_idx == len(waypoints) - 1)
            arrive_dist = _RTH_FINAL_ARRIVE_M if is_final else _RTH_ARRIVE_DIST_M

            # ── Look-ahead: skip to the furthest waypoint we're already close to ──
            if not is_final:
                best_skip = wp_idx
                cx, cy = get_position()
                for j in range(wp_idx + 1, len(waypoints)):
                    sjx, sjy = waypoints[j]
                    dj = math.sqrt((sjx - cx)**2 + (sjy - cy)**2)
                    if dj < _RTH_ARRIVE_DIST_M:
                        best_skip = j
                if best_skip > wp_idx:
                    _log('WP_LOOKAHEAD_SKIP', wp_i=wp_idx,
                         wp_x=wx, wp_y=wy,
                         note=f'skip_{wp_idx}_to_{best_skip}')
                    print(f"🏠 [RTH] Look-ahead: skip wp {wp_idx}→{best_skip}")
                    wp_idx = best_skip
                    continue

            cx, cy = get_position()
            dx = wx - cx; dy = wy - cy
            dist_initial = math.sqrt(dx*dx + dy*dy)
            _log('WP_START', wp_i=wp_idx, wp_x=wx, wp_y=wy, dist=dist_initial,
                 note=f'arrive_tol={arrive_dist:.3f} final={is_final}')

            # Check if we need a U-turn first (waypoint is behind us)
            # Only for the final waypoint (origin) — intermediate ones use look-ahead
            desired_deg = math.degrees(math.atan2(dy, dx))
            current_deg = get_heading_deg()
            initial_error = abs(_wrap_angle_deg(desired_deg - current_deg))
            if is_final and initial_error > _RTH_UTURN_THRESHOLD_DEG and rth_active:
                _log('UTURN_START', wp_i=wp_idx, wp_x=wx, wp_y=wy,
                     desired=desired_deg, error=initial_error,
                     note=f'hdg_err={initial_error:.0f} > {_RTH_UTURN_THRESHOLD_DEG}')
                # Determine turn direction: steer toward the waypoint
                turn_error = _wrap_angle_deg(desired_deg - current_deg)
                steer_dir = STEER_LEFT_PW if turn_error < 0 else STEER_RIGHT_PW

                # Drive forward with full lock steering to arc around
                uturn_ticks = 0
                for _ in range(120):  # max ~6s
                    if not rth_active:
                        break
                    # Check obstacle
                    p = pico.get_latest() if pico else None
                    if p and 0 < p.laser_mm < _RTH_OBSTACLE_DIST_MM:
                        if pico:
                            pico.send_brake()
                        _rth_reset_drive_state()
                        time.sleep(0.5)
                        continue
                    _rth_drive(_RTH_NAV_SPEED, steer_dir, forward=True)
                    time.sleep(0.05)
                    uturn_ticks += 1
                    # Recalculate desired heading from current position
                    ux, uy = get_position()
                    fresh_desired = math.degrees(math.atan2(wy - uy, wx - ux))
                    new_hdg = get_heading_deg()
                    new_err = abs(_wrap_angle_deg(fresh_desired - new_hdg))
                    if new_err < 45:
                        break
                if pico:
                    pico.send_brake()
                _rth_reset_drive_state()
                time.sleep(0.1)
                _log('UTURN_DONE', wp_i=wp_idx, wp_x=wx, wp_y=wy,
                     desired=desired_deg, error=abs(_wrap_angle_deg(
                         desired_deg - get_heading_deg())),
                     note=f'uturn_ticks={uturn_ticks}')

            # Navigate to this waypoint
            max_ticks = 600  # ~12s at 50Hz
            stuck_count = 0
            stuck_check_time = time.monotonic()
            stuck_check_pos = get_position()
            wp_arrived = False
            for tick in range(max_ticks):
                tick_counter[0] += 1
                if not rth_active:
                    _log('CANCELLED', wp_i=wp_idx, wp_x=wx, wp_y=wy, note='mid_nav_cancel')
                    break

                # Get fresh sensor packet
                packet = pico.get_latest() if pico else None

                # Check laser for obstacles
                if packet and 0 < packet.laser_mm < _RTH_OBSTACLE_DIST_MM:
                    rth_status = "obstacle"
                    _log('OBSTACLE_START', wp_i=wp_idx, wp_x=wx, wp_y=wy, pkt=packet,
                         note=f'laser={packet.laser_mm}mm < {_RTH_OBSTACLE_DIST_MM}mm')
                    if pico:
                        pico.send_brake()
                    _rth_reset_drive_state()
                    obs_deadline = time.monotonic() + 10.0
                    while rth_active and time.monotonic() < obs_deadline:
                        time.sleep(0.1)
                        p = pico.get_latest() if pico else None
                        if p and (p.laser_mm <= 0 or p.laser_mm >= _RTH_OBSTACLE_CLEAR_MM):
                            _log('OBSTACLE_CLEAR', wp_i=wp_idx, wp_x=wx, wp_y=wy, pkt=p,
                                 note=f'laser={p.laser_mm}mm')
                            break
                    else:
                        if rth_active:
                            _log('OBSTACLE_TIMEOUT', wp_i=wp_idx, wp_x=wx, wp_y=wy,
                                 note='10s_timeout')
                    if not rth_active:
                        break
                    rth_status = "navigating"
                    _rth_reset_drive_state()
                    stuck_check_time = time.monotonic()
                    stuck_check_pos = get_position()

                cx, cy = get_position()
                dx = wx - cx
                dy = wy - cy
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < arrive_dist:
                    _log('WP_ARRIVED', wp_i=wp_idx, wp_x=wx, wp_y=wy, dist=dist, pkt=packet,
                         note=f'ticks={tick}')
                    wp_arrived = True
                    break  # arrived at waypoint

                # ── Look-ahead during navigation: skip if a later wp is closer ──
                if not is_final and tick % 5 == 0:
                    best_skip = wp_idx
                    for j in range(wp_idx + 1, len(waypoints)):
                        sjx, sjy = waypoints[j]
                        dj = math.sqrt((sjx - cx)**2 + (sjy - cy)**2)
                        if dj < _RTH_ARRIVE_DIST_M:
                            best_skip = j
                    if best_skip > wp_idx:
                        _log('WP_LOOKAHEAD_SKIP', wp_i=wp_idx,
                             wp_x=wx, wp_y=wy, dist=dist, pkt=packet,
                             note=f'nav_skip_{wp_idx}_to_{best_skip}')
                        wp_idx = best_skip
                        wp_arrived = True
                        break

                # Stuck detection
                now = time.monotonic()
                moved_cm = 0.0
                if now - stuck_check_time >= _RTH_STUCK_WINDOW_S:
                    sx, sy = stuck_check_pos
                    moved = math.sqrt((cx - sx)**2 + (cy - sy)**2)
                    moved_cm = moved * 100
                    if moved < _RTH_STUCK_DIST_M:
                        stuck_count += 1
                        _log('STUCK', wp_i=wp_idx, wp_x=wx, wp_y=wy, dist=dist, pkt=packet,
                             stuck_c=stuck_count, moved=moved_cm,
                             note=f'moved_{moved_cm:.1f}cm < {_RTH_STUCK_DIST_M*100:.1f}cm')
                        print(f"⚠️  [RTH] Stuck detected ({stuck_count}/{_RTH_MAX_STUCK_COUNT}) "
                              f"— moved only {moved_cm:.1f}cm in {_RTH_STUCK_WINDOW_S}s")
                        if stuck_count >= _RTH_MAX_STUCK_COUNT:
                            _log('WP_SKIPPED', wp_i=wp_idx, wp_x=wx, wp_y=wy, dist=dist,
                                 stuck_c=stuck_count, note='max_stuck_reached')
                            print(f"⚠️  [RTH] Skipping waypoint {wp_idx} after {stuck_count} stuck events")
                            if pico:
                                pico.send_brake()
                            _rth_reset_drive_state()
                            time.sleep(0.3)
                            break
                        _log('REVERSE_NUDGE', wp_i=wp_idx, wp_x=wx, wp_y=wy, pkt=packet,
                             note='reverse_ramp_0.8s')
                        _rth_reset_drive_state()
                        for _ in range(16):
                            _rth_drive(50, STEER_CENTER_PW, forward=False)
                            time.sleep(0.05)
                        if pico:
                            pico.send_brake()
                        _rth_reset_drive_state()
                        time.sleep(0.2)
                    else:
                        stuck_count = max(0, stuck_count - 1)
                    stuck_check_time = now
                    stuck_check_pos = (cx, cy)

                # Desired heading toward waypoint
                desired_deg = math.degrees(math.atan2(dy, dx))
                current_deg = get_heading_deg()
                error = _wrap_angle_deg(desired_deg - current_deg)

                # If waypoint is behind us (>120°) and not final, skip it
                if not is_final and abs(error) > 120:
                    _log('WP_BEHIND_SKIP', wp_i=wp_idx, wp_x=wx, wp_y=wy,
                         dist=dist, desired=desired_deg, error=error, pkt=packet,
                         note=f'hdg_err={error:.0f}_skip')
                    break

                # Speed: reduce on large heading error
                if abs(error) < 30:
                    speed = _RTH_NAV_SPEED
                elif abs(error) < 60:
                    speed = _RTH_SLOW_SPEED
                else:
                    speed = _RTH_SLOW_SPEED

                # Proportional steering
                steer_deg = max(-_RTH_MAX_STEER_DEG, min(_RTH_MAX_STEER_DEG, error * _RTH_STEER_GAIN))
                steer_pw = _steer_angle_to_pw(steer_deg)
                steer_pw = max(STEER_LEFT_PW, min(STEER_RIGHT_PW, steer_pw))
                current_steer_pw = steer_pw

                if tick < 5 or tick % 10 == 0:
                    _log('NAV', wp_i=wp_idx, wp_x=wx, wp_y=wy, dist=dist,
                         desired=desired_deg, error=error, steer=steer_deg,
                         steer_p=steer_pw, spd=speed, pkt=packet,
                         stuck_c=stuck_count, moved=moved_cm,
                         note=f'applied={_rth_applied_pwm} spinning={_rth_wheels_spinning}')

                _rth_drive(speed, steer_pw, forward=True)
                time.sleep(0.05)

            # End of waypoint loop
            if not wp_arrived and rth_active:
                _log('WP_TIMEOUT', wp_i=wp_idx, wp_x=wx, wp_y=wy,
                     note=f'max_ticks={max_ticks} stuck={stuck_count}')
            wp_idx += 1

        # Done — stop motors
        if pico:
            pico.send_brake()
        _rth_reset_drive_state()

        if rth_active:
            rth_status = "arrived"
            cx, cy = get_position()
            dist_from_origin = math.sqrt(cx*cx + cy*cy)
            _log('ARRIVED_HOME', dist=dist_from_origin,
                 note=f'final_pos=({cx:.4f},{cy:.4f}) dist_from_origin={dist_from_origin:.4f}m')
            print(f"🏠 [RTH] Arrived at ({cx:.3f}, {cy:.3f})")
        else:
            rth_status = "cancelled"
            _log('FINAL_CANCELLED')

    except Exception as e:
        import traceback
        _log('ERROR', note=f'{type(e).__name__}: {e}')
        print(f"❌ [RTH] Error: {e}")
        traceback.print_exc()
        if pico:
            pico.send_brake()
        _rth_reset_drive_state()
        rth_status = "idle"
    finally:
        rth_active = False
        _log('SHUTDOWN', note=f'status={rth_status}')
        _csv_file.close()
        print(f"📝 [RTH] Log saved → {_log_path}")


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
    global engine_running, rth_active, rth_status
    engine_running = not engine_running
    state = "ON" if engine_running else "OFF"
    print(f"🏎️ Engine {state}")
    if not engine_running:
        if rth_active:
            rth_active = False
            rth_status = "cancelled"
        if pico:
            pico.send_stop()


@socketio.on('engine_off')
def on_engine_off():
    global engine_running, rth_active, rth_status
    engine_running = False
    # Cancel RTH if active
    if rth_active:
        rth_active = False
        rth_status = "cancelled"
    if pico:
        pico.send_stop()


@socketio.on('drive')
def on_drive(data):
    """Drive command from web UI or gamepad relay: {throttle: -100..100, steering: -100..100}"""
    global current_steer_pw
    if not engine_running or not pico or rth_active:
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
        pico.send_lr_pwm(int(speed), int(speed), steer_pw, forward=True)
    else:
        pico.send_lr_pwm(int(speed), int(speed), steer_pw, forward=False)


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


@socketio.on('return_to_home')
def on_return_to_home():
    global rth_active, rth_thread, rth_status
    if rth_active:
        # Cancel RTH
        rth_active = False
        if pico:
            pico.send_brake()
        rth_status = "cancelled"
        print("🏠 [RTH] Cancelled by user")
    else:
        # Start RTH (requires engine to be on)
        if not engine_running:
            print("⚠️  [RTH] Engine must be ON to start RTH")
            return
        rth_active = True
        rth_status = "navigating"
        rth_thread = threading.Thread(target=_rth_thread_func, daemon=True, name="rth-nav")
        rth_thread.start()
        print("🏠 [RTH] Started return-to-home")


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
                _refresh_gamepad_axis_profiles()

            for event in events:
                # Left Stick Y → Throttle
                if event.code == 'ABS_Y':
                    val = event.state
                    max_speed = _GAMEPAD_GEAR_MAP.get(gamepad_gear, 35.0)
                    gamepad_throttle = normalize_centered_axis(
                        val,
                        _GAMEPAD_AXIS_PROFILES["ABS_Y"],
                        output_scale=max_speed,
                        invert=True,
                    )

                # Right Stick X → Steering (ABS_Z on EvoFox-style pads)
                elif event.code == 'ABS_Z':
                    val = event.state
                    gamepad_steering = normalize_centered_axis(
                        val,
                        _GAMEPAD_AXIS_PROFILES["ABS_Z"],
                        output_scale=100.0,
                    )

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
        if engine_running and gamepad_connected and not rth_active:
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
                    pico.send_lr_pwm(int(speed), int(speed), steer_pw, forward=True)
            else:
                if pico:
                    pico.send_lr_pwm(int(speed), int(speed), steer_pw, forward=False)
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
                # RTH state
                'rth_active': rth_active,
                'rth_status': rth_status,
                'rth_current_wp': rth_current_wp,
                'rth_total_wp': rth_total_wp,
                'rth_waypoints': [{'x': round(wx, 3), 'y': round(wy, 3)} for wx, wy in rth_waypoints] if rth_active else [],
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
  .btn-rth-active { background: var(--amber) !important; color: #000 !important; border-color: var(--amber) !important; animation: pulse 1.5s infinite; }
  .btn-rth-obstacle { background: var(--danger) !important; color: #fff !important; border-color: var(--danger) !important; animation: pulse 0.5s infinite; }
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
      <button class="btn" id="btnRTH" onclick="toggleRTH()" style="border-color:var(--amber);color:var(--amber)">🏠 RTH</button>
    </div>
    <div style="margin-top:4px;display:flex;align-items:center;gap:8px">
      <span id="rthStatus" style="font-size:10px;color:var(--muted);display:none"></span>
    </div>
    <div style="margin-top:4px;font-size:9px;color:var(--muted)">
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

  // RTH button & status
  const btnRTH = document.getElementById('btnRTH');
  const rthStatus = document.getElementById('rthStatus');
  if (d.rth_active) {
    if (d.rth_status === 'obstacle') {
      btnRTH.className = 'btn btn-rth-obstacle';
      btnRTH.textContent = '⚠ OBSTACLE';
    } else {
      btnRTH.className = 'btn btn-rth-active';
      btnRTH.textContent = '■ CANCEL RTH';
    }
    rthStatus.style.display = 'inline';
    const progress = d.rth_total_wp > 0 ? `${d.rth_current_wp + 1}/${d.rth_total_wp}` : '';
    const statusText = d.rth_status === 'obstacle' ? '⚠ OBSTACLE — waiting' : `Navigating ${progress}`;
    rthStatus.innerHTML = `🏠 RTH: <span style="color:var(--amber)">${statusText}</span>`;
  } else {
    btnRTH.className = 'btn';
    btnRTH.style.borderColor = 'var(--amber)';
    btnRTH.style.color = 'var(--amber)';
    btnRTH.textContent = '🏠 RTH';
    if (d.rth_status === 'arrived') {
      rthStatus.style.display = 'inline';
      rthStatus.innerHTML = '🏠 <span style="color:var(--primary)">Arrived home ✓</span>';
    } else if (d.rth_status === 'cancelled') {
      rthStatus.style.display = 'inline';
      rthStatus.innerHTML = '🏠 <span style="color:var(--muted)">RTH cancelled</span>';
    } else {
      rthStatus.style.display = 'none';
    }
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

  // RTH waypoint path
  let rthSvg = '';
  const rthWps = d.rth_waypoints || [];
  if (d.rth_active && rthWps.length > 0) {
    // Draw remaining waypoints from current onward
    const remaining = rthWps.slice(d.rth_current_wp || 0);
    if (remaining.length > 0) {
      // Line from rover to first remaining waypoint, then through the rest
      let rthPath = `M${rx.toFixed(1)},${ry.toFixed(1)}`;
      remaining.forEach(p => {
        rthPath += ` L${w2sx(p.x).toFixed(1)},${w2sy(p.y).toFixed(1)}`;
      });
      rthSvg = `<path d="${rthPath}" fill="none" stroke="#f59e0b" stroke-width="2" stroke-dasharray="6 3" opacity="0.7"/>`;
      // Dot on current target waypoint
      if (remaining.length > 0) {
        const tw = remaining[0];
        rthSvg += `<circle cx="${w2sx(tw.x)}" cy="${w2sy(tw.y)}" r="4" fill="#f59e0b" opacity="0.8">
          <animate attributeName="r" values="3;6;3" dur="1s" repeatCount="indefinite"/>
        </circle>`;
      }
    }
  }

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
    <!-- RTH path -->
    ${rthSvg}
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
function toggleRTH() { socket.emit('return_to_home'); }

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
