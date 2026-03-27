#!/usr/bin/env python3
"""
test_drive.py — Compass-Guided Drive Test (ncurses edition)
============================================================
Uses the QMC5883L magnetometer to lock a compass heading at startup, then
applies a PID correction via the differential steering to keep the car
driving straight.

Two operating modes
───────────────────
  STRAIGHT mode  — drives on the calibrated heading (original behaviour).
  DIRECTION mode — user picks a named cardinal / intercardinal direction or
                   types an exact bearing.  Waypoints can be queued so the
                   car drives North → East → South etc. in sequence.

Usage
─────
    cd /home/pi/rpi_car/scripts/diagnostics
    python3 test_drive.py

Controls (main screen)
──────────────────────
    SPACE    arm / disarm motors  (car starts DISARMED)
    R        re-calibrate heading from current position (locks new heading)
    D        open Direction Picker overlay
    C        clear the waypoint queue
    N        skip to the next waypoint in the queue
    Q / ESC  stop and exit

Direction Picker overlay
────────────────────────
    1-8      select cardinal / intercardinal (N NE E SE S SW W NW)
    0        enter a custom bearing (0-359°) via number row
    +        append selected direction as next queue entry
    ENTER    replace current target with selected direction
    ESC      cancel / close picker
"""

import os
import sys
import time
import math
import curses
import csv
from collections import deque
from datetime import datetime

# ── path so we can import core modules by bare name ──────────────────────────
_DIAG_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR  = os.path.normpath(os.path.join(_DIAG_DIR, '..', '..'))
_LOG_DIR   = os.path.join(_ROOT_DIR, 'rover_logs')
_CAL_FILE  = os.path.join(_DIAG_DIR, 'compass_cal.json')
sys.path.insert(0, os.path.join(_DIAG_DIR, '..', 'core'))

# ── optional GPIO / motor ────────────────────────────────────────────────────
try:
    from motor import CarSystem
    _MOTOR_OK = True
except Exception as _motor_err:
    _MOTOR_OK = False
    print(f"⚠️  Motor import failed: {_motor_err}  — display-only mode")

# ── Pico sensor reader ───────────────────────────────────────────────────────
try:
    from pico_sensor_reader import PicoSensorReader
    _PICO_OK = True
except ImportError as _pico_err:
    _PICO_OK = False
    print(f"⚠️  pico_sensor_reader import failed: {_pico_err}")

from compass_calibration import (
    compute_heading_degrees,
    identity_calibration,
    load_calibration as load_compass_calibration,
)


# ══════════════════════════════════════════════════════════════════════════════
#  CSV LOGGER
# ══════════════════════════════════════════════════════════════════════════════
_LOG_COLUMNS = [
    'timestamp', 'elapsed_s', 'armed',
    'target_heading', 'current_heading', 'heading_error',
    'gyro_z', 'correction',
    'pid_p', 'pid_i_contrib', 'pid_d_contrib', 'integral',
    'mag_x', 'mag_y', 'mag_z',
    'accel_x', 'accel_y', 'accel_z',
    'rpm_rear_right', 'rpm_rear_left', 'rpm_front_right',
    'speed_pct', 'loop_hz',
]


class DriveLogger:
    """Thin wrapper around a CSV writer for the straight-drive test."""

    def __init__(self):
        os.makedirs(_LOG_DIR, exist_ok=True)
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(_LOG_DIR, f'straight_drive_{stamp}.csv')
        self._fh  = open(self.path, 'w', newline='', buffering=1)
        self._w   = csv.DictWriter(self._fh, fieldnames=_LOG_COLUMNS)
        self._w.writeheader()

    def write(self, **kwargs):
        row = {k: kwargs.get(k, '') for k in _LOG_COLUMNS}
        self._w.writerow(row)

    def close(self):
        self._fh.flush()
        self._fh.close()


# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════
DRIVE_SPEED    = 35       # Base PWM % while armed
LOOP_HZ        = 20       # Control loop frequency (Hz)
CALIB_FRAMES   = 40       # Heading samples averaged at startup / re-cal
MAX_STEER_DEG  = 25.0     # Maximum steering correction (degrees)

KP = 0.80                 # Proportional gain  (deg error  → deg steer)
KD = 0.10                 # Derivative gain    (deg/s gyro → deg trim)
KI = 0.10                 # Integral gain      (persistent bias removal)
I_MAX = 20.0              # Anti-windup clamp  — max I contribution = KI×I_MAX = 2°

HEADING_SIGN = -1.0       # -1.0: CW-positive compass — error < 0 means drifted LEFT → steer RIGHT (+)
GYRO_SIGN    =  1.0       # Flip to -1.0 if gyro D-term fights the correction

# ── Magnetometer calibration values ─────────────────────────────────────────
_COMPASS_CAL = identity_calibration()

# ── Direction map  (name → bearing °, shortcut key) ─────────────────────────
DIRECTIONS = {
    "1": ("N",   0.0),
    "2": ("NE",  45.0),
    "3": ("E",   90.0),
    "4": ("SE",  135.0),
    "5": ("S",   180.0),
    "6": ("SW",  225.0),
    "7": ("W",   270.0),
    "8": ("NW",  315.0),
    "0": ("Custom", None),   # special: prompts for a number
}

# How close to the target heading (°) before we consider alignment done
ALIGN_TOLERANCE_DEG = 5.0
# Pivot-turn speed used during the align phase (%, passed to car.pivot_turn)
PIVOT_SPEED = 35


# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION I/O
# ══════════════════════════════════════════════════════════════════════════════
def _load_calibration() -> bool:
    """Read compass_cal.json into the shared calibration object."""
    global _COMPASS_CAL
    if not os.path.exists(_CAL_FILE):
        return False
    try:
        _COMPASS_CAL = load_compass_calibration(_CAL_FILE)
        return True
    except Exception as e:
        print(f"⚠️  Could not read calibration file: {e}")
        return False


# ══════════════════════════════════════════════════════════════════════════════
#  MATHS HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def compute_heading(mag_x: float, mag_y: float, mag_z: float = 0.0) -> float | None:
    """Return a calibrated CW compass bearing (0-360°) from raw XYZ data."""
    return compute_heading_degrees((mag_x, mag_y, mag_z), _COMPASS_CAL)

def normalize_error(current: float, target: float) -> float:
    diff = current - target
    while diff >  180.0: diff -= 360.0
    while diff < -180.0: diff += 360.0
    return diff

def circular_mean(angles_deg: list) -> float:
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return (math.degrees(math.atan2(s, c)) + 360.0) % 360.0

def cardinal(deg: float) -> str:
    labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return labels[int((deg + 22.5) / 45.0) % 8]

def drift_label(error: float) -> str:
    # CW-positive compass: error = current − target
    # error < 0 → heading dropped → car turned LEFT (CCW)
    # error > 0 → heading rose   → car turned RIGHT (CW)
    if abs(error) < 1.5:  return "▲  ON TRACK"
    if error < 0:         return "◄  drifting LEFT"
    return                "►  drifting RIGHT"

def bearing_label(deg: float) -> str:
    """Return a human-readable label like '90.0° E'."""
    return f"{deg:5.1f}° {cardinal(deg)}"

def pivot_direction(error: float) -> str:
    """'LEFT' or 'RIGHT' to turn to reduce error (CW-positive compass).
    error < 0 → current heading below target → must turn RIGHT (CW).
    error > 0 → current heading above target → must turn LEFT (CCW)."""
    return "RIGHT" if error < 0 else "LEFT"


# ══════════════════════════════════════════════════════════════════════════════
#  DISPLAY FORMATTING
# ══════════════════════════════════════════════════════════════════════════════
def _row(content: str, W: int) -> str:
    inner = W - 4
    text  = content[:inner]
    return f"║ {text.ljust(inner)} ║"

def _bar(value: float, scale: float, bar_width: int) -> str:
    half   = bar_width // 2
    filled = int(min(abs(value) / scale, 1.0) * half)
    cells  = [" "] * bar_width
    cells[half] = "│"
    if value >= 0:
        for i in range(half, min(half + filled + 1, bar_width)): cells[i] = "█"
    else:
        for i in range(max(half - filled, 0), half): cells[i] = "█"
    return "◄" + "".join(cells) + "►"

def draw_dashboard_curses(
    stdscr,
    target_hdg, current_hdg, error,
    gyro_z, correction,
    mx, my, mz, ax, ay, az,
    rpm_rr, rpm_rl, rpm_fr,
    speed, armed, loop_hz,
    pico_fresh, elapsed_s,
    calibrating=False,
    waypoint_queue=None,   # deque of (label, bearing) tuples
    aligning=False,        # True while pivot-turning to new heading
    mode_label="STRAIGHT", # string shown in header
):
    max_y, max_x = stdscr.getmaxyx()
    W   = max(60, max_x)
    DIV = "─" * (W - 2)
    EQ  = "═" * (W - 2)
    R   = lambda s: _row(s, W)

    bar_w = max(20, W - 20)

    if calibrating:
        status = "⟳  CALIBRATING…"
    elif aligning:
        status = f"↻  ALIGNING → {bearing_label(target_hdg)}  (turning {pivot_direction(error)})"
    elif armed:
        status = f"ARMED  ─  DRIVING → {bearing_label(target_hdg)}"
    else:
        status = "DISARMED  (press SPACE to arm)"

    pico_str = "✓ fresh" if pico_fresh else "✗ stale"

    needle_w = min(37, W - 30)
    needle   = ["-"] * needle_w
    mid_n    = needle_w // 2
    needle[mid_n] = "N" if abs(normalize_error(current_hdg, 0)) < 22.5 else "^"
    target_offset = int(normalize_error(target_hdg, current_hdg) / 180.0 * (needle_w // 2))
    tpos = mid_n + target_offset
    if 0 <= tpos < needle_w and tpos != mid_n:
        needle[tpos] = "T"
    needle_str = "".join(needle)

    # Waypoint queue summary (up to 5 visible)
    wq = list(waypoint_queue) if waypoint_queue else []
    if wq:
        wq_parts = [f"{lbl}({brg:.0f}°)" for lbl, brg in wq[:5]]
        if len(wq) > 5:
            wq_parts.append(f"…+{len(wq)-5}")
        wq_str = " → ".join(wq_parts)
    else:
        wq_str = "(empty — press D to add)"

    lines = [
        f"╔{EQ}╗",
        R(f"  COMPASS DRIVE TEST [{mode_label}]          Q=quit  D=direction  C=clear  N=next"),
        f"╠{DIV}╣",
        R(f"  Status  : {status:<40} Elapsed: {elapsed_s:6.1f} s   Pico: {pico_str}"),
        f"╠{DIV}╣",
        R("  HEADING"),
        R(f"    Target  : {target_hdg:6.1f}°  ({cardinal(target_hdg):<3})        "
          f"Current : {current_hdg:6.1f}°  ({cardinal(current_hdg):<3})"),
        R(f"    Error   : {error:+6.1f}°   {drift_label(error):<22}  "
          f"Gyro-Z : {gyro_z:+6.1f} °/s"),
        R(f"    Compass : [{needle_str}]  ^=now  T=target"),
        f"╠{DIV}╣",
        R("  STEERING CORRECTION"),
        R(f"    Output  : {correction:+6.1f}°"),
        R(f"    {_bar(correction, MAX_STEER_DEG, bar_w)}"),
        f"╠{DIV}╣",
        R("  WAYPOINT QUEUE"),
        R(f"    {wq_str}"),
        f"╠{DIV}╣",
        R("  SENSORS"),
        R(f"    Mag (G) : X {mx:+7.4f}   Y {my:+7.4f}   Z {mz:+7.4f}"),
        R(f"    Acc (g) : X {ax:+7.4f}   Y {ay:+7.4f}   Z {az:+7.4f}"),
        f"╠{DIV}╣",
        R("  MOTORS"),
        R(f"    Speed   : {speed:3d}%        RPM  RR: {rpm_rr:6.1f}   RL: {rpm_rl:6.1f}   FR: {rpm_fr:6.1f}"),
        R(f"    Loop    : {loop_hz:4.1f} Hz"),
        f"╠{DIV}╣",
        R(f"  PID  Kp={KP}  Kd={KD}  Ki={KI}  max_steer=±{MAX_STEER_DEG:.0f}°   "
          f"SPACE=arm/disarm   R=recal   Q=quit"),
        f"╚{EQ}╝",
    ]

    stdscr.erase()
    for idx, line in enumerate(lines):
        if idx < max_y - 1:
            try:
                stdscr.addstr(idx, 0, line[:max_x - 1])
            except curses.error:
                pass
    stdscr.refresh()


def draw_direction_picker(stdscr, selected_key: str, custom_buf: str, typing_custom: bool):
    """Draw the direction-picker overlay centred on the screen."""
    max_y, max_x = stdscr.getmaxyx()
    W = min(54, max_x - 4)
    H = 18
    top  = max(0, (max_y - H) // 2)
    left = max(0, (max_x - W) // 2)

    EQ  = "═" * (W - 2)
    DIV = "─" * (W - 2)

    def ov(row: str):
        inner = W - 4
        return f"║ {row[:inner].ljust(inner)} ║"

    compass_rose = [
        "         N (1)",
        "    NW(8)   NE(2)",
        "  W (7)  +  E (3)",
        "    SW(6)   SE(4)",
        "         S (5)",
    ]

    sel_label, sel_brg = DIRECTIONS.get(selected_key, ("—", None))
    if sel_brg is not None:
        sel_str = f"{sel_label} = {sel_brg:.1f}°"
    elif typing_custom:
        sel_str = f"Custom bearing: {custom_buf}_"
    else:
        sel_str = "Press 0 then type bearing (0-359) + ENTER"

    lines = (
        [f"╔{EQ}╗", ov("  DIRECTION PICKER"), f"╠{DIV}╣"]
        + [ov(f"  {r}") for r in compass_rose]
        + [
            f"╠{DIV}╣",
            ov(f"  Selected  : {sel_str}"),
            f"╠{DIV}╣",
            ov("  ENTER = set target immediately"),
            ov("  +     = append to waypoint queue"),
            ov("  ESC   = cancel"),
            f"╚{EQ}╝",
        ]
    )

    for i, line in enumerate(lines):
        row = top + i
        if row < max_y - 1:
            try:
                stdscr.addstr(row, left, line[:max_x - left - 1])
            except curses.error:
                pass
    stdscr.refresh()


# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION
# ══════════════════════════════════════════════════════════════════════════════
def calibrate_heading(pico: "PicoSensorReader", n: int = CALIB_FRAMES) -> float:
    samples = []
    deadline = time.monotonic() + n * (1.0 / LOOP_HZ) * 3 + 2.0
    while len(samples) < n and time.monotonic() < deadline:
        pkt = pico.get_latest()
        if pkt:
            heading = compute_heading(pkt.mag_x, pkt.mag_y, pkt.mag_z)
            if heading is not None:
                samples.append(heading)
        time.sleep(1.0 / LOOP_HZ)
    if not samples:
        raise RuntimeError("No magnetometer data during calibration.")
    return circular_mean(samples)


# ══════════════════════════════════════════════════════════════════════════════
#  DIRECTION PICKER (blocking modal, called from inside curses loop)
# ══════════════════════════════════════════════════════════════════════════════
def run_direction_picker(stdscr, pico, car, armed,
                         target_heading, current_hdg, waypoint_queue,
                         gyro_z, mx, my, mz, ax, ay, az,
                         rpm_rr, rpm_rl, rpm_fr, loop_hz,
                         start_time):
    """
    Blocking modal.  Returns (new_target_heading, append_to_queue_flag)
    or (None, False) if cancelled.
    """
    selected_key  = "1"   # default highlight
    custom_buf    = ""
    typing_custom = False

    # Draw the underlying dashboard first so it's visible behind the overlay
    draw_dashboard_curses(
        stdscr, target_heading, current_hdg,
        normalize_error(current_hdg, target_heading),
        gyro_z, 0.0, mx, my, mz, ax, ay, az,
        rpm_rr, rpm_rl, rpm_fr, 0, armed, loop_hz,
        pico.is_fresh(), time.monotonic() - start_time,
    )
    draw_direction_picker(stdscr, selected_key, custom_buf, typing_custom)

    stdscr.nodelay(False)   # Block for key input inside picker
    result = (None, False)

    while True:
        try:
            ch = stdscr.getch()
        except KeyboardInterrupt:
            break

        # ── Typing a custom bearing ───────────────────────────────
        if typing_custom:
            if ch in (curses.KEY_ENTER, 10, 13):
                try:
                    val = float(custom_buf) % 360.0
                    custom_buf = f"{val:.1f}"
                    typing_custom = False
                    # Store parsed bearing back so Enter/+ work
                    DIRECTIONS["0"] = ("Custom", val)
                    selected_key = "0"
                except ValueError:
                    custom_buf = ""
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                custom_buf = custom_buf[:-1]
            elif chr(ch) in "0123456789." if 0 < ch < 256 else False:
                if len(custom_buf) < 7:
                    custom_buf += chr(ch)
            draw_direction_picker(stdscr, selected_key, custom_buf, typing_custom)
            continue

        # ── Normal picker navigation ──────────────────────────────
        if ch == 27:   # ESC — cancel
            break
        elif chr(ch) in DIRECTIONS if 0 < ch < 256 else False:
            key = chr(ch)
            selected_key = key
            if key == "0":
                typing_custom = True
                custom_buf = ""
                DIRECTIONS["0"] = ("Custom", None)
        elif ch in (curses.KEY_ENTER, 10, 13):
            # Set target immediately
            _, brg = DIRECTIONS.get(selected_key, (None, None))
            if brg is not None:
                result = (brg, False)
                break
        elif ch == ord('+'):
            # Append to waypoint queue
            lbl, brg = DIRECTIONS.get(selected_key, (None, None))
            if brg is not None:
                waypoint_queue.append((lbl, brg))
                result = (None, True)   # signal: queued, don't change target yet
                break

        draw_direction_picker(stdscr, selected_key, custom_buf, typing_custom)

    stdscr.nodelay(True)
    return result


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP (Inside curses wrapper)
# ══════════════════════════════════════════════════════════════════════════════
def drive_loop(stdscr, pico, car, initial_target_heading, logger=None):
    # Curses setup
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    # PID state
    target_heading = initial_target_heading
    integral   = 0.0
    dt         = 1.0 / LOOP_HZ
    armed      = False
    start_time = time.monotonic()
    loop_hz    = float(LOOP_HZ)

    # Direction / waypoint state
    waypoint_queue: deque = deque()   # deque of (label, bearing) tuples
    aligning  = False                 # True during pivot-turn align phase
    mode_label = "STRAIGHT"           # shown in header

    # Stale display defaults
    current_hdg = target_heading
    gyro_z = mx = my = mz = ax = ay = az = 0.0
    rpm_rr = rpm_rl = rpm_fr = 0.0
    correction = 0.0
    calibrating = False

    while True:
        t0 = time.monotonic()

        # ── Key handling ─────────────────────────────────────────────
        ch = stdscr.getch()
        if ch != -1:
            if ch == ord(' '):
                armed = not armed
                if not armed and car:
                    car.brake()
                    integral = 0.0
                    aligning  = False

            elif ch in (ord('r'), ord('R')):
                # Re-calibrate: lock current direction as new target
                if car and armed:
                    car.brake()
                    armed = False
                aligning = False
                calibrating = True
                draw_dashboard_curses(
                    stdscr, target_heading, current_hdg, 0.0,
                    gyro_z, 0.0, mx, my, mz, ax, ay, az,
                    rpm_rr, rpm_rl, rpm_fr, 0, False, loop_hz,
                    pico.is_fresh(), time.monotonic() - start_time,
                    calibrating=True,
                )
                try:
                    target_heading = calibrate_heading(pico)
                    integral = 0.0
                    mode_label = "STRAIGHT"
                except RuntimeError:
                    pass
                calibrating = False

            elif ch in (ord('d'), ord('D')):
                # Open direction picker overlay
                was_armed = armed
                if car and armed:
                    car.brake()
                    armed = False
                new_brg, queued = run_direction_picker(
                    stdscr, pico, car, was_armed,
                    target_heading, current_hdg, waypoint_queue,
                    gyro_z, mx, my, mz, ax, ay, az,
                    rpm_rr, rpm_rl, rpm_fr, loop_hz, start_time,
                )
                if new_brg is not None:
                    # Immediate: start aligning to the new bearing
                    target_heading = new_brg
                    integral = 0.0
                    aligning = True
                    armed    = False
                    mode_label = f"→ {bearing_label(target_heading)}"
                # If queued=True and queue was empty: pop and start aligning
                if queued and len(waypoint_queue) == 1 and new_brg is None:
                    lbl, brg = waypoint_queue.popleft()
                    target_heading = brg
                    integral = 0.0
                    aligning = True
                    armed    = False
                    mode_label = f"→ {lbl} ({brg:.0f}°)"

            elif ch in (ord('c'), ord('C')):
                waypoint_queue.clear()

            elif ch in (ord('n'), ord('N')):
                # Skip to next waypoint
                if waypoint_queue:
                    if car and armed:
                        car.brake()
                        armed = False
                    lbl, brg = waypoint_queue.popleft()
                    target_heading = brg
                    integral = 0.0
                    aligning = True
                    mode_label = f"→ {lbl} ({brg:.0f}°)"

            elif ch in (ord('q'), ord('Q'), 27, 3):
                break

        # ── Read sensors ─────────────────────────────────────────────
        pkt = pico.get_latest()
        if pkt:
            heading = compute_heading(pkt.mag_x, pkt.mag_y, pkt.mag_z)
            if heading is not None:
                current_hdg = heading
            gyro_z      = pkt.gyro_z
            mx, my, mz  = pkt.mag_x, pkt.mag_y, pkt.mag_z
            ax, ay, az  = pkt.accel_x, pkt.accel_y, pkt.accel_z
            rpm_rr      = getattr(pkt, "rpm_right", 0.0)
            rpm_rl      = getattr(pkt, "rpm_left", 0.0)
            rpm_fr      = 0.0

        # ── Heading error ─────────────────────────────────────────────
        error = normalize_error(current_hdg, target_heading)

        # ── Align phase: pivot-turn until heading is close enough ─────
        if aligning:
            if abs(error) <= ALIGN_TOLERANCE_DEG:
                # Aligned — engage forward drive
                aligning = False
                armed    = True
                integral = 0.0
            else:
                if car:
                    # CW-positive: error < 0 → heading too low → spin RIGHT
                    direction = "left" if error > 0 else "right"
                    car.pivot_turn(direction, PIVOT_SPEED)
                correction = 0.0

        # ── PID forward drive ─────────────────────────────────────────
        if not aligning:
            integral   = max(-I_MAX, min(I_MAX, integral + error * dt))
            correction = (
                HEADING_SIGN * (KP * error + KI * integral)
                - GYRO_SIGN  * KD * gyro_z
            )
            correction = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, correction))

            if armed and car:
                car.set_speed(DRIVE_SPEED)
                car.set_steering(correction)

        # ── Auto-advance waypoint queue after align+drive cycle ───────
        # (When the user has a queue loaded the mode is directional; we
        #  advance to the next waypoint automatically when a future
        #  distance/time threshold is added.  For now the user uses N.)

        # ── Render dashboard ──────────────────────────────────────────
        draw_dashboard_curses(
            stdscr, target_heading, current_hdg, error,
            gyro_z, correction, mx, my, mz, ax, ay, az,
            rpm_rr, rpm_rl, rpm_fr, DRIVE_SPEED if (armed or aligning) else 0,
            armed, loop_hz, pico.is_fresh(),
            time.monotonic() - start_time,
            calibrating=calibrating,
            waypoint_queue=waypoint_queue,
            aligning=aligning,
            mode_label=mode_label,
        )

        # ── Log row ───────────────────────────────────────────────────
        if logger is not None:
            pid_p         = HEADING_SIGN * KP * error
            pid_i_contrib = HEADING_SIGN * KI * integral
            pid_d_contrib = -GYRO_SIGN  * KD * gyro_z
            logger.write(
                timestamp=time.time(),
                elapsed_s=round(time.monotonic() - start_time, 4),
                armed=int(armed),
                target_heading=round(target_heading, 4),
                current_heading=round(current_hdg, 4),
                heading_error=round(error, 4),
                gyro_z=round(gyro_z, 4),
                correction=round(correction, 4),
                pid_p=round(pid_p, 4),
                pid_i_contrib=round(pid_i_contrib, 4),
                pid_d_contrib=round(pid_d_contrib, 4),
                integral=round(integral, 4),
                mag_x=round(mx, 6),
                mag_y=round(my, 6),
                mag_z=round(mz, 6),
                accel_x=round(ax, 6),
                accel_y=round(ay, 6),
                accel_z=round(az, 6),
                rpm_rear_right=round(rpm_rr, 2),
                rpm_rear_left=round(rpm_rl, 2),
                rpm_front_right=round(rpm_fr, 2),
                speed_pct=DRIVE_SPEED if armed else 0,
                loop_hz=round(loop_hz, 2),
            )

        # ── Timing ────────────────────────────────────────────────────
        elapsed_tick = time.monotonic() - t0
        sleep_time   = max(0.0, dt - elapsed_tick)
        time.sleep(sleep_time)
        actual_dt  = time.monotonic() - t0
        loop_hz    = 1.0 / max(actual_dt, 1e-6)


# ══════════════════════════════════════════════════════════════════════════════
#  SETUP & BOOTSTRAP
# ══════════════════════════════════════════════════════════════════════════════
def main():
    if not _PICO_OK:
        print("❌ Pico sensor reader unavailable — aborting.")
        sys.exit(1)

    print("🔌 Connecting to Pico sensor bridge…")
    pico = PicoSensorReader()
    time.sleep(1.2)

    print("⏳ Waiting for first sensor packet…")
    for _ in range(60):
        if pico.is_fresh():
            break
        time.sleep(0.1)
    else:
        print("⚠️  No data from Pico — check UART connection.  Continuing anyway.")

    car = None
    if _MOTOR_OK:
        car = CarSystem()
    else:
        print("⚠️  No motor controller — display-only mode.")

    if _load_calibration():
        print(f"✅ Compass calibration loaded from {_CAL_FILE}")
        print(
            f"   version={_COMPASS_CAL.get('version', 1)}  "
            f"method={_COMPASS_CAL.get('method', 'unknown')}"
        )
        print(
            f"   offset=({_COMPASS_CAL['offset_x']:.2f}, {_COMPASS_CAL['offset_y']:.2f})  "
            f"scale=({_COMPASS_CAL['scale_x']:.4f}, {_COMPASS_CAL['scale_y']:.4f})  "
            f"heading_offset={_COMPASS_CAL['heading_offset_deg']:.2f}°"
        )
    else:
        print(f"⚠️  No compass_cal.json found — using raw (uncalibrated) headings.")
        print(f"   Run compass_calibrate_and_test.py first for accurate headings.")

    print(f"📐 Calibrating heading ({CALIB_FRAMES} samples)…")
    try:
        target_heading = calibrate_heading(pico)
    except RuntimeError as e:
        print(f"❌ Calibration failed: {e}")
        if car: car.stop()
        pico.close()
        sys.exit(1)
        
    print(f"✅ Target heading: {target_heading:.1f}° ({cardinal(target_heading)})")
    time.sleep(0.4)

    logger = DriveLogger()
    print(f"📝 Logging to: {logger.path}")

    # Boot into the flicker-free ncurses loop
    try:
        curses.wrapper(drive_loop, pico, car, target_heading, logger)
    except KeyboardInterrupt:
        pass
    finally:
        if car:
            car.brake()
            time.sleep(0.15)
            car.stop()
        pico.close()
        logger.close()
        print(f"\n✅ Test ended.  Motors stopped.")
        print(f"📊 Log saved → {logger.path}")
        print(f"   Plot with: python3 plot_straight_drive.py\n")

if __name__ == "__main__":
    main()
