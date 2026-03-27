#!/usr/bin/env python3
"""
Ncurses motor + steering tester for 2WD Ackermann chassis.

All commands are sent over UART to the Pico W which controls:
  - 2x rear motors via L298N (unified speed, no per-wheel control)
  - 1x Ackermann front steering servo (GP15)
  - Pan-tilt servos (GP2/GP3)

Steering limits (physically tested):
  CENTER = 1440 us, LEFT = 940 us, RIGHT = 2150 us

Live sensor telemetry from Pico (gyro, accel, laser, mag, battery).
"""

import os
import sys
import time
import curses

# Add core/ to path for bare-name imports of core modules
_diag_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_diag_dir, '..', 'core'))

from pico_sensor_reader import (
    init_pico_reader,
    get_sensor_packet,
    get_diagnostics,
    get_battery_voltage,
    get_current_sense,
    send_motor_command,
    send_reverse_command,
    send_brake,
    send_stop,
)

# ── Steering servo limits (from physically tested steering.py) ──
STEER_CENTER_PW = 1440   # us - straight ahead
STEER_LEFT_PW   = 940    # us - max left lock
STEER_RIGHT_PW  = 2150   # us - max right lock
STEER_STEP       = 20    # us per keypress (matches steering.py)

# ── Motor limits ──
BATTERY_VOLTAGE   = 11.1
MOTOR_MAX_SAFE_V  = 12.0
MAX_PWM_DUTY      = min(95, round((MOTOR_MAX_SAFE_V / BATTERY_VOLTAGE) * 100))
DEFAULT_SPEED     = min(30, MAX_PWM_DUTY)
SPEED_STEP        = 5
MIN_SPEED         = 5


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def draw_ui(stdscr, speed, steer_pw, driving, direction, status_msg, diag, sensor):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    def put(row, col, text, attr=curses.A_NORMAL):
        if 0 <= row < max_y - 1:
            stdscr.addnstr(row, col, text, max_x - col - 1, attr)

    put(0, 0, "2WD Ackermann Motor + Steering Test (Pi -> UART -> Pico)", curses.A_BOLD)
    put(1, 0, "-" * 62)

    # Controls
    put(2, 0, "MOTORS                        STEERING")
    put(3, 0, "  W : Drive forward             L / Left  : Steer left")
    put(4, 0, "  S : Drive reverse             R / Right : Steer right")
    put(5, 0, "  B : Brake (H-bridge lock)     C         : Center steering")
    put(6, 0, "  Space / X : Coast stop")
    put(7, 0, "  + / - : Change speed          [ / ] : Fine steer (1 us)")
    put(8, 0, "  Q : Quit")

    put(10, 0, "-" * 62)

    # Motor status
    put(11, 0, "MOTOR STATUS:", curses.A_BOLD)
    put(12, 0, status_msg)
    put(13, 0, f"Speed: {speed}%  (Max: {MAX_PWM_DUTY}%)   Direction: {direction.upper()}")

    # Steering status
    put(15, 0, "STEERING:", curses.A_BOLD)
    # Build visual steering bar
    bar_width = 40
    range_pw = STEER_RIGHT_PW - STEER_LEFT_PW
    pos = (steer_pw - STEER_LEFT_PW) / range_pw  # 0.0=left, 1.0=right
    marker = int(pos * (bar_width - 1))
    bar = ['-'] * bar_width
    bar[bar_width // 2] = '|'  # center mark
    bar[marker] = '#'
    bar_str = ''.join(bar)

    put(16, 0, f"Pulse Width: {steer_pw} us   [{STEER_LEFT_PW}..{STEER_CENTER_PW}..{STEER_RIGHT_PW}]")
    angle_deg = (steer_pw - STEER_CENTER_PW) / ((STEER_RIGHT_PW - STEER_LEFT_PW) / 2) * 50
    lr = "LEFT" if angle_deg < -2 else ("RIGHT" if angle_deg > 2 else "CENTER")
    put(17, 0, f"Angle: {angle_deg:+.0f} deg  ({lr})")
    put(18, 0, f"  L [{bar_str}] R")

    put(20, 0, "-" * 62)

    # Sensor data
    put(21, 0, "LIVE SENSORS (from Pico):", curses.A_BOLD)
    if sensor:
        put(22, 2, f"Gyro Z   : {sensor.gyro_z:+7.2f} deg/s")
        put(23, 2, f"Accel    : X={sensor.accel_x:+.3f}  Y={sensor.accel_y:+.3f}  Z={sensor.accel_z:+.3f} g")
        put(24, 2, f"Laser    : {sensor.laser_mm} mm")
        put(25, 2, f"Mag      : X={sensor.mag_x:+.4f}  Y={sensor.mag_y:+.4f}  Z={sensor.mag_z:+.4f} Ga")
        put(26, 2, f"IMU Temp : {sensor.temp_c:.1f} C")
    else:
        put(22, 2, "No sensor data yet...")

    # Battery
    batt_v = get_battery_voltage()
    curr_a = get_current_sense()
    if batt_v >= 0 or curr_a >= 0:
        put(27, 2, f"Battery  : {batt_v:.2f} V   Current: {curr_a:.2f} A")

    # Pico connection diagnostics
    put(29, 0, "-" * 62)
    put(30, 0, "PICO LINK:", curses.A_BOLD)
    if diag:
        conn_ok = diag.get('connected', False)
        fresh = diag.get('fresh', False)
        frames = diag.get('frames', 0)
        pkts = diag.get('packets_received', 0)
        pico_errors = diag.get('errors', 0)
        age = diag.get('age_s', -1)

        if conn_ok and fresh:
            attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            conn_str = "CONNECTED (live)"
        elif conn_ok:
            attr = curses.color_pair(1) if curses.has_colors() else curses.A_DIM
            conn_str = f"CONNECTED (stale - {age}s ago)"
        else:
            attr = curses.color_pair(1) if curses.has_colors() else curses.A_DIM
            conn_str = "NO DATA"

        put(31, 2, f"Link: {conn_str}", attr)
        put(32, 2, f"Frames: {frames}  |  Pkts rx: {pkts}  |  Pico errors: {pico_errors}")
    else:
        put(31, 2, "Initializing...")

    stdscr.refresh()


def main_loop(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)

    # Initialize Pico UART bridge
    try:
        init_pico_reader('/dev/ttyS0')
    except Exception as e:
        stdscr.addstr(0, 0, f"FATAL: Cannot open Pico UART: {e}")
        stdscr.refresh()
        time.sleep(3)
        return

    speed = DEFAULT_SPEED
    steer_pw = STEER_CENTER_PW
    driving = False
    direction = "stopped"
    status_msg = "All motors stopped. Steering centered."

    def _send_current():
        """Re-send the current drive state to Pico (keeps watchdog alive)."""
        if not driving:
            return
        if direction == "forward":
            send_motor_command(speed, steer_pw)
        elif direction == "reverse":
            send_reverse_command(speed, steer_pw)

    try:
        while True:
            k = stdscr.getch()
            if k != -1:
                # Handle arrow keys (multi-byte escape sequences)
                if k == curses.KEY_LEFT:
                    key = "steer_left"
                elif k == curses.KEY_RIGHT:
                    key = "steer_right"
                else:
                    try:
                        key = chr(k).lower()
                    except ValueError:
                        key = None

                if key == "q":
                    break

                # ── Motor controls ──
                elif key == "w":
                    driving = True
                    direction = "forward"
                    send_motor_command(speed, steer_pw)
                    status_msg = f"FORWARD at {speed}%"

                elif key == "s":
                    driving = True
                    direction = "reverse"
                    send_reverse_command(speed, steer_pw)
                    status_msg = f"REVERSE at {speed}%"

                elif key == "b":
                    send_brake()
                    driving = False
                    direction = "braked"
                    status_msg = "BRAKED (H-bridge magnetic lock)"

                elif key in (" ", "x"):
                    send_stop()
                    driving = False
                    direction = "stopped"
                    steer_pw = STEER_CENTER_PW
                    status_msg = "All motors stopped. Steering centered."

                elif key in ("+", "="):
                    speed = min(MAX_PWM_DUTY, speed + SPEED_STEP)
                    _send_current()
                    if driving:
                        status_msg = f"{direction.upper()} at {speed}%"

                elif key in ("-", "_"):
                    speed = max(MIN_SPEED, speed - SPEED_STEP)
                    _send_current()
                    if driving:
                        status_msg = f"{direction.upper()} at {speed}%"

                # ── Steering controls ──
                elif key in ("l", "steer_left"):
                    steer_pw = clamp(steer_pw - STEER_STEP, STEER_LEFT_PW, STEER_RIGHT_PW)
                    _send_current()
                    if not driving:
                        # Steer-only: send 0 speed so Pico updates the servo
                        send_motor_command(0, steer_pw)
                    status_msg = f"Steer LEFT  {steer_pw} us" + (f" + {direction.upper()} {speed}%" if driving else "")

                elif key in ("r", "steer_right"):
                    steer_pw = clamp(steer_pw + STEER_STEP, STEER_LEFT_PW, STEER_RIGHT_PW)
                    _send_current()
                    if not driving:
                        send_motor_command(0, steer_pw)
                    status_msg = f"Steer RIGHT {steer_pw} us" + (f" + {direction.upper()} {speed}%" if driving else "")

                elif key == "c":
                    steer_pw = STEER_CENTER_PW
                    _send_current()
                    if not driving:
                        send_motor_command(0, steer_pw)
                    status_msg = "Steering CENTERED" + (f" + {direction.upper()} {speed}%" if driving else "")

                elif key == "[":
                    steer_pw = clamp(steer_pw - 1, STEER_LEFT_PW, STEER_RIGHT_PW)
                    _send_current()
                    if not driving:
                        send_motor_command(0, steer_pw)

                elif key == "]":
                    steer_pw = clamp(steer_pw + 1, STEER_LEFT_PW, STEER_RIGHT_PW)
                    _send_current()
                    if not driving:
                        send_motor_command(0, steer_pw)

            # Keep watchdog alive while driving (Pico auto-stops after 500ms)
            if driving:
                _send_current()

            # Read sensor data
            sensor = get_sensor_packet()
            diag = get_diagnostics()

            # Draw UI
            draw_ui(stdscr, speed, steer_pw, driving, direction, status_msg, diag, sensor)

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        pass
    finally:
        send_stop()


if __name__ == "__main__":
    curses.wrapper(main_loop)
    print("Motor test completed. Motors stopped, steering centered.")
