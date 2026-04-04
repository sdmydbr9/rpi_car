import os
import sys

# Add core/ to path for bare-name imports of core modules
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))

import RPi.GPIO as GPIO
import time
import curses
import threading
import math
from datetime import datetime
from inputs import get_gamepad

# Import CarSystem for unified motor control with closed-loop RPM sync
from motor import CarSystem
from gamepad_axis import (
    build_default_axis_profiles,
    describe_axis_profile,
    load_inputs_axis_profiles,
    normalize_centered_axis,
)

# Import the Pico bridge 
from pico_sensor_reader import (
    init_pico_reader, get_battery_voltage, get_current_sense,
    get_rpm, get_gyro_z, get_sensor_packet
)
import atexit

# =================================================================
# ⚙️ HARDWARE CONFIGURATION
# =================================================================
# TUNING
RAMP_STEP = 15      # How fast motors accelerate (PWM %/tick)
GYRO_KP = 0.18      # Sensitivity of straight-line correction
MAX_YAW_RATE = 140.0

# STATE VARIABLES
running = False
status_msg = "Gamepad Disconnected"
curr_L = 0.0; curr_R = 0.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# GAMEPAD & GEAR GLOBALS
gp_throttle = 0.0
gp_steering = 0.0
JOY_CENTER = 128
JOY_DEADZONE = 15 
AXIS_PROFILES = build_default_axis_profiles(center=JOY_CENTER, deadzone=JOY_DEADZONE)
_axis_profiles_loaded = False

current_gear = "1"
current_max_speed = 35.0  # Default to Gear 1

# BTN_SELECT double-press kill-switch
_select_last_press = 0.0
_DOUBLE_PRESS_WINDOW = 0.6  # seconds

# =================================================================
# 🏎️ CAR SYSTEM SETUP (unified motor control with RPM sync)
# =================================================================
pico = init_pico_reader()

car_system = CarSystem()

# Wheel sync not used in gamepad mode (no GPIO encoders, uses open-loop _set_raw_motors)

# Extra safety: ensure logger stops on any signal-based exit
def _gamepad_cleanup():
    try:
        car_system.stop()
    except Exception:
        pass
atexit.register(_gamepad_cleanup)

# =================================================================
# 🎯 MOTOR CONTROL KERNEL (via CarSystem + WheelSpeedController)
# =================================================================
def drive_like_a_car(throttle, steering, current_gz):
    global curr_L, curr_R
    target_yaw_rate = (steering / 100.0) * MAX_YAW_RATE
    corrected_steering = max(-100, min(100, steering + ((target_yaw_rate - current_gz) * GYRO_KP)))
    servo_angle = max(-50.0, min(50.0, corrected_steering * 0.5))
    target_speed = abs(throttle)

    if abs(throttle) < 5 and abs(steering) < 5:
        target_speed = 0.0

    if curr_L < target_speed:
        curr_L = min(target_speed, curr_L + RAMP_STEP)
    elif curr_L > target_speed:
        curr_L = max(target_speed, curr_L - RAMP_STEP)
    curr_R = curr_L

    # Map gear string for CarSystem
    gear_map = {"1": "1", "2": "2", "3": "3", "SPORT 🚀": "S"}
    gear_key = gear_map.get(current_gear, "1")
    car_system.set_gear(gear_key)

    if curr_L < 1.0:
        curr_L = curr_R = 0.0
        if abs(servo_angle) >= 1.0:
            car_system.send_steering_only(angle=servo_angle)
        else:
            car_system.stop()
        return

    car_system._apply_steering(curr_L, servo_angle, forward=(throttle >= 0))


def _refresh_axis_profiles():
    global AXIS_PROFILES, _axis_profiles_loaded
    AXIS_PROFILES, source_path = load_inputs_axis_profiles(AXIS_PROFILES)
    _axis_profiles_loaded = True
    if source_path:
        summary = ", ".join(
            describe_axis_profile(axis_code, AXIS_PROFILES[axis_code])
            for axis_code in ("ABS_Y", "ABS_Z")
        )
        print(f"🎮 Axis calibration from {source_path}: {summary}")

# =================================================================
# 🎮 GAMEPAD INPUT THREAD (EVOFOX 8-BIT SCALING)
# =================================================================
def gamepad_loop():
    global gp_throttle, gp_steering, status_msg, running, _axis_profiles_loaded, _select_last_press
    global current_gear, current_max_speed
    
    while True:
        try:
            events = get_gamepad()
            if not _axis_profiles_loaded:
                _refresh_axis_profiles()
            for event in events:
                # 🕹️ Left Stick Y (Throttle)
                if event.code == 'ABS_Y':
                    val = event.state
                    gp_throttle = normalize_centered_axis(
                        val,
                        AXIS_PROFILES["ABS_Y"],
                        output_scale=current_max_speed,
                        invert=True,
                    )
                
                # 🕹️ Right Stick X (Steering) -> ABS_Z
                elif event.code == 'ABS_Z':
                    val = event.state
                    gp_steering = normalize_centered_axis(
                        val,
                        AXIS_PROFILES["ABS_Z"],
                        output_scale=100.0,
                    )
                
                # ⚙️ GEAR SHIFTING (A, B, X, Y buttons)
                # Note: `inputs` maps face buttons differently per driver. 
                # Checking both standard linux mappings (SOUTH/EAST) and letter mappings (A/B).
                elif event.code in ['BTN_SOUTH', 'BTN_A'] and event.state == 1:
                    current_gear = "1"
                    current_max_speed = 35.0
                    
                elif event.code in ['BTN_EAST', 'BTN_B'] and event.state == 1:
                    current_gear = "2"
                    current_max_speed = 60.0
                    
                elif event.code in ['BTN_WEST', 'BTN_X'] and event.state == 1:
                    current_gear = "3"
                    current_max_speed = 80.0
                    
                elif event.code in ['BTN_NORTH', 'BTN_Y'] and event.state == 1:
                    current_gear = "SPORT 🚀"
                    current_max_speed = 100.0
                
                # ▶️ Start Button
                elif event.code == 'BTN_START' and event.state == 1:
                    running = not running

                # ⏹️ Select Button double-press → kill-switch (engine OFF only)
                elif event.code == 'BTN_SELECT' and event.state == 1:
                    now = time.time()
                    if (now - _select_last_press) < _DOUBLE_PRESS_WINDOW:
                        running = False
                        _select_last_press = 0.0
                        status_msg = "🚭 SELECT×2 → ENGINE OFF"
                    else:
                        _select_last_press = now
                
            status_msg = "🎮 EvoFox Active"
        except Exception:
            _axis_profiles_loaded = False
            status_msg = "⚠️ Gamepad Error"
            time.sleep(1)

gp_thread = threading.Thread(target=gamepad_loop, daemon=True)
gp_thread.start()

# =================================================================
# 🧠 DRIVER BRAIN
# =================================================================
def motor_driver_loop():
    global running, smoothed_steering, smoothed_throttle
    while True:
        if not running:
            drive_like_a_car(0, 0, 0)
            smoothed_steering = smoothed_throttle = 0.0
            time.sleep(0.1); continue

        gz = get_gyro_z()
        
        smoothed_steering = (0.3 * gp_steering) + (0.7 * smoothed_steering)
        smoothed_throttle = (0.4 * gp_throttle) + (0.6 * smoothed_throttle)
        
        drive_like_a_car(smoothed_throttle, smoothed_steering, gz)

        time.sleep(0.02) 

t = threading.Thread(target=motor_driver_loop, daemon=True)
t.start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================
def main(stdscr):
    global running
    stdscr.nodelay(True)
    curses.curs_set(0)

    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "--- 🏎️ RASPBERRY PI GAMEPAD ROVER ---", curses.A_BOLD)
        
        # Display the active gear and max speed
        stdscr.addstr(2, 0, f"GEAR: [{current_gear}] (Max Pwr: {int(current_max_speed)}%)", curses.A_BOLD)
        
        stdscr.addstr(4, 0, f"INPUT: Throttle: {int(gp_throttle)}% | Steering: {int(gp_steering)}%")
        stdscr.addstr(5, 0, f"MOTORS: Left: {int(curr_L)}% | Right: {int(curr_R)}%")
        
        v_batt = get_battery_voltage()
        amps = get_current_sense()
        stdscr.addstr(7, 0, f"POWER:  {v_batt:.2f}V @ {amps:.2f}A")

        # Wheel sync status
        sync_telem = car_system.get_sync_telemetry()
        sync_status = sync_telem.get('status', 'OFF')
        wheels = sync_telem.get('wheels', {})
        rl_rpm = wheels.get('rl', {}).get('actual_rpm', 0)
        rr_rpm = wheels.get('rr', {}).get('actual_rpm', 0)
        stdscr.addstr(8, 0, f"SYNC:   {sync_status} | RL:{rl_rpm:.0f} RR:{rr_rpm:.0f} RPM")

        stdscr.addstr(10, 0, f"STATUS: {status_msg}")
        
        if running:
            stdscr.addstr(13, 0, "● ON AIR (LIVE)", curses.A_REVERSE)
        else:
            stdscr.addstr(13, 0, "○ STANDBY (Press Start on Gamepad)", curses.A_DIM)

        stdscr.addstr(15, 0, "[A] Gear 1  [B] Gear 2  [X] Gear 3  [Y] SPORT")
        stdscr.addstr(16, 0, "[SELECT×2] Kill Engine  [Q] Quit  [SPACE] Emergency Stop")

        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord(' '): running = False

        time.sleep(0.1)

    # Cleanup
    running = False
    time.sleep(0.2)
    car_system.stop()
    car_system.cleanup()

if __name__ == "__main__":
    curses.wrapper(main)
