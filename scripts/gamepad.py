import RPi.GPIO as GPIO
import time
import curses
import threading
import os
import math
from datetime import datetime
from inputs import get_gamepad

# Import CarSystem for unified motor control with closed-loop RPM sync
from motor import CarSystem

# Import the Pico bridge 
from pico_sensor_reader import (
    init_pico_reader, get_battery_voltage, get_current_sense,
    get_rpm, get_gyro_z, get_sensor_packet
)
import atexit
from drive_logger import DriveLogger

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

# Attach closed-loop wheel speed sync (compensates plastic RR gear)
try:
    car_system.attach_wheel_sync(
        get_rpm_fn=get_rpm,
        get_freshness_fn=lambda: pico is not None and pico.is_connected() and pico.is_fresh(max_age_s=1.0),
    )
except Exception as e:
    print(f"⚠️  Wheel sync init error: {e}")

# Drive telemetry logger (atexit registered inside DriveLogger.start())
drive_logger = DriveLogger(source="gamepad")
drive_logger.start()

# Extra safety: ensure logger stops on any signal-based exit
def _gamepad_cleanup():
    drive_logger.stop()
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
    
    left_target = throttle + corrected_steering
    right_target = throttle - corrected_steering

    max_val = max(abs(left_target), abs(right_target))
    if max_val > 100.0:
        left_target = (left_target / max_val) * 100.0
        right_target = (right_target / max_val) * 100.0

    is_forward_l = left_target >= 0
    is_forward_r = right_target >= 0
    
    if abs(throttle) < 5 and abs(steering) < 5: 
        left_target = right_target = 0
        is_forward_l = is_forward_r = True

    # Ramp toward target speeds
    target_L = abs(left_target)
    target_R = abs(right_target)
    if curr_L < target_L: curr_L = min(target_L, curr_L + RAMP_STEP)
    elif curr_L > target_L: curr_L = max(target_L, curr_L - RAMP_STEP)
    if curr_R < target_R: curr_R = min(target_R, curr_R + RAMP_STEP)
    elif curr_R > target_R: curr_R = max(target_R, curr_R - RAMP_STEP)

    # Map gear string for CarSystem
    gear_map = {"1": "1", "2": "2", "3": "3", "SPORT 🚀": "S"}
    gear_key = gear_map.get(current_gear, "1")
    car_system.set_gear(gear_key)

    # Drive through CarSystem → WheelSpeedController (closed-loop RPM sync)
    car_system._set_raw_motors(curr_L, curr_R, is_forward_l, is_forward_r)

# =================================================================
# 🎮 GAMEPAD INPUT THREAD (EVOFOX 8-BIT SCALING)
# =================================================================
def gamepad_loop():
    global gp_throttle, gp_steering, status_msg, running
    global current_gear, current_max_speed
    
    while True:
        try:
            events = get_gamepad()
            for event in events:
                # 🕹️ Left Stick Y (Throttle)
                if event.code == 'ABS_Y':
                    val = event.state
                    if abs(val - JOY_CENTER) < JOY_DEADZONE: 
                        gp_throttle = 0.0
                    else:
                        # Throttle relies on the CURRENT active gear
                        gp_throttle = ((JOY_CENTER - val) / 128.0) * current_max_speed
                
                # 🕹️ Right Stick X (Steering) -> ABS_Z
                elif event.code == 'ABS_Z':
                    val = event.state
                    if abs(val - JOY_CENTER) < JOY_DEADZONE: 
                        gp_steering = 0.0
                    else:
                        gp_steering = ((val - JOY_CENTER) / 128.0) * 100.0
                
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

        # ── Comprehensive drive telemetry logging ────────────
        if drive_logger.is_running:
            try:
                pkt = get_sensor_packet()
                sync_telem = car_system.get_sync_telemetry()
                wheels = sync_telem.get('wheels', {})
                gear_map = {"1": "1", "2": "2", "3": "3", "SPORT 🚀": "S"}
                drive_logger.log_tick({
                    'laser_front_mm': pkt.laser_mm if pkt else -1,
                    'sonar_front_cm': -1,  # No sonar in standalone gamepad mode
                    'accel_x': round(pkt.accel_x, 4) if pkt else 0,
                    'accel_y': round(pkt.accel_y, 4) if pkt else 0,
                    'accel_z': round(pkt.accel_z, 4) if pkt else 0,
                    'gyro_x': round(pkt.gyro_x, 2) if pkt else 0,
                    'gyro_y': round(pkt.gyro_y, 2) if pkt else 0,
                    'gyro_z': round(pkt.gyro_z, 2) if pkt else 0,
                    'temp_c': round(pkt.temp_c, 1) if pkt else 0,
                    'battery_mv': round(pkt.adc_a0, 1) if pkt else 0,
                    'current_mv': round(pkt.adc_a1, 1) if pkt else 0,
                    'rpm_rear_left': round(pkt.rpm_rear_left, 1) if pkt else 0,
                    'rpm_rear_right': round(pkt.rpm_rear_right, 1) if pkt else 0,
                    'rpm_front_right': round(pkt.rpm_front_right, 1) if pkt else 0,
                    'cmd_pwm_left': round(curr_L, 1),
                    'cmd_pwm_right': round(curr_R, 1),
                    'applied_pwm_fl': round(wheels.get('fl', {}).get('applied_pwm', 0), 1),
                    'applied_pwm_fr': round(wheels.get('fr', {}).get('applied_pwm', 0), 1),
                    'applied_pwm_rl': round(wheels.get('rl', {}).get('applied_pwm', 0), 1),
                    'applied_pwm_rr': round(wheels.get('rr', {}).get('applied_pwm', 0), 1),
                    'gear': gear_map.get(current_gear, "1"),
                    'throttle_input': round(smoothed_throttle, 1),
                    'steering_input': round(smoothed_steering, 1),
                    'is_braking': False,
                    'is_forward': True,
                    'sync_status': sync_telem.get('status', 'OFF'),
                    'target_rpm_left': round(wheels.get('rl', {}).get('target_rpm', 0), 1),
                    'target_rpm_right': round(wheels.get('rr', {}).get('target_rpm', 0), 1),
                })
            except Exception:
                pass  # Never crash the drive loop for logging

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
        stdscr.addstr(11, 0, f"LOG:    {drive_logger.tick_count} rows → {drive_logger.filepath or 'N/A'}")
        
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
    drive_logger.stop()
    time.sleep(0.2)
    car_system.stop()
    car_system.cleanup()

if __name__ == "__main__":
    curses.wrapper(main)
