import RPi.GPIO as GPIO
import time
import curses
import threading
import csv
import os
import math
from datetime import datetime
from inputs import get_gamepad

# Import the Pico bridge 
from pico_sensor_reader import (
    init_pico_reader, get_battery_voltage, get_current_sense,
    get_rpm, get_gyro_z
)

# =================================================================
# ⚙️ HARDWARE CONFIGURATION
# =================================================================
# DRIVER 1: FRONT
FL_IN1 = 17; FL_IN2 = 27; FL_ENA = 12
FR_IN3 = 23; FR_IN4 = 22; FR_ENB = 13
# DRIVER 2: REAR
RL_IN1 = 9;  RL_IN2 = 11; RL_ENA = 26
RR_IN3 = 10; RR_IN4 = 7;  RR_ENB = 16

# TUNING
RAMP_STEP = 15      # How fast motors accelerate
GYRO_KP = 0.18      # Sensitivity of straight-line correction
MAX_YAW_RATE = 140.0
TRIM_FL = 0.6; TRIM_FR = 0.6
TRIM_RL = 1.0; TRIM_RR = 1.0

# STATE VARIABLES
running = False
status_msg = "Gamepad Disconnected"
curr_L = 0.0; curr_R = 0.0
global_max_duty = 95.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# GAMEPAD & GEAR GLOBALS
gp_throttle = 0.0
gp_steering = 0.0
JOY_CENTER = 128
JOY_DEADZONE = 15 

current_gear = "1"
current_max_speed = 35.0  # Default to Gear 1

# NEW BUTTON STATES
btn_select_clicks = 0
btn_select_last_press = 0
btn_select_timeout = 0.5  # Time window for double-click (seconds)

lb_pressed = False
rb_pressed = False
autopilot_enabled = False

emergency_brake_active = False

# =================================================================
# 📊 TELEMETRY LOGGER & SETUP
# =================================================================
if not os.path.exists("rover_logs"): os.makedirs("rover_logs")
log_filename = datetime.now().strftime("rover_logs/rover_manual_log_%Y%m%d_%H%M%S.csv")
log_file = open(log_filename, mode='w', newline='')
log_writer = csv.writer(log_file)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pico = init_pico_reader()
motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB, RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]
GPIO.setup(motor_pins, GPIO.OUT)

pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)

# =================================================================
# 🎯 MOTOR CONTROL KERNEL
# =================================================================
def calculate_safe_duty(throttle_percent, trim=1.0):
    global global_max_duty
    v_target = (throttle_percent / 100.0) * 6.0 * trim
    v_batt = get_battery_voltage()
    current_amps = get_current_sense()
    
    if v_batt < 5.0: v_batt = 12.6 # Default fallback
    v_drop = max(1.8, min(3.0, 1.8 + 0.6 * current_amps))
    v_effective = max(1.0, v_batt - v_drop)
    
    duty = (v_target / v_effective) * 100.0
    global_max_duty = max(0.0, min(95.0, (8.0 / v_effective) * 100.0))
    return max(0.0, min(global_max_duty, duty))

def ramp_motors(target_L, target_R, dir_L, dir_R):
    global curr_L, curr_R
    if curr_L < target_L: curr_L = min(target_L, curr_L + RAMP_STEP)
    elif curr_L > target_L: curr_L = max(target_L, curr_L - RAMP_STEP)
    if curr_R < target_R: curr_R = min(target_R, curr_R + RAMP_STEP)
    elif curr_R > target_R: curr_R = max(target_R, curr_R - RAMP_STEP)

    pwm_fl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_FL))
    pwm_rl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_RL))
    pwm_fr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_FR))
    pwm_rr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_RR))

    GPIO.output([FL_IN1, RL_IN1], 1 if dir_L == 1 else 0)
    GPIO.output([FL_IN2, RL_IN2], 1 if dir_L == -1 else 0)
    GPIO.output([FR_IN3, RR_IN3], 1 if dir_R == 1 else 0)
    GPIO.output([FR_IN4, RR_IN4], 1 if dir_R == -1 else 0)

def drive_like_a_car(throttle, steering, current_gz):
    target_yaw_rate = (steering / 100.0) * MAX_YAW_RATE
    corrected_steering = max(-100, min(100, steering + ((target_yaw_rate - current_gz) * GYRO_KP)))
    
    left_target = throttle + corrected_steering
    right_target = throttle - corrected_steering

    max_val = max(abs(left_target), abs(right_target))
    if max_val > 100.0:
        left_target = (left_target / max_val) * 100.0
        right_target = (right_target / max_val) * 100.0

    dir_L = 1 if left_target >= 0 else (-1 if left_target < 0 else 0)
    dir_R = 1 if right_target >= 0 else (-1 if right_target < 0 else 0)
    
    if abs(throttle) < 5 and abs(steering) < 5: 
        dir_L = dir_R = 0
        left_target = right_target = 0

    ramp_motors(abs(left_target), abs(right_target), dir_L, dir_R)

# =================================================================
# 🎮 GAMEPAD INPUT THREAD (EVOFOX 8-BIT SCALING)
# =================================================================
def gamepad_loop():
    global gp_throttle, gp_steering, status_msg, running
    global current_gear, current_max_speed
    global btn_select_clicks, btn_select_last_press
    global lb_pressed, rb_pressed, autopilot_enabled, emergency_brake_active
    
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
                    if autopilot_enabled:
                        # Emergency brake when autopilot is active
                        emergency_brake_active = True
                    else:
                        # Normal gear shifting
                        current_gear = "3"
                        current_max_speed = 80.0
                    
                elif event.code in ['BTN_NORTH', 'BTN_Y'] and event.state == 1:
                    current_gear = "SPORT 🚀"
                    current_max_speed = 100.0
                
                # 🟡 BTN_WEST/BTN_X release - reset emergency brake
                elif event.code in ['BTN_WEST', 'BTN_X'] and event.state == 0:
                    if autopilot_enabled:
                        emergency_brake_active = False
                
                # 🔘 SELECT Button - Double-click to turn off engine
                elif event.code == 'BTN_SELECT' and event.state == 1:
                    current_time = time.time()
                    if current_time - btn_select_last_press < btn_select_timeout:
                        btn_select_clicks += 1
                    else:
                        btn_select_clicks = 1
                    btn_select_last_press = current_time
                    
                    if btn_select_clicks >= 2:
                        running = False
                        btn_select_clicks = 0
                
                # 🟥 LEFT BUMPER (LB / BTN_TL)
                elif event.code == 'BTN_TL' and event.state == 1:
                    lb_pressed = True
                    # Check if both LB and RB are pressed
                    if lb_pressed and rb_pressed:
                        autopilot_enabled = not autopilot_enabled
                        if not autopilot_enabled:
                            emergency_brake_active = False
                
                elif event.code == 'BTN_TL' and event.state == 0:
                    lb_pressed = False
                
                # 🟥 RIGHT BUMPER (RB / BTN_TR)
                elif event.code == 'BTN_TR' and event.state == 1:
                    rb_pressed = True
                    # Check if both LB and RB are pressed
                    if lb_pressed and rb_pressed:
                        autopilot_enabled = not autopilot_enabled
                        if not autopilot_enabled:
                            emergency_brake_active = False
                
                elif event.code == 'BTN_TR' and event.state == 0:
                    rb_pressed = False
                
                # ▶️ Start Button
                elif event.code == 'BTN_START' and event.state == 1:
                    running = not running
                
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
    global autopilot_enabled, emergency_brake_active
    
    while True:
        if not running:
            drive_like_a_car(0, 0, 0)
            smoothed_steering = smoothed_throttle = 0.0
            time.sleep(0.1); continue

        gz = get_gyro_z()
        
        # If emergency brake is active (autopilot active + X button pressed)
        throttle_to_use = 0 if emergency_brake_active else smoothed_throttle
        
        smoothed_steering = (0.3 * gp_steering) + (0.7 * smoothed_steering)
        smoothed_throttle = (0.4 * gp_throttle) + (0.6 * smoothed_throttle)
        
        drive_like_a_car(throttle_to_use, smoothed_steering, gz)
        time.sleep(0.02) 

t = threading.Thread(target=motor_driver_loop, daemon=True)
t.start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================
def main(stdscr):
    global running, autopilot_enabled
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
        stdscr.addstr(9, 0, f"STATUS: {status_msg}")
        
        # Autopilot status
        autopilot_status = "🤖 AUTOPILOT ACTIVE" if autopilot_enabled else "○ Autopilot Inactive"
        autopilot_color = curses.A_REVERSE if autopilot_enabled else curses.A_DIM
        stdscr.addstr(10, 0, autopilot_status, autopilot_color)
        
        # Emergency brake status (only show when autopilot is active)
        if autopilot_enabled and emergency_brake_active:
            stdscr.addstr(11, 0, "🛑 EMERGENCY BRAKE ACTIVE", curses.A_REVERSE)
        
        if running:
            stdscr.addstr(13, 0, "● ON AIR (LIVE)", curses.A_REVERSE)
        else:
            stdscr.addstr(13, 0, "○ STANDBY (Press Start on Gamepad)", curses.A_DIM)

        stdscr.addstr(16, 0, "[A] Gear 1  [B] Gear 2  [X] Gear 3*  [Y] SPORT", curses.A_DIM)
        stdscr.addstr(17, 0, "[START] Engine Toggle  [SELECT] Double-click to Turn Off", curses.A_DIM)
        stdscr.addstr(18, 0, "[LB+RB] Toggle Autopilot  [X]* Emergency Brake (in Autopilot)", curses.A_DIM)
        stdscr.addstr(19, 0, "* X function changes based on autopilot state", curses.A_DIM)
        stdscr.addstr(21, 0, "[Q] Quit    [SPACE] Emergency Stop")

        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord(' '): running = False

        time.sleep(0.1)

    # Cleanup
    running = False
    time.sleep(0.2)
    pwm_fl.stop(); pwm_fr.stop(); pwm_rl.stop(); pwm_rr.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    curses.wrapper(main)
