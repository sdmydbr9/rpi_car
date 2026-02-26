import RPi.GPIO as GPIO
import time
import curses
import threading
import csv
import os
import cv2
import zmq
import json
import subprocess
from datetime import datetime
from picamera2 import Picamera2

# Import the Pico bridge 
from pico_sensor_reader import (
    init_pico_reader, get_battery_voltage, get_current_sense,
    get_rpm, get_gyro_z
)

# =================================================================
# ⚙️ HARDWARE & NETWORK CONFIGURATION
# =================================================================
INTEL_CPU_IP = "192.168.29.105"  
ZMQ_PORT = "4567"                
MTX_URL = "rtsp://localhost:8554/rover" 

# MOTOR PINS
FL_IN1 = 17; FL_IN2 = 27; FL_ENA = 12
FR_IN3 = 23; FR_IN4 = 22; FR_ENB = 13
RL_IN1 = 9;  RL_IN2 = 11; RL_ENA = 26
RR_IN3 = 10; RR_IN4 = 7;  RR_ENB = 16

# TUNING
MAX_SPEED = 100 # 🔴 Set to 100 for Max PWM capability
RAMP_STEP = 15  # Faster ramp for more "punch"
GYRO_KP = 0.15
MAX_YAW_RATE = 120.0
TRIM_FL = 1.0; TRIM_FR = 1.0
TRIM_RL = 1.0; TRIM_RR = 1.0

TARGET_STOP_AREA = 100000 

# STATE VARIABLES
running = False
status_msg = "Ready"
curr_L = 0.0; curr_R = 0.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# =================================================================
# 👁️ DISTRIBUTED VISION & STREAMING GLOBALS
# =================================================================
camera_status = "⏳ INITIALIZING..."
vision_steering_pull = 0.0
vision_target_found = False
vision_bbox_area = 0        
available_targets = {}      
selected_target_name = ""   
vision_mode = "IDLE"        

# =================================================================
# 🎯 POWER KERNEL: 8V TARGET
# =================================================================
def calculate_safe_duty(throttle_percent, trim=1.0):
    """
    Calculates PWM duty to hit an 8.0V target at 100% throttle.
    """
    v_batt = get_battery_voltage()
    # Failsafe for sensor read errors
    if v_batt < 7.0: v_batt = 12.0 
    
    # 🔴 Target Voltage is now 8.0V for "Push"
    target_v = 8.0 * (throttle_percent / 100.0) * trim
    
    # Duty = (Target Voltage / Available Battery Voltage) * 100
    duty = (target_v / v_batt) * 100.0
    
    # Cap at 100%, Floor at 0%
    return max(0.0, min(100.0, duty))

# =================================================================
# 📊 HARDWARE SETUP
# =================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pico = init_pico_reader()
motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB, RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]
GPIO.setup(motor_pins, GPIO.OUT)

pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)

def ramp_motors(target_L, target_R, dir_L, dir_R):
    global curr_L, curr_R
    curr_L = min(target_L, curr_L + RAMP_STEP) if curr_L < target_L else max(target_L, curr_L - RAMP_STEP)
    curr_R = min(target_R, curr_R + RAMP_STEP) if curr_R < target_R else max(target_R, curr_R - RAMP_STEP)
    
    pwm_fl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_FL))
    pwm_rl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_RL))
    pwm_fr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_FR))
    pwm_rr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_RR))
    
    GPIO.output([FL_IN1, RL_IN1], 1 if dir_L == 1 else 0)
    GPIO.output([FL_IN2, RL_IN2], 1 if dir_L == -1 else 0)
    GPIO.output([FR_IN3, RR_IN3], 1 if dir_R == 1 else 0)
    GPIO.output([FR_IN4, RR_IN4], 1 if dir_R == -1 else 0)

def drive_like_a_car(throttle, steering, current_gz):
    target_yaw = (steering / 100.0) * MAX_YAW_RATE
    corr_steer = max(-100, min(100, steering + ((target_yaw - current_gz) * GYRO_KP)))
    L, R = throttle + corr_steer, throttle - corr_steer
    ramp_motors(abs(L), abs(R), (1 if L>=0 else -1), (1 if R>=0 else -1))

# =================================================================
# 👁️ HEADLESS VISION & PERSISTENT TRACKING
# =================================================================
def headless_vision_loop():
    global vision_steering_pull, vision_target_found, available_targets
    global vision_mode, selected_target_name, camera_status, vision_bbox_area
    
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config); picam2.start()
    camera_status = "✅ ONLINE"

    ffmpeg_cmd = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24', '-s', '640x480', '-r', '30', '-i', '-', '-c:v', 'h264_v4l2m2m', '-b:v', '1.5M', '-f', 'rtsp', MTX_URL
    ]
    ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    context = zmq.Context(); socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, 2000); socket.connect(f"tcp://{INTEL_CPU_IP}:{ZMQ_PORT}")

    while True:
        try:
            frame = picam2.capture_array()
        except: continue

        if vision_mode in ["SCANNING", "TRACKING", "SEARCHING"]:
            net_frame = cv2.resize(frame, (320, 240))
            _, buffer = cv2.imencode('.jpg', net_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            try:
                socket.send(buffer.tobytes())
                live_targets = json.loads(socket.recv_string())
                
                if vision_mode == "SCANNING":
                    available_targets = live_targets
                    vision_mode = "WAITING" if available_targets else "IDLE"
                
                elif vision_mode in ["TRACKING", "SEARCHING"]:
                    if selected_target_name in live_targets:
                        vision_mode = "TRACKING"
                        x, y, w, h = [v*2 for v in live_targets[selected_target_name]]
                        vision_bbox_area = w * h
                        vision_steering_pull = (( (x + w//2) - 320) / 320.0) * 100.0
                        vision_target_found = True
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    else:
                        vision_mode = "SEARCHING"
                        vision_target_found = False
                        vision_steering_pull = 0.0
            except: 
                camera_status = "⚠️ TIMEOUT"; time.sleep(0.5)

        try: ffmpeg_process.stdin.write(frame.tobytes())
        except: pass

threading.Thread(target=headless_vision_loop, daemon=True).start()

# =================================================================
# 🧠 SMART DRIVER BRAIN (MAX PUSH HUNTER)
# =================================================================
def auto_pilot():
    global running, status_msg, smoothed_steering, smoothed_throttle
    while True:
        if not running:
            drive_like_a_car(0, 0, 0); time.sleep(0.1); continue

        gz = get_gyro_z()
        raw_throttle = 0.0; raw_steering = 0.0

        if vision_mode == "TRACKING" and vision_target_found:
            if vision_bbox_area > TARGET_STOP_AREA:
                raw_throttle = 0
                raw_steering = vision_steering_pull * 1.5
                status_msg = f"🏁 AT TARGET: {selected_target_name}"
            else:
                # 🔴 Constant Max Throttle for "Push"
                raw_throttle = MAX_SPEED 
                raw_steering = vision_steering_pull * 1.8
                status_msg = f"🐺 MAX PUSH HUNT: {selected_target_name}"

        elif vision_mode == "SEARCHING":
            raw_throttle = 0; raw_steering = 0 # Stationary Search
            status_msg = f"🔍 LOOKING FOR {selected_target_name}..."

        else:
            raw_throttle = 0; raw_steering = 0
            status_msg = "👁️ IDLE"

        smoothed_steering = (0.3 * smoothed_steering) + (0.7 * raw_steering)
        smoothed_throttle = (0.5 * smoothed_throttle) + (0.5 * raw_throttle)
        drive_like_a_car(smoothed_throttle, smoothed_steering, gz)
        time.sleep(0.05)

threading.Thread(target=auto_pilot, daemon=True).start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================
def main(stdscr):
    global running, vision_mode, selected_target_name
    stdscr.nodelay(True); curses.curs_set(0)
    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "--- 🏎️ MAX POWER AI HUNTER ---", curses.A_BOLD)
        stdscr.addstr(2, 0, f"STATUS: {status_msg}")
        stdscr.addstr(4, 0, f"VOLTS: {get_battery_voltage():.2f}V | AMPS: {get_current_sense():.2f}A")
        
        if vision_mode == "WAITING":
            stdscr.addstr(6, 0, "🎯 SELECT TARGET:", curses.A_BLINK)
            for i, name in enumerate(available_targets.keys()):
                stdscr.addstr(7 + i, 2, f"[{i+1}] {name.upper()}")
        elif selected_target_name:
            stdscr.addstr(6, 0, f"🎯 MISSION: {selected_target_name.upper()}")

        stdscr.addstr(14, 0, "[S] Start [SPACE] Stop [R] New Scan [Q] Quit")
        
        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord('s'): running = True
        elif key == ord(' '): running = False
        elif key == ord('r'):
            vision_mode = "SCANNING"; selected_target_name = ""; running = False
        
        if vision_mode == "WAITING" and ord('1') <= key <= ord('9'):
            idx = key - ord('1')
            if idx < len(available_targets):
                selected_target_name = list(available_targets.keys())[idx]
                vision_mode = "TRACKING"; running = True
        time.sleep(0.1)

    GPIO.cleanup()

if __name__ == "__main__":
    curses.wrapper(main)
