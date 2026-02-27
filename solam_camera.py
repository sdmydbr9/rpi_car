# =================================================================
# 📝 1. ABSOLUTE FIRST STEP: TURN ON LOGGING
# =================================================================
import logging
logging.basicConfig(
    filename='rover_debug.log',
    level=logging.DEBUG,
    format='%(asctime)s - [%(threadName)s] - %(levelname)s - %(message)s'
)
logging.info("🚀 === SCRIPT LAUNCHED: LOADING LIBRARIES ===")

# =================================================================
# 📦 2. SAFE IMPORTS
# =================================================================
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
import atexit
from datetime import datetime

logging.info("Standard libraries loaded.")

# Import the Pico bridge
try:
    logging.info("Importing Pico bridge...")
    from pico_sensor_reader import (
        init_pico_reader, get_battery_voltage, get_current_sense,
        get_rpm, get_gyro_z
    )
    logging.info("Pico bridge imported successfully.")
except Exception as e:
    logging.error(f"Failed to import Pico bridge: {e}", exc_info=True)

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
MAX_SPEED = 100 
RAMP_STEP = 15  
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
    try:
        v_batt = get_battery_voltage()
        if v_batt < 7.0: v_batt = 12.0
        target_v = 8.0 * (throttle_percent / 100.0) * trim
        duty = (target_v / v_batt) * 100.0
        return max(0.0, min(100.0, duty))
    except Exception as e:
        logging.error(f"Error calculating duty cycle: {e}")
        return 0.0

# =================================================================
# 📊 HARDWARE SETUP
# =================================================================
logging.info("Setting up GPIO pins...")
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

logging.info("Calling init_pico_reader()...")
pico = init_pico_reader()
logging.info("Pico reader initialized.")

motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB, RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]
GPIO.setup(motor_pins, GPIO.OUT)

pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)
logging.info("PWM started on all motors.")

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

    logging.info("Vision Thread: Starting up...")
    
    try:
        logging.info("Vision Thread: Importing Picamera2 (Deferred)...")
        # 🟢 NEW: We import the camera HERE, so if it hangs, the main UI still works!
        from picamera2 import Picamera2
        
        logging.info("Vision Thread: Initializing Picamera2...")
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
        picam2.configure(config)
        logging.info("Vision Thread: Camera configured. Calling start()...")
        picam2.start()
        logging.info("Vision Thread: ✅ Camera started successfully!")
        camera_status = "✅ ONLINE"
        
        atexit.register(picam2.close)
        atexit.register(picam2.stop)
        
        logging.info("Vision Thread: Starting FFMPEG Subprocess...")
        ffmpeg_cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', '640x480', '-r', '30', '-i', '-', '-c:v', 'h264_v4l2m2m', '-b:v', '1.5M', '-f', 'rtsp', MTX_URL
        ]
        ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        atexit.register(lambda: ffmpeg_process.kill())
        logging.info("Vision Thread: FFMPEG Subprocess started.")

        logging.info(f"Vision Thread: Connecting ZMQ to {INTEL_CPU_IP}:{ZMQ_PORT}...")
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.connect(f"tcp://{INTEL_CPU_IP}:{ZMQ_PORT}")
        logging.info("Vision Thread: ZMQ initialized.")

    except Exception as e:
        logging.error(f"Vision Thread: FATAL ERROR during initialization: {e}", exc_info=True)
        camera_status = "❌ INIT FAILED"
        return

    logging.info("Vision Thread: Entering main capture loop.")
    while True:
        try:
            frame = picam2.capture_array()
        except Exception as e:
            logging.debug(f"Vision Thread: Frame capture failed: {e}")
            time.sleep(0.1)
            continue

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
            except zmq.error.Again:
                camera_status = "⚠️ ZMQ TIMEOUT"
                logging.warning("Vision Thread: ZMQ Timeout receiving from Intel CPU.")
            except Exception as e:
                camera_status = "⚠️ NET ERROR"
                logging.error(f"Vision Thread: Network error: {e}")

        try: 
            ffmpeg_process.stdin.write(frame.tobytes())
        except Exception as e:
            logging.error(f"Vision Thread: FFMPEG pipe write failed: {e}")

# =================================================================
# 🧠 SMART DRIVER BRAIN (MAX PUSH HUNTER)
# =================================================================
def auto_pilot():
    global running, status_msg, smoothed_steering, smoothed_throttle
    logging.info("AutoPilot Thread: Started.")
    while True:
        try:
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
                    raw_throttle = MAX_SPEED
                    raw_steering = vision_steering_pull * 1.8
                    status_msg = f"🐺 MAX PUSH HUNT: {selected_target_name}"

            elif vision_mode == "SEARCHING":
                raw_throttle = 0; raw_steering = 0
                status_msg = f"🔍 LOOKING FOR {selected_target_name}..."

            else:
                raw_throttle = 0; raw_steering = 0
                status_msg = "👁️ IDLE"

            smoothed_steering = (0.3 * smoothed_steering) + (0.7 * raw_steering)
            smoothed_throttle = (0.5 * smoothed_throttle) + (0.5 * raw_throttle)
            drive_like_a_car(smoothed_throttle, smoothed_steering, gz)
            time.sleep(0.05)
        except Exception as e:
            logging.error(f"AutoPilot Thread ERROR: {e}", exc_info=True)
            time.sleep(0.5)

# Start Threads
threading.Thread(target=headless_vision_loop, daemon=True, name="Thread-Vision").start()
threading.Thread(target=auto_pilot, daemon=True, name="Thread-AutoPilot").start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================
def main(stdscr):
    global running, vision_mode, selected_target_name
    logging.info("Main Thread: Entering Curses UI loop.")
    stdscr.nodelay(True); curses.curs_set(0)
    
    try:
        while True:
            stdscr.erase()
            stdscr.addstr(0, 0, "--- 🏎️ MAX POWER AI HUNTER ---", curses.A_BOLD)
            stdscr.addstr(2, 0, f"STATUS: {status_msg} | CAM: {camera_status}")
            
            try:
                volts = get_battery_voltage()
                amps = get_current_sense()
                stdscr.addstr(4, 0, f"VOLTS: {volts:.2f}V | AMPS: {amps:.2f}A")
            except Exception as e:
                stdscr.addstr(4, 0, "VOLTS: ERR | AMPS: ERR")
                logging.error(f"UI Thread: Sensor read error: {e}")

            if vision_mode == "WAITING":
                stdscr.addstr(6, 0, "🎯 SELECT TARGET:", curses.A_BLINK)
                for i, name in enumerate(available_targets.keys()):
                    stdscr.addstr(7 + i, 2, f"[{i+1}] {name.upper()}")
            elif selected_target_name:
                stdscr.addstr(6, 0, f"🎯 MISSION: {selected_target_name.upper()}")

            stdscr.addstr(14, 0, "[S] Start [SPACE] Stop [R] New Scan [Q] Quit")

            key = stdscr.getch()
            if key == ord('q'): 
                logging.info("User requested quit ('Q' pressed).")
                break
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
    finally:
        logging.info("Main Thread: Exiting UI and cleaning up GPIO...")
        GPIO.cleanup()
        logging.info("=== SCRIPT TERMINATED SUCCESSFULLY ===")

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        logging.warning("Script interrupted via Ctrl+C")
        GPIO.cleanup()
    except Exception as e:
        logging.error(f"Fatal error in main wrapper: {e}", exc_info=True)
        GPIO.cleanup()
