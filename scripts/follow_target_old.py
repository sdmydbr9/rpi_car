"""
follow_target — target-pursuit vision module for the rover.

This is a library module controlled by main.py via the public API:
    start()          – initialise hardware & launch worker threads
    stop()           – tear down threads & release hardware
    is_active()      – True while the module is running
    get_status()     – current telemetry dict
    handle_touch(x,y)– touch-to-pursue a screen coordinate
    handle_command(c)– send a control command (start/stop/scan/reset)
    get_template_jpg()– JPEG bytes of the current tracking template

Do NOT run this file directly.
"""
# =================================================================
# 1. LOGGING: structured telemetry CSV + minimal text log
# =================================================================
import logging
import os as _os
# PROJECT_ROOT is the top-level rpi_car directory (parent of scripts/)
PROJECT_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
logging.basicConfig(
    filename=_os.path.join(PROJECT_ROOT, 'rover_debug.log'),
    level=logging.WARNING,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# =================================================================
# 2. SAFE IMPORTS
# =================================================================
import RPi.GPIO as GPIO
import time
import threading
import csv
import os
import cv2
import zmq
import json
import subprocess
import atexit
from datetime import datetime

# Import the Pico bridge
try:
    from pico_sensor_reader import (
        init_pico_reader, get_battery_voltage, get_current_sense,
        get_rpm, get_gyro_z, get_accel_xyz, get_ir_sensors
    )
except Exception as e:
    logging.error(f"Pico bridge import failed: {e}", exc_info=True)

# Import SensorSystem (laser scanner servo + VL53L0X + rear sonar)
try:
    from sensors import SensorSystem
except Exception as e:
    logging.error(f"SensorSystem import failed: {e}", exc_info=True)
    SensorSystem = None

# =================================================================
# TELEMETRY CSV LOGGER
# =================================================================
_TELEM_FIELDS = [
    'timestamp', 'dt_ms',
    # Sensors
    'laser_front_cm', 'sonar_front_cm',
    'ir_left', 'ir_right',
    # MPU6050
    'accel_x', 'accel_y', 'accel_z', 'accel_mag',
    'gyro_z',
    # Speed
    'rpm', 'battery_v', 'current_a',
    # Target / Vision
    'vision_mode', 'target_name', 'target_found',
    'target_bbox_area', 'vision_steer_pull',
    # Movement / Autopilot
    'avoid_state', 'raw_throttle', 'raw_steering',
    'smooth_throttle', 'smooth_steering',
    'motor_L', 'motor_R',
    # Sweep & Heading
    'sweep_active', 'sweep_esc_angle', 'sweep_esc_dist',
    'rover_heading', 'target_bearing', 'laser_healthy',
    # Touch tracking
    'touch_x', 'touch_y', 'touch_conf',
    # Status
    'status_msg',
]

_telem_file = None
_telem_writer = None
_telem_lock = threading.Lock()

def _init_telemetry():
    global _telem_file, _telem_writer
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(PROJECT_ROOT, 'rover_logs', f'telem_{ts}.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _telem_file = open(path, 'w', newline='', buffering=1)
    _telem_writer = csv.DictWriter(_telem_file, fieldnames=_TELEM_FIELDS, extrasaction='ignore')
    _telem_writer.writeheader()
    return path

def tlog(**kwargs):
    """Write one telemetry row. Call from any thread."""
    if _telem_writer is None:
        return
    kwargs['timestamp'] = time.time()
    with _telem_lock:
        try:
            _telem_writer.writerow(kwargs)
        except Exception:
            pass

telem_path = _init_telemetry()

# =================================================================
# ⚙️ HARDWARE & NETWORK CONFIGURATION
# =================================================================
INTEL_CPU_IP = "192.168.29.105"
ZMQ_PORT = "4567"
MTX_URL = "rtsp://localhost:8554/rover"

# CAPTURE & NETWORK RESOLUTION (adaptive per vision mode)
CAPTURE_W, CAPTURE_H = 1280, 720       # High-res capture for far-object detail
SCAN_W, SCAN_H = 1280, 720             # Full res for initial target handoff
SCAN_JPEG_Q = 95                        # Max quality for signature capture
TRACK_W, TRACK_H = 640, 360            # Half res for fast chase loop
TRACK_JPEG_Q = 80                       # Good enough for tracking

# MOTOR PINS
FL_IN1 = 17; FL_IN2 = 27; FL_ENA = 12
FR_IN3 = 23; FR_IN4 = 22; FR_ENB = 13
RL_IN1 = 9;  RL_IN2 = 11; RL_ENA = 26
RR_IN3 = 10; RR_IN4 = 7;  RR_ENB = 16

# TUNING
MAX_SPEED = 100 
RAMP_STEP = 15  
GYRO_KP = 0.04
MAX_YAW_RATE = 50.0
STEER_RATE_LIMIT = 25.0      # Max steering change per autopilot cycle (high for slalom agility)
VIS_PULL_ALPHA   = 0.3       # EMA smoothing for vision_steering_pull (0=ignore new, 1=no smooth)
VIS_PULL_MAX_JUMP = 25.0     # Max vis_pull change per vision frame (reject wild template jumps)
SPEED_STEER_LIMIT = 75.0     # Max steering (%) at full throttle (high for slalom around obstacles)
TRIM_FL = 1.0; TRIM_FR = 1.0
TRIM_RL = 1.0; TRIM_RR = 1.0

# TARGET APPROACH TUNING (continuous deceleration curve)
# With 80x80 template: scale 1.0=6400, 2.0=25600, 3.0=57600, 4.0=102400
TARGET_STOP_AREA  = 90000   # Full stop (approx scale 3.7x, ~300x300 on screen)
TARGET_SLOW_AREA  = 15000   # Start decelerating (approx scale 1.4x, ~112x112)
TARGET_CREEP_SPEED = 18     # Minimum crawl speed when close to target (%)

# OBSTACLE SLALOM TUNING (smooth reactive steering — never stop-and-pivot!)
OBS_EMERGENCY_CM  = 10      # Last-resort hard steer + near-stop (cm)
OBS_HARD_STEER_CM = 30      # Aggressive slalom zone (cm)
OBS_SOFT_STEER_CM = 60      # Moderate slalom zone (cm)
OBS_AWARE_CM      = 120     # Gentle steering bias zone (cm)
SLALOM_STEER_GAIN = 95.0    # Max steering authority for slalom (aggressive!)
SLALOM_SPEED_MIN  = 22      # Min speed (%) during hard slalom (keep momentum!)
IR_STEER_FORCE    = 55.0    # Steering force from IR side detection (degrees)
TILT_THRESHOLD_G  = 1.4     # Accel magnitude collision detect (g) — lowered from 1.8
GYRO_SPIKE_DPS    = 60.0    # Gyro spike collision detect (deg/s) — catches impacts

# HEADING: PURE GYRO INTEGRATION (no vision correction)
# heading = ∫ gyro_z · dt  — the gyroscope is the SOLE heading source.
# Vision steers the wheels directly (via pull);  heading is only used for
# "memory" when the target is lost after an obstacle dodge.
# Complementary filter REMOVED: it confused lateral translation (slalom)
# with rotation, creating a positive-feedback drift spiral.
GYRO_INTEGRATE_MAX    = 50.0    # Reject gyro above this (dps) — real turns <30, spikes 60-130+
BEARING_GZ_CALM       = 25.0    # Only record bearing when |gz| < this (stable heading)

# STATE VARIABLES
running = False
status_msg = "Ready"
curr_L = 0.0; curr_R = 0.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# SENSOR STATE (updated by sensor thread)
sensor_front_cm = -1        # Forward laser distance (cm)
sonar_front_cm = -1         # Front HC-SR04 ultrasonic (cm)
ir_left_blocked = False     # IR proximity sensor left
ir_right_blocked = False    # IR proximity sensor right
accel_x = 0.0; accel_y = 0.0; accel_z = 0.0
accel_magnitude = 1.0       # Current accel magnitude (g)
gyro_z_val = 0.0
sensor_status = "INIT"      # Sensor subsystem status

# MPU6050 calibration offsets (computed at startup)
mpu_offset_ax = 0.0
mpu_offset_ay = 0.0
mpu_offset_az = 0.0
mpu_offset_gz = 0.0

# SWEEP STATE (on-demand, triggered by obstacle)
sweep_active = False        # True while a sweep is in progress
sweep_escape_angle = 0      # Best escape angle from last sweep (-60..+60)
sweep_escape_dist = 0       # Distance at best escape angle (cm)
sweep_data = []             # Last sweep readings [(angle, cm), ...]
sweep_requested = False     # Set True by autopilot to request a sweep

# HEADING TRACKER (dead-reckoning from gyro_z integration)
# Combines vision bearing with gyro to return to target after slaloming.
rover_heading_deg    = 0.0    # Accumulated yaw (degrees, + = clockwise)
target_bearing_deg   = 0.0    # Absolute bearing toward last known target
bearing_valid        = False  # True after target first seen
_heading_prev_time   = 0.0    # For dt calculation in autopilot loop

# LASER HEALTH TRACKING
laser_consecutive_999 = 0     # Consecutive 999 (invalid) reads
LASER_DEAD_THRESHOLD  = 40    # After 40 x 50ms = 2s of 999, laser is offline
laser_is_healthy      = True  # False when laser appears dead

# SLALOM DIRECTION MEMORY
last_swerve_dir = 1           # +1 = swerve right, -1 = swerve left

# Avoidance state (global for web UI visibility)
avoid_state = "CLEAR"

# TOUCH-TO-PURSUE STATE
touch_template = None        # BGR numpy array (the target crop)
touch_track_pos = None       # (x, y, w, h) in capture-frame coords
touch_track_conf = 0.0       # Template match confidence 0-1
touch_requested = None       # (norm_x, norm_y) from web UI click, or None
touch_lost_count = 0         # Consecutive frames without match
touch_current_scale = 1.0    # Current best scale (grows as rover approaches)
TOUCH_TMPL_SIZE = 80         # Template crop size (px at capture res)
TOUCH_SEARCH_MARGIN = 180    # ROI margin around last known position
TOUCH_CONF_THRESH = 0.30     # Min confidence for a valid match
TOUCH_LOST_MAX = 45          # Frames before giving up (~1.5s at 30fps) — gives time to slalom & return
TOUCH_SCALES = [0.8, 1.0, 1.3, 1.8, 2.5]  # Fewer scales = faster loop = more real-time

# MODULE LIFECYCLE STATE
_shutdown_event = threading.Event()
_threads = []
_active = False
_active_lock = threading.Lock()

# =================================================================
# DISTRIBUTED VISION & STREAMING GLOBALS
# =================================================================
camera_status = "INITIALIZING..."
vision_steering_pull = 0.0
vision_target_found = False
vision_bbox_area = 0
available_targets = {}
selected_target_name = ""
vision_mode = "IDLE"

# =================================================================
# POWER KERNEL: 8V TARGET
# =================================================================
def calculate_safe_duty(throttle_percent, trim=1.0):
    try:
        v_batt = get_battery_voltage()
        if v_batt < 7.0: v_batt = 12.0
        target_v = 8.0 * (throttle_percent / 100.0) * trim
        duty = (target_v / v_batt) * 100.0
        return max(0.0, min(100.0, duty))
    except Exception:
        return 0.0

# =================================================================
# HARDWARE SETUP (deferred to start())
# =================================================================
# GPIO and Pico are initialised inside start() so that importing
# this module does NOT grab hardware resources.
pico = None

# --- MPU6050 CALIBRATION (stationary, ~2 seconds) ---
def calibrate_mpu(samples=40, delay=0.05):
    """Sample MPU at rest, compute offsets. Rover must be stationary!"""
    global mpu_offset_ax, mpu_offset_ay, mpu_offset_az, mpu_offset_gz
    sum_ax = sum_ay = sum_az = sum_gz = 0.0
    good = 0
    for _ in range(samples):
        try:
            ax, ay, az = get_accel_xyz()
            gz = get_gyro_z()
            sum_ax += ax; sum_ay += ay; sum_az += az; sum_gz += gz
            good += 1
        except Exception:
            pass
        time.sleep(delay)
    if good > 0:
        mpu_offset_ax = sum_ax / good
        mpu_offset_ay = sum_ay / good
        mpu_offset_az = (sum_az / good) - 1.0  # gravity = 1g on Z
        mpu_offset_gz = sum_gz / good

# calibrate_mpu() is called inside start(), not at import time

motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB, RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]

# PWMs are created in start() and cleaned up in stop()
pwm_fl = None; pwm_fr = None; pwm_rl = None; pwm_rr = None

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

motor_L_out = 0.0
motor_R_out = 0.0

def drive_like_a_car(throttle, steering, current_gz):
    global motor_L_out, motor_R_out
    # Gyro drift correction: only active when driving roughly straight.
    # When steering is already high, the driver (autopilot) is intentionally
    # turning — the gyro should NOT fight that or it creates a feedback loop.
    straight_factor = max(0.0, 1.0 - abs(steering) / 60.0)
    gyro_correction = -current_gz * GYRO_KP * straight_factor
    corr_steer = max(-100.0, min(100.0, steering + gyro_correction))

    L = throttle + corr_steer
    R = throttle - corr_steer
    # CRITICAL: clamp each motor independently to [-100, 100]
    # Without this, one wheel can hit 200% while the other is 0% → spin-in-place
    L = max(-100.0, min(100.0, L))
    R = max(-100.0, min(100.0, R))
    motor_L_out, motor_R_out = L, R
    ramp_motors(abs(L), abs(R), (1 if L>=0 else -1), (1 if R>=0 else -1))

# =================================================================
# SENSOR POLLING THREAD (front laser, sonar, IR, MPU6050)
# Laser sweep is ON-DEMAND: only triggered when obstacle is detected
# =================================================================
def sensor_polling_loop():
    """Read sensors continuously; laser sweep only when requested."""
    global sensor_front_cm, sonar_front_cm
    global ir_left_blocked, ir_right_blocked
    global accel_x, accel_y, accel_z, accel_magnitude, gyro_z_val
    global sensor_status
    global sweep_active, sweep_escape_angle, sweep_escape_dist
    global sweep_data, sweep_requested
    global laser_consecutive_999, laser_is_healthy

    sensors = None
    if SensorSystem is not None:
        try:
            sensors = SensorSystem()
            sensor_status = "OK"
        except Exception as e:
            logging.error(f"SensorSystem init failed: {e}")
            sensor_status = "PARTIAL"
    else:
        sensor_status = "NO SENSORS"

    cycle = 0
    while not _shutdown_event.is_set():
        try:
            # --- Forward laser (center, servo parked) + health tracking --
            if sensors and not sweep_active:
                fwd = sensors.read_laser_cm()
                # Ignore chassis reflections < 10cm
                if fwd > 0 and fwd < 10:
                    fwd = 999
                    
                if fwd > 0:
                    sensor_front_cm = fwd
                    laser_consecutive_999 = 0
                    laser_is_healthy = True
                else:
                    sensor_front_cm = 999
                    laser_consecutive_999 += 1
                    if laser_consecutive_999 > LASER_DEAD_THRESHOLD:
                        laser_is_healthy = False

            # --- ON-DEMAND SWEEP ---
            if sensors and sweep_requested and not sweep_active:
                sweep_requested = False
                sweep_active = True
                sensor_status = "SWEEPING"

                readings = []
                for angle in range(-60, 61, 10):
                    try:
                        sensors.set_servo_angle(angle)
                        time.sleep(0.04)
                        d = sensors.read_laser_cm()
                        dist = d if d > 0 else 999
                        readings.append((angle, dist))
                    except Exception:
                        readings.append((angle, 999))

                for i, angle in enumerate(range(60, -61, -10)):
                    try:
                        sensors.set_servo_angle(angle)
                        time.sleep(0.04)
                        d = sensors.read_laser_cm()
                        dist = d if d > 0 else 999
                        for j, (a1, d1) in enumerate(readings):
                            if a1 == angle:
                                readings[j] = (angle, (d1 + dist) / 2.0)
                                break
                    except Exception:
                        pass

                sensors.center_servo()

                sweep_data = readings
                best_angle = 0
                best_dist = 0
                for angle, dist in readings:
                    if dist > best_dist:
                        best_dist = dist
                        best_angle = angle

                sweep_escape_angle = best_angle
                sweep_escape_dist = best_dist
                sweep_active = False
                sensor_status = "OK"

                # Log the sweep result
                tlog(laser_front_cm=sensor_front_cm, sweep_active=True,
                     sweep_esc_angle=best_angle, sweep_esc_dist=best_dist,
                     status_msg=f"SWEEP_DONE:{readings}")

            # --- Front sonar every 3rd cycle (HC-SR04 on GPIO 25/24) --
            if sensors and cycle % 3 == 0:
                try:
                    s_front = sensors.get_front_sonar_distance()
                    # Ignored readings < 15cm as they might be chassis/bumper noise
                    if s_front > 0 and s_front < 15:
                        s_front = 999
                    sonar_front_cm = s_front if s_front > 0 else 999
                except Exception:
                    pass

            # --- IR proximity ---
            try:
                # ir_left_blocked, ir_right_blocked = get_ir_sensors()
                # TEMPORARY FIX: Force IR to False to stop random "IR_BOXED" stops 
                ir_left_blocked, ir_right_blocked = False, False
            except Exception:
                pass

            # --- MPU6050 (calibrated) ---
            try:
                ax, ay, az = get_accel_xyz()
                accel_x = ax - mpu_offset_ax
                accel_y = ay - mpu_offset_ay
                accel_z = az - mpu_offset_az
                accel_magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
                gyro_z_val = get_gyro_z() - mpu_offset_gz
            except Exception:
                accel_magnitude = 1.0

            # --- Telemetry log every 5th cycle (~250ms) ---
            if cycle % 5 == 0:
                try:
                    tlog(
                        laser_front_cm=sensor_front_cm,
                        sonar_front_cm=sonar_front_cm,
                        ir_left=ir_left_blocked, ir_right=ir_right_blocked,
                        accel_x=round(accel_x, 3), accel_y=round(accel_y, 3),
                        accel_z=round(accel_z, 3), accel_mag=round(accel_magnitude, 3),
                        gyro_z=round(gyro_z_val, 2),
                        rpm=get_rpm(), battery_v=round(get_battery_voltage(), 2),
                        current_a=round(get_current_sense(), 2),
                    )
                except Exception:
                    pass

            cycle += 1
            time.sleep(0.05)
        except Exception as e:
            logging.error(f"Sensor Thread ERROR: {e}")
            time.sleep(0.3)

# =================================================================
# HEADLESS VISION & PERSISTENT TRACKING
# =================================================================
def headless_vision_loop():
    global vision_steering_pull, vision_target_found, available_targets
    global vision_mode, selected_target_name, camera_status, vision_bbox_area
    global touch_template, touch_track_pos, touch_track_conf
    global touch_lost_count, touch_requested, running
    global _picam2_ref, _ffmpeg_ref

    try:
        os.environ["LIBCAMERA_LOG_LEVELS"] = "*:ERROR"
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (CAPTURE_W, CAPTURE_H), "format": "BGR888"})
        picam2.configure(config)
        picam2.start()
        camera_status = "[OK] ONLINE"
        
        ffmpeg_cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', f'{CAPTURE_W}x{CAPTURE_H}', '-r', '30', '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2.5M', '-f', 'rtsp', MTX_URL
        ]
        ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        _picam2_ref = picam2
        _ffmpeg_ref = ffmpeg_process

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.connect(f"tcp://{INTEL_CPU_IP}:{ZMQ_PORT}")

    except Exception as e:
        logging.error(f"Vision init FATAL: {e}", exc_info=True)
        camera_status = "[!!] INIT FAILED"
        return

    while not _shutdown_event.is_set():
        try:
            frame = picam2.capture_array()
        except Exception:
            if _shutdown_event.is_set():
                break
            time.sleep(0.1)
            continue

        # CRITICAL: drain the camera buffer so we always work on the LATEST frame.
        # Without this, template matching on a 100ms-old frame means the rover
        # steers toward where the target WAS, not where it IS.
        for _ in range(4):  # Drop up to 4 stale buffered frames
            try:
                frame = picam2.capture_array()
            except Exception:
                break

        # --- TOUCH-TO-PURSUE: capture template on click ---
        if touch_requested is not None:
            nx, ny = touch_requested
            touch_requested = None
            px = int(nx * CAPTURE_W)
            py = int(ny * CAPTURE_H)
            half = TOUCH_TMPL_SIZE // 2
            x1 = max(0, px - half)
            y1 = max(0, py - half)
            x2 = min(CAPTURE_W, x1 + TOUCH_TMPL_SIZE)
            y2 = min(CAPTURE_H, y1 + TOUCH_TMPL_SIZE)
            if x2 - x1 > 20 and y2 - y1 > 20:
                touch_template = frame[y1:y2, x1:x2].copy()
                touch_track_pos = (x1, y1, x2 - x1, y2 - y1)
                touch_track_conf = 1.0
                touch_lost_count = 0
                touch_current_scale = 1.0
                vision_mode = "TOUCH_TRACKING"
                selected_target_name = "TOUCH TARGET"
                # Reset heading state for clean pursuit
                rover_heading_deg = 0.0
                target_bearing_deg = 0.0
                bearing_valid = False
                vision_target_found = True
                vision_bbox_area = (x2 - x1) * (y2 - y1)
                running = True
                tlog(touch_x=round(nx, 3), touch_y=round(ny, 3),
                     vision_mode="TOUCH_TRACKING",
                     status_msg="TOUCH TARGET ACQUIRED")

        # --- TOUCH TEMPLATE TRACKING (multi-scale, local) ---
        if vision_mode == "TOUCH_TRACKING" and touch_template is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_tmpl = cv2.cvtColor(touch_template, cv2.COLOR_BGR2GRAY)
            base_th, base_tw = gray_tmpl.shape[:2]
            found = False
            best_val = -1
            best_pos = None
            best_scale = 1.0

            # Build focused scale list: prioritize scales near current (max 5)
            cs = touch_current_scale
            focused_scales = sorted(set(
                [max(0.4, cs * 0.85), cs, min(5.0, cs * 1.2)]
            ))
            # Only add global scales if we haven't found anything yet
            if touch_lost_count > 3:
                focused_scales = sorted(set(list(focused_scales) + TOUCH_SCALES))

            # ROI around last known position (or full frame)
            if touch_track_pos is not None:
                lx, ly, lw, lh = touch_track_pos
                cx, cy = lx + lw // 2, ly + lh // 2
            else:
                cx, cy = CAPTURE_W // 2, CAPTURE_H // 2

            for scale in focused_scales:
                sw = int(base_tw * scale)
                sh = int(base_th * scale)
                if sw < 12 or sh < 12 or sw > CAPTURE_W or sh > CAPTURE_H:
                    continue

                scaled_tmpl = cv2.resize(gray_tmpl, (sw, sh))

                # ROI search first (fast)
                m = TOUCH_SEARCH_MARGIN + max(sw, sh)
                sx1 = max(0, cx - m)
                sy1 = max(0, cy - m)
                sx2 = min(CAPTURE_W, cx + m)
                sy2 = min(CAPTURE_H, cy + m)

                if sx2 - sx1 > sw + 2 and sy2 - sy1 > sh + 2:
                    roi = gray[sy1:sy2, sx1:sx2]
                    res = cv2.matchTemplate(roi, scaled_tmpl, cv2.TM_CCOEFF_NORMED)
                    _, mx_val, _, mx_loc = cv2.minMaxLoc(res)
                    if mx_val > best_val:
                        best_val = mx_val
                        best_pos = (mx_loc[0] + sx1, mx_loc[1] + sy1, sw, sh)
                        best_scale = scale

            # Full-frame fallback REMOVED — it's too slow and matches random
            # textures which send the rover in the wrong direction. Better to
            # declare lost quickly and stop than chase a phantom.

            if best_val > TOUCH_CONF_THRESH and best_pos is not None:
                found = True
                touch_track_pos = best_pos
                touch_track_conf = best_val
                # Smooth scale tracking (don't jump wildly)
                touch_current_scale = 0.7 * touch_current_scale + 0.3 * best_scale

            if found:
                tx, ty_p, tw_t, th_t = touch_track_pos
                # Bbox area now scales with object apparent size!
                vision_bbox_area = tw_t * th_t
                frame_cx = CAPTURE_W // 2
                obj_cx = tx + tw_t // 2
                raw_pull = ((obj_cx - frame_cx) / float(frame_cx)) * 100.0
                # Temporal smoothing: reject wild single-frame jumps from template matching
                pull_delta = raw_pull - vision_steering_pull
                if abs(pull_delta) > VIS_PULL_MAX_JUMP:
                    pull_delta = VIS_PULL_MAX_JUMP if pull_delta > 0 else -VIS_PULL_MAX_JUMP
                vision_steering_pull = vision_steering_pull + VIS_PULL_ALPHA * pull_delta
                vision_target_found = True
                touch_lost_count = 0
                running = True  # Resume driving when target re-acquired
                clr = (0, 255, 0) if touch_track_conf > 0.6 else \
                      (0, 255, 255) if touch_track_conf > 0.4 else (0, 0, 255)
                cv2.rectangle(frame, (tx, ty_p), (tx + tw_t, ty_p + th_t), clr, 3)
                cv2.putText(frame, f"LOCK {touch_track_conf:.0%} s{touch_current_scale:.1f}x",
                            (tx, max(ty_p - 8, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, clr, 2)
                # Slowly refresh template to adapt (only from base-scale crop)
                if touch_track_conf > 0.55 and 0.8 < best_scale < 1.3:
                    crop = frame[ty_p:ty_p + th_t, tx:tx + tw_t]
                    if crop.size > 0:
                        resized_crop = cv2.resize(crop, (base_tw, base_th))
                        touch_template = cv2.addWeighted(
                            touch_template, 0.85, resized_crop, 0.15, 0)
            else:
                touch_lost_count += 1
                touch_track_conf *= 0.85  # Faster confidence decay
                vision_target_found = False
                vision_steering_pull = 0.0
                # If heading memory exists, let autopilot handle return navigation
                # Otherwise stop immediately to avoid blind driving
                if not bearing_valid:
                    running = False
                if touch_lost_count > TOUCH_LOST_MAX:
                    vision_mode = "IDLE"
                    selected_target_name = ""
                    touch_template = None
                    touch_track_pos = None
                    status_msg = "TARGET LOST — STOPPED"
                else:
                    status_msg = f"TARGET LOST ({touch_lost_count}/{TOUCH_LOST_MAX})"
                    if touch_track_pos:
                        tx, ty_p, tw_t, th_t = touch_track_pos
                        cv2.rectangle(frame, (tx, ty_p), (tx + tw_t, ty_p + th_t),
                                      (0, 0, 255), 1)
                        cv2.putText(frame, f"LOST {touch_lost_count}/{TOUCH_LOST_MAX}",
                                    (tx, max(ty_p - 8, 16)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        elif vision_mode in ["SCANNING", "TRACKING", "SEARCHING"]:
            # 🎯 Adaptive resolution: full-res scan for far-object detection,
            #    half-res for fast chase loop once target signature is locked.
            if vision_mode == "SCANNING":
                net_frame = frame  # Full 1280x720 — best detail for initial handoff
                jpeg_quality = SCAN_JPEG_Q
                coord_scale_x = 1
                coord_scale_y = 1
            else:
                net_frame = cv2.resize(frame, (TRACK_W, TRACK_H))
                jpeg_quality = TRACK_JPEG_Q
                coord_scale_x = CAPTURE_W / TRACK_W   # 2.0
                coord_scale_y = CAPTURE_H / TRACK_H   # 2.0

            _, buffer = cv2.imencode('.jpg', net_frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])

            try:
                socket.send(buffer.tobytes())
                live_targets = json.loads(socket.recv_string())

                if vision_mode == "SCANNING":
                    available_targets = live_targets
                    vision_mode = "WAITING" if available_targets else "IDLE"

                elif vision_mode in ["TRACKING", "SEARCHING"]:
                    if selected_target_name in live_targets:
                        vision_mode = "TRACKING"
                        nx, ny, nw, nh = live_targets[selected_target_name]
                        # Map from network-frame coords → capture-frame coords
                        x = int(nx * coord_scale_x)
                        y = int(ny * coord_scale_y)
                        w = int(nw * coord_scale_x)
                        h = int(nh * coord_scale_y)
                        vision_bbox_area = w * h
                        # Steering: normalized center offset (scale-invariant)
                        frame_center_x = CAPTURE_W // 2  # 640
                        raw_pull = (((x + w // 2) - frame_center_x) / float(frame_center_x)) * 100.0
                        # Temporal smoothing
                        pull_delta = raw_pull - vision_steering_pull
                        if abs(pull_delta) > VIS_PULL_MAX_JUMP:
                            pull_delta = VIS_PULL_MAX_JUMP if pull_delta > 0 else -VIS_PULL_MAX_JUMP
                        vision_steering_pull = vision_steering_pull + VIS_PULL_ALPHA * pull_delta
                        vision_target_found = True
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    else:
                        vision_mode = "SEARCHING"
                        vision_target_found = False
                        vision_steering_pull = 0.0
            except zmq.error.Again:
                camera_status = "[!] ZMQ TIMEOUT"
            except Exception as e:
                camera_status = "[!] NET ERROR"

        try: 
            ffmpeg_process.stdin.write(frame.tobytes())
        except Exception:
            pass

# =================================================================
# SMART DRIVER BRAIN — SLALOM OBSTACLE AVOIDANCE + HEADING MEMORY
# Smooth reactive steering around obstacles. Never stop-and-pivot.
# Combines laser, IR, gyro heading, and vision for fluid navigation.
# =================================================================
def _choose_swerve_dir():
    """Pick swerve direction: +1 = right, -1 = left.
    Priority: IR sensors → heading memory → last swerve direction."""
    # IR sensors: highest priority (immediate proximity)
    if ir_left_blocked and not ir_right_blocked:
        return 1    # obstacle left → go right
    if ir_right_blocked and not ir_left_blocked:
        return -1   # obstacle right → go left
    # Heading memory: swerve toward the side the target is on
    if bearing_valid:
        h_err = target_bearing_deg - rover_heading_deg
        h_err = (h_err + 180) % 360 - 180
        if h_err > 5:
            return 1   # target to our right → swerve right past obstacle
        elif h_err < -5:
            return -1
    # Default: maintain last swerve direction (consistent S-curve)
    return last_swerve_dir

def _target_speed_factor(bbox_area):
    """Return 0.0-1.0 speed multiplier based on how close the target is.
    Smooth deceleration curve so the rover never rams the target."""
    if bbox_area <= 0:
        return 1.0
    if bbox_area >= TARGET_STOP_AREA:
        return 0.0
    if bbox_area >= TARGET_SLOW_AREA:
        # Quadratic ease-out deceleration from TARGET_SLOW_AREA to TARGET_STOP_AREA
        t = (bbox_area - TARGET_SLOW_AREA) / (TARGET_STOP_AREA - TARGET_SLOW_AREA)
        # t goes 0..1 as we approach the target
        # speed goes 1..0 with smooth decel (quadratic ease-out)
        factor = (1.0 - t) ** 2
        # Enforce minimum creep so we don't stall far away
        return max(factor, TARGET_CREEP_SPEED / 100.0) if t < 0.85 else factor
    return 1.0

def auto_pilot():
    global running, status_msg, smoothed_steering, smoothed_throttle
    global avoid_state
    global rover_heading_deg, target_bearing_deg, bearing_valid
    global _heading_prev_time, last_swerve_dir

    _heading_prev_time = time.monotonic()
    prev_bbox_area = 0

    while not _shutdown_event.is_set():
        try:
            # --- HEADING INTEGRATION (always, even when stopped) ------
            now = time.monotonic()
            dt = min(now - _heading_prev_time, 0.15)   # Tight cap: missed cycles must not multiply stale gz
            _heading_prev_time = now
            gz = gyro_z_val  # calibrated by sensor thread
            # Reject extreme gyro (vibration/impact, not real rotation)
            if abs(gz) < GYRO_INTEGRATE_MAX:
                rover_heading_deg += gz * dt
            rover_heading_deg = (rover_heading_deg + 180) % 360 - 180

            if not running:
                drive_like_a_car(0, 0, 0)
                avoid_state = "CLEAR"
                prev_bbox_area = 0
                time.sleep(0.1)
                continue

            raw_throttle = 0.0
            raw_steering = 0.0
            obs_note = ""
            # FUSE laser + sonar: take the MINIMUM (most conservative) valid reading.
            # When laser is dead (999), sonar alone protects; and vice versa.
            laser = sensor_front_cm if sensor_front_cm > 0 else 999
            sonar = sonar_front_cm if sonar_front_cm > 0 else 999
            front = min(laser, sonar)

            # -- 1. TARGET PURSUIT (+ record bearing for heading memory) -
            tracking = vision_mode in ("TRACKING", "TOUCH_TRACKING")
            if tracking and vision_target_found:
                area = max(vision_bbox_area, 1)

                # PURE GYRO HEADING — vision drives the wheels, not the heading.
                # Record bearing = heading + camera_offset so we can return to
                # the target after an obstacle dodge (heading memory).
                # Only update bearing when gyro is calm (no vibration/spike),
                # guaranteeing the stored bearing is trustworthy.
                tgt_off = vision_steering_pull * 0.6  # ~60deg half-FOV
                if abs(gz) < BEARING_GZ_CALM:
                    target_bearing_deg = rover_heading_deg + tgt_off
                    target_bearing_deg = (target_bearing_deg + 180) % 360 - 180
                    bearing_valid = True

                # Target speed factor: forced to 1.0 because we are ONLY using
                # front sonar/laser to gauge distance to target (do not rely on bbox area).
                tgt_factor = 1.0

                if tgt_factor <= 0.01:
                    # Arrived — hold position, keep steering to stay centered
                    raw_throttle = 0
                    raw_steering = vision_steering_pull * 1.2
                    status_msg = f">> AT TARGET: {selected_target_name}"
                else:
                    # Chase: speed scales with distance-to-target
                    raw_throttle = MAX_SPEED * tgt_factor
                    # Steering: proportional, gentler to avoid over-rotation
                    steer_gain = 1.0 * tgt_factor + 0.6 * (1.0 - tgt_factor)
                    raw_steering = vision_steering_pull * steer_gain
                    pct = int(tgt_factor * 100)
                    status_msg = f">> HUNTING: {selected_target_name} [{pct}%]"

                # Track approach rate for future predictive braking
                prev_bbox_area = area

            elif vision_mode == "SEARCHING" or (tracking and not vision_target_found):
                # Target lost: heading memory → crawl back toward target
                if bearing_valid:
                    h_err = target_bearing_deg - rover_heading_deg
                    h_err = (h_err + 180) % 360 - 180
                    raw_throttle = MAX_SPEED * 0.30
                    raw_steering = max(-80.0, min(80.0, h_err * 2.0))
                    status_msg = f">> RETURN hdg:{h_err:+.0f}deg"
                else:
                    raw_throttle = 0; raw_steering = 0
                    status_msg = f">> SEARCHING: {selected_target_name}..."
            else:
                raw_throttle = 0; raw_steering = 0
                status_msg = "IDLE"

            # -- 2. COLLISION CHECK (accel spike OR gyro spike) ------------
            if accel_magnitude > TILT_THRESHOLD_G or abs(gz) > GYRO_SPIKE_DPS:
                raw_throttle = 0
                raw_steering = 0
                avoid_state = "COLLISION"
                status_msg = f"!! IMPACT g={accel_magnitude:.1f} gz={gz:.0f}"
                drive_like_a_car(0, 0, 0)
                time.sleep(0.25)
                avoid_state = "CLEAR"
                continue

            # -- 3. SMOOTH SLALOM OBSTACLE AVOIDANCE -------------------
            #    NEVER stop-and-pivot. Steer AROUND obstacles aggressively
            #    while maintaining forward momentum.
            elif raw_throttle > 0:

                # Effective front distance: if laser is dead, infer from IR
                if not laser_is_healthy:
                    if ir_left_blocked and ir_right_blocked:
                        eff_front = OBS_EMERGENCY_CM
                    elif ir_left_blocked or ir_right_blocked:
                        eff_front = OBS_HARD_STEER_CM * 0.8
                    else:
                        eff_front = front   # 999 — no info, drive cautiously
                else:
                    eff_front = front

                slalom_steer = 0.0
                slalom_speed = 1.0     # multiplier 0..1
                is_hunting = tracking and vision_target_found

                if is_hunting:
                    # --- TARGET APPROACH: Stop at the target, DO NOT DODGE! ---
                    if eff_front <= OBS_EMERGENCY_CM:
                        slalom_speed = 0.0
                        avoid_state = "AT_TARGET"
                        obs_note = f" [STOPPED {eff_front:.0f}cm]"
                    elif eff_front <= OBS_HARD_STEER_CM:
                        # Smooth decel
                        t = (eff_front - OBS_EMERGENCY_CM) / (OBS_HARD_STEER_CM - OBS_EMERGENCY_CM)
                        slalom_speed = t * 0.4
                        avoid_state = "APPROACHING"
                        obs_note = f" [APPROACH {eff_front:.0f}cm]"
                    elif eff_front <= OBS_SOFT_STEER_CM:
                        t = (eff_front - OBS_HARD_STEER_CM) / (OBS_SOFT_STEER_CM - OBS_HARD_STEER_CM)
                        slalom_speed = 0.4 + t * 0.6
                        avoid_state = "APPROACHING"
                        obs_note = f" [SLOW {eff_front:.0f}cm]"
                    else:
                        avoid_state = "CLEAR"
                else:
                    if eff_front <= OBS_EMERGENCY_CM:
                        # --- EMERGENCY: steer max, tiny crawl forward --------
                        slalom_speed = SLALOM_SPEED_MIN / 200.0
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN
                        avoid_state = "EMERGENCY"
                        obs_note = f" [EMER {eff_front:.0f}cm]"
    
                    elif eff_front <= OBS_HARD_STEER_CM:
                        # --- HARD SLALOM: max steer, reduced speed -----------
                        slalom_speed = SLALOM_SPEED_MIN / 100.0
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN
                        avoid_state = "SLALOM"
                        obs_note = f" [HARD {eff_front:.0f}cm]"
    
                    elif eff_front <= OBS_SOFT_STEER_CM:
                        # --- MODERATE SLALOM: proportional steer + speed -----
                        t = 1.0 - (eff_front - OBS_HARD_STEER_CM) / (OBS_SOFT_STEER_CM - OBS_HARD_STEER_CM)
                        slalom_speed = (SLALOM_SPEED_MIN / 100.0) + (1.0 - t) * (0.7 - SLALOM_SPEED_MIN / 100.0)
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN * t * 0.7
                        avoid_state = "SLALOM"
                        obs_note = f" [MED {eff_front:.0f}cm {int(t*100)}%]"
    
                    elif eff_front <= OBS_AWARE_CM:
                        # --- GENTLE BIAS: subtle pre-steer, near full speed --
                        t = 1.0 - (eff_front - OBS_SOFT_STEER_CM) / (OBS_AWARE_CM - OBS_SOFT_STEER_CM)
                        slalom_speed = 0.7 + (1.0 - t) * 0.3
                        swerve = _choose_swerve_dir()
                        slalom_steer = swerve * SLALOM_STEER_GAIN * t * 0.25
                        avoid_state = "AWARE"
                        obs_note = f" [AWARE {eff_front:.0f}cm]"
    
                    else:
                        avoid_state = "CLEAR"

                # IR side overrides (always active, even if laser says clear)
                if ir_left_blocked and not ir_right_blocked:
                    slalom_steer = max(slalom_steer, IR_STEER_FORCE)
                    slalom_speed = min(slalom_speed, 0.5)
                    obs_note += " [IR-L]"
                    if avoid_state == "CLEAR":
                        avoid_state = "SLALOM"
                elif ir_right_blocked and not ir_left_blocked:
                    slalom_steer = min(slalom_steer, -IR_STEER_FORCE)
                    slalom_speed = min(slalom_speed, 0.5)
                    obs_note += " [IR-R]"
                    if avoid_state == "CLEAR":
                        avoid_state = "SLALOM"
                elif ir_left_blocked and ir_right_blocked:
                    slalom_speed = 0.0
                    slalom_steer = 0.0
                    avoid_state = "IR_BOXED"
                    obs_note = " [IR-BOTH]"

                # Apply obstacle speed scaling
                raw_throttle *= slalom_speed

                # Blend: obstacle steering dominates during slalom,
                # target steering dominates when clear
                if avoid_state in ("SLALOM", "EMERGENCY", "IR_BOXED"):
                    raw_steering = slalom_steer * 0.80 + raw_steering * 0.20
                elif avoid_state == "AWARE":
                    raw_steering = raw_steering * 0.55 + slalom_steer * 0.45

                if obs_note:
                    status_msg += obs_note

            # -- 4. CLAMP & SMOOTH --
            # Speed-proportional steer cap: the faster we go, the less we turn
            # At 100% throttle → max ±SPEED_STEER_LIMIT; at 0% → full ±100
            speed_ratio = min(1.0, abs(smoothed_throttle) / 100.0)
            max_steer = 100.0 - speed_ratio * (100.0 - SPEED_STEER_LIMIT)
            raw_steering = max(-max_steer, min(max_steer, raw_steering))

            # Rate-limit steering: prevent snap-turns that launch the rover into spins
            steer_delta = raw_steering - smoothed_steering
            if abs(steer_delta) > STEER_RATE_LIMIT:
                steer_delta = STEER_RATE_LIMIT if steer_delta > 0 else -STEER_RATE_LIMIT
            smoothed_steering = max(-max_steer, min(max_steer, smoothed_steering + steer_delta))

            # Faster smoothing when braking (safety), slower when accelerating
            if raw_throttle < smoothed_throttle:
                thr_alpha = 0.8   # Brake hard (80% new value)
            else:
                thr_alpha = 0.4   # Accel gently
            smoothed_throttle = (1.0 - thr_alpha) * smoothed_throttle + thr_alpha * raw_throttle
            drive_like_a_car(smoothed_throttle, smoothed_steering, gz)

            # Telemetry: log every autopilot cycle
            tlog(
                vision_mode=vision_mode,
                target_name=selected_target_name,
                target_found=int(vision_target_found),
                target_bbox_area=vision_bbox_area,
                vision_steer_pull=round(vision_steering_pull, 1),
                touch_conf=round(touch_track_conf, 3),
                avoid_state=avoid_state,
                raw_throttle=round(raw_throttle, 1),
                raw_steering=round(raw_steering, 1),
                smooth_throttle=round(smoothed_throttle, 1),
                smooth_steering=round(smoothed_steering, 1),
                motor_L=round(motor_L_out, 1),
                motor_R=round(motor_R_out, 1),
                laser_front_cm=round(sensor_front_cm, 1),
                sonar_front_cm=round(sonar_front_cm, 1),
                ir_left=int(ir_left_blocked),
                ir_right=int(ir_right_blocked),
                accel_mag=round(accel_magnitude, 2),
                gyro_z=round(gz, 1),
                sweep_active=int(sweep_active),
                sweep_esc_angle=sweep_escape_angle,
                sweep_esc_dist=round(sweep_escape_dist, 1),
                rover_heading=round(rover_heading_deg, 1),
                target_bearing=round(target_bearing_deg, 1),
                laser_healthy=int(laser_is_healthy),
                status_msg=status_msg,
            )
            time.sleep(0.05)
        except Exception as e:
            logging.error(f"AutoPilot error: {e}")
            avoid_state = "CLEAR"
            time.sleep(0.5)

# Module-level refs set by worker threads (for cleanup in stop())
_picam2_ref = None
_ffmpeg_ref = None


# =================================================================
# PUBLIC API — called by main.py
# =================================================================

def start():
    """Initialise hardware, calibrate sensors, and launch worker threads.

    Must be called exactly once.  Call stop() to tear everything down.
    MediaMTX must already be running (managed by main.py).
    """
    global pwm_fl, pwm_fr, pwm_rl, pwm_rr, _active, pico

    with _active_lock:
        if _active:
            return
        _active = True

    _shutdown_event.clear()

    # --- Pico sensor bridge ---
    pico = init_pico_reader()

    # --- GPIO / motor PWM ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(motor_pins, GPIO.OUT)
    pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
    pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
    pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
    pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)

    # --- MPU6050 calibration (rover must be stationary) ---
    calibrate_mpu()

    # --- Worker threads ---
    _threads.clear()
    for target, name in [
        (headless_vision_loop, "Thread-FT-Vision"),
        (auto_pilot,           "Thread-FT-AutoPilot"),
        (sensor_polling_loop,  "Thread-FT-Sensors"),
    ]:
        t = threading.Thread(target=target, daemon=True, name=name)
        t.start()
        _threads.append(t)

    logging.info("follow_target: started")


def stop():
    """Signal all threads to exit, release camera & GPIO."""
    global pwm_fl, pwm_fr, pwm_rl, pwm_rr, running, _active
    global _picam2_ref, _ffmpeg_ref

    with _active_lock:
        if not _active:
            return
        _active = False

    running = False
    _shutdown_event.set()

    # Wait for threads (they are daemon so won't block forever)
    for t in _threads:
        t.join(timeout=5.0)
    _threads.clear()

    # Stop motors
    try:
        drive_like_a_car(0, 0, 0)
    except Exception:
        pass

    # Release camera
    if _picam2_ref is not None:
        try:
            _picam2_ref.stop()
            _picam2_ref.close()
        except Exception:
            pass
        _picam2_ref = None

    # Kill ffmpeg
    if _ffmpeg_ref is not None:
        try:
            _ffmpeg_ref.kill()
            _ffmpeg_ref.wait(timeout=2)
        except Exception:
            pass
        _ffmpeg_ref = None

    # Release PWMs
    for p in (pwm_fl, pwm_fr, pwm_rl, pwm_rr):
        if p is not None:
            try:
                p.stop()
            except Exception:
                pass
    pwm_fl = pwm_fr = pwm_rl = pwm_rr = None

    try:
        GPIO.cleanup()
    except Exception:
        pass

    logging.info("follow_target: stopped")


def is_active() -> bool:
    """Return True while the module is running."""
    return _active


def get_status() -> dict:
    """Build and return the current telemetry dictionary."""
    try:
        batt = get_battery_voltage()
    except Exception:
        batt = 0.0
    return {
        'running': running,
        'status': status_msg,
        'camera': camera_status,
        'mode': vision_mode,
        'target': selected_target_name,
        'found': vision_target_found,
        'bbox': list(touch_track_pos) if touch_track_pos else None,
        'confidence': round(touch_track_conf, 2),
        'bbox_area': vision_bbox_area,
        'sensors': {
            'front_laser': round(sensor_front_cm, 1),
            'front_sonar': round(sonar_front_cm, 1),
            'ir_left': ir_left_blocked,
            'ir_right': ir_right_blocked,
            'accel': round(accel_magnitude, 2),
            'gyro': round(gyro_z_val, 1),
        },
        'motors': {
            'L': round(motor_L_out, 1),
            'R': round(motor_R_out, 1),
        },
        'avoid': avoid_state,
        'battery': round(batt, 2),
        'sweep': sweep_active,
    }


def handle_touch(nx: float, ny: float) -> dict:
    """Register a touch-to-pursue at normalised (0-1) screen coordinates."""
    global touch_requested
    nx = max(0.0, min(1.0, float(nx)))
    ny = max(0.0, min(1.0, float(ny)))
    touch_requested = (nx, ny)
    return {'ok': True, 'x': nx, 'y': ny}


def handle_command(cmd: str) -> dict:
    """Process a control command: start | stop | scan | reset."""
    global running, vision_mode, selected_target_name
    cmd = str(cmd).strip().lower()
    if cmd == 'start':
        running = True
    elif cmd == 'stop':
        running = False
    elif cmd == 'scan':
        vision_mode = "SCANNING"
        selected_target_name = ""
        running = False
    elif cmd == 'reset':
        running = False
        vision_mode = "IDLE"
        selected_target_name = ""
    return {'ok': True, 'cmd': cmd}


def get_template_jpg() -> bytes | None:
    """Return the current tracking template as JPEG bytes, or None."""
    if touch_template is None:
        return None
    ok, buf = cv2.imencode('.jpg', touch_template, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return buf.tobytes() if ok else None
