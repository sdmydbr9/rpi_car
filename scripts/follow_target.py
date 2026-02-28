"""
follow_target — target-pursuit vision module for the rover.

INTEGRATED version: shares CarSystem, SensorSystem, and Pico bridge
with the main car system (just like AutoPilot does).

Public API (called by main.py):
    init(car_system, sensor_system, get_laser, get_sonar, get_ir, get_gyro_z,
         get_accel_xyz, get_battery_voltage, get_current_sense, get_rpm)
                     – wire up shared hardware references
    start()          – launch worker threads (hardware already initialized)
    stop()           – tear down threads, stop motors
    is_active()      – True while the module is running
    get_status()     – current telemetry dict
    handle_touch(x,y)– touch-to-pursue a screen coordinate
    handle_command(c)– send a control command (start/stop/scan/reset)
    get_template_jpg()– JPEG bytes of the current tracking template

Do NOT run this file directly.
"""
# =================================================================
# 1. LOGGING
# =================================================================
import logging
import os as _os
PROJECT_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
logging.basicConfig(
    filename=_os.path.join(PROJECT_ROOT, 'rover_debug.log'),
    level=logging.WARNING,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# =================================================================
# 2. IMPORTS
# =================================================================
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

# =================================================================
# TELEMETRY CSV LOGGER
# =================================================================
_TELEM_FIELDS = [
    'timestamp', 'dt_ms',
    'laser_front_cm', 'sonar_front_cm',
    'ir_left', 'ir_right',
    'accel_x', 'accel_y', 'accel_z', 'accel_mag',
    'gyro_z',
    'rpm', 'battery_v', 'current_a',
    'vision_mode', 'target_name', 'target_found',
    'target_bbox_area', 'vision_steer_pull',
    'avoid_state', 'raw_throttle', 'raw_steering',
    'smooth_throttle', 'smooth_steering',
    'motor_L', 'motor_R',
    'sweep_active', 'sweep_esc_angle', 'sweep_esc_dist',
    'rover_heading', 'target_bearing', 'laser_healthy',
    'touch_x', 'touch_y', 'touch_conf',
    'status_msg',
]

_telem_file = None
_telem_writer = None
_telem_lock = threading.Lock()

def _init_telemetry():
    global _telem_file, _telem_writer
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(PROJECT_ROOT, 'rover_logs', f'hunter_telem_{ts}.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _telem_file = open(path, 'w', newline='', buffering=1)
    _telem_writer = csv.DictWriter(_telem_file, fieldnames=_TELEM_FIELDS, extrasaction='ignore')
    _telem_writer.writeheader()
    return path

def tlog(**kwargs):
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
# CONFIGURATION
# =================================================================
INTEL_CPU_IP = "192.168.29.105"
ZMQ_PORT = "4567"
MTX_URL = "rtsp://localhost:8554/rover"

CAPTURE_W, CAPTURE_H = 1280, 720
SCAN_W, SCAN_H = 1280, 720
SCAN_JPEG_Q = 95
TRACK_W, TRACK_H = 640, 360
TRACK_JPEG_Q = 80

# TUNING
MAX_SPEED = 100
RAMP_STEP = 15
GYRO_KP = 0.04
MAX_YAW_RATE = 50.0
STEER_RATE_LIMIT = 25.0
VIS_PULL_ALPHA = 0.3
VIS_PULL_MAX_JUMP = 25.0
SPEED_STEER_LIMIT = 75.0

# TARGET APPROACH
TARGET_STOP_AREA = 90000
TARGET_SLOW_AREA = 15000
TARGET_CREEP_SPEED = 18

# OBSTACLE SLALOM
OBS_EMERGENCY_CM = 10
OBS_HARD_STEER_CM = 30
OBS_SOFT_STEER_CM = 60
OBS_AWARE_CM = 120
SLALOM_STEER_GAIN = 95.0
SLALOM_SPEED_MIN = 22
IR_STEER_FORCE = 55.0
TILT_THRESHOLD_G = 1.4
GYRO_SPIKE_DPS = 60.0

# HEADING
GYRO_INTEGRATE_MAX = 50.0
BEARING_GZ_CALM = 25.0

# =================================================================
# SHARED HARDWARE REFERENCES (set by init())
# =================================================================
_car_system = None       # motor.CarSystem — shared with main.py
_sensor_system = None    # sensors.SensorSystem — shared with main.py
_get_laser = None        # callable -> float (cm)
_get_sonar = None        # callable -> float (cm)
_get_ir = None           # callable -> (bool, bool)
_get_gyro_z = None       # callable -> float (deg/s)
_get_accel_xyz = None    # callable -> (float, float, float)
_get_battery_voltage = None  # callable -> float
_get_current_sense = None    # callable -> float
_get_rpm = None              # callable -> float
_initialized = False

# =================================================================
# STATE VARIABLES
# =================================================================
running = False
user_paused = False   # True when user explicitly pressed STOP; prevents auto-resume
status_msg = "Ready"
curr_L = 0.0
curr_R = 0.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# SENSOR STATE
sensor_front_cm = -1
sonar_front_cm = -1
ir_left_blocked = False
ir_right_blocked = False
accel_x = 0.0; accel_y = 0.0; accel_z = 0.0
accel_magnitude = 1.0
gyro_z_val = 0.0
sensor_status = "INIT"

# MPU6050 calibration offsets
mpu_offset_ax = 0.0
mpu_offset_ay = 0.0
mpu_offset_az = 0.0
mpu_offset_gz = 0.0

# SWEEP STATE
sweep_active = False
sweep_escape_angle = 0
sweep_escape_dist = 0
sweep_data = []
sweep_requested = False

# HEADING TRACKER
rover_heading_deg = 0.0
target_bearing_deg = 0.0
bearing_valid = False
_heading_prev_time = 0.0

# LASER HEALTH
laser_consecutive_999 = 0
LASER_DEAD_THRESHOLD = 40
laser_is_healthy = True

# SLALOM
last_swerve_dir = 1
avoid_state = "CLEAR"
motor_L_out = 0.0
motor_R_out = 0.0

# TOUCH-TO-PURSUE STATE
touch_template = None
touch_track_pos = None
touch_track_conf = 0.0
touch_requested = None
touch_lost_count = 0
touch_current_scale = 1.0
TOUCH_TMPL_SIZE = 80
TOUCH_SEARCH_MARGIN = 180
TOUCH_CONF_THRESH = 0.30
TOUCH_LOST_MAX = 45
TOUCH_SCALES = [0.8, 1.0, 1.3, 1.8, 2.5]

# MODULE LIFECYCLE STATE
_shutdown_event = threading.Event()
_threads = []
_active = False
_active_lock = threading.Lock()

# VISION GLOBALS
camera_status = "INITIALIZING..."
vision_steering_pull = 0.0
vision_target_found = False
vision_bbox_area = 0
available_targets = {}
selected_target_name = ""
vision_mode = "IDLE"


# =================================================================
# INIT — wire up shared hardware (called once from main.py)
# =================================================================
def init(car_system, sensor_system, get_laser, get_sonar, get_ir,
         get_gyro_z_fn, get_accel_xyz_fn, get_battery_voltage_fn,
         get_current_sense_fn, get_rpm_fn):
    """Wire up shared hardware references from main.py.

    Must be called before start(). This replaces the old standalone
    GPIO/Pico/SensorSystem initialization.
    """
    global _car_system, _sensor_system
    global _get_laser, _get_sonar, _get_ir, _get_gyro_z, _get_accel_xyz
    global _get_battery_voltage, _get_current_sense, _get_rpm
    global _initialized

    _car_system = car_system
    _sensor_system = sensor_system
    _get_laser = get_laser
    _get_sonar = get_sonar
    _get_ir = get_ir
    _get_gyro_z = get_gyro_z_fn
    _get_accel_xyz = get_accel_xyz_fn
    _get_battery_voltage = get_battery_voltage_fn
    _get_current_sense = get_current_sense_fn
    _get_rpm = get_rpm_fn
    _initialized = True
    print("🎯 [Hunter] Initialized with shared CarSystem + SensorSystem")


# =================================================================
# MOTOR CONTROL — uses shared CarSystem
# =================================================================
def _drive_motors(left_pct, right_pct, left_fwd, right_fwd):
    """Drive motors via the shared CarSystem._set_raw_motors().

    left_pct, right_pct: 0-100 duty cycle
    left_fwd, right_fwd: True = forward, False = reverse
    """
    if _car_system is None:
        return
    try:
        _car_system._set_raw_motors(
            abs(left_pct), abs(right_pct),
            left_fwd, right_fwd
        )
    except Exception as e:
        logging.error(f"Hunter motor error: {e}")


def drive_like_a_car(throttle, steering, current_gz):
    """Differential drive with gyro drift correction.

    Uses the shared CarSystem for motor output.
    """
    global motor_L_out, motor_R_out

    straight_factor = max(0.0, 1.0 - abs(steering) / 60.0)
    gyro_correction = -current_gz * GYRO_KP * straight_factor
    corr_steer = max(-100.0, min(100.0, steering + gyro_correction))

    L = throttle + corr_steer
    R = throttle - corr_steer
    L = max(-100.0, min(100.0, L))
    R = max(-100.0, min(100.0, R))
    motor_L_out, motor_R_out = L, R

    _drive_motors(abs(L), abs(R), L >= 0, R >= 0)


# =================================================================
# MPU6050 CALIBRATION
# =================================================================
def calibrate_mpu(samples=40, delay=0.05):
    global mpu_offset_ax, mpu_offset_ay, mpu_offset_az, mpu_offset_gz
    if _get_accel_xyz is None or _get_gyro_z is None:
        return
    sum_ax = sum_ay = sum_az = sum_gz = 0.0
    good = 0
    for _ in range(samples):
        try:
            ax, ay, az = _get_accel_xyz()
            gz = _get_gyro_z()
            sum_ax += ax; sum_ay += ay; sum_az += az; sum_gz += gz
            good += 1
        except Exception:
            pass
        time.sleep(delay)
    if good > 0:
        mpu_offset_ax = sum_ax / good
        mpu_offset_ay = sum_ay / good
        mpu_offset_az = (sum_az / good) - 1.0
        mpu_offset_gz = sum_gz / good


# =================================================================
# SENSOR POLLING THREAD — uses shared sensors
# =================================================================
def sensor_polling_loop():
    global sensor_front_cm, sonar_front_cm
    global ir_left_blocked, ir_right_blocked
    global accel_x, accel_y, accel_z, accel_magnitude, gyro_z_val
    global sensor_status
    global sweep_active, sweep_escape_angle, sweep_escape_dist
    global sweep_data, sweep_requested
    global laser_consecutive_999, laser_is_healthy

    sensor_status = "OK" if _sensor_system is not None else "NO SENSORS"

    cycle = 0
    while not _shutdown_event.is_set():
        try:
            # --- Forward laser (from shared Pico bridge) ---
            if _get_laser and not sweep_active:
                fwd = _get_laser()
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

            # --- ON-DEMAND SWEEP (uses shared SensorSystem) ---
            if _sensor_system and sweep_requested and not sweep_active:
                sweep_requested = False
                sweep_active = True
                sensor_status = "SWEEPING"

                readings = []
                for angle in range(-60, 61, 10):
                    try:
                        _sensor_system.set_servo_angle(angle)
                        time.sleep(0.04)
                        d = _sensor_system.read_laser_cm()
                        dist = d if d > 0 else 999
                        readings.append((angle, dist))
                    except Exception:
                        readings.append((angle, 999))

                for i, angle in enumerate(range(60, -61, -10)):
                    try:
                        _sensor_system.set_servo_angle(angle)
                        time.sleep(0.04)
                        d = _sensor_system.read_laser_cm()
                        dist = d if d > 0 else 999
                        for j, (a1, d1) in enumerate(readings):
                            if a1 == angle:
                                readings[j] = (angle, (d1 + dist) / 2.0)
                                break
                    except Exception:
                        pass

                _sensor_system.center_servo()

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

                tlog(laser_front_cm=sensor_front_cm, sweep_active=True,
                     sweep_esc_angle=best_angle, sweep_esc_dist=best_dist,
                     status_msg=f"SWEEP_DONE:{readings}")

            # --- Front sonar every 3rd cycle ---
            if _get_sonar and cycle % 3 == 0:
                try:
                    s_front = _get_sonar()
                    if s_front > 0 and s_front < 15:
                        s_front = 999
                    sonar_front_cm = s_front if s_front > 0 else 999
                except Exception:
                    pass

            # --- IR proximity ---
            try:
                if _get_ir:
                    ir_left_blocked, ir_right_blocked = _get_ir()
                else:
                    ir_left_blocked, ir_right_blocked = False, False
            except Exception:
                ir_left_blocked, ir_right_blocked = False, False

            # --- MPU6050 (calibrated) ---
            try:
                if _get_accel_xyz and _get_gyro_z:
                    ax, ay, az = _get_accel_xyz()
                    accel_x = ax - mpu_offset_ax
                    accel_y = ay - mpu_offset_ay
                    accel_z = az - mpu_offset_az
                    accel_magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
                    gyro_z_val = _get_gyro_z() - mpu_offset_gz
                else:
                    accel_magnitude = 1.0
            except Exception:
                accel_magnitude = 1.0

            # --- Telemetry log every 5th cycle ---
            if cycle % 5 == 0:
                try:
                    batt = _get_battery_voltage() if _get_battery_voltage else 0
                    curr = _get_current_sense() if _get_current_sense else 0
                    rpm = _get_rpm() if _get_rpm else 0
                    tlog(
                        laser_front_cm=sensor_front_cm,
                        sonar_front_cm=sonar_front_cm,
                        ir_left=ir_left_blocked, ir_right=ir_right_blocked,
                        accel_x=round(accel_x, 3), accel_y=round(accel_y, 3),
                        accel_z=round(accel_z, 3), accel_mag=round(accel_magnitude, 3),
                        gyro_z=round(gyro_z_val, 2),
                        rpm=rpm, battery_v=round(batt, 2),
                        current_a=round(curr, 2),
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

        # Drain buffer for latest frame
        for _ in range(4):
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
                rover_heading_deg = 0.0
                target_bearing_deg = 0.0
                bearing_valid = False
                vision_target_found = True
                vision_bbox_area = (x2 - x1) * (y2 - y1)
                user_paused = False
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

            cs = touch_current_scale
            focused_scales = sorted(set(
                [max(0.4, cs * 0.85), cs, min(5.0, cs * 1.2)]
            ))
            if touch_lost_count > 3:
                focused_scales = sorted(set(list(focused_scales) + TOUCH_SCALES))

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

            if best_val > TOUCH_CONF_THRESH and best_pos is not None:
                found = True
                touch_track_pos = best_pos
                touch_track_conf = best_val
                touch_current_scale = 0.7 * touch_current_scale + 0.3 * best_scale

            if found:
                tx, ty_p, tw_t, th_t = touch_track_pos
                vision_bbox_area = tw_t * th_t
                frame_cx = CAPTURE_W // 2
                obj_cx = tx + tw_t // 2
                raw_pull = ((obj_cx - frame_cx) / float(frame_cx)) * 100.0
                pull_delta = raw_pull - vision_steering_pull
                if abs(pull_delta) > VIS_PULL_MAX_JUMP:
                    pull_delta = VIS_PULL_MAX_JUMP if pull_delta > 0 else -VIS_PULL_MAX_JUMP
                vision_steering_pull = vision_steering_pull + VIS_PULL_ALPHA * pull_delta
                vision_target_found = True
                touch_lost_count = 0
                if not user_paused:
                    running = True
                clr = (0, 255, 0) if touch_track_conf > 0.6 else \
                      (0, 255, 255) if touch_track_conf > 0.4 else (0, 0, 255)
                cv2.rectangle(frame, (tx, ty_p), (tx + tw_t, ty_p + th_t), clr, 3)
                cv2.putText(frame, f"LOCK {touch_track_conf:.0%} s{touch_current_scale:.1f}x",
                            (tx, max(ty_p - 8, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, clr, 2)
                if touch_track_conf > 0.55 and 0.8 < best_scale < 1.3:
                    crop = frame[ty_p:ty_p + th_t, tx:tx + tw_t]
                    if crop.size > 0:
                        resized_crop = cv2.resize(crop, (base_tw, base_th))
                        touch_template = cv2.addWeighted(
                            touch_template, 0.85, resized_crop, 0.15, 0)
            else:
                touch_lost_count += 1
                touch_track_conf *= 0.85
                vision_target_found = False
                vision_steering_pull = 0.0
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
            if vision_mode == "SCANNING":
                net_frame = frame
                jpeg_quality = SCAN_JPEG_Q
                coord_scale_x = 1
                coord_scale_y = 1
            else:
                net_frame = cv2.resize(frame, (TRACK_W, TRACK_H))
                jpeg_quality = TRACK_JPEG_Q
                coord_scale_x = CAPTURE_W / TRACK_W
                coord_scale_y = CAPTURE_H / TRACK_H

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
                        x = int(nx * coord_scale_x)
                        y = int(ny * coord_scale_y)
                        w = int(nw * coord_scale_x)
                        h = int(nh * coord_scale_y)
                        vision_bbox_area = w * h
                        frame_center_x = CAPTURE_W // 2
                        raw_pull = (((x + w // 2) - frame_center_x) / float(frame_center_x)) * 100.0
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
# =================================================================
def _choose_swerve_dir():
    if ir_left_blocked and not ir_right_blocked:
        return 1
    if ir_right_blocked and not ir_left_blocked:
        return -1
    if bearing_valid:
        h_err = target_bearing_deg - rover_heading_deg
        h_err = (h_err + 180) % 360 - 180
        if h_err > 5:
            return 1
        elif h_err < -5:
            return -1
    return last_swerve_dir


def _target_speed_factor(bbox_area):
    if bbox_area <= 0:
        return 1.0
    if bbox_area >= TARGET_STOP_AREA:
        return 0.0
    if bbox_area >= TARGET_SLOW_AREA:
        t = (bbox_area - TARGET_SLOW_AREA) / (TARGET_STOP_AREA - TARGET_SLOW_AREA)
        factor = (1.0 - t) ** 2
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
            # --- HEADING INTEGRATION ---
            now = time.monotonic()
            dt = min(now - _heading_prev_time, 0.15)
            _heading_prev_time = now
            gz = gyro_z_val
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
            laser = sensor_front_cm if sensor_front_cm > 0 else 999
            sonar = sonar_front_cm if sonar_front_cm > 0 else 999
            front = min(laser, sonar)

            # -- 1. TARGET PURSUIT --
            tracking = vision_mode in ("TRACKING", "TOUCH_TRACKING")
            if tracking and vision_target_found:
                area = max(vision_bbox_area, 1)
                tgt_off = vision_steering_pull * 0.6
                if abs(gz) < BEARING_GZ_CALM:
                    target_bearing_deg = rover_heading_deg + tgt_off
                    target_bearing_deg = (target_bearing_deg + 180) % 360 - 180
                    bearing_valid = True

                tgt_factor = 1.0
                if tgt_factor <= 0.01:
                    raw_throttle = 0
                    raw_steering = vision_steering_pull * 1.2
                    status_msg = f">> AT TARGET: {selected_target_name}"
                else:
                    raw_throttle = MAX_SPEED * tgt_factor
                    steer_gain = 1.0 * tgt_factor + 0.6 * (1.0 - tgt_factor)
                    raw_steering = vision_steering_pull * steer_gain
                    pct = int(tgt_factor * 100)
                    status_msg = f">> HUNTING: {selected_target_name} [{pct}%]"
                prev_bbox_area = area

            elif vision_mode == "SEARCHING" or (tracking and not vision_target_found):
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

            # -- 2. COLLISION CHECK --
            if accel_magnitude > TILT_THRESHOLD_G or abs(gz) > GYRO_SPIKE_DPS:
                raw_throttle = 0
                raw_steering = 0
                avoid_state = "COLLISION"
                status_msg = f"!! IMPACT g={accel_magnitude:.1f} gz={gz:.0f}"
                drive_like_a_car(0, 0, 0)
                time.sleep(0.25)
                avoid_state = "CLEAR"
                continue

            # -- 3. SMOOTH SLALOM OBSTACLE AVOIDANCE --
            elif raw_throttle > 0:
                if not laser_is_healthy:
                    if ir_left_blocked and ir_right_blocked:
                        eff_front = OBS_EMERGENCY_CM
                    elif ir_left_blocked or ir_right_blocked:
                        eff_front = OBS_HARD_STEER_CM * 0.8
                    else:
                        eff_front = front
                else:
                    eff_front = front

                slalom_steer = 0.0
                slalom_speed = 1.0
                is_hunting = tracking and vision_target_found

                if is_hunting:
                    if eff_front <= OBS_EMERGENCY_CM:
                        slalom_speed = 0.0
                        avoid_state = "AT_TARGET"
                        obs_note = f" [STOPPED {eff_front:.0f}cm]"
                    elif eff_front <= OBS_HARD_STEER_CM:
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
                        slalom_speed = SLALOM_SPEED_MIN / 200.0
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN
                        avoid_state = "EMERGENCY"
                        obs_note = f" [EMER {eff_front:.0f}cm]"
                    elif eff_front <= OBS_HARD_STEER_CM:
                        slalom_speed = SLALOM_SPEED_MIN / 100.0
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN
                        avoid_state = "SLALOM"
                        obs_note = f" [HARD {eff_front:.0f}cm]"
                    elif eff_front <= OBS_SOFT_STEER_CM:
                        t = 1.0 - (eff_front - OBS_HARD_STEER_CM) / (OBS_SOFT_STEER_CM - OBS_HARD_STEER_CM)
                        slalom_speed = (SLALOM_SPEED_MIN / 100.0) + (1.0 - t) * (0.7 - SLALOM_SPEED_MIN / 100.0)
                        swerve = _choose_swerve_dir()
                        last_swerve_dir = swerve
                        slalom_steer = swerve * SLALOM_STEER_GAIN * t * 0.7
                        avoid_state = "SLALOM"
                        obs_note = f" [MED {eff_front:.0f}cm {int(t*100)}%]"
                    elif eff_front <= OBS_AWARE_CM:
                        t = 1.0 - (eff_front - OBS_SOFT_STEER_CM) / (OBS_AWARE_CM - OBS_SOFT_STEER_CM)
                        slalom_speed = 0.7 + (1.0 - t) * 0.3
                        swerve = _choose_swerve_dir()
                        slalom_steer = swerve * SLALOM_STEER_GAIN * t * 0.25
                        avoid_state = "AWARE"
                        obs_note = f" [AWARE {eff_front:.0f}cm]"
                    else:
                        avoid_state = "CLEAR"

                # IR side overrides
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

                raw_throttle *= slalom_speed

                if avoid_state in ("SLALOM", "EMERGENCY", "IR_BOXED"):
                    raw_steering = slalom_steer * 0.80 + raw_steering * 0.20
                elif avoid_state == "AWARE":
                    raw_steering = raw_steering * 0.55 + slalom_steer * 0.45

                if obs_note:
                    status_msg += obs_note

            # -- 4. CLAMP & SMOOTH --
            speed_ratio = min(1.0, abs(smoothed_throttle) / 100.0)
            max_steer = 100.0 - speed_ratio * (100.0 - SPEED_STEER_LIMIT)
            raw_steering = max(-max_steer, min(max_steer, raw_steering))

            steer_delta = raw_steering - smoothed_steering
            if abs(steer_delta) > STEER_RATE_LIMIT:
                steer_delta = STEER_RATE_LIMIT if steer_delta > 0 else -STEER_RATE_LIMIT
            smoothed_steering = max(-max_steer, min(max_steer, smoothed_steering + steer_delta))

            if raw_throttle < smoothed_throttle:
                thr_alpha = 0.8
            else:
                thr_alpha = 0.4
            smoothed_throttle = (1.0 - thr_alpha) * smoothed_throttle + thr_alpha * raw_throttle
            drive_like_a_car(smoothed_throttle, smoothed_steering, gz)

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
    """Launch worker threads. Hardware must be wired via init() first.

    MediaMTX must already be running (managed by main.py).
    The camera is taken over by this module (main camera must be released first).
    """
    global _active

    if not _initialized:
        raise RuntimeError("follow_target.init() must be called before start()")

    with _active_lock:
        if _active:
            return
        _active = True

    _shutdown_event.clear()

    # MPU6050 calibration (rover must be stationary)
    calibrate_mpu()

    # Worker threads
    _threads.clear()
    for target, name in [
        (headless_vision_loop, "Thread-FT-Vision"),
        (auto_pilot,           "Thread-FT-AutoPilot"),
        (sensor_polling_loop,  "Thread-FT-Sensors"),
    ]:
        t = threading.Thread(target=target, daemon=True, name=name)
        t.start()
        _threads.append(t)

    print("🎯 [Hunter] Started (using shared CarSystem)")


def stop():
    """Signal all threads to exit, release camera, stop motors.

    Does NOT call GPIO.cleanup() — that's owned by CarSystem in main.py.
    """
    global running, user_paused, _active
    global _picam2_ref, _ffmpeg_ref

    with _active_lock:
        if not _active:
            return
        _active = False

    running = False
    user_paused = False
    _shutdown_event.set()

    # Wait for threads
    for t in _threads:
        t.join(timeout=5.0)
    _threads.clear()

    # Stop motors via shared CarSystem
    if _car_system is not None:
        try:
            _car_system.stop()
        except Exception:
            pass

    # Release camera (this module took it over)
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

    # NOTE: No GPIO.cleanup() — CarSystem owns all motor GPIO
    print("🎯 [Hunter] Stopped (CarSystem GPIO preserved)")


def is_active() -> bool:
    return _active


def get_status() -> dict:
    try:
        batt = _get_battery_voltage() if _get_battery_voltage else 0.0
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
    global touch_requested
    nx = max(0.0, min(1.0, float(nx)))
    ny = max(0.0, min(1.0, float(ny)))
    touch_requested = (nx, ny)
    return {'ok': True, 'x': nx, 'y': ny}


def handle_command(cmd: str) -> dict:
    global running, user_paused, vision_mode, selected_target_name
    global touch_template, touch_track_pos, touch_track_conf, touch_lost_count
    global vision_target_found, vision_steering_pull, vision_bbox_area
    global motor_L_out, motor_R_out
    cmd = str(cmd).strip().lower()
    if cmd == 'start':
        # Only start hunting if a target has been selected
        if vision_mode in ("TOUCH_TRACKING", "TRACKING") and (
            touch_template is not None or selected_target_name
        ):
            user_paused = False
            running = True
            return {'ok': True, 'cmd': cmd}
        else:
            return {'ok': False, 'cmd': cmd, 'error': 'no target selected'}
    elif cmd == 'stop':
        # Completely stop: pause pursuit and halt motors immediately
        user_paused = True
        running = False
        motor_L_out = 0.0
        motor_R_out = 0.0
        drive_like_a_car(0, 0, 0)
        if _car_system is not None:
            try:
                _car_system.stop()
            except Exception:
                pass
    elif cmd == 'scan':
        vision_mode = "SCANNING"
        selected_target_name = ""
        user_paused = False
        running = False
    elif cmd == 'reset':
        # Full reset: stop car, clear target template and all tracking state
        user_paused = False
        running = False
        vision_mode = "IDLE"
        selected_target_name = ""
        touch_template = None
        touch_track_pos = None
        touch_track_conf = 0.0
        touch_lost_count = 0
        vision_target_found = False
        vision_steering_pull = 0.0
        vision_bbox_area = 0
        motor_L_out = 0.0
        motor_R_out = 0.0
        drive_like_a_car(0, 0, 0)
        if _car_system is not None:
            try:
                _car_system.stop()
            except Exception:
                pass
    return {'ok': True, 'cmd': cmd}


def get_template_jpg() -> bytes | None:
    if touch_template is None:
        return None
    ok, buf = cv2.imencode('.jpg', touch_template, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return buf.tobytes() if ok else None
