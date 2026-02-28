# =================================================================
# 1. LOGGING: structured telemetry CSV + minimal text log
# =================================================================
import logging
logging.basicConfig(
    filename='rover_debug.log',
    level=logging.WARNING,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# =================================================================
# 2. SAFE IMPORTS
# =================================================================
import RPi.GPIO as GPIO
import time
import curses
import threading
import csv
import os
import shutil
import socket as sock_mod
import cv2
import zmq
import json
import subprocess
import atexit
import signal
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.request

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
    'laser_front_cm', 'sonar_rear_cm',
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
    # Sweep
    'sweep_active', 'sweep_esc_angle', 'sweep_esc_dist',
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
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rover_logs', f'telem_{ts}.csv')
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

# MediaMTX configuration
MEDIAMTX_BIN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bin", "mediamtx")
MEDIAMTX_RTSP_PORT = 8554
MEDIAMTX_WEBRTC_PORT = 8889
MEDIAMTX_CONFIG_PATH = "/tmp/solam_mediamtx.yml"
_mediamtx_proc = None

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

TARGET_STOP_AREA = 400000  # Scaled for 1280x720 capture (was 100000 at 640x480)

# OBSTACLE AVOIDANCE TUNING
OBS_EMERGENCY_CM = 15       # Hard stop if anything is this close (cm)
OBS_CLOSE_CM = 40           # Slow down + steer away
OBS_AWARE_CM = 80           # Reduce speed slightly
OBS_REAR_MIN_CM = 20        # Rear collision threshold (for reverse maneuvers)
TILT_THRESHOLD_G = 1.8      # Accel magnitude that signals tip/collision (g)
OBS_STEER_GAIN = 60.0       # How aggressively to steer away from obstacles

# STATE VARIABLES
running = False
status_msg = "Ready"
curr_L = 0.0; curr_R = 0.0
smoothed_steering = 0.0
smoothed_throttle = 0.0

# SENSOR STATE (updated by sensor thread)
sensor_front_cm = -1        # Forward laser distance (cm)
sensor_rear_cm = -1         # Rear HC-SR04 ultrasonic (cm)
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

# Avoidance state (global for web UI visibility)
avoid_state = "CLEAR"

# TOUCH-TO-PURSUE STATE
touch_template = None        # BGR numpy array (the target crop)
touch_track_pos = None       # (x, y, w, h) in capture-frame coords
touch_track_conf = 0.0       # Template match confidence 0-1
touch_requested = None       # (norm_x, norm_y) from web UI click, or None
touch_lost_count = 0         # Consecutive frames without match
TOUCH_TMPL_SIZE = 80         # Template crop size (px at capture res)
TOUCH_SEARCH_MARGIN = 150    # ROI margin around last known position
TOUCH_CONF_THRESH = 0.35     # Min confidence for a valid match
TOUCH_LOST_MAX = 60          # Frames before declaring target lost (~3s)
WEB_PORT = 8080              # HTTP server port for hunter UI

# =================================================================
# MEDIAMTX LIFECYCLE MANAGEMENT
# =================================================================
def _is_port_open(port, timeout=0.3):
    try:
        with sock_mod.create_connection(("127.0.0.1", int(port)), timeout=timeout):
            return True
    except Exception:
        return False

def _resolve_mediamtx_bin():
    """Find a working mediamtx binary."""
    candidates = [
        MEDIAMTX_BIN,
        "/usr/local/bin/mediamtx",
        "/usr/bin/mediamtx",
        "/opt/mediamtx/mediamtx",
    ]
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    found = shutil.which("mediamtx")
    return found or ""

def _build_mediamtx_config():
    """Build a minimal MediaMTX YAML config that accepts RTSP publish from FFMPEG."""
    return (
        "logLevel: info\n"
        f"rtspAddress: :{MEDIAMTX_RTSP_PORT}\n"
        f"webrtcAddress: :{MEDIAMTX_WEBRTC_PORT}\n"
        "hls: no\n"
        "rtmp: no\n"
        "srt: no\n"
        "paths:\n"
        "  all_others:\n"
    )

def start_mediamtx():
    """Start MediaMTX process. Returns True if running (managed or external)."""
    global _mediamtx_proc

    if _is_port_open(MEDIAMTX_RTSP_PORT):
        return True

    mtx_bin = _resolve_mediamtx_bin()
    if not mtx_bin:
        logging.error("MediaMTX binary not found")
        return False

    try:
        with open(MEDIAMTX_CONFIG_PATH, "w") as f:
            f.write(_build_mediamtx_config())
    except Exception as e:
        logging.error(f"MediaMTX config write failed: {e}")
        return False

    try:
        _mediamtx_proc = subprocess.Popen(
            [mtx_bin, MEDIAMTX_CONFIG_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except Exception as e:
        logging.error(f"MediaMTX start failed: {e}")
        return False

    def _drain():
        try:
            for line in _mediamtx_proc.stdout:
                pass  # silently consume
        except Exception:
            pass
    threading.Thread(target=_drain, daemon=True, name="Thread-MediaMTX-Log").start()

    deadline = time.time() + 6.0
    while time.time() < deadline:
        if _mediamtx_proc.poll() is not None:
            if _is_port_open(MEDIAMTX_RTSP_PORT):
                return True
            return False
        if _is_port_open(MEDIAMTX_RTSP_PORT):
            return True
        time.sleep(0.2)

    stop_mediamtx()
    return False

def stop_mediamtx():
    """Gracefully terminate the managed MediaMTX process."""
    global _mediamtx_proc
    if _mediamtx_proc is None:
        return
    try:
        _mediamtx_proc.terminate()
        _mediamtx_proc.wait(timeout=3.0)
    except Exception:
        try:
            _mediamtx_proc.kill()
        except Exception:
            pass
    _mediamtx_proc = None

atexit.register(stop_mediamtx)

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
# WEB UI HTTP SERVER (touch-to-pursue + telemetry)
# =================================================================
class RoverHandler(BaseHTTPRequestHandler):
    """Serve hunter UI, handle touch clicks, proxy WHEP, return telemetry."""

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.end_headers()

    def do_GET(self):
        if self.path in ('/', '/hunter', '/hunter.html'):
            fpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'public', 'hunter.html')
            try:
                with open(fpath, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self._cors()
                self.end_headers()
                self.wfile.write(content)
            except FileNotFoundError:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b'hunter.html not found')

        elif self.path == '/api/status':
            try:
                batt = get_battery_voltage()
            except Exception:
                batt = 0.0
            data = {
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
                    'front': round(sensor_front_cm, 1),
                    'rear': round(sensor_rear_cm, 1),
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
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        elif self.path.startswith('/api/template.jpg'):
            if touch_template is not None:
                _, buf = cv2.imencode('.jpg', touch_template,
                                      [cv2.IMWRITE_JPEG_QUALITY, 90])
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self._cors()
                self.end_headers()
                self.wfile.write(buf.tobytes())
            else:
                self.send_response(204)
                self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global touch_requested, running, vision_mode, selected_target_name
        content_len = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_len) if content_len else b''

        if self.path == '/api/touch':
            data = json.loads(body)
            nx = float(data.get('x', 0.5))
            ny = float(data.get('y', 0.5))
            touch_requested = (max(0.0, min(1.0, nx)),
                               max(0.0, min(1.0, ny)))
            resp = json.dumps({'ok': True, 'x': nx, 'y': ny}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(resp)

        elif self.path == '/api/command':
            data = json.loads(body)
            cmd = data.get('cmd', '')
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
            resp = json.dumps({'ok': True, 'cmd': cmd}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(resp)

        elif self.path == '/api/whep':
            try:
                req = urllib.request.Request(
                    f'http://127.0.0.1:{MEDIAMTX_WEBRTC_PORT}/rover/whep',
                    data=body,
                    headers={'Content-Type': 'application/sdp'},
                    method='POST',
                )
                resp = urllib.request.urlopen(req, timeout=5)
                answer = resp.read()
                self.send_response(201)
                self.send_header('Content-Type', 'application/sdp')
                loc = resp.headers.get('Location', '')
                if loc:
                    self.send_header('Location', loc)
                self._cors()
                self.end_headers()
                self.wfile.write(answer)
            except Exception as ex:
                self.send_response(502)
                self._cors()
                self.end_headers()
                self.wfile.write(str(ex).encode())

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        pass  # Silence HTTP request logging


def _start_web_server():
    """Run the HTTP server for the hunter web UI."""
    server = HTTPServer(('0.0.0.0', WEB_PORT), RoverHandler)
    server.serve_forever()

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
# HARDWARE SETUP
# =================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pico = init_pico_reader()

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

calibrate_mpu()

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

motor_L_out = 0.0
motor_R_out = 0.0

def drive_like_a_car(throttle, steering, current_gz):
    global motor_L_out, motor_R_out
    target_yaw = (steering / 100.0) * MAX_YAW_RATE
    corr_steer = max(-100, min(100, steering + ((target_yaw - current_gz) * GYRO_KP)))
    L, R = throttle + corr_steer, throttle - corr_steer
    motor_L_out, motor_R_out = L, R
    ramp_motors(abs(L), abs(R), (1 if L>=0 else -1), (1 if R>=0 else -1))

# =================================================================
# SENSOR POLLING THREAD (front laser, sonar, IR, MPU6050)
# Laser sweep is ON-DEMAND: only triggered when obstacle is detected
# =================================================================
def sensor_polling_loop():
    """Read sensors continuously; laser sweep only when requested."""
    global sensor_front_cm, sensor_rear_cm
    global ir_left_blocked, ir_right_blocked
    global accel_x, accel_y, accel_z, accel_magnitude, gyro_z_val
    global sensor_status
    global sweep_active, sweep_escape_angle, sweep_escape_dist
    global sweep_data, sweep_requested

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
    while True:
        try:
            # --- Forward laser (center, servo parked) ---
            if sensors and not sweep_active:
                fwd = sensors.read_laser_cm()
                sensor_front_cm = fwd if fwd > 0 else 999

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

            # --- Rear sonar every 3rd cycle ---
            if sensors and cycle % 3 == 0:
                try:
                    rear = sensors.get_rear_sonar_distance()
                    sensor_rear_cm = rear if rear > 0 else 999
                except Exception:
                    pass

            # --- IR proximity ---
            try:
                ir_left_blocked, ir_right_blocked = get_ir_sensors()
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
                        sonar_rear_cm=sensor_rear_cm,
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

    try:
        os.environ["LIBCAMERA_LOG_LEVELS"] = "*:ERROR"
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (CAPTURE_W, CAPTURE_H), "format": "BGR888"})
        picam2.configure(config)
        picam2.start()
        camera_status = "[OK] ONLINE"
        atexit.register(picam2.close)
        atexit.register(picam2.stop)
        
        ffmpeg_cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', f'{CAPTURE_W}x{CAPTURE_H}', '-r', '30', '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2.5M', '-f', 'rtsp', MTX_URL
        ]
        ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        atexit.register(lambda: ffmpeg_process.kill())

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.connect(f"tcp://{INTEL_CPU_IP}:{ZMQ_PORT}")

    except Exception as e:
        logging.error(f"Vision init FATAL: {e}", exc_info=True)
        camera_status = "[!!] INIT FAILED"
        return

    while True:
        try:
            frame = picam2.capture_array()
        except Exception:
            time.sleep(0.1)
            continue

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
                vision_mode = "TOUCH_TRACKING"
                selected_target_name = "TOUCH TARGET"
                vision_target_found = True
                vision_bbox_area = (x2 - x1) * (y2 - y1)
                running = True
                tlog(touch_x=round(nx, 3), touch_y=round(ny, 3),
                     vision_mode="TOUCH_TRACKING",
                     status_msg="TOUCH TARGET ACQUIRED")

        # --- TOUCH TEMPLATE TRACKING (local, no ZMQ needed) ---
        if vision_mode == "TOUCH_TRACKING" and touch_template is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_tmpl = cv2.cvtColor(touch_template, cv2.COLOR_BGR2GRAY)
            th, tw = gray_tmpl.shape[:2]
            found = False

            # ROI search around last known position
            if touch_track_pos is not None:
                lx, ly, lw, lh = touch_track_pos
                cx, cy = lx + lw // 2, ly + lh // 2
                m = TOUCH_SEARCH_MARGIN
                sx1 = max(0, cx - tw // 2 - m)
                sy1 = max(0, cy - th // 2 - m)
                sx2 = min(CAPTURE_W, cx + tw // 2 + m)
                sy2 = min(CAPTURE_H, cy + th // 2 + m)
                if sx2 - sx1 > tw + 4 and sy2 - sy1 > th + 4:
                    roi = gray[sy1:sy2, sx1:sx2]
                    res = cv2.matchTemplate(roi, gray_tmpl, cv2.TM_CCOEFF_NORMED)
                    _, mx_val, _, mx_loc = cv2.minMaxLoc(res)
                    if mx_val > TOUCH_CONF_THRESH:
                        found = True
                        touch_track_pos = (mx_loc[0] + sx1, mx_loc[1] + sy1, tw, th)
                        touch_track_conf = mx_val

            # Full-frame fallback (downscaled for speed)
            if not found:
                small = cv2.resize(gray, (CAPTURE_W // 2, CAPTURE_H // 2))
                small_tmpl = cv2.resize(gray_tmpl, (max(tw // 2, 4), max(th // 2, 4)))
                if small_tmpl.shape[0] > 4 and small_tmpl.shape[1] > 4:
                    res = cv2.matchTemplate(small, small_tmpl, cv2.TM_CCOEFF_NORMED)
                    _, mx_val, _, mx_loc = cv2.minMaxLoc(res)
                    if mx_val > TOUCH_CONF_THRESH:
                        found = True
                        touch_track_pos = (mx_loc[0] * 2, mx_loc[1] * 2, tw, th)
                        touch_track_conf = mx_val

            if found:
                tx, ty_p, tw_t, th_t = touch_track_pos
                vision_bbox_area = tw_t * th_t
                frame_cx = CAPTURE_W // 2
                obj_cx = tx + tw_t // 2
                vision_steering_pull = ((obj_cx - frame_cx) / float(frame_cx)) * 100.0
                vision_target_found = True
                touch_lost_count = 0
                clr = (0, 255, 0) if touch_track_conf > 0.6 else \
                      (0, 255, 255) if touch_track_conf > 0.4 else (0, 0, 255)
                cv2.rectangle(frame, (tx, ty_p), (tx + tw_t, ty_p + th_t), clr, 3)
                cv2.putText(frame, f"LOCK {touch_track_conf:.0%}",
                            (tx, max(ty_p - 8, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, clr, 2)
                # Slowly refresh template to adapt to appearance changes
                if touch_track_conf > 0.55:
                    crop = frame[ty_p:ty_p + th_t, tx:tx + tw_t]
                    if crop.shape == touch_template.shape:
                        touch_template = cv2.addWeighted(
                            touch_template, 0.8, crop, 0.2, 0)
            else:
                touch_lost_count += 1
                touch_track_conf *= 0.92
                vision_target_found = False
                vision_steering_pull = 0.0
                if touch_lost_count > TOUCH_LOST_MAX:
                    vision_mode = "IDLE"
                    selected_target_name = ""
                    touch_template = None
                    touch_track_pos = None
                    running = False
                    status_msg = "TARGET LOST"
                elif touch_track_pos:
                    tx, ty_p, tw_t, th_t = touch_track_pos
                    cv2.rectangle(frame, (tx, ty_p), (tx + tw_t, ty_p + th_t),
                                  (0, 0, 255), 1)
                    cv2.putText(frame, "SEARCHING",
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
                        vision_steering_pull = (((x + w // 2) - frame_center_x) / float(frame_center_x)) * 100.0
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
# SMART DRIVER BRAIN (OBSTACLE-AWARE HUNTER)
# =================================================================
def auto_pilot():
    global running, status_msg, smoothed_steering, smoothed_throttle
    global sweep_requested, avoid_state

    escape_steer = 0.0           # Steering override during escape
    escape_timer = 0             # Cycles remaining in escape maneuver
    ESCAPE_CYCLES = 20           # Drive toward escape for ~1 second (20x50ms)

    while True:
        try:
            if not running:
                drive_like_a_car(0, 0, 0)
                avoid_state = "CLEAR"
                time.sleep(0.1)
                continue

            gz = get_gyro_z()
            raw_throttle = 0.0
            raw_steering = 0.0
            obs_note = ""

            # -- 1. VISION-BASED TARGET PURSUIT --
            tracking = vision_mode in ("TRACKING", "TOUCH_TRACKING")
            if tracking and vision_target_found:
                if vision_bbox_area > TARGET_STOP_AREA:
                    raw_throttle = 0
                    raw_steering = vision_steering_pull * 1.5
                    status_msg = f">> AT TARGET: {selected_target_name}"
                else:
                    raw_throttle = MAX_SPEED
                    raw_steering = vision_steering_pull * 1.8
                    status_msg = f">> HUNTING: {selected_target_name}"
            elif vision_mode == "SEARCHING" or (tracking and not vision_target_found):
                raw_throttle = 0; raw_steering = 0
                status_msg = f">> SEARCHING: {selected_target_name}..."
            else:
                raw_throttle = 0; raw_steering = 0
                status_msg = "IDLE"

            # -- 2. MPU6050 TIP / COLLISION CHECK --
            if accel_magnitude > TILT_THRESHOLD_G:
                raw_throttle = 0
                raw_steering = 0
                avoid_state = "CLEAR"
                status_msg = "!! TILT/COLLISION STOP"

            # -- 3. OBSTACLE AVOIDANCE STATE MACHINE (only when moving) --
            elif raw_throttle > 0:
                front = sensor_front_cm if sensor_front_cm > 0 else 999

                # IR emergency: both sides blocked
                if ir_left_blocked and ir_right_blocked:
                    raw_throttle = 0
                    raw_steering = 0
                    avoid_state = "CLEAR"
                    obs_note = " [IR-BLOCKED]"

                elif avoid_state == "CLEAR":
                    # Normal driving -- check if obstacle detected
                    if front < OBS_EMERGENCY_CM:
                        # Too close! Emergency stop, request sweep
                        raw_throttle = 0
                        raw_steering = 0
                        sweep_requested = True
                        avoid_state = "SWEEP_WAIT"
                        obs_note = f" [E-STOP {front:.0f}cm -> SWEEP]"

                    elif front < OBS_CLOSE_CM:
                        # Obstacle ahead, slow down and request sweep
                        raw_throttle *= 0.2
                        sweep_requested = True
                        avoid_state = "SWEEP_WAIT"
                        obs_note = f" [OBS {front:.0f}cm -> SWEEP]"

                    elif front < OBS_AWARE_CM:
                        # Getting closer, reduce speed
                        raw_throttle *= 0.7
                        obs_note = f" [SLOW {front:.0f}cm]"

                    # IR side nudge (gentle, no sweep needed)
                    if ir_left_blocked and not ir_right_blocked:
                        raw_steering += OBS_STEER_GAIN * 0.4
                        obs_note += " [IR-L]"
                    elif ir_right_blocked and not ir_left_blocked:
                        raw_steering -= OBS_STEER_GAIN * 0.4
                        obs_note += " [IR-R]"

                elif avoid_state == "SWEEP_WAIT":
                    # Waiting for sweep to complete
                    raw_throttle = 0
                    raw_steering = 0
                    obs_note = " [SWEEPING...]"

                    if not sweep_active and not sweep_requested:
                        # Sweep done! Read escape route
                        esc_angle = sweep_escape_angle   # -60..+60
                        esc_dist = sweep_escape_dist
                        if esc_dist < OBS_EMERGENCY_CM:
                            # No clear path at all -- stay stopped
                            obs_note = f" [NO ESCAPE! {esc_dist:.0f}cm]"
                            avoid_state = "CLEAR"  # Will re-trigger on next cycle
                        else:
                            # Convert escape angle to steering (-100..+100)
                            # angle: -60=hard left, 0=straight, +60=hard right
                            escape_steer = (esc_angle / 60.0) * 100.0
                            escape_timer = ESCAPE_CYCLES
                            avoid_state = "ESCAPING"
                            obs_note = f" [ESCAPE {esc_angle}deg]"

                elif avoid_state == "ESCAPING":
                    # Drive toward escape route for a fixed duration
                    raw_throttle = MAX_SPEED * 0.5  # Half speed during escape
                    raw_steering = escape_steer
                    escape_timer -= 1
                    obs_note = f" [ESC {escape_timer} {escape_steer:+.0f}deg]"

                    # Check if path ahead is now clear
                    if front > OBS_AWARE_CM or escape_timer <= 0:
                        avoid_state = "CLEAR"
                        obs_note = " [PATH CLEAR]"
                if obs_note:
                    status_msg += obs_note

            # -- 4. CLAMP & SMOOTH --
            raw_steering = max(-100, min(100, raw_steering))
            smoothed_steering = (0.3 * smoothed_steering) + (0.7 * raw_steering)
            smoothed_throttle = (0.5 * smoothed_throttle) + (0.5 * raw_throttle)
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
                sonar_rear_cm=round(sensor_rear_cm, 1),
                ir_left=int(ir_left_blocked),
                ir_right=int(ir_right_blocked),
                accel_mag=round(accel_magnitude, 2),
                gyro_z=round(gz, 1),
                sweep_active=int(sweep_active),
                sweep_esc_angle=sweep_escape_angle,
                sweep_esc_dist=round(sweep_escape_dist, 1),
                status_msg=status_msg,
            )
            time.sleep(0.05)
        except Exception as e:
            logging.error(f"AutoPilot error: {e}")
            avoid_state = "CLEAR"
            time.sleep(0.5)

# Start MediaMTX first, then threads
if not start_mediamtx():
    logging.warning("MediaMTX failed to start -- RTSP streaming may not work")

threading.Thread(target=_start_web_server, daemon=True, name="Thread-WebUI").start()
threading.Thread(target=headless_vision_loop, daemon=True, name="Thread-Vision").start()
threading.Thread(target=auto_pilot, daemon=True, name="Thread-AutoPilot").start()
threading.Thread(target=sensor_polling_loop, daemon=True, name="Thread-Sensors").start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================
def main(stdscr):
    global running, vision_mode, selected_target_name
    try:
        stdscr.nodelay(True)
        curses.curs_set(0)
        stdscr.clear()
        
        while True:
            try:
                stdscr.erase()
                max_y, max_x = stdscr.getmaxyx()
                
                # Header
                stdscr.addstr(0, 0, "=== MAX POWER AI HUNTER ===", curses.A_BOLD)
                stdscr.addstr(1, 0, f"WEB UI: http://<IP>:{WEB_PORT}/")
                
                # Status & Camera
                status_line = f"STATUS: {status_msg} | CAM: {camera_status}"
                stdscr.addstr(2, 0, status_line[:max_x-1])
                
                # Battery
                try:
                    volts = get_battery_voltage()
                    amps = get_current_sense()
                    power_line = f"VOLTS: {volts:.2f}V | AMPS: {amps:.2f}A"
                    stdscr.addstr(4, 0, power_line[:max_x-1])
                except Exception as e:
                    stdscr.addstr(4, 0, "VOLTS: ERR | AMPS: ERR")
                    logging.error(f"UI Thread: Sensor read error: {e}")
                
                # Obstacle sensor readout
                obs_line = (f"FRONT:{sensor_front_cm:4.0f}cm  "
                            f"REAR:{sensor_rear_cm:4.0f}cm  "
                            f"IR:{'L' if ir_left_blocked else '.'}{'R' if ir_right_blocked else '.'}  "
                            f"G:{accel_magnitude:.1f}")
                stdscr.addstr(5, 0, obs_line[:max_x-1])
                
                # Sweep / escape info
                if sweep_active:
                    sweep_line = f"SENSORS: {sensor_status} | SWEEP IN PROGRESS..."
                elif sweep_escape_dist > 0:
                    sweep_line = (f"SENSORS: {sensor_status} | "
                                  f"ESCAPE: {sweep_escape_angle:+d}deg @ {sweep_escape_dist:.0f}cm")
                else:
                    sweep_line = f"SENSORS: {sensor_status}"
                stdscr.addstr(6, 0, sweep_line[:max_x-1])
                
                # Vision mode & targets
                if vision_mode == "WAITING":
                    stdscr.addstr(8, 0, "[*] SELECT TARGET:", curses.A_BOLD)
                    for i, name in enumerate(available_targets.keys()):
                        if 9 + i < max_y - 2:
                            stdscr.addstr(9 + i, 2, f"[{i+1}] {name.upper()}")
                elif selected_target_name:
                    target_line = f"[*] MISSION: {selected_target_name.upper()}"
                    stdscr.addstr(8, 0, target_line[:max_x-1])
                
                # Controls (at bottom)
                ctrl_line = "[S]tart [SPACE]Stop [R]eScan [Q]uit"
                if max_y > 14:
                    stdscr.addstr(max_y - 2, 0, ctrl_line[:max_x-1])
                
                # CRITICAL: Flush to screen
                stdscr.refresh()
                
                # Non-blocking input
                key = stdscr.getch()
                if key != -1:  # -1 means no input available
                    if key == ord('q'): 
                        break
                    elif key == ord('s'): 
                        running = True
                    elif key == ord(' '): 
                        running = False
                    elif key == ord('r'):
                        vision_mode = "SCANNING"
                        selected_target_name = ""
                        running = False
                    
                    if vision_mode == "WAITING" and ord('1') <= key <= ord('9'):
                        idx = key - ord('1')
                        if idx < len(available_targets):
                            selected_target_name = list(available_targets.keys())[idx]
                            vision_mode = "TRACKING"
                            running = True
                
                time.sleep(0.1)
                
            except curses.error:
                # Terminal resized or other curses error
                time.sleep(0.2)
                continue
            except Exception as e:
                logging.error(f"UI render error: {e}", exc_info=False)
                time.sleep(0.2)
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        pass  # Ctrl+C
        GPIO.cleanup()
    except Exception as e:
        logging.error(f"Fatal: {e}")
        GPIO.cleanup()
