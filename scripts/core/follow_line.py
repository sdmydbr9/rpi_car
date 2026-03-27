#!/usr/bin/env python3
"""
follow_line.py — Line-following mode with web interface.

Tilts the camera servo downward, detects any-color line using
per-channel local-contrast analysis, and provides a web UI with
a live annotated stream + Start/Stop controls.

Uses MPU gyroscope for heading correction.  Steering is Ackermann
(front servo) with 2WD rear drive — both rear motors always run at
the same speed.

The "Start" button stays greyed-out until a line is confidently
detected.  Once started the rover follows the line at low speed
until the line disappears, then brakes automatically.

Usage:
    python3 scripts/core/follow_line.py [--port 8080] [--tilt 120] [--speed 25]

Web UI:  http://<pi-ip>:<port>
"""

# ── stdlib ──────────────────────────────────────────────
import sys, os, time, threading, signal, argparse, io, logging, math
from collections import deque
from datetime import datetime

# Resolve project paths so imports work from anywhere
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)
for p in (SCRIPT_DIR, SCRIPTS_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

# ── logging setup ──────────────────────────────────────
LOG_DIR = os.path.join(PROJECT_ROOT, "rover_logs")
os.makedirs(LOG_DIR, exist_ok=True)
_log_file = os.path.join(LOG_DIR, f"line_follow_{datetime.now():%Y%m%d_%H%M%S}.log")

log = logging.getLogger("line_follow")
log.setLevel(logging.DEBUG)
_fh = logging.FileHandler(_log_file)
_fh.setLevel(logging.DEBUG)
_ch = logging.StreamHandler()
_ch.setLevel(logging.INFO)
_fmt = logging.Formatter("%(asctime)s %(levelname)-5s %(message)s", datefmt="%H:%M:%S")
_fh.setFormatter(_fmt)
_ch.setFormatter(_fmt)
log.addHandler(_fh)
log.addHandler(_ch)
log.propagate = False
log.info("Log file: %s", _log_file)

# ── third-party ────────────────────────────────────────
import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template

# ── project hardware ───────────────────────────────────
from pico_sensor_reader import (
    init_pico_reader,
    send_pan_tilt,
    get_gyro_z,
    get_magnetometer,
    is_pico_fresh,
)
from motor import CarSystem

# ── camera (Picamera2 on RPi, fallback to OpenCV) ─────
try:
    from picamera2 import Picamera2
    _USE_PICAMERA2 = True
except ImportError:
    _USE_PICAMERA2 = False
    print("⚠️  Picamera2 not found — falling back to OpenCV VideoCapture")


# ===================================================================
#  CONFIGURATION (tunable via CLI or constants)
# ===================================================================
PAN_CENTER   = 90
DEFAULT_TILT = 120       # 90=horizontal, 120=fully down
CAMERA_W     = 640
CAMERA_H     = 480
JPEG_QUALITY = 65

# Line detection
ROI_TOP_FRAC       = 0.35    # ignore top 35 % of frame (above ground)
MIN_CONTOUR_AREA   = 500     # px² — reject noise
LINE_CONFIRM_FRAMES = 5      # consecutive detections before "ready"
LINE_READY_DROP_MISSES = 3   # misses allowed before ready state drops back to SEARCHING
CONTRAST_THRESH    = 25       # per-channel local-contrast threshold

# Driving
DEFAULT_SPEED  = 38           # base PWM %
STEERING_KP    = 0.16
STEERING_KI    = 0.020
STEERING_KD    = 0.035
MAX_STEER      = 35           # clamp
INTEGRAL_MAX   = 10.0
LINE_LOST_TIMEOUT = 0.5       # seconds with no line → stop
CENTER_DEADBAND_PX = 12
ERROR_FILTER_ALPHA = 0.22
LOOKAHEAD_FILTER_ALPHA = 0.18
STEER_SLEW_RATE = 180.0       # deg/sec max steering change

# Static motor trim — compensates for different-motor torque imbalance.
# Negative = nudge left (counteract right-drift).  Tune with --bias.
# Start at 0; if car consistently drifts right, decrease in steps of 1-2.
DEFAULT_MOTOR_BIAS = 0.0

# Gyro rate-damping correction
# Opposes instantaneous yaw rate directly.  Fades to zero as vision error
# grows so that intentional line-following turns are not fought.
# With Ackermann steering, the correction adjusts the servo angle directly
# (in degrees) instead of applying per-side PWM differential.
GYRO_KP         = 0.18  # steering-angle degrees per °/s of yaw-rate error
GYRO_MAX_CORR   = 6.0   # max steering correction (degrees)
GYRO_DEADZONE   = 4.0   # °/s
GYRO_FADE_STEER = 20.0  # fade gyro damping as steering angle grows

# Magnetometer-assisted heading hold.
# Vision remains primary; fused heading only helps stabilise straight travel
# and carry through brief weak/noisy detections without fighting real turns.
# With Ackermann steering, the correction adjusts the servo angle directly.
MAG_HEADING_KP        = 0.10   # steering-angle degrees per degree of heading error
MAG_HEADING_MAX_CORR  = 5.0    # max steering correction (degrees)
MAG_BLEND_ALPHA       = 0.98   # complementary filter gyro weight
MAG_LOCK_CENTER_PX    = 18     # capture target heading only when line is well centred
MAG_FADE_PX           = 65     # fade heading assist out during larger visual turns
MAG_MIN_FIELD         = 1e-3   # ignore near-zero field vectors

# Rover geometry and motion model.
# Ackermann front-wheel steering with 2WD rear drive.
WHEEL_DIAMETER_M     = 0.065
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
ROVER_TRACK_WIDTH_M  = 0.14

# Line lock — spatial + colour gate applied once a line is confirmed.
# Prevents the rover jumping to a different nearby line mid-follow.
LOCK_SPATIAL_MARGIN = 120   # px: contour centroid must be within this of the locked cx
LOCK_COLOUR_TOL     = 40    # 0-255: mean BGR of locked contour must stay within this
LOCK_TEMPLATE_MIN_SCORE = 2.2
LOCK_TEMPLATE_STRICT_SCORE = 2.8
LOCK_HUE_TOL = 22.0
LOCK_SAT_TOL = 70.0
LOCK_VAL_TOL = 90.0
LOCK_THICKNESS_FRAC = 0.75
LOCK_ROW_HIT_FRAC = 0.70
LOCK_ELONGATION_FRAC = 0.90
LOCK_BLEND_ALPHA = 0.18
MEMORY_BLEND_ALPHA = 0.08

# Path trail drawn on annotated frame (rolling centroid history)
PATH_HISTORY_LEN = 60       # how many centroid samples to keep (~1.5 s at 40 fps)

# Spine: number of horizontal scan rows used to trace the line ahead of the rover
SPINE_ROWS = 12

# Keep the camera servo fixed for the entire follow-line run.
FIXED_PAN = PAN_CENTER

# Curve lookahead — blends near centroid error with far spine error so the rover
# begins turning before the centroid itself drifts, giving smooth curve tracking.
LOOKAHEAD_WEIGHT = 0.28   # 0 = pure centroid, 1 = pure far-spine lookahead
LOOKAHEAD_ROWS   = 3      # how many far spine points to average for lookahead

# Curve-adaptive speed: reduce drive speed proportionally when cornering hard.
# At |steer|=MAX_STEER the speed is multiplied by CURVE_SPEED_MIN_FRAC.
# e.g. base_speed=20, MIN_FRAC=0.50  →  speed on tightest curve = 10 %
CURVE_SPEED_MIN_FRAC = 0.35

# Curve estimation from the extracted line spine.
CURVE_FIT_MIN_POINTS   = 4
CURVE_NEAR_FRAC        = 0.18
CURVE_MID_FRAC         = 0.48
CURVE_FAR_FRAC         = 0.78
CURVE_MAX_NORM         = 3.5
PATH_HEADING_STEER_KP  = 0.55
PATH_CURVATURE_STEER_KP = 7.5

# Path-dynamics fusion: use vision-estimated curve, gyro yaw
# rate, and fused compass heading together.
TARGET_YAW_HEADING_GAIN    = 0.60
TARGET_YAW_CURVATURE_GAIN  = 22.0
MAX_TARGET_YAW_RATE_DPS    = 75.0
YAW_RATE_BLEND_MAX         = 0.45
TARGET_HEADING_BLEND       = 0.22

# Curve recovery: when the line briefly disappears on a bend, keep a cautious
# turn for a short time and relax the detector just enough to reacquire.
RECOVERY_RELAX_WINDOW = 0.35
RECOVERY_STEER_GAIN = 1.15
RECOVERY_MIN_STEER = 10.0
RECOVERY_SPEED_MIN_FRAC = 0.28
RECOVERY_SPEED_MAX_FRAC = 0.52
RECOVERY_TEMPLATE_MIN_SCORE = 1.6
RECOVERY_TEMPLATE_STRICT_SCORE = 2.1
LOCK_STEER_MARGIN_GAIN = 2.4
RECOVERY_YAW_RATE_STEER_GAIN = 0.35

# Detection pipeline tuning
LOCKED_ROI_HALF_WIDTH = 150
ROI_RESIZE_SCALE = 0.5
FAST_PATH_CONFIDENCE_FRAMES = 5

# Steering / path feedforward tuning
LOOKAHEAD_CURVATURE_GAIN = 0.30
FEEDFORWARD_KP = 0.055
PREDICTION_FRAMES_AHEAD = 2.0
ACTIVE_CORNERING_YAW_RATE_DPS = 18.0
ACTIVE_CORNERING_CURVATURE = 0.35

# Recovery cadence
RECOVERY_BRAKE_TOTAL_FRAMES = 2

# Sensor bias estimation
GYRO_BIAS_ALPHA = 0.03

# Morphology kernels for cleaning up the binary mask
_MORPH_KERNEL_SEARCH = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
_MORPH_KERNEL_LOCKED = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


# ===================================================================
#  LINE FOLLOWER CORE
# ===================================================================
class LineFollower:
    """Detect and follow any-colour line using a downward-facing camera."""

    # ── states ──
    SEARCHING     = "SEARCHING"
    LINE_DETECTED = "LINE_DETECTED"
    FOLLOWING     = "FOLLOWING"
    LINE_LOST     = "LINE_LOST"
    STOPPED       = "STOPPED"

    def __init__(self, tilt=DEFAULT_TILT, speed=DEFAULT_SPEED,
                 motor_bias=DEFAULT_MOTOR_BIAS,
                 use_mpu=True):
        self._tilt = tilt
        self._base_speed = speed
        self._motor_bias = motor_bias
        self._use_mpu      = use_mpu
        self._use_picamera2 = _USE_PICAMERA2
        log.info("Sensors: MPU=%s", "ON" if use_mpu else "OFF")

        # --- hardware ---
        init_pico_reader()
        send_pan_tilt(FIXED_PAN, self._tilt)
        time.sleep(0.4)
        log.info("Camera fixed at pan=%d° tilt=%d°", FIXED_PAN, self._tilt)

        self._car = CarSystem()
        log.info("Motor system ready (Ackermann steering via Pico UART)")

        # --- camera ---
        if self._use_picamera2:
            self._cam = Picamera2()
            cfg = self._cam.create_still_configuration(
                main={"size": (CAMERA_W, CAMERA_H), "format": "RGB888"},
                buffer_count=4,
            )
            self._cam.configure(cfg)
            self._cam.start()
        else:
            self._cam = cv2.VideoCapture(0)
            self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
            self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
        time.sleep(0.5)
        log.info("Camera streaming (image flipped 180°)")

        # --- shared state ---
        self._lock = threading.Lock()
        self.state = self.SEARCHING
        self._line_cx = CAMERA_W / 2.0
        self._line_cy = CAMERA_H / 2.0
        self._confirm_count = 0
        self._ready_miss_count = 0
        self._line_lost_since = 0.0
        self._following = False
        self._annotated_frame = None
        self._running = True

        # --- line-lock state ---
        # Once a line is confirmed we record its centroid and mean colour so that
        # we can reject unrelated blobs that wander into the camera view.
        self._locked_cx     = None   # int   — locked centroid-x (pixels)
        self._locked_colour = None   # np.ndarray shape(3,) — mean BGR of locked contour
        self._locked_signature = None
        self._line_memory = None
        self._recent_template_scores: deque = deque(maxlen=FAST_PATH_CONFIDENCE_FRAMES)
        self._detection_mode = "full"
        self._template_confidence = 0.0
        self._last_template_score = 0.0
        self._mask_scratch = {}

        # --- path history (rolling centroid trail) ---
        self._path_history: deque = deque(maxlen=PATH_HISTORY_LEN)
        # Spine = list of (x, y) tuples tracing the line ahead, shared for the UI
        self._spine: list = []

        # --- pan servo state ---
        self._pan_angle       = float(FIXED_PAN)   # fixed pan angle (degrees)
        self._lookahead_error = 0.0                # last lookahead error (for CSV)
        self._filtered_error = 0.0
        self._filtered_lookahead = 0.0
        self._filtered_steer = 0.0
        self._last_control_ts = time.monotonic()
        self._last_seen_error = 0.0
        self._last_seen_lookahead = 0.0
        self._last_found_ts = 0.0
        self._curve_heading_deg = 0.0
        self._curve_curvature = 0.0
        self._predicted_line_cx = CAMERA_W / 2.0
        self._feedforward_steer = 0.0
        self._target_yaw_rate_dps = 0.0
        self._last_target_ts = time.monotonic()
        self._active_cornering = False

        # --- sensor state (gyro + magnetometer) ---
        self._gyro_z_raw = 0.0
        self._gyro_z = 0.0          # latest yaw rate (°/s)
        self._gyro_bias_z = 0.0
        self._mag_x = 0.0
        self._mag_y = 0.0
        self._mag_z = 0.0
        self._mag_heading = None
        self._fused_heading = 0.0
        self._target_heading = None
        self._heading_correction = 0.0
        self._last_sensor_ts = time.monotonic()
        self._fused_yaw_rate_dps = 0.0
        self._last_steer = 0.0      # for telemetry / logging
        self._gyro_correction    = 0.0
        self._vision_steer = 0.0
        self._integral   = 0.0      # PID integral accumulator
        self._prev_filtered_error = 0.0  # derivative-on-measurement state

        self._drive_command_active = False
        self._last_drive_command = None
        self._control_update_skipped = False
        self._control_cycle_index = 0
        self._recovery_brake_frames = 0
        self._recovery_phase = "idle"

        # --- CSV telemetry log (for visualization) ---
        self._csv_path = os.path.join(LOG_DIR,
            f"line_follow_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self._csv_file = open(self._csv_path, "w")
        self._csv_file.write(
            "time,state,found,cx,error,lookahead_err,vis_steer,gyro_corr,head_corr,"
            "motor_bias,total_steer,gyro_z,mag_heading,fused_heading,target_heading,target_yaw_rate,"
            "yaw_rate,curve_heading,curve_curvature,predicted_cx,feedforward_steer,"
            "speed,pan_angle,"
            "detection_mode,template_confidence,gyro_bias,recovery_phase,control_skip\n")
        log.info("CSV telemetry: %s", self._csv_path)
        self._t0 = time.monotonic()

        # --- worker thread ---
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ─────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────
    def start_following(self):
        with self._lock:
            if self.state == self.LINE_DETECTED:
                self._following = True
                self.state = self.FOLLOWING
                self._integral = 0.0     # reset PID state
                self._prev_filtered_error = 0.0
                self._filtered_error = 0.0
                self._filtered_lookahead = 0.0
                self._filtered_steer = 0.0
                self._last_control_ts = time.monotonic()
                self._last_steer = 0.0
                self._vision_steer = 0.0
                self._last_seen_error = 0.0
                self._last_seen_lookahead = 0.0
                self._curve_heading_deg = 0.0
                self._curve_curvature = 0.0
                self._predicted_line_cx = CAMERA_W / 2.0
                self._feedforward_steer = 0.0
                self._target_yaw_rate_dps = 0.0
                self._target_heading = None
                self._last_target_ts = time.monotonic()
                self._active_cornering = False
                self._recovery_brake_frames = 0
                self._recovery_phase = "idle"
                self._control_update_skipped = False
                self._control_cycle_index = 0
                self._last_drive_command = None
                self._ready_miss_count = 0
                log.info("START following (speed=%d%%)", self._base_speed)

    def stop_following(self):
        with self._lock:
            self._following = False
            self.state = self.STOPPED
            self._locked_cx     = None   # release line lock
            self._locked_colour = None
            self._locked_signature = None
            self._line_memory = None
            self._recent_template_scores.clear()
            self._detection_mode = "full"
            self._template_confidence = 0.0
            self._last_template_score = 0.0
            self._path_history.clear()
            self._spine = []
            self._pan_angle = float(FIXED_PAN)
            self._target_heading = None
            self._filtered_error = 0.0
            self._filtered_lookahead = 0.0
            self._filtered_steer = 0.0
            self._prev_filtered_error = 0.0
            self._last_control_ts = time.monotonic()
            self._last_steer = 0.0
            self._last_seen_error = 0.0
            self._last_seen_lookahead = 0.0
            self._curve_heading_deg = 0.0
            self._curve_curvature = 0.0
            self._predicted_line_cx = CAMERA_W / 2.0
            self._feedforward_steer = 0.0
            self._target_yaw_rate_dps = 0.0
            self._last_target_ts = time.monotonic()
            self._active_cornering = False
            self._recovery_brake_frames = 0
            self._recovery_phase = "idle"
            self._control_update_skipped = False
            self._control_cycle_index = 0
            self._last_drive_command = None
            self._drive_command_active = False
            self._ready_miss_count = 0
        self._car.stop()
        log.info("STOP following (user request)")

    def get_status(self):
        with self._lock:
            return {
                "state":       self.state,
                "line_x":      round(self._line_cx, 2),
                "frame_w":     CAMERA_W,
                "frame_h":     CAMERA_H,
                "gyro_z":      round(self._gyro_z, 1),
                "gyro_bias":   round(self._gyro_bias_z, 3),
                "mag_heading": None if self._mag_heading is None else round(self._mag_heading, 1),
                "fused_heading": round(self._fused_heading, 1),
                "target_heading": None if self._target_heading is None else round(self._target_heading, 1),
                "target_yaw_rate": round(self._target_yaw_rate_dps, 1),
                "yaw_rate": round(self._fused_yaw_rate_dps, 1),
                "curve_heading": round(self._curve_heading_deg, 1),
                "curve_curvature": round(self._curve_curvature, 2),
                "predicted_cx": round(self._predicted_line_cx, 2),
                "feedforward_steer": round(self._feedforward_steer, 2),
                "steer":       round(self._last_steer, 1),
                "pan_angle":   round(self._pan_angle, 1),
                "lookahead":   round(self._lookahead_error, 1),
                "locked":      self._locked_cx is not None,
                "detection_mode": self._detection_mode,
                "template_confidence": round(self._template_confidence, 2),
                "recovery_phase": self._recovery_phase,
                "control_skip": self._control_update_skipped,
                "spine":       list(self._spine),
                "trail":       list(self._path_history)[-20:],
            }

    def get_jpeg(self):
        """Return latest annotated frame as JPEG bytes (or None)."""
        with self._lock:
            f = None if self._annotated_frame is None else self._annotated_frame.copy()
        if f is None:
            return None
        ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return buf.tobytes() if ok else None

    def cleanup(self):
        self._running = False
        self._car.stop()
        self._car.cleanup()
        if self._use_picamera2:
            self._cam.stop()
        else:
            self._cam.release()
        try:
            self._csv_file.close()
        except Exception:
            pass
        log.info("Line follower shut down")

    def _get_mask_buffer(self, shape, key):
        shape = (int(shape[0]), int(shape[1]))
        buf = self._mask_scratch.get(key)
        if buf is None or buf.shape != shape:
            buf = np.zeros(shape, dtype=np.uint8)
            self._mask_scratch[key] = buf
        else:
            buf[:] = 0
        return buf

    def _get_detection_window(self, frame_shape):
        h, w = frame_shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        roi_left = 0
        roi_right = w
        if self._locked_cx is not None and self._confirm_count >= LINE_CONFIRM_FRAMES:
            cx = int(round(self._locked_cx))
            roi_left = max(0, cx - LOCKED_ROI_HALF_WIDTH)
            roi_right = min(w, cx + LOCKED_ROI_HALF_WIDTH)
        return roi_top, roi_left, roi_right

    @staticmethod
    def _restore_detection_mask(mask_small, full_h, full_w, roi_left, roi_w):
        full_mask = np.zeros((full_h, full_w), dtype=np.uint8)
        if mask_small is None or roi_w <= 0:
            return full_mask
        restored = cv2.resize(mask_small, (roi_w, full_h), interpolation=cv2.INTER_NEAREST)
        full_mask[:, roi_left:roi_left + roi_w] = restored
        return full_mask

    @staticmethod
    def _restore_contour(contour, roi_left, scale_x, scale_y):
        if contour is None:
            return None
        restored = contour.astype(np.float32).copy()
        restored[:, :, 0] = restored[:, :, 0] * scale_x + roi_left
        restored[:, :, 1] = restored[:, :, 1] * scale_y
        return np.rint(restored).astype(np.int32)

    def _fast_path_ready(self, recovery_active):
        return (
            not recovery_active
            and self._locked_cx is not None
            and self._locked_signature is not None
            and len(self._recent_template_scores) == FAST_PATH_CONFIDENCE_FRAMES
            and all(score >= LOCK_TEMPLATE_STRICT_SCORE for score in self._recent_template_scores)
        )

    def _detect_line_fast(self, small_roi, roi_top, roi_left, full_h, full_w, scale_x, scale_y):
        small_h, small_w = small_roi.shape[:2]
        if small_h == 0 or small_w == 0:
            return None

        overlay_h = max(1, int(round(24.0 / max(scale_y, 1e-6))))
        ksize = max(15, (small_w // 6) | 1)
        gray = cv2.cvtColor(small_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        bg = cv2.GaussianBlur(gray, (ksize, ksize), 0)
        dark = cv2.subtract(bg, gray)

        fast_channel = dark
        if (
            self._locked_signature is not None
            and float(self._locked_signature["hsv"][1]) >= 45.0
        ):
            hsv = cv2.cvtColor(small_roi, cv2.COLOR_BGR2HSV)
            sat = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)
            sat_bg = cv2.GaussianBlur(sat, (ksize, ksize), 0)
            fast_channel = cv2.subtract(sat, sat_bg)

        _, mask_small = cv2.threshold(
            fast_channel, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        mask_small[small_h - overlay_h:, :] = 0
        mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_CLOSE, _MORPH_KERNEL_LOCKED)

        area = float(cv2.countNonZero(mask_small))
        min_fast_area = max(36.0, MIN_CONTOUR_AREA / max(scale_x * scale_y, 1e-6) * 0.45)
        if area < min_fast_area:
            return None

        ys, xs = np.where(mask_small > 0)
        if len(xs) == 0 or len(ys) == 0:
            return None
        bbox_w = float(xs.max() - xs.min() + 1)
        bbox_h = float(ys.max() - ys.min() + 1)
        if bbox_h < max(10.0, small_h * 0.14):
            return None

        step = max(1, small_h // (SPINE_ROWS + 1))
        row_hits = 0
        for i in range(1, SPINE_ROWS + 1):
            row = small_h - i * step
            if row < 0:
                break
            cols = np.where(mask_small[row, :] > 0)[0]
            if len(cols) == 0:
                continue
            row_hits += 1
        if row_hits < max(3, SPINE_ROWS // 4):
            return None

        M = cv2.moments(mask_small, binaryImage=True)
        if M["m00"] <= 0:
            return None

        cx = roi_left + (M["m10"] / M["m00"]) * scale_x
        cy_full = roi_top + (M["m01"] / M["m00"]) * scale_y
        if self._locked_cx is not None:
            spatial_margin = LOCK_SPATIAL_MARGIN + min(120.0, abs(self._last_steer) * LOCK_STEER_MARGIN_GAIN)
            if abs(cx - self._locked_cx) > spatial_margin:
                return None

        full_mask = self._restore_detection_mask(
            mask_small, full_h, full_w, roi_left, int(round(scale_x * small_w))
        )
        return True, cx, cy_full, full_mask, None, None, None, self._last_template_score, "fast"

    # ─────────────────────────────────────────────────
    #  Line detection  (any colour, with line-lock)
    # ─────────────────────────────────────────────────
    def _detect_line(self, frame):
        h, w = frame.shape[:2]
        roi_top, roi_left, roi_right = self._get_detection_window(frame.shape)
        roi_full = frame[roi_top:, :]
        full_h, full_w = roi_full.shape[:2]
        crop_w = max(0, roi_right - roi_left)
        if full_h == 0 or crop_w == 0:
            return False, 0.0, 0.0, None, None, None, None, 0.0, "full"

        roi = roi_full[:, roi_left:roi_right]
        if ROI_RESIZE_SCALE != 1.0:
            small_w = max(1, int(round(roi.shape[1] * ROI_RESIZE_SCALE)))
            small_h = max(1, int(round(roi.shape[0] * ROI_RESIZE_SCALE)))
            roi_small = cv2.resize(roi, (small_w, small_h), interpolation=cv2.INTER_AREA)
        else:
            roi_small = roi
        rh, rw = roi_small.shape[:2]
        scale_x = crop_w / max(float(rw), 1.0)
        scale_y = full_h / max(float(rh), 1.0)

        # Optional: ignore telemetry overlay at bottom
        overlay_h = max(1, int(round(24.0 / max(scale_y, 1e-6))))

        recovery_active = (
            self._following
            and self._line_lost_since > 0.0
            and (time.monotonic() - self._line_lost_since) < RECOVERY_RELAX_WINDOW
        )
        tracking_locked = self._locked_signature is not None or self._line_memory is not None
        if self._fast_path_ready(recovery_active):
            fast_result = self._detect_line_fast(
                roi_small, roi_top, roi_left, full_h, full_w, scale_x, scale_y
            )
            if fast_result is not None:
                return fast_result

        # Gaussian kernel ≈ 1/6 of ROI width, forced odd
        ksize = max(15, (rw // 6) | 1)

        # Detect dark or strongly coloured line against local background
        gray = cv2.cvtColor(roi_small, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        bg = cv2.GaussianBlur(gray, (ksize, ksize), 0)
        dark = cv2.subtract(bg, gray)

        bg_bgr = cv2.GaussianBlur(roi_small, (ksize, ksize), 0)
        colour_delta = cv2.absdiff(roi_small, bg_bgr)
        db, dg, dr = cv2.split(colour_delta)
        colour_contrast = np.maximum(np.maximum(db, dg), dr)

        hsv = cv2.cvtColor(roi_small, cv2.COLOR_BGR2HSV)
        sat = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)
        sat_bg = cv2.GaussianBlur(sat, (ksize, ksize), 0)
        sat_boost = cv2.subtract(sat, sat_bg)

        # Binary mask
        _, dark_mask = cv2.threshold(dark, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, colour_mask = cv2.threshold(colour_contrast, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, sat_mask = cv2.threshold(sat_boost, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mask_small = cv2.bitwise_or(dark_mask, cv2.bitwise_and(colour_mask, sat_mask))

        # Remove bottom overlay region
        mask_small[rh - overlay_h:, :] = 0

        # Morphology
        morph_kernel = _MORPH_KERNEL_LOCKED if tracking_locked and not recovery_active else _MORPH_KERNEL_SEARCH
        mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_CLOSE, morph_kernel)
        mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_OPEN, morph_kernel)

        min_contour_area = max(40.0, MIN_CONTOUR_AREA / max(scale_x * scale_y, 1e-6))
        if tracking_locked:
            min_contour_area *= 0.55

        min_line_height = max(40.0 / scale_y, rh * 0.18)
        min_line_elongation = 2.2
        min_spine_hits = max(4, SPINE_ROWS // 3)
        max_short_side = max(28.0 / scale_x, rw * 0.18)
        if tracking_locked:
            min_line_height = min(min_line_height, max(32.0 / scale_y, rh * 0.14))
            min_line_elongation = min(min_line_elongation, 1.9)
            min_spine_hits = min(min_spine_hits, max(3, SPINE_ROWS // 4))
        if recovery_active and tracking_locked:
            min_line_height = min(min_line_height, max(24.0 / scale_y, rh * 0.10))
            min_line_elongation = min(min_line_elongation, 1.4)
            min_spine_hits = min(min_spine_hits, max(2, SPINE_ROWS // 5))
            max_short_side = max(max_short_side, rw * 0.26)

        def _count_row_hits(c_mask):
            step = max(1, rh // (SPINE_ROWS + 1))
            hits = 0
            for i in range(1, SPINE_ROWS + 1):
                row = rh - i * step
                if row < 0:
                    break
                cols = np.where(c_mask[row, :] > 0)[0]
                if len(cols) == 0:
                    for dr in range(1, step // 2 + 3):
                        found = False
                        for r2 in (row - dr, row + dr):
                            if 0 <= r2 < rh:
                                cols = np.where(c_mask[r2, :] > 0)[0]
                                if len(cols):
                                    found = True
                                    break
                        if found:
                            break
                if len(cols) == 0:
                    continue
                hits += 1
            return hits

        def _line_metrics(c):
            x, y, ww, hh = cv2.boundingRect(c)
            rect = cv2.minAreaRect(c)
            (_, _), (side_a, side_b), _ = rect
            long_side = max(side_a, side_b)
            short_side = max(1.0, min(side_a, side_b))
            elongation = long_side / short_side
            c_mask = self._get_mask_buffer((rh, rw), "metrics")
            cv2.drawContours(c_mask, [c], -1, 255, -1)
            row_hits = _count_row_hits(c_mask)
            return {
                "x": x,
                "y": y,
                "w": ww,
                "h": hh,
                "long_side": long_side,
                "short_side": short_side,
                "elongation": elongation,
                "bottom": y + hh,
                "row_hits": row_hits,
            }

        def _passes_line_shape(c):
            m = _line_metrics(c)
            if m["h"] < min_line_height:
                return False
            if m["long_side"] < min_line_height:
                return False
            if m["elongation"] < min_line_elongation:
                return False
            if m["short_side"] > max_short_side:
                return False
            if m["row_hits"] < min_spine_hits:
                return False
            return True

        contours, _ = cv2.findContours(mask_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        def _contour_signature(c, metrics=None):
            m = metrics if metrics is not None else _line_metrics(c)
            c_mask = self._get_mask_buffer((rh, rw), "signature")
            cv2.drawContours(c_mask, [c], -1, 255, -1)
            mean_bgr = np.array(cv2.mean(roi_small, mask=c_mask)[:3], dtype=np.float32)
            mean_hsv = np.array(cv2.mean(hsv, mask=c_mask)[:3], dtype=np.float32)
            M_ = cv2.moments(c)
            if M_["m00"] == 0:
                cx_ = roi_left + (m["x"] + m["w"] / 2.0) * scale_x
                cy_ = roi_top + (m["y"] + m["h"] / 2.0) * scale_y
            else:
                cx_ = roi_left + (M_["m10"] / M_["m00"]) * scale_x
                cy_ = roi_top + (M_["m01"] / M_["m00"]) * scale_y
            return {
                "bgr": mean_bgr,
                "hsv": mean_hsv,
                "short_side": m["short_side"],
                "row_hits": m["row_hits"],
                "elongation": m["elongation"],
                "cx": cx_,
                "cy": cy_,
            }

        candidates = []
        for c in contours:
            if cv2.contourArea(c) <= min_contour_area:
                continue
            if not _passes_line_shape(c):
                continue
            metrics = _line_metrics(c)
            signature = _contour_signature(c, metrics)
            candidates.append({
                "contour": c,
                "metrics": metrics,
                "signature": signature,
            })
        full_mask = self._restore_detection_mask(mask_small, full_h, full_w, roi_left, crop_w)
        if not candidates:
            return False, 0.0, 0.0, full_mask, None, None, None, 0.0, "full"

        locked_cx  = self._locked_cx
        locked_col = self._locked_colour
        template = self._locked_signature if self._locked_signature is not None else self._line_memory
        template_min_score = (
            RECOVERY_TEMPLATE_MIN_SCORE
            if recovery_active else LOCK_TEMPLATE_MIN_SCORE
        )
        template_strict_score = (
            RECOVERY_TEMPLATE_STRICT_SCORE
            if recovery_active else LOCK_TEMPLATE_STRICT_SCORE
        )

        if template is not None:
            for cand in candidates:
                cand["template_score"] = self._template_similarity(cand["signature"], template)
            template_valid = [
                cand for cand in candidates
                if cand["template_score"] >= template_min_score
            ]
            if template_valid:
                candidates = template_valid
            else:
                return False, 0.0, 0.0, full_mask, None, None, None, 0.0, "full"

        if locked_cx is not None and locked_col is not None:
            spatial_margin = LOCK_SPATIAL_MARGIN + int(
                min(120.0, abs(self._last_steer) * LOCK_STEER_MARGIN_GAIN)
            )
            if recovery_active:
                spatial_margin += 60
            locked_valid = []
            for cand in candidates:
                cx_ = cand["signature"]["cx"]
                if abs(cx_ - locked_cx) > spatial_margin:
                    continue
                col = cand["signature"]["bgr"]
                template_score = cand.get("template_score", 0.0)
                # Allow deliberate colour transitions (for example red → blue
                # tape on the same path) when the contour still matches the
                # learned line shape/signature strongly.
                if (
                    np.max(np.abs(col - locked_col)) > LOCK_COLOUR_TOL
                    and (template is None or template_score < template_strict_score)
                ):
                    continue
                if template is not None and template_score < template_strict_score:
                    continue
                locked_valid.append(cand)
            if locked_valid:
                candidates = locked_valid
            elif self._locked_signature is not None:
                return False, 0.0, 0.0, full_mask, None, None, None, 0.0, "full"

        def _score_candidate(cand):
            c = cand["contour"]
            area = cv2.contourArea(c)
            m = cand["metrics"]
            x = m["x"]
            y = m["y"]
            ww = m["w"]
            hh = m["h"]
            if ww <= 0 or hh <= 0:
                return -1e9

            aspect = hh / ww
            bottom = m["bottom"]
            score = area
            score += 60.0 * min(aspect, 12.0)
            score += 90.0 * min(m["elongation"], 10.0)
            score += 80.0 * m["row_hits"]
            score += 1.5 * bottom

            if bottom >= rh - 8:
                score += 250.0

            M_ = cv2.moments(c)
            if M_["m00"] > 0 and locked_cx is not None:
                cx_ = roi_left + (M_["m10"] / M_["m00"]) * scale_x
                score -= 2.0 * abs(cx_ - locked_cx)

            if template is not None:
                score += 180.0 * cand.get("template_score", 0.0)

            return score

        best_cand = max(candidates, key=_score_candidate)
        best = best_cand["contour"]
        M = cv2.moments(best)
        if M["m00"] == 0:
            return False, 0.0, 0.0, full_mask, None, None, None, 0.0, "full"

        cx = roi_left + (M["m10"] / M["m00"]) * scale_x
        cy_full = roi_top + (M["m01"] / M["m00"]) * scale_y
        signature = best_cand["signature"]
        mean_bgr = signature["bgr"]
        best_template_score = best_cand.get("template_score", 0.0)
        contour_full = self._restore_contour(best, roi_left, scale_x, scale_y)

        return True, cx, cy_full, full_mask, contour_full, mean_bgr, signature, best_template_score, "full"

    # ─────────────────────────────────────────────────
    #  Spine extraction — traces the line ahead row by row
    # ─────────────────────────────────────────────────
    @staticmethod
    def _extract_spine(mask, roi_top):
        """Extract line spine by scanning the binary mask at SPINE_ROWS heights.

        Uses the full binary mask (ROI-relative) rather than contour-point
        sampling.  This is far more robust on thick, curved, or noisy lines
        because every lit pixel at each scan row contributes to the x-average.

        Returns a list of (x, y) full-frame coords from near (bottom) to far
        (top of ROI).  Rows with no lit pixels are skipped, so the list may
        have fewer than SPINE_ROWS entries.
        """
        if mask is None:
            return []

        rh, rw = mask.shape[:2]
        step   = max(1, rh // (SPINE_ROWS + 1))
        spine  = []

        for i in range(1, SPINE_ROWS + 1):
            row = rh - i * step          # scan bottom-to-top inside ROI
            if row < 0:
                break
            cols = np.where(mask[row, :] > 0)[0]
            # If the exact row is empty, search ±(step//2+2) nearby rows
            if len(cols) == 0:
                for dr in range(1, step // 2 + 3):
                    for r2 in (row - dr, row + dr):
                        if 0 <= r2 < rh:
                            cols = np.where(mask[r2, :] > 0)[0]
                            if len(cols):
                                break
                    if len(cols):
                        break
            if len(cols) == 0:
                continue
            x_mid = float(np.mean(cols))
            spine.append((x_mid, row + roi_top))

        return spine

    # ─────────────────────────────────────────────────
    #  Curve lookahead error
    # ─────────────────────────────────────────────────
    def _compute_lookahead_error(self, spine, frame_w):
        """Return the steering error at a lookahead point ahead of the rover.

        Averages the x-positions of the LOOKAHEAD_ROWS farthest spine points
        (i.e. highest up in the image = farthest ahead on the ground).  On a
        straight line these match the centroid; on a curve they shift to one
        side, giving an early-warning signal so the PID begins correcting
        before the centroid drifts off-centre.
        """
        if len(spine) < 2:
            return 0.0
        far_pts = spine[-LOOKAHEAD_ROWS:]
        la_x = sum(p[0] for p in far_pts) / len(far_pts)
        return la_x - (frame_w / 2.0)

    def _estimate_curve_geometry(self, spine, frame_w, frame_h):
        """Estimate line heading and curvature by fitting the extracted spine."""
        if spine is None or len(spine) < CURVE_FIT_MIN_POINTS:
            return None

        roi_top = int(frame_h * ROI_TOP_FRAC)
        roi_h = max(1.0, frame_h - roi_top)
        mid_x = frame_w / 2.0

        samples = np.array(
            [(float(frame_h - y), float(x - mid_x)) for x, y in spine],
            dtype=np.float32,
        )
        samples = samples[np.argsort(samples[:, 0])]
        s = samples[:, 0]
        x = samples[:, 1]

        if len(s) >= 2:
            keep = np.concatenate(([True], np.diff(s) > 1.0))
            s = s[keep]
            x = x[keep]

        if len(s) < 3 or (s[-1] - s[0]) < max(24.0, roi_h * 0.18):
            return None

        try:
            weights = np.linspace(2.0, 0.5, len(s), dtype=np.float32)
            coeffs = np.polyfit(s, x, 2, w=weights)
        except np.linalg.LinAlgError:
            return None

        poly = np.poly1d(coeffs)
        d1 = np.polyder(poly, 1)
        d2 = np.polyder(poly, 2)

        s_min = float(s[0])
        s_max = float(s[-1])
        s_near = max(s_min, roi_h * CURVE_NEAR_FRAC)
        s_mid = min(s_max, max(s_near + 8.0, roi_h * CURVE_MID_FRAC))
        s_far = min(s_max, max(s_mid + 8.0, roi_h * CURVE_FAR_FRAC))
        if s_far <= s_near + 6.0:
            return None

        x_mid = float(poly(s_mid))
        x_far = float(poly(s_far))
        slope_near = float(d1(s_near))
        slope_mid = float(d1(s_mid))
        slope_far = float(d1(s_far))

        if len(s) >= 2:
            near_diffs = np.diff(s[:min(len(s), 4)])
            near_diffs = near_diffs[near_diffs > 1.0]
            near_spacing = float(np.median(near_diffs)) if len(near_diffs) else max(8.0, (s_far - s_near) / 3.0)
        else:
            near_spacing = max(8.0, (s_far - s_near) / 3.0)
        speed_scale = 0.5  # fixed (no encoder speed feedback)
        s_predict = min(s_max, s_near + near_spacing * PREDICTION_FRAMES_AHEAD * speed_scale)
        predicted_x = float(poly(s_predict))

        heading_near = math.degrees(math.atan(slope_near))
        heading_mid = math.degrees(math.atan(slope_mid))
        heading_far = math.degrees(math.atan(slope_far))
        heading_deg = max(-45.0, min(45.0, 0.65 * heading_mid + 0.35 * heading_far))

        curvature_px = float(d2(s_mid)) / max((1.0 + slope_mid * slope_mid) ** 1.5, 1e-6)
        curvature_norm = max(
            -CURVE_MAX_NORM,
            min(CURVE_MAX_NORM, curvature_px * roi_h),
        )

        coverage = min(1.0, (s_max - s_min) / max(roi_h * 0.55, 1.0))
        confidence = min(1.0, len(s) / SPINE_ROWS) * coverage

        return {
            "lookahead_x": x_far,
            "mid_x": x_mid,
            "predicted_x": predicted_x,
            "heading_deg": heading_deg,
            "delta_heading_deg": heading_far - heading_near,
            "curvature_norm": curvature_norm,
            "confidence": confidence,
        }

    def _update_path_targets(self, curve_geometry, vision_error, lookahead_error):
        now = time.monotonic()
        dt = max(1e-3, min(0.1, now - self._last_target_ts))
        self._last_target_ts = now

        self._curve_heading_deg = 0.0
        self._curve_curvature = 0.0
        self._predicted_line_cx = CAMERA_W / 2.0
        self._target_yaw_rate_dps = 0.0
        self._active_cornering = False

        if self._target_heading is None:
            self._target_heading = self._fused_heading

        if curve_geometry is not None:
            confidence = curve_geometry.get("confidence", 1.0)
            self._curve_heading_deg = float(curve_geometry["heading_deg"])
            self._curve_curvature = float(curve_geometry["curvature_norm"])
            self._predicted_line_cx = (CAMERA_W / 2.0) + confidence * float(curve_geometry.get("predicted_x", 0.0))

            speed_scale = 0.35  # fixed (no encoder speed feedback)
            desired_yaw = confidence * (
                TARGET_YAW_HEADING_GAIN * self._curve_heading_deg
                + TARGET_YAW_CURVATURE_GAIN * self._curve_curvature * speed_scale
            )
            self._target_yaw_rate_dps = max(
                -MAX_TARGET_YAW_RATE_DPS,
                min(MAX_TARGET_YAW_RATE_DPS, desired_yaw),
            )
            self._active_cornering = (
                abs(self._target_yaw_rate_dps) >= ACTIVE_CORNERING_YAW_RATE_DPS
                or abs(self._curve_curvature) >= ACTIVE_CORNERING_CURVATURE
            )

            desired_heading = self._wrap_angle_deg(
                self._fused_heading + confidence * self._curve_heading_deg
            )
            self._target_heading = self._blend_angle_deg(
                self._target_heading, desired_heading, TARGET_HEADING_BLEND
            )
            self._target_heading = self._wrap_angle_deg(
                self._target_heading + self._target_yaw_rate_dps * dt
            )
            return

        if (
            abs(vision_error) <= MAG_LOCK_CENTER_PX
            and abs(lookahead_error) < 10
            and abs(self._fused_yaw_rate_dps) < 10
        ):
            self._target_heading = self._blend_angle_deg(
                self._target_heading, self._fused_heading, 0.35
            )

    # ─────────────────────────────────────────────────
    #  Annotate frame for the web stream
    # ─────────────────────────────────────────────────
    def _annotate(self, frame, found, cx, cy, contour, spine):
        out = frame.copy()
        h, w = out.shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        cx_i = int(round(cx)) if found else w // 2
        cy_i = int(round(cy)) if found else h // 2

        # Draw ROI boundary
        cv2.line(out, (0, roi_top), (w, roi_top), (100, 100, 100), 1)

        # Frame centre reference
        mid_x = w // 2
        cv2.line(out, (mid_x, roi_top), (mid_x, h), (80, 80, 80), 1)

        # ── Path trail (rolling centroid history) ──────────────────────────
        # Draw the rover's recent centroid positions as a fading dotted trail
        history = list(self._path_history)
        n = len(history)
        for i, (hx, hy) in enumerate(history):
            alpha = int(80 + 150 * (i / max(n - 1, 1)))   # fade older points
            radius = 3 if i < n - 1 else 5
            cv2.circle(out, (int(round(hx)), int(round(hy))), radius, (alpha, alpha, 0), -1)
        if n >= 2:
            for i in range(1, n):
                alpha = int(80 + 150 * (i / (n - 1)))
                p0 = (int(round(history[i - 1][0])), int(round(history[i - 1][1])))
                p1 = (int(round(history[i][0])), int(round(history[i][1])))
                cv2.line(out, p0, p1, (0, alpha, alpha), 1)

        # ── Spine (projected path ahead of rover) ─────────────────────────
        # Draw the extracted spine points as a bright green path ahead
        if spine and len(spine) >= 2:
            pts = np.array([(int(round(sx)), int(round(sy))) for sx, sy in spine], dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], False, (0, 255, 80), 3)
            for sx, sy in spine:
                cv2.circle(out, (int(round(sx)), int(round(sy))), 4, (0, 255, 80), -1)

        if found and contour is not None:
            # Draw contour outline (shift to full-frame coords)
            shifted = contour.copy()
            shifted[:, :, 1] += roi_top
            cv2.drawContours(out, [shifted], -1, (0, 255, 0), 2)

            # Centroid
            cv2.circle(out, (cx_i, cy_i), 8, (0, 255, 255), -1)

            # Steering arrow
            cv2.arrowedLine(out, (mid_x, h - 30), (cx_i, cy_i), (0, 200, 255), 3, tipLength=0.25)

            # Lock indicator — small padlock icon (circle + text)
            if self._locked_cx is not None:
                cv2.circle(out, (cx_i, cy_i), 14, (0, 200, 255), 2)
                cv2.putText(out, "LOCKED", (cx_i - 28, cy_i - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

        # Status overlay
        state = self.state
        colour_map = {
            self.SEARCHING:     ((0, 165, 255), "SEARCHING..."),
            self.LINE_DETECTED: ((0, 255, 0),   "LINE DETECTED"),
            self.FOLLOWING:     ((255, 180, 0),  "FOLLOWING"),
            self.LINE_LOST:     ((0, 0, 255),    "LINE LOST"),
            self.STOPPED:       ((128, 128, 128),"STOPPED"),
        }
        colour, label = colour_map.get(state, ((255, 255, 255), state))
        cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)

        if found:
            err = cx - mid_x
            cv2.putText(out, f"err={err:+.1f}px {self._detection_mode}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Sensor telemetry overlay — two lines at bottom
        y1 = h - 28
        y0 = h - 12
        cv2.putText(out, f"pan={self._pan_angle:.0f}\u00b0  la={self._lookahead_error:+.0f}px  curv={self._curve_curvature:+.2f}",
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1)
        cv2.putText(out, f"gyro={self._gyro_z:+.1f} bias={self._gyro_bias_z:+.2f} yaw={self._fused_yaw_rate_dps:+.1f}", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(out, f"steer={self._last_steer:+.1f}",
                    (320, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return out

    # ─────────────────────────────────────────────────
    #  Main processing loop (runs on worker thread)
    # ─────────────────────────────────────────────────
    def _read_sensors(self):
        """Read MPU gyro and magnetometer."""
        now = time.monotonic()
        with self._lock:
            last_sensor_ts = self._last_sensor_ts
            last_steer = self._last_steer
            drive_command_active = self._drive_command_active
            fused_heading = self._fused_heading
            gyro_bias_z = self._gyro_bias_z
        dt = min(max(now - last_sensor_ts, 0.0), 0.2)

        gyro_z_raw = get_gyro_z()  # °/s yaw rate
        mag_x, mag_y, mag_z = get_magnetometer()

        # Gyro bias estimation — only update when not driving and not steering
        stationary = (
            abs(last_steer) < 2.0
            and not drive_command_active
        )
        if stationary:
            gyro_bias_z += GYRO_BIAS_ALPHA * (gyro_z_raw - gyro_bias_z)
        gyro_z = gyro_z_raw - gyro_bias_z

        field_sq = mag_x * mag_x + mag_y * mag_y
        if field_sq > (MAG_MIN_FIELD * MAG_MIN_FIELD):
            mag_heading = math.degrees(math.atan2(mag_y, mag_x))
            pred_heading = fused_heading + gyro_z * dt
            fused_heading = self._wrap_angle_deg(
                MAG_BLEND_ALPHA * pred_heading
                + (1.0 - MAG_BLEND_ALPHA) * mag_heading
            )
        else:
            mag_heading = None
            fused_heading = self._wrap_angle_deg(fused_heading + gyro_z * dt)

        # No encoders — yaw rate is pure gyro
        fused_yaw_rate_dps = gyro_z

        with self._lock:
            self._last_sensor_ts = now
            self._gyro_z_raw = gyro_z_raw
            self._gyro_bias_z = gyro_bias_z
            self._gyro_z = gyro_z
            self._mag_x = mag_x
            self._mag_y = mag_y
            self._mag_z = mag_z
            self._mag_heading = mag_heading
            self._fused_heading = fused_heading
            self._fused_yaw_rate_dps = fused_yaw_rate_dps

    @staticmethod
    def _wrap_angle_deg(angle):
        return (angle + 180.0) % 360.0 - 180.0

    @classmethod
    def _blend_angle_deg(cls, base, target, alpha):
        return cls._wrap_angle_deg(base + alpha * cls._wrap_angle_deg(target - base))

    @staticmethod
    def _hue_distance(h1, h2):
        diff = abs(float(h1) - float(h2))
        return min(diff, 180.0 - diff)

    @staticmethod
    def _copy_signature(sig):
        if sig is None:
            return None
        return {
            "bgr": np.array(sig["bgr"], dtype=np.float32),
            "hsv": np.array(sig["hsv"], dtype=np.float32),
            "short_side": float(sig["short_side"]),
            "row_hits": float(sig["row_hits"]),
            "elongation": float(sig["elongation"]),
        }

    def _blend_signature(self, base, new, alpha):
        if new is None:
            return self._copy_signature(base)
        if base is None:
            return self._copy_signature(new)

        out = self._copy_signature(base)
        out["bgr"] = (1.0 - alpha) * out["bgr"] + alpha * np.array(new["bgr"], dtype=np.float32)

        base_h = float(out["hsv"][0])
        new_h = float(new["hsv"][0])
        h_diff = ((new_h - base_h + 90.0) % 180.0) - 90.0
        out["hsv"][0] = (base_h + alpha * h_diff) % 180.0
        out["hsv"][1] = (1.0 - alpha) * out["hsv"][1] + alpha * float(new["hsv"][1])
        out["hsv"][2] = (1.0 - alpha) * out["hsv"][2] + alpha * float(new["hsv"][2])
        out["short_side"] = (1.0 - alpha) * out["short_side"] + alpha * float(new["short_side"])
        out["row_hits"] = (1.0 - alpha) * out["row_hits"] + alpha * float(new["row_hits"])
        out["elongation"] = (1.0 - alpha) * out["elongation"] + alpha * float(new["elongation"])
        return out

    def _template_similarity(self, sig, template):
        if sig is None or template is None:
            return 0.0

        hue_err = self._hue_distance(sig["hsv"][0], template["hsv"][0])
        sat_err = abs(float(sig["hsv"][1]) - float(template["hsv"][1]))
        val_err = abs(float(sig["hsv"][2]) - float(template["hsv"][2]))
        thick_frac = abs(float(sig["short_side"]) - float(template["short_side"])) / max(1.0, float(template["short_side"]))
        hit_frac = abs(float(sig["row_hits"]) - float(template["row_hits"])) / max(1.0, float(template["row_hits"]))
        elong_frac = abs(float(sig["elongation"]) - float(template["elongation"])) / max(1.0, float(template["elongation"]))

        # If the remembered line is clearly saturated (e.g. blue tape), strongly
        # suppress dull grey/black candidates even if they are line-shaped.
        if float(template["hsv"][1]) > 45.0 and float(sig["hsv"][1]) < max(18.0, float(template["hsv"][1]) * 0.35):
            return -1.0

        sat_weight = 0.4 if float(template["hsv"][1]) < 30.0 else 1.0
        hue_score = max(0.0, 1.0 - hue_err / LOCK_HUE_TOL) * sat_weight
        sat_score = max(0.0, 1.0 - sat_err / LOCK_SAT_TOL)
        val_score = max(0.0, 1.0 - val_err / LOCK_VAL_TOL)
        thick_score = max(0.0, 1.0 - thick_frac / LOCK_THICKNESS_FRAC)
        hit_score = max(0.0, 1.0 - hit_frac / LOCK_ROW_HIT_FRAC)
        elong_score = max(0.0, 1.0 - elong_frac / LOCK_ELONGATION_FRAC)

        return (
            1.8 * hue_score
            + 1.2 * sat_score
            + 0.6 * val_score
            + 1.0 * thick_score
            + 0.7 * hit_score
            + 0.5 * elong_score
        )

    def _compute_steering(self, vision_error, lookahead_error, curve_geometry=None):
        now = time.monotonic()
        dt = max(1e-3, min(0.1, now - self._last_control_ts))
        self._last_control_ts = now

        # Filter inputs
        self._filtered_error += ERROR_FILTER_ALPHA * (vision_error - self._filtered_error)
        self._filtered_lookahead += LOOKAHEAD_FILTER_ALPHA * (lookahead_error - self._filtered_lookahead)

        lookahead_weight = LOOKAHEAD_WEIGHT
        if curve_geometry is not None:
            lookahead_weight += LOOKAHEAD_CURVATURE_GAIN * min(
                1.0, abs(float(curve_geometry.get("curvature_norm", 0.0))) / 2.0
            )
        if (
            abs(self._filtered_error) >= CENTER_DEADBAND_PX
            and self._filtered_error * self._filtered_lookahead < 0.0
        ):
            disagreement = min(
                1.0,
                (abs(self._filtered_error) + abs(self._filtered_lookahead))
                / max(CAMERA_W * 0.35, 1.0),
            )
            lookahead_weight *= max(0.2, 1.0 - 0.75 * disagreement)
            self._integral *= 0.88
        lookahead_weight = max(0.0, min(0.92, lookahead_weight))

        blended = ((1.0 - lookahead_weight) * self._filtered_error
                   + lookahead_weight * self._filtered_lookahead)
        self._lookahead_error = self._filtered_lookahead

        geometry_steer = 0.0
        feedforward_steer = 0.0
        if curve_geometry is not None:
            confidence = curve_geometry.get("confidence", 1.0)
            geometry_steer = confidence * (
                PATH_HEADING_STEER_KP * float(curve_geometry["heading_deg"])
                + PATH_CURVATURE_STEER_KP * float(curve_geometry["curvature_norm"])
            )
            feedforward_steer = confidence * FEEDFORWARD_KP * float(curve_geometry.get("predicted_x", 0.0))
        self._feedforward_steer = feedforward_steer

        # Time-based PID
        kp = STEERING_KP  # fixed (no speed-adaptive reduction without encoders)
        p = kp * blended

        deriv = -(blended - self._prev_filtered_error) / dt
        d = STEERING_KD * deriv
        self._prev_filtered_error = blended

        if abs(vision_error) >= CENTER_DEADBAND_PX and self._last_steer * vision_error < 0.0:
            self._integral *= 0.72

        raw_pre_i = p + self._integral + d + geometry_steer + feedforward_steer + self._motor_bias
        integral_frozen = abs(vision_error) > CAMERA_W * 0.25
        if (
            not integral_frozen
            and (abs(raw_pre_i) < MAX_STEER or (raw_pre_i * blended < 0))
        ):
            self._integral += STEERING_KI * blended * dt
            self._integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, self._integral))

        vision_steer = p + self._integral + d + geometry_steer + feedforward_steer + self._motor_bias
        self._vision_steer = vision_steer

        target_steer = max(-MAX_STEER, min(MAX_STEER, vision_steer))

        # Slew-rate limit the output
        max_step = STEER_SLEW_RATE * dt
        delta = target_steer - self._filtered_steer
        delta = max(-max_step, min(max_step, delta))
        self._filtered_steer += delta

        self._last_steer = self._filtered_steer
        return self._filtered_steer

    def _issue_drive_command(self, speed, steer_angle, forward=True, cache=True):
        """Send speed + steering angle to Pico via CarSystem."""
        duty_cap = self._car.power_limiter.max_safe_duty
        speed = max(0.0, min(duty_cap, float(speed)))
        steer_angle = max(-MAX_STEER, min(MAX_STEER, float(steer_angle)))
        if cache:
            self._last_drive_command = {
                "speed": speed,
                "steer": steer_angle,
                "forward": forward,
            }
        self._drive_command_active = speed > 1.0
        self._car._apply_steering(speed, steer_angle, forward=forward)

    def _reuse_last_drive_command(self):
        if self._last_drive_command is None:
            return False
        self._drive_command_active = self._last_drive_command["speed"] > 1.0
        return True

    def _compute_recovery_command(self):
        if self._line_lost_since == 0.0:
            return None

        elapsed = time.monotonic() - self._line_lost_since
        if elapsed < 0.0 or elapsed >= LINE_LOST_TIMEOUT:
            return None

        if self._recovery_brake_frames > 0:
            self._recovery_brake_frames -= 1
            if self._recovery_brake_frames == RECOVERY_BRAKE_TOTAL_FRAMES - 1:
                self._recovery_phase = "brake"
                return {"mode": "brake"}
            self._recovery_phase = "hold"
            return {
                "mode": "hold",
                "speed": 0.0,
                "steer": self._last_steer,
            }

        sign_source = self._last_seen_error
        if abs(sign_source) < CENTER_DEADBAND_PX:
            sign_source = self._last_seen_lookahead
        if abs(sign_source) < CENTER_DEADBAND_PX:
            sign_source = self._last_steer
        if abs(sign_source) < 1e-3:
            return None

        steer = self._last_steer * RECOVERY_STEER_GAIN
        steer += (
            RECOVERY_YAW_RATE_STEER_GAIN
            * MAX_STEER
            * max(-1.0, min(1.0, self._target_yaw_rate_dps / MAX_TARGET_YAW_RATE_DPS))
        )
        min_steer = RECOVERY_MIN_STEER
        if abs(self._last_seen_error) > CAMERA_W * 0.18:
            min_steer += 4.0
        if abs(steer) < min_steer:
            steer = math.copysign(min_steer, sign_source)
        else:
            steer = math.copysign(abs(steer), sign_source)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        strength = max(0.0, 1.0 - elapsed / LINE_LOST_TIMEOUT)
        speed_frac = (
            RECOVERY_SPEED_MIN_FRAC
            + (RECOVERY_SPEED_MAX_FRAC - RECOVERY_SPEED_MIN_FRAC) * strength
        )
        speed = self._base_speed * speed_frac
        self._recovery_phase = "recover"
        return {
            "mode": "recover",
            "speed": speed,
            "steer": steer,
        }

    def _compute_motion_corrections(self, vision_error, steer_angle):
        """Adjust steering angle using gyro yaw-rate damping and magnetometer heading hold."""

        gyro_corr = 0.0
        if self._use_mpu:
            rate_error = self._target_yaw_rate_dps - self._fused_yaw_rate_dps
            if abs(rate_error) > GYRO_DEADZONE:
                raw = GYRO_KP * rate_error
                gyro_corr = max(-GYRO_MAX_CORR, min(GYRO_MAX_CORR, raw))
                steer_angle += gyro_corr
        self._gyro_correction = round(gyro_corr, 2)

        heading_corr = 0.0
        if self._target_heading is not None and not self._active_cornering:
            heading_error = self._wrap_angle_deg(self._target_heading - self._fused_heading)
            curve_fade = min(1.0, abs(self._target_yaw_rate_dps) / max(MAX_TARGET_YAW_RATE_DPS * 0.45, 1.0))
            fade = max(0.25 * curve_fade, max(0.0, 1.0 - abs(vision_error) / MAG_FADE_PX))
            raw = MAG_HEADING_KP * heading_error * fade
            heading_corr = max(-MAG_HEADING_MAX_CORR, min(MAG_HEADING_MAX_CORR, raw))
            steer_angle += heading_corr
        self._heading_correction = round(heading_corr, 2)

        steer_angle = max(-MAX_STEER, min(MAX_STEER, steer_angle))

        log.debug("motion: yaw*=%.1f yaw=%.1f gyro=%.2f hdg=%.2f fused=%+.1f target=%s → steer=%.1f",
                  self._target_yaw_rate_dps, self._fused_yaw_rate_dps,
                  gyro_corr, heading_corr, self._fused_heading,
                  f"{self._target_heading:+.1f}" if self._target_heading is not None else "None",
                  steer_angle)

        return steer_angle

    def _capture_frame(self):
        if self._use_picamera2:
            frame = self._cam.capture_array()
        else:
            ok, frame = self._cam.read()
            if not ok:
                return None
        if frame is None:
            return None
        return cv2.flip(frame, -1)

    def _update_detection_state(
        self, found, cx, cy, mean_colour, signature, template_score, detection_mode
    ):
        should_brake = False
        with self._lock:
            if found:
                self._line_cx = cx
                self._line_cy = cy
                self._detection_mode = detection_mode
                self._last_template_score = float(template_score)
                if detection_mode == "full":
                    self._recent_template_scores.append(self._last_template_score)
                self._template_confidence = (
                    sum(self._recent_template_scores) / len(self._recent_template_scores)
                    if self._recent_template_scores else self._last_template_score
                )
                self._confirm_count = min(self._confirm_count + 1, LINE_CONFIRM_FRAMES + 5)
                self._ready_miss_count = 0
                self._line_lost_since = 0.0
                self._recovery_brake_frames = 0
                self._recovery_phase = "tracking"

                if self._confirm_count >= LINE_CONFIRM_FRAMES and self.state != self.STOPPED:
                    if self._locked_cx is None:
                        self._locked_cx = cx
                        self._locked_colour = mean_colour
                        self._locked_signature = self._copy_signature(signature)
                        self._line_memory = self._blend_signature(
                            self._line_memory, signature, MEMORY_BLEND_ALPHA
                        )
                        log.info(
                            "Line locked at cx=%.1f colour=%.0f,%.0f,%.0f",
                            cx,
                            *(mean_colour if mean_colour is not None else (0, 0, 0)),
                        )
                    else:
                        self._locked_cx = 0.85 * self._locked_cx + 0.15 * cx
                        if mean_colour is not None:
                            self._locked_colour = (
                                0.85 * self._locked_colour + 0.15 * mean_colour
                            )
                        if signature is not None:
                            self._locked_signature = self._blend_signature(
                                self._locked_signature, signature, LOCK_BLEND_ALPHA
                            )
                            self._line_memory = self._blend_signature(
                                self._line_memory, signature, MEMORY_BLEND_ALPHA
                            )

                    if self._following:
                        self.state = self.FOLLOWING
                    elif self.state not in (self.FOLLOWING, self.STOPPED):
                        self.state = self.LINE_DETECTED

                if self._confirm_count >= LINE_CONFIRM_FRAMES and self.state != self.STOPPED:
                    self._path_history.append((cx, cy))
            else:
                self._detection_mode = "full"
                self._last_template_score = 0.0
                self._template_confidence = 0.0
                self._recent_template_scores.clear()
                self._confirm_count = max(0, self._confirm_count - 2)
                self._ready_miss_count = min(
                    self._ready_miss_count + 1, LINE_READY_DROP_MISSES + 2
                )
                if self._following:
                    if self._line_lost_since == 0.0:
                        self._line_lost_since = time.monotonic()
                        self._recovery_brake_frames = RECOVERY_BRAKE_TOTAL_FRAMES
                        self._recovery_phase = "brake"
                    elif (time.monotonic() - self._line_lost_since) > LINE_LOST_TIMEOUT:
                        self._following = False
                        self.state = self.LINE_LOST
                        self._integral = 0.0
                        self._prev_filtered_error = 0.0
                        self._filtered_error = 0.0
                        self._filtered_lookahead = 0.0
                        self._filtered_steer = 0.0
                        self._last_control_ts = time.monotonic()
                        self._last_steer = 0.0
                        self._vision_steer = 0.0
                        self._curve_heading_deg = 0.0
                        self._curve_curvature = 0.0
                        self._predicted_line_cx = CAMERA_W / 2.0
                        self._feedforward_steer = 0.0
                        self._target_yaw_rate_dps = 0.0
                        self._locked_cx = None
                        self._locked_colour = None
                        self._locked_signature = None
                        self._line_memory = None
                        self._target_heading = None
                        self._last_target_ts = time.monotonic()
                        self._active_cornering = False
                        self._recovery_brake_frames = 0
                        self._recovery_phase = "lost"
                        self._control_cycle_index = 0
                        self._last_drive_command = None
                        self._drive_command_active = False
                        self._ready_miss_count = 0
                        should_brake = True
                else:
                    self._recovery_phase = "idle"
                    ready_held = (
                        self.state == self.LINE_DETECTED
                        and self._ready_miss_count < LINE_READY_DROP_MISSES
                    )
                    if ready_held:
                        self.state = self.LINE_DETECTED
                    elif self.state not in (self.STOPPED, self.LINE_LOST):
                        self.state = self.SEARCHING
            cur_state = self.state

        if should_brake:
            self._car.brake()
            log.warning("Line lost for %.1fs — braking", LINE_LOST_TIMEOUT)
        return cur_state

    def _process_frame(self, frame):
        h, w = frame.shape[:2]
        found, cx, cy, mask, contour, mean_colour, signature, template_score, detection_mode = self._detect_line(frame)
        cur_state = self._update_detection_state(
            found, cx, cy, mean_colour, signature, template_score, detection_mode
        )

        roi_top = int(h * ROI_TOP_FRAC)
        spine = self._extract_spine(mask, roi_top) if found else []
        curve_geometry = self._estimate_curve_geometry(spine, w, h) if found else None

        error = cx - (w / 2.0) if found else 0.0
        if abs(error) < CENTER_DEADBAND_PX:
            error = 0.0

        lookahead_error = 0.0
        if found:
            lookahead_error = self._compute_lookahead_error(spine, w)
            if abs(lookahead_error) < CENTER_DEADBAND_PX:
                lookahead_error = 0.0

        with self._lock:
            self._spine = spine
            if found:
                self._last_seen_error = error
                self._last_seen_lookahead = lookahead_error
                self._last_found_ts = time.monotonic()

        return {
            "frame": frame,
            "state": cur_state,
            "found": found,
            "cx": cx,
            "cy": cy,
            "mask": mask,
            "contour": contour,
            "spine": spine,
            "curve_geometry": curve_geometry,
            "error": error,
            "lookahead_error": lookahead_error,
        }

    def _drive_step(self, frame_data):
        found = frame_data["found"]
        error = frame_data["error"]
        lookahead_error = frame_data["lookahead_error"]
        curve_geometry = frame_data["curve_geometry"]

        if self._following and found:
            self._control_cycle_index += 1
            high_speed_skip = (
                (self._control_cycle_index % 2 == 0)
                and self._last_drive_command is not None
            )
            self._control_update_skipped = high_speed_skip
            if high_speed_skip:
                self._reuse_last_drive_command()
                return

            self._update_path_targets(curve_geometry, error, lookahead_error)
            steer = self._compute_steering(error, lookahead_error, curve_geometry)
            duty_cap = self._car.power_limiter.max_safe_duty
            curve_signal = max(
                abs(steer),
                0.6 * abs(lookahead_error) / max(CAMERA_W / 2.0, 1.0) * MAX_STEER,
            )
            curve_mag = min(1.0, curve_signal / MAX_STEER)
            curve_factor = max(
                CURVE_SPEED_MIN_FRAC,
                1.0 - (curve_mag ** 1.5) * (1.0 - CURVE_SPEED_MIN_FRAC)
            )
            speed = max(0.0, min(duty_cap, float(self._base_speed) * curve_factor))

            # Apply gyro + heading corrections to steering angle
            steer = self._compute_motion_corrections(error, steer)

            self._issue_drive_command(speed, steer)
            return

        if self._following and not found:
            self._control_update_skipped = False
            if self._target_heading is not None:
                now = time.monotonic()
                dt = max(1e-3, min(0.1, now - self._last_target_ts))
                self._last_target_ts = now
                self._target_heading = self._wrap_angle_deg(
                    self._target_heading + self._target_yaw_rate_dps * dt
                )
            recovery = self._compute_recovery_command()
            if recovery is None:
                self._recovery_phase = "idle"
                self._drive_command_active = False
                return

            self._gyro_correction = 0.0
            self._heading_correction = 0.0
            self._vision_steer = 0.0
            self._feedforward_steer = 0.0
            if recovery["mode"] == "brake":
                self._last_drive_command = None
                self._drive_command_active = False
                self._car.brake()
                return

            steer = float(recovery["steer"])
            self._filtered_steer = steer
            self._last_steer = steer
            speed = float(recovery["speed"])
            self._issue_drive_command(speed, steer)
            return

        self._control_update_skipped = False

    def _log_telemetry(self, frame_data, log_counter):
        if not self._following:
            return

        found = frame_data["found"]
        cx = frame_data["cx"]
        error = frame_data["error"]
        cur_state = frame_data["state"]
        t = time.monotonic() - self._t0
        self._csv_file.write(
            f"{t:.3f},{cur_state},{int(found)},{cx:.2f},{error:.2f},"
            f"{self._lookahead_error:.1f},"
            f"{self._vision_steer:.2f},{self._gyro_correction:.2f},"
            f"{self._heading_correction:.2f},{self._motor_bias:.2f},"
            f"{self._last_steer:.2f},"
            f"{self._gyro_z:.2f},"
            f"{self._mag_heading if self._mag_heading is not None else 0.0:.2f},"
            f"{self._fused_heading:.2f},"
            f"{self._target_heading if self._target_heading is not None else 0.0:.2f},"
            f"{self._target_yaw_rate_dps:.2f},"
            f"{self._fused_yaw_rate_dps:.2f},"
            f"{self._curve_heading_deg:.2f},"
            f"{self._curve_curvature:.3f},"
            f"{self._predicted_line_cx:.2f},"
            f"{self._feedforward_steer:.2f},"
            f"{self._base_speed},"
            f"{self._pan_angle:.1f},"
            f"{self._detection_mode},{self._template_confidence:.2f},"
            f"{self._gyro_bias_z:.3f},{self._recovery_phase},{int(self._control_update_skipped)}\n"
        )
        if log_counter % 8 == 0:
            self._csv_file.flush()

        if log_counter % 8 != 0:
            return

        log.debug(
            "mode=%s err=%+.1fpx la=%+.0f curve=(hdg=%+.1f curv=%+.2f pred=%.1f yaw*=%.1f) "
            "steer=%+.1f° (vis=%.1f ff=%.1f I=%.1f gyro=%+.1f head=%+.1f skip=%s) "
            "gz=%+.1f°/s bias=%+.2f yaw=%.1f hdg=%+.1f tgt=%s pan=%.0f°",
            self._detection_mode,
            error,
            self._lookahead_error,
            self._curve_heading_deg,
            self._curve_curvature,
            self._predicted_line_cx,
            self._target_yaw_rate_dps,
            self._last_steer,
            self._vision_steer,
            self._feedforward_steer,
            self._integral,
            self._gyro_correction,
            self._heading_correction,
            self._control_update_skipped,
            self._gyro_z,
            self._gyro_bias_z,
            self._fused_yaw_rate_dps,
            self._fused_heading,
            f"{self._target_heading:+.1f}" if self._target_heading is not None else "None",
            self._pan_angle,
        )

    def _loop(self):
        prev_state = None
        log_counter = 0

        while self._running:
            frame = self._capture_frame()
            if frame is None:
                time.sleep(0.03)
                continue

            self._read_sensors()
            frame_data = self._process_frame(frame)

            if frame_data["state"] != prev_state:
                log.info("State: %s → %s", prev_state or "(init)", frame_data["state"])
                prev_state = frame_data["state"]

            self._drive_step(frame_data)

            log_counter += 1
            self._log_telemetry(frame_data, log_counter)

            annotated = self._annotate(
                frame,
                frame_data["found"],
                frame_data["cx"],
                frame_data["cy"],
                frame_data["contour"],
                frame_data["spine"],
            )
            with self._lock:
                self._annotated_frame = annotated

            time.sleep(0.025)  # ~40 fps cap


# ===================================================================
#  WEB SERVER
# ===================================================================
def create_app(follower):
    app = Flask(__name__, template_folder=os.path.join(SCRIPT_DIR, "templates"))

    @app.route("/")
    def index():
        return render_template("follow_line.html")

    @app.route("/stream")
    def stream():
        def gen():
            while True:
                jpg = follower.get_jpeg()
                if jpg:
                    yield (b"--frame\r\n"
                           b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                time.sleep(0.03)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/status")
    def api_status():
        return jsonify(follower.get_status())

    @app.route("/api/start", methods=["POST"])
    def api_start():
        follower.start_following()
        return jsonify({"ok": True})

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        follower.stop_following()
        return jsonify({"ok": True})

    return app


# ===================================================================
#  ENTRY POINT
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="Line-following mode with web UI")
    parser.add_argument("--port",  type=int, default=8080, help="Web UI port (default 8080)")
    parser.add_argument("--tilt",  type=int, default=DEFAULT_TILT,
                        help=f"Tilt servo angle — 90=horizontal, 120=max down (default {DEFAULT_TILT})")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"Base driving speed %% (default {DEFAULT_SPEED})")
    parser.add_argument("--bias",  type=float, default=DEFAULT_MOTOR_BIAS,
                        help=f"Static motor trim (degrees). Negative compensates right-drift. "
                             f"Tune in steps of 1-2 (default {DEFAULT_MOTOR_BIAS})")
    parser.add_argument("--no-mpu", dest="no_mpu", action="store_true",
                        help="Disable MPU gyro heading correction (vision only)")
    args = parser.parse_args()

    follower = LineFollower(
        tilt=args.tilt, speed=args.speed, motor_bias=args.bias,
        use_mpu=not args.no_mpu,
    )

    def _shutdown(*_):
        follower.cleanup()
        log.info("Exiting")
        sys.exit(0)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    app = create_app(follower)
    log.info("Web UI → http://0.0.0.0:%d", args.port)
    try:
        from waitress import serve
    except ImportError:
        log.error("waitress is required to serve the web UI. Install dependencies from requirements.txt.")
        follower.cleanup()
        raise SystemExit(1)
    serve(app, host="0.0.0.0", port=args.port, threads=8)


if __name__ == "__main__":
    main()
