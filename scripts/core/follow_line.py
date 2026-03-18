#!/usr/bin/env python3
"""
follow_line.py — Line-following mode with web interface.

Tilts the camera servo downward, detects any-color line using
per-channel local-contrast analysis, and provides a web UI with
a live annotated stream + Start/Stop controls.

Uses MPU gyroscope for heading correction and LM393 wheel encoders
for differential speed balancing to keep the rover driving straight.

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
log.info("Log file: %s", _log_file)

# ── third-party ────────────────────────────────────────
import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

# ── project hardware ───────────────────────────────────
from pico_sensor_reader import (
    init_pico_reader,
    send_pan_tilt,
    get_gyro_z,
    get_rpm,
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
MAX_DIFF_FRAC = 0.65          # max left/right speed difference
PIVOT_THRESHOLD = 0.92        # only allow reverse near absolute max steer

# Static motor trim — compensates for different-motor torque imbalance.
# Negative = nudge left (counteract right-drift).  Tune with --bias.
# Start at 0; if car consistently drifts right, decrease in steps of 1-2.
DEFAULT_MOTOR_BIAS = 0.0

# Gyro rate-damping correction
# Opposes instantaneous yaw rate directly.  Fades to zero as vision error
# grows so that intentional line-following turns are not fought.
# Physical test confirmed: Left/CCW = +gz, Right/CW = -gz (standard RHR).
#   gz > 0 (yawing left):  slow left + speed right  → resists leftward spin
#   gz < 0 (yawing right): speed left + slow right  → resists rightward spin
GYRO_KP         = 0.18  # per-side PWM % per °/s
GYRO_MAX_CORR   = 6.0   # per-side PWM clamp (%)
GYRO_DEADZONE   = 4.0   # °/s
GYRO_FADE_STEER = 20.0  # fade gyro damping as steering angle grows

# Magnetometer-assisted heading hold.
# Vision remains primary; fused heading only helps stabilise straight travel
# and carry through brief weak/noisy detections without fighting real turns.
MAG_HEADING_KP        = 0.10   # per-side PWM % per degree of heading error
MAG_HEADING_MAX_CORR  = 5.0    # per-side PWM clamp (%)
MAG_BLEND_ALPHA       = 0.98   # complementary filter gyro weight
MAG_LOCK_CENTER_PX    = 18     # capture target heading only when line is well centred
MAG_FADE_PX           = 65     # fade heading assist out during larger visual turns
MAG_MIN_FIELD         = 1e-3   # ignore near-zero field vectors

# Encoder differential correction (LM393) — per-side PWM speed bias.
# Mirrors main.py: apply half the correction to each side instead of
# converting RPM error into a steering angle.
# front_left encoder is absent; right side uses rear_right + front_right.
# Correction is gated: both sides must be spinning above ENCODER_MIN_RPM
# (prevents spurious corrections when one encoder hasn’t started firing yet)
# and suppressed when vision is actively steering.
ENCODER_KP          = 1.5   # % PWM per normalised RPM-imbalance
ENCODER_DEADZONE    = 2.0   # ignore RPM difference below this
ENCODER_MAX_CORR    = 12.0  # clamp per-side PWM correction (%)
ENCODER_MIN_RPM     = 3.0   # both sides must exceed this before correction fires
ENCODER_SUPPRESS_PX = 50    # suppress encoder corr. during hard vision turns

# Rover geometry and motion model.
# This rover is skid-steer, so encoder-derived yaw rate depends primarily on
# left/right track width rather than steering-linkage geometry.
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

# Path-dynamics fusion: use vision-estimated curve, encoder speed, gyro yaw
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

# Morphology kernel for cleaning up the binary mask
_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


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
                 use_mpu=True, use_encoders=True):
        self._tilt = tilt
        self._base_speed = speed
        self._motor_bias = motor_bias
        self._use_mpu      = use_mpu
        self._use_encoders = use_encoders
        log.info("Sensors: MPU=%s  Encoders=%s",
                 "ON" if use_mpu else "OFF",
                 "ON" if use_encoders else "OFF")

        # --- hardware ---
        init_pico_reader()
        send_pan_tilt(FIXED_PAN, self._tilt)
        time.sleep(0.4)
        log.info("Camera fixed at pan=%d° tilt=%d°", FIXED_PAN, self._tilt)

        self._car = CarSystem()
        # Attach per-wheel dynamic PWM controller so every wheel is
        # independently closed-loop controlled via its encoder.
        # This automatically compensates for the RR plastic gearbox
        # requiring more PWM to reach the same RPM as the metal gears.
        if use_encoders:
            self._car.attach_wheel_sync(get_rpm, get_freshness_fn=is_pico_fresh)
        log.info("Motor system ready (wheel_sync=%s)",
                 "ON" if use_encoders else "OFF")

        # --- camera ---
        if _USE_PICAMERA2:
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
        self._line_cx = CAMERA_W // 2
        self._line_cy = CAMERA_H // 2
        self._confirm_count = 0
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
        self._target_yaw_rate_dps = 0.0
        self._last_target_ts = time.monotonic()

        # --- sensor state (gyro + encoders) ---
        self._gyro_z = 0.0          # latest yaw rate (°/s)
        self._mag_x = 0.0
        self._mag_y = 0.0
        self._mag_z = 0.0
        self._mag_heading = None
        self._fused_heading = 0.0
        self._target_heading = None
        self._heading_correction = 0.0
        self._last_sensor_ts = time.monotonic()
        self._rpm_left  = 0.0       # latest left-side RPM
        self._rpm_right = 0.0       # latest right-side RPM
        self._forward_speed_mps = 0.0
        self._encoder_yaw_rate_dps = 0.0
        self._fused_yaw_rate_dps = 0.0
        self._last_steer = 0.0      # for telemetry / logging
        self._gyro_correction    = 0.0
        self._encoder_correction = 0.0   # kept for CSV backward-compat; always 0
        self._vision_steer = 0.0
        self._integral   = 0.0      # PID integral accumulator
        self._prev_error = 0.0      # PID derivative (previous error)

        # --- per-wheel applied PWM (read from wheel_sync telemetry each tick) ---
        self._pwm_fl = 0.0   # front-left  (mirrored from RL — no encoder)
        self._pwm_fr = 0.0   # front-right
        self._pwm_rl = 0.0   # rear-left
        self._pwm_rr = 0.0   # rear-right  (plastic gearbox — boosted by sync)

        # --- CSV telemetry log (for visualization) ---
        self._csv_path = os.path.join(LOG_DIR,
            f"line_follow_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self._csv_file = open(self._csv_path, "w")
        self._csv_file.write(
            "time,state,found,cx,error,lookahead_err,vis_steer,gyro_corr,head_corr,enc_corr,"
            "motor_bias,total_steer,gyro_z,mag_heading,fused_heading,target_heading,target_yaw_rate,"
            "yaw_rate,enc_yaw_rate,curve_heading,curve_curvature,speed_mps,rpm_l,rpm_r,speed,pan_angle,"
            "pwm_fl,pwm_fr,pwm_rl,pwm_rr\n")
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
                self._prev_error = 0.0
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
                self._target_yaw_rate_dps = 0.0
                self._target_heading = None
                self._last_target_ts = time.monotonic()
                log.info("START following (speed=%d%%)", self._base_speed)

    def stop_following(self):
        with self._lock:
            self._following = False
            self.state = self.STOPPED
            self._locked_cx     = None   # release line lock
            self._locked_colour = None
            self._locked_signature = None
            self._path_history.clear()
            self._spine = []
            self._pan_angle = float(FIXED_PAN)
            self._target_heading = None
            self._filtered_error = 0.0
            self._filtered_lookahead = 0.0
            self._filtered_steer = 0.0
            self._last_control_ts = time.monotonic()
            self._last_steer = 0.0
            self._last_seen_error = 0.0
            self._last_seen_lookahead = 0.0
            self._curve_heading_deg = 0.0
            self._curve_curvature = 0.0
            self._target_yaw_rate_dps = 0.0
            self._last_target_ts = time.monotonic()
        self._car.stop()
        log.info("STOP following (user request)")

    def get_status(self):
        with self._lock:
            return {
                "state":       self.state,
                "line_x":      self._line_cx,
                "frame_w":     CAMERA_W,
                "frame_h":     CAMERA_H,
                "gyro_z":      round(self._gyro_z, 1),
                "mag_heading": None if self._mag_heading is None else round(self._mag_heading, 1),
                "fused_heading": round(self._fused_heading, 1),
                "target_heading": None if self._target_heading is None else round(self._target_heading, 1),
                "target_yaw_rate": round(self._target_yaw_rate_dps, 1),
                "yaw_rate": round(self._fused_yaw_rate_dps, 1),
                "enc_yaw_rate": round(self._encoder_yaw_rate_dps, 1),
                "speed_mps": round(self._forward_speed_mps, 3),
                "curve_heading": round(self._curve_heading_deg, 1),
                "curve_curvature": round(self._curve_curvature, 2),
                "rpm_l":       round(self._rpm_left, 1),
                "rpm_r":       round(self._rpm_right, 1),
                "steer":       round(self._last_steer, 1),
                "pan_angle":   round(self._pan_angle, 1),
                "lookahead":   round(self._lookahead_error, 1),
                "locked":      self._locked_cx is not None,
                "spine":       list(self._spine),
                "trail":       list(self._path_history)[-20:],
                "pwm_fl":      self._pwm_fl,
                "pwm_fr":      self._pwm_fr,
                "pwm_rl":      self._pwm_rl,
                "pwm_rr":      self._pwm_rr,
                "sync_active": self._car.wheel_sync is not None,
            }

    def get_jpeg(self):
        """Return latest annotated frame as JPEG bytes (or None)."""
        with self._lock:
            f = self._annotated_frame
        if f is None:
            return None
        ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return buf.tobytes() if ok else None

    def cleanup(self):
        self._running = False
        self._car.stop()
        self._car.cleanup()
        if _USE_PICAMERA2:
            self._cam.stop()
        else:
            self._cam.release()
        try:
            self._csv_file.close()
        except Exception:
            pass
        log.info("Line follower shut down")

    # ─────────────────────────────────────────────────
    #  Line detection  (any colour, with line-lock)
    # ─────────────────────────────────────────────────
    def _detect_line(self, frame):
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        roi = frame[roi_top:, :]
        rh, rw = roi.shape[:2]

        # Optional: ignore telemetry overlay at bottom
        overlay_h = 24

        # Gaussian kernel ≈ 1/6 of ROI width, forced odd
        ksize = max(31, (rw // 6) | 1)

        # Detect dark or strongly coloured line against local background
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        bg = cv2.GaussianBlur(gray, (ksize, ksize), 0)
        dark = cv2.subtract(bg, gray)

        bg_bgr = cv2.GaussianBlur(roi, (ksize, ksize), 0)
        colour_delta = cv2.absdiff(roi, bg_bgr)
        db, dg, dr = cv2.split(colour_delta)
        colour_contrast = np.maximum(np.maximum(db, dg), dr)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        sat = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)
        sat_bg = cv2.GaussianBlur(sat, (ksize, ksize), 0)
        sat_boost = cv2.subtract(sat, sat_bg)

        # Binary mask
        _, dark_mask = cv2.threshold(dark, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, colour_mask = cv2.threshold(colour_contrast, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, sat_mask = cv2.threshold(sat_boost, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mask = cv2.bitwise_or(dark_mask, cv2.bitwise_and(colour_mask, sat_mask))

        # Remove bottom overlay region
        mask[rh-overlay_h:, :] = 0

        # Morphology
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)

        recovery_active = (
            self._following
            and self._line_lost_since > 0.0
            and (time.monotonic() - self._line_lost_since) < RECOVERY_RELAX_WINDOW
        )
        tracking_locked = self._locked_signature is not None or self._line_memory is not None
        min_line_height = max(40, int(rh * 0.18))
        min_line_elongation = 2.2
        min_spine_hits = max(4, SPINE_ROWS // 3)
        max_short_side = max(28.0, rw * 0.18)
        if tracking_locked:
            min_line_height = min(min_line_height, max(32, int(rh * 0.14)))
            min_line_elongation = min(min_line_elongation, 1.9)
            min_spine_hits = min(min_spine_hits, max(3, SPINE_ROWS // 4))
        if recovery_active and tracking_locked:
            min_line_height = min(min_line_height, max(24, int(rh * 0.10)))
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
            c_mask = np.zeros((rh, rw), dtype=np.uint8)
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

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        def _contour_signature(c, metrics=None):
            m = metrics if metrics is not None else _line_metrics(c)
            c_mask = np.zeros((rh, rw), dtype=np.uint8)
            cv2.drawContours(c_mask, [c], -1, 255, -1)
            mean_bgr = np.array(cv2.mean(roi, mask=c_mask)[:3], dtype=np.float32)
            mean_hsv = np.array(cv2.mean(hsv, mask=c_mask)[:3], dtype=np.float32)
            M_ = cv2.moments(c)
            if M_["m00"] == 0:
                cx_ = m["x"] + m["w"] // 2
                cy_ = m["y"] + m["h"] // 2
            else:
                cx_ = int(M_["m10"] / M_["m00"])
                cy_ = int(M_["m01"] / M_["m00"])
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
            if cv2.contourArea(c) <= MIN_CONTOUR_AREA:
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
        if not candidates:
            return False, 0, 0, mask, None, None, None

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
                return False, 0, 0, mask, None, None, None

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
                return False, 0, 0, mask, None, None, None

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
                cx_ = int(M_["m10"] / M_["m00"])
                score -= 2.0 * abs(cx_ - locked_cx)

            if template is not None:
                score += 180.0 * cand.get("template_score", 0.0)

            return score

        best_cand = max(candidates, key=_score_candidate)
        best = best_cand["contour"]
        M = cv2.moments(best)
        if M["m00"] == 0:
            return False, 0, 0, mask, best, None, None

        cx = int(M["m10"] / M["m00"])
        cy_full = int(M["m01"] / M["m00"]) + roi_top

        signature = best_cand["signature"]
        mean_bgr = signature["bgr"]

        return True, cx, cy_full, mask, best, mean_bgr, signature

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
            x_mid = int(np.mean(cols))
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
        return la_x - (frame_w // 2)

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
            coeffs = np.polyfit(s, x, 2)
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
        self._target_yaw_rate_dps = 0.0

        if self._target_heading is None:
            self._target_heading = self._fused_heading

        if curve_geometry is not None:
            confidence = curve_geometry.get("confidence", 1.0)
            self._curve_heading_deg = float(curve_geometry["heading_deg"])
            self._curve_curvature = float(curve_geometry["curvature_norm"])

            speed_scale = max(
                0.35,
                min(1.25, abs(self._forward_speed_mps) / 0.22 if abs(self._forward_speed_mps) > 1e-3 else 0.35),
            )
            desired_yaw = confidence * (
                TARGET_YAW_HEADING_GAIN * self._curve_heading_deg
                + TARGET_YAW_CURVATURE_GAIN * self._curve_curvature * speed_scale
            )
            self._target_yaw_rate_dps = max(
                -MAX_TARGET_YAW_RATE_DPS,
                min(MAX_TARGET_YAW_RATE_DPS, desired_yaw),
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
            cv2.circle(out, (hx, hy), radius, (alpha, alpha, 0), -1)
        if n >= 2:
            for i in range(1, n):
                alpha = int(80 + 150 * (i / (n - 1)))
                cv2.line(out, history[i - 1], history[i], (0, alpha, alpha), 1)

        # ── Spine (projected path ahead of rover) ─────────────────────────
        # Draw the extracted spine points as a bright green path ahead
        if spine and len(spine) >= 2:
            pts = np.array(spine, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], False, (0, 255, 80), 3)
            for sx, sy in spine:
                cv2.circle(out, (sx, sy), 4, (0, 255, 80), -1)

        if found and contour is not None:
            # Draw contour outline (shift to full-frame coords)
            shifted = contour.copy()
            shifted[:, :, 1] += roi_top
            cv2.drawContours(out, [shifted], -1, (0, 255, 0), 2)

            # Centroid
            cv2.circle(out, (cx, cy), 8, (0, 255, 255), -1)

            # Steering arrow
            cv2.arrowedLine(out, (mid_x, h - 30), (cx, cy), (0, 200, 255), 3, tipLength=0.25)

            # Lock indicator — small padlock icon (circle + text)
            if self._locked_cx is not None:
                cv2.circle(out, (cx, cy), 14, (0, 200, 255), 2)
                cv2.putText(out, "LOCKED", (cx - 28, cy - 18),
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
            cv2.putText(out, f"err={err:+d}px", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Sensor telemetry overlay — two lines at bottom
        y1 = h - 28
        y0 = h - 12
        cv2.putText(out, f"pan={self._pan_angle:.0f}\u00b0  la={self._lookahead_error:+.0f}px  curv={self._curve_curvature:+.2f}",
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1)
        cv2.putText(out, f"gyro={self._gyro_z:+.1f} yaw={self._fused_yaw_rate_dps:+.1f} target={self._target_yaw_rate_dps:+.1f}", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(out, f"RPM L={self._rpm_left:.0f} R={self._rpm_right:.0f}",
                    (320, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(out, f"steer={self._last_steer:+.1f}",
                    (460, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return out

    # ─────────────────────────────────────────────────
    #  Main processing loop (runs on worker thread)
    # ─────────────────────────────────────────────────
    def _read_sensors(self):
        """Read MPU gyro, magnetometer, LM393 encoder RPMs, and wheel PWM."""
        now = time.monotonic()
        dt = min(max(now - self._last_sensor_ts, 0.0), 0.2)
        self._last_sensor_ts = now

        self._gyro_z = get_gyro_z()  # °/s yaw rate
        self._mag_x, self._mag_y, self._mag_z = get_magnetometer()
        field_sq = self._mag_x * self._mag_x + self._mag_y * self._mag_y
        if field_sq > (MAG_MIN_FIELD * MAG_MIN_FIELD):
            self._mag_heading = math.degrees(math.atan2(self._mag_y, self._mag_x))
            pred_heading = self._fused_heading + self._gyro_z * dt
            self._fused_heading = self._wrap_angle_deg(
                MAG_BLEND_ALPHA * pred_heading
                + (1.0 - MAG_BLEND_ALPHA) * self._mag_heading
            )
        else:
            self._mag_heading = None
            self._fused_heading = self._wrap_angle_deg(self._fused_heading + self._gyro_z * dt)

        rpm = get_rpm()
        # Left side: only rear_left available (front_left encoder is absent)
        self._rpm_left = rpm.get("rear_left", 0.0)
        # Right side: average rear_right and front_right when both are valid
        rr = rpm.get("rear_right", 0.0)
        fr = rpm.get("front_right", 0.0)
        if rr > 0.0 and fr > 0.0:
            self._rpm_right = (rr + fr) / 2.0
        elif rr > 0.0:
            self._rpm_right = rr
        else:
            self._rpm_right = fr

        left_speed_mps = (self._rpm_left / 60.0) * WHEEL_CIRCUMFERENCE_M
        right_speed_mps = (self._rpm_right / 60.0) * WHEEL_CIRCUMFERENCE_M
        self._forward_speed_mps = 0.5 * (left_speed_mps + right_speed_mps)
        self._encoder_yaw_rate_dps = math.degrees(
            (right_speed_mps - left_speed_mps) / max(ROVER_TRACK_WIDTH_M, 1e-6)
        )
        yaw_blend = min(
            YAW_RATE_BLEND_MAX,
            max(0.0, abs(self._forward_speed_mps) / 0.45) * YAW_RATE_BLEND_MAX,
        )
        self._fused_yaw_rate_dps = (
            (1.0 - yaw_blend) * self._gyro_z
            + yaw_blend * self._encoder_yaw_rate_dps
        )

        # Read per-wheel applied PWM from wheel_sync (available after first tick)
        if self._car.wheel_sync is not None:
            telem  = self._car.get_sync_telemetry()
            wheels = telem.get("wheels", {})
            self._pwm_fl = wheels.get("fl", {}).get("applied_pwm", 0.0)
            self._pwm_fr = wheels.get("fr", {}).get("applied_pwm", 0.0)
            self._pwm_rl = wheels.get("rl", {}).get("applied_pwm", 0.0)
            self._pwm_rr = wheels.get("rr", {}).get("applied_pwm", 0.0)

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

        blended = ((1.0 - lookahead_weight) * self._filtered_error
                   + lookahead_weight * self._filtered_lookahead)
        self._lookahead_error = self._filtered_lookahead

        geometry_steer = 0.0
        if curve_geometry is not None:
            confidence = curve_geometry.get("confidence", 1.0)
            geometry_steer = confidence * (
                PATH_HEADING_STEER_KP * float(curve_geometry["heading_deg"])
                + PATH_CURVATURE_STEER_KP * float(curve_geometry["curvature_norm"])
            )

        # Time-based PID
        p = STEERING_KP * blended

        deriv = (blended - self._prev_error) / dt
        d = STEERING_KD * deriv
        self._prev_error = blended

        if abs(vision_error) >= CENTER_DEADBAND_PX and self._last_steer * vision_error < 0.0:
            self._integral *= 0.72

        raw_pre_i = p + self._integral + d + geometry_steer + self._motor_bias
        if abs(raw_pre_i) < MAX_STEER or (raw_pre_i * blended < 0):
            self._integral += STEERING_KI * blended * dt
            self._integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, self._integral))

        vision_steer = p + self._integral + d + geometry_steer + self._motor_bias
        self._vision_steer = vision_steer

        target_steer = max(-MAX_STEER, min(MAX_STEER, vision_steer))

        # Slew-rate limit the output
        max_step = STEER_SLEW_RATE * dt
        delta = target_steer - self._filtered_steer
        delta = max(-max_step, min(max_step, delta))
        self._filtered_steer += delta

        self._last_steer = self._filtered_steer
        return self._filtered_steer

    def _mix_drive_command(self, speed, steer):
        duty_cap = self._car.power_limiter.max_safe_duty
        speed = max(0.0, min(duty_cap, float(speed)))
        steer_frac = max(-1.0, min(1.0, steer / MAX_STEER))

        diff = MAX_DIFF_FRAC * steer_frac
        left_cmd = speed * (1.0 + diff)
        right_cmd = speed * (1.0 - diff)

        peak = max(left_cmd, right_cmd, 1.0)
        scale = min(1.0, duty_cap / peak)
        left_cmd *= scale
        right_cmd *= scale

        l_fwd = True
        r_fwd = True
        if abs(steer_frac) > PIVOT_THRESHOLD:
            pivot_frac = (abs(steer_frac) - PIVOT_THRESHOLD) / (1.0 - PIVOT_THRESHOLD)
            pivot_frac = max(0.0, min(1.0, pivot_frac))
            if steer_frac > 0:
                right_cmd *= (1.0 - 2.0 * pivot_frac)
                if right_cmd < 0:
                    r_fwd = False
                    right_cmd = abs(right_cmd)
            else:
                left_cmd *= (1.0 - 2.0 * pivot_frac)
                if left_cmd < 0:
                    l_fwd = False
                    left_cmd = abs(left_cmd)

        base_l = min(duty_cap, left_cmd)
        base_r = min(duty_cap, right_cmd)
        return base_l, base_r, l_fwd, r_fwd

    def _compute_recovery_command(self):
        if self._line_lost_since == 0.0:
            return None

        elapsed = time.monotonic() - self._line_lost_since
        if elapsed < 0.0 or elapsed >= LINE_LOST_TIMEOUT:
            return None

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
        return (*self._mix_drive_command(speed, steer), steer)

    def _compute_motion_corrections(self, vision_error, base_l, base_r):
        """Track target yaw rate + fused heading with gyro, encoders, and mag."""
        duty_cap = self._car.power_limiter.max_safe_duty
        left_speed  = float(base_l)
        right_speed = float(base_r)

        gyro_corr = 0.0
        if self._use_mpu:
            rate_error = self._target_yaw_rate_dps - self._fused_yaw_rate_dps
            if abs(rate_error) > GYRO_DEADZONE:
                raw = GYRO_KP * rate_error
                gyro_corr = max(-GYRO_MAX_CORR, min(GYRO_MAX_CORR, raw))
                half = gyro_corr / 2.0
                left_speed  = max(0.0, min(duty_cap, left_speed  - half))
                right_speed = max(0.0, min(duty_cap, right_speed + half))
        self._gyro_correction = round(gyro_corr, 2)

        heading_corr = 0.0
        if self._target_heading is not None:
            heading_error = self._wrap_angle_deg(self._target_heading - self._fused_heading)
            curve_fade = min(1.0, abs(self._target_yaw_rate_dps) / max(MAX_TARGET_YAW_RATE_DPS * 0.45, 1.0))
            fade = max(0.25 * curve_fade, max(0.0, 1.0 - abs(vision_error) / MAG_FADE_PX))
            raw = MAG_HEADING_KP * heading_error * fade
            heading_corr = max(-MAG_HEADING_MAX_CORR, min(MAG_HEADING_MAX_CORR, raw))
            half = heading_corr / 2.0
            left_speed  = max(0.0, min(duty_cap, left_speed  - half))
            right_speed = max(0.0, min(duty_cap, right_speed + half))
        self._heading_correction = round(heading_corr, 2)

        log.debug("motion: yaw*=%.1f yaw=%.1f enc=%.1f gyro=%.2f hdg=%.2f fused=%+.1f target=%s → L=%.1f R=%.1f RR_pwm=%.1f",
                  self._target_yaw_rate_dps, self._fused_yaw_rate_dps, self._encoder_yaw_rate_dps,
                  gyro_corr, heading_corr, self._fused_heading,
                  f"{self._target_heading:+.1f}" if self._target_heading is not None else "None",
                  left_speed, right_speed, self._pwm_rr)

        return left_speed, right_speed

    def _loop(self):
        _prev_state = None
        _log_counter = 0

        while self._running:
            # 1. Capture
            if _USE_PICAMERA2:
                frame = self._cam.capture_array()
            else:
                ok, frame = self._cam.read()
                if not ok:
                    time.sleep(0.03)
                    continue
            if frame is None:
                time.sleep(0.03)
                continue

            # 1b. Flip 180° (camera is mounted inverted)
            frame = cv2.flip(frame, -1)

            h, w = frame.shape[:2]

            # 1c. Read sensors
            self._read_sensors()

            # 2. Detect (line-lock applied internally)
            found, cx, cy, mask, contour, mean_colour, signature = self._detect_line(frame)

            # 3. Update state machine + line-lock
            with self._lock:
                if found:
                    self._line_cx = cx
                    self._line_cy = cy
                    self._confirm_count = min(self._confirm_count + 1,
                                              LINE_CONFIRM_FRAMES + 5)
                    self._line_lost_since = 0.0

                    if self._confirm_count >= LINE_CONFIRM_FRAMES:
                        if self.state != self.STOPPED:
                            # Establish lock on the first confirmed detection
                            if self._locked_cx is None:
                                self._locked_cx     = cx
                                self._locked_colour = mean_colour
                                self._locked_signature = self._copy_signature(signature)
                                self._line_memory = self._blend_signature(
                                    self._line_memory, signature, MEMORY_BLEND_ALPHA)
                                log.info("Line locked at cx=%d colour=%.0f,%.0f,%.0f",
                                         cx, *(mean_colour if mean_colour is not None
                                               else (0, 0, 0)))
                            else:
                                # Slide the spatial anchor toward current cx so the
                                # lock keeps up with curves.  Rate 0.15 (~6.5 frame
                                # time-constant) is faster than the old 0.08 so that
                                # tight curves don't cause the lock to lag and reject
                                # the correct contour via LOCK_SPATIAL_MARGIN.
                                self._locked_cx = int(
                                    0.85 * self._locked_cx + 0.15 * cx)
                                if mean_colour is not None:
                                    self._locked_colour = (
                                        0.85 * self._locked_colour + 0.15 * mean_colour
                                    )
                                self._locked_signature = self._blend_signature(
                                    self._locked_signature, signature, LOCK_BLEND_ALPHA)
                                self._line_memory = self._blend_signature(
                                    self._line_memory, signature, MEMORY_BLEND_ALPHA)

                            if self._following:
                                self.state = self.FOLLOWING
                            elif self.state not in (self.FOLLOWING, self.STOPPED):
                                self.state = self.LINE_DETECTED

                    # Update path history with each confirmed centroid
                    if self._confirm_count >= LINE_CONFIRM_FRAMES and self.state != self.STOPPED:
                        self._path_history.append((cx, cy))

                else:
                    self._confirm_count = max(0, self._confirm_count - 2)
                    if self._following:
                        if self._line_lost_since == 0.0:
                            self._line_lost_since = time.monotonic()
                        elif (time.monotonic() - self._line_lost_since) > LINE_LOST_TIMEOUT:
                            self._following = False
                            self.state = self.LINE_LOST
                            self._integral      = 0.0   # reset PID state so windup
                            self._prev_error    = 0.0   # doesn't fire on next detection
                            self._filtered_error = 0.0
                            self._filtered_lookahead = 0.0
                            self._filtered_steer = 0.0
                            self._last_control_ts = time.monotonic()
                            self._last_steer = 0.0
                            self._vision_steer = 0.0
                            self._curve_heading_deg = 0.0
                            self._curve_curvature = 0.0
                            self._target_yaw_rate_dps = 0.0
                            self._locked_cx     = None  # release lock — will re-acquire
                            self._locked_colour = None
                            self._locked_signature = None
                            self._target_heading = None
                            self._last_target_ts = time.monotonic()
                            self._car.brake()
                            log.warning("Line lost for %.1fs — braking", LINE_LOST_TIMEOUT)
                    else:
                        if self.state not in (self.STOPPED, self.LINE_LOST):
                            self.state = self.SEARCHING

                cur_state = self.state

            # Extract spine (projected path ahead) for annotation + UI
            roi_top = int(h * ROI_TOP_FRAC)
            spine = self._extract_spine(mask, roi_top) if found else []
            curve_geometry = self._estimate_curve_geometry(spine, w, h) if found else None
            with self._lock:
                self._spine = spine

            # Log state transitions
            if cur_state != _prev_state:
                log.info("State: %s → %s", _prev_state or "(init)", cur_state)
                _prev_state = cur_state

            # 4. Motor control (when following and line visible)
            error = cx - (w // 2) if found else 0
            if abs(error) < CENTER_DEADBAND_PX:
                error = 0
            lookahead_error = 0.0
            if found:
                lookahead_error = self._compute_lookahead_error(spine, w)
                if abs(lookahead_error) < CENTER_DEADBAND_PX:
                    lookahead_error = 0
                self._last_seen_error = error
                self._last_seen_lookahead = lookahead_error
                self._last_found_ts = time.monotonic()
            if self._following and found:
                self._update_path_targets(curve_geometry, error, lookahead_error)
                steer = self._compute_steering(error, lookahead_error, curve_geometry)
                duty_cap = self._car.power_limiter.max_safe_duty
                # Curve-adaptive speed: slow down proportionally when cornering hard.
                # Reduces overshoot momentum and gives the rover more time to turn.
                curve_mag = abs(steer) / MAX_STEER
                curve_factor = max(
                    CURVE_SPEED_MIN_FRAC,
                    1.0 - (curve_mag ** 1.5) * (1.0 - CURVE_SPEED_MIN_FRAC)
                )
                speed = max(0, min(duty_cap, int(self._base_speed * curve_factor)))
                base_l, base_r, l_fwd, r_fwd = self._mix_drive_command(speed, steer)

                # Straight-line corrections (gyro + encoder) only apply when
                # both sides drive forward — skip during pivot turns.
                if l_fwd and r_fwd:
                    left_speed, right_speed = self._compute_motion_corrections(
                        error, base_l, base_r)
                else:
                    left_speed, right_speed = base_l, base_r
                self._car._set_raw_motors(left_speed, right_speed, l_fwd, r_fwd)
            elif self._following and not found:
                if self._target_heading is not None:
                    now = time.monotonic()
                    dt = max(1e-3, min(0.1, now - self._last_target_ts))
                    self._last_target_ts = now
                    self._target_heading = self._wrap_angle_deg(
                        self._target_heading + self._target_yaw_rate_dps * dt
                    )
                recovery = self._compute_recovery_command()
                if recovery is not None:
                    left_speed, right_speed, l_fwd, r_fwd, steer = recovery
                    self._gyro_correction = 0.0
                    self._heading_correction = 0.0
                    self._vision_steer = 0.0
                    self._filtered_steer = steer
                    self._last_steer = steer
                    self._car._set_raw_motors(left_speed, right_speed, l_fwd, r_fwd)

            # 4b. CSV + debug logging (every iteration while following)
            _log_counter += 1
            if self._following:
                t = time.monotonic() - self._t0
                self._csv_file.write(
                    f"{t:.3f},{cur_state},{int(found)},{cx},{error},"
                    f"{self._lookahead_error:.1f},"
                    f"{self._vision_steer:.2f},{self._gyro_correction:.2f},"
                    f"{self._heading_correction:.2f},{self._encoder_correction:.2f},{self._motor_bias:.2f},"
                    f"{self._last_steer:.2f},"
                    f"{self._gyro_z:.2f},"
                    f"{self._mag_heading if self._mag_heading is not None else 0.0:.2f},"
                    f"{self._fused_heading:.2f},"
                    f"{self._target_heading if self._target_heading is not None else 0.0:.2f},"
                    f"{self._target_yaw_rate_dps:.2f},"
                    f"{self._fused_yaw_rate_dps:.2f},"
                    f"{self._encoder_yaw_rate_dps:.2f},"
                    f"{self._curve_heading_deg:.2f},"
                    f"{self._curve_curvature:.3f},"
                    f"{self._forward_speed_mps:.3f},"
                    f"{self._rpm_left:.1f},"
                    f"{self._rpm_right:.1f},{self._base_speed},"
                    f"{self._pan_angle:.1f},"
                    f"{self._pwm_fl:.1f},{self._pwm_fr:.1f},"
                    f"{self._pwm_rl:.1f},{self._pwm_rr:.1f}\n")
                if _log_counter % 8 == 0:   # ~5 Hz
                    log.debug(
                        "err=%+dpx la=%+.0f curve=(hdg=%+.1f curv=%+.2f yaw*=%.1f) "
                        "steer=%+.1f° (vis=%.1f I=%.1f gyro=%+.1f head=%+.1f) "
                        "gz=%+.1f°/s yaw=%.1f encYaw=%.1f hdg=%+.1f tgt=%s v=%.2f "
                        "rpmL=%.0f rpmR=%.0f "
                        "PWM FL=%.1f FR=%.1f RL=%.1f RR=%.1f pan=%.0f°",
                        error, self._lookahead_error,
                        self._curve_heading_deg, self._curve_curvature, self._target_yaw_rate_dps,
                        self._last_steer,
                        self._vision_steer, self._integral, self._gyro_correction, self._heading_correction,
                        self._gyro_z, self._fused_yaw_rate_dps, self._encoder_yaw_rate_dps, self._fused_heading,
                        f"{self._target_heading:+.1f}" if self._target_heading is not None else "None",
                        self._forward_speed_mps, self._rpm_left, self._rpm_right,
                        self._pwm_fl, self._pwm_fr, self._pwm_rl, self._pwm_rr,
                        self._pan_angle,
                    )

            # 5. Annotate for web stream
            annotated = self._annotate(frame, found, cx, cy, contour, spine)
            with self._lock:
                self._annotated_frame = annotated

            time.sleep(0.025)  # ~40 fps cap


# ===================================================================
#  WEB SERVER (Flask — no external JS dependencies)
# ===================================================================
HTML_PAGE = """<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Follower</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111827;color:#f3f4f6;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
     display:flex;justify-content:center;padding:12px}
.wrap{max-width:720px;width:100%}
h1{text-align:center;font-size:1.4rem;margin-bottom:10px;letter-spacing:.5px}
.vid{position:relative;background:#000;border-radius:12px;overflow:hidden;width:100%}
.vid img{width:100%;display:block}
/* Canvas sits exactly on top of the img, pointer-events off so clicks pass through */
.vid canvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.bar{display:flex;align-items:center;justify-content:center;gap:8px;
     margin:14px 0;padding:14px;border-radius:10px;font-weight:600;font-size:1.05rem;
     transition:background .3s}
.dot{width:12px;height:12px;border-radius:50%;flex-shrink:0}

.s-search  {background:#292524}.s-search  .dot{background:#f59e0b;animation:pulse 1.2s infinite}
.s-detect  {background:#14532d}.s-detect  .dot{background:#22c55e}
.s-follow  {background:#1e3a5f}.s-follow  .dot{background:#3b82f6;animation:pulse .6s infinite}
.s-lost    {background:#450a0a}.s-lost    .dot{background:#ef4444}
.s-stopped {background:#1f2937}.s-stopped .dot{background:#6b7280}

@keyframes pulse{0%,100%{opacity:1}50%{opacity:.25}}

.ctrls{display:flex;gap:12px;margin-top:14px}
.btn{flex:1;padding:16px;border:none;border-radius:10px;font-size:1.15rem;
     font-weight:700;cursor:pointer;transition:background .2s,transform .1s;
     letter-spacing:.3px}
.btn:active{transform:scale(.97)}
.btn-go{background:#16a34a;color:#fff}
.btn-go:disabled{background:#374151;color:#6b7280;cursor:not-allowed;transform:none}
.btn-stop{background:#dc2626;color:#fff}
.btn-stop:disabled{background:#374151;color:#6b7280;cursor:not-allowed;transform:none}
.info{text-align:center;margin-top:12px;font-size:.8rem;color:#6b7280}
.lock-badge{display:inline-block;margin-left:8px;padding:2px 7px;border-radius:6px;
            font-size:.72rem;font-weight:700;letter-spacing:.4px;vertical-align:middle;
            background:#0e4429;color:#4ade80;border:1px solid #4ade80}
</style>
</head><body>
<div class="wrap">
  <h1>&#x1F6E4;&#xFE0F; Line Follower</h1>
  <div class="vid">
    <img id="feed" src="/stream" alt="camera">
    <canvas id="overlay"></canvas>
  </div>

  <div id="bar" class="bar s-search">
    <div class="dot"></div>
    <span id="lbl">Searching for line&hellip;</span>
    <span id="lockbadge" class="lock-badge" style="display:none">&#x1F512; LOCKED</span>
  </div>

  <div class="ctrls">
    <button id="bgo" class="btn btn-go" disabled onclick="go()">&#9654; START</button>
    <button id="bst" class="btn btn-stop" disabled onclick="st()">&#9632; STOP</button>
  </div>
  <p class="info">Camera pointed down &mdash; place a line (any colour) in view</p>
  <div id="telem" style="margin-top:10px;text-align:center;font-size:.75rem;color:#9ca3af;font-family:monospace"></div>
</div>

<script>
const bar        = document.getElementById("bar"),
      lbl        = document.getElementById("lbl"),
      bgo        = document.getElementById("bgo"),
      bst        = document.getElementById("bst"),
      lockbadge  = document.getElementById("lockbadge"),
      feed       = document.getElementById("feed"),
      canvas     = document.getElementById("overlay"),
      ctx        = canvas.getContext("2d");

const S = {
  SEARCHING:    {c:"s-search", t:"Searching for line\\u2026",       go:0, st:0},
  LINE_DETECTED:{c:"s-detect", t:"\\u2713 Line detected \\u2014 Ready!", go:1, st:0},
  FOLLOWING:    {c:"s-follow", t:"\\u27F6 Following line\\u2026",   go:0, st:1},
  LINE_LOST:    {c:"s-lost",   t:"\\u2717 Line lost \\u2014 Stopped", go:0, st:0},
  STOPPED:      {c:"s-stopped",t:"Stopped",                          go:0, st:0},
};

// Scale from camera native resolution to displayed canvas pixels.
// Dimensions are read from the first /api/status response so they
// stay correct if CAMERA_W / CAMERA_H are changed server-side.
let scaleX = 1, scaleY = 1;
let CAM_W = 1280, CAM_H = 720;  // defaults, overwritten by first status poll

function syncCanvasSize() {
  const rect = feed.getBoundingClientRect();
  canvas.width  = rect.width;
  canvas.height = rect.height;
  scaleX = rect.width  / CAM_W;
  scaleY = rect.height / CAM_H;
}
feed.addEventListener("load", syncCanvasSize);
window.addEventListener("resize", syncCanvasSize);
syncCanvasSize();

function px(camX) { return camX * scaleX; }
function py(camY) { return camY * scaleY; }

function drawOverlay(d) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // ── Trail (path history) ────────────────────────────────────────────────
  const trail = d.trail || [];
  if (trail.length >= 2) {
    for (let i = 1; i < trail.length; i++) {
      const t = i / (trail.length - 1);
      const alpha = (0.25 + 0.65 * t).toFixed(2);
      ctx.beginPath();
      ctx.moveTo(px(trail[i-1][0]), py(trail[i-1][1]));
      ctx.lineTo(px(trail[i][0]),   py(trail[i][1]));
      ctx.strokeStyle = `rgba(0,220,220,${alpha})`;
      ctx.lineWidth   = 2;
      ctx.stroke();
    }
    // Dot on most recent position
    const last = trail[trail.length - 1];
    ctx.beginPath();
    ctx.arc(px(last[0]), py(last[1]), 5, 0, 2 * Math.PI);
    ctx.fillStyle = "rgba(0,220,220,1)";
    ctx.fill();
  }

  // ── Spine (path ahead) ──────────────────────────────────────────────────
  const spine = d.spine || [];
  if (spine.length >= 2) {
    ctx.beginPath();
    ctx.moveTo(px(spine[0][0]), py(spine[0][1]));
    for (let i = 1; i < spine.length; i++) {
      ctx.lineTo(px(spine[i][0]), py(spine[i][1]));
    }
    ctx.strokeStyle = "rgba(0,255,80,0.85)";
    ctx.lineWidth   = 3;
    ctx.lineJoin    = "round";
    ctx.stroke();

    // Arrowhead at the farthest point
    const tip  = spine[spine.length - 1];
    const prev = spine[spine.length - 2];
    const ang  = Math.atan2(py(tip[1]) - py(prev[1]), px(tip[0]) - px(prev[0]));
    const hs   = 10;
    ctx.beginPath();
    ctx.moveTo(px(tip[0]), py(tip[1]));
    ctx.lineTo(px(tip[0]) - hs * Math.cos(ang - 0.4),
               py(tip[1]) - hs * Math.sin(ang - 0.4));
    ctx.lineTo(px(tip[0]) - hs * Math.cos(ang + 0.4),
               py(tip[1]) - hs * Math.sin(ang + 0.4));
    ctx.closePath();
    ctx.fillStyle = "rgba(0,255,80,0.9)";
    ctx.fill();

    // Spine dots
    spine.forEach(([sx, sy]) => {
      ctx.beginPath();
      ctx.arc(px(sx), py(sy), 3, 0, 2 * Math.PI);
      ctx.fillStyle = "rgba(0,255,80,0.7)";
      ctx.fill();
    });
  }
}

function upd(d) {
  const s = S[d.state] || S.SEARCHING;
  bar.className    = "bar " + s.c;
  lbl.textContent  = s.t;
  bgo.disabled     = !s.go;
  bst.disabled     = !s.st;
  lockbadge.style.display = d.locked ? "inline-block" : "none";
  drawOverlay(d);
}

function go() { fetch("/api/start", {method:"POST"}) }
function st() { fetch("/api/stop",  {method:"POST"}) }

const telem = document.getElementById("telem");
setInterval(() => {
  fetch("/api/status").then(r => r.json()).then(d => {
    if (d.frame_w && d.frame_h) { CAM_W = d.frame_w; CAM_H = d.frame_h; }
    upd(d);
    telem.textContent =
      `gyro=${(d.gyro_z||0).toFixed(1)}\u00b0/s  ` +
      `RPM L=${(d.rpm_l||0).toFixed(0)} R=${(d.rpm_r||0).toFixed(0)}  ` +
      `steer=${(d.steer||0).toFixed(1)}\u00b0  ` +
      `pan=${(d.pan_angle||0).toFixed(1)}\u00b0  ` +
      `la=${(d.lookahead||0).toFixed(0)}px  ` +
      `trail=${(d.trail||[]).length}pts  spine=${(d.spine||[]).length}pts`;
  }).catch(() => {});
}, 200);
</script>
</body></html>"""


def create_app(follower):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return HTML_PAGE

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
                        help="Disable MPU gyro heading correction (vision + encoders only)")
    parser.add_argument("--no-encoders", dest="no_encoders", action="store_true",
                        help="Disable encoder speed-balance correction (vision + MPU only)")
    args = parser.parse_args()

    follower = LineFollower(
        tilt=args.tilt, speed=args.speed, motor_bias=args.bias,
        use_mpu=not args.no_mpu, use_encoders=not args.no_encoders,
    )

    def _shutdown(*_):
        follower.cleanup()
        log.info("Exiting")
        sys.exit(0)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    app = create_app(follower)
    log.info("Web UI → http://0.0.0.0:%d", args.port)
    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
