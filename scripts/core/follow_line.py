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
import sys, os, time, threading, signal, argparse, io, logging
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
from pico_sensor_reader import init_pico_reader, send_pan_tilt, get_gyro_z, get_rpm, is_pico_fresh
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
DEFAULT_SPEED  = 50           # base PWM %  (lower = more time to correct)
STEERING_KP    = 0.25         # reduced from 0.45 — was causing violent oscillation
STEERING_KI    = 0.005        # integral: accumulates to fix persistent drift
STEERING_KD    = 0.12         # derivative: dampens oscillation
MAX_STEER      = 45           # clamp
INTEGRAL_MAX   = 15.0         # tightened from 25.0 — limits windup on line-loss
# Tank-turn (unicycle) mixer: inner wheel transitions slow → stop → reverse
# as |steer| grows.  No STEER_DIFFERENTIAL constant is needed — the
# unicycle model naturally keeps the outer wheel at ~speed while the
# inner side goes negative (all 4 wheels engaged in a genuine pivot)
# once |steer_frac| > 0.5.  See _loop for the mixer implementation.
LINE_LOST_TIMEOUT = 0.5       # seconds with no line → stop

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
GYRO_KP       = 0.30    # per-side PWM % per °/s  (e.g. 30°/s → ±9% each)
GYRO_MAX_CORR = 10.0    # per-side PWM clamp (%)
GYRO_DEADZONE = 3.5     # °/s — still noise floor is ±2.47 °/s
GYRO_FADE_PX  = 80      # pixel error at which gyro contribution → 0 (soft fade)

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

# Line lock — spatial + colour gate applied once a line is confirmed.
# Prevents the rover jumping to a different nearby line mid-follow.
LOCK_SPATIAL_MARGIN = 120   # px: contour centroid must be within this of the locked cx
LOCK_COLOUR_TOL     = 40    # 0-255: mean BGR of locked contour must stay within this

# Path trail drawn on annotated frame (rolling centroid history)
PATH_HISTORY_LEN = 60       # how many centroid samples to keep (~1.5 s at 40 fps)

# Spine: number of horizontal scan rows used to trace the line ahead of the rover
SPINE_ROWS = 12

# Keep the camera servo fixed for the entire follow-line run.
FIXED_PAN = PAN_CENTER

# Curve lookahead — blends near centroid error with far spine error so the rover
# begins turning before the centroid itself drifts, giving smooth curve tracking.
LOOKAHEAD_WEIGHT = 0.50   # 0 = pure centroid, 1 = pure far-spine lookahead
LOOKAHEAD_ROWS   = 4      # how many far spine points to average for lookahead

# Curve-adaptive speed: reduce drive speed proportionally when cornering hard.
# At |steer|=MAX_STEER the speed is multiplied by CURVE_SPEED_MIN_FRAC.
# e.g. base_speed=20, MIN_FRAC=0.50  →  speed on tightest curve = 10 %
CURVE_SPEED_MIN_FRAC = 0.50

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

        # --- path history (rolling centroid trail) ---
        self._path_history: deque = deque(maxlen=PATH_HISTORY_LEN)
        # Spine = list of (x, y) tuples tracing the line ahead, shared for the UI
        self._spine: list = []

        # --- pan servo state ---
        self._pan_angle       = float(FIXED_PAN)   # fixed pan angle (degrees)
        self._lookahead_error = 0.0                # last lookahead error (for CSV)

        # --- sensor state (gyro + encoders) ---
        self._gyro_z = 0.0          # latest yaw rate (°/s)
        self._rpm_left  = 0.0       # latest left-side RPM
        self._rpm_right = 0.0       # latest right-side RPM
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
            "time,state,found,cx,error,lookahead_err,vis_steer,gyro_corr,enc_corr,"
            "motor_bias,total_steer,gyro_z,rpm_l,rpm_r,speed,pan_angle,"
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
                log.info("START following (speed=%d%%)", self._base_speed)

    def stop_following(self):
        with self._lock:
            self._following = False
            self.state = self.STOPPED
            self._locked_cx     = None   # release line lock
            self._locked_colour = None
            self._path_history.clear()
            self._spine = []
            self._pan_angle = float(FIXED_PAN)
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
        """Detect the line in the bottom portion of *frame*.

        When a line is locked (self._locked_cx is set) only contours whose
        centroid lies within LOCK_SPATIAL_MARGIN pixels of the locked cx AND
        whose mean BGR colour is within LOCK_COLOUR_TOL of the locked colour
        are accepted.  This prevents the rover jumping to a different nearby
        line mid-follow.

        Returns (found, cx, cy_full, mask, contour, mean_colour)
            found       – bool
            cx          – centroid-x in full-frame coords
            cy_full     – centroid-y in full-frame coords
            mask        – binary mask (ROI-relative)
            contour     – best valid contour (ROI-relative)
            mean_colour – np.ndarray shape(3,) mean BGR of the contour region
        """
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        roi = frame[roi_top:, :]
        rh, rw = roi.shape[:2]

        # Gaussian kernel ≈ 1/6 of ROI width, forced odd
        ksize = max(31, (rw // 6) | 1)

        # Per-channel local contrast
        channels = cv2.split(roi)
        diffs = []
        for ch in channels:
            bg = cv2.GaussianBlur(ch, (ksize, ksize), 0)
            diffs.append(cv2.absdiff(ch, bg))
        diff_max = np.maximum(np.maximum(diffs[0], diffs[1]), diffs[2])

        # Binary mask
        _, mask = cv2.threshold(diff_max, CONTRAST_THRESH, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)

        # Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]
        if not valid:
            return False, 0, 0, mask, None, None

        # ── Line-lock filtering ──
        # If we have an established lock, reject contours that are spatially
        # or chromatically too far from the locked line.
        locked_cx  = self._locked_cx
        locked_col = self._locked_colour
        if locked_cx is not None and locked_col is not None:
            def _contour_colour(c):
                c_mask = np.zeros((rh, rw), dtype=np.uint8)
                cv2.drawContours(c_mask, [c], -1, 255, -1)
                return cv2.mean(roi, mask=c_mask)[:3]   # BGR

            def _passes_lock(c):
                M_ = cv2.moments(c)
                if M_["m00"] == 0:
                    return False
                cx_ = int(M_["m10"] / M_["m00"])
                if abs(cx_ - locked_cx) > LOCK_SPATIAL_MARGIN:
                    return False
                col = np.array(_contour_colour(c))
                if np.max(np.abs(col - locked_col)) > LOCK_COLOUR_TOL:
                    return False
                return True

            locked_valid = [c for c in valid if _passes_lock(c)]
            if locked_valid:
                valid = locked_valid
            # If nothing passes the lock we fall through to largest contour
            # (graceful degradation — line may have temporarily smeared)

        best = max(valid, key=cv2.contourArea)
        M = cv2.moments(best)
        if M["m00"] == 0:
            return False, 0, 0, mask, best, None

        cx = int(M["m10"] / M["m00"])
        cy_full = int(M["m01"] / M["m00"]) + roi_top

        # Mean colour of the winning contour
        c_mask = np.zeros((rh, rw), dtype=np.uint8)
        cv2.drawContours(c_mask, [best], -1, 255, -1)
        mean_bgr = np.array(cv2.mean(roi, mask=c_mask)[:3])

        return True, cx, cy_full, mask, best, mean_bgr

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
        cv2.putText(out, f"pan={self._pan_angle:.0f}\u00b0  la={self._lookahead_error:+.0f}px",
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1)
        cv2.putText(out, f"gyro={self._gyro_z:+.1f} d/s", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(out, f"RPM L={self._rpm_left:.0f} R={self._rpm_right:.0f}",
                    (200, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(out, f"steer={self._last_steer:+.1f}",
                    (460, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return out

    # ─────────────────────────────────────────────────
    #  Main processing loop (runs on worker thread)
    # ─────────────────────────────────────────────────
    def _read_sensors(self):
        """Read MPU gyro, LM393 encoder RPMs, and per-wheel applied PWM."""
        self._gyro_z = get_gyro_z()  # °/s yaw rate
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

        # Read per-wheel applied PWM from wheel_sync (available after first tick)
        if self._car.wheel_sync is not None:
            telem  = self._car.get_sync_telemetry()
            wheels = telem.get("wheels", {})
            self._pwm_fl = wheels.get("fl", {}).get("applied_pwm", 0.0)
            self._pwm_fr = wheels.get("fr", {}).get("applied_pwm", 0.0)
            self._pwm_rl = wheels.get("rl", {}).get("applied_pwm", 0.0)
            self._pwm_rr = wheels.get("rr", {}).get("applied_pwm", 0.0)

    def _compute_steering(self, vision_error, lookahead_error):
        """Compute final motor-steering angle (degrees).

        Two vision inputs are blended:
          1. vision_error    — centroid offset from frame centre (near, immediate)
          2. lookahead_error — mean offset of far spine points   (far, anticipatory)

        The blended near+far error feeds the PID.
        """
        # ── Blend near-centroid and far-spine errors ──
        blended = ((1.0 - LOOKAHEAD_WEIGHT) * vision_error
                   + LOOKAHEAD_WEIGHT * lookahead_error)
        self._lookahead_error = lookahead_error

        # ── Vision PID on blended error (with clamping anti-windup) ──
        p = STEERING_KP * blended
        d = STEERING_KD * (blended - self._prev_error)
        self._prev_error = blended

        # Clamping anti-windup: only accumulate the integral when the PID output
        # is not already saturated, OR when the new error would help unwind it.
        # This prevents integral windup on sharp curves where steer is clamped for
        # many frames and then causes a large overshoot when the centroid crosses zero.
        raw_output = p + self._integral + d + self._motor_bias
        if abs(raw_output) < MAX_STEER or (raw_output * blended < 0):
            self._integral += STEERING_KI * blended
            self._integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, self._integral))
        i = self._integral

        vision_steer = p + i + d + self._motor_bias
        self._vision_steer = vision_steer

        steer = max(-MAX_STEER, min(MAX_STEER, vision_steer))
        self._last_steer = steer
        return steer

    def _compute_straight_corrections(self, vision_error, base_l, base_r):
        """Apply gyro rate-damping bias to per-side PWM.

        Gyro rate-damping: opposes instantaneous yaw rate directly, fades to
        zero as |last_steer| grows so intentional curve turns are not fought.

        Per-wheel speed balancing (including RR plastic-gearbox compensation)
        is now handled entirely by wheel_sync inside _set_raw_motors, which
        runs a closed-loop PID for each individual wheel using live RPM data.
        """
        duty_cap = self._car.power_limiter.max_safe_duty
        left_speed  = float(base_l)
        right_speed = float(base_r)

        # ── Gyro rate-damping ──
        gyro_corr = 0.0
        if self._use_mpu:
            gz = self._gyro_z
            if abs(gz) > GYRO_DEADZONE:
                fade = max(0.0, 1.0 - abs(self._last_steer) / GYRO_FADE_PX)
                raw  = GYRO_KP * gz * fade
                gyro_corr = max(-GYRO_MAX_CORR, min(GYRO_MAX_CORR, raw))
                half = gyro_corr / 2.0
                left_speed  = max(0.0, min(duty_cap, left_speed  - half))
                right_speed = max(0.0, min(duty_cap, right_speed + half))
        self._gyro_correction = round(gyro_corr, 2)

        log.debug("straight: gz=%+.1f gyro=%.2f → L=%.1f R=%.1f  RR_pwm=%.1f",
                  self._gyro_z, gyro_corr, left_speed, right_speed, self._pwm_rr)

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
            found, cx, cy, mask, contour, mean_colour = self._detect_line(frame)

            # 3. Update state machine + line-lock
            with self._lock:
                if found:
                    self._line_cx = cx
                    self._line_cy = cy
                    self._confirm_count = min(self._confirm_count + 1,
                                              LINE_CONFIRM_FRAMES + 5)
                    self._line_lost_since = 0.0

                    if self._confirm_count >= LINE_CONFIRM_FRAMES:
                        # Establish lock on the first confirmed detection
                        if self._locked_cx is None:
                            self._locked_cx     = cx
                            self._locked_colour = mean_colour
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

                        if self._following:
                            self.state = self.FOLLOWING
                        elif self.state not in (self.FOLLOWING, self.STOPPED):
                            self.state = self.LINE_DETECTED

                    # Update path history with each confirmed centroid
                    if self._confirm_count >= LINE_CONFIRM_FRAMES:
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
                            self._locked_cx     = None  # release lock — will re-acquire
                            self._locked_colour = None
                            self._car.brake()
                            log.warning("Line lost for %.1fs — braking", LINE_LOST_TIMEOUT)
                    else:
                        if self.state not in (self.STOPPED, self.LINE_LOST):
                            self.state = self.SEARCHING

                cur_state = self.state

            # Extract spine (projected path ahead) for annotation + UI
            roi_top = int(h * ROI_TOP_FRAC)
            spine = self._extract_spine(mask, roi_top) if found else []
            with self._lock:
                self._spine = spine

            # Log state transitions
            if cur_state != _prev_state:
                log.info("State: %s → %s", _prev_state or "(init)", cur_state)
                _prev_state = cur_state

            # 4. Motor control (when following and line visible)
            error = cx - (w // 2) if found else 0
            if self._following and found:
                # Lookahead error from far spine gives early curve warning
                lookahead_error = self._compute_lookahead_error(spine, w)
                steer = self._compute_steering(error, lookahead_error)
                duty_cap = self._car.power_limiter.max_safe_duty
                # Curve-adaptive speed: slow down proportionally when cornering hard.
                # Reduces overshoot momentum and gives the rover more time to turn.
                curve_factor = max(CURVE_SPEED_MIN_FRAC,
                                   1.0 - (abs(steer) / MAX_STEER) * (1.0 - CURVE_SPEED_MIN_FRAC))
                speed = max(0, min(duty_cap, int(self._base_speed * curve_factor)))

                # Tank-turn unicycle mixer.
                # steer_frac: -1.0 (hard left) … +1.0 (hard right)
                # Outer wheel always gets ~speed; inner wheel goes:
                #   |steer_frac| < 0.5 → slowing down
                #   |steer_frac| = 0.5 → stopped
                #   |steer_frac| > 0.5 → reverse  (true 4-wheel tank pivot)
                steer_frac  = steer / MAX_STEER
                fwd_comp    = speed * (1.0 - abs(steer_frac))
                rot_comp    = speed * steer_frac
                l_raw       = fwd_comp + rot_comp   # steer>0: outer (left stays ≈speed)
                r_raw       = fwd_comp - rot_comp   # steer>0: inner (right → 0 → -)
                l_fwd       = l_raw >= 0.0
                r_fwd       = r_raw >= 0.0
                base_l      = min(duty_cap, abs(l_raw))
                base_r      = min(duty_cap, abs(r_raw))

                # Straight-line corrections (gyro + encoder) only apply when
                # both sides drive forward — skip during pivot turns.
                if l_fwd and r_fwd:
                    left_speed, right_speed = self._compute_straight_corrections(
                        error, base_l, base_r)
                else:
                    left_speed, right_speed = base_l, base_r
                self._car._set_raw_motors(left_speed, right_speed, l_fwd, r_fwd)
            elif self._following and not found:
                # Coast with last known steering while within timeout
                pass

            # 4b. CSV + debug logging (every iteration while following)
            _log_counter += 1
            if self._following:
                t = time.monotonic() - self._t0
                self._csv_file.write(
                    f"{t:.3f},{cur_state},{int(found)},{cx},{error},"
                    f"{self._lookahead_error:.1f},"
                    f"{self._vision_steer:.2f},{self._gyro_correction:.2f},"
                    f"{self._encoder_correction:.2f},{self._motor_bias:.2f},"
                    f"{self._last_steer:.2f},"
                    f"{self._gyro_z:.2f},{self._rpm_left:.1f},"
                    f"{self._rpm_right:.1f},{self._base_speed},"
                    f"{self._pan_angle:.1f},"
                    f"{self._pwm_fl:.1f},{self._pwm_fr:.1f},"
                    f"{self._pwm_rl:.1f},{self._pwm_rr:.1f}\n")
                if _log_counter % 8 == 0:   # ~5 Hz
                    log.debug(
                        "err=%+dpx la=%+.0f steer=%+.1f° (vis=%.1f I=%.1f gyro=%+.1f) "
                        "gz=%+.1f°/s rpmL=%.0f rpmR=%.0f "
                        "PWM FL=%.1f FR=%.1f RL=%.1f RR=%.1f pan=%.0f°",
                        error, self._lookahead_error, self._last_steer,
                        self._vision_steer, self._integral, self._gyro_correction,
                        self._gyro_z, self._rpm_left, self._rpm_right,
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
