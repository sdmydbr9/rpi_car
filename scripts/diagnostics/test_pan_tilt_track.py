#!/usr/bin/env python3
"""
test_pan_tilt_track.py — Pan-Tilt camera tracker with web UI.

All-in-one: launches MediaMTX, Picamera2, click-to-track object tracking,
Flask web UI, pan-tilt servo control, and CSV telemetry logging.

Click anywhere in the browser to lock on; the camera gimbal follows the
object using a CSRT tracker.  No DNN — designed for 60 fps on a Pi.

Usage:
    python3 scripts/test_pan_tilt_track.py
    python3 scripts/test_pan_tilt_track.py --port /dev/ttyAMA0

Then open  http://<pi-ip>:5050  in a browser.

Pico firmware protocol (over UART):
    PT:<pan>,<tilt>\n   set angles (60-120)
    PC\n                center both
"""

import argparse
import csv
import io
import logging
import os
import sys
import time
import signal
import threading
import subprocess
import shutil
import socket as _socket
from datetime import datetime

import serial
import cv2
from flask import Flask, request, jsonify

# ---------------------------------------------------------------------------
# Paths / Logging
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, "rover_logs")
os.makedirs(LOG_DIR, exist_ok=True)

logger = logging.getLogger("pantilt")
logger.setLevel(logging.DEBUG)
_console = logging.StreamHandler()
_console.setLevel(logging.INFO)
_console.setFormatter(logging.Formatter("%(asctime)s  %(levelname)-5s  %(message)s", datefmt="%H:%M:%S"))
logger.addHandler(_console)

# ---------------------------------------------------------------------------
# Servo limits (must match Pico firmware)
# ---------------------------------------------------------------------------
PAN_CENTER  = 90;  TILT_CENTER = 90
PAN_MIN = 60;  PAN_MAX = 120
TILT_MIN = 60; TILT_MAX = 120

# ---------------------------------------------------------------------------
# MediaMTX
# ---------------------------------------------------------------------------
MEDIAMTX_RTSP_PORT  = 8554
MEDIAMTX_WEBRTC_PORT = 8889
MEDIAMTX_STREAM     = "pantilt"

# ---------------------------------------------------------------------------
# Capture — 640×480 @ 60fps
# ---------------------------------------------------------------------------
CAPTURE_W, CAPTURE_H = 640, 480
CAMERA_FPS = 60
CAMERA_HFLIP = True
CAMERA_VFLIP = True

# ---------------------------------------------------------------------------
# PID tracking tuning  (tuned for ~60fps CSRT + fast robocar motion)
# ---------------------------------------------------------------------------
DEADZONE_X = 0.05
DEADZONE_Y = 0.05
KP_PAN  = 10.0
KI_PAN  = 0.2
KP_TILT = 9.0
KI_TILT = 0.15
KD_PAN  = 1.8
KD_TILT = 1.5
MAX_STEP_PAN  = 3.5
MAX_STEP_TILT = 3.0
SMOOTH_ALPHA  = 0.25
OUTPUT_SMOOTH = 0.5     # EMA alpha for servo position approach
SERVO_UPDATE_HZ = 30
INTEGRAL_MAX = 2.0      # anti-windup clamp
INTEGRAL_DECAY = 0.85   # decay integral when error is in deadzone
SERVO_HYSTERESIS = 0.6  # must exceed this beyond current int to change

# Tracker settings
RE_INIT_INTERVAL = 120   # periodic re-init to prevent drift
TRACKER_LOST_MAX = 90    # tolerate brief occlusions
FFMPEG_SKIP = 2          # push every Nth frame to ffmpeg (1=all)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =====================================================================
# FLASK APP
# =====================================================================
app = Flask(__name__)

HTML_PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pan-Tilt Tracker</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#eee;font-family:system-ui,sans-serif;
     display:flex;flex-direction:column;align-items:center;height:100vh;overflow:hidden}
h1{font-size:1.1rem;padding:8px 0;letter-spacing:1px;color:#0ff}
#wrap{position:relative;width:100%;max-width:960px;aspect-ratio:4/3;background:#000;
      border:1px solid #333;overflow:hidden;cursor:crosshair}
#vid{width:100%;height:100%;object-fit:contain}
#hud{padding:6px 12px;font-size:.82rem;color:#aaa;width:100%;max-width:960px;
     display:flex;justify-content:space-between;flex-wrap:wrap;gap:4px}
.tag{background:#222;border:1px solid #444;border-radius:4px;padding:2px 8px}
.tag.active{border-color:#0f0;color:#0f0}
#box{position:absolute;border:2px solid #0f0;box-shadow:0 0 12px rgba(0,255,0,.35);
     pointer-events:none;display:none;transition:all 50ms linear}
#box .lbl{position:absolute;top:-18px;left:0;font-size:11px;background:rgba(0,0,0,.7);
          color:#0f0;padding:1px 5px;white-space:nowrap;border-radius:2px}
#crosshair{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
           width:30px;height:30px;pointer-events:none}
#crosshair::before,#crosshair::after{content:'';position:absolute;background:#0f04}
#crosshair::before{width:100%;height:1px;top:50%}
#crosshair::after{width:1px;height:100%;left:50%}
#btns{padding:8px;display:flex;gap:8px}
button{background:#222;color:#eee;border:1px solid #555;border-radius:4px;
       padding:6px 16px;cursor:pointer;font-size:.85rem}
button:hover{background:#333}
button.danger{border-color:#f44;color:#f44}
</style></head><body>
<h1>PAN-TILT TRACKER</h1>
<div id="wrap">
  <video id="vid" autoplay muted playsinline></video>
  <div id="box"><span class="lbl">tracking</span></div>
  <div id="crosshair"></div>
</div>
<div id="hud">
  <span class="tag" id="hud-mode">IDLE</span>
  <span class="tag" id="hud-target">&mdash;</span>
  <span class="tag" id="hud-pan">Pan 90&deg;</span>
  <span class="tag" id="hud-tilt">Tilt 90&deg;</span>
  <span class="tag" id="hud-fps">0 fps</span>
</div>
<div id="btns">
  <button onclick="doCenter()">CENTER</button>
  <button class="danger" onclick="doStop()">STOP TRACKING</button>
</div>
<script>
const WHEP = location.protocol+'//'+location.hostname+':"""+str(MEDIAMTX_WEBRTC_PORT)+"""/"""+MEDIAMTX_STREAM+"""/whep';
const vid = document.getElementById('vid');
let pc = null;

async function startWhep(){
  if(pc) try{pc.close()}catch(e){}
  pc = new RTCPeerConnection();
  pc.addTransceiver('video',{direction:'recvonly'});
  pc.addTransceiver('audio',{direction:'recvonly'});
  pc.ontrack = e => { if(e.track.kind==='video') vid.srcObject = e.streams[0]; };
  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  await new Promise(r=>{
    if(pc.iceGatheringState==='complete') return r();
    pc.onicegatheringstatechange=()=>{if(pc.iceGatheringState==='complete')r()};
    setTimeout(r, 2000);
  });
  try{
    const res = await fetch(WHEP,{method:'POST',
      headers:{'Content-Type':'application/sdp'},
      body:pc.localDescription.sdp});
    if(!res.ok) throw new Error(res.status);
    const answer = await res.text();
    await pc.setRemoteDescription({type:'answer',sdp:answer});
  }catch(e){
    console.warn('WHEP failed, retrying in 2s', e);
    setTimeout(startWhep, 2000);
  }
}
startWhep();

// --- Click-to-track ---
const wrap = document.getElementById('wrap');
const boxEl = document.getElementById('box');

wrap.addEventListener('click', async e => {
  const rect = wrap.getBoundingClientRect();
  const nx = (e.clientX - rect.left) / rect.width;
  const ny = (e.clientY - rect.top)  / rect.height;
  const resp = await fetch('/api/touch', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({x: nx, y: ny})});
  const data = await resp.json();
  console.log('touch', data);
});

function doCenter(){
  fetch('/api/center',{method:'POST'});
}
function doStop(){
  fetch('/api/stop',{method:'POST'});
  boxEl.style.display='none';
}

// --- Poll status ---
async function poll(){
  try{
    const r = await fetch('/api/status');
    const s = await r.json();
    document.getElementById('hud-mode').textContent = s.mode;
    document.getElementById('hud-mode').classList.toggle('active', s.mode==='TRACKING');
    document.getElementById('hud-target').textContent = s.target || '\\u2014';
    document.getElementById('hud-pan').textContent = 'Pan '+s.pan.toFixed(0)+'\\u00b0';
    document.getElementById('hud-tilt').textContent = 'Tilt '+s.tilt.toFixed(0)+'\\u00b0';
    document.getElementById('hud-fps').textContent = s.fps.toFixed(1)+' fps';
    // Single tracked-object box only
    const b = s.bbox;
    if(b && s.mode==='TRACKING'){
      boxEl.style.display='block';
      boxEl.style.left   = (b[0]*100)+'%';
      boxEl.style.top    = (b[1]*100)+'%';
      boxEl.style.width  = (b[2]*100)+'%';
      boxEl.style.height = (b[3]*100)+'%';
      boxEl.querySelector('.lbl').textContent = s.target||'tracking';
    } else {
      boxEl.style.display='none';
    }
  }catch(e){}
}
setInterval(poll, 100);
</script></body></html>"""


# =====================================================================
# CSV TELEMETRY LOGGER
# =====================================================================
CSV_COLUMNS = [
    "timestamp", "elapsed_s", "fps", "mode", "target",
    "track_cx", "track_cy",       # normalised 0-1 target centre
    "err_x", "err_y",             # smoothed error fed to PID
    "int_x", "int_y",             # integral accumulators
    "pan_deg", "tilt_deg",        # servo angles
    "dp", "dt",                   # PID output this frame
    "bbox_x", "bbox_y", "bbox_w", "bbox_h",  # tracker bbox (px)
    "tracker_lost",               # consecutive lost frames
]


class TelemetryLogger:
    """Writes one CSV row per vision-loop iteration."""

    def __init__(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(LOG_DIR, f"pantilt_track_{stamp}.csv")
        self._file = open(self._path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._file.flush()
        self._t0 = time.monotonic()
        logger.info("Logging to %s", self._path)

    def log(self, row: dict):
        row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = f"{time.monotonic() - self._t0:.3f}"
        self._writer.writerow(row)

    def flush(self):
        self._file.flush()

    def close(self):
        self._file.close()


# =====================================================================
# TRACKER ENGINE  (CSRT tracker — no DNN, tap-to-track only)
# =====================================================================
class PanTiltTracker:

    def __init__(self, serial_port="/dev/ttyS0"):
        # State
        self.pan  = float(PAN_CENTER)
        self.tilt = float(TILT_CENTER)
        self._err_x = 0.0
        self._err_y = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._int_x = 0.0          # integral accumulators
        self._int_y = 0.0
        self._last_servo_t = 0.0
        self._last_dp = 0.0        # for logging
        self._last_dt = 0.0
        self._target_pan = float(PAN_CENTER)   # PID target (raw)
        self._target_tilt = float(TILT_CENTER)
        self._last_sent_pan = PAN_CENTER   # dedup servo commands
        self._last_sent_tilt = TILT_CENTER

        # Tracking
        self.mode = "IDLE"          # IDLE | TRACKING
        self.locked_name = ""
        self.track_cx = 0.5
        self.track_cy = 0.5
        self.fps = 0.0

        # OpenCV CSRT tracker (accurate + fast enough at 640×480)
        self._tracker = None
        self._tracker_bbox = None   # (x, y, w, h) in pixels
        self._tracker_lost = 0
        self._frame_count = 0       # for periodic re-init
        self._init_roi = None       # original ROI image for re-init

        # Telemetry
        self._telem = TelemetryLogger()

        # Serial
        self.ser = None
        for port in [serial_port, "/dev/ttyAMA0", "/dev/serial0"]:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.05)
                logger.info("Serial: %s", port)
                break
            except Exception:
                continue
        if self.ser is None:
            logger.warning("No serial port to Pico — servos will not move")

        # Camera
        os.environ["LIBCAMERA_LOG_LEVELS"] = "*:ERROR"
        from picamera2 import Picamera2
        from libcamera import Transform
        self.picam2 = Picamera2()
        transform = Transform(hflip=CAMERA_HFLIP, vflip=CAMERA_VFLIP)
        cfg = self.picam2.create_video_configuration(
            main={"size": (CAPTURE_W, CAPTURE_H), "format": "BGR888"},
            transform=transform,
            buffer_count=6,
            controls={"FrameDurationLimits": (16666, 16666)},  # 60fps
        )
        self.picam2.configure(cfg)
        self.picam2.start()
        logger.info("Camera %dx%d @ %dfps (hflip=%s, vflip=%s)",
                    CAPTURE_W, CAPTURE_H, CAMERA_FPS, CAMERA_HFLIP, CAMERA_VFLIP)

        # ffmpeg → MediaMTX RTSP ingest
        self._ffmpeg = None
        self._start_ffmpeg()

        # Center servos
        self._send_center()

        # Lock
        self._lock = threading.Lock()
        self._running = True

    # ---- ffmpeg publisher ----
    def _start_ffmpeg(self):
        url = f"rtsp://127.0.0.1:{MEDIAMTX_RTSP_PORT}/{MEDIAMTX_STREAM}"
        # ffmpeg input rate = camera fps / FFMPEG_SKIP
        ffmpeg_fps = CAMERA_FPS // FFMPEG_SKIP
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{CAPTURE_W}x{CAPTURE_H}", "-r", str(ffmpeg_fps), "-i", "-",
            "-c:v", "h264_v4l2m2m", "-b:v", "3M",
            "-g", str(ffmpeg_fps),
            "-f", "rtsp", "-rtsp_transport", "tcp", url,
        ]
        try:
            self._ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.DEVNULL)
            logger.info("ffmpeg → %s (%dx%d @ %dfps)", url, CAPTURE_W, CAPTURE_H, ffmpeg_fps)
        except Exception as e:
            logger.error("ffmpeg start failed: %s", e)

    # ---- Serial ----
    def _send_pt(self):
        if not self.ser:
            return
        p = int(clamp(self._last_sent_pan,  PAN_MIN, PAN_MAX))
        t = int(clamp(self._last_sent_tilt, TILT_MIN, TILT_MAX))
        try:
            self.ser.write(f"PT:{p},{t}\n".encode())
        except Exception:
            pass

    def _send_center(self):
        self.pan  = float(PAN_CENTER)
        self.tilt = float(TILT_CENTER)
        self._err_x = 0.0
        self._err_y = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._int_x = 0.0
        self._int_y = 0.0
        self._target_pan = float(PAN_CENTER)
        self._target_tilt = float(TILT_CENTER)
        self._last_sent_pan = PAN_CENTER
        self._last_sent_tilt = TILT_CENTER
        if self.ser:
            try:
                self.ser.write(b"PC\n")
            except Exception:
                pass

    # ---- Object tracker (CSRT — accurate correlation tracker) ----
    def _init_tracker(self, frame, bbox):
        """Initialise CSRT tracker. bbox = (x, y, w, h) in pixels."""
        self._tracker = cv2.TrackerCSRT.create()
        self._tracker.init(frame, bbox)
        self._tracker_bbox = tuple(int(v) for v in bbox)
        self._tracker_lost = 0
        self._frame_count = 0
        self._int_x = 0.0
        self._int_y = 0.0
        logger.info("CSRT tracker init bbox=(%d,%d,%d,%d)", *self._tracker_bbox)

    def _reinit_tracker(self, frame):
        """Re-initialise tracker on current bbox to prevent drift."""
        if self._tracker_bbox is None:
            return
        bbox = self._tracker_bbox
        self._tracker = cv2.TrackerCSRT.create()
        self._tracker.init(frame, bbox)
        self._tracker_lost = 0
        logger.debug("Tracker re-init at (%d,%d,%d,%d)", *bbox)

    def _update_tracker(self, frame):
        """Update tracker. Returns (cx_norm, cy_norm) or None."""
        if self._tracker is None:
            return None
        fh, fw = frame.shape[:2]
        ok, bbox = self._tracker.update(frame)
        if ok:
            x, y, bw, bh = (int(v) for v in bbox)
            if x < 0 or y < 0 or x + bw > fw or y + bh > fh or bw < 5 or bh < 5:
                ok = False
            else:
                self._tracker_bbox = (x, y, bw, bh)
                self._tracker_lost = 0
                cx = (x + bw / 2) / fw
                cy = (y + bh / 2) / fh
                return (cx, cy)
        if not ok:
            self._tracker_lost += 1
            if self._tracker_lost > TRACKER_LOST_MAX:
                logger.info("Tracker lost target after %d frames", TRACKER_LOST_MAX)
                self.mode = "IDLE"
                self.locked_name = ""
                self._tracker = None
                self._tracker_bbox = None
            return None

    # ---- Servo update (PID controller with rate limiting) ----
    def _update_servos(self, cx_norm, cy_norm):
        now = time.monotonic()
        dt_s = now - self._last_servo_t
        if dt_s < (1.0 / SERVO_UPDATE_HZ):
            return

        err_x = (cx_norm - 0.5) * 2.0
        err_y = (cy_norm - 0.5) * 2.0

        in_deadzone_x = abs(err_x) < DEADZONE_X
        in_deadzone_y = abs(err_y) < DEADZONE_Y

        if in_deadzone_x:
            err_x = 0.0
            # Decay integral when error is small (prevent windup drift)
            self._int_x *= INTEGRAL_DECAY
        if in_deadzone_y:
            err_y = 0.0
            self._int_y *= INTEGRAL_DECAY

        # If both axes are in deadzone, skip servo command entirely
        if in_deadzone_x and in_deadzone_y:
            self._err_x *= 0.5
            self._err_y *= 0.5
            self._prev_err_x = self._err_x
            self._prev_err_y = self._err_y
            self._last_dp = 0.0
            self._last_dt = 0.0
            self._last_servo_t = now
            return

        # EMA smoothing
        self._err_x = SMOOTH_ALPHA * err_x + (1.0 - SMOOTH_ALPHA) * self._err_x
        self._err_y = SMOOTH_ALPHA * err_y + (1.0 - SMOOTH_ALPHA) * self._err_y

        # Integral (with anti-windup)
        self._int_x = clamp(self._int_x + self._err_x * dt_s, -INTEGRAL_MAX, INTEGRAL_MAX)
        self._int_y = clamp(self._int_y + self._err_y * dt_s, -INTEGRAL_MAX, INTEGRAL_MAX)

        # Derivative
        d_err_x = self._err_x - self._prev_err_x
        d_err_y = self._err_y - self._prev_err_y
        self._prev_err_x = self._err_x
        self._prev_err_y = self._err_y

        # PID control
        dp = -(self._err_x * KP_PAN  + self._int_x * KI_PAN  + d_err_x * KD_PAN)
        dt_ctrl = (self._err_y * KP_TILT + self._int_y * KI_TILT + d_err_y * KD_TILT)

        dp = clamp(dp, -MAX_STEP_PAN, MAX_STEP_PAN)
        dt_ctrl = clamp(dt_ctrl, -MAX_STEP_TILT, MAX_STEP_TILT)

        # Two-stage smoothing: PID drives target, servo smoothly approaches
        self._target_pan  = clamp(self._target_pan  + dp, PAN_MIN, PAN_MAX)
        self._target_tilt = clamp(self._target_tilt + dt_ctrl, TILT_MIN, TILT_MAX)
        self.pan  += OUTPUT_SMOOTH * (self._target_pan  - self.pan)
        self.tilt += OUTPUT_SMOOTH * (self._target_tilt - self.tilt)
        self.pan  = clamp(self.pan,  PAN_MIN, PAN_MAX)
        self.tilt = clamp(self.tilt, TILT_MIN, TILT_MAX)
        self._last_dp = dp
        self._last_dt = dt_ctrl

        # Hysteresis: only change integer position when float exceeds
        # current int by SERVO_HYSTERESIS (prevents 1° quantization jitter)
        p_int = self._last_sent_pan
        t_int = self._last_sent_tilt
        if abs(self.pan - p_int) > SERVO_HYSTERESIS:
            p_int = int(round(self.pan))
        if abs(self.tilt - t_int) > SERVO_HYSTERESIS:
            t_int = int(round(self.tilt))
        if p_int != self._last_sent_pan or t_int != self._last_sent_tilt:
            self._last_sent_pan = p_int
            self._last_sent_tilt = t_int
            self._send_pt()

        self._last_servo_t = now

    # ---- Main vision loop (runs in thread) ----
    def vision_loop(self):
        fps_t = time.monotonic()
        fps_n = 0
        log_counter = 0
        ffmpeg_counter = 0

        while self._running:
            try:
                frame = self.picam2.capture_array()
            except Exception:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            track_target = None

            with self._lock:
                # Update tracker if tracking
                if self.mode == "TRACKING" and self._tracker is not None:
                    self._frame_count += 1
                    pos = self._update_tracker(frame)
                    if pos:
                        track_target = pos
                        # Periodic re-init to prevent drift
                        if self._frame_count % RE_INIT_INTERVAL == 0:
                            self._reinit_tracker(frame)

                if track_target:
                    self.track_cx, self.track_cy = track_target
                    self._update_servos(track_target[0], track_target[1])
                elif self.mode != "IDLE":
                    self._err_x *= 0.85
                    self._err_y *= 0.85

            # ---- Annotate + push to ffmpeg (skip frames to save CPU) ----
            ffmpeg_counter += 1
            if self._ffmpeg and self._ffmpeg.stdin and ffmpeg_counter >= FFMPEG_SKIP:
                ffmpeg_counter = 0
                if self.mode == "TRACKING" and self._tracker_bbox is not None:
                    bx, by, bw, bh = self._tracker_bbox
                    cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
                    label = self.locked_name or "tracking"
                    cv2.putText(frame, label, (bx, by - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                cv2.line(frame, (w//2-15, h//2), (w//2+15, h//2), (0,255,0), 1)
                cv2.line(frame, (w//2, h//2-15), (w//2, h//2+15), (0,255,0), 1)
                cv2.putText(frame, f"Pan:{self.pan:.0f} Tilt:{self.tilt:.0f}  {self.fps:.0f}fps",
                            (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)
                if self.mode != "IDLE":
                    cv2.putText(frame, f"[{self.mode}] {self.locked_name}",
                                (4, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
                try:
                    self._ffmpeg.stdin.write(frame.tobytes())
                except Exception:
                    pass

            # ---- Telemetry logging (every 3rd frame to reduce I/O) ----
            log_counter += 1
            if log_counter % 3 == 0:
                bbox = self._tracker_bbox or (0, 0, 0, 0)
                self._telem.log({
                    "fps": f"{self.fps:.1f}",
                    "mode": self.mode,
                    "target": self.locked_name,
                    "track_cx": f"{self.track_cx:.4f}",
                    "track_cy": f"{self.track_cy:.4f}",
                    "err_x": f"{self._err_x:.4f}",
                    "err_y": f"{self._err_y:.4f}",
                    "int_x": f"{self._int_x:.4f}",
                    "int_y": f"{self._int_y:.4f}",
                    "pan_deg": f"{self.pan:.2f}",
                    "tilt_deg": f"{self.tilt:.2f}",
                    "dp": f"{self._last_dp:.3f}",
                    "dt": f"{self._last_dt:.3f}",
                    "bbox_x": bbox[0], "bbox_y": bbox[1],
                    "bbox_w": bbox[2], "bbox_h": bbox[3],
                    "tracker_lost": self._tracker_lost,
                })
                if log_counter % 180 == 0:
                    self._telem.flush()

            fps_n += 1
            now = time.monotonic()
            if now - fps_t >= 1.0:
                self.fps = fps_n / (now - fps_t)
                fps_n = 0
                fps_t = now

        # Cleanup
        self._send_center()
        self._telem.close()
        time.sleep(0.2)
        self.picam2.stop()
        if self._ffmpeg:
            try:
                self._ffmpeg.stdin.close()
                self._ffmpeg.terminate()
            except Exception:
                pass

    # ---- Public API (called from Flask routes) ----
    def handle_touch(self, nx, ny):
        """Tap anywhere to track — initialise CSRT on that region."""
        with self._lock:
            try:
                frame = self.picam2.capture_array()
                h, w = frame.shape[:2]
            except Exception:
                return {"ok": False}

            # Build an ROI around the tap point
            cx = int(nx * w)
            cy = int(ny * h)
            sz = 100  # initial box size
            bx = clamp(cx - sz // 2, 0, w - sz)
            by = clamp(cy - sz // 2, 0, h - sz)
            bw = min(sz, w - bx)
            bh = min(sz, h - by)
            if bw < 20 or bh < 20:
                return {"ok": False}

            self._init_tracker(frame, (bx, by, bw, bh))
            self.locked_name = "target"
            self.mode = "TRACKING"
            logger.info("Touch track at (%.2f, %.2f) → bbox (%d,%d,%d,%d)",
                        nx, ny, bx, by, bw, bh)
            return {"ok": True, "locked": "target"}

    def handle_stop(self):
        with self._lock:
            logger.info("Stop tracking")
            self.mode = "IDLE"
            self.locked_name = ""
            self._tracker = None
            self._tracker_bbox = None
            self._send_center()

    def handle_center(self):
        with self._lock:
            logger.info("Center servos")
            self._send_center()

    def get_status(self):
        with self._lock:
            bbox_norm = None
            if self._tracker_bbox and self.mode == "TRACKING":
                bx, by, bw, bh = self._tracker_bbox
                bbox_norm = [bx / CAPTURE_W, by / CAPTURE_H,
                             bw / CAPTURE_W, bh / CAPTURE_H]
            return {
                "mode": self.mode,
                "target": self.locked_name,
                "pan": self.pan,
                "tilt": self.tilt,
                "fps": self.fps,
                "bbox": bbox_norm,
                "track_pos": [self.track_cx, self.track_cy] if self.mode != "IDLE" else None,
            }

    def shutdown(self):
        self._running = False


# =====================================================================
# MEDIAMTX MANAGEMENT
# =====================================================================
def _find_mediamtx():
    for p in [os.path.join(PROJECT_ROOT, "bin", "mediamtx"),
              "/usr/local/bin/mediamtx", "/usr/bin/mediamtx"]:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    found = shutil.which("mediamtx")
    return found or ""

def _port_open(port):
    try:
        with _socket.create_connection(("127.0.0.1", port), timeout=0.3):
            return True
    except Exception:
        return False

def start_mediamtx():
    """Start MediaMTX with a publisher-only path for our stream."""
    if _port_open(MEDIAMTX_RTSP_PORT) and _port_open(MEDIAMTX_WEBRTC_PORT):
        print("📡 MediaMTX already running")
        return None  # external instance

    mtx_bin = _find_mediamtx()
    if not mtx_bin:
        print("❌ mediamtx binary not found")
        sys.exit(1)

    cfg_text = (
        "logLevel: warn\n"
        f"rtspAddress: :{MEDIAMTX_RTSP_PORT}\n"
        f"webrtcAddress: :{MEDIAMTX_WEBRTC_PORT}\n"
        "hls: no\nrtmp: no\nsrt: no\n"
        "paths:\n"
        f"  {MEDIAMTX_STREAM}:\n"
        "    source: publisher\n"
    )
    cfg_path = "/tmp/pantilt_mediamtx.yml"
    with open(cfg_path, "w") as f:
        f.write(cfg_text)

    proc = subprocess.Popen([mtx_bin, cfg_path],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    # Wait for readiness
    for _ in range(40):
        if _port_open(MEDIAMTX_RTSP_PORT):
            print(f"📡 MediaMTX started (RTSP :{MEDIAMTX_RTSP_PORT}, WebRTC :{MEDIAMTX_WEBRTC_PORT})")
            return proc
        time.sleep(0.15)
    print("⚠ MediaMTX did not become ready in time")
    return proc


# =====================================================================
# WIRE UP FLASK ROUTES
# =====================================================================
tracker: PanTiltTracker = None  # set in main()

@app.route("/")
def index():
    return HTML_PAGE

@app.route("/api/status")
def api_status():
    return jsonify(tracker.get_status())

@app.route("/api/touch", methods=["POST"])
def api_touch():
    data = request.get_json(force=True)
    result = tracker.handle_touch(float(data.get("x", 0.5)), float(data.get("y", 0.5)))
    return jsonify(result)

@app.route("/api/stop", methods=["POST"])
def api_stop():
    tracker.handle_stop()
    return jsonify({"ok": True})

@app.route("/api/center", methods=["POST"])
def api_center():
    tracker.handle_center()
    return jsonify({"ok": True})


# =====================================================================
# MAIN
# =====================================================================
def main():
    global tracker

    parser = argparse.ArgumentParser(description="Pan-Tilt Tracker with Web UI")
    parser.add_argument("--port", default="/dev/ttyS0", help="Serial port to Pico")
    parser.add_argument("--web-port", type=int, default=5050, help="Flask web UI port")
    args = parser.parse_args()

    # 1. Start MediaMTX
    mtx_proc = start_mediamtx()

    # 2. Create tracker (camera + serial)
    tracker = PanTiltTracker(serial_port=args.port)

    # 3. Vision loop in background thread
    vision_thread = threading.Thread(target=tracker.vision_loop, daemon=True)
    vision_thread.start()

    logger.info("Web UI → http://0.0.0.0:%d", args.web_port)
    logger.info("WebRTC → http://<pi-ip>:%d/%s/", MEDIAMTX_WEBRTC_PORT, MEDIAMTX_STREAM)
    logger.info("Click anywhere in the video to start tracking!")

    def _shutdown(sig=None, frame=None):
        logger.info("Shutting down…")
        tracker.shutdown()
        if mtx_proc:
            mtx_proc.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # 4. Start Flask (blocking)
    app.run(host="0.0.0.0", port=args.web_port, threaded=True)


if __name__ == "__main__":
    main()
