"""
vision.py — Object Detection System using MobileNetSSD on Pi Camera.

Runs a dedicated detection thread that:
  1. Captures frames from Picamera2
  2. Runs MobileNetSSD DNN inference (OpenCV)
  3. Annotates frames with bounding boxes + class labels
  4. Estimates "virtual distance" from bounding-box size
  5. Exposes a camera-obstacle-distance callable for autopilot fusion

The annotated frames are served to the MJPEG stream so the user sees
live object detection overlaid on the camera feed.

Detection classes (PASCAL VOC 21):
  background, aeroplane, bicycle, bird, boat, bottle, bus, car, cat,
  chair, cow, diningtable, dog, horse, motorbike, person, pottedplant,
  sheep, sofa, train, tvmonitor
"""

import time
import threading
import cv2
import numpy as np
from collections import deque


# ── PASCAL VOC class labels for MobileNetSSD ──────────
CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor",
]

# Colours per class for bounding-box rendering (BGR)
# Use a hash to generate a stable colour per class index
CLASS_COLORS = {}
for i, name in enumerate(CLASSES):
    np.random.seed(i * 42)
    CLASS_COLORS[i] = tuple(int(c) for c in np.random.randint(100, 255, size=3))


class VisionSystem:
    """
    Real-time object detection using MobileNetSSD on Picamera2 frames.

    Parameters
    ----------
    picam2 : Picamera2
        An already-started Picamera2 instance (BGR888, 640×480).
    confidence_threshold : float
        Minimum confidence to accept a detection (0.0–1.0).
    model_prototxt : str
        Path to MobileNetSSD deploy prototxt.
    model_weights : str
        Path to MobileNetSSD caffemodel weights.
    """

    def __init__(
        self,
        picam2,
        confidence_threshold=0.45,
        model_prototxt="models/MobileNetSSD_deploy.prototxt",
        model_weights="models/MobileNetSSD_deploy.caffemodel",
    ):
        self._picam2 = picam2
        self._confidence = confidence_threshold

        # Load DNN model
        print("🧠 [Vision] Loading MobileNetSSD model...")
        self._net = cv2.dnn.readNetFromCaffe(model_prototxt, model_weights)
        # Use CPU backend (Pi doesn't have CUDA typically)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print("✅ [Vision] MobileNetSSD model loaded successfully")

        # ── Shared state (protected by lock) ──────────
        self._lock = threading.Lock()
        self._latest_detections = []       # list of detection dicts
        self._latest_annotated_frame = None
        self._latest_raw_frame = None
        self._latest_frame_id = 0
        self._detection_fps = 0.0
        self._active = False               # True = run DNN inference
        self._stream_enabled = False       # True = capture frames for stream/MJPEG
        self._running = False              # True = thread alive
        self._camera_obstacle_distance = 999.0  # virtual distance (cm)

        # ── Path corridor definition ──────────────────
        # Objects overlapping this ROI (relative to frame) are "in path"
        # Center 60% of width, bottom 70% of height
        self._path_x_min = 0.20   # left boundary (20% from left)
        self._path_x_max = 0.80   # right boundary (80% from left)
        self._path_y_min = 0.30   # top boundary (30% from top)
        self._path_y_max = 1.00   # bottom boundary (100% = bottom)

        # ── Distance estimation constant ──────────────
        # virtual_distance = DISTANCE_K / bbox_height_ratio
        # where bbox_height_ratio = bbox_height / frame_height
        # Tuned so a person filling ~50% of frame height ≈ 30cm away
        self.DISTANCE_K = 18.0
        self.DISTANCE_MIN = 5.0    # clamp minimum (cm)
        self.DISTANCE_MAX = 200.0   # clamp maximum (cm)

        # FPS tracking
        self._fps_history = deque(maxlen=10)

        self._thread = None

    # ── Properties ─────────────────────────────────────

    @property
    def active(self):
        return self._active

    @active.setter
    def active(self, value):
        self._active = bool(value)

    @property
    def detection_fps(self):
        with self._lock:
            return self._detection_fps

    @property
    def stream_enabled(self):
        return self._stream_enabled

    @stream_enabled.setter
    def stream_enabled(self, value):
        self._stream_enabled = bool(value)

    # ── Start / Stop ───────────────────────────────────

    def start(self):
        """Launch the detection daemon thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._detection_loop, daemon=True)
        self._thread.start()
        print("🧠 [Vision] Detection thread started")

    def stop(self):
        """Signal the detection thread to stop."""
        self._running = False
        self._active = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        print("🧠 [Vision] Detection thread stopped")

    # ── Public getters ─────────────────────────────────

    def get_camera_obstacle_distance(self):
        """
        Return the minimum 'virtual distance' (cm) among all in-path
        detections.  Returns 999.0 if no obstacles detected.
        Same convention as sonar (higher = safer).
        """
        with self._lock:
            return self._camera_obstacle_distance

    def get_detections(self):
        """Return a snapshot of current detections list."""
        with self._lock:
            return list(self._latest_detections)

    def get_detections_summary(self):
        """
        Return a compact summary for telemetry:
        {count, closest_class, closest_distance, vision_fps, active}
        """
        with self._lock:
            dets = self._latest_detections
            in_path = [d for d in dets if d.get("in_path")]
            closest = min(in_path, key=lambda d: d["distance"], default=None) if in_path else None
            return {
                "count": len(dets),
                "in_path_count": len(in_path),
                "closest_class": closest["class_name"] if closest else "",
                "closest_confidence": round(closest["confidence"] * 100) if closest else 0,
                "closest_distance": round(closest["distance"], 1) if closest else 999.0,
                "camera_obstacle_distance": round(self._camera_obstacle_distance, 1),
                "vision_fps": round(self._detection_fps, 1),
                "vision_active": self._active,
            }

    def get_frame_with_id(self):
        """
        Return the latest annotated frame (with bounding boxes) if
        available, otherwise the raw frame, along with a monotonic
        frame id. Returns (None, frame_id) if no frame exists yet.
        """
        with self._lock:
            if self._latest_annotated_frame is not None:
                return self._latest_annotated_frame.copy(), self._latest_frame_id
            elif self._latest_raw_frame is not None:
                return self._latest_raw_frame.copy(), self._latest_frame_id
            return None, self._latest_frame_id

    def get_frame(self):
        """Backwards-compatible frame getter."""
        frame, _ = self.get_frame_with_id()
        return frame

    # ── Detection thread ───────────────────────────────

    def _detection_loop(self):
        """
        Main detection loop.  Runs on a dedicated daemon thread.
        Always captures frames (for MJPEG).  Only runs DNN when active.
        """
        print("🧠 [Vision] Detection loop running...")

        while self._running:
            try:
                # Nothing needs camera frames right now.
                if not self._stream_enabled and not self._active:
                    time.sleep(0.05)
                    continue

                # ── 1. Capture frame ──────────────────
                frame = self._picam2.capture_array()
                if frame is None:
                    time.sleep(0.05)
                    continue

                (h, w) = frame.shape[:2]

                # ── 2. Store raw frame (always, for MJPEG fallback) ───
                with self._lock:
                    self._latest_raw_frame = frame.copy()
                    self._latest_frame_id += 1

                # ── 3. If not active, skip DNN ────────
                if not self._active:
                    with self._lock:
                        self._latest_annotated_frame = frame.copy()
                        self._latest_detections = []
                        self._camera_obstacle_distance = 999.0
                        self._detection_fps = 0.0
                    time.sleep(0.03)  # ~30 FPS for raw frames
                    continue

                # ── 4. DNN inference ──────────────────
                t_start = time.time()

                blob = cv2.dnn.blobFromImage(
                    cv2.resize(frame, (300, 300)),
                    0.007843,          # scale factor (1/127.5)
                    (300, 300),        # input size
                    127.5,             # mean subtraction
                )
                self._net.setInput(blob)
                detections_raw = self._net.forward()

                t_inference = time.time() - t_start

                # ── 5. Parse detections ───────────────
                parsed = []
                min_path_distance = 999.0
                annotated = frame.copy()

                for i in range(detections_raw.shape[2]):
                    confidence = detections_raw[0, 0, i, 2]
                    if confidence < self._confidence:
                        continue

                    class_id = int(detections_raw[0, 0, i, 1])
                    if class_id < 0 or class_id >= len(CLASSES):
                        continue
                    class_name = CLASSES[class_id]
                    if class_name == "background":
                        continue

                    # Bounding box (pixel coords)
                    box = detections_raw[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (x1, y1, x2, y2) = box.astype("int")
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(w, x2)
                    y2 = min(h, y2)

                    bbox_h = y2 - y1
                    bbox_w = x2 - x1
                    if bbox_h <= 0 or bbox_w <= 0:
                        continue

                    # ── 6. In-path check ──────────────
                    # Bounding box center
                    cx = (x1 + x2) / (2.0 * w)
                    cy = (y1 + y2) / (2.0 * h)
                    # Check if bbox overlaps the path corridor
                    in_path = (
                        cx >= self._path_x_min
                        and cx <= self._path_x_max
                        and cy >= self._path_y_min
                    )

                    # ── 7. Distance estimation ────────
                    bbox_height_ratio = bbox_h / float(h)
                    distance = self.DISTANCE_K / max(bbox_height_ratio, 0.01)
                    distance = max(self.DISTANCE_MIN, min(self.DISTANCE_MAX, distance))

                    if in_path and distance < min_path_distance:
                        min_path_distance = distance

                    det = {
                        "class_name": class_name,
                        "class_id": class_id,
                        "confidence": float(confidence),
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                        "in_path": in_path,
                        "distance": round(distance, 1),
                    }
                    parsed.append(det)

                    # ── 8. Draw bounding box ──────────
                    color = (0, 0, 255) if in_path else CLASS_COLORS.get(class_id, (0, 255, 0))
                    thickness = 2 if in_path else 1
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

                    # Label
                    label = f"{class_name} {confidence:.0%} {distance:.0f}cm"
                    label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                    # Background rectangle for label
                    cv2.rectangle(
                        annotated,
                        (x1, y1 - label_size[1] - baseline - 4),
                        (x1 + label_size[0], y1),
                        color,
                        cv2.FILLED,
                    )
                    cv2.putText(
                        annotated, label,
                        (x1, y1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1,
                    )

                # ── Draw path corridor overlay ────────
                path_x1 = int(self._path_x_min * w)
                path_x2 = int(self._path_x_max * w)
                path_y1 = int(self._path_y_min * h)
                # Translucent corridor lines
                cv2.line(annotated, (path_x1, path_y1), (path_x1, h), (0, 255, 255), 1)
                cv2.line(annotated, (path_x2, path_y1), (path_x2, h), (0, 255, 255), 1)
                cv2.line(annotated, (path_x1, path_y1), (path_x2, path_y1), (0, 255, 255), 1)

                # ── FPS + status overlay ──────────────
                self._fps_history.append(1.0 / max(t_inference, 0.001))
                avg_fps = sum(self._fps_history) / len(self._fps_history)

                status_text = f"Vision: {avg_fps:.1f} FPS | {len(parsed)} obj"
                if min_path_distance < 999:
                    status_text += f" | PATH: {min_path_distance:.0f}cm"
                cv2.putText(
                    annotated, status_text,
                    (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 255), 1,
                )

                # ── 9. Update shared state ────────────
                with self._lock:
                    self._latest_annotated_frame = annotated
                    self._latest_detections = parsed
                    self._camera_obstacle_distance = min_path_distance
                    self._detection_fps = avg_fps

            except Exception as e:
                print(f"❌ [Vision] Detection error: {e}")
                time.sleep(0.1)

        print("🧠 [Vision] Detection loop exited")
