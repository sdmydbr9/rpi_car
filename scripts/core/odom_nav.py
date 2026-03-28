"""
odom_nav.py — Odometry-aware navigation helpers

Provides:
  - Distance/angle-based escape maneuvers (replaces sleep-based timing)
  - Short-lived obstacle memory in the odometry frame
  - Stuck/slip detection using commanded vs actual motion
  - Return-to-start and backtrack-recent-path behavior
  - Trail recording for the web UI mini-map
"""

import math
import time
import threading
from collections import deque
from typing import Optional, Tuple, List, Dict

# ---------------------------------------------------------------------------
# Obstacle Memory (project laser hits into world frame, keep 1-2 s)
# ---------------------------------------------------------------------------

class ObstacleMemory:
    """Short-lived memory of detected obstacles in the odometry (world) frame."""

    MAX_AGE_S = 2.0
    MAX_POINTS = 60

    def __init__(self):
        self._lock = threading.Lock()
        # Each entry: (x_world, y_world, timestamp)
        self._points: deque = deque(maxlen=self.MAX_POINTS)

    def add_laser_hit(self, robot_x: float, robot_y: float,
                      robot_heading_rad: float, laser_dist_m: float,
                      laser_angle_rad: float = 0.0) -> None:
        """Project a laser hit from robot-local frame into world frame and store it."""
        if laser_dist_m <= 0 or laser_dist_m > 3.0:
            return  # ignore invalid / far readings
        # Laser reading in robot frame → world frame
        world_angle = robot_heading_rad + laser_angle_rad
        obs_x = robot_x + laser_dist_m * math.cos(world_angle)
        obs_y = robot_y + laser_dist_m * math.sin(world_angle)
        with self._lock:
            self._points.append((obs_x, obs_y, time.monotonic()))

    def get_obstacles(self) -> List[Dict]:
        """Return list of {x, y, age_s} for live obstacles (pruned by age)."""
        now = time.monotonic()
        with self._lock:
            # Prune old entries
            while self._points and (now - self._points[0][2]) > self.MAX_AGE_S:
                self._points.popleft()
            return [
                {"x": round(x, 3), "y": round(y, 3), "age_s": round(now - t, 2)}
                for x, y, t in self._points
            ]

    def is_near_known_obstacle(self, x: float, y: float,
                                radius_m: float = 0.15) -> bool:
        """Check if a world point is near any remembered obstacle."""
        now = time.monotonic()
        r2 = radius_m * radius_m
        with self._lock:
            for ox, oy, t in self._points:
                if (now - t) > self.MAX_AGE_S:
                    continue
                if (ox - x) ** 2 + (oy - y) ** 2 < r2:
                    return True
        return False

    def clear(self):
        with self._lock:
            self._points.clear()


# ---------------------------------------------------------------------------
# Stuck / Slip Detection
# ---------------------------------------------------------------------------

class StuckSlipDetector:
    """Detect when the rover is stuck or experiencing wheel slip.

    Stuck:  Motor command is high but v_linear barely changes over a window.
    Slip:   Encoder-reported speed much higher than odometry v_linear (spinning wheels).
    """

    STUCK_WINDOW_S = 1.0          # evaluation window
    STUCK_CMD_THRESHOLD = 30.0    # PWM % considered "trying to move"
    STUCK_VELOCITY_THRESHOLD = 0.02  # m/s — less than this = not moving
    SLIP_RPM_TO_MS = 0.0375 * 2 * math.pi / 60.0  # wheel_radius * 2π / 60

    def __init__(self):
        self._lock = threading.Lock()
        self._history: deque = deque(maxlen=50)  # (timestamp, cmd_pwm, v_linear, avg_rpm)
        self.stuck = False
        self.slip = False
        self._last_check = 0.0

    def update(self, cmd_pwm: float, v_linear: float,
               rpm_left: float, rpm_right: float) -> None:
        """Feed new sample. Call at ~20 Hz."""
        now = time.monotonic()
        avg_rpm = (abs(rpm_left) + abs(rpm_right)) / 2.0
        with self._lock:
            self._history.append((now, abs(cmd_pwm), abs(v_linear), avg_rpm))

        # Evaluate every 0.25 s
        if now - self._last_check < 0.25:
            return
        self._last_check = now
        self._evaluate(now)

    def _evaluate(self, now: float):
        with self._lock:
            # Filter to recent window
            window = [(t, c, v, r) for t, c, v, r in self._history
                      if (now - t) <= self.STUCK_WINDOW_S]

        if len(window) < 5:
            self.stuck = False
            self.slip = False
            return

        avg_cmd = sum(c for _, c, _, _ in window) / len(window)
        avg_vel = sum(v for _, _, v, _ in window) / len(window)
        avg_rpm = sum(r for _, _, _, r in window) / len(window)

        # Stuck: commanding motion but not moving
        self.stuck = (avg_cmd > self.STUCK_CMD_THRESHOLD and
                      avg_vel < self.STUCK_VELOCITY_THRESHOLD)

        # Slip: encoder says wheels spinning fast but odometry shows slow movement
        expected_vel = avg_rpm * self.SLIP_RPM_TO_MS
        if expected_vel > 0.05:  # only check if wheels reporting meaningful RPM
            ratio = avg_vel / expected_vel
            self.slip = ratio < 0.3  # less than 30% of expected → slip
        else:
            self.slip = False

    def reset(self):
        with self._lock:
            self._history.clear()
        self.stuck = False
        self.slip = False


# ---------------------------------------------------------------------------
# Trail Recorder (for web UI mini-map)
# ---------------------------------------------------------------------------

class TrailRecorder:
    """Record odometry positions for breadcrumb trail display."""

    MAX_POINTS = 1000
    MIN_DIST_M = 0.01  # record new point every ~1 cm

    def __init__(self):
        self._lock = threading.Lock()
        self._trail: deque = deque(maxlen=self.MAX_POINTS)

    def update(self, x: float, y: float) -> None:
        with self._lock:
            if self._trail:
                lx, ly = self._trail[-1]
                dx, dy = x - lx, y - ly
                if dx * dx + dy * dy < self.MIN_DIST_M ** 2:
                    return
            self._trail.append((x, y))

    def get_trail(self) -> List[Dict]:
        with self._lock:
            return [{"x": round(px, 3), "y": round(py, 3)} for px, py in self._trail]

    def get_recent_path(self, n: int = 20) -> List[Tuple[float, float]]:
        """Return the last N trail points as [(x,y), ...] for backtracking."""
        with self._lock:
            pts = list(self._trail)
        return [(x, y) for x, y in pts[-n:]]

    def get_full_path(self) -> List[Tuple[float, float]]:
        """Return all trail points as [(x,y), ...] for full backtracking."""
        with self._lock:
            return [(x, y) for x, y in self._trail]

    def clear(self):
        with self._lock:
            self._trail.clear()


# ---------------------------------------------------------------------------
# Distance/Angle-Based Maneuver Helpers
# ---------------------------------------------------------------------------

def wait_for_distance(get_position, start_x: float, start_y: float,
                      target_dist_m: float, timeout_s: float = 5.0) -> bool:
    """Block until the rover has moved target_dist_m from (start_x, start_y).

    Returns True if distance was reached, False on timeout.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        x, y = get_position()
        dx, dy = x - start_x, y - start_y
        if math.sqrt(dx * dx + dy * dy) >= target_dist_m:
            return True
        time.sleep(0.02)
    return False


def wait_for_heading_change(get_heading_deg, start_heading: float,
                            target_delta_deg: float, timeout_s: float = 5.0) -> bool:
    """Block until heading has changed by target_delta_deg from start_heading.

    Returns True if angle reached, False on timeout.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = get_heading_deg()
        delta = abs(_wrap_angle_deg(current - start_heading))
        if delta >= abs(target_delta_deg):
            return True
        time.sleep(0.02)
    return False


def _wrap_angle_deg(angle: float) -> float:
    """Normalize angle to [-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# ---------------------------------------------------------------------------
# Return-to-Start Navigator
# ---------------------------------------------------------------------------

class ReturnToStartNavigator:
    """Simple return-to-start using odometry pose.

    Strategy: backtrack along the full recorded trail (reversed),
    thinned to ~10 cm waypoint spacing for smooth driving.
    Final waypoint is always (0, 0) — the start position.
    """

    WAYPOINT_SPACING_M = 0.10  # thin trail to ~10 cm between waypoints
    ARRIVE_DIST_M = 0.08       # 8 cm arrival tolerance per waypoint
    FINAL_ARRIVE_M = 0.05      # 5 cm for the final origin waypoint
    NAV_SPEED = 35              # PWM % for RTH driving
    STEER_GAIN = 1.8            # heading error → steering angle gain
    MAX_STEER = 35              # max steering angle (degrees)

    def __init__(self, car_system, get_position, get_heading_deg,
                 trail_recorder: TrailRecorder, obstacle_memory: ObstacleMemory):
        self._car = car_system
        self._get_pos = get_position
        self._get_hdg = get_heading_deg
        self._trail = trail_recorder
        self._obstacles = obstacle_memory
        self._active = False
        self._thread: Optional[threading.Thread] = None

    @property
    def active(self):
        return self._active

    def start(self):
        if self._active:
            return
        self._active = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._active = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _thin_waypoints(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Reduce dense trail to waypoints spaced ~WAYPOINT_SPACING_M apart."""
        if not points:
            return []
        result = [points[0]]
        for x, y in points[1:]:
            lx, ly = result[-1]
            dx, dy = x - lx, y - ly
            if dx * dx + dy * dy >= self.WAYPOINT_SPACING_M ** 2:
                result.append((x, y))
        return result

    def _run(self):
        """Backtrack along full recorded path, then drive to origin."""
        import logging
        try:
            logging.info("[RTH] Starting return-to-start navigation")

            # Get full trail and reverse it for backtracking
            full_trail = self._trail.get_full_path()

            if len(full_trail) < 2:
                # No meaningful trail — drive directly to origin
                logging.info("[RTH] No trail recorded, driving directly to origin")
                waypoints = [(0.0, 0.0)]
            else:
                # Reverse trail (backtrack from current position toward start)
                full_trail.reverse()
                # Thin to manageable waypoint spacing
                waypoints = self._thin_waypoints(full_trail)
                # Always end at exact origin
                lx, ly = waypoints[-1] if waypoints else (1.0, 1.0)
                if math.sqrt(lx * lx + ly * ly) > self.FINAL_ARRIVE_M:
                    waypoints.append((0.0, 0.0))

            logging.info("[RTH] Following %d waypoints back to start", len(waypoints))

            for i, (wx, wy) in enumerate(waypoints):
                if not self._active:
                    logging.info("[RTH] Cancelled by user at waypoint %d/%d", i, len(waypoints))
                    break
                is_final = (i == len(waypoints) - 1)
                arrive_dist = self.FINAL_ARRIVE_M if is_final else self.ARRIVE_DIST_M
                self._navigate_to(wx, wy, arrive_dist)

            # Arrived (or cancelled) — stop motors
            self._car.brake()
            logging.info("[RTH] Navigation complete — stopped at (%.2f, %.2f)",
                         *self._get_pos())
        except Exception as e:
            import logging
            logging.error("[RTH] Navigation error: %s", e, exc_info=True)
            self._car.brake()
        finally:
            self._active = False

    def _navigate_to(self, target_x: float, target_y: float,
                     arrive_dist: float = 0.08):
        """Steer toward target waypoint until within arrive_dist."""
        MAX_TICKS = 400  # ~8 s at 20 Hz — generous timeout per waypoint

        for _ in range(MAX_TICKS):
            if not self._active:
                return

            cx, cy = self._get_pos()
            dx = target_x - cx
            dy = target_y - cy
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < arrive_dist:
                return

            # Desired heading toward waypoint (math atan2 → angle from +X axis)
            desired_deg = math.degrees(math.atan2(dy, dx))
            current_deg = self._get_hdg()
            error = _wrap_angle_deg(desired_deg - current_deg)

            # If heading error is very large (>90°), slow down
            speed = self.NAV_SPEED if abs(error) < 60 else max(20, self.NAV_SPEED // 2)

            # Proportional steering
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, error * self.STEER_GAIN))
            self._car._apply_steering(speed, steer, forward=True)
            time.sleep(0.05)

        # Timeout on this waypoint — move on
        self._car.stop()
        time.sleep(0.1)
