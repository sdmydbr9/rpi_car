"""
odom_nav.py — Odometry-aware navigation helpers

Provides:
  - Distance/angle-based escape maneuvers (replaces sleep-based timing)
  - Short-lived obstacle memory in the odometry frame
  - Stuck/slip detection using commanded vs actual motion
  - Return-to-start and backtrack-recent-path behavior
  - Trail recording for the web UI mini-map
"""

import csv
import math
import os
import time
import threading
from collections import deque
from typing import Callable, Optional, Tuple, List, Dict

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
    """Standalone-style return-to-start navigator used by the main app."""

    WAYPOINT_SPACING_M = 0.30
    ARRIVE_DIST_M = 0.20
    FINAL_ARRIVE_M = 0.10
    NAV_SPEED = 50
    SLOW_SPEED = 40
    STEER_GAIN = 1.8
    MAX_STEER = 35
    OBSTACLE_DIST_MM = 200
    OBSTACLE_CLEAR_MM = 300
    STUCK_WINDOW_S = 2.0
    STUCK_DIST_M = 0.015
    MAX_STUCK_COUNT = 3
    UTURN_THRESHOLD_DEG = 90
    BREAKAWAY_STEP = 5
    BREAKAWAY_MAX = 90
    RPM_MOVING = 3.0
    PHYSICAL_STEER_MAX_DEG = 30.0
    SERVO_STEER_MAX_DEG = 50.0

    def __init__(
        self,
        car_system,
        get_position,
        get_heading_deg,
        trail_recorder: TrailRecorder,
        obstacle_memory: ObstacleMemory,
        get_sensor_packet: Optional[Callable[[], object]] = None,
        get_linear_velocity: Optional[Callable[[], float]] = None,
        get_angular_velocity: Optional[Callable[[], float]] = None,
        log_dir: Optional[str] = None,
    ):
        self._car = car_system
        self._get_pos = get_position
        self._get_hdg = get_heading_deg
        self._trail = trail_recorder
        self._obstacles = obstacle_memory
        self._get_sensor_packet = get_sensor_packet or (lambda: None)
        self._get_linear_velocity = get_linear_velocity or (lambda: 0.0)
        self._get_angular_velocity = get_angular_velocity or (lambda: 0.0)
        self._log_dir = log_dir
        self._active = False
        self._thread: Optional[threading.Thread] = None
        self._status = "idle"
        self._waypoints: List[Tuple[float, float]] = []
        self._current_wp = 0
        self._total_wp = 0
        self._applied_pwm = 0
        self._wheels_spinning = False
        self._state_lock = threading.Lock()

    @property
    def active(self):
        return self._active

    @property
    def status(self):
        return self._status

    @property
    def current_waypoint(self):
        return self._current_wp

    @property
    def total_waypoints(self):
        return self._total_wp

    def get_waypoints(self) -> List[Tuple[float, float]]:
        with self._state_lock:
            return list(self._waypoints)

    def start(self):
        if self._active:
            return
        self._active = True
        self._status = "navigating"
        self._thread = threading.Thread(target=self._run, daemon=True, name="rth-nav")
        self._thread.start()

    def stop(self):
        self._active = False
        self._status = "cancelled"
        self._reset_drive_state()
        try:
            self._car.brake()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _thin_waypoints(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not points:
            return []
        result = [points[0]]
        for x, y in points[1:]:
            lx, ly = result[-1]
            dx, dy = x - lx, y - ly
            if dx * dx + dy * dy >= self.WAYPOINT_SPACING_M ** 2:
                result.append((x, y))
        return result

    def _physical_to_servo_angle(self, angle_deg: float) -> float:
        angle_deg = max(-self.MAX_STEER, min(self.MAX_STEER, float(angle_deg)))
        servo = angle_deg * (self.SERVO_STEER_MAX_DEG / self.PHYSICAL_STEER_MAX_DEG)
        return max(-self.SERVO_STEER_MAX_DEG, min(self.SERVO_STEER_MAX_DEG, servo))

    def _drive(self, target_pwm: float, steer_deg: float, forward: bool = True) -> int:
        packet = self._get_sensor_packet()
        avg_rpm = 0.0
        if packet is not None:
            avg_rpm = (abs(packet.rpm_left) + abs(packet.rpm_right)) / 2.0

        if avg_rpm >= self.RPM_MOVING:
            self._wheels_spinning = True
            if self._applied_pwm > target_pwm:
                self._applied_pwm = max(int(target_pwm), self._applied_pwm - 3)
            else:
                self._applied_pwm = int(target_pwm)
        else:
            self._wheels_spinning = False
            if self._applied_pwm < target_pwm:
                self._applied_pwm = int(target_pwm)
            self._applied_pwm = min(self.BREAKAWAY_MAX, self._applied_pwm + self.BREAKAWAY_STEP)

        pwm = int(max(0, min(100, self._applied_pwm)))
        servo_angle = self._physical_to_servo_angle(steer_deg)
        self._car._apply_steering(
            pwm,
            servo_angle,
            forward=forward,
            speed_l=pwm,
            speed_r=pwm,
        )
        return pwm

    def _reset_drive_state(self):
        self._applied_pwm = 0
        self._wheels_spinning = False

    def _open_log(self):
        if not self._log_dir:
            return None, None
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(self._log_dir, f"rth_log_{timestamp}.csv")
            csv_file = open(log_path, "w", newline="", buffering=1)
            writer = csv.writer(csv_file)
            writer.writerow([
                "elapsed_s", "event", "wp_idx", "wp_total",
                "target_x", "target_y", "pos_x", "pos_y", "heading_deg",
                "v_linear", "v_angular", "dist_m", "desired_deg", "error_deg",
                "steer_deg", "cmd_pwm", "laser_mm", "rpm_left", "rpm_right", "note",
            ])
            return csv_file, writer
        except Exception:
            return None, None

    def _log(self, writer, start_time: float, event: str, wp_idx: int = 0,
             target_x: float = 0.0, target_y: float = 0.0, dist_m: float = 0.0,
             desired_deg: float = 0.0, error_deg: float = 0.0,
             steer_deg: float = 0.0, cmd_pwm: float = 0.0,
             packet=None, note: str = ""):
        if writer is None:
            return
        pos_x, pos_y = self._get_pos()
        heading_deg = self._get_hdg()
        v_linear = self._get_linear_velocity()
        v_angular = self._get_angular_velocity()
        laser_mm = packet.laser_mm if packet is not None else 0
        rpm_left = packet.rpm_left if packet is not None else 0.0
        rpm_right = packet.rpm_right if packet is not None else 0.0
        writer.writerow([
            f"{time.monotonic() - start_time:.3f}",
            event,
            wp_idx,
            self._total_wp,
            f"{target_x:.4f}",
            f"{target_y:.4f}",
            f"{pos_x:.4f}",
            f"{pos_y:.4f}",
            f"{heading_deg:.1f}",
            f"{v_linear:.4f}",
            f"{v_angular:.4f}",
            f"{dist_m:.4f}",
            f"{desired_deg:.1f}",
            f"{error_deg:.1f}",
            f"{steer_deg:.1f}",
            int(cmd_pwm),
            laser_mm,
            f"{rpm_left:.1f}",
            f"{rpm_right:.1f}",
            note,
        ])

    def _run(self):
        import logging

        csv_file, writer = self._open_log()
        started_at = time.monotonic()

        try:
            logging.info("[RTH] Starting return-to-start navigation")
            trail_points = self._trail.get_full_path()

            if len(trail_points) < 2:
                waypoints = [(0.0, 0.0)]
                logging.info("[RTH] No trail recorded, driving directly to origin")
            else:
                trail_points.reverse()
                waypoints = self._thin_waypoints(trail_points)
                last_x, last_y = waypoints[-1] if waypoints else (1.0, 1.0)
                if math.sqrt(last_x * last_x + last_y * last_y) > self.FINAL_ARRIVE_M:
                    waypoints.append((0.0, 0.0))

            with self._state_lock:
                self._waypoints = waypoints
                self._current_wp = 0
                self._total_wp = len(waypoints)

            logging.info("[RTH] Following %d waypoints back to start", len(waypoints))
            self._log(writer, started_at, "INIT", note=f"trail_points={len(trail_points)}")
            self._reset_drive_state()

            wp_idx = 0
            while wp_idx < len(waypoints):
                if not self._active:
                    self._status = "cancelled"
                    break

                target_x, target_y = waypoints[wp_idx]
                is_final = (wp_idx == len(waypoints) - 1)
                arrive_dist = self.FINAL_ARRIVE_M if is_final else self.ARRIVE_DIST_M
                with self._state_lock:
                    self._current_wp = wp_idx

                if not is_final:
                    best_skip = wp_idx
                    pos_x, pos_y = self._get_pos()
                    for skip_idx in range(wp_idx + 1, len(waypoints)):
                        skip_x, skip_y = waypoints[skip_idx]
                        if math.hypot(skip_x - pos_x, skip_y - pos_y) < self.ARRIVE_DIST_M:
                            best_skip = skip_idx
                    if best_skip > wp_idx:
                        logging.info("[RTH] Look-ahead skip wp %d -> %d", wp_idx, best_skip)
                        self._log(writer, started_at, "LOOKAHEAD_SKIP", wp_idx, target_x, target_y, note=f"skip_to={best_skip}")
                        wp_idx = best_skip
                        continue

                pos_x, pos_y = self._get_pos()
                desired_deg = math.degrees(math.atan2(target_y - pos_y, target_x - pos_x))
                initial_error = abs(_wrap_angle_deg(desired_deg - self._get_hdg()))

                if is_final and initial_error > self.UTURN_THRESHOLD_DEG and self._active:
                    logging.info("[RTH] Executing U-turn before final origin approach")
                    turn_dir = -self.MAX_STEER if _wrap_angle_deg(desired_deg - self._get_hdg()) < 0 else self.MAX_STEER
                    for _ in range(120):
                        if not self._active:
                            break
                        packet = self._get_sensor_packet()
                        if packet is not None and 0 < packet.laser_mm < self.OBSTACLE_DIST_MM:
                            self._car.brake()
                            self._reset_drive_state()
                            time.sleep(0.5)
                            continue
                        self._drive(self.NAV_SPEED, turn_dir, forward=True)
                        time.sleep(0.05)
                        cur_x, cur_y = self._get_pos()
                        fresh_desired = math.degrees(math.atan2(target_y - cur_y, target_x - cur_x))
                        if abs(_wrap_angle_deg(fresh_desired - self._get_hdg())) < 45:
                            break
                    self._car.brake()
                    self._reset_drive_state()
                    time.sleep(0.1)

                max_ticks = 600
                stuck_count = 0
                stuck_check_time = time.monotonic()
                stuck_check_pos = self._get_pos()
                waypoint_done = False

                for tick in range(max_ticks):
                    if not self._active:
                        self._status = "cancelled"
                        break

                    packet = self._get_sensor_packet()
                    if packet is not None and 0 < packet.laser_mm < self.OBSTACLE_DIST_MM:
                        self._status = "obstacle"
                        self._log(writer, started_at, "OBSTACLE", wp_idx, target_x, target_y, packet=packet, note=f"laser={packet.laser_mm}")
                        self._car.brake()
                        self._reset_drive_state()
                        deadline = time.monotonic() + 10.0
                        while self._active and time.monotonic() < deadline:
                            time.sleep(0.1)
                            packet = self._get_sensor_packet()
                            if packet is not None and (packet.laser_mm <= 0 or packet.laser_mm >= self.OBSTACLE_CLEAR_MM):
                                break
                        self._status = "navigating"
                        stuck_check_time = time.monotonic()
                        stuck_check_pos = self._get_pos()
                        continue

                    pos_x, pos_y = self._get_pos()
                    dx = target_x - pos_x
                    dy = target_y - pos_y
                    dist = math.hypot(dx, dy)

                    if dist < arrive_dist:
                        self._log(writer, started_at, "WP_ARRIVED", wp_idx, target_x, target_y, dist, packet=packet)
                        waypoint_done = True
                        break

                    if not is_final and tick % 5 == 0:
                        best_skip = wp_idx
                        for skip_idx in range(wp_idx + 1, len(waypoints)):
                            skip_x, skip_y = waypoints[skip_idx]
                            if math.hypot(skip_x - pos_x, skip_y - pos_y) < self.ARRIVE_DIST_M:
                                best_skip = skip_idx
                        if best_skip > wp_idx:
                            logging.info("[RTH] Navigation skip wp %d -> %d", wp_idx, best_skip)
                            wp_idx = best_skip
                            waypoint_done = True
                            break

                    now = time.monotonic()
                    if now - stuck_check_time >= self.STUCK_WINDOW_S:
                        start_x, start_y = stuck_check_pos
                        moved = math.hypot(pos_x - start_x, pos_y - start_y)
                        if moved < self.STUCK_DIST_M:
                            stuck_count += 1
                            logging.warning("[RTH] Stuck near wp %d (%d/%d)", wp_idx, stuck_count, self.MAX_STUCK_COUNT)
                            self._log(writer, started_at, "STUCK", wp_idx, target_x, target_y, dist, packet=packet, note=f"count={stuck_count}")
                            if stuck_count >= self.MAX_STUCK_COUNT:
                                self._car.brake()
                                self._reset_drive_state()
                                time.sleep(0.3)
                                break
                            self._reset_drive_state()
                            for _ in range(16):
                                self._drive(50, 0.0, forward=False)
                                time.sleep(0.05)
                            self._car.brake()
                            self._reset_drive_state()
                            time.sleep(0.2)
                        else:
                            stuck_count = max(0, stuck_count - 1)
                        stuck_check_time = now
                        stuck_check_pos = (pos_x, pos_y)

                    desired_deg = math.degrees(math.atan2(dy, dx))
                    current_deg = self._get_hdg()
                    error_deg = _wrap_angle_deg(desired_deg - current_deg)

                    if not is_final and abs(error_deg) > 120:
                        self._log(writer, started_at, "WP_BEHIND_SKIP", wp_idx, target_x, target_y, dist, desired_deg, error_deg, packet=packet)
                        break

                    speed = self.NAV_SPEED if abs(error_deg) < 30 else self.SLOW_SPEED
                    steer_deg = max(-self.MAX_STEER, min(self.MAX_STEER, error_deg * self.STEER_GAIN))
                    applied_pwm = self._drive(speed, steer_deg, forward=True)

                    if tick < 5 or tick % 10 == 0:
                        self._log(
                            writer,
                            started_at,
                            "NAV",
                            wp_idx,
                            target_x,
                            target_y,
                            dist,
                            desired_deg,
                            error_deg,
                            steer_deg,
                            applied_pwm,
                            packet=packet,
                            note=f"spinning={self._wheels_spinning}",
                        )
                    time.sleep(0.05)

                if not waypoint_done and self._active:
                    self._log(writer, started_at, "WP_TIMEOUT", wp_idx, target_x, target_y, note=f"stuck={stuck_count}")
                wp_idx += 1

            self._car.brake()
            self._reset_drive_state()
            if self._active:
                self._status = "arrived"
                logging.info("[RTH] Arrived near origin at (%.3f, %.3f)", *self._get_pos())
            else:
                self._status = "cancelled"
        except Exception as exc:
            import logging
            self._status = "idle"
            logging.error("[RTH] Navigation error: %s", exc, exc_info=True)
            try:
                self._car.brake()
            except Exception:
                pass
            self._reset_drive_state()
        finally:
            self._active = False
            with self._state_lock:
                self._current_wp = 0
            if csv_file is not None:
                csv_file.close()
