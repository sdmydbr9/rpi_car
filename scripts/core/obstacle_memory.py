"""
obstacle_memory.py — Forward Obstacle Memory + Polar Hazard Map + Short-Horizon Planner

Lightweight planner for a rover with a single fixed forward-facing VL53L0X laser.
Remembers recent obstacle hits in world coordinates, reprojects them into body frame
each tick, builds a polar hazard map, and scores candidate steering arcs.

Three observation classes:
  - HAZARD:   obstacle detected at measured range
  - CLEAR:    forward corridor observed clear up to measured range
  - UNKNOWN:  no recent observation (NOT safe to assume clear)

Design:
  - All world-frame storage; body-frame reprojection each tick
  - Decaying confidence via age
  - Rover-width inflation on hazard bins
  - Candidate arc scoring with collision/proximity/unknown/smoothness costs
  - Probe behavior when all arcs score poorly
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict

# ── Tuning Defaults ────────────────────────────────────

MEMORY_TTL_S = 1.2            # seconds before observation expires
MAX_REMEMBERED_RANGE_M = 1.5  # ignore observations farther than this
MIN_POSE_CHANGE_M = 0.02      # ignore laser if rover barely moved
MIN_RANGE_M = 0.02            # ignore laser readings below this
DUPLICATE_MERGE_M = 0.04      # merge points closer than this

ROVER_HALF_WIDTH_M = 0.10     # half rover body width for inflation
SAFETY_MARGIN_M = 0.05        # extra inflation beyond half-width

PLANNER_LOOKAHEAD_M = 0.6     # arc simulation distance
WHEELBASE_M = 0.210           # Ackermann wheelbase (matches fused_odometry)

# Polar hazard map
BIN_MIN_DEG = -70.0
BIN_MAX_DEG = 70.0
BIN_SIZE_DEG = 5.0

# Clear corridor half-width (narrow beam + some margin)
CLEAR_CORRIDOR_HALF_DEG = 4.0  # degrees either side of beam center

# Arc candidates (steering angles in degrees)
ARC_CANDIDATES_DEG = [-25, -15, -8, 0, 8, 15, 25]

# Cost weights for arc scoring
COST_COLLISION = 1000.0      # per collision sample
COST_PROXIMITY = 8.0         # scales with 1/distance to obstacle
COST_UNKNOWN = 3.0           # per unknown-sector sample along arc
COST_STEER_CHANGE = 0.5      # per degree of steering change from current
COST_STEER_MAG = 0.1         # per degree of absolute steering magnitude

# Probe behavior
PROBE_ANGLE_DEG = 12.0       # cautious pivot angle in probe state
ARC_BLOCKED_THRESHOLD = 50.0 # total cost above which an arc is "poor"

# Max stored observations
MAX_HAZARD_POINTS = 80
MAX_CLEAR_CORRIDORS = 30


# ── Data Structures ────────────────────────────────────

@dataclass
class HazardPoint:
    """A single obstacle observation in world coordinates."""
    world_x: float
    world_y: float
    timestamp: float         # time.monotonic()
    confidence: float = 1.0
    radius: float = 0.03    # inflation radius of the point itself
    range_m: float = 0.0    # original measured range

    @property
    def age(self) -> float:
        return time.monotonic() - self.timestamp

    def decay_weight(self, ttl: float) -> float:
        """Returns 0.0–1.0 weight based on age; 0 when expired."""
        age = self.age
        if age >= ttl:
            return 0.0
        return self.confidence * (1.0 - age / ttl)


@dataclass
class ClearCorridor:
    """A forward-clear observation: beam heading + range at time of measurement."""
    world_x: float           # rover position when measured
    world_y: float
    heading_rad: float       # rover heading when measured
    clear_range_m: float     # how far ahead was clear
    timestamp: float

    @property
    def age(self) -> float:
        return time.monotonic() - self.timestamp

    def decay_weight(self, ttl: float) -> float:
        age = self.age
        if age >= ttl:
            return 0.0
        return 1.0 - age / ttl


@dataclass
class PolarBin:
    """One angular bin in the forward hazard map."""
    center_deg: float
    hazard_score: float = 0.0
    clear_score: float = 0.0
    observed: bool = False    # True if any recent observation covers this bin

    @property
    def is_unknown(self) -> bool:
        return not self.observed and self.hazard_score < 0.01

    @property
    def net_score(self) -> float:
        """Higher = more dangerous.  Clear reduces hazard."""
        return max(0.0, self.hazard_score - 0.5 * self.clear_score)


@dataclass
class ArcScore:
    """Result of scoring one candidate steering arc."""
    steer_deg: float
    total_cost: float = 0.0
    collision_cost: float = 0.0
    proximity_cost: float = 0.0
    unknown_cost: float = 0.0
    smoothness_cost: float = 0.0
    samples: int = 0


@dataclass
class PlannerDebug:
    """Telemetry snapshot from the latest planning tick."""
    active: bool = False
    obstacle_count: int = 0
    clear_corridor_count: int = 0
    best_arc_deg: float = 0.0
    best_arc_cost: float = 0.0
    all_arcs_blocked: bool = False
    probe_active: bool = False
    blocked_reason: str = ""
    hazard_bins_summary: str = ""
    forward_corridor_confidence: float = 0.0
    arc_scores: List[ArcScore] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "planner_active": self.active,
            "obstacle_count": self.obstacle_count,
            "clear_corridor_count": self.clear_corridor_count,
            "best_arc_deg": round(self.best_arc_deg, 1),
            "best_arc_cost": round(self.best_arc_cost, 1),
            "all_arcs_blocked": self.all_arcs_blocked,
            "probe_active": self.probe_active,
            "blocked_reason": self.blocked_reason,
            "hazard_bins_summary": self.hazard_bins_summary,
            "forward_corridor_confidence": round(self.forward_corridor_confidence, 2),
        }


# ── Obstacle Memory ───────────────────────────────────

class ForwardObstacleMemory:
    """Stores recent obstacle hits and clear corridors in world coordinates.

    Call `record_laser()` each tick with the current odometry pose and laser
    reading.  Call `get_hazard_map()` to build a polar hazard map in body
    frame, and `score_arcs()` to evaluate candidate steering arcs.
    """

    def __init__(
        self,
        ttl_s: float = MEMORY_TTL_S,
        max_range_m: float = MAX_REMEMBERED_RANGE_M,
        min_pose_change_m: float = MIN_POSE_CHANGE_M,
    ):
        self.ttl_s = ttl_s
        self.max_range_m = max_range_m
        self.min_pose_change_m = min_pose_change_m

        self._hazards: deque[HazardPoint] = deque(maxlen=MAX_HAZARD_POINTS)
        self._corridors: deque[ClearCorridor] = deque(maxlen=MAX_CLEAR_CORRIDORS)

        # Last pose where we recorded something (to suppress duplicates)
        self._last_record_x = 0.0
        self._last_record_y = 0.0

        # Latest debug/telemetry snapshot
        self.debug = PlannerDebug()

    # ── Recording ──────────────────────────────────

    def record_laser(
        self,
        robot_x: float, robot_y: float, robot_heading_rad: float,
        laser_range_m: float,
    ) -> None:
        """Record one forward laser measurement from the current pose.

        If laser_range_m is valid and < max_range, store a hazard point.
        Also store the forward corridor as "clear" up to the measured range.
        """
        now = time.monotonic()

        # Ignore tiny pose changes to reduce jitter
        dx = robot_x - self._last_record_x
        dy = robot_y - self._last_record_y
        if (dx * dx + dy * dy) < self.min_pose_change_m ** 2:
            # Still record clear corridor even without movement, but
            # don't store duplicate hazard points
            if laser_range_m >= self.max_range_m or laser_range_m < MIN_RANGE_M:
                # Far/invalid reading → just corridor
                self._record_corridor(robot_x, robot_y, robot_heading_rad,
                                      min(laser_range_m, self.max_range_m), now)
            return

        self._last_record_x = robot_x
        self._last_record_y = robot_y

        # Valid obstacle hit?
        if MIN_RANGE_M < laser_range_m < self.max_range_m:
            # Project into world coords
            hit_x = robot_x + laser_range_m * math.cos(robot_heading_rad)
            hit_y = robot_y + laser_range_m * math.sin(robot_heading_rad)

            # Check for duplicate (merge if very close to existing recent point)
            if not self._has_nearby_recent(hit_x, hit_y, DUPLICATE_MERGE_M, now):
                self._hazards.append(HazardPoint(
                    world_x=hit_x, world_y=hit_y,
                    timestamp=now, confidence=1.0,
                    range_m=laser_range_m,
                ))

            # Corridor is clear UP TO the hit point
            self._record_corridor(robot_x, robot_y, robot_heading_rad,
                                  laser_range_m, now)
        elif laser_range_m >= self.max_range_m:
            # Far reading: no obstacle, corridor clear up to max
            self._record_corridor(robot_x, robot_y, robot_heading_rad,
                                  self.max_range_m, now)

    def _record_corridor(self, rx, ry, heading, clear_m, now):
        if clear_m < MIN_RANGE_M:
            return
        self._corridors.append(ClearCorridor(
            world_x=rx, world_y=ry,
            heading_rad=heading,
            clear_range_m=clear_m,
            timestamp=now,
        ))

    def _has_nearby_recent(self, x, y, radius, now):
        r2 = radius * radius
        for hp in self._hazards:
            if (now - hp.timestamp) > self.ttl_s:
                continue
            if (hp.world_x - x) ** 2 + (hp.world_y - y) ** 2 < r2:
                return True
        return False

    # ── Pruning ────────────────────────────────────

    def _prune(self):
        """Remove expired observations."""
        now = time.monotonic()
        while self._hazards and (now - self._hazards[0].timestamp) > self.ttl_s:
            self._hazards.popleft()
        while self._corridors and (now - self._corridors[0].timestamp) > self.ttl_s:
            self._corridors.popleft()

    # ── Reprojection into body frame ───────────────

    @staticmethod
    def _world_to_body(
        wx: float, wy: float,
        robot_x: float, robot_y: float, robot_heading: float,
    ) -> Tuple[float, float]:
        """Transform a world point into robot body frame (forward=+x, left=+y)."""
        dx = wx - robot_x
        dy = wy - robot_y
        cos_h = math.cos(-robot_heading)
        sin_h = math.sin(-robot_heading)
        bx = dx * cos_h - dy * sin_h
        by = dx * sin_h + dy * cos_h
        return bx, by

    @staticmethod
    def _body_to_polar(bx: float, by: float) -> Tuple[float, float]:
        """Body frame (bx, by) → (range_m, bearing_deg).  0° = forward, left=negative."""
        range_m = math.sqrt(bx * bx + by * by)
        bearing_rad = math.atan2(by, bx)
        return range_m, math.degrees(bearing_rad)

    # ── Polar Hazard Map ───────────────────────────

    def build_hazard_map(
        self,
        robot_x: float, robot_y: float, robot_heading: float,
    ) -> List[PolarBin]:
        """Build a polar hazard map in body coordinates from memory.

        Returns a list of PolarBin objects covering BIN_MIN_DEG..BIN_MAX_DEG.
        """
        self._prune()
        now = time.monotonic()

        # Initialize bins
        n_bins = int((BIN_MAX_DEG - BIN_MIN_DEG) / BIN_SIZE_DEG) + 1
        bins: List[PolarBin] = []
        for i in range(n_bins):
            center = BIN_MIN_DEG + i * BIN_SIZE_DEG
            bins.append(PolarBin(center_deg=center))

        inflation_deg = math.degrees(
            math.atan2(ROVER_HALF_WIDTH_M + SAFETY_MARGIN_M, 0.3)
        )  # inflation at ~0.3m distance reference

        # Score hazard points
        for hp in self._hazards:
            w = hp.decay_weight(self.ttl_s)
            if w <= 0:
                continue
            bx, by = self._world_to_body(
                hp.world_x, hp.world_y, robot_x, robot_y, robot_heading
            )
            if bx < -0.05:
                continue  # behind rover
            rng, bearing = self._body_to_polar(bx, by)
            if rng > self.max_range_m * 1.2:
                continue

            # Inflate hazard: closer obstacles get wider inflation
            if rng > 0.05:
                width_deg = math.degrees(
                    math.atan2(ROVER_HALF_WIDTH_M + SAFETY_MARGIN_M + hp.radius, rng)
                )
            else:
                width_deg = inflation_deg

            # Apply hazard score to affected bins
            score = w * (1.0 + 0.5 / max(rng, 0.1))  # closer = more hazardous
            for b in bins:
                if abs(b.center_deg - bearing) <= width_deg:
                    b.hazard_score += score
                    b.observed = True

        # Score clear corridors (only narrow forward sector)
        for cc in self._corridors:
            w = cc.decay_weight(self.ttl_s)
            if w <= 0:
                continue
            # Where was the rover when this corridor was observed?
            # The corridor was clear from that pose, in the heading at that time.
            # Reproject the corridor endpoint into current body frame
            end_x = cc.world_x + cc.clear_range_m * math.cos(cc.heading_rad)
            end_y = cc.world_y + cc.clear_range_m * math.sin(cc.heading_rad)
            bx_start, by_start = self._world_to_body(
                cc.world_x, cc.world_y, robot_x, robot_y, robot_heading
            )
            bx_end, by_end = self._world_to_body(
                end_x, end_y, robot_x, robot_y, robot_heading
            )

            # Compute bearing range of the corridor in body frame
            _, bearing_start = self._body_to_polar(
                max(bx_start, 0.01), by_start
            )
            _, bearing_end = self._body_to_polar(
                max(bx_end, 0.01), by_end
            )
            # The corridor is narrow: only ±CLEAR_CORRIDOR_HALF_DEG around
            # the midpoint bearing
            mid_bearing = (bearing_start + bearing_end) / 2.0

            for b in bins:
                if abs(b.center_deg - mid_bearing) <= CLEAR_CORRIDOR_HALF_DEG:
                    b.clear_score += w * 0.5
                    b.observed = True

        return bins

    # ── Arc Scoring ────────────────────────────────

    def score_arcs(
        self,
        robot_x: float, robot_y: float, robot_heading: float,
        current_steer_deg: float = 0.0,
        candidates: Optional[List[float]] = None,
        lookahead_m: float = PLANNER_LOOKAHEAD_M,
    ) -> List[ArcScore]:
        """Score candidate steering arcs using the hazard map.

        Each candidate is a steering angle in degrees.
        Returns sorted list (lowest cost first).
        """
        if candidates is None:
            candidates = ARC_CANDIDATES_DEG

        hazard_map = self.build_hazard_map(robot_x, robot_y, robot_heading)

        results = []
        for steer_deg in candidates:
            score = self._score_one_arc(
                steer_deg, hazard_map,
                robot_x, robot_y, robot_heading,
                current_steer_deg, lookahead_m,
            )
            results.append(score)

        results.sort(key=lambda a: a.total_cost)
        return results

    def _score_one_arc(
        self,
        steer_deg: float,
        hazard_map: List[PolarBin],
        robot_x: float, robot_y: float, robot_heading: float,
        current_steer_deg: float,
        lookahead_m: float,
    ) -> ArcScore:
        """Simulate a short arc and accumulate cost from the hazard map."""
        steer_rad = math.radians(steer_deg)
        n_samples = 5
        step_m = lookahead_m / n_samples

        arc = ArcScore(steer_deg=steer_deg, samples=n_samples)

        # Simulate Ackermann kinematics along the arc
        sim_x, sim_y, sim_heading = 0.0, 0.0, 0.0  # body frame
        for i in range(1, n_samples + 1):
            # Ackermann: turning radius = wheelbase / tan(steer)
            if abs(steer_rad) > 0.01:
                turn_radius = WHEELBASE_M / math.tan(steer_rad)
                d_heading = step_m / turn_radius
            else:
                d_heading = 0.0

            sim_heading += d_heading
            sim_x += step_m * math.cos(sim_heading)
            sim_y += step_m * math.sin(sim_heading)

            # Find the bearing/range of this sample point in body frame
            rng, bearing = self._body_to_polar(sim_x, sim_y)

            # Find the matching hazard bin
            binn = self._find_bin(hazard_map, bearing)
            if binn is None:
                # Outside map range — treat as moderate unknown
                arc.unknown_cost += COST_UNKNOWN * 0.5
                continue

            # Collision check
            if binn.net_score > 2.0:
                arc.collision_cost += COST_COLLISION
            elif binn.net_score > 0.5:
                arc.proximity_cost += COST_PROXIMITY * binn.net_score

            # Unknown space penalty
            if binn.is_unknown:
                arc.unknown_cost += COST_UNKNOWN

        # Steering smoothness cost
        arc.smoothness_cost = (
            COST_STEER_CHANGE * abs(steer_deg - current_steer_deg)
            + COST_STEER_MAG * abs(steer_deg)
        )

        arc.total_cost = (
            arc.collision_cost + arc.proximity_cost
            + arc.unknown_cost + arc.smoothness_cost
        )
        return arc

    @staticmethod
    def _find_bin(hazard_map: List[PolarBin], bearing_deg: float) -> Optional[PolarBin]:
        """Find the bin closest to the given bearing."""
        best = None
        best_dist = 999.0
        for b in hazard_map:
            d = abs(b.center_deg - bearing_deg)
            if d < best_dist:
                best_dist = d
                best = b
        if best_dist > BIN_SIZE_DEG:
            return None
        return best

    # ── Planning Decision ──────────────────────────

    def plan(
        self,
        robot_x: float, robot_y: float, robot_heading: float,
        laser_range_m: float,
        current_steer_deg: float = 0.0,
    ) -> PlannerDebug:
        """Full planning tick: record, score arcs, produce decision.

        Returns a PlannerDebug with the result.  The caller reads
        `debug.best_arc_deg` for the recommended steering angle and
        `debug.all_arcs_blocked` to decide whether to enter probe/recover.
        """
        # Record new observation
        self.record_laser(robot_x, robot_y, robot_heading, laser_range_m)

        # Score arcs
        arcs = self.score_arcs(
            robot_x, robot_y, robot_heading,
            current_steer_deg,
        )

        # Build debug info
        self._prune()
        hazard_map = self.build_hazard_map(robot_x, robot_y, robot_heading)

        dbg = PlannerDebug(active=True)
        dbg.obstacle_count = sum(
            1 for h in self._hazards if h.decay_weight(self.ttl_s) > 0
        )
        dbg.clear_corridor_count = sum(
            1 for c in self._corridors if c.decay_weight(self.ttl_s) > 0
        )
        dbg.arc_scores = arcs

        if arcs:
            best = arcs[0]
            dbg.best_arc_deg = best.steer_deg
            dbg.best_arc_cost = best.total_cost
            dbg.all_arcs_blocked = best.total_cost >= ARC_BLOCKED_THRESHOLD
        else:
            dbg.all_arcs_blocked = True

        # Forward corridor confidence (how clear is straight ahead?)
        fwd_bins = [b for b in hazard_map if abs(b.center_deg) <= 10]
        if fwd_bins:
            max_hazard = max(b.net_score for b in fwd_bins)
            any_observed = any(b.observed for b in fwd_bins)
            if any_observed:
                dbg.forward_corridor_confidence = max(0.0, 1.0 - max_hazard / 3.0)
            else:
                dbg.forward_corridor_confidence = 0.0  # unknown ≠ clear
        else:
            dbg.forward_corridor_confidence = 0.0

        # Hazard bins summary (compact string for telemetry)
        dbg.hazard_bins_summary = self._bins_summary(hazard_map)

        if dbg.all_arcs_blocked:
            if arcs and arcs[0].collision_cost > 0:
                dbg.blocked_reason = "collision"
            elif arcs and arcs[0].unknown_cost > arcs[0].proximity_cost:
                dbg.blocked_reason = "unknown_space"
            else:
                dbg.blocked_reason = "high_proximity"

        self.debug = dbg
        return dbg

    @staticmethod
    def _bins_summary(hazard_map: List[PolarBin]) -> str:
        """Compact hazard map: one char per bin.  ·=clear ░=low ▓=high █=blocked ?=unknown."""
        chars = []
        for b in hazard_map:
            ns = b.net_score
            if b.is_unknown:
                chars.append('?')
            elif ns < 0.3:
                chars.append('·')
            elif ns < 1.5:
                chars.append('░')
            elif ns < 3.0:
                chars.append('▓')
            else:
                chars.append('█')
        return ''.join(chars)

    # ── Accessors ──────────────────────────────────

    def get_obstacle_count(self) -> int:
        self._prune()
        return sum(1 for h in self._hazards if h.decay_weight(self.ttl_s) > 0)

    def get_clear_corridor_count(self) -> int:
        self._prune()
        return sum(1 for c in self._corridors if c.decay_weight(self.ttl_s) > 0)

    def clear(self):
        """Wipe all stored observations."""
        self._hazards.clear()
        self._corridors.clear()
        self._last_record_x = 0.0
        self._last_record_y = 0.0
        self.debug = PlannerDebug()
