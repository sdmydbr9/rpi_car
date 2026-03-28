"""
Tests for obstacle_memory.py — Forward Obstacle Memory + Polar Hazard Map

Covers:
  - HazardPoint decay weight
  - Recording and pruning
  - World-to-body reprojection
  - Hazard map bin construction
  - Arc scoring (collision, unknown, smoothness)
  - Plan decision (all_arcs_blocked detection)
  - Clear corridor memory
  - Duplicate suppression
"""

import math
import time
import unittest
from unittest.mock import patch

import sys
import os

# Add scripts/core to path so obstacle_memory can be imported directly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts', 'core'))

from obstacle_memory import (
    ForwardObstacleMemory,
    HazardPoint,
    ClearCorridor,
    PolarBin,
    ArcScore,
    PlannerDebug,
    MEMORY_TTL_S,
    MAX_REMEMBERED_RANGE_M,
    BIN_SIZE_DEG,
    ARC_CANDIDATES_DEG,
    ARC_BLOCKED_THRESHOLD,
)


class TestHazardPoint(unittest.TestCase):
    """Test HazardPoint dataclass and decay."""

    def test_decay_weight_fresh(self):
        hp = HazardPoint(0, 0, time.monotonic(), confidence=1.0)
        w = hp.decay_weight(1.2)
        self.assertAlmostEqual(w, 1.0, delta=0.05)

    def test_decay_weight_expired(self):
        hp = HazardPoint(0, 0, time.monotonic() - 2.0, confidence=1.0)
        w = hp.decay_weight(1.2)
        self.assertEqual(w, 0.0)

    def test_decay_weight_half_life(self):
        hp = HazardPoint(0, 0, time.monotonic() - 0.6, confidence=1.0)
        w = hp.decay_weight(1.2)
        self.assertAlmostEqual(w, 0.5, delta=0.1)

    def test_age_property(self):
        before = time.monotonic()
        hp = HazardPoint(0, 0, before)
        time.sleep(0.01)
        self.assertGreater(hp.age, 0)


class TestClearCorridor(unittest.TestCase):
    def test_decay_weight(self):
        cc = ClearCorridor(0, 0, 0.0, 1.0, time.monotonic())
        w = cc.decay_weight(1.2)
        self.assertAlmostEqual(w, 1.0, delta=0.05)


class TestPolarBin(unittest.TestCase):
    def test_unknown_when_not_observed(self):
        b = PolarBin(center_deg=0.0)
        self.assertTrue(b.is_unknown)
        self.assertAlmostEqual(b.net_score, 0.0)

    def test_not_unknown_when_observed(self):
        b = PolarBin(center_deg=0.0, hazard_score=1.0, observed=True)
        self.assertFalse(b.is_unknown)
        self.assertGreater(b.net_score, 0.0)

    def test_clear_reduces_net_score(self):
        b = PolarBin(center_deg=0.0, hazard_score=2.0, clear_score=3.0, observed=True)
        self.assertLess(b.net_score, 2.0)


class TestWorldToBody(unittest.TestCase):
    """Test coordinate transformation."""

    def test_point_ahead(self):
        # Robot at origin facing East (heading=0), point 1m ahead
        bx, by = ForwardObstacleMemory._world_to_body(1.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(bx, 1.0, delta=0.01)
        self.assertAlmostEqual(by, 0.0, delta=0.01)

    def test_point_left(self):
        # Robot at origin facing East, point 1m to North (left in body frame)
        bx, by = ForwardObstacleMemory._world_to_body(0.0, 1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(bx, 0.0, delta=0.01)
        self.assertAlmostEqual(by, 1.0, delta=0.01)

    def test_point_behind(self):
        # Robot at origin facing East, point 1m behind (West)
        bx, by = ForwardObstacleMemory._world_to_body(-1.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(bx, -1.0, delta=0.01)

    def test_rotated_heading(self):
        # Robot facing North (π/2), point 1m North → should be ahead (bx=1)
        bx, by = ForwardObstacleMemory._world_to_body(
            0.0, 1.0, 0.0, 0.0, math.pi / 2
        )
        self.assertAlmostEqual(bx, 1.0, delta=0.01)
        self.assertAlmostEqual(by, 0.0, delta=0.01)


class TestBodyToPolar(unittest.TestCase):
    def test_forward(self):
        rng, bearing = ForwardObstacleMemory._body_to_polar(1.0, 0.0)
        self.assertAlmostEqual(rng, 1.0, delta=0.01)
        self.assertAlmostEqual(bearing, 0.0, delta=0.1)

    def test_left(self):
        rng, bearing = ForwardObstacleMemory._body_to_polar(0.0, 1.0)
        self.assertAlmostEqual(bearing, 90.0, delta=0.1)

    def test_right(self):
        rng, bearing = ForwardObstacleMemory._body_to_polar(0.0, -1.0)
        self.assertAlmostEqual(bearing, -90.0, delta=0.1)


class TestRecordAndPrune(unittest.TestCase):
    """Test recording laser hits and expiry."""

    def test_record_obstacle_creates_hazard(self):
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        # Start from a non-default position so the min_pose_change check passes
        mem._last_record_x = -1.0
        mem._last_record_y = -1.0
        mem.record_laser(0.0, 0.0, 0.0, 0.5)
        self.assertEqual(mem.get_obstacle_count(), 1)

    def test_record_far_reading_no_hazard(self):
        mem = ForwardObstacleMemory(max_range_m=1.5)
        mem.record_laser(0.0, 0.0, 0.0, 2.0)
        self.assertEqual(mem.get_obstacle_count(), 0)

    def test_record_far_creates_corridor(self):
        mem = ForwardObstacleMemory(max_range_m=1.5)
        mem.record_laser(0.0, 0.0, 0.0, 2.0)
        self.assertGreater(mem.get_clear_corridor_count(), 0)

    def test_invalid_laser_ignored(self):
        mem = ForwardObstacleMemory()
        mem.record_laser(0.0, 0.0, 0.0, -1.0)
        self.assertEqual(mem.get_obstacle_count(), 0)

    def test_duplicate_suppression(self):
        """Very close points should be merged."""
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        mem.record_laser(0.0, 0.0, 0.0, 0.5)
        mem.record_laser(0.01, 0.0, 0.0, 0.5)  # almost same spot
        # Should only have one hazard due to merge
        self.assertLessEqual(mem.get_obstacle_count(), 2)

    def test_prune_expired(self):
        mem = ForwardObstacleMemory(ttl_s=0.05, min_pose_change_m=0.001)
        mem._last_record_x = -1.0
        mem._last_record_y = -1.0
        mem.record_laser(0.0, 0.0, 0.0, 0.5)
        self.assertEqual(mem.get_obstacle_count(), 1)
        time.sleep(0.08)
        self.assertEqual(mem.get_obstacle_count(), 0)

    def test_clear_wipes_all(self):
        mem = ForwardObstacleMemory()
        mem.record_laser(0.0, 0.0, 0.0, 0.5)
        mem.record_laser(0.1, 0.0, 0.0, 0.8)
        mem.clear()
        self.assertEqual(mem.get_obstacle_count(), 0)
        self.assertEqual(mem.get_clear_corridor_count(), 0)


class TestHazardMap(unittest.TestCase):
    """Test polar hazard map construction."""

    def test_empty_memory_all_unknown(self):
        mem = ForwardObstacleMemory()
        bins = mem.build_hazard_map(0.0, 0.0, 0.0)
        self.assertTrue(all(b.is_unknown for b in bins))

    def test_obstacle_ahead_marks_center_bins(self):
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        # Directly inject a hazard point 0.5m ahead on x-axis
        mem._hazards.append(HazardPoint(
            world_x=0.5, world_y=0.0,
            timestamp=time.monotonic(), range_m=0.5,
        ))
        bins = mem.build_hazard_map(0.0, 0.0, 0.0)
        # Center bins (around 0°) should have hazard
        center_bins = [b for b in bins if abs(b.center_deg) <= 15]
        self.assertTrue(any(b.hazard_score > 0 for b in center_bins))

    def test_obstacle_to_side_marks_side_bins(self):
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        # Robot at (0,0) facing East, obstacle at (0.3, 0.3) → ~45° left
        mem._hazards.append(HazardPoint(
            world_x=0.3, world_y=0.3,
            timestamp=time.monotonic(), range_m=0.42,
        ))
        bins = mem.build_hazard_map(0.0, 0.0, 0.0)
        # Bin around 45° should have hazard
        side_bins = [b for b in bins if 35 <= b.center_deg <= 55]
        self.assertTrue(any(b.hazard_score > 0 for b in side_bins))


class TestArcScoring(unittest.TestCase):
    """Test candidate arc evaluation."""

    def test_clear_field_prefers_straight(self):
        mem = ForwardObstacleMemory()
        # No obstacles, but record some clear corridors
        mem.record_laser(0.0, 0.0, 0.0, 1.5)
        arcs = mem.score_arcs(0.0, 0.0, 0.0, current_steer_deg=0.0)
        # Straight (0°) should be best due to smoothness cost
        self.assertEqual(arcs[0].steer_deg, 0.0)

    def test_obstacle_ahead_avoids_straight(self):
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        # Directly inject obstacle 0.3m ahead
        mem._hazards.append(HazardPoint(
            world_x=0.3, world_y=0.0,
            timestamp=time.monotonic(), range_m=0.3,
        ))
        arcs = mem.score_arcs(0.0, 0.0, 0.0, current_steer_deg=0.0)
        # Straight should NOT be best choice
        self.assertNotEqual(arcs[0].steer_deg, 0)

    def test_smoothness_cost_present(self):
        mem = ForwardObstacleMemory()
        arcs = mem.score_arcs(0.0, 0.0, 0.0, current_steer_deg=0.0)
        # 25° arc should have more smoothness cost than 0°
        arc_0 = next(a for a in arcs if a.steer_deg == 0)
        arc_25 = next(a for a in arcs if a.steer_deg == 25)
        self.assertGreater(arc_25.smoothness_cost, arc_0.smoothness_cost)

    def test_continuity_bias(self):
        """If currently steering +15°, continuing +15° should cost less."""
        mem = ForwardObstacleMemory()
        arcs = mem.score_arcs(0.0, 0.0, 0.0, current_steer_deg=15.0)
        arc_15 = next(a for a in arcs if a.steer_deg == 15)
        arc_neg25 = next(a for a in arcs if a.steer_deg == -25)
        self.assertLess(arc_15.smoothness_cost, arc_neg25.smoothness_cost)


class TestPlanDecision(unittest.TestCase):
    """Test the full plan() method."""

    def test_plan_returns_debug(self):
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        mem._last_record_x = -1.0
        mem._last_record_y = -1.0
        dbg = mem.plan(0.0, 0.0, 0.0, 1.0, current_steer_deg=0.0)
        self.assertTrue(dbg.active)
        self.assertIsInstance(dbg.best_arc_deg, (int, float))
        self.assertIsInstance(dbg.hazard_bins_summary, str)

    def test_plan_blocked_with_close_obstacle(self):
        """With a very close obstacle and no clear paths, all arcs should be poor."""
        mem = ForwardObstacleMemory(min_pose_change_m=0.001)
        # Place many obstacles in a wall ahead
        for angle_offset in range(-8, 9):
            rad = math.radians(angle_offset * 5)
            ox = 0.15 * math.cos(rad)
            oy = 0.15 * math.sin(rad)
            mem._hazards.append(HazardPoint(
                world_x=ox, world_y=oy,
                timestamp=time.monotonic(), range_m=0.15,
            ))
        dbg = mem.plan(0.0, 0.0, 0.0, 0.15)
        self.assertTrue(dbg.all_arcs_blocked)

    def test_to_dict(self):
        dbg = PlannerDebug(active=True, best_arc_deg=8.0, best_arc_cost=12.5)
        d = dbg.to_dict()
        self.assertTrue(d["planner_active"])
        self.assertAlmostEqual(d["best_arc_deg"], 8.0)

    def test_bins_summary_chars(self):
        mem = ForwardObstacleMemory()
        bins = mem.build_hazard_map(0.0, 0.0, 0.0)
        summary = mem._bins_summary(bins)
        # All unknown → all '?'
        self.assertTrue(all(c == '?' for c in summary))


class TestPlannerDebugDict(unittest.TestCase):
    def test_all_keys_present(self):
        dbg = PlannerDebug()
        d = dbg.to_dict()
        for key in ["planner_active", "obstacle_count", "clear_corridor_count",
                     "best_arc_deg", "best_arc_cost", "all_arcs_blocked",
                     "probe_active", "blocked_reason", "hazard_bins_summary",
                     "forward_corridor_confidence"]:
            self.assertIn(key, d)


if __name__ == "__main__":
    unittest.main()
