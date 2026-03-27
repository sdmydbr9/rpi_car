import unittest
from collections import deque
import threading

import numpy as np

from scripts.core.follow_line import CAMERA_H, CAMERA_W, LineFollower, LINE_CONFIRM_FRAMES
from scripts.core.wheel_sync import WheelSpeedController


class CurveGeometryTests(unittest.TestCase):
    def _make_follower(self, speed_mps=0.24):
        follower = LineFollower.__new__(LineFollower)
        follower._forward_speed_mps = speed_mps
        return follower

    def test_curve_geometry_returns_prediction_term(self):
        follower = self._make_follower(0.26)
        spine = []
        for i in range(8):
            y = CAMERA_H - 45 - i * 22
            x = (CAMERA_W / 2.0) + 6.0 + 2.2 * i + 0.9 * (i ** 2)
            spine.append((x, y))

        geom = follower._estimate_curve_geometry(spine, CAMERA_W, CAMERA_H)

        self.assertIsNotNone(geom)
        self.assertIn("predicted_x", geom)
        self.assertGreater(geom["predicted_x"], 0.0)
        self.assertNotAlmostEqual(geom["predicted_x"], geom["mid_x"], places=3)
        self.assertGreaterEqual(geom["confidence"], 0.1)


class WheelSyncTargetRpmTests(unittest.TestCase):
    def test_explicit_target_rpm_path_respects_gear_cap_and_ratio(self):
        rpm_state = {
            "rear_left": 110.0,
            "rear_right": 95.0,
            "front_right": 92.0,
        }
        ctrl = WheelSpeedController(
            get_rpm_fn=lambda: rpm_state,
            get_duty_cap_fn=lambda: 63.0,
            get_freshness_fn=lambda: True,
        )
        ctrl.gear = "1"

        out = ctrl.correct_target_rpms(
            360.0,
            180.0,
            speed_l_hint=30.0,
            speed_r_hint=15.0,
            duty_cap=63.0,
        )

        self.assertEqual(len(out), 4)
        telem = ctrl.get_telemetry()
        self.assertAlmostEqual(
            telem["wheels"]["rl"]["target_rpm"],
            ctrl.gear_max_rpm,
            delta=0.5,
        )
        self.assertAlmostEqual(
            telem["wheels"]["rr"]["target_rpm"],
            ctrl.gear_max_rpm / 2.0,
            delta=0.5,
        )

    def test_rr_gets_max_pwm_when_encoder_is_stuck_but_right_side_should_move(self):
        rpm_state = {
            "rear_left": 0.0,
            "rear_right": 0.0,
            "front_right": 24.0,
        }
        ctrl = WheelSpeedController(
            get_rpm_fn=lambda: rpm_state,
            get_duty_cap_fn=lambda: 63.0,
            get_freshness_fn=lambda: True,
        )
        ctrl.gear = "1"

        fl_pwm, fr_pwm, rl_pwm, rr_pwm = ctrl.correct_target_rpms(
            0.0,
            90.0,
            speed_l_hint=0.0,
            speed_r_hint=1.0,
            duty_cap=63.0,
        )

        self.assertEqual(rr_pwm, 63.0)
        self.assertGreater(fr_pwm, 0.0)
        self.assertEqual(rl_pwm, 0.0)
        self.assertEqual(fl_pwm, 0.0)


class FollowLineCompatibilityTests(unittest.TestCase):
    def test_compute_target_rpms_prefers_wheel_sync_pwm_to_rpm_scale(self):
        follower = LineFollower.__new__(LineFollower)

        class _PowerLimiter:
            max_safe_duty = 80.0

        class _WheelSync:
            gear_max_rpm = 120.0
            pwm_to_rpm = 15.0

        class _Car:
            power_limiter = _PowerLimiter()
            wheel_sync = _WheelSync()

            def get_target_rpm_cap(self):
                return 120.0

        follower._car = _Car()

        left_rpm, right_rpm = follower._compute_target_rpms(40.0, 20.0)

        self.assertAlmostEqual(left_rpm, 600.0)
        self.assertAlmostEqual(right_rpm, 300.0)

    def test_compute_target_rpms_falls_back_to_wheel_sync_cap(self):
        follower = LineFollower.__new__(LineFollower)

        class _PowerLimiter:
            max_safe_duty = 80.0

        class _WheelSync:
            gear_max_rpm = 120.0

        class _LegacyCar:
            power_limiter = _PowerLimiter()
            wheel_sync = _WheelSync()

        follower._car = _LegacyCar()

        left_rpm, right_rpm = follower._compute_target_rpms(40.0, 20.0)

        self.assertAlmostEqual(left_rpm, 60.0)
        self.assertAlmostEqual(right_rpm, 30.0)

    def test_reuse_last_drive_command_true_skip_makes_no_motor_call(self):
        follower = LineFollower.__new__(LineFollower)
        follower._last_drive_command = {
            "left_speed": 32.0,
            "right_speed": 28.0,
            "target_rpm_left": 90.0,
            "target_rpm_right": 84.0,
        }
        follower._command_target_rpm_left = 0.0
        follower._command_target_rpm_right = 0.0
        follower._drive_command_active = False

        called = {"count": 0}

        def _unexpected_issue(*args, **kwargs):
            called["count"] += 1

        follower._issue_drive_command = _unexpected_issue

        reused = follower._reuse_last_drive_command()

        self.assertTrue(reused)
        self.assertEqual(called["count"], 0)
        self.assertTrue(follower._drive_command_active)
        self.assertEqual(follower._command_target_rpm_left, 90.0)
        self.assertEqual(follower._command_target_rpm_right, 84.0)


class FollowLineStateTests(unittest.TestCase):
    def _make_state_follower(self):
        follower = LineFollower.__new__(LineFollower)
        follower._lock = threading.Lock()
        follower._line_cx = 0.0
        follower._line_cy = 0.0
        follower._detection_mode = "full"
        follower._last_template_score = 0.0
        follower._recent_template_scores = deque(maxlen=5)
        follower._template_confidence = 0.0
        follower._confirm_count = 0
        follower._ready_miss_count = 0
        follower._line_lost_since = 0.0
        follower._recovery_brake_frames = 0
        follower._recovery_phase = "idle"
        follower._following = False
        follower.state = LineFollower.SEARCHING
        follower._locked_cx = None
        follower._locked_colour = None
        follower._locked_signature = None
        follower._line_memory = None
        follower._path_history = deque(maxlen=16)
        follower._integral = 0.0
        follower._prev_filtered_error = 0.0
        follower._filtered_error = 0.0
        follower._filtered_lookahead = 0.0
        follower._filtered_steer = 0.0
        follower._last_control_ts = 0.0
        follower._last_steer = 0.0
        follower._vision_steer = 0.0
        follower._curve_heading_deg = 0.0
        follower._curve_curvature = 0.0
        follower._predicted_line_cx = CAMERA_W / 2.0
        follower._feedforward_steer = 0.0
        follower._target_yaw_rate_dps = 0.0
        follower._target_heading = None
        follower._last_target_ts = 0.0
        follower._active_cornering = False
        follower._control_cycle_index = 0
        follower._last_drive_command = None
        follower._command_target_rpm_left = 0.0
        follower._command_target_rpm_right = 0.0
        follower._drive_command_active = False
        follower._car = type("CarStub", (), {"brake": lambda self: None})()
        return follower

    def test_line_detected_state_has_miss_hysteresis(self):
        follower = self._make_state_follower()
        mean_colour = np.array([10.0, 20.0, 30.0], dtype=np.float32)
        signature = {
            "bgr": np.array([10.0, 20.0, 30.0], dtype=np.float32),
            "hsv": np.array([5.0, 50.0, 120.0], dtype=np.float32),
            "short_side": 12.0,
            "row_hits": 6.0,
            "elongation": 4.0,
        }

        for _ in range(LINE_CONFIRM_FRAMES):
            state = follower._update_detection_state(
                True, 320.0, 240.0, mean_colour, signature, 1.0, "full"
            )
        self.assertEqual(state, LineFollower.LINE_DETECTED)

        state = follower._update_detection_state(
            False, 0.0, 0.0, None, None, 0.0, "full"
        )
        self.assertEqual(state, LineFollower.LINE_DETECTED)

        state = follower._update_detection_state(
            False, 0.0, 0.0, None, None, 0.0, "full"
        )
        self.assertEqual(state, LineFollower.LINE_DETECTED)

        state = follower._update_detection_state(
            False, 0.0, 0.0, None, None, 0.0, "full"
        )
        self.assertEqual(state, LineFollower.SEARCHING)


if __name__ == "__main__":
    unittest.main()
