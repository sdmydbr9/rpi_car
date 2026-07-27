from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import hardware_test


class FakePico:
    def __init__(self):
        self.last_error = ""
        self.commands: list[tuple] = []

    def drive(self, direction, throttle, steering):
        self.commands.append(("drive", direction, throttle, steering))
        return True

    def stop(self):
        self.commands.append(("stop",))
        return True

    def reset_emergency_stop(self):
        self.commands.append(("estop_reset",))
        return True


class HardwareTestHelpers(unittest.TestCase):
    def setUp(self):
        self.pico = FakePico()

    def test_confirmation_must_match_exact_safety_word(self):
        self.assertTrue(
            hardware_test.require_wheels_lifted(lambda _prompt: "LIFTED")
        )
        self.assertFalse(
            hardware_test.require_wheels_lifted(lambda _prompt: "lifted")
        )

    def test_stale_estop_is_cleared_before_movement(self):
        hardware_test.clear_stale_estop(
            self.pico,
            sleep_fn=lambda _duration: None,
        )
        self.assertEqual(
            self.pico.commands,
            [
                ("stop",),
                ("estop_reset",),
                ("drive", "N", 0, 0),
            ],
        )

    def test_servo_sequence_keeps_motors_neutral(self):
        hardware_test.run_servo_test(
            self.pico,
            steering_angle=30,
            hold_seconds=0,
            sleep_fn=lambda _duration: None,
        )
        self.assertEqual(
            self.pico.commands,
            [
                ("drive", "N", 0, 0),
                ("drive", "N", 0, -30),
                ("drive", "N", 0, 0),
                ("drive", "N", 0, 30),
                ("drive", "N", 0, 0),
            ],
        )

    def test_motor_sequence_runs_both_directions_and_stops(self):
        hardware_test.run_motor_test(
            self.pico,
            throttle=15,
            run_seconds=0,
            sleep_fn=lambda _duration: None,
        )
        self.assertEqual(
            self.pico.commands,
            [
                ("drive", "F", 15, 0),
                ("stop",),
                ("drive", "R", 15, 0),
                ("stop",),
            ],
        )

    def test_cleanup_stops_and_centers(self):
        hardware_test.safe_stop_and_center(self.pico)
        self.assertEqual(
            self.pico.commands,
            [("stop",), ("drive", "N", 0, 0)],
        )


if __name__ == "__main__":
    unittest.main()
