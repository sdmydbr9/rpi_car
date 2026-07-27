from pathlib import Path
import unittest


FIRMWARE = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "firmware"
    / "pico_manual_controller.py"
)


class FirmwareContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = FIRMWARE.read_text(encoding="utf-8")

    def test_watchdog_and_direction_dwell_constants(self):
        self.assertIn("WATCHDOG_TIMEOUT_MS = 300", self.source)
        self.assertIn("DIRECTION_CHANGE_DWELL_MS = 150", self.source)
        self.assertIn(
            "time.ticks_diff(now_ms, _direction_dwell_started_ms)",
            self.source,
        )

    def test_motor_ramp_runs_at_100_hz_with_95_percent_cap(self):
        self.assertIn("MAX_PWM_DUTY = 95", self.source)
        self.assertIn("MOTOR_TICK_MS = 10", self.source)
        self.assertNotIn("set_left_pwm(100)", self.source)
        self.assertNotIn("set_right_pwm(100)", self.source)

    def test_uart_output_is_limited_to_explicit_query_branches(self):
        response_lines = [
            line.strip()
            for line in self.source.splitlines()
            if "uart.write(" in line
        ]
        self.assertEqual(len(response_lines), 4)
        self.assertIn('elif line == "PING":', self.source)
        self.assertIn('elif line == "BAT?":', self.source)
        self.assertNotIn("telemetry", self.source.lower())


if __name__ == "__main__":
    unittest.main()
