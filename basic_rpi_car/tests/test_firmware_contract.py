from pathlib import Path
import unittest


FIRMWARE = Path(__file__).resolve().parents[1] / "pico_firmware.py"


class FirmwareContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = FIRMWARE.read_text(encoding="utf-8")

    def test_uart_and_all_hardware_pins_are_preserved(self):
        expected_fragments = [
            "UART_BAUD = 115200",
            "tx=Pin(0)",
            "rx=Pin(1)",
            "steer_servo = PWM(Pin(15))",
            "motor_left_pwm = PWM(Pin(10))",
            "motor_left_in1 = Pin(17, Pin.OUT)",
            "motor_left_in2 = Pin(12, Pin.OUT)",
            "motor_right_pwm = PWM(Pin(16))",
            "motor_right_in1 = Pin(13, Pin.OUT)",
            "motor_right_in2 = Pin(14, Pin.OUT)",
            "scl=Pin(9)",
            "sda=Pin(8)",
            "ADS1115_ADDRESS = 0x48",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

    def test_motor_and_safety_constants_are_preserved(self):
        expected_fragments = [
            "MAX_PWM_DUTY = 95",
            "WATCHDOG_TIMEOUT_MS = 300",
            "MOTOR_TICK_MS = 10",
            "MAX_PWM_DELTA_PER_TICK = 3",
            "DIRECTION_CHANGE_DWELL_MS = 150",
            "motor_left_pwm.freq(1000)",
            "motor_right_pwm.freq(1000)",
            "steer_servo.freq(50)",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

    def test_full_command_protocol_is_supported(self):
        expected_branches = [
            'if line.startswith("D:")',
            'elif line == "B"',
            'elif line == "S"',
            'elif line == "E"',
            'elif line == "ERST"',
            'elif line.startswith("SC:")',
            'elif line == "PING"',
            'elif line == "BAT?"',
        ]
        for branch in expected_branches:
            with self.subTest(branch=branch):
                self.assertIn(branch, self.source)

    def test_output_is_only_sent_for_explicit_diagnostic_queries(self):
        uart_writes = [
            line for line in self.source.splitlines() if "uart.write(" in line
        ]
        self.assertEqual(len(uart_writes), 4)
        self.assertNotIn("print(", self.source)
        self.assertNotIn("telemetry", self.source.lower())

    def test_pwm_is_always_clamped_to_the_95_percent_cap(self):
        self.assertIn(
            "clamp(duty, 0, MAX_PWM_DUTY) / 100 * 65535",
            self.source,
        )
        self.assertNotIn("set_left_pwm(100)", self.source)
        self.assertNotIn("set_right_pwm(100)", self.source)


if __name__ == "__main__":
    unittest.main()
