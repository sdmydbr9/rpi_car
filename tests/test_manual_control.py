import os
import sys
import unittest


CORE_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts", "core")
sys.path.insert(0, os.path.abspath(CORE_DIR))

from manual_control import ControlRejected, DriveInput, ManualControlArbiter
from pico_control import (
    PicoController,
    format_drive_command,
    throttle_to_pwm,
)
from steering_calibration import default_steering_calibration


class FakePico:
    def __init__(self):
        self.connected = True
        self.last_error = ""
        self.commands = []
        self.battery = 11.25
        self.send_ok = True

    def connect(self):
        self.connected = True
        return True

    def drive(self, direction, throttle, steering):
        self.commands.append(("drive", direction, throttle, steering))
        return self.send_ok

    def brake(self):
        self.commands.append(("brake",))
        return self.send_ok

    def stop(self):
        self.commands.append(("stop",))
        return True

    def emergency_stop(self):
        self.commands.append(("estop",))
        return True

    def reset_emergency_stop(self):
        self.commands.append(("estop_reset",))
        return True

    def read_battery_voltage(self, _divider_ratio):
        self.commands.append(("battery",))
        return self.battery


class FakeSerial:
    def __init__(self, *_args, **_kwargs):
        self.is_open = True
        self.writes = []
        self.responses = []

    def write(self, payload):
        self.writes.append(payload)

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


class ProtocolTests(unittest.TestCase):
    def test_throttle_is_clamped_and_mapped_to_95_percent(self):
        self.assertEqual(throttle_to_pwm(100), 95)
        self.assertEqual(throttle_to_pwm(200), 95)
        self.assertEqual(throttle_to_pwm(-10), 0)

    def test_drive_packet_contains_direction_pwm_and_calibrated_pulse(self):
        packet = format_drive_command(
            "F", 100, 0, default_steering_calibration()
        )
        self.assertEqual(packet, b"D:F,95,1440\n")

    def test_drive_packet_clamps_steering_and_neutral_throttle(self):
        calibration = default_steering_calibration()
        self.assertEqual(
            format_drive_command("F", 200, 999, calibration),
            b"D:F,95,2150\n",
        )
        self.assertEqual(
            format_drive_command("N", 80, -999, calibration),
            b"D:N,0,940\n",
        )

    def test_battery_is_only_read_after_explicit_request(self):
        serial_instance = FakeSerial()
        serial_instance.responses.append(b"BAT:2250.0\n")
        controller = PicoController(
            serial_factory=lambda *_args, **_kwargs: serial_instance
        )
        serial_instance.writes.clear()
        voltage = controller.read_battery_voltage(divider_ratio=5.0)
        self.assertEqual(voltage, 11.25)
        self.assertEqual(serial_instance.writes, [b"BAT?\n"])


class ArbitrationTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.0]
        self.pico = FakePico()
        self.arbiter = ManualControlArbiter(
            self.pico, now_fn=lambda: self.clock[0]
        )

    def claim_and_arm(self):
        self.arbiter.claim_remote("phone")
        self.arbiter.arm("remote", "phone")

    def test_remote_owner_is_exclusive(self):
        self.arbiter.claim_remote("phone")
        with self.assertRaises(ControlRejected):
            self.arbiter.claim_remote("browser")

    def test_gamepad_connection_revokes_remote_owner(self):
        self.claim_and_arm()
        self.arbiter.set_gamepad_connected(True)
        self.assertFalse(self.arbiter.armed)
        self.assertIsNone(self.arbiter.remote_owner)
        with self.assertRaises(ControlRejected):
            self.arbiter.claim_remote("phone")

    def test_gamepad_disconnect_stops_and_disarms(self):
        self.arbiter.set_gamepad_connected(True)
        self.arbiter.arm("gamepad")
        self.arbiter.accept_drive(
            "gamepad", DriveInput(1, "F", 20, 0, False)
        )
        self.arbiter.set_gamepad_connected(False)
        self.assertFalse(self.arbiter.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_moving_remote_times_out_after_300_ms(self):
        self.claim_and_arm()
        self.arbiter.accept_drive(
            "remote", DriveInput(1, "F", 40, 0, False), "phone"
        )
        self.clock[0] += 0.301
        self.arbiter.tick()
        self.assertFalse(self.arbiter.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_stationary_armed_remote_does_not_need_a_stream(self):
        self.claim_and_arm()
        self.clock[0] += 10
        self.arbiter.tick()
        self.assertTrue(self.arbiter.armed)

    def test_active_brake_is_refreshed_then_times_out(self):
        self.claim_and_arm()
        self.arbiter.accept_drive(
            "remote", DriveInput(1, "N", 0, 0, True), "phone"
        )
        self.clock[0] += 0.11
        self.arbiter.tick()
        self.assertEqual(self.pico.commands[-1], ("brake",))
        self.clock[0] += 0.20
        self.arbiter.tick()
        self.assertFalse(self.arbiter.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_uart_write_failure_disarms(self):
        self.claim_and_arm()
        self.pico.send_ok = False
        with self.assertRaises(ControlRejected):
            self.arbiter.accept_drive(
                "remote", DriveInput(1, "F", 20, 0, False), "phone"
            )
        self.assertFalse(self.arbiter.armed)

    def test_direction_reversal_passes_through_neutral(self):
        self.claim_and_arm()
        self.arbiter.accept_drive(
            "remote", DriveInput(1, "F", 30, 0, False), "phone"
        )
        with self.assertRaises(ControlRejected):
            self.arbiter.accept_drive(
                "remote", DriveInput(2, "R", 30, 0, False), "phone"
            )
        self.assertEqual(self.pico.commands[-1], ("stop",))
        self.arbiter.accept_drive(
            "remote", DriveInput(3, "R", 0, 0, False), "phone"
        )
        self.arbiter.accept_drive(
            "remote", DriveInput(4, "R", 30, 0, False), "phone"
        )
        self.assertEqual(self.pico.commands[-1], ("drive", "R", 30, 0))

    def test_emergency_stop_latches_and_blocks_arming(self):
        self.arbiter.claim_remote("phone")
        self.arbiter.emergency_stop()
        with self.assertRaises(ControlRejected):
            self.arbiter.arm("remote", "phone")
        with self.assertRaises(ControlRejected):
            self.arbiter.reset_emergency_stop("remote", "phone")
        self.clock[0] += 0.20
        self.arbiter.reset_emergency_stop("remote", "phone")
        self.arbiter.arm("remote", "phone")
        self.assertTrue(self.arbiter.armed)

    def test_battery_read_is_rejected_while_armed(self):
        self.claim_and_arm()
        with self.assertRaises(ControlRejected):
            self.arbiter.read_battery(5.0)


if __name__ == "__main__":
    unittest.main()
