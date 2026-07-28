from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import main as car


class FakeSerial:
    def __init__(self, *_args, **_kwargs):
        self.is_open = True
        self.writes: list[bytes] = []
        self.responses: list[bytes] = []

    def write(self, payload):
        self.writes.append(payload)

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


class FakePico:
    def __init__(self):
        self.connected = True
        self.port = "/dev/fake"
        self.last_error = ""
        self.commands: list[tuple] = []
        self.send_ok = True
        self.closed = False

    def connect(self):
        self.commands.append(("connect",))
        return self.connected

    def drive(self, direction, throttle, steering):
        self.commands.append(("drive", direction, throttle, steering))
        if not self.send_ok:
            self.last_error = "simulated UART failure"
        return self.send_ok

    def brake(self):
        self.commands.append(("brake",))
        if not self.send_ok:
            self.last_error = "simulated UART failure"
        return self.send_ok

    def stop(self):
        self.commands.append(("stop",))
        if not self.send_ok:
            self.last_error = "simulated UART failure"
        return self.send_ok

    def emergency_stop(self):
        self.commands.append(("estop",))
        return self.send_ok

    def reset_emergency_stop(self):
        self.commands.append(("estop_reset",))
        return self.send_ok

    def close(self):
        self.closed = True


class FakePigpioClient:
    def __init__(self):
        self.connected = 1
        self.pulses: list[tuple[int, int]] = []
        self.current_pulse = 0
        self.forced_readback: int | None = None
        self.readbacks: list[int] = []
        self.stopped = False

    def set_servo_pulsewidth(self, gpio, pulse):
        self.pulses.append((gpio, pulse))
        self.current_pulse = pulse
        return 0

    def get_servo_pulsewidth(self, _gpio):
        if self.readbacks:
            return self.readbacks.pop(0)
        if self.forced_readback is not None:
            return self.forced_readback
        return self.current_pulse

    def stop(self):
        self.stopped = True


class FakePiSteering:
    def __init__(self):
        self.last_error = ""
        self.angles: list[int] = []
        self.center_calls = 0
        self.closed = False
        self.send_ok = True
        self.verify_ok = True
        self.verify_calls = 0
        self.calibration = car.DEFAULT_STEERING_CALIBRATION

    def set_steering(self, angle, sequence=None):
        self.angles.append(angle)
        if not self.send_ok:
            self.last_error = "simulated GPIO12 failure"
        return self.send_ok

    def verify_steering(self, _angle):
        self.verify_calls += 1
        if not self.verify_ok:
            self.last_error = "simulated GPIO12 readback failure"
        return self.verify_ok

    def center(self):
        self.center_calls += 1
        return self.set_steering(0)

    def close(self):
        self.closed = True


class FakeGamepadDevice:
    name = "Generic test pad"

    @staticmethod
    def get_char_device_path():
        return "/dev/input/event-test"


def configure_test_gamepad(
    controller,
    steering_code="ABS_RX",
    steering_current=128,
):
    controller.configure_gamepad(
        "Test gamepad",
        "/dev/input/test",
        car.AxisInfo("ABS_Y", 0, 255, 128),
        car.AxisInfo(steering_code, 0, 255, steering_current),
    )


class MappingTests(unittest.TestCase):
    def test_axis_normalization_and_deadzone(self):
        self.assertEqual(car.normalize_gamepad_axis(128, 0, 255), 0.0)
        self.assertEqual(car.normalize_gamepad_axis(130, 0, 255), 0.0)
        self.assertEqual(car.normalize_gamepad_axis(0, 0, 255), -1.0)
        self.assertEqual(car.normalize_gamepad_axis(255, 0, 255), 1.0)
        self.assertEqual(
            car.normalize_gamepad_axis(-32768, -32768, 32767),
            -1.0,
        )
        self.assertEqual(
            car.normalize_gamepad_axis(32767, -32768, 32767),
            1.0,
        )
        self.assertEqual(
            car.normalize_gamepad_axis(255, -32768, 32767),
            0.0,
        )
        self.assertEqual(
            car.normalize_gamepad_axis("bad", 0, 255),
            0.0,
        )

    def test_axis_selection_prefers_rx_and_falls_back_to_z(self):
        rx = car.AxisInfo("ABS_RX", -32768, 32767, 0)
        centered_z = car.AxisInfo("ABS_Z", 0, 255, 128)
        trigger_z = car.AxisInfo("ABS_Z", 0, 255, 0)
        self.assertIs(
            car.select_steering_axis(
                {"ABS_RX": rx, "ABS_Z": trigger_z}
            ),
            rx,
        )
        self.assertIs(
            car.select_steering_axis({"ABS_Z": centered_z}),
            centered_z,
        )
        self.assertIs(
            car.select_steering_axis({"ABS_Z": trigger_z}),
            trigger_z,
        )

    def test_throttle_and_drive_packet_are_clamped(self):
        self.assertEqual(car.throttle_to_pwm(100), 95)
        self.assertEqual(car.throttle_to_pwm(200), 95)
        self.assertEqual(car.throttle_to_pwm(-1), 0)
        self.assertEqual(
            car.format_drive_command(
                "F",
                100,
                0,
                car.DEFAULT_STEERING_CALIBRATION,
            ),
            b"D:F,95,1440\n",
        )
        self.assertEqual(
            car.format_drive_command(
                "N",
                100,
                -999,
                car.DEFAULT_STEERING_CALIBRATION,
            ),
            b"D:N,0,940\n",
        )

    def test_calibrated_asymmetric_steering_mapping(self):
        calibration = car.DEFAULT_STEERING_CALIBRATION
        self.assertEqual(car.steering_to_pulse(-50, calibration), 940)
        self.assertEqual(car.steering_to_pulse(0, calibration), 1440)
        self.assertEqual(car.steering_to_pulse(50, calibration), 2150)


class PicoTransportTests(unittest.TestCase):
    def test_connect_applies_calibration_and_stops(self):
        serial_instance = FakeSerial()
        pico = car.PicoController(
            serial_factory=lambda *_args, **_kwargs: serial_instance
        )
        self.assertTrue(pico.connect())
        self.assertEqual(
            serial_instance.writes,
            [b"SC:940,1440,2150\n", b"S\n"],
        )

    def test_ping_is_an_explicit_request(self):
        serial_instance = FakeSerial()
        serial_instance.responses.append(b"PONG\n")
        pico = car.PicoController(
            serial_factory=lambda *_args, **_kwargs: serial_instance
        )
        pico.connect()
        serial_instance.writes.clear()
        self.assertTrue(pico.ping())
        self.assertEqual(serial_instance.writes, [b"PING\n"])

    def test_battery_returns_raw_adc_millivolts(self):
        serial_instance = FakeSerial()
        serial_instance.responses.append(b"BAT:2250.0\n")
        pico = car.PicoController(
            serial_factory=lambda *_args, **_kwargs: serial_instance
        )
        pico.connect()
        serial_instance.writes.clear()
        self.assertEqual(pico.read_battery_adc_mv(), 2250.0)
        self.assertEqual(serial_instance.writes, [b"BAT?\n"])


class PiSteeringTests(unittest.TestCase):
    def test_gpio12_uses_calibrated_servo_pulses(self):
        client = FakePigpioClient()
        steering = car.PiSteeringController(
            pigpio_factory=lambda: client
        )
        self.assertTrue(steering.connect())
        self.assertTrue(steering.set_steering(-50))
        self.assertTrue(steering.set_steering(50))
        steering.close()
        self.assertEqual(
            client.pulses,
            [(12, 1440), (12, 940), (12, 2150), (12, 0)],
        )

    def test_transient_readback_mismatch_is_corrected(self):
        client = FakePigpioClient()
        steering = car.PiSteeringController(
            pigpio_factory=lambda: client
        )
        self.assertTrue(steering.connect())
        client.pulses.clear()
        client.readbacks = [1440, 2150]
        self.assertTrue(steering.set_steering(50, sequence=6))
        self.assertEqual(client.pulses, [(12, 2150), (12, 2150)])

    def test_readback_mismatch_retries_once_then_fails(self):
        client = FakePigpioClient()
        steering = car.PiSteeringController(
            pigpio_factory=lambda: client
        )
        self.assertTrue(steering.connect())
        client.pulses.clear()
        client.forced_readback = 1440
        self.assertFalse(steering.set_steering(50, sequence=7))
        self.assertEqual(client.pulses, [(12, 2150), (12, 2150)])
        self.assertIn("readback mismatch", steering.last_error)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = [10.0]
        self.pico = FakePico()
        self.logs: list[str] = []
        self.controller = car.CarController(
            self.pico,
            log=self.logs.append,
            now_fn=lambda: self.clock[0],
        )
        self.controller.set_gamepad_connected(True)
        configure_test_gamepad(self.controller)
        self.pico.commands.clear()

    def arm(self):
        self.controller.handle_event("BTN_START", 1)
        self.assertTrue(self.controller.armed)
        self.pico.commands.clear()

    def test_starts_disarmed_and_requires_neutral(self):
        self.assertFalse(self.controller.armed)
        self.controller.handle_event("ABS_Y", 0)
        self.controller.handle_event("BTN_START", 1)
        self.assertFalse(self.controller.armed)
        self.assertTrue(
            any("controls_not_neutral" in line for line in self.logs)
        )
        self.controller.handle_event("ABS_Y", 128)
        self.controller.handle_event("BTN_START", 1)
        self.assertTrue(self.controller.armed)

    def test_left_stick_drives_and_right_stick_steers(self):
        self.arm()
        self.controller.handle_event("ABS_Y", 0)
        self.assertEqual(self.pico.commands[-1], ("drive", "F", 100, 0))
        self.controller.handle_event("ABS_RX", 255)
        self.assertEqual(self.pico.commands[-1], ("drive", "F", 100, 50))
        self.controller.refresh_active_command()
        self.assertEqual(self.pico.commands[-1], ("drive", "F", 100, 50))

    def test_reverse_passes_through_stop_before_drive(self):
        self.arm()
        self.controller.handle_event("ABS_Y", 0)
        self.controller.handle_event("ABS_Y", 255)
        self.assertEqual(self.pico.commands[-1], ("stop",))
        self.controller.refresh_active_command()
        self.assertEqual(self.pico.commands[-1], ("drive", "R", 100, 0))

    def test_active_command_is_refreshed(self):
        self.arm()
        self.controller.handle_event("ABS_Y", 0)
        count = len(self.pico.commands)
        self.controller.refresh_active_command()
        self.assertEqual(len(self.pico.commands), count + 1)
        self.assertEqual(self.pico.commands[-1], ("drive", "F", 100, 0))

    def test_either_thumb_button_holds_brake(self):
        self.arm()
        self.controller.handle_event("BTN_THUMBL", 1)
        self.controller.handle_event("BTN_THUMBR", 1)
        self.controller.handle_event("BTN_THUMBL", 0)
        self.assertTrue(self.controller.brake)
        self.assertEqual(self.pico.commands[-1], ("brake",))
        self.controller.handle_event("BTN_THUMBR", 0)
        self.assertFalse(self.controller.brake)

    def test_start_toggles_disarm(self):
        self.arm()
        self.controller.handle_event("BTN_START", 1)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_double_select_stops_and_disarms(self):
        self.arm()
        self.controller.handle_event("BTN_SELECT", 1, event_time=10.0)
        self.controller.handle_event("BTN_SELECT", 1, event_time=10.5)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_trigger_x_latches_and_resets_estop_at_neutral(self):
        self.arm()
        self.controller.handle_event("BTN_TL2", 1)
        self.controller.handle_event("BTN_WEST", 1)
        self.assertTrue(self.controller.estop)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.pico.commands[-1], ("estop",))
        self.clock[0] += 0.21
        self.controller.handle_event("BTN_WEST", 1)
        self.assertFalse(self.controller.estop)
        self.assertEqual(self.pico.commands[-1], ("estop_reset",))
        self.assertFalse(self.controller.armed)

    def test_estop_reset_requires_neutral(self):
        self.arm()
        self.controller.handle_event("BTN_TL2", 1)
        self.controller.handle_event("BTN_X", 1)
        self.clock[0] += 0.21
        self.controller.handle_event("ABS_Y", 0)
        self.controller.handle_event("BTN_X", 1)
        self.assertTrue(self.controller.estop)
        self.assertNotEqual(self.pico.commands[-1], ("estop_reset",))

    def test_gamepad_disconnect_stops_and_disarms(self):
        self.arm()
        self.controller.handle_event("ABS_Y", 0)
        self.controller.set_gamepad_connected(False)
        self.assertFalse(self.controller.armed)
        self.assertFalse(self.controller.gamepad_connected)
        self.assertEqual(self.pico.commands[-1], ("stop",))

    def test_uart_write_failure_disarms(self):
        self.arm()
        self.pico.send_ok = False
        self.controller.handle_event("ABS_Y", 0)
        self.assertFalse(self.controller.armed)
        self.assertTrue(any("UART failure" in line for line in self.logs))

    def test_shutdown_sends_estop_and_closes_uart(self):
        self.controller.shutdown()
        self.assertEqual(self.pico.commands[-1], ("estop",))
        self.assertTrue(self.pico.closed)

    def test_gamepad_steering_is_sent_to_gpio12_driver(self):
        steering = FakePiSteering()
        controller = car.CarController(
            self.pico,
            pi_steering=steering,
            log=self.logs.append,
            now_fn=lambda: self.clock[0],
        )
        controller.set_gamepad_connected(True)
        configure_test_gamepad(controller)
        controller.handle_event("BTN_START", 1)
        controller.handle_event("ABS_RX", 255)
        self.assertEqual(steering.angles[-1], 50)

    def test_unselected_abs_z_trigger_does_not_change_steering(self):
        steering = FakePiSteering()
        controller = car.CarController(
            self.pico,
            pi_steering=steering,
            log=self.logs.append,
            now_fn=lambda: self.clock[0],
        )
        controller.set_gamepad_connected(True)
        configure_test_gamepad(controller)
        controller.handle_event("BTN_START", 1)
        controller.handle_event("ABS_RX", 255)
        command_count = len(self.pico.commands)
        angle_count = len(steering.angles)
        controller.handle_event("ABS_Z", 0)
        self.assertEqual(controller.steering, 50)
        self.assertEqual(len(self.pico.commands), command_count)
        self.assertEqual(len(steering.angles), angle_count)

    def test_stationary_health_failure_stops_and_disarms(self):
        steering = FakePiSteering()
        controller = car.CarController(
            self.pico,
            pi_steering=steering,
            log=self.logs.append,
            now_fn=lambda: self.clock[0],
        )
        controller.set_gamepad_connected(True)
        configure_test_gamepad(controller)
        controller.handle_event("BTN_START", 1)
        self.pico.commands.clear()
        steering.verify_ok = False
        self.clock[0] += car.STEERING_HEALTH_INTERVAL_S
        controller.refresh_active_command()
        self.assertFalse(controller.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))
        self.assertEqual(steering.verify_calls, 1)
        self.assertTrue(
            any("action=health_check" in line for line in self.logs)
        )

    def test_gpio12_failure_stops_and_disarms_the_car(self):
        steering = FakePiSteering()
        controller = car.CarController(
            self.pico,
            pi_steering=steering,
            log=self.logs.append,
            now_fn=lambda: self.clock[0],
        )
        controller.set_gamepad_connected(True)
        configure_test_gamepad(controller)
        controller.handle_event("BTN_START", 1)
        self.assertTrue(controller.armed)
        steering.send_ok = False
        controller.handle_event("ABS_RX", 255)
        self.assertFalse(controller.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))
        self.assertTrue(
            any("event=steering_failure" in line for line in self.logs)
        )


class RuntimeConfigurationTests(unittest.TestCase):
    def test_runtime_discovers_ranges_and_prefers_abs_rx(self):
        pico = FakePico()
        logs: list[str] = []
        controller = car.CarController(pico, log=logs.append)
        runtime = car.BasicCarRuntime(controller)
        axes = {
            "ABS_Y": car.AxisInfo("ABS_Y", -32768, 32767, 0),
            "ABS_Z": car.AxisInfo("ABS_Z", 0, 255, 0),
            "ABS_RX": car.AxisInfo("ABS_RX", -32768, 32767, 0),
        }

        def axis_reader(_path, code):
            return axes[code]

        with patch.object(car, "read_axis_info", side_effect=axis_reader):
            runtime._configure_connected_gamepad(FakeGamepadDevice())

        self.assertTrue(controller.gamepad_connected)
        self.assertIs(controller.throttle_axis, axes["ABS_Y"])
        self.assertIs(controller.steering_axis, axes["ABS_RX"])
        self.assertEqual(controller.axis_configuration_error, "")

    def test_shanwan_abs_z_waits_for_live_center_then_can_arm(self):
        pico = FakePico()
        logs: list[str] = []
        controller = car.CarController(pico, log=logs.append)
        runtime = car.BasicCarRuntime(controller)
        axes = {
            "ABS_Y": car.AxisInfo("ABS_Y", 0, 255, 128),
            "ABS_Z": car.AxisInfo("ABS_Z", 0, 255, 0),
        }

        def axis_reader(_path, code):
            if code not in axes:
                raise OSError("axis unavailable")
            return axes[code]

        with patch.object(car, "read_axis_info", side_effect=axis_reader):
            runtime._configure_connected_gamepad(FakeGamepadDevice())

        self.assertIs(controller.steering_axis, axes["ABS_Z"])
        self.assertIn("waiting for", controller.axis_configuration_error)
        controller.handle_event("BTN_START", 1)
        self.assertFalse(controller.armed)
        controller.handle_event("ABS_Z", 128)
        self.assertEqual(controller.axis_configuration_error, "")
        controller.handle_event("BTN_START", 1)
        self.assertTrue(controller.armed)
        self.assertTrue(
            any("event=gamepad_steering_confirmed" in line for line in logs)
        )

    def test_axis_configuration_error_prevents_arming(self):
        pico = FakePico()
        logs: list[str] = []
        controller = car.CarController(pico, log=logs.append)
        controller.set_gamepad_connected(True)
        controller.configure_gamepad(
            "Trigger-only pad",
            "/dev/input/test",
            car.AxisInfo("ABS_Y", 0, 255, 128),
            None,
            "ABS_Z is likely a trigger",
        )
        controller.handle_event("BTN_START", 1)
        self.assertFalse(controller.armed)
        self.assertTrue(
            any("reason=axis_configuration" in line for line in logs)
        )


class LoggingTests(unittest.TestCase):
    def tearDown(self):
        for handler in list(car.LOGGER.handlers):
            car.LOGGER.removeHandler(handler)
            handler.close()
        car.LOGGER.addHandler(car.logging.NullHandler())
        car.LOGGER.setLevel(car.logging.NOTSET)

    def test_rotating_log_contains_correlated_steering_readback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "car.log"
            logger = car.configure_logging(path)
            client = FakePigpioClient()
            steering = car.PiSteeringController(
                pigpio_factory=lambda: client,
                logger=logger,
            )
            self.assertTrue(steering.connect())
            self.assertTrue(steering.set_steering(50, sequence=42))
            for handler in logger.handlers:
                handler.flush()
            contents = path.read_text(encoding="utf-8")
            self.assertIn("event=steering_apply seq=42", contents)
            self.assertIn("requested_us=2150 readback_us=2150", contents)
            file_handlers = [
                handler
                for handler in logger.handlers
                if isinstance(handler, car.RotatingFileHandler)
            ]
            self.assertEqual(len(file_handlers), 1)
            self.assertEqual(file_handlers[0].maxBytes, 5 * 1024 * 1024)
            self.assertEqual(file_handlers[0].backupCount, 3)


if __name__ == "__main__":
    unittest.main()
