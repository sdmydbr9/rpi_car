import unittest

import scripts.main as server

from tests.test_manual_control import FakePico


class SocketApiTests(unittest.TestCase):
    def setUp(self):
        self.pico = FakePico()
        server.arbiter = server.ManualControlArbiter(self.pico)
        server.arbiter.set_status_callback(server.broadcast_status)
        self.client = server.socketio.test_client(server.app)

    def tearDown(self):
        if self.client.is_connected():
            self.client.disconnect()

    def event_names(self):
        return [item["name"] for item in self.client.get_received()]

    def test_connect_has_no_telemetry_stream(self):
        names = self.event_names()
        self.assertIn("connection_response", names)
        self.assertIn("control_status", names)
        self.assertNotIn("telemetry_update", names)

    def test_claim_arm_and_direct_drive(self):
        self.client.get_received()
        self.client.emit("control_claim")
        self.client.emit("arm")
        self.client.emit(
            "drive_input",
            {
                "seq": 1,
                "direction": "F",
                "throttle": 25,
                "steering": -10,
                "brake": False,
            },
        )
        self.assertEqual(self.pico.commands[-1], ("drive", "F", 25, -10))

    def test_non_owner_can_engage_estop(self):
        second_client = server.socketio.test_client(server.app)
        second_client.emit("emergency_stop")
        self.assertEqual(self.pico.commands[-1], ("estop",))
        second_client.disconnect()

    def test_owner_disconnect_stops_and_disarms(self):
        self.client.get_received()
        self.client.emit("control_claim")
        self.client.emit("arm")
        self.client.emit(
            "drive_input",
            {
                "seq": 1,
                "direction": "F",
                "throttle": 20,
                "steering": 0,
                "brake": False,
            },
        )
        self.client.disconnect()
        self.assertFalse(server.arbiter.armed)
        self.assertEqual(self.pico.commands[-1], ("stop",))


if __name__ == "__main__":
    unittest.main()
