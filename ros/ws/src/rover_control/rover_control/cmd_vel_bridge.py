import json
import os
import threading
import time
from dataclasses import dataclass
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class TargetCommand:
    move: bool
    motion_sign: int
    gear: str
    speed_pct: int
    steer_deg: int


class RoverCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__('rover_cmd_vel_bridge')

        self.server_url = self.declare_parameter('server_url', 'http://127.0.0.1:5000').value.rstrip('/')
        self.server_url_file = str(
            self.declare_parameter('server_url_file', '/app/ros/.server_url').value
        )
        self.url_refresh_sec = float(self.declare_parameter('url_refresh_sec', 1.0).value)
        command_topic = self.declare_parameter('command_topic', '/cmd_vel').value

        self.timer_hz = float(self.declare_parameter('timer_hz', 10.0).value)
        self.command_timeout_sec = float(self.declare_parameter('command_timeout_sec', 0.6).value)
        self.request_timeout_sec = float(self.declare_parameter('request_timeout_sec', 0.25).value)

        self.linear_deadband = float(self.declare_parameter('linear_deadband', 0.05).value)
        self.angular_deadband = float(self.declare_parameter('angular_deadband', 0.05).value)

        self.max_steer_deg = int(self.declare_parameter('max_steer_deg', 90).value)
        self.min_speed_pct = int(self.declare_parameter('min_speed_pct', 20).value)
        self.max_speed_pct = int(self.declare_parameter('max_speed_pct', 60).value)
        self.forward_gear = str(self.declare_parameter('forward_gear', '1').value)
        self.shift_pause_sec = float(self.declare_parameter('shift_pause_sec', 0.25).value)
        self.auto_engine_start = bool(self.declare_parameter('auto_engine_start', True).value)
        self.auto_release_emergency_brake = bool(
            self.declare_parameter('auto_release_emergency_brake', True).value
        )
        self.control_rearm_sec = float(self.declare_parameter('control_rearm_sec', 1.0).value)

        self.log_requests = bool(self.declare_parameter('log_requests', False).value)

        if self.forward_gear not in {'1', '2', '3', 'S'}:
            self.get_logger().warning(
                "Invalid forward_gear parameter; using '1'."
            )
            self.forward_gear = '1'

        if self.max_speed_pct < self.min_speed_pct:
            self.max_speed_pct = self.min_speed_pct

        self._cmd_lock = threading.Lock()
        self._latest_cmd = Twist()
        self._latest_cmd_time = 0.0
        self._have_cmd = False

        self._last_error_log_time = 0.0

        self._last_sent_gear = None
        self._last_sent_speed = None
        self._last_sent_steer = None
        self._throttle_active = False
        self._motion_sign = 0
        self._shift_block_until = 0.0
        self._timed_out_once = False
        self._active_server_url = self.server_url
        self._last_url_refresh_time = 0.0
        self._server_urls = []
        self._control_ready = False
        self._last_control_arm_attempt = 0.0
        self._refresh_server_urls(force=True)

        self.create_subscription(Twist, command_topic, self._on_cmd_vel, 10)
        period = 1.0 / max(1.0, self.timer_hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Bridge ready: topic={command_topic} server={self._active_server_url} "
            f"server_file={self.server_url_file} speed={self.min_speed_pct}-{self.max_speed_pct}%"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._latest_cmd.linear.x = msg.linear.x
            self._latest_cmd.angular.z = msg.angular.z
            self._latest_cmd_time = time.monotonic()
            self._have_cmd = True
        self._timed_out_once = False

    def _on_timer(self) -> None:
        self._refresh_server_urls()

        with self._cmd_lock:
            have_cmd = self._have_cmd
            cmd = Twist()
            cmd.linear.x = self._latest_cmd.linear.x
            cmd.angular.z = self._latest_cmd.angular.z
            cmd_age = time.monotonic() - self._latest_cmd_time if have_cmd else 1e9

        if (not have_cmd) or (cmd_age > self.command_timeout_sec):
            self._send_stop()
            if have_cmd and not self._timed_out_once:
                self.get_logger().warn('cmd_vel timeout; rover stopped for safety.')
                self._timed_out_once = True
            return

        target = self._cmd_to_target(cmd)

        if not target.move:
            self._send_stop()
            return

        now = time.monotonic()
        if self._motion_sign != 0 and target.motion_sign != self._motion_sign:
            self._send_stop(force=True)
            self._shift_block_until = now + self.shift_pause_sec
            return

        if now < self._shift_block_until:
            return

        self._apply_target(target)

    def _cmd_to_target(self, msg: Twist) -> TargetCommand:
        linear = self._clamp(float(msg.linear.x), -1.0, 1.0)
        angular = self._clamp(float(msg.angular.z), -1.0, 1.0)

        if abs(linear) < self.linear_deadband and abs(angular) < self.angular_deadband:
            return TargetCommand(False, 0, self.forward_gear, 0, 0)

        if linear > self.linear_deadband:
            sign = 1
        elif linear < -self.linear_deadband:
            sign = -1
        else:
            sign = 1

        speed_norm = abs(linear)
        if speed_norm < self.linear_deadband:
            speed_norm = max(0.25, min(1.0, abs(angular)))

        speed_pct = int(round(
            self.min_speed_pct + (self.max_speed_pct - self.min_speed_pct) * speed_norm
        ))
        speed_pct = int(self._clamp(speed_pct, self.min_speed_pct, self.max_speed_pct))

        steer_deg = int(round(-angular * float(self.max_steer_deg)))
        if abs(steer_deg) < 2:
            steer_deg = 0
        steer_deg = int(self._clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg))

        gear = self.forward_gear if sign >= 0 else 'R'

        return TargetCommand(True, sign, gear, speed_pct, steer_deg)

    def _apply_target(self, target: TargetCommand) -> None:
        if not self._ensure_control_ready():
            return

        if self._last_sent_gear != target.gear:
            if not self._http_get(f'/gear/{target.gear}'):
                self._control_ready = False
                return
            self._last_sent_gear = target.gear

        if self._last_sent_speed is None or abs(target.speed_pct - self._last_sent_speed) >= 2:
            if not self._http_get(f'/speed_limit_enable/{target.speed_pct}'):
                self._control_ready = False
                return
            self._last_sent_speed = target.speed_pct

        if self._last_sent_steer != target.steer_deg:
            if not self._http_get(f'/steer/{target.steer_deg}'):
                self._control_ready = False
                return
            self._last_sent_steer = target.steer_deg

        if not self._throttle_active:
            if not self._http_get('/forward'):
                self._control_ready = False
                return
            self._throttle_active = True

        if self._throttle_active:
            self._motion_sign = target.motion_sign

    def _send_stop(self, force: bool = False) -> None:
        if self._throttle_active or force:
            if self._http_get('/stop'):
                self._throttle_active = False
            else:
                self._control_ready = False

        if self._last_sent_steer not in (None, 0):
            if self._http_get('/steer/0'):
                self._last_sent_steer = 0
            else:
                self._control_ready = False

        self._motion_sign = 0

    def _ensure_control_ready(self, force: bool = False) -> bool:
        if not self.auto_engine_start and not self.auto_release_emergency_brake:
            self._control_ready = True
            return True

        now = time.monotonic()
        if not force and self._control_ready:
            return True
        if not force and (now - self._last_control_arm_attempt) < self.control_rearm_sec:
            return self._control_ready

        self._last_control_arm_attempt = now
        ok = True

        if self.auto_engine_start:
            ok = self._http_get('/engine/start') and ok
        if self.auto_release_emergency_brake:
            ok = self._http_get('/emergency_brake/off') and ok

        self._control_ready = ok
        if ok:
            self.get_logger().info('Rover control armed (engine on, emergency brake released).')
        return ok

    def _http_get(self, path: str) -> bool:
        self._refresh_server_urls()
        if self._try_http_get(path):
            return True

        # Force-refresh candidate URLs (e.g. host IP file changed) and retry once.
        self._refresh_server_urls(force=True)
        return self._try_http_get(path)

    def _try_http_get(self, path: str) -> bool:
        last_exc = None
        for base_url in self._ordered_server_urls():
            url = f'{base_url}{path}'
            try:
                with urlopen(url, timeout=self.request_timeout_sec) as response:
                    body = response.read(128)
            except (HTTPError, URLError, TimeoutError, OSError) as exc:
                last_exc = exc
                continue

            previous_url = self._active_server_url
            self._active_server_url = base_url
            if previous_url != self._active_server_url:
                self.get_logger().info(
                    f"Switched rover API endpoint: {previous_url} -> {self._active_server_url}"
                )
            self._maybe_update_server_url_from_body(path, body)
            if self.log_requests:
                self.get_logger().info(f'HTTP GET {path} via {self._active_server_url}')
            return True

        now = time.monotonic()
        if now - self._last_error_log_time > 1.0 and last_exc is not None:
            self.get_logger().warn(
                f"HTTP command failed ({path}) via {self._ordered_server_urls()}: {last_exc}"
            )
            self._last_error_log_time = now
        return False

    def _refresh_server_urls(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_url_refresh_time) < self.url_refresh_sec:
            return
        self._last_url_refresh_time = now

        discovered_file_url = self._read_server_url_file()
        candidates = []
        for candidate in (
            self._active_server_url,
            discovered_file_url,
            self.server_url,
        ):
            normalized = self._normalize_server_url(candidate)
            if normalized and normalized not in candidates:
                candidates.append(normalized)

        if candidates:
            self._server_urls = candidates
            self._active_server_url = candidates[0]

    def _ordered_server_urls(self):
        ordered = []
        for candidate in [self._active_server_url] + list(self._server_urls):
            normalized = self._normalize_server_url(candidate)
            if normalized and normalized not in ordered:
                ordered.append(normalized)
        if not ordered:
            normalized = self._normalize_server_url(self.server_url)
            if normalized:
                ordered.append(normalized)
        return ordered

    def _read_server_url_file(self):
        file_path = self.server_url_file
        if not file_path:
            return None

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                raw = f.read().strip()
        except OSError:
            return None

        return self._normalize_server_url(raw)

    def _maybe_update_server_url_from_body(self, path: str, body: bytes) -> None:
        # Opportunistically learn current host IP from the rover API status endpoint.
        if path != '/api/server-ip' or not body:
            return
        try:
            payload = json.loads(body.decode('utf-8'))
        except Exception:
            return

        ip = str(payload.get('ip', '')).strip()
        if not ip:
            return

        discovered = self._normalize_server_url(f'http://{ip}:5000')
        if not discovered:
            return
        if discovered not in self._server_urls:
            self._server_urls.append(discovered)

    @staticmethod
    def _normalize_server_url(raw_url):
        if raw_url is None:
            return None
        value = str(raw_url).strip()
        if not value:
            return None
        if not value.startswith(('http://', 'https://')):
            value = f'http://{value}'
        return value.rstrip('/')

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def shutdown(self) -> None:
        self._send_stop(force=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
