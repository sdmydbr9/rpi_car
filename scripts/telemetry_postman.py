"""
Telemetry Postman — background daemon thread that ships RPi Car telemetry
to the Grafana / Telegraf / InfluxDB stack at 2 Hz using InfluxDB line protocol.

Transport : HTTP POST  →  Telegraf influxdb_listener  →  InfluxDB 2.x  →  Grafana
Endpoint  : http://<GRAFANA_HOST>:8186/write?db=car_telemetry&precision=ns
Zero extra Python dependencies — stdlib urllib.request only.

Usage in main.py:
    from telemetry_postman import TelemetryPostman
    postman = TelemetryPostman(car_state, car_system)
    postman.start()
    ...
    postman.stop()   # called in finally block
"""

import threading
import time
import logging
import urllib.request
import urllib.error
import subprocess
import re
from typing import Any, Dict


def _os_cpu_temp() -> float:
    """Read CPU temperature from sysfs (°C)."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return 0.0


def _os_cpu_clock() -> float:
    """Read current CPU clock (MHz)."""
    for path in ('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq',
                 '/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq'):
        try:
            with open(path) as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            pass
    return 0.0


def _os_gpu_clock() -> float:
    """Read GPU clock via vcgencmd (MHz)."""
    try:
        r = subprocess.run(['vcgencmd', 'measure_clock', 'gpu'],
                          capture_output=True, text=True, timeout=1)
        m = re.search(r'=(\d+)', r.stdout)
        if m:
            return int(m.group(1)) / 1_000_000.0
    except Exception:
        pass
    return 0.0

# ─────────────────────── CONFIG ──────────────────────────────────────────────
GRAFANA_HOST = "192.168.29.105"
TELEGRAF_PORT = 8186                # Telegraf [[inputs.influxdb_listener]] port
INFLUX_DB     = "car_telemetry"     # passed as ?db= (Telegraf maps it to the bucket)
SEND_INTERVAL = 0.5                 # seconds — 2 Hz
HTTP_TIMEOUT  = 1.0                 # seconds — never block the postman long
HOST_TAG      = "rpi_car"

log = logging.getLogger("TelemetryPostman")


# ─────────────────────── LINE PROTOCOL HELPERS ───────────────────────────────

def _escape_tag(val: str) -> str:
    """Escape spaces and commas in tag values (InfluxDB line protocol rules)."""
    return str(val).replace(" ", r"\ ").replace(",", r"\,").replace("=", r"\=")


def _fmt_field(val: Any) -> str | None:
    """
    Render a Python value to an InfluxDB line-protocol field representation.
    Returns None for values that should be skipped (None / empty string).
    """
    if val is None:
        return None
    if isinstance(val, bool):
        return "1i" if val else "0i"
    if isinstance(val, int):
        return f"{val}i"
    if isinstance(val, float):
        if val != val:          # NaN guard
            return None
        return f"{val:.6g}"
    # string → skip (we don't want to store arbitrary strings as fields here)
    return None


def _build_line(measurement: str, tags: Dict[str, str],
                fields: Dict[str, Any], ts_ns: int) -> str | None:
    """
    Build a single InfluxDB line-protocol line.
    Returns None when no numeric fields are present (avoids empty writes).
    """
    tag_str = ",".join(
        f"{_escape_tag(k)}={_escape_tag(v)}"
        for k, v in tags.items()
        if v is not None
    )
    field_parts = []
    for k, v in fields.items():
        rendered = _fmt_field(v)
        if rendered is not None:
            field_parts.append(f"{k}={rendered}")
    if not field_parts:
        return None
    tag_block = f",{tag_str}" if tag_str else ""
    return f"{measurement}{tag_block} {','.join(field_parts)} {ts_ns}"


# ─────────────────────── POSTMAN CLASS ───────────────────────────────────────

class TelemetryPostman:
    """
    Daemon thread that collects car telemetry at 2 Hz and POSTs it
    to a Telegraf influxdb_listener endpoint.
    """

    def __init__(self, car_state: dict, car_system: Any,
                 host: str = GRAFANA_HOST, port: int = TELEGRAF_PORT):
        self._car_state  = car_state
        self._car_system = car_system
        self._url        = (
            f"http://{host}:{port}/write"
            f"?db={INFLUX_DB}&precision=ns"
        )
        self._stop_event = threading.Event()
        self._thread     = threading.Thread(
            target=self._run,
            name="TelemetryPostman",
            daemon=True,
        )
        log.info("[Postman] Initialized → %s (%.1f Hz)", self._url, 1 / SEND_INTERVAL)

    # ── public API ──────────────────────────────────────────────────────────

    def start(self):
        self._stop_event.clear()
        self._thread.start()
        log.info("[Postman] Started (interval=%.2fs)", SEND_INTERVAL)

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=3.0)
        log.info("[Postman] Stopped")

    # ── internal loop ───────────────────────────────────────────────────────

    def _run(self):
        consecutive_errors = 0
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                lines = self._collect()
                if lines:
                    self._post("\n".join(lines))
                    consecutive_errors = 0
            except Exception as exc:          # pragma: no cover
                consecutive_errors += 1
                if consecutive_errors <= 3 or consecutive_errors % 20 == 0:
                    log.warning("[Postman WARN] collect/post error: %s", exc)
            elapsed = time.monotonic() - t0
            sleep_for = max(0.0, SEND_INTERVAL - elapsed)
            self._stop_event.wait(sleep_for)

    def _post(self, body: str):
        data = body.encode("utf-8")
        req  = urllib.request.Request(
            self._url,
            data=data,
            method="POST",
            headers={"Content-Type": "text/plain; charset=utf-8"},
        )
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                status = resp.status
        except urllib.error.HTTPError as e:
            status = e.code
        if status not in (200, 204):
            log.warning("[Postman WARN] HTTP %d from Telegraf", status)

    # ── telemetry collection ────────────────────────────────────────────────

    def _collect(self) -> list[str]:
        cs    = self._car_state
        ts    = time.time_ns()
        lines = []

        # ── wheel_sync snapshot ───────────────────────────────────────────
        try:
            sync = self._car_system.get_sync_telemetry() or {}
        except Exception:
            sync = {}
        wheels = sync.get("wheels", {})
        sync_status = sync.get("status", "UNKNOWN")

        # ── 1. drive ─────────────────────────────────────────────────────
        gear      = str(cs.get("gear", "N"))
        direction = str(cs.get("direction", "stop"))
        line = _build_line(
            "drive",
            {"host": HOST_TAG, "gear": gear, "direction": direction},
            {
                "speed_mpm"         : cs.get("encoder_speed_mpm") or 0.0,
                "current_pwm"       : cs.get("current_pwm", 0.0),
                "steer_angle"       : cs.get("steer_angle", 0),
                "rpm"               : cs.get("encoder_rpm") or 0.0,
                "encoder_available" : cs.get("encoder_available", False),
                "gas_pressed"       : cs.get("gas_pressed", False),
                "brake_pressed"     : cs.get("brake_pressed", False),
                "engine_running"    : cs.get("engine_running", False),
                "emergency_brake"   : cs.get("emergency_brake_active", False),
                "is_braking"        : cs.get("is_braking", False),
                "wheel_sync_active" : sync_status == "ACTIVE",
            },
            ts,
        )
        if line:
            lines.append(line)

        # ── 2. power ─────────────────────────────────────────────────────
        # -1 is the sentinel for "sensor not connected" — omit those fields
        batt_v = cs.get("battery_voltage", -1)
        curr_a = cs.get("current_amps", -1)
        power_fields: Dict[str, Any] = {
            "effective_max_duty" : cs.get("effective_max_duty", 0.0),
            "l298n_voltage_drop" : cs.get("l298n_voltage_drop", 0.0),
            "effective_motor_v"  : cs.get("effective_motor_voltage", 0.0),
        }
        if batt_v >= 0:
            power_fields["battery_voltage"] = batt_v
        if curr_a >= 0:
            power_fields["current_amps"] = curr_a
        line = _build_line("power", {"host": HOST_TAG}, power_fields, ts)
        if line:
            lines.append(line)

        # ── 3. sensors ───────────────────────────────────────────────────
        sonar_dist = cs.get("sonar_distance")
        # Pico UART sensor packet (latest values stored on car_state by main.py)
        accel_x = cs.get("accel_x", 0.0)
        accel_y = cs.get("accel_y", 0.0)
        accel_z = cs.get("accel_z", 0.0)
        gyro_z  = cs.get("gyro_z", 0.0)
        imu_temp = cs.get("imu_temp_c", 0.0)
        # laser distance — main.py exposes camera_obstacle_distance which is
        # from the vision system; the raw laser_mm comes from pico
        laser_mm = cs.get("laser_mm", 0)

        line = _build_line(
            "sensors",
            {"host": HOST_TAG},
            {
                "sonar_dist_cm" : sonar_dist if sonar_dist is not None else -1.0,
                "laser_mm"      : laser_mm,
                "accel_x"       : accel_x,
                "accel_y"       : accel_y,
                "accel_z"       : accel_z,
                "gyro_z"        : gyro_z,
                "imu_temp_c"    : imu_temp if imu_temp else 0.0,
                "pid_correction": cs.get("pid_correction", 0.0),
                "left_obstacle" : cs.get("left_obstacle", False),
                "right_obstacle": cs.get("right_obstacle", False),
            },
            ts,
        )
        if line:
            lines.append(line)

        # ── 4. per-wheel (4 separate lines, name tag) ─────────────────────
        for wname in ("fl", "fr", "rl", "rr"):
            w = wheels.get(wname, {})
            line = _build_line(
                "wheel",
                {"host": HOST_TAG, "name": wname},
                {
                    "target_rpm"  : w.get("target_rpm", 0.0),
                    "actual_rpm"  : w.get("actual_rpm", 0.0),
                    "applied_pwm" : w.get("applied_pwm", 0.0),
                    "error"       : w.get("error", 0.0),
                    "on_target"   : w.get("on_target", False),
                    "has_encoder" : w.get("has_encoder", False),
                },
                ts,
            )
            if line:
                lines.append(line)

        # ── 5. autonomy ─────────────────────────────────────────────────
        auto_state    = str(cs.get("autonomous_state", "IDLE"))
        obstacle_state = str(cs.get("obstacle_state", "IDLE"))
        line = _build_line(
            "autonomy",
            {"host": HOST_TAG, "state": auto_state, "obstacle": obstacle_state},
            {
                "autonomous_mode"       : cs.get("autonomous_mode", False),
                "hunter_mode"           : cs.get("hunter_mode", False),
                "target_yaw"            : cs.get("target_yaw", 0.0),
                "current_heading"       : cs.get("current_heading", 0.0),
                "slalom_sign"           : cs.get("slalom_sign", 0),
                "speed_limit"           : cs.get("speed_limit", 100),
                "speed_limit_enabled"   : cs.get("speed_limit_enabled", False),
            },
            ts,
        )
        if line:
            lines.append(line)

        # ── 6. vision ────────────────────────────────────────────────────
        line = _build_line(
            "vision",
            {"host": HOST_TAG},
            {
                "vision_active"      : cs.get("vision_active", False),
                "detections_count"   : cs.get("camera_detections_count", 0),
                "in_path_count"      : cs.get("camera_in_path_count", 0),
                "closest_confidence" : cs.get("camera_closest_confidence", 0.0),
                "vision_fps"         : cs.get("vision_fps", 0.0),
                "camera_fps_actual"  : cs.get("camera_actual_fps", 0.0),
                "cam_obstacle_cm"    : cs.get("camera_obstacle_distance", 0.0),
            },
            ts,
        )
        if line:
            lines.append(line)

        # ── 7. system (CPU / GPU) — read directly from OS, not car_state ──
        line = _build_line(
            "system",
            {"host": HOST_TAG},
            {
                "cpu_temp_c"    : _os_cpu_temp(),
                "cpu_clock_mhz" : _os_cpu_clock(),
                "gpu_clock_mhz" : _os_gpu_clock(),
                "gamepad_conn"  : cs.get("gamepad_connected", False),
                "narr_speaking" : cs.get("narration_speaking", False),
            },
            ts,
        )
        if line:
            lines.append(line)

        return lines
