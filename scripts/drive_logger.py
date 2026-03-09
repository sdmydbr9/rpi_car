"""
drive_logger.py — Comprehensive Drive Telemetry Logger

Records all sensor data at ~50Hz during manual driving (gamepad or web UI)
to a CSV file in rover_logs/ for post-drive analysis and visualization.

Logged data:
  - Timestamp + elapsed time
  - Distance sensors: VL53L0X laser (mm), HC-SR04 sonar (cm)
  - IMU (MPU6500): accelerometer XYZ (g), gyroscope XYZ (deg/s), temperature
  - Power: battery voltage (mV via ADS1115 A0), current sense (mV via A1)
  - Wheel encoders (LM393): RPM for rear_left, rear_right, front_right
  - Motor output: commanded PWM L/R, applied PWM per wheel (FL/FR/RL/RR)
  - Control state: gear, throttle input, steering input, braking, direction
  - Wheel sync: PID status, target RPM per side

Usage:
    from drive_logger import DriveLogger

    logger = DriveLogger(source="webui")
    logger.start()
    # In your 50Hz loop:
    logger.log_tick({...})
    # On stop:
    logger.stop()
"""

import atexit
import csv
import os
import time
import threading
from datetime import datetime

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, 'rover_logs')

# CSV column order — kept flat for easy pandas consumption
COLUMNS = [
    # Timing
    'timestamp', 'elapsed_s',
    # Source
    'source',
    # Distance sensors
    'laser_front_mm', 'sonar_front_cm',
    # IMU — accelerometer (g)
    'accel_x', 'accel_y', 'accel_z',
    # IMU — gyroscope (deg/s)
    'gyro_x', 'gyro_y', 'gyro_z',
    # IMU — temperature
    'temp_c',
    # Power (raw mV from ADS1115)
    'battery_mv', 'current_mv',
    # Wheel encoder RPMs (LM393)
    'rpm_rear_left', 'rpm_rear_right', 'rpm_front_right',
    # Motor commands (what the driver requested)
    'cmd_pwm_left', 'cmd_pwm_right',
    # Motor output (what the wheels actually received after sync)
    'applied_pwm_fl', 'applied_pwm_fr', 'applied_pwm_rl', 'applied_pwm_rr',
    # Control inputs
    'gear', 'throttle_input', 'steering_input', 'is_braking', 'is_forward',
    # Wheel sync controller state
    'sync_status', 'target_rpm_left', 'target_rpm_right',
]


class DriveLogger:
    """Thread-safe CSV telemetry logger with buffered I/O."""

    def __init__(self, source="unknown"):
        self._source = source
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._start_time = None
        self._running = False
        self._tick_count = 0
        self._filepath = None

    @property
    def filepath(self):
        return self._filepath

    @property
    def is_running(self):
        return self._running

    @property
    def tick_count(self):
        return self._tick_count

    def start(self, source: str = None):
        """Open a new CSV log file and write the header.

        Args:
            source: Optional override for the session source label
                    (e.g. "webui", "gamepad", "gamepad_hot_start").
                    If omitted the instance-level default is used.
        """
        if self._running:
            return
        if source is not None:
            self._source = source
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._filepath = os.path.join(LOG_DIR, f"drive_log_{ts}.csv")
        self._file = open(self._filepath, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS,
                                      extrasaction='ignore')
        self._writer.writeheader()
        self._file.flush()
        self._start_time = time.time()
        self._tick_count = 0
        self._running = True
        # Register atexit handler so logs are saved on any exit
        atexit.register(self.stop)
        print(f"[DriveLogger] Recording ({self._source}) → {self._filepath}")

    def stop(self):
        """Flush and close the log file."""
        if not self._running:
            return
        self._running = False
        with self._lock:
            if self._file:
                try:
                    self._file.flush()
                    os.fsync(self._file.fileno())
                except Exception:
                    pass
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None
                self._writer = None
        dur = time.time() - self._start_time if self._start_time else 0
        # Unregister atexit to avoid double-stop
        try:
            atexit.unregister(self.stop)
        except Exception:
            pass
        print(f"[DriveLogger] Stopped — {self._tick_count} rows, "
              f"{dur:.1f}s, file: {self._filepath}")

    def log_tick(self, data: dict):
        """Write one row of telemetry data.

        Args:
            data: dict with keys matching COLUMNS (missing keys default to '').
                  'timestamp', 'elapsed_s', and 'source' are auto-filled.
        """
        if not self._running or self._writer is None:
            return

        now = time.time()
        row = dict(data)
        row['timestamp'] = round(now, 4)
        row['elapsed_s'] = round(now - self._start_time, 4)
        row['source'] = self._source

        with self._lock:
            if self._writer is not None:
                self._writer.writerow(row)
                self._tick_count += 1
                # fsync every 50 ticks (~1s at 50Hz) to ensure data
                # reaches disk even if the process is killed abruptly
                if self._tick_count % 50 == 0:
                    self._file.flush()
                    try:
                        os.fsync(self._file.fileno())
                    except Exception:
                        pass

    def __del__(self):
        if self._running:
            self.stop()
