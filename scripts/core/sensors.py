"""
sensors.py — VL53L0X Forward Laser (Fixed Centre) + Camera Pan/Tilt Gimbal

Hardware
────────
  VL53L0X          Pico I2C — fixed centre-forward, no servo scanning
  Pan/Tilt Gimbal  Pico GP6/GP7 — camera only (laser is NOT on the gimbal)

The VL53L0X laser is mounted in a fixed bracket, pointing straight ahead.
The pan/tilt gimbal carries the camera only and is controlled separately.
No laser scanning sweep is available — the planner works with the single
fixed forward beam + odometry-based obstacle memory.
"""

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    from motor import MockGPIO
    GPIO = MockGPIO()

import time

# ── Pico sensor bridge (VL53L0X fixed forward + pan/tilt camera) ────
try:
    from pico_sensor_reader import (
        get_laser_distance_mm as _pico_laser_mm,
        get_laser_distance_cm as _pico_laser_cm,
        send_pan_tilt as _pico_send_pan_tilt,
        send_center as _pico_send_center,
    )
    _pico_available = True
    print("🔴 VL53L0X: Reading from Pico bridge (UART), fixed centre-forward")
except ImportError:
    _pico_available = False
    _pico_laser_mm = lambda: -1
    _pico_laser_cm = lambda: -1
    _pico_send_pan_tilt = lambda p, t: None
    _pico_send_center = lambda: None
    print("⚠️  Pico bridge unavailable — laser will return defaults")

# --- PIN CONFIGURATION ---

# --- PAN/TILT CONSTANTS ---
_PAN_CENTER  = 90
_TILT_CENTER = 90
_PAN_MIN     = 60
_PAN_MAX     = 120
_SETTLE_TIME = 0.05     # 50 ms settle after each pan move


class SensorSystem:
    """Manages forward VL53L0X laser (fixed) and camera pan/tilt (via Pico).

    The VL53L0X is fixed centre-forward — no scanning servo.
    Pan/tilt commands only move the camera gimbal.
    """

    def __init__(self):
        # 1. GPIO mode
        try:
            mode = GPIO.getmode()
            if mode is None:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
        except Exception as e:
            print(f"⚠️ GPIO Config Error: {e}")

        # 2. Laser is fixed forward (not on gimbal)
        self._laser_available = _pico_available

        # 3. Centre the camera pan/tilt on start-up
        _pico_send_center()
        time.sleep(0.3)
        print("🎯 VL53L0X fixed forward | Camera pan/tilt centred via Pico")

    # ──────────────────────────────────────────────────
    #  Camera pan/tilt servo helpers (via Pico UART)
    # ──────────────────────────────────────────────────

    def set_servo_angle(self, user_deg):
        """Set camera pan servo to a user-space angle (-30 … 0 … +30).

        -30 = hard left, 0 = centre, +30 = hard right.
        This controls the CAMERA gimbal only (not the laser).
        """
        user_deg = max(-30, min(30, user_deg))
        pan = _PAN_CENTER + user_deg   # e.g. 90 + (-30) = 60
        _pico_send_pan_tilt(pan, _TILT_CENTER)
        time.sleep(_SETTLE_TIME)

    def center_servo(self):
        """Return camera pan/tilt to forward-facing centre."""
        _pico_send_center()
        time.sleep(_SETTLE_TIME)

    # ──────────────────────────────────────────────────
    #  Laser distance helpers (VL53L0X — fixed forward)
    # ──────────────────────────────────────────────────

    def read_laser_mm(self):
        """Return VL53L0X reading in millimetres from Pico bridge, or -1 on error."""
        if not self._laser_available:
            return -1
        try:
            mm = _pico_laser_mm()
            if mm is None or mm > 2000:   # out-of-range / saturated
                return -1
            return mm
        except Exception:
            return -1

    def read_laser_cm(self):
        """Return VL53L0X distance in centimetres from Pico bridge, or -1 on error."""
        if not self._laser_available:
            return -1
        try:
            cm = _pico_laser_cm()
            if cm is None or cm < 0:
                return -1
            return cm
        except Exception:
            return -1

    def get_forward_distance(self):
        """Return forward distance in cm from the fixed VL53L0X laser.

        The laser is fixed centre-forward — no servo movement needed.
        Drop-in replacement for the old ``get_sonar_distance()``.
        """
        return self.read_laser_cm()

    # ──────────────────────────────────────────────────
    #  Scan sweep (legacy compat — returns single forward reading)
    # ──────────────────────────────────────────────────

    def scan_sweep(self, start_deg=-30, end_deg=30, step_deg=10):
        """Legacy compatibility — returns a single forward reading at 0°.

        The VL53L0X is fixed; pan servo is camera-only. No real sweep
        is possible. Returns [(0, forward_cm)] so callers that expect
        sweep data get a safe single reading.
        """
        dist = self.read_laser_cm()
        return [(0, dist)]

    # ──────────────────────────────────────────────────
    #  Cleanup
    # ──────────────────────────────────────────────────

    def cleanup(self):
        """Centre camera pan/tilt on shutdown."""
        try:
            _pico_send_center()
        except Exception:
            pass
