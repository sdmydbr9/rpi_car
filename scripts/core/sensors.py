"""
sensors.py — Laser Scanner (VL53L0X via Pico on Pan/Tilt Gimbal)

Hardware
────────
  VL53L0X          Pico I2C — time-of-flight laser via UART bridge
  Pan/Tilt Gimbal  Pico GP2/GP3 — controlled via UART PT: protocol

The VL53L0X laser is mounted on the camera pan/tilt gimbal (Pico servos).
Scanning sweeps the pan servo via UART commands while tilt stays centred.
Range: ±30° (pan servo 60–120°, centre = 90°).

User-space angles:  -30 (left)  …  0 (center)  …  +30 (right)
Pan servo angles:    60           90              120
Mapping: pan_angle = 90 + user_angle
"""

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    from motor import MockGPIO
    GPIO = MockGPIO()

import time

# ── Pico sensor bridge (VL53L0X + pan/tilt now on Pico) ────────
try:
    from pico_sensor_reader import (
        get_laser_distance_mm as _pico_laser_mm,
        get_laser_distance_cm as _pico_laser_cm,
        send_pan_tilt as _pico_send_pan_tilt,
        send_center as _pico_send_center,
    )
    _pico_available = True
    print("🔴 VL53L0X: Reading from Pico bridge (UART), laser on pan/tilt gimbal")
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
    """Manages the laser scanner (VL53L0X on pan/tilt gimbal via Pico)."""

    def __init__(self):
        # 1. GPIO mode
        try:
            mode = GPIO.getmode()
            if mode is None:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
        except Exception as e:
            print(f"⚠️ GPIO Config Error: {e}")

        # 2. Laser on pan/tilt (controlled via Pico UART, no Pi GPIO needed)
        self._laser_available = _pico_available

        # 3. Centre the pan/tilt on start-up
        _pico_send_center()
        time.sleep(0.3)
        print("🎯 Laser scanner ready (pan/tilt centred via Pico)")

    # ──────────────────────────────────────────────────
    #  Pan/tilt servo helpers (via Pico UART)
    # ──────────────────────────────────────────────────

    def set_servo_angle(self, user_deg):
        """Set pan servo to a user-space angle (-30 … 0 … +30).

        -30 = hard left, 0 = centre, +30 = hard right.
        Tilt stays centred. Sends PT: command to Pico via UART.
        """
        user_deg = max(-30, min(30, user_deg))
        pan = _PAN_CENTER + user_deg   # e.g. 90 + (-30) = 60
        _pico_send_pan_tilt(pan, _TILT_CENTER)
        time.sleep(_SETTLE_TIME)

    def center_servo(self):
        """Return pan/tilt to forward-facing centre."""
        _pico_send_center()
        time.sleep(_SETTLE_TIME)

    # ──────────────────────────────────────────────────
    #  Laser distance helpers
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
        """Centre the servo and return forward distance in cm.

        Drop-in replacement for the old ``get_sonar_distance()`` —
        used by the smoothing buffer and physics_loop.
        """
        self.center_servo()
        return self.read_laser_cm()

    # kept for backward-compat (main.py references)
    def get_sonar_distance(self):
        """Alias → get_forward_distance() for code that still says 'sonar'."""
        return self.get_forward_distance()

    def get_sonar_distance_raw(self):
        """Alias → get_forward_distance() for backward compat. Sonar hardware removed."""
        return self.get_forward_distance()

    # Backward-compat aliases
    def get_front_sonar_distance(self):
        """Alias → get_forward_distance() for backward compatibility."""
        return self.get_forward_distance()

    def get_rear_sonar_distance(self):
        """Alias → get_forward_distance() for backward compatibility."""
        return self.get_forward_distance()

    # ──────────────────────────────────────────────────
    #  Full sweep scan (-30° to +30° in STEP increments)
    # ──────────────────────────────────────────────────

    def scan_sweep(self, start_deg=-30, end_deg=30, step_deg=10):
        """Sweep pan servo and sample VL53L0X at each angle.

        Parameters
        ----------
        start_deg : int   Left-most user-space angle (default -30).
        end_deg   : int   Right-most user-space angle (default +30).
        step_deg  : int   Angle increment (default 10).

        Returns
        -------
        list[tuple[int, float]]
            ``[(angle_deg, distance_cm), …]`` for each step.
            distance_cm is -1 when the laser read fails.

        Timing: 7 steps × ~0.05 s settle = ~0.35 s total.
        """
        readings = []
        for angle in range(start_deg, end_deg + 1, step_deg):
            self.set_servo_angle(angle)
            dist = self.read_laser_cm()
            readings.append((angle, dist))
        # Return pan to centre after sweep
        self.center_servo()
        return readings

    # ──────────────────────────────────────────────────
    #  Cleanup
    # ──────────────────────────────────────────────────

    def cleanup(self):
        """Centre pan/tilt on shutdown."""
        try:
            _pico_send_center()
        except Exception:
            pass
