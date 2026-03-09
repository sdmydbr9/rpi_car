"""
sensors.py — Laser Scanner (VL53L0X via Pico on Pan/Tilt Gimbal) + Sonar

Hardware
────────
  VL53L0X          Pico I2C — time-of-flight laser via UART bridge
  Pan/Tilt Gimbal  Pico GP2/GP3 — controlled via UART PT: protocol
  HC-SR04 Sonar    GPIO 25/24 — stays on Pi (ultrasonic forward)

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
# HC-SR04 Ultrasonic Sonar (stays on Pi, faces forward)
SONAR_TRIG = 25
SONAR_ECHO = 24

# --- PAN/TILT CONSTANTS ---
_PAN_CENTER  = 90
_TILT_CENTER = 90
_PAN_MIN     = 60
_PAN_MAX     = 120
_SETTLE_TIME = 0.05     # 50 ms settle after each pan move


class SensorSystem:
    """Manages the laser scanner (VL53L0X on pan/tilt gimbal via Pico) and sonar."""

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

        # 3. HC-SR04 Ultrasonic Sonar (stays on Pi, forward-facing)
        self._sonar_available = False
        try:
            GPIO.setup(SONAR_TRIG, GPIO.OUT)
            GPIO.setup(SONAR_ECHO, GPIO.IN)
            GPIO.output(SONAR_TRIG, False)
            time.sleep(0.05)
            self._sonar_available = True
            print("📡 Sonar: Connected (HC-SR04, TRIG=GPIO25, ECHO=GPIO24)")
        except Exception as e:
            print(f"⚠️  Sonar: Init failed — {e}")

        # 4. Centre the pan/tilt on start-up
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
        """Read HC-SR04 ultrasonic distance in cm.

        Returns distance in cm, or -1 on timeout/error.
        """
        if not self._sonar_available:
            return -1
        try:
            # Trigger pulse
            GPIO.output(SONAR_TRIG, True)
            time.sleep(0.00001)
            GPIO.output(SONAR_TRIG, False)

            # Wait for echo HIGH (with timeout)
            pulse_start = time.time()
            timeout = pulse_start + 0.06   # 60 ms ≈ ~10 m max
            while GPIO.input(SONAR_ECHO) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return -1

            # Wait for echo LOW
            pulse_end = time.time()
            timeout = pulse_end + 0.06
            while GPIO.input(SONAR_ECHO) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return -1

            duration = pulse_end - pulse_start
            distance = round(duration * 17150, 1)

            # Sanity: ignore readings > 400 cm (HC-SR04 max)
            if distance > 400:
                return -1

            return distance
        except Exception:
            return -1

    # Backward-compat aliases
    def get_front_sonar_distance(self):
        """Alias → get_sonar_distance_raw() for backward compatibility."""
        return self.get_sonar_distance_raw()

    def get_rear_sonar_distance(self):
        """Alias → get_sonar_distance_raw() for backward compatibility."""
        return self.get_sonar_distance_raw()

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
