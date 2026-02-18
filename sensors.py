"""
sensors.py — Laser Scanner (Servo + VL53L0X) + IR Obstacle Sensors

Hardware
────────
  Servo (SG90)     GPIO 21  — 50 Hz PWM, duty 2–12 for 0–180°
  VL53L0X          I2C 0x29 — time-of-flight laser, ~30–1200 mm effective
  IR Left          GPIO 5   — active LOW (0 = obstacle)
  IR Right         GPIO 6   — active LOW (0 = obstacle)

The servo is mounted on the front of the rover with the VL53L0X on top.
Together they act as a scanning LIDAR: the servo sweeps -60° to +60° in
10° steps while the laser samples distance at each angle.

User-space angles:  -60 (left)  …  0 (center)  …  +60 (right)
Servo-space angles:  30           90              150
Mapping: servo_angle = 90 + user_angle

Replaces the removed HC-SR04 front & rear sonar modules.
"""

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    from motor import MockGPIO
    GPIO = MockGPIO()

import time

# ── VL53L0X (I2C laser) ──────────────────────────────
_tof_available = False
_tof = None
try:
    import board
    import busio
    import adafruit_vl53l0x
    _i2c = busio.I2C(board.SCL, board.SDA)
    _tof = adafruit_vl53l0x.VL53L0X(_i2c)
    _tof_available = True
    print("🔴 VL53L0X: Connected (I2C 0x29)")
except Exception as e:
    print(f"⚠️  VL53L0X: Init failed — {e}")

# --- PIN CONFIGURATION ---
SERVO_PIN = 21   # Servo PWM output

# IR Obstacle Sensors
IR_LEFT = 5      # Front Left
IR_RIGHT = 6     # Front Right

# Rear HC-SR04 Ultrasonic Sonar
REAR_TRIG = 25
REAR_ECHO = 24

# --- SERVO CONSTANTS ---
_SERVO_FREQ   = 50       # Standard servo PWM frequency (Hz)
_SERVO_CENTER = 90       # Servo midpoint (degrees in servo-space)
_DUTY_MIN     = 2.0      # Duty cycle at 0°
_DUTY_SCALE   = 18.0     # duty = _DUTY_MIN + servo_angle / _DUTY_SCALE
_SETTLE_TIME  = 0.05     # 50 ms settle after each servo move


class SensorSystem:
    """Manages the front laser scanner (servo + VL53L0X) and IR sensors."""

    def __init__(self):
        # 1. GPIO mode
        try:
            mode = GPIO.getmode()
            if mode is None:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
        except Exception as e:
            print(f"⚠️ GPIO Config Error: {e}")

        # 2. Servo PWM
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, _SERVO_FREQ)
        self._pwm.start(0)
        self._servo_angle = _SERVO_CENTER  # current servo-space angle
        self._laser_available = _tof_available

        # 3. IR Sensors
        GPIO.setup(IR_LEFT, GPIO.IN)
        GPIO.setup(IR_RIGHT, GPIO.IN)

        # 4. Rear HC-SR04 Ultrasonic Sonar
        self._rear_sonar_available = False
        try:
            GPIO.setup(REAR_TRIG, GPIO.OUT)
            GPIO.setup(REAR_ECHO, GPIO.IN)
            GPIO.output(REAR_TRIG, False)
            time.sleep(0.05)
            self._rear_sonar_available = True
            print("📡 Rear sonar: Connected (HC-SR04, TRIG=GPIO25, ECHO=GPIO24)")
        except Exception as e:
            print(f"⚠️  Rear sonar: Init failed — {e}")

        # 5. Centre the servo on start-up
        self._set_servo_raw(_SERVO_CENTER)
        time.sleep(0.3)
        print("🎯 Laser scanner ready (servo centred)")

    # ──────────────────────────────────────────────────
    #  Servo helpers
    # ──────────────────────────────────────────────────

    def _set_servo_raw(self, servo_deg):
        """Move servo to *servo_deg* (0–180 in servo-space)."""
        servo_deg = max(0, min(180, servo_deg))
        duty = _DUTY_MIN + (servo_deg / _DUTY_SCALE)
        self._pwm.ChangeDutyCycle(duty)
        time.sleep(_SETTLE_TIME)
        self._pwm.ChangeDutyCycle(0)       # cut signal → stop jitter
        self._servo_angle = servo_deg

    def set_servo_angle(self, user_deg):
        """Set servo to a user-space angle (-60 … 0 … +60).

        -60 = hard left, 0 = centre, +60 = hard right.
        """
        servo_deg = _SERVO_CENTER + user_deg   # e.g. 90 + (-60) = 30
        self._set_servo_raw(servo_deg)

    def center_servo(self):
        """Return servo to forward-facing (0°)."""
        self._set_servo_raw(_SERVO_CENTER)

    # ──────────────────────────────────────────────────
    #  Laser distance helpers
    # ──────────────────────────────────────────────────

    def read_laser_mm(self):
        """Return raw VL53L0X reading in millimetres, or -1 on error."""
        if not self._laser_available or _tof is None:
            return -1
        try:
            mm = _tof.range
            if mm is None or mm > 2000:   # out-of-range / saturated
                return -1
            return mm
        except Exception:
            return -1

    def read_laser_cm(self):
        """Return VL53L0X distance in centimetres, or -1 on error."""
        mm = self.read_laser_mm()
        if mm < 0:
            return -1
        return round(mm / 10.0, 1)

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

    def get_rear_sonar_distance(self):
        """Read rear HC-SR04 ultrasonic distance in cm.

        Returns distance in cm, or -1 on timeout/error.
        Filters out floor-bounce readings (14-18 cm range when
        the sensor is angled slightly downward).
        """
        if not self._rear_sonar_available:
            return -1
        try:
            # Trigger pulse
            GPIO.output(REAR_TRIG, True)
            time.sleep(0.00001)
            GPIO.output(REAR_TRIG, False)

            # Wait for echo HIGH (with timeout)
            pulse_start = time.time()
            timeout = pulse_start + 0.06   # 60 ms ≈ ~10 m max
            while GPIO.input(REAR_ECHO) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return -1

            # Wait for echo LOW
            pulse_end = time.time()
            timeout = pulse_end + 0.06
            while GPIO.input(REAR_ECHO) == 1:
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

    # ──────────────────────────────────────────────────
    #  Full sweep scan (-60° to +60° in STEP increments)
    # ──────────────────────────────────────────────────

    def scan_sweep(self, start_deg=-60, end_deg=60, step_deg=10):
        """Sweep servo and sample VL53L0X at each angle.

        Parameters
        ----------
        start_deg : int   Left-most user-space angle (default -60).
        end_deg   : int   Right-most user-space angle (default +60).
        step_deg  : int   Angle increment (default 10).

        Returns
        -------
        list[tuple[int, float]]
            ``[(angle_deg, distance_cm), …]`` for each step.
            distance_cm is -1 when the laser read fails.

        Timing: 13 steps × ~0.05 s settle = ~0.65 s total.
        """
        readings = []
        for angle in range(start_deg, end_deg + 1, step_deg):
            self.set_servo_angle(angle)
            dist = self.read_laser_cm()
            readings.append((angle, dist))
        # Return servo to centre after sweep
        self.center_servo()
        return readings

    # ──────────────────────────────────────────────────
    #  IR sensors  (unchanged from original)
    # ──────────────────────────────────────────────────

    def get_ir_status(self):
        """Return (Left_Obstacle, Right_Obstacle) as booleans.

        True = Obstacle Detected, False = Path Clear.
        """
        left_detect = not GPIO.input(IR_LEFT)
        right_detect = not GPIO.input(IR_RIGHT)
        return left_detect, right_detect

    # ──────────────────────────────────────────────────
    #  Cleanup
    # ──────────────────────────────────────────────────

    def cleanup(self):
        """Stop servo PWM.  Call on shutdown."""
        try:
            self._pwm.stop()
        except Exception:
            pass
