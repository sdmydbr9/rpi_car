#!/usr/bin/env python3
"""
auto_slalom.py — Autonomous Laser-Scanning Slalom Controller
=============================================================
Standalone script for obstacle-avoidance slalom using:
  • VL53L0X laser on servo for continuous 120° sector scanning
  • Front IR sensors (left + right) for close-range detection
  • MPU6050 accelerometer for crash detection, gyro for yaw tracking
  • Rear HC-SR04 sonar for safe reversing
  • Differential steering for smooth swerving (no pivot turns)

Run:  python3 auto_slalom.py
Controls:  S = Start | P = Pause | R = Resume/Retry | Q = Quit
Logs:      slalom.logs (append mode, comprehensive timestamped data)
"""

import time
import math
import threading
import logging
import curses
import signal
import sys
import os
import json
from collections import deque
from enum import Enum, auto

# ─────────────────────────────────────────────────────────────────────────────
#  TUNEABLE CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────

# Motor / PWM
BASE_SPEED          = 40        # forward cruise PWM % (was 50 — too fast for sensor reaction time)
MAX_PWM_DUTY        = 63        # voltage cap  (7 V / 11.1 V × 100)
PWM_FREQ            = 1000      # Hz
REVERSE_SPEED       = 40
REVERSE_DURATION    = 1.0       # seconds
RECOVERY_TURN_DURATION = 1.0    # seconds of wide turn after reverse
CRASH_REVERSE_SPEED = 35
CRASH_REVERSE_DURATION = 0.5
DIRECTION_CHANGE_DWELL = 0.15   # seconds between fwd↔rev

# Per-wheel trims  (calibrated to make the car go straight)
FL_TRIM = 0.60
FR_TRIM = 0.60
RL_TRIM = 1.00
RR_TRIM = 1.00

# Obstacle distances (cm)
WARN_DIST           = 60        # start swerving  (was 100 — too reactive in slalom, every sector triggered)
CRITICAL_DIST       = 30        # reduce speed  (was 35 — slightly lower to avoid crawling)
ALL_BLOCKED_CM      = 18        # every forward sector is this close → boxed in (was 30 — too conservative for slalom)
EMERGENCY_IR_DIST   = 0         # IR triggers (binary, active LOW)
HARD_SWERVE_DIST    = 25        # force min swerve when forward obstacle nearer  (was 50 — triggered constantly in slalom)
MIN_FORCED_SWERVE   = 25        # minimum swerve degrees when forced  (was 40 — too aggressive, caused circling)
EMERGENCY_FWD_CM    = 18        # emergency max-swerve when centre sectors this close  (was 25)
SIDE_CLEAR_MIN_CM   = 20        # side must have at least this to be considered clear (was 40 — false boxed-in)
CIRCLE_HEADING_LIMIT = 120      # degrees — trigger boxed-in if car rotates this much  (was 150 — detect earlier)

# Swerve geometry
SWERVE_MAX_ANGLE    = 70        # max differential angle (degrees)  (was 90 — caused near-pivot turns)
IR_SWERVE_BOOST     = 55        # added degrees when IR fires  (was 65)
SPEED_FLOOR_FACTOR  = 0.55      # minimum speed fraction near obstacles
INNER_WHEEL_MIN     = 0.25      # inner wheels keep 25% speed for forward motion  (was 0.00 — pivot turns)
SWERVE_SPEED_FLOOR  = 1.00      # outer wheel stays at full BASE_SPEED during swerve
SWERVE_OUTER_BOOST  = 1.05      # slight boost to outer wheel  (was 1.15 — too aggressive differential)
SWERVE_SMOOTH_ALPHA = 0.40      # EMA smoothing factor  (was 0.70 — locked swerve at max too quickly)

# Heading-rate damping — prevents persistent one-direction swerve (circling)
HEADING_DAMP_START  = 30        # degrees — start damping after this much heading change
HEADING_DAMP_FULL   = 120       # degrees — full damping at this heading change
HEADING_DAMP_MAX    = 0.70      # max swerve reduction factor (0–1)

# PID heading correction (gyro-Z)
GYRO_KP             = 1.5
GYRO_KI             = 0.0
GYRO_KD             = 0.3

# Crash detection (accelerometer)
# NOTE: mpu6050 lib returns m/s², NOT g.  1 g ≈ 9.81 m/s².
ACCEL_G             = 9.81      # m/s² per g
CRASH_G             = 1.5       # g-force lateral impact threshold
CRASH_MS2           = CRASH_G * ACCEL_G   # ~14.7 m/s² – actual comparison value
MOVING_THRESHOLD_G  = 0.3       # detect if wheels spin but car is stuck
CRASH_COOLDOWN      = 3.0       # seconds between consecutive crash events

# Sector map
SCAN_RANGE_DEG      = 60        # ±60° from centre  → 120° total
SCAN_STEP_DEG       = 10        # 13 sectors
SCAN_SETTLE_MS      = 0.050     # 50 ms servo settle per step
SECTOR_STALE_S      = 3.0       # discard sector data older than this
SECTOR_DECAY_S      = 1.5       # reduce confidence after this age
SECTOR_DEFAULT_CM   = 120.0     # assumed distance when stale / unknown

# Floor rejection — reject very short readings (sensor noise / too close)
FLOOR_REJECT_CM     = 12        # absolute minimum (anything shorter = noise)

# Grace period — react as soon as any sector has data
GRACE_SWEEPS        = 0         # was 2 — blinded car for ~1.5 s, causing collisions

# Rear sonar
REAR_CLEAR_CM       = 15        # minimum rear clearance to allow reverse
REAR_STOP_CM        = 10        # abort reverse if rear drops below this
REAR_MIN_VALID_CM   = 3.0       # reject readings shorter than this (sensor noise)
REAR_FLOOR_LOW      = 14        # floor-bounce band
REAR_FLOOR_HIGH     = 18

# MPU6050 calibration
CALIBRATION_SAMPLES = 200
CALIBRATION_INTERVAL = 0.010    # seconds between samples

# Timing
LOOP_HZ             = 50
LOOP_PERIOD         = 1.0 / LOOP_HZ
UI_REFRESH_HZ       = 5
REAR_SONAR_HZ       = 5
SENSOR_LOG_EVERY_N  = 1         # log every Nth control cycle (1 = every)

# ─────────────────────────────────────────────────────────────────────────────
#  GPIO PIN MAP  (BCM numbering)
# ─────────────────────────────────────────────────────────────────────────────

# Front-left motor
FL_IN1 = 17;  FL_IN2 = 27;  FL_ENA = 12
# Front-right motor
FR_IN3 = 23;  FR_IN4 = 22;  FR_ENB = 13
# Rear-left motor
RL_IN1 = 10;  RL_IN2 = 7;   RL_ENA = 19
# Rear-right motor
RR_IN3 = 9;   RR_IN4 = 11;  RR_ENB = 18

# Sensors
IR_LEFT_PIN   = 5
IR_RIGHT_PIN  = 6
SERVO_PIN     = 21
REAR_TRIG_PIN = 25
REAR_ECHO_PIN = 24

# ─────────────────────────────────────────────────────────────────────────────
#  STATE ENUM
# ─────────────────────────────────────────────────────────────────────────────

class State(Enum):
    CALIBRATING    = auto()
    READY          = auto()
    DRIVING        = auto()
    PAUSED         = auto()
    BOXED_IN       = auto()
    CRASH_RECOVERY = auto()
    SHUTDOWN       = auto()

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING SETUP  →  slalom.logs
# ─────────────────────────────────────────────────────────────────────────────

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slalom.logs")

_file_formatter = logging.Formatter(
    fmt="%(asctime)s.%(msecs)03d | %(levelname)-5s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

_log = logging.getLogger("auto_slalom")
_log.setLevel(logging.DEBUG)

_fh = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
_fh.setLevel(logging.DEBUG)
_fh.setFormatter(_file_formatter)
_log.addHandler(_fh)

# Ring buffer for curses display
_ui_log_lines: deque = deque(maxlen=80)

class _UIHandler(logging.Handler):
    """Copy log records into a deque for the curses display."""
    def emit(self, record):
        try:
            msg = self.format(record)
            _ui_log_lines.append(msg)
        except Exception:
            pass

_uih = _UIHandler()
_uih.setLevel(logging.INFO)
_uih.setFormatter(logging.Formatter("%(asctime)s.%(msecs)03d | %(message)s",
                                     datefmt="%H:%M:%S"))
_log.addHandler(_uih)

# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE ABSTRACTION
# ─────────────────────────────────────────────────────────────────────────────

try:
    import RPi.GPIO as GPIO
    ON_PI = True
except ImportError:
    ON_PI = False
    _log.warning("RPi.GPIO not found – running in MOCK mode")

    class _MockPWM:
        def __init__(self, *a, **kw): self._dc = 0
        def start(self, dc): self._dc = dc
        def ChangeDutyCycle(self, dc): self._dc = dc
        def stop(self): pass

    class GPIO:                       # type: ignore[no-redef]
        BCM = 11; OUT = 0; IN = 1; PUD_UP = 22
        LOW = 0; HIGH = 1; BOTH = 33
        _state: dict = {}
        @classmethod
        def setmode(cls, m): pass
        @classmethod
        def setwarnings(cls, b): pass
        @classmethod
        def setup(cls, pin, mode, **kw): cls._state[pin] = 0
        @classmethod
        def output(cls, pin, val):
            if isinstance(pin, (list, tuple)):
                for p in pin: cls._state[p] = val
            else:
                cls._state[pin] = val
        @classmethod
        def input(cls, pin):
            return cls._state.get(pin, 1)
        @classmethod
        def cleanup(cls): cls._state.clear()
        @classmethod
        def PWM(cls, pin, freq): return _MockPWM(pin, freq)

try:
    import board, busio, adafruit_vl53l0x
    _i2c = busio.I2C(board.SCL, board.SDA)
    _tof = adafruit_vl53l0x.VL53L0X(_i2c)
    HAS_TOF = True
except Exception as exc:
    HAS_TOF = False
    _tof = None
    _log.warning("VL53L0X unavailable: %s", exc)

try:
    from mpu6050 import mpu6050 as _mpu6050_lib
    _imu = _mpu6050_lib(0x68)
    HAS_IMU = True
except Exception as exc:
    HAS_IMU = False
    _imu = None
    _log.warning("MPU6050 unavailable: %s", exc)

# ─────────────────────────────────────────────────────────────────────────────
#  LOW-LEVEL MOTOR DRIVER
# ─────────────────────────────────────────────────────────────────────────────

class MotorDriver:
    """Direct 4-wheel differential drive with trims & voltage cap."""

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Direction pins
        self._dir_pins = [
            FL_IN1, FL_IN2, FR_IN3, FR_IN4,
            RL_IN1, RL_IN2, RR_IN3, RR_IN4,
        ]
        for p in self._dir_pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)

        # PWM pins
        GPIO.setup(FL_ENA, GPIO.OUT)
        GPIO.setup(FR_ENB, GPIO.OUT)
        GPIO.setup(RL_ENA, GPIO.OUT)
        GPIO.setup(RR_ENB, GPIO.OUT)

        self.pwm_fl = GPIO.PWM(FL_ENA, PWM_FREQ)
        self.pwm_fr = GPIO.PWM(FR_ENB, PWM_FREQ)
        self.pwm_rl = GPIO.PWM(RL_ENA, PWM_FREQ)
        self.pwm_rr = GPIO.PWM(RR_ENB, PWM_FREQ)

        for p in (self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr):
            p.start(0)

        self._last_dir = "stop"
        self._current_pwm = [0.0, 0.0, 0.0, 0.0]  # FL FR RL RR

        _log.info("MOTOR_INIT pins=%s pwm_freq=%d max_duty=%d",
                  self._dir_pins, PWM_FREQ, MAX_PWM_DUTY)

    # ── direction helpers ────────────────────────────────────────────────

    def _set_forward(self):
        """H-bridge forward: IN_A=HIGH, IN_B=LOW.
        Matches motor.py CarSystem: (pin, l_fwd=True) → IN1=HIGH, IN2=LOW."""
        if self._last_dir == "forward":
            return
        if self._last_dir == "reverse":
            self._coast_pause()
        # FL
        GPIO.output(FL_IN1, GPIO.HIGH); GPIO.output(FL_IN2, GPIO.LOW)
        # FR
        GPIO.output(FR_IN3, GPIO.HIGH); GPIO.output(FR_IN4, GPIO.LOW)
        # RL
        GPIO.output(RL_IN1, GPIO.HIGH); GPIO.output(RL_IN2, GPIO.LOW)
        # RR
        GPIO.output(RR_IN3, GPIO.HIGH); GPIO.output(RR_IN4, GPIO.LOW)
        self._last_dir = "forward"

    def _set_reverse(self):
        """H-bridge reverse: IN_A=LOW, IN_B=HIGH."""
        if self._last_dir == "reverse":
            return
        if self._last_dir == "forward":
            self._coast_pause()
        # FL
        GPIO.output(FL_IN1, GPIO.LOW); GPIO.output(FL_IN2, GPIO.HIGH)
        # FR
        GPIO.output(FR_IN3, GPIO.LOW); GPIO.output(FR_IN4, GPIO.HIGH)
        # RL
        GPIO.output(RL_IN1, GPIO.LOW); GPIO.output(RL_IN2, GPIO.HIGH)
        # RR
        GPIO.output(RR_IN3, GPIO.LOW); GPIO.output(RR_IN4, GPIO.HIGH)
        self._last_dir = "reverse"

    def _coast_pause(self):
        """Brief coast before direction change to avoid shoot-through."""
        for p in (self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr):
            p.ChangeDutyCycle(0)
        for p in self._dir_pins:
            GPIO.output(p, GPIO.LOW)
        self._last_dir = "stop"
        time.sleep(DIRECTION_CHANGE_DWELL)

    def _set_duty(self, fl, fr, rl, rr):
        """Set PWM duty for all 4 motors, capped at MAX_PWM_DUTY."""
        vals = [
            min(MAX_PWM_DUTY, max(0, fl)),
            min(MAX_PWM_DUTY, max(0, fr)),
            min(MAX_PWM_DUTY, max(0, rl)),
            min(MAX_PWM_DUTY, max(0, rr)),
        ]
        self.pwm_fl.ChangeDutyCycle(vals[0])
        self.pwm_fr.ChangeDutyCycle(vals[1])
        self.pwm_rl.ChangeDutyCycle(vals[2])
        self.pwm_rr.ChangeDutyCycle(vals[3])
        self._current_pwm = vals

    # ── public drive commands ────────────────────────────────────────────

    def forward_differential(self, left_speed, right_speed):
        """Drive forward with independent left / right speeds (0-100).
        Applies per-wheel trims and MAX_PWM_DUTY cap."""
        self._set_forward()
        fl = left_speed  * FL_TRIM
        fr = right_speed * FR_TRIM
        rl = left_speed  * RL_TRIM
        rr = right_speed * RR_TRIM
        self._set_duty(fl, fr, rl, rr)

    def reverse_straight(self, speed):
        """Reverse at given speed, applying trims."""
        self._set_reverse()
        fl = speed * FL_TRIM
        fr = speed * FR_TRIM
        rl = speed * RL_TRIM
        rr = speed * RR_TRIM
        self._set_duty(fl, fr, rl, rr)

    def reverse_differential(self, left_speed, right_speed):
        """Reverse with independent left / right speeds."""
        self._set_reverse()
        fl = left_speed  * FL_TRIM
        fr = right_speed * FR_TRIM
        rl = left_speed  * RL_TRIM
        rr = right_speed * RR_TRIM
        self._set_duty(fl, fr, rl, rr)

    def brake(self):
        """Magnetic lock – H-bridge short (both pins HIGH + 100 % PWM)."""
        for p in self._dir_pins:
            GPIO.output(p, GPIO.HIGH)
        self._set_duty(MAX_PWM_DUTY, MAX_PWM_DUTY,
                       MAX_PWM_DUTY, MAX_PWM_DUTY)
        self._last_dir = "brake"

    def coast(self):
        """All pins LOW, PWM 0 — free-wheeling coast."""
        for p in (self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr):
            p.ChangeDutyCycle(0)
        for p in self._dir_pins:
            GPIO.output(p, GPIO.LOW)
        self._last_dir = "stop"
        self._current_pwm = [0.0, 0.0, 0.0, 0.0]

    def cleanup(self):
        """Release PWM and GPIO resources."""
        self.coast()
        for p in (self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr):
            p.stop()

    @property
    def current_pwm(self):
        return list(self._current_pwm)

# ─────────────────────────────────────────────────────────────────────────────
#  LASER SCANNER  (servo + VL53L0X)
# ─────────────────────────────────────────────────────────────────────────────

class LaserScanner:
    """Continuous-sweep sector mapper using a servo-mounted VL53L0X."""

    def __init__(self):
        self._angles = list(range(-SCAN_RANGE_DEG, SCAN_RANGE_DEG + 1,
                                  SCAN_STEP_DEG))
        self._n_sectors = len(self._angles)

        # Per-sector storage: [distance_cm, timestamp]
        self._sector_data = [[SECTOR_DEFAULT_CM, 0.0]
                              for _ in range(self._n_sectors)]
        self._lock = threading.Lock()
        self._sweep_count = 0
        self._running = False
        self._thread = None

        # Servo PWM setup
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz standard servo
        self._pwm.start(0)

        _log.info("SCANNER_INIT sectors=%d angles=%s",
                  self._n_sectors, self._angles)

    # ── servo helpers ────────────────────────────────────────────────────

    def _set_servo_angle(self, user_deg):
        """Move servo to user-space angle (-60…+60).
        Servo-space = 90 + user_deg.  Duty = 2 + servo_deg / 18."""
        servo_deg = 90 + user_deg
        servo_deg = max(0, min(180, servo_deg))
        duty = 2.0 + servo_deg / 18.0
        self._pwm.ChangeDutyCycle(duty)
        time.sleep(SCAN_SETTLE_MS)
        self._pwm.ChangeDutyCycle(0)   # stop jitter

    def center_servo(self):
        """Return servo to forward-facing (0°)."""
        self._set_servo_angle(0)

    # ── laser reading ────────────────────────────────────────────────────

    @staticmethod
    def _read_laser_cm():
        """Read VL53L0X distance in cm, or -1 on error."""
        if not HAS_TOF or _tof is None:
            return -1
        try:
            mm = _tof.range
            if mm is None or mm > 2000:
                return -1
            cm = round(mm / 10.0, 1)
            if cm < FLOOR_REJECT_CM:
                return -1
            return cm
        except Exception:
            return -1

    # ── sector map access ────────────────────────────────────────────────

    def get_sector_map(self):
        """Return list of (angle_deg, distance_cm, confidence) tuples."""
        now = time.time()
        result = []
        with self._lock:
            for i in range(self._n_sectors):
                age = now - self._sector_data[i][1]
                dist = self._sector_data[i][0]
                if age > SECTOR_STALE_S or self._sector_data[i][1] == 0:
                    dist = SECTOR_DEFAULT_CM
                    conf = 0.0
                elif age > SECTOR_DECAY_S:
                    conf = max(0.3, 1.0 - (age - SECTOR_DECAY_S) /
                               (SECTOR_STALE_S - SECTOR_DECAY_S))
                else:
                    conf = 1.0
                result.append((self._angles[i], dist, conf))
        return result

    def get_min_forward_distance(self):
        """Return minimum distance in the centre sectors (±15°)."""
        smap = self.get_sector_map()
        centre_dists = [d for a, d, c in smap if -15 <= a <= 15]
        if centre_dists:
            return min(centre_dists)
        return SECTOR_DEFAULT_CM

    # ── thread control ───────────────────────────────────────────────────

    def start(self):
        """Start continuous background scanning."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        _log.info("SCANNER_START continuous sweep ±%d° step=%d°",
                  SCAN_RANGE_DEG, SCAN_STEP_DEG)

    def stop(self):
        """Stop scanning thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.center_servo()
        _log.info("SCANNER_STOP")

    @property
    def sweep_count(self):
        """Number of completed sweeps."""
        return self._sweep_count

    def _scan_loop(self):
        """Background thread: sweep servo back and forth, sampling laser."""
        idx = 0
        direction = 1  # +1 = left-to-right, -1 = right-to-left

        while self._running:
            angle = self._angles[idx]
            self._set_servo_angle(angle)
            dist = self._read_laser_cm()
            now = time.time()

            if dist > 0:
                with self._lock:
                    self._sector_data[idx] = [dist, now]

            # Advance index
            idx += direction
            if idx >= self._n_sectors:
                idx = self._n_sectors - 2
                direction = -1
                self._sweep_count += 1
            elif idx < 0:
                idx = 1
                direction = 1
                self._sweep_count += 1

    def cleanup(self):
        """Stop scanning and release servo PWM."""
        self.stop()
        try:
            self._pwm.stop()
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────────────────────
#  IR OBSTACLE SENSORS
# ─────────────────────────────────────────────────────────────────────────────

class IRSensors:
    def __init__(self):
        GPIO.setup(IR_LEFT_PIN, GPIO.IN)
        GPIO.setup(IR_RIGHT_PIN, GPIO.IN)
        _log.info("IR_INIT left=GPIO%d right=GPIO%d (active LOW)",
                  IR_LEFT_PIN, IR_RIGHT_PIN)

    def read(self):
        """Returns (left_obstacle: bool, right_obstacle: bool).
        Active LOW: GPIO 0 = obstacle present."""
        l = not GPIO.input(IR_LEFT_PIN)
        r = not GPIO.input(IR_RIGHT_PIN)
        return l, r

# ─────────────────────────────────────────────────────────────────────────────
#  REAR HC-SR04 SONAR
# ─────────────────────────────────────────────────────────────────────────────

class RearSonar:
    """Threaded rear distance sensor with median filtering and floor
    detection.  Runs at REAR_SONAR_HZ in the background."""

    def __init__(self):
        GPIO.setup(REAR_TRIG_PIN, GPIO.OUT)
        GPIO.setup(REAR_ECHO_PIN, GPIO.IN)
        GPIO.output(REAR_TRIG_PIN, False)
        time.sleep(0.05)

        self._dist = -1.0
        self._floor_bounce = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._history = deque(maxlen=5)

        _log.info("REAR_SONAR_INIT trig=GPIO%d echo=GPIO%d",
                  REAR_TRIG_PIN, REAR_ECHO_PIN)

    def _measure_once(self):
        """Single HC-SR04 measurement.  Returns distance in cm or -1."""
        try:
            GPIO.output(REAR_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(REAR_TRIG_PIN, False)

            t0 = time.time()
            timeout = t0 + 0.06
            while GPIO.input(REAR_ECHO_PIN) == 0:
                t0 = time.time()
                if t0 > timeout:
                    return -1

            t1 = time.time()
            timeout = t1 + 0.06
            while GPIO.input(REAR_ECHO_PIN) == 1:
                t1 = time.time()
                if t1 > timeout:
                    return -1

            duration = t1 - t0
            dist = round(duration * 17150, 1)
            if dist > 400 or dist < REAR_MIN_VALID_CM:
                return -1
            return dist
        except Exception:
            return -1

    def read(self):
        """Return (distance_cm, floor_bounce_flag)."""
        with self._lock:
            return self._dist, self._floor_bounce

    def start(self):
        """Start background measurement thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        _log.info("REAR_SONAR_START rate=%d Hz", REAR_SONAR_HZ)

    def stop(self):
        """Stop measurement thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        _log.info("REAR_SONAR_STOP")

    def _loop(self):
        """Background measurement loop with median filtering."""
        period = 1.0 / REAR_SONAR_HZ
        while self._running:
            d = self._measure_once()
            if d > 0:
                self._history.append(d)

            # Median of recent readings
            if self._history:
                med = sorted(self._history)[len(self._history) // 2]
            else:
                med = -1.0

            # Floor-bounce detection
            fb = 1 if (REAR_FLOOR_LOW < med < REAR_FLOOR_HIGH) else 0

            with self._lock:
                self._dist = med
                self._floor_bounce = fb

            time.sleep(period)

# ─────────────────────────────────────────────────────────────────────────────
#  IMU  (MPU6050 — accelerometer + gyroscope)
# ─────────────────────────────────────────────────────────────────────────────

class IMU:
    """MPU6050 wrapper with yaw integration, crash detection, and PID
    heading correction."""

    def __init__(self):
        self._heading = 0.0          # integrated yaw (degrees)
        self._gz_offset = 0.0        # gyro-Z bias
        self._accel_baseline = {"x": 0.0, "y": 0.0, "z": 9.81}
        self._accel_noise = {"x": 0.05, "y": 0.05, "z": 0.2}
        self._crash_threshold_ms2 = CRASH_MS2
        self._calibrated = False
        self._last_time = time.time()

        # PID state
        self._integral = 0.0
        self._prev_error = 0.0

    def calibrate(self):
        """Collect CALIBRATION_SAMPLES to compute offsets and noise floor.
        Car MUST be stationary during calibration."""
        if not HAS_IMU:
            _log.warning("IMU_CALIBRATE skipped – no MPU6050")
            return {"status": "no_imu"}

        _log.info("IMU_CALIBRATE collecting %d samples …",
                  CALIBRATION_SAMPLES)

        gz_samples = []
        ax_samples = []
        ay_samples = []
        az_samples = []

        for i in range(CALIBRATION_SAMPLES):
            try:
                g = _imu.get_gyro_data()
                a = _imu.get_accel_data()
                gz_samples.append(g["z"])
                ax_samples.append(a["x"])
                ay_samples.append(a["y"])
                az_samples.append(a["z"])
            except Exception as exc:
                _log.debug("IMU_CALIBRATE sample %d error: %s", i, exc)
            time.sleep(CALIBRATION_INTERVAL)

        if len(gz_samples) < CALIBRATION_SAMPLES * 0.5:
            _log.error("IMU_CALIBRATE failed – only %d valid samples",
                       len(gz_samples))
            return {"status": "failed", "samples": len(gz_samples)}

        self._gz_offset = sum(gz_samples) / len(gz_samples)
        self._accel_baseline = {
            "x": sum(ax_samples) / len(ax_samples),
            "y": sum(ay_samples) / len(ay_samples),
            "z": sum(az_samples) / len(az_samples),
        }

        def _std(lst, mean):
            return math.sqrt(sum((v - mean) ** 2 for v in lst) / len(lst))

        self._accel_noise = {
            "x": _std(ax_samples, self._accel_baseline["x"]),
            "y": _std(ay_samples, self._accel_baseline["y"]),
            "z": _std(az_samples, self._accel_baseline["z"]),
        }

        self._calibrated = True
        self._heading = 0.0
        self._last_time = time.time()

        result = {
            "status": "ok",
            "samples": len(gz_samples),
            "gyro_z_offset": round(self._gz_offset, 4),
            "accel_baseline": {k: round(v, 4)
                               for k, v in self._accel_baseline.items()},
            "accel_noise": {k: round(v, 4)
                            for k, v in self._accel_noise.items()},
            "crash_threshold_g": CRASH_G,
            "crash_threshold_ms2": round(self._crash_threshold_ms2, 2),
            "note": "accel values are m/s2 (not g)",
        }
        _log.info("IMU_CALIBRATE_DONE %s", json.dumps(result))
        return result

    def update(self):
        """Read IMU, integrate yaw, return full snapshot.
        Call this every control cycle."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if not HAS_IMU or not self._calibrated:
            return {
                "heading": self._heading,
                "dt": dt,
                "accel": {"x": 0, "y": 0, "z": 9.81},
                "gyro": {"z": 0},
                "lateral_g": 0.0,
                "crash": False,
            }

        try:
            accel = _imu.get_accel_data()
            gyro = _imu.get_gyro_data()
        except Exception:
            return {
                "heading": self._heading,
                "dt": dt,
                "accel": {"x": 0, "y": 0, "z": 9.81},
                "gyro": {"z": 0},
                "lateral_g": 0.0,
                "crash": False,
            }

        # Integrate yaw from gyro-Z
        gz_corrected = gyro["z"] - self._gz_offset
        self._heading += gz_corrected * dt

        # Lateral acceleration (crash detection)
        dx = accel["x"] - self._accel_baseline["x"]
        dy = accel["y"] - self._accel_baseline["y"]
        lateral_ms2 = math.sqrt(dx * dx + dy * dy)
        lateral_g = lateral_ms2 / ACCEL_G

        crash = lateral_ms2 > self._crash_threshold_ms2

        return {
            "heading": self._heading,
            "dt": dt,
            "accel": accel,
            "gyro": gyro,
            "lateral_g": round(lateral_g, 3),
            "crash": crash,
        }

    def pid_correction(self, target_heading, dt):
        """PID correction for heading maintenance."""
        error = target_heading - self._heading
        self._integral += error * dt
        derivative = (error - self._prev_error) / max(dt, 0.001)
        self._prev_error = error
        correction = (GYRO_KP * error +
                      GYRO_KI * self._integral +
                      GYRO_KD * derivative)
        return correction

    def reset_heading(self):
        """Reset integrated heading to zero."""
        self._heading = 0.0
        self._integral = 0.0
        self._prev_error = 0.0

    @property
    def heading(self):
        return self._heading

    @property
    def calibrated(self):
        return self._calibrated

# ─────────────────────────────────────────────────────────────────────────────
#  SWERVE COMPUTATION
# ─────────────────────────────────────────────────────────────────────────────

def compute_swerve(sector_map, ir_left: bool, ir_right: bool,
                   sweep_ready: bool = True):
    """Given the sector map and IR states, compute:
      - swerve_angle  (degrees, + = steer right, - = steer left)
      - speed_factor  (0.4 … 1.0, fraction of BASE_SPEED)
      - threat_mag    (0 … 1+, for display)
      - boxed_in      (bool)

    Uses laser data as soon as available (per-sector confidence handles
    staleness).  IR always contributes for close-range override.
    Forced swerve ensures the car never drives straight into obstacles
    even when threats are roughly symmetric.
    """
    threat_x = 0.0   # lateral  (positive = obstacle on the right)
    threat_y = 0.0   # forward  (positive = obstacle ahead)
    min_dist = SECTOR_DEFAULT_CM     # global minimum across all sectors
    min_fwd  = SECTOR_DEFAULT_CM     # minimum in centre sectors only
    blocked_count = 0

    # Side-clearance accumulators for choosing escape direction
    left_sum   = 0.0;  left_cnt  = 0
    right_sum  = 0.0;  right_cnt = 0
    left_min   = SECTOR_DEFAULT_CM   # minimum distance on left side
    right_min  = SECTOR_DEFAULT_CM   # minimum distance on right side

    use_laser = sweep_ready

    if use_laser:
        for angle_deg, dist_cm, confidence in sector_map:
            if dist_cm < min_dist:
                min_dist = dist_cm

            # Forward-facing minimum (centre 3 sectors)
            if -15 <= angle_deg <= 15 and dist_cm < min_fwd:
                min_fwd = dist_cm

            # Side clearance tracking
            if angle_deg < -5:
                left_sum  += dist_cm;  left_cnt  += 1
                if dist_cm < left_min:
                    left_min = dist_cm
            elif angle_deg > 5:
                right_sum += dist_cm;  right_cnt += 1
                if dist_cm < right_min:
                    right_min = dist_cm

            if dist_cm >= WARN_DIST:
                continue

            if dist_cm < ALL_BLOCKED_CM:
                blocked_count += 1

            # Quadratic weight: much stronger reaction to close obstacles
            # Angle-based weighting: peripheral sectors (±60°) contribute
            # less than forward sectors to prevent violent swerve from a
            # single side reading (cos(60°)=0.5, clamped to 0.3 minimum)
            proximity = (WARN_DIST - dist_cm) / WARN_DIST
            angle_weight = max(0.3, math.cos(math.radians(angle_deg)))
            weight = (proximity ** 2) * confidence * angle_weight
            rad = math.radians(angle_deg)
            threat_x += weight * math.sin(rad)
            threat_y += weight * math.cos(rad)

    # IR provides a strong lateral shove
    if ir_left:
        threat_x -= 1.5          # obstacle on left → push threat left
    if ir_right:
        threat_x += 1.5          # obstacle on right → push threat right

    threat_mag = math.sqrt(threat_x * threat_x + threat_y * threat_y)

    # Boxed-in: trigger when surrounded or both IRs confirm.
    n_sectors = len(sector_map)
    if use_laser:
        boxed = ((blocked_count >= n_sectors * 0.5) and (ir_left or ir_right)) or \
                (ir_left and ir_right and min_dist < ALL_BLOCKED_CM) or \
                (blocked_count >= n_sectors * 0.7)
    else:
        boxed = ir_left and ir_right

    # ── Swerve angle: steer AWAY from the threat vector ──────────────
    # Amplify swerve proportionally to threat magnitude and proximity.
    # With weak motors, the raw atan2 angle is too small to cause real
    # turning — multiply by a gain that scales with how close obstacles are.
    if threat_mag < 0.05:
        swerve_angle = 0.0
    else:
        raw_angle = -math.degrees(math.atan2(threat_x, max(threat_y, 0.01)))
        # Gain: 1.2× baseline, up to 2.0× when obstacles are very close
        # (was 2.0× base / 4.0× max — way too aggressive, produced 60° swerve
        #  from mild asymmetric sectors and caused persistent circling)
        proximity_gain = 1.0 + (1.0 - min(min_fwd, WARN_DIST) / WARN_DIST)
        swerve_gain = 1.2 * proximity_gain   # 1.2–2.4× amplification
        swerve_angle = raw_angle * swerve_gain
        swerve_angle = max(-SWERVE_MAX_ANGLE, min(SWERVE_MAX_ANGLE, swerve_angle))

    # ── Forced swerve: when forward obstacle is close but computed    ──
    # ── swerve is too small (symmetric threats cancel out),           ──
    # ── pick the clearest side and guarantee a minimum swerve.        ──
    left_avg  = left_sum  / max(1, left_cnt)
    right_avg = right_sum / max(1, right_cnt)

    # Determine which side is truly clear (using both avg and min)
    right_viable = right_min >= SIDE_CLEAR_MIN_CM
    left_viable  = left_min  >= SIDE_CLEAR_MIN_CM

    if use_laser and min_fwd < HARD_SWERVE_DIST and abs(swerve_angle) < MIN_FORCED_SWERVE:
        # Force swerve toward clearer side — even when neither side meets
        # SIDE_CLEAR_MIN_CM, bias toward the one with more room.
        # This prevents driving straight into obstacles when symmetric
        # threats cancel the threat vector (swerve ≈ 0).
        if right_viable or left_viable:
            proximity = 1.0 - (min_fwd / HARD_SWERVE_DIST)
            forced = MIN_FORCED_SWERVE + (SWERVE_MAX_ANGLE - MIN_FORCED_SWERVE) * proximity
            if right_viable and (not left_viable or right_avg >= left_avg):
                swerve_angle = max(swerve_angle, forced)
            else:
                swerve_angle = min(swerve_angle, -forced)
        elif right_avg > 0 or left_avg > 0:
            # Neither side meets SIDE_CLEAR_MIN_CM but bias toward
            # whichever has more room — better than driving straight
            proximity = 1.0 - (min_fwd / HARD_SWERVE_DIST)
            forced = MIN_FORCED_SWERVE * (0.5 + 0.5 * proximity)
            if right_avg >= left_avg:
                swerve_angle = max(swerve_angle, forced)
            else:
                swerve_angle = min(swerve_angle, -forced)
        # else: no data at all — boxed-in will trigger later

    # ── Emergency max-swerve for extremely close forward obstacles ────
    # Only force max-swerve if the escape side is genuinely clear;
    # otherwise, mark as boxed-in (both sides and front blocked).
    if use_laser and min_fwd < EMERGENCY_FWD_CM:
        if right_viable and (not left_viable or right_avg >= left_avg):
            swerve_angle = SWERVE_MAX_ANGLE
        elif left_viable:
            swerve_angle = -SWERVE_MAX_ANGLE
        else:
            # Neither side has enough clearance — flag as boxed-in
            boxed = True

    # IR overrides AFTER forced/emergency swerve — IR has FINAL authority
    # because it's the closest-range sensor and most reliable
    if ir_left and not ir_right:
        swerve_angle = max(swerve_angle, IR_SWERVE_BOOST)
        # If close-range IR fires, force away from obstacle at max swerve
        if min_fwd < EMERGENCY_FWD_CM:
            swerve_angle = SWERVE_MAX_ANGLE
    elif ir_right and not ir_left:
        swerve_angle = min(swerve_angle, -IR_SWERVE_BOOST)
        if min_fwd < EMERGENCY_FWD_CM:
            swerve_angle = -SWERVE_MAX_ANGLE

    # Speed reduction: primarily based on forward obstacles.
    # Side obstacles contribute at reduced weight (×2.0 factor scales them
    # up before comparison) so the car slows only when side obstacles are
    # very close, not just because something is at 60° periphery.
    side_influence = min(min_fwd, min_dist * 2.0) if min_dist < SECTOR_DEFAULT_CM else min_fwd
    overall_min = max(min_fwd * 0.7, side_influence)  # forward biased
    if overall_min < CRITICAL_DIST:
        speed_factor = max(SPEED_FLOOR_FACTOR,
                           overall_min / CRITICAL_DIST)
    else:
        speed_factor = 1.0

    return swerve_angle, speed_factor, threat_mag, boxed


def differential_speeds(base_speed: float, swerve_angle: float):
    """Convert swerve angle to (left_speed, right_speed) for differential drive.
    Positive swerve = steer right → left wheels faster, right wheels slower."""
    turn_factor = abs(swerve_angle) / SWERVE_MAX_ANGLE
    inner_factor = max(INNER_WHEEL_MIN, 1.0 - turn_factor * (1.0 - INNER_WHEEL_MIN))

    if swerve_angle > 0:
        # steer right → right wheels are inner
        left_speed  = base_speed
        right_speed = base_speed * inner_factor
    elif swerve_angle < 0:
        # steer left → left wheels are inner
        left_speed  = base_speed * inner_factor
        right_speed = base_speed
    else:
        left_speed  = base_speed
        right_speed = base_speed

    return left_speed, right_speed

# ─────────────────────────────────────────────────────────────────────────────
#  MAIN CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────

class SlalomController:
    """Top-level state machine orchestrating all subsystems."""

    def __init__(self):
        self.state = State.CALIBRATING
        self._running = False
        self._start_time = time.time()
        self._cycle_count = 0
        self._telem = {}
        self._telem_lock = threading.Lock()

        # Subsystems
        self.motor   = MotorDriver()
        self.scanner = LaserScanner()
        self.ir      = IRSensors()
        self.sonar   = RearSonar()
        self.imu     = IMU()

        # Counters
        self._crash_count = 0
        self._ir_left_count = 0
        self._ir_right_count = 0
        self._boxed_in_count = 0
        self._last_crash_time = 0.0
        self._last_boxed_recovery_time = 0.0

        # Heading tracking
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0

        # Drive thread
        self._drive_thread = None

        _log.info("========================================================================")
        _log.info("AUTO_SLALOM INIT  pid=%d", os.getpid())
        _log.info("========================================================================")

        self._set_state(State.CALIBRATING)

    def get_telemetry(self):
        """Return a snapshot of the current telemetry dict."""
        with self._telem_lock:
            return dict(self._telem)

    def _update_telem(self, **kw):
        """Update telemetry dict with provided key-value pairs."""
        with self._telem_lock:
            self._telem.update(kw)

    def _set_state(self, new_state):
        """Transition to a new state, logging the change."""
        old = self.state
        self.state = new_state
        _log.info("STATE_CHANGE %s → %s", old.name, new_state.name)

    def calibrate(self):
        """Calibrate the IMU. Call before starting to drive."""
        result = self.imu.calibrate()
        _log.info("CALIBRATION_COMPLETE result=%s", json.dumps(result))
        self._set_state(State.READY)
        return result

    def start_driving(self):
        """Start scanner, sonar, wait for first sweep, then begin driving."""
        self.scanner.start()

        # Wait for first scanner sweep
        _log.info("WAITING_FOR_SCANNER first sweep...")
        t_wait = time.time()
        while self.scanner.sweep_count < 1 and (time.time() - t_wait) < 5.0:
            time.sleep(0.02)
        _log.info("SCANNER_READY sweeps=%d (%.1fms wait)",
                  self.scanner.sweep_count,
                  (time.time() - t_wait) * 1000)

        self.sonar.start()

        self._running = True
        self._start_time = time.time()
        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0

        self._set_state(State.DRIVING)
        _log.info("DRIVING_START base_speed=%d", BASE_SPEED)

        self._drive_thread = threading.Thread(target=self._drive_loop,
                                               daemon=True)
        self._drive_thread.start()

    def pause(self):
        """Pause driving. Motors coast."""
        if self.state == State.DRIVING:
            self._running = False
            self.motor.coast()
            self._set_state(State.PAUSED)

    def resume(self):
        """Resume driving after pause."""
        if self.state == State.PAUSED:
            self._running = True
            self.imu.reset_heading()
            self._target_heading = 0.0
            self._heading_at_swerve_start = 0.0
            self._set_state(State.DRIVING)
            self._drive_thread = threading.Thread(target=self._drive_loop,
                                                   daemon=True)
            self._drive_thread.start()

    def stop(self):
        """Stop driving completely."""
        self._running = False
        self.motor.coast()

    # ── MAIN DRIVE LOOP ─────────────────────────────────────────────────

    def _drive_loop(self):
        _log.info("DRIVE_LOOP_START hz=%d", LOOP_HZ)
        prev_ir_l = False
        prev_ir_r = False
        prev_swerve = 0.0          # for swerve smoothing (EMA)
        cycle = 0

        while self._running and self.state in (State.DRIVING,
                                                 State.BOXED_IN,
                                                 State.CRASH_RECOVERY):
            t_start = time.time()
            cycle += 1
            self._cycle_count = cycle

            # ── 1. read sensors ──────────────────────────────────────
            imu_data    = self.imu.update()
            ir_l, ir_r  = self.ir.read()
            sector_map  = self.scanner.get_sector_map()
            rear_d, rear_fb = self.sonar.read()
            min_fwd     = self.scanner.get_min_forward_distance()

            # IR activation counters
            if ir_l and not prev_ir_l:
                self._ir_left_count += 1
                _log.warning("IR_LEFT_TRIGGERED count=%d", self._ir_left_count)
            if ir_r and not prev_ir_r:
                self._ir_right_count += 1
                _log.warning("IR_RIGHT_TRIGGERED count=%d", self._ir_right_count)
            prev_ir_l = ir_l
            prev_ir_r = ir_r

            # ── 2. crash detection ───────────────────────────────────
            if (imu_data["crash"] and
                    self.state == State.DRIVING and
                    (t_start - self._last_crash_time) > CRASH_COOLDOWN):
                self._last_crash_time = t_start
                self._crash_count += 1
                _log.critical(
                    "CRASH_DETECTED count=%d lateral_g=%.3f "
                    "accel=[%.3f,%.3f,%.3f] gyro_z=%.3f heading=%.2f",
                    self._crash_count,
                    imu_data["lateral_g"],
                    imu_data["accel"]["x"], imu_data["accel"]["y"],
                    imu_data["accel"]["z"],
                    imu_data["gyro"]["z"],
                    imu_data["heading"],
                )
                self._do_crash_recovery(rear_d)
                continue

            # ── 3. state-dependent behaviour ─────────────────────────
            if self.state == State.DRIVING:
                sweep_ready = self.scanner.sweep_count >= GRACE_SWEEPS
                raw_swerve, speed_factor, threat_mag, boxed = \
                    compute_swerve(sector_map, ir_l, ir_r,
                                   sweep_ready=sweep_ready)

                # Smooth the swerve angle via EMA to prevent jerky steering.
                # IR-triggered swerves bypass smoothing for instant reaction.
                if ir_l or ir_r:
                    swerve_angle = raw_swerve   # instant IR reaction
                    prev_swerve = raw_swerve
                else:
                    swerve_angle = (SWERVE_SMOOTH_ALPHA * raw_swerve +
                                    (1.0 - SWERVE_SMOOTH_ALPHA) * prev_swerve)
                    prev_swerve = swerve_angle

                if boxed:
                    # Cooldown: don't re-trigger boxed-in within 2s of
                    # last recovery — prevents instant re-entry loop
                    if (t_start - self._last_boxed_recovery_time) < 2.0:
                        _log.debug("BOXED_IN_SUPPRESSED cooldown=%.1fs",
                                   t_start - self._last_boxed_recovery_time)
                    else:
                        self._boxed_in_count += 1
                        _log.warning("BOXED_IN count=%d min_fwd=%.1f ir=[%s,%s]",
                                     self._boxed_in_count, min_fwd,
                                     ir_l, ir_r)
                        self._do_boxed_in_recovery()
                        continue

                # ── Circle detection: if the car has rotated too far ──
                # ── from its swerve-start heading, it's spinning.     ──
                heading_delta = abs(imu_data["heading"] - self._heading_at_swerve_start)
                if heading_delta > CIRCLE_HEADING_LIMIT and min_fwd < WARN_DIST:
                    self._boxed_in_count += 1
                    _log.warning(
                        "CIRCLE_DETECTED count=%d heading_delta=%.1f "
                        "min_fwd=%.1f – triggering recovery",
                        self._boxed_in_count, heading_delta, min_fwd)
                    self._do_boxed_in_recovery()
                    continue

                # PID heading correction when driving fairly straight
                dt = imu_data["dt"]
                if abs(swerve_angle) < 15:
                    correction = self.imu.pid_correction(
                        self._target_heading, dt)
                    swerve_angle += correction * 0.3
                    # Reset circle-detection anchor when driving straight
                    self._heading_at_swerve_start = imu_data["heading"]
                else:
                    # re-anchor heading so PID doesn't fight swerves
                    self._target_heading = self.imu.heading

                # ── Heading-rate damping: reduce swerve when heading  ──
                # ── has accumulated significantly in one direction.   ──
                # ── Prevents persistent one-direction turning that    ──
                # ── causes circling.  IR-triggered swerves exempt.    ──
                if not (ir_l or ir_r):
                    heading_delta = abs(imu_data["heading"] -
                                        self._heading_at_swerve_start)
                    if heading_delta > HEADING_DAMP_START and abs(swerve_angle) > 10:
                        damp_frac = min(HEADING_DAMP_MAX,
                                        HEADING_DAMP_MAX *
                                        (heading_delta - HEADING_DAMP_START) /
                                        (HEADING_DAMP_FULL - HEADING_DAMP_START))
                        swerve_angle *= (1.0 - damp_frac)

                # Compute wheel speeds
                effective_speed = BASE_SPEED * speed_factor
                if abs(swerve_angle) > 25:
                    # ACTIVE SWERVE: differential steering for obstacle
                    # avoidance.  Outer wheel slightly boosted, inner wheel
                    # stays at fraction of BASE_SPEED for forward progress.
                    turn_ratio = abs(swerve_angle) / SWERVE_MAX_ANGLE
                    outer_speed = BASE_SPEED * SWERVE_OUTER_BOOST
                    inner_speed = BASE_SPEED * max(INNER_WHEEL_MIN,
                                                    1.0 - turn_ratio)
                    if swerve_angle > 0:  # steer right → left=outer
                        left_spd, right_spd = outer_speed, inner_speed
                    else:                 # steer left  → right=outer
                        left_spd, right_spd = inner_speed, outer_speed
                else:
                    if abs(swerve_angle) > 10:
                        effective_speed = max(effective_speed,
                                             BASE_SPEED * SWERVE_SPEED_FLOOR)
                    left_spd, right_spd = differential_speeds(
                        effective_speed, swerve_angle)

                # Drive
                self.motor.forward_differential(left_spd, right_spd)

                # Update telemetry
                self._update_telem(
                    swerve_angle=round(swerve_angle, 1),
                    speed_factor=round(speed_factor, 2),
                    threat_mag=round(threat_mag, 2),
                    left_speed=round(left_spd, 1),
                    right_speed=round(right_spd, 1),
                    ir_left=ir_l,
                    ir_right=ir_r,
                    rear_dist=rear_d,
                    rear_floor=rear_fb,
                    heading=imu_data["heading"],
                    accel=imu_data["accel"],
                    gyro_z=round(imu_data["gyro"].get("z", 0), 2),
                    lateral_g=imu_data["lateral_g"],
                    min_forward=round(min_fwd, 1),
                    sector_map=[(a, round(d, 1), round(c, 2))
                                for a, d, c in sector_map],
                    crash_count=self._crash_count,
                    ir_left_count=self._ir_left_count,
                    ir_right_count=self._ir_right_count,
                    boxed_in_count=self._boxed_in_count,
                    cycle_hz=round(1.0 / max(0.001,
                                   time.time() - t_start), 1),
                    uptime=round(time.time() - self._start_time, 1),
                )

            # ── 4. comprehensive sensor log ──────────────────────────
            if cycle % SENSOR_LOG_EVERY_N == 0:
                sectors_str = " ".join(
                    f"{a}:{d:.0f}" for a, d, _ in sector_map)
                _log.debug(
                    "CYCLE=%d st=%s spd=[%.1f,%.1f] swerve=%.1f "
                    "sf=%.2f threat=%.2f min_fwd=%.1f "
                    "ir=[%d,%d] rear=%.1f(fb=%d) "
                    "hdg=%.1f gz=%.2f lat_g=%.3f "
                    "ax=%.3f ay=%.3f az=%.3f "
                    "sectors=[%s]",
                    cycle,
                    self.state.name,
                    self._telem.get("left_speed", 0),
                    self._telem.get("right_speed", 0),
                    self._telem.get("swerve_angle", 0),
                    self._telem.get("speed_factor", 1),
                    self._telem.get("threat_mag", 0),
                    min_fwd,
                    ir_l, ir_r,
                    rear_d, rear_fb,
                    imu_data["heading"],
                    imu_data["gyro"].get("z", 0),
                    imu_data["lateral_g"],
                    imu_data["accel"].get("x", 0),
                    imu_data["accel"].get("y", 0),
                    imu_data["accel"].get("z", 0),
                    sectors_str,
                )

            # ── 5. pace the loop ─────────────────────────────────────
            elapsed = time.time() - t_start
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        _log.info("DRIVE_LOOP_END cycles=%d", cycle)

    # ── BOXED-IN recovery ────────────────────────────────────────────────

    def _do_boxed_in_recovery(self):
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.3)

        rear_d, _ = self.sonar.read()
        _log.info("BOXED_IN_RECOVERY rear_dist=%.1f", rear_d)

        if rear_d < 0 or rear_d < REAR_CLEAR_CM:
            _log.error("FULLY_STUCK rear_blocked=%.1f – waiting for manual "
                       "intervention (press R)", rear_d)
            self.motor.coast()
            # Stay in BOXED_IN; user presses R to retry
            self._running = False
            return

        # Reverse with rear sonar monitoring
        _log.info("REVERSE_START speed=%d duration=%.1fs",
                  REVERSE_SPEED, REVERSE_DURATION)
        self.motor.reverse_straight(REVERSE_SPEED)
        t0 = time.time()

        while time.time() - t0 < REVERSE_DURATION and self._running:
            rd, rfb = self.sonar.read()
            _log.debug("REVERSING rear=%.1f floor=%d", rd, rfb)
            if 0 < rd < REAR_STOP_CM:
                _log.warning("REVERSE_ABORT rear=%.1f < %d cm",
                             rd, REAR_STOP_CM)
                break
            time.sleep(0.05)

        self.motor.brake()
        _log.info("REVERSE_END")

        # Wait for a fresh scanner sweep after reversing — stale sector
        # data from pre-reverse position leads to wrong turn direction
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.5):
            time.sleep(0.02)
        _log.debug("POST_REVERSE_SCAN_WAIT sweeps=%d (%.0fms)",
                   self.scanner.sweep_count - sweep_before,
                   (time.time() - t_wait) * 1000)

        # Determine which side has more room and do a wide turn
        sector_map = self.scanner.get_sector_map()
        left_dists  = [d for a, d, c in sector_map if a < -5]
        right_dists = [d for a, d, c in sector_map if a > 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))

        if left_avg >= right_avg:
            _log.info("RECOVERY_TURN LEFT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            # Turn left: right wheels faster
            self.motor.forward_differential(BASE_SPEED * 0.3,
                                            BASE_SPEED * 0.9)
        else:
            _log.info("RECOVERY_TURN RIGHT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            self.motor.forward_differential(BASE_SPEED * 0.9,
                                            BASE_SPEED * 0.3)

        time.sleep(RECOVERY_TURN_DURATION)
        self.motor.coast()
        time.sleep(0.1)

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    # ── CRASH recovery ───────────────────────────────────────────────────

    def _do_crash_recovery(self, rear_dist):
        """Handle crash event: brake, assess, reverse if possible."""
        self._set_state(State.CRASH_RECOVERY)

        _log.info("CRASH_RECOVERY_START – braking, waiting 1 s")
        self.motor.brake()
        time.sleep(1.0)

        # Take post-crash IMU reading
        heading_before = self.imu.heading
        imu_snap = self.imu.update()
        heading_after = imu_snap["heading"]

        _log.info("CRASH_POST heading_before=%.1f heading_after=%.1f "
                  "accel=[%.3f,%.3f,%.3f]",
                  heading_before, heading_after,
                  imu_snap["accel"].get("x", 0),
                  imu_snap["accel"].get("y", 0),
                  imu_snap["accel"].get("z", 0))

        # Try to reverse away from the crash site
        rd, _ = self.sonar.read()
        if rd < 0 or rd > REAR_CLEAR_CM:
            _log.info("CRASH_REVERSE speed=%d dur=%.1fs rear=%.1f",
                      CRASH_REVERSE_SPEED, CRASH_REVERSE_DURATION, rd)
            self.motor.reverse_straight(CRASH_REVERSE_SPEED)
            t0 = time.time()
            while time.time() - t0 < CRASH_REVERSE_DURATION and self._running:
                rd2, _ = self.sonar.read()
                if 0 < rd2 < REAR_STOP_CM:
                    _log.warning("CRASH_REVERSE_ABORT rear=%.1f", rd2)
                    break
                time.sleep(0.05)
            self.motor.brake()
        else:
            _log.warning("CRASH_NO_REVERSE rear_blocked=%.1f", rd)

        time.sleep(0.3)
        self.motor.coast()

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0

        _log.info("CRASH_RECOVERY_DONE count=%d", self._crash_count)
        self._set_state(State.DRIVING)

    # ── SHUTDOWN ─────────────────────────────────────────────────────────

    def shutdown(self):
        """Cleanly shut down all subsystems."""
        self._running = False
        self._set_state(State.SHUTDOWN)
        _log.info("SHUTDOWN requested")

        self.motor.coast()
        time.sleep(0.2)
        self.motor.cleanup()
        self.scanner.cleanup()
        self.sonar.stop()

        uptime = time.time() - self._start_time
        _log.info("========================================================================")
        _log.info(
            "SHUTDOWN_COMPLETE  uptime=%.1fs  crashes=%d  "
            "ir_left=%d  ir_right=%d  boxed_in=%d  cycles=%d",
            uptime,
            self._crash_count,
            self._ir_left_count,
            self._ir_right_count,
            self._boxed_in_count,
            self._cycle_count,
        )
        _log.info("========================================================================")


# ─────────────────────────────────────────────────────────────────────────────
#  CURSES UI
# ─────────────────────────────────────────────────────────────────────────────

def _curses_main(stdscr, controller):
    """Full-screen curses display with keyboard controls."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(int(1000 / UI_REFRESH_HZ))

    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)   # OK
    curses.init_pair(2, curses.COLOR_YELLOW, -1)   # WARN
    curses.init_pair(3, curses.COLOR_RED,    -1)   # DANGER
    curses.init_pair(4, curses.COLOR_CYAN,   -1)   # INFO
    curses.init_pair(5, curses.COLOR_WHITE,  -1)   # HEADER

    COL_OK   = curses.color_pair(1)
    COL_WARN = curses.color_pair(2)
    COL_DANG = curses.color_pair(3)
    COL_INFO = curses.color_pair(4)
    COL_HDR  = curses.color_pair(5) | curses.A_BOLD

    _log.info("UI_START curses terminal")

    while True:
        key = stdscr.getch()
        ch = chr(key).lower() if 0 < key < 256 else ""

        if ch == "q":
            _log.info("USER_QUIT")
            break
        elif ch == "s":
            if controller.state == State.READY:
                controller.start_driving()
        elif ch == "p":
            controller.pause()
        elif ch == "r":
            if controller.state == State.PAUSED:
                controller.resume()
            elif controller.state == State.BOXED_IN:
                controller._running = True
                controller.imu.reset_heading()
                controller._target_heading = 0.0
                controller._heading_at_swerve_start = 0.0
                controller._last_boxed_recovery_time = time.time()
                controller._set_state(State.DRIVING)
                controller._drive_thread = threading.Thread(
                    target=controller._drive_loop, daemon=True)
                controller._drive_thread.start()

        # ── draw ─────────────────────────────────────────────────────
        stdscr.erase()
        t = controller.get_telemetry()

        def _safe(row, col, text, attr=0):
            try:
                stdscr.addnstr(row, col, str(text), stdscr.getmaxyx()[1] - col - 1, attr)
            except curses.error:
                pass

        row = 0
        state_col = COL_OK
        if controller.state in (State.BOXED_IN, State.CRASH_RECOVERY):
            state_col = COL_DANG
        elif controller.state == State.PAUSED:
            state_col = COL_WARN

        header = f"  AUTO SLALOM  |  {controller.state.name}  |  " \
                 f"Cycles: {controller._cycle_count}"
        _safe(row, 0, header, COL_HDR)
        row += 2

        # Sector map bar chart
        sm = t.get("sector_map", [])
        bar_max = 30
        for angle, dist, conf in sm:
            bar_len = int(min(dist, 120) / 120 * bar_max)
            bar_char = "█" * bar_len
            bar_empty = "░" * (bar_max - bar_len)
            col = COL_OK if dist > WARN_DIST else (COL_WARN if dist > CRITICAL_DIST else COL_DANG)
            label = f"{angle:+4d}° {dist:5.0f}cm "
            _safe(row, 0, label + bar_char + bar_empty, col)
            row += 1

        row += 1

        # Speed / swerve
        _safe(row, 0,
              f"Speed: L={t.get('left_speed', 0):5.1f}"
              f"%  R={t.get('right_speed', 0):5.1f}"
              f"%  |  Swerve: {t.get('swerve_angle', 0):+6.1f}°"
              f"  SF={t.get('speed_factor', 1):.2f}"
              f"  Threat={t.get('threat_mag', 0):.2f}",
              COL_INFO)
        row += 1

        # IR status
        ir_l_str = "■ LEFT"  if t.get("ir_left", False) else "□ left"
        ir_r_str = "■ RIGHT" if t.get("ir_right", False) else "□ right"
        ir_l_col = COL_DANG if t.get("ir_left", False) else COL_OK
        ir_r_col = COL_DANG if t.get("ir_right", False) else COL_OK
        _safe(row, 0, f"IR: ", COL_INFO)
        _safe(row, 4, ir_l_str, ir_l_col)
        _safe(row, 15, ir_r_str, ir_r_col)

        # Rear sonar
        rd = t.get("rear_dist", -1)
        rear_col = COL_OK if rd > REAR_CLEAR_CM else (COL_WARN if rd > REAR_STOP_CM else COL_DANG)
        flr = " FLOOR" if t.get("rear_floor", 0) else ""
        _safe(row, 30, f"Rear: {rd:.1f}cm{flr}", rear_col)
        row += 1

        # Heading
        _safe(row, 0,
              f"Heading: {t.get('heading', 0):.1f}°  "
              f"Gyro-Z: {t.get('gyro_z', 0):.1f}  "
              f"Lat-G: {t.get('lateral_g', 0):.3f}",
              COL_INFO)
        row += 1

        # Counters
        _safe(row, 0,
              f"Crashes: {t.get('crash_count', 0)}  "
              f"IR-L: {t.get('ir_left_count', 0)}  "
              f"IR-R: {t.get('ir_right_count', 0)}  "
              f"Boxed: {t.get('boxed_in_count', 0)}  "
              f"Hz: {t.get('cycle_hz', 0):.0f}  "
              f"Up: {t.get('uptime', 0):.0f}s",
              COL_INFO)
        row += 2

        # Recent log lines
        a = row
        n_log_lines = max(0, stdscr.getmaxyx()[0] - a - 3)
        recent = list(_ui_log_lines)[-n_log_lines:]
        for line in recent:
            _safe(row, 0, line)
            row += 1

        # Controls bar
        ctrl_row = stdscr.getmaxyx()[0] - 1
        controls = "  S=Start  P=Pause  R=Resume/Retry  Q=Quit  "
        _safe(ctrl_row, 0, controls, COL_HDR)

        stdscr.refresh()


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    controller = SlalomController()

    def _sighandler(sig, frame):
        _log.info("SIGNAL_%s received",
                  signal.Signals(sig).name if hasattr(signal, "Signals")
                  else sig)
        controller.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    try:
        controller.calibrate()
        curses.wrapper(lambda stdscr: _curses_main(stdscr, controller))
    except Exception as exc:
        _log.critical("FATAL %s", exc, exc_info=True)
    finally:
        controller.shutdown()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
