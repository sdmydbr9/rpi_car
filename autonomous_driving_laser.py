#!/usr/bin/env python3
"""
autonomous_driving_laser.py — Autonomous Laser-Scanning Driving Controller
=========================================================================
Standalone script for autonomous obstacle-avoidance driving using:
  • VL53L0X laser on servo for continuous 120° sector scanning
  • Front IR sensors (left + right) for close-range detection
  • MPU6050 accelerometer for crash detection, gyro for yaw tracking
  • Rear HC-SR04 sonar for safe reversing
  • Differential steering for smooth swerving (no pivot turns)

Run:  python3 autonomous_driving_laser.py
Controls:  S = Start | P = Pause | R = Resume/Retry | Q = Quit
Logs:      autonomous_driving_laser.logs (append mode, comprehensive timestamped data)
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
BASE_SPEED          = 35        # forward cruise PWM % (reduced from 40 for better reaction time)
MAX_PWM_DUTY        = 63        # voltage cap  (7 V / 11.1 V × 100)
PWM_FREQ            = 1000      # Hz
REVERSE_SPEED       = 40
REVERSE_DURATION    = 1.0       # seconds
RECOVERY_TURN_DURATION = 1.5    # seconds of wide turn after reverse (was 1.0 — too short to reorient)
CRASH_REVERSE_SPEED = 35
CRASH_REVERSE_DURATION = 0.5
DIRECTION_CHANGE_DWELL = 0.15   # seconds between fwd↔rev

# Per-wheel trims  (calibrated to make the car go straight)
FL_TRIM = 0.60
FR_TRIM = 0.60
RL_TRIM = 1.00
RR_TRIM = 1.00

# Obstacle distances (cm)
WARN_DIST           = 80        # start swerving — increased from 60 for earlier reaction
CRITICAL_DIST       = 40        # reduce speed — increased from 30 to slow down earlier
ALL_BLOCKED_CM      = 22        # every forward sector is this close → boxed in (raised from 18)
EMERGENCY_IR_DIST   = 0         # IR triggers (binary, active LOW)
HARD_SWERVE_DIST    = 60        # force min swerve when forward obstacle nearer (was 55 — still not reacting early enough)
MIN_FORCED_SWERVE   = 45        # minimum swerve degrees when forced (was 35 — not acute enough)
EMERGENCY_FWD_CM    = 30        # emergency max-swerve when centre sectors this close (was 25 — triggered too late)
TANK_TURN_DIST      = 20        # cm (200mm) — full tank turn (spin in place) at this distance or less
TANK_TURN_SPEED     = 40        # PWM % for each side during tank turn
SIDE_CLEAR_MIN_CM   = 20        # side must have at least this to be considered clear
CIRCLE_HEADING_LIMIT = 120      # degrees — trigger boxed-in if car rotates this much
# Stuck detection — car not turning despite close obstacles
STUCK_HEADING_THRESH = 5.0      # degrees — max heading change to count as "not turning"
STUCK_CYCLE_LIMIT    = 50       # cycles (~1.0s) — trigger recovery after this many stuck cycles (raised from 35; corrected servo should prevent most stuck)
STUCK_MIN_FWD_CM     = 50       # cm — only count as stuck when forward obstacle is this close (was 35 — missed stuck at 42cm)

# Swerve geometry
SWERVE_MAX_ANGLE    = 85        # max differential angle (degrees)  (was 70 — not acute enough, car still hit obstacles)
IR_SWERVE_BOOST     = 65        # added degrees when IR fires  (was 55 — increased for harder swerve)
IR_CLOSE_MAX_SWERVE_CM = 25     # cm — full differential (100,0 / 0,100) below this distance
IR_STUCK_REVERSE_DUR   = 0.5    # seconds to reverse when single IR fires
SPEED_FLOOR_FACTOR  = 0.45      # minimum speed fraction near obstacles (was 0.55 — too fast near walls)
INNER_WHEEL_MIN     = 0.12      # inner wheels keep 12% speed — stronger turn (was 0.20 — not enough differential)
SWERVE_SPEED_FLOOR  = 1.00      # outer wheel stays at full BASE_SPEED during swerve
SWERVE_OUTER_BOOST  = 1.05      # slight boost to outer wheel  (was 1.15 — too aggressive differential)
SWERVE_SMOOTH_ALPHA = 0.70      # EMA smoothing factor — raised from 0.60 for faster swerve response

# Heading-rate damping — prevents persistent one-direction swerve (circling)
HEADING_DAMP_START  = 30        # degrees — start damping after this much heading change
HEADING_DAMP_FULL   = 120       # degrees — full damping at this heading change
HEADING_DAMP_MAX    = 0.45      # max swerve reduction factor (0–1) — reduced from 0.70 to allow stronger swerves

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
CRASH_COOLDOWN      = 1.5       # seconds between consecutive crash events (reduced from 3.0 — missed 2nd impact)

# Navigation — proactive "navigate toward open space" approach
NAVIGATE_DIST       = 80        # cm – start looking for best direction when fwd closer
BEST_DIR_ADVANTAGE  = 30        # cm – best direction must have this much more room than fwd
TARGET_HEADING_TOL  = 12        # degrees – turn considered complete within this tolerance
TURN_OUTER_SPEED    = 55        # PWM % for outer wheels during active heading turns (was 50)
TURN_INNER_SPEED    = 10        # PWM % for inner wheels (was 15 — not enough differential to avoid obstacles)
MIN_STEER_ANGLE     = 20        # degrees – below this, motors can't actually turn; drive straight
CRUISE_CLEAR_DIST   = 120       # cm – when forward is this clear, just cruise straight
NAV_SMOOTH_ALPHA    = 0.3       # EMA smoothing for target heading updates (prevents oscillation)

# MPU resilience
MPU_MAX_CONSEC_ERRORS = 10      # consecutive MPU read failures → switch to fallback mode
MPU_RETRY_INTERVAL    = 50      # cycles – how often to retry MPU in fallback mode

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

class AvoidanceZone(Enum):
    """FSM obstacle avoidance zones — steering aggression scales with proximity."""
    CLEAR    = auto()   # > WARN_DIST (80cm): cruise normally
    CAUTION  = auto()   # CRITICAL_DIST .. WARN_DIST (40-80cm): moderate swerve
    DANGER   = auto()   # TANK_TURN_DIST .. CRITICAL_DIST (20-40cm): hard swerve, minimal forward
    CRITICAL = auto()   # ≤ TANK_TURN_DIST (20cm / 200mm): tank turn — spin in place

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING SETUP  →  slalom.logs
# ─────────────────────────────────────────────────────────────────────────────

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "autonomous_driving_laser.logs")

_file_formatter = logging.Formatter(
    fmt="%(asctime)s.%(msecs)03d | %(levelname)-5s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

_log = logging.getLogger("autonomous_driving_laser")
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

    def tank_turn(self, direction, speed):
        """Spin in place: one side forward, other side backward.
        direction: 'left' (rotate CCW) or 'right' (rotate CW).
        speed: PWM % for each side (before trims)."""
        # Coast briefly to avoid H-bridge shoot-through
        for p in (self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr):
            p.ChangeDutyCycle(0)
        for p in self._dir_pins:
            GPIO.output(p, GPIO.LOW)
        time.sleep(DIRECTION_CHANGE_DWELL)

        if direction == 'left':
            # Left wheels backward, right wheels forward → rotate left (CCW)
            GPIO.output(FL_IN1, GPIO.LOW);  GPIO.output(FL_IN2, GPIO.HIGH)
            GPIO.output(RL_IN1, GPIO.LOW);  GPIO.output(RL_IN2, GPIO.HIGH)
            GPIO.output(FR_IN3, GPIO.HIGH); GPIO.output(FR_IN4, GPIO.LOW)
            GPIO.output(RR_IN3, GPIO.HIGH); GPIO.output(RR_IN4, GPIO.LOW)
        else:
            # Left wheels forward, right wheels backward → rotate right (CW)
            GPIO.output(FL_IN1, GPIO.HIGH); GPIO.output(FL_IN2, GPIO.LOW)
            GPIO.output(RL_IN1, GPIO.HIGH); GPIO.output(RL_IN2, GPIO.LOW)
            GPIO.output(FR_IN3, GPIO.LOW);  GPIO.output(FR_IN4, GPIO.HIGH)
            GPIO.output(RR_IN3, GPIO.LOW);  GPIO.output(RR_IN4, GPIO.HIGH)

        self._last_dir = f"tank_{direction}"
        self._set_duty(speed * FL_TRIM, speed * FR_TRIM,
                       speed * RL_TRIM, speed * RR_TRIM)

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
        Servo-space = 90 - user_deg  (negated so +angle = physical RIGHT).
        Duty = 2 + servo_deg / 18."""
        servo_deg = 90 - user_deg
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
        """Return minimum distance in the forward sectors (±25°)."""
        smap = self.get_sector_map()
        centre_dists = [d for a, d, c in smap if -25 <= a <= 25]
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

        # MPU error tracking
        self._consecutive_errors = 0

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
            self._last_time = now
            return {
                "heading": self._heading,
                "dt": dt,
                "accel": {"x": 0, "y": 0, "z": 9.81},
                "gyro": {"z": 0},
                "lateral_g": 0.0,
                "crash": False,
                "mpu_valid": False,
                "mpu_errors": self._consecutive_errors,
            }

        try:
            accel = _imu.get_accel_data()
            gyro = _imu.get_gyro_data()
        except Exception as exc:
            self._consecutive_errors += 1
            # Print to console so user sees MPU issues in real-time
            if self._consecutive_errors == 1 or self._consecutive_errors % 5 == 0:
                print(f"\n⚠ MPU6050 READ ERROR #{self._consecutive_errors}: {exc}")
            _log.error("MPU_READ_ERROR count=%d err=%s",
                       self._consecutive_errors, exc)
            # DON'T update heading – wait for next valid signal from MPU
            self._last_time = now
            return {
                "heading": self._heading,  # keep last known heading
                "dt": dt,
                "accel": {"x": 0, "y": 0, "z": 9.81},
                "gyro": {"z": 0},
                "lateral_g": 0.0,
                "crash": False,
                "mpu_valid": False,
                "mpu_errors": self._consecutive_errors,
            }

        # Successfully read MPU – reset error counter
        if self._consecutive_errors > 0:
            _log.info("MPU_SIGNAL_RESTORED after %d errors",
                      self._consecutive_errors)
        self._consecutive_errors = 0
        self._last_time = now

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
            "mpu_valid": True,
            "mpu_errors": 0,
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

            # Forward-facing minimum (wider ±25° — was ±15°, missed oblique obstacles)
            if -25 <= angle_deg <= 25 and dist_cm < min_fwd:
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
    # Relaxed thresholds: 40%/60% instead of 50%/70% — catch wedged-against-wall scenarios
    n_sectors = len(sector_map)
    if use_laser:
        boxed = ((blocked_count >= n_sectors * 0.4) and (ir_left or ir_right)) or \
                (ir_left and ir_right and min_dist < ALL_BLOCKED_CM) or \
                (blocked_count >= n_sectors * 0.6)
    else:
        boxed = ir_left and ir_right

    # Both IR always means boxed-in — front is physically blocked
    if ir_left and ir_right:
        boxed = True

    # ── Swerve angle: steer AWAY from the threat vector ──────────────
    # Amplify swerve proportionally to threat magnitude and proximity.
    # With weak motors, the raw atan2 angle is too small to cause real
    # turning — multiply by a gain that scales with how close obstacles are.
    if threat_mag < 0.05:
        swerve_angle = 0.0
    else:
        raw_angle = -math.degrees(math.atan2(threat_x, max(threat_y, 0.01)))
        # Gain scales with proximity: 2.0× at WARN_DIST, up to 4.0× at close range
        # (was 1.5–3.0× — too weak when obstacle is straight ahead producing
        #  a near-zero lateral angle)
        proximity_gain = 1.0 + 1.0 * (1.0 - min(min_fwd, WARN_DIST) / WARN_DIST)
        swerve_gain = 2.0 * proximity_gain   # 2.0–4.0× amplification
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

    # Speed reduction: progressive slowdown based on forward obstacles.
    # Start slowing from WARN_DIST, aggressive reduction below CRITICAL_DIST.
    side_influence = min(min_fwd, min_dist * 2.0) if min_dist < SECTOR_DEFAULT_CM else min_fwd
    overall_min = max(min_fwd * 0.7, side_influence)  # forward biased
    if overall_min < CRITICAL_DIST:
        # Quadratic reduction for tighter braking near obstacles
        ratio = overall_min / CRITICAL_DIST
        speed_factor = max(SPEED_FLOOR_FACTOR, ratio * ratio)
    elif overall_min < WARN_DIST:
        # Steeper linear reduction between WARN_DIST and CRITICAL_DIST
        # (was 0.85+0.15 — car kept 86% speed at 42cm, too fast to react)
        speed_factor = 0.75 + 0.25 * ((overall_min - CRITICAL_DIST) /
                                        (WARN_DIST - CRITICAL_DIST))
    else:
        speed_factor = 1.0

    return swerve_angle, speed_factor, threat_mag, boxed, min_fwd, left_avg, right_avg


def find_best_direction(sector_map, min_fwd):
    """Analyze the laser sector map and find the best direction to navigate.

    Uses corridor scoring: each direction is scored by the average distance
    of itself and its immediate neighbours, finding the widest clear path
    rather than a single lucky sector.

    Returns:
        best_angle: float – target angle relative to car forward (-60..+60)
        best_dist:  float – obstacle distance in the best direction (cm)
        should_turn: bool – True if the car should actively turn toward best_angle
    """
    if not sector_map:
        return 0.0, SECTOR_DEFAULT_CM, False

    n = len(sector_map)
    dists = [d for _, d, _ in sector_map]
    angles = [a for a, _, _ in sector_map]

    # Corridor score: average of sector and its immediate neighbours
    # This finds the direction with the widest clear corridor, not just
    # one sector that happens to be far.
    best_score = -1.0
    best_idx = n // 2  # default: forward

    for i in range(n):
        # Gather corridor (self + up to 2 neighbours on each side)
        corridor = [dists[i]]
        if i > 0:
            corridor.append(dists[i - 1])
        if i < n - 1:
            corridor.append(dists[i + 1])
        if i > 1:
            corridor.append(dists[i - 2])
        if i < n - 2:
            corridor.append(dists[i + 2])

        score = sum(corridor) / len(corridor)

        # Only a very small forward bonus (2%) and only when forward is clear
        if abs(angles[i]) <= 10 and min_fwd > WARN_DIST:
            score *= 1.02

        if score > best_score:
            best_score = score
            best_idx = i

    best_angle = angles[best_idx]
    best_dist = dists[best_idx]

    # Should we actively turn toward this direction?
    # Yes if: forward is closing in AND the best direction is significantly
    # better AND the angle is large enough for motors to actually execute
    should_turn = (min_fwd < NAVIGATE_DIST and
                   best_dist > min_fwd + BEST_DIR_ADVANTAGE and
                   abs(best_angle) >= MIN_STEER_ANGLE)

    return float(best_angle), best_dist, should_turn


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


def get_avoidance_zone(min_fwd):
    """Determine FSM obstacle avoidance zone based on minimum forward distance.

    Returns:
        AvoidanceZone – one of CLEAR, CAUTION, DANGER, CRITICAL
    """
    if min_fwd <= TANK_TURN_DIST:
        return AvoidanceZone.CRITICAL
    elif min_fwd <= CRITICAL_DIST:
        return AvoidanceZone.DANGER
    elif min_fwd <= WARN_DIST:
        return AvoidanceZone.CAUTION
    else:
        return AvoidanceZone.CLEAR


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────

class AutonomousController:
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

        # Stuck detection
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0

        # Navigation state — heading-based "navigate toward open space"
        self._nav_target_heading = None   # None = cruise, float = heading to turn toward
        self._nav_smoothed_best = 0.0     # EMA-smoothed best direction angle
        self._prev_zone = AvoidanceZone.CLEAR  # FSM zone tracking
        self._mpu_fallback_mode = False   # True when MPU has too many errors
        self._fallback_cycles = 0         # cycle counter in fallback mode

        # Drive thread
        self._drive_thread = None

        _log.info("========================================================================")
        _log.info("AUTONOMOUS_DRIVING INIT  pid=%d", os.getpid())
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
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._nav_target_heading = None
        self._nav_smoothed_best = 0.0
        self._mpu_fallback_mode = False
        self._fallback_cycles = 0

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
            self._stuck_cycles = 0
            self._stuck_heading_ref = 0.0
            self._nav_target_heading = None
            self._nav_smoothed_best = 0.0
            self._mpu_fallback_mode = False
            self._fallback_cycles = 0
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
            min_fwd = self.scanner.get_min_forward_distance()  # fallback; overwritten by compute_swerve

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
                raw_swerve, speed_factor, threat_mag, boxed, min_fwd, left_avg, right_avg = \
                    compute_swerve(sector_map, ir_l, ir_r,
                                   sweep_ready=sweep_ready)

                # ── Swerve EMA smoothing (prevents violent oscillation) ──
                raw_swerve = (SWERVE_SMOOTH_ALPHA * raw_swerve +
                              (1.0 - SWERVE_SMOOTH_ALPHA) * prev_swerve)
                prev_swerve = raw_swerve

                # ── Heading-rate damping (reduces swerve when circling) ──
                heading_delta_abs = abs(imu_data["heading"] -
                                        self._heading_at_swerve_start)
                if heading_delta_abs > HEADING_DAMP_START:
                    damp_frac = min(1.0,
                                    (heading_delta_abs - HEADING_DAMP_START) /
                                    (HEADING_DAMP_FULL - HEADING_DAMP_START))
                    damp_factor = 1.0 - damp_frac * HEADING_DAMP_MAX
                    raw_swerve *= damp_factor

                # ── MPU status check ─────────────────────────────────
                mpu_valid = imu_data.get("mpu_valid", True)
                mpu_errors = imu_data.get("mpu_errors", 0)
                imu_heading = imu_data["heading"]

                if mpu_errors >= MPU_MAX_CONSEC_ERRORS and not self._mpu_fallback_mode:
                    self._mpu_fallback_mode = True
                    self._fallback_cycles = 0
                    _log.warning("MPU_FALLBACK_ENABLED errors=%d – switching to laser-only nav",
                                 mpu_errors)
                elif mpu_valid and self._mpu_fallback_mode:
                    self._mpu_fallback_mode = False
                    self._fallback_cycles = 0
                    _log.info("MPU_FALLBACK_DISABLED – MPU signal restored")

                # ── Find best direction (proactive navigation) ───────
                best_angle, best_dist, should_turn = \
                    find_best_direction(sector_map, min_fwd)

                # Smooth best direction via EMA
                self._nav_smoothed_best = (
                    NAV_SMOOTH_ALPHA * best_angle +
                    (1.0 - NAV_SMOOTH_ALPHA) * self._nav_smoothed_best)

                # ── Boxed-in detection (keep existing logic) ─────────
                if boxed:
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

                # ── Stuck detection ──────────────────────────────────
                if min_fwd < STUCK_MIN_FWD_CM:
                    heading_change = abs(imu_heading - self._stuck_heading_ref)
                    if heading_change < STUCK_HEADING_THRESH:
                        self._stuck_cycles += 1
                    else:
                        self._stuck_cycles = 0
                        self._stuck_heading_ref = imu_heading
                    if self._stuck_cycles >= STUCK_CYCLE_LIMIT:
                        self._stuck_cycles = 0
                        self._boxed_in_count += 1
                        _log.warning(
                            "STUCK_DETECTED count=%d cycles=%d "
                            "heading_change=%.1f min_fwd=%.1f",
                            self._boxed_in_count, STUCK_CYCLE_LIMIT,
                            heading_change, min_fwd)
                        self._do_boxed_in_recovery()
                        continue
                else:
                    self._stuck_cycles = 0
                    self._stuck_heading_ref = imu_heading

                # ── Circle detection ─────────────────────────────────
                heading_delta = abs(imu_heading - self._heading_at_swerve_start)
                if heading_delta > CIRCLE_HEADING_LIMIT and min_fwd < WARN_DIST:
                    self._boxed_in_count += 1
                    _log.warning(
                        "CIRCLE_DETECTED count=%d heading_delta=%.1f "
                        "min_fwd=%.1f – triggering recovery",
                        self._boxed_in_count, heading_delta, min_fwd)
                    self._do_boxed_in_recovery()
                    continue

                # ═══════════════════════════════════════════════════════
                #  NAVIGATION DECISION — FSM Avoidance Zones
                # ═══════════════════════════════════════════════════════

                swerve_angle = 0.0
                use_tank = False

                # ── Branch 1: IR emergency – reverse, scan, swerve ─
                if ir_l and ir_r:
                    # Both IR triggered → front is blocked regardless of
                    # laser data.  Reverse, scan, pivot toward open side.
                    _log.warning("IR_BOTH_BLOCKED ir=[%d,%d] – "
                                 "reversing + pivot", ir_l, ir_r)
                    self._do_ir_both_blocked_recovery()
                    continue
                elif ir_l:
                    # Left IR triggered → stuck on left, can't turn left.
                    # Reverse briefly, scan, then swerve hard right.
                    _log.warning("IR_LEFT_STUCK – reversing + swerve right")
                    self._do_ir_stuck_recovery('left')
                    continue
                elif ir_r:
                    # Right IR triggered → stuck on right, can't turn right.
                    # Reverse briefly, scan, then swerve hard left.
                    _log.warning("IR_RIGHT_STUCK – reversing + swerve left")
                    self._do_ir_stuck_recovery('right')
                    continue

                # ── Determine FSM avoidance zone ────────────────────
                zone = get_avoidance_zone(min_fwd)
                if zone != self._prev_zone:
                    _log.info("FSM_ZONE %s → %s min_fwd=%.1f",
                              self._prev_zone.name, zone.name, min_fwd)
                    self._prev_zone = zone

                # ── MPU fallback (only when NOT in danger/critical) ──
                if (self._mpu_fallback_mode and
                        zone not in (AvoidanceZone.CRITICAL,
                                     AvoidanceZone.DANGER)):
                    self._fallback_cycles += 1
                    swerve_angle, left_spd, right_spd = \
                        self._fallback_navigate(sector_map, min_fwd,
                                                best_angle, best_dist,
                                                should_turn, speed_factor)
                    # Periodically retry MPU
                    if self._fallback_cycles % MPU_RETRY_INTERVAL == 0:
                        _log.info("MPU_RETRY_ATTEMPT cycle=%d", self._fallback_cycles)

                # ── CRITICAL zone (≤20cm / 200mm): TANK TURN ─────────
                elif zone == AvoidanceZone.CRITICAL:
                    self._nav_target_heading = None
                    # Choose turn direction: toward the side with more room
                    if right_avg >= left_avg:
                        turn_dir = 'right'
                        swerve_angle = SWERVE_MAX_ANGLE
                    else:
                        turn_dir = 'left'
                        swerve_angle = -SWERVE_MAX_ANGLE
                    self.motor.tank_turn(turn_dir, TANK_TURN_SPEED)
                    use_tank = True
                    left_spd = TANK_TURN_SPEED
                    right_spd = TANK_TURN_SPEED
                    self._heading_at_swerve_start = imu_heading
                    _log.info("FSM_TANK_TURN dir=%s min_fwd=%.1f "
                              "L_avg=%.1f R_avg=%.1f",
                              turn_dir, min_fwd, left_avg, right_avg)

                # ── DANGER zone (20–40cm): hard swerve, inner ≈ 0 ────
                elif zone == AvoidanceZone.DANGER:
                    self._nav_target_heading = None
                    swerve_angle = raw_swerve
                    # Force strong swerve if computed angle is too weak
                    if abs(swerve_angle) < MIN_FORCED_SWERVE:
                        swerve_angle = (SWERVE_MAX_ANGLE
                                        if right_avg >= left_avg
                                        else -SWERVE_MAX_ANGLE)
                    # Speed scales with distance: near TANK_TURN_DIST → very slow,
                    # near CRITICAL_DIST → moderate
                    zone_ratio = max(0.0, min(1.0,
                        (min_fwd - TANK_TURN_DIST) /
                        max(1, CRITICAL_DIST - TANK_TURN_DIST)))
                    outer_spd = TURN_OUTER_SPEED * (0.4 + 0.6 * zone_ratio)
                    inner_spd = max(0, TURN_INNER_SPEED * zone_ratio * 0.3)
                    if swerve_angle > 0:   # turn right
                        left_spd, right_spd = outer_spd, inner_spd
                    else:                   # turn left
                        left_spd, right_spd = inner_spd, outer_spd
                    self._heading_at_swerve_start = imu_heading
                    _log.debug("FSM_DANGER swerve=%.1f min_fwd=%.1f "
                               "spd=[%.1f,%.1f] zr=%.2f",
                               swerve_angle, min_fwd,
                               left_spd, right_spd, zone_ratio)

                # ── CAUTION zone (40–80cm): moderate swerve / nav ────
                elif zone == AvoidanceZone.CAUTION:
                    # Heading-based navigation only if safe enough
                    if should_turn and min_fwd > CRITICAL_DIST + 5:
                        # Re-evaluate target heading to track shifting best dir
                        desired_target = imu_heading - self._nav_smoothed_best
                        if self._nav_target_heading is None:
                            self._nav_target_heading = desired_target
                            _log.info("NAV_TURN_START target=%.1f best_angle=%.1f "
                                      "best_dist=%.1f min_fwd=%.1f",
                                      self._nav_target_heading,
                                      self._nav_smoothed_best,
                                      best_dist, min_fwd)
                        else:
                            # Smooth-update target if best direction shifted
                            target_diff = abs(desired_target - self._nav_target_heading)
                            if target_diff > 15:
                                self._nav_target_heading = (
                                    0.3 * desired_target +
                                    0.7 * self._nav_target_heading)

                        heading_error = self._nav_target_heading - imu_heading
                        while heading_error > 180:
                            heading_error -= 360
                        while heading_error < -180:
                            heading_error += 360

                        swerve_angle = heading_error

                        if abs(heading_error) > TARGET_HEADING_TOL:
                            if heading_error < 0:
                                left_spd = TURN_OUTER_SPEED
                                right_spd = TURN_INNER_SPEED
                            else:
                                left_spd = TURN_INNER_SPEED
                                right_spd = TURN_OUTER_SPEED
                            _log.debug("NAV_TURNING error=%.1f target=%.1f "
                                       "heading=%.1f spd=[%.1f,%.1f]",
                                       heading_error, self._nav_target_heading,
                                       imu_heading, left_spd, right_spd)
                        else:
                            # Turn complete
                            self._nav_target_heading = None
                            self._target_heading = imu_heading
                            self._heading_at_swerve_start = imu_heading
                            left_spd = BASE_SPEED * speed_factor
                            right_spd = left_spd
                            _log.info("NAV_TURN_COMPLETE heading=%.1f", imu_heading)
                    else:
                        # Obstacle avoidance with swerve
                        self._nav_target_heading = None
                        swerve_angle = raw_swerve
                        if 0 < abs(swerve_angle) < MIN_STEER_ANGLE:
                            swerve_angle = (MIN_STEER_ANGLE
                                            if swerve_angle > 0
                                            else -MIN_STEER_ANGLE)
                        if abs(swerve_angle) >= MIN_STEER_ANGLE:
                            if swerve_angle > 0:
                                left_spd = TURN_OUTER_SPEED
                                right_spd = TURN_INNER_SPEED
                            else:
                                left_spd = TURN_INNER_SPEED
                                right_spd = TURN_OUTER_SPEED
                        else:
                            effective_speed = BASE_SPEED * speed_factor
                            left_spd, right_spd = differential_speeds(
                                effective_speed, swerve_angle)
                        self._heading_at_swerve_start = imu_heading
                        _log.debug("FSM_CAUTION swerve=%.1f min_fwd=%.1f "
                                   "spd=[%.1f,%.1f]",
                                   swerve_angle, min_fwd, left_spd, right_spd)

                # ── CLEAR zone (>80cm): cruise / heading nav ─────────
                else:
                    dt = imu_data["dt"]
                    if should_turn:
                        if self._nav_target_heading is None:
                            self._nav_target_heading = imu_heading - self._nav_smoothed_best
                            _log.info("NAV_TURN_START target=%.1f best_angle=%.1f "
                                      "best_dist=%.1f min_fwd=%.1f",
                                      self._nav_target_heading,
                                      self._nav_smoothed_best,
                                      best_dist, min_fwd)

                        heading_error = self._nav_target_heading - imu_heading
                        while heading_error > 180:
                            heading_error -= 360
                        while heading_error < -180:
                            heading_error += 360

                        swerve_angle = heading_error

                        if abs(heading_error) > TARGET_HEADING_TOL:
                            if heading_error < 0:
                                left_spd = TURN_OUTER_SPEED
                                right_spd = TURN_INNER_SPEED
                            else:
                                left_spd = TURN_INNER_SPEED
                                right_spd = TURN_OUTER_SPEED
                            _log.debug("NAV_TURNING error=%.1f target=%.1f "
                                       "heading=%.1f spd=[%.1f,%.1f]",
                                       heading_error, self._nav_target_heading,
                                       imu_heading, left_spd, right_spd)
                        else:
                            self._nav_target_heading = None
                            self._target_heading = imu_heading
                            self._heading_at_swerve_start = imu_heading
                            left_spd = BASE_SPEED * speed_factor
                            right_spd = left_spd
                            _log.info("NAV_TURN_COMPLETE heading=%.1f", imu_heading)
                    else:
                        self._nav_target_heading = None
                        # Cruising – PID heading correction
                        correction = self.imu.pid_correction(
                            self._target_heading, dt)
                        if abs(correction * 0.3) > MIN_STEER_ANGLE * 0.5:
                            swerve_angle = correction * 0.3
                        else:
                            swerve_angle = 0.0
                        effective_speed = BASE_SPEED * speed_factor
                        left_spd, right_spd = differential_speeds(
                            effective_speed, swerve_angle)
                        self._heading_at_swerve_start = imu_heading

                # Drive (skip if tank turn already applied)
                if not use_tank:
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
                    mpu_fallback=self._mpu_fallback_mode,
                    nav_target=self._nav_target_heading,
                    best_dir=round(self._nav_smoothed_best, 1),
                    best_dist=round(best_dist, 1),
                    mpu_errors=mpu_errors,
                    avoidance_zone=zone.name,
                    tank_turn=use_tank,
                )

            # ── 4. comprehensive sensor log ──────────────────────────
            if cycle % SENSOR_LOG_EVERY_N == 0:
                sectors_str = " ".join(
                    f"{a}:{d:.0f}" for a, d, _ in sector_map)
                _log.debug(
                    "CYCLE=%d st=%s zone=%s spd=[%.1f,%.1f] swerve=%.1f "
                    "sf=%.2f threat=%.2f min_fwd=%.1f tank=%s "
                    "ir=[%d,%d] rear=%.1f(fb=%d) "
                    "hdg=%.1f gz=%.2f lat_g=%.3f "
                    "ax=%.3f ay=%.3f az=%.3f "
                    "sectors=[%s]",
                    cycle,
                    self.state.name,
                    self._telem.get("avoidance_zone", "?"),
                    self._telem.get("left_speed", 0),
                    self._telem.get("right_speed", 0),
                    self._telem.get("swerve_angle", 0),
                    self._telem.get("speed_factor", 1),
                    self._telem.get("threat_mag", 0),
                    min_fwd,
                    self._telem.get("tank_turn", False),
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

    # ── Laser-only fallback navigation (used when MPU fails) ────────────

    def _fallback_navigate(self, sector_map, min_fwd, best_angle,
                           best_dist, should_turn, speed_factor):
        """Navigate using laser only – no MPU heading tracking.

        As the car turns, the laser rotates with it, so best_angle
        naturally converges toward 0 (self-correcting).

        Returns (swerve_angle, left_spd, right_spd).
        """
        if should_turn or min_fwd < WARN_DIST:
            # Steer toward the best direction using strong differential
            if abs(best_angle) < MIN_STEER_ANGLE:
                # Best direction is roughly forward – cruise
                effective_speed = BASE_SPEED * speed_factor
                _log.debug("FALLBACK_CRUISE fwd=%.1f best_angle=%.1f",
                           min_fwd, best_angle)
                return 0.0, effective_speed, effective_speed

            if best_angle > 0:
                # Turn right: left=outer
                left_spd = TURN_OUTER_SPEED
                right_spd = TURN_INNER_SPEED
            else:
                # Turn left: right=outer
                left_spd = TURN_INNER_SPEED
                right_spd = TURN_OUTER_SPEED

            _log.debug("FALLBACK_STEER angle=%.1f dist=%.1f "
                       "min_fwd=%.1f spd=[%.1f,%.1f]",
                       best_angle, best_dist, min_fwd,
                       left_spd, right_spd)
            return best_angle, left_spd, right_spd
        else:
            # Forward is clear enough – cruise straight
            effective_speed = BASE_SPEED * speed_factor
            _log.debug("FALLBACK_CLEAR fwd=%.1f", min_fwd)
            return 0.0, effective_speed, effective_speed

    # ── BOXED-IN recovery ────────────────────────────────────────────────

    def _do_boxed_in_recovery(self):
        """Recovery from boxed-in / stuck situation.
        First attempt: aggressive forward turn toward the clearest side.
        Last resort:   reverse only if forward turn fails and rear is clear.
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.2)

        # Get fresh scan data
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.0):
            time.sleep(0.02)

        sector_map = self.scanner.get_sector_map()
        left_dists  = [d for a, d, c in sector_map if a < -5]
        right_dists = [d for a, d, c in sector_map if a > 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))
        left_max  = max(left_dists)  if left_dists  else 0
        right_max = max(right_dists) if right_dists else 0

        _log.info("BOXED_IN_RECOVERY left_avg=%.1f right_avg=%.1f "
                  "left_max=%.1f right_max=%.1f",
                  left_avg, right_avg, left_max, right_max)

        # ── Attempt 1: aggressive forward turn toward clearest side ──
        # This avoids stopping and reversing — just swerve hard forward.
        if left_max >= SIDE_CLEAR_MIN_CM or right_max >= SIDE_CLEAR_MIN_CM:
            if left_avg >= right_avg:
                turn_dir = 'left'
                _log.info("RECOVERY_TANK_TURN LEFT left_avg=%.1f right_avg=%.1f",
                          left_avg, right_avg)
            else:
                turn_dir = 'right'
                _log.info("RECOVERY_TANK_TURN RIGHT left_avg=%.1f right_avg=%.1f",
                          left_avg, right_avg)
            self.motor.tank_turn(turn_dir, TANK_TURN_SPEED)

            time.sleep(RECOVERY_TURN_DURATION)
            self.motor.coast()
            time.sleep(0.1)

            self.imu.reset_heading()
            self._target_heading = 0.0
            self._heading_at_swerve_start = 0.0
            self._stuck_cycles = 0
            self._stuck_heading_ref = 0.0
            self._last_boxed_recovery_time = time.time()
            self._set_state(State.DRIVING)
            return

        # ── Attempt 2 (last resort): reverse if truly boxed in on all sides ──
        rear_d, _ = self.sonar.read()
        _log.info("BOXED_IN_REVERSE_FALLBACK rear_dist=%.1f", rear_d)

        if rear_d < 0 or rear_d < REAR_CLEAR_CM:
            _log.error("FULLY_STUCK rear_blocked=%.1f – waiting for manual "
                       "intervention (press R)", rear_d)
            self.motor.coast()
            self._running = False
            return

        # Short reverse
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

        # Wait for fresh scan after reverse
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.5):
            time.sleep(0.02)

        # Re-scan and turn toward clearest side
        sector_map = self.scanner.get_sector_map()
        left_dists  = [d for a, d, c in sector_map if a < -5]
        right_dists = [d for a, d, c in sector_map if a > 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))

        if left_avg >= right_avg:
            turn_dir = 'left'
            _log.info("RECOVERY_TANK_TURN_POST_REV LEFT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
        else:
            turn_dir = 'right'
            _log.info("RECOVERY_TANK_TURN_POST_REV RIGHT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
        self.motor.tank_turn(turn_dir, TANK_TURN_SPEED)

        time.sleep(RECOVERY_TURN_DURATION)
        self.motor.coast()
        time.sleep(0.1)

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    # ── IR STUCK recovery (single IR) ────────────────────────────────────

    def _do_ir_stuck_recovery(self, stuck_side):
        """Recovery when a single IR sensor fires — car is stuck on one side.

        The IR sensor detected an obstacle right against the car body,
        meaning the car physically cannot turn away.  Strategy:
          1. Brake
          2. Reverse briefly to create clearance
          3. Wait for a fresh laser scan
          4. Drive forward with maximum swerve AWAY from the stuck side
             (100,0 or 0,100 wheel speed)

        stuck_side: 'left' or 'right'
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.15)

        # Reverse briefly to create clearance
        rd, _ = self.sonar.read()
        if rd < 0 or rd > REAR_CLEAR_CM:
            _log.info("IR_STUCK_REVERSE side=%s speed=%d dur=%.1fs rear=%.1f",
                      stuck_side, REVERSE_SPEED, IR_STUCK_REVERSE_DUR, rd)
            self.motor.reverse_straight(REVERSE_SPEED)
            t0 = time.time()
            while time.time() - t0 < IR_STUCK_REVERSE_DUR and self._running:
                rd2, rfb = self.sonar.read()
                if 0 < rd2 < REAR_STOP_CM:
                    _log.warning("IR_STUCK_REVERSE_ABORT rear=%.1f", rd2)
                    break
                time.sleep(0.05)
            self.motor.brake()
            time.sleep(0.1)
        else:
            _log.warning("IR_STUCK_NO_REVERSE rear=%.1f – rear blocked", rd)

        # Wait for fresh scan
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.0):
            time.sleep(0.02)

        # Tank turn away from the stuck side
        if stuck_side == 'left':
            # Obstacle on left → tank turn right
            _log.info("IR_STUCK_TANK_RIGHT after reverse")
            self.motor.tank_turn('right', TANK_TURN_SPEED)
        else:
            # Obstacle on right → tank turn left
            _log.info("IR_STUCK_TANK_LEFT after reverse")
            self.motor.tank_turn('left', TANK_TURN_SPEED)

        time.sleep(RECOVERY_TURN_DURATION)
        self.motor.coast()
        time.sleep(0.1)

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    # ── IR BOTH-BLOCKED recovery ─────────────────────────────────────────

    def _do_ir_both_blocked_recovery(self):
        """Recovery when BOTH IR sensors fire — front is completely blocked.

        Laser data is IGNORED for the blocking decision (IR overrides).
        Strategy:
          1. Brake
          2. Reverse for full REVERSE_DURATION (sonar-safe)
          3. Wait for a fresh laser scan
          4. Pivot toward the clearest side (100,0 or 0,100)
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.2)

        # Reverse — rear must be clear
        rd, _ = self.sonar.read()
        if rd < 0 or rd > REAR_CLEAR_CM:
            _log.info("IR_BOTH_BLOCKED_REVERSE speed=%d dur=%.1fs rear=%.1f",
                      REVERSE_SPEED, REVERSE_DURATION, rd)
            self.motor.reverse_straight(REVERSE_SPEED)
            t0 = time.time()
            while time.time() - t0 < REVERSE_DURATION and self._running:
                rd2, rfb = self.sonar.read()
                if 0 < rd2 < REAR_STOP_CM:
                    _log.warning("IR_BOTH_REVERSE_ABORT rear=%.1f", rd2)
                    break
                time.sleep(0.05)
            self.motor.brake()
            time.sleep(0.1)
        else:
            _log.warning("IR_BOTH_BLOCKED_NO_REVERSE rear=%.1f – "
                         "rear blocked, fully stuck", rd)
            self.motor.coast()
            self._running = False
            return

        # Wait for fresh scan after reversing
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.5):
            time.sleep(0.02)

        # Scan to find clearest side and pivot toward it
        sector_map = self.scanner.get_sector_map()
        left_dists  = [d for a, d, c in sector_map if a < -5]
        right_dists = [d for a, d, c in sector_map if a > 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))

        # Tank turn toward clearest side
        if left_avg >= right_avg:
            _log.info("IR_BOTH_TANK_LEFT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            self.motor.tank_turn('left', TANK_TURN_SPEED)
        else:
            _log.info("IR_BOTH_TANK_RIGHT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            self.motor.tank_turn('right', TANK_TURN_SPEED)

        # Longer pivot for both-blocked situations
        time.sleep(RECOVERY_TURN_DURATION * 1.2)
        self.motor.coast()
        time.sleep(0.1)

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
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

        # Recovery turn after crash reverse — steer away from closest side
        sweep_before = self.scanner.sweep_count
        t_wait = time.time()
        while (self.scanner.sweep_count <= sweep_before and
               (time.time() - t_wait) < 1.0):
            time.sleep(0.02)

        sector_map = self.scanner.get_sector_map()
        left_dists  = [d for a, d, c in sector_map if a < -5]
        right_dists = [d for a, d, c in sector_map if a > 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))

        turn_dur = 0.8  # shorter than boxed-in recovery turn
        if left_avg >= right_avg:
            _log.info("CRASH_TANK_TURN LEFT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            self.motor.tank_turn('left', TANK_TURN_SPEED)
        else:
            _log.info("CRASH_TANK_TURN RIGHT left_avg=%.1f right_avg=%.1f",
                      left_avg, right_avg)
            self.motor.tank_turn('right', TANK_TURN_SPEED)
        time.sleep(turn_dur)
        self.motor.coast()

        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0

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
                controller._stuck_cycles = 0
                controller._stuck_heading_ref = 0.0
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

        header = f"  AUTONOMOUS DRIVE  |  {controller.state.name}  |  " \
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

        # Speed / swerve / zone
        zone_name = t.get('avoidance_zone', 'CLEAR')
        zone_col = (COL_DANG if zone_name in ('CRITICAL', 'DANGER')
                    else COL_WARN if zone_name == 'CAUTION' else COL_OK)
        tank_str = " TANK" if t.get('tank_turn', False) else ""
        _safe(row, 0,
              f"Speed: L={t.get('left_speed', 0):5.1f}"
              f"%  R={t.get('right_speed', 0):5.1f}"
              f"%  |  Swerve: {t.get('swerve_angle', 0):+6.1f}°"
              f"  SF={t.get('speed_factor', 1):.2f}"
              f"  Threat={t.get('threat_mag', 0):.2f}",
              COL_INFO)
        row += 1
        _safe(row, 0, f"Zone: {zone_name}{tank_str}", zone_col)
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
    controller = AutonomousController()

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
