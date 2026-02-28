#!/usr/bin/env python3
"""
autonomous_driving_laser.py — Autonomous Sonar+Laser Driving Controller
=========================================================================
Standalone script for autonomous obstacle-avoidance driving using:
  • Front HC-SR04 sonar as PRIMARY always-on forward obstacle detector
  • VL53L0X laser on servo as SECONDARY on-demand directional scanner
    – Activates only when sonar detects obstacle within LASER_TRIGGER_DIST
    – Two-phase scan: quick 5-angle check, full sweep if ambiguous
    – Determines clearest escape direction for steering
  • Front IR sensors (left + right) for close-range detection
  • MPU6050 accelerometer for crash detection, gyro for yaw tracking
  • Differential steering for smooth swerving (no pivot turns)
  • No rear sensor — reversing is blind (short timed bursts)

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

from pico_sensor_reader import (
    init_pico_reader,
    get_gyro_z         as pico_get_gyro_z,
    get_accel_xyz      as pico_get_accel_xyz,
    get_laser_distance_mm as pico_get_laser_mm,
    get_ir_sensors     as pico_get_ir,
    get_rpm            as pico_get_rpm,
    get_sensor_packet  as pico_get_sensor_packet,
)

# ─────────────────────────────────────────────────────────────────────────────
#  TUNEABLE CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────

# Motor / PWM
BASE_SPEED          = 40       # forward cruise PWM % (reduced from 40 for better reaction time)
MAX_PWM_DUTY        = 60        # voltage cap  (7 V / 11.1 V × 100)
PWM_FREQ            = 1000      # Hz
REVERSE_SPEED       = 50
REVERSE_DURATION    = 1.2       # seconds (was 1.0 — not enough distance gained)
RECOVERY_TURN_DURATION = 1.8    # seconds of wide turn after reverse (was 1.5 — still not reorienting enough)
CRASH_REVERSE_SPEED = 70
CRASH_REVERSE_DURATION = 0.6
DIRECTION_CHANGE_DWELL = 0.15   # seconds between fwd↔rev

# Per-wheel trims  (calibrated to make the car go straight)
FL_TRIM = 0.60
FR_TRIM = 0.60
RL_TRIM = 1.00
RR_TRIM = 1.00

# Obstacle distances (cm)
APPROACH_DIST       = 100       # pre-emptive laser scan trigger — raised from 120 for much earlier scanning/swerving
WARN_DIST           = 50       # start moderate swerving — raised from 90 for earlier swerve reaction
CRITICAL_DIST       = 30        # hard swerve zone — increased from 40 so hard turns start earlier
ALL_BLOCKED_CM      = 15        # every forward sector is this close → boxed in (raised from 18)
EMERGENCY_IR_DIST   = 0         # IR triggers (binary, active LOW)
HARD_SWERVE_DIST    = 30       # force min swerve when forward obstacle nearer (was 80 — start forced swerve much earlier)
MIN_FORCED_SWERVE   = 60       # minimum swerve degrees when forced (was 60 — sharper forced swerve)
EMERGENCY_FWD_CM    = 25        # emergency max-swerve when centre sectors this close (was 35 — react sooner)
TANK_TURN_DIST      = 15        # cm — tank turn only as absolute last resort (was 20 — swerve should prevent reaching this)
TANK_TURN_SPEED     = 55        # PWM % for each side during tank turn (was 45 — not enough torque to rotate)
SIDE_CLEAR_MIN_CM   = 50        # side must have at least this to be considered clear
CRITICAL_TANK_DURATION = 0.8    # seconds — minimum tank turn duration before re-scanning (was 0.6 — too short)
CRITICAL_STUCK_LIMIT   = 8      # consecutive CRITICAL cycles without escape → boxed-in recovery (was 5 — give swerve more time)
CIRCLE_HEADING_LIMIT = 120      # degrees — trigger boxed-in if car rotates this much (raised from 90 — allow sharper swerves)
# Stuck detection — car not turning despite close obstacles
STUCK_HEADING_THRESH = 5.0      # degrees — max heading change to count as "not turning"
STUCK_CYCLE_LIMIT    = 60       # cycles (~1.2s) — trigger recovery (raised from 50 — swerve should resolve before this)
STUCK_MIN_FWD_CM     = 60       # cm — only count as stuck when forward obstacle is this close (was 50 — detect stuck earlier at new swerve distances)

# Swerve geometry
SWERVE_MAX_ANGLE    = 85        # max differential angle (degrees)  (was 70 — not acute enough, car still hit obstacles)
SWERVE_MIN_DYNAMIC  = 35        # minimum dynamic swerve angle (at APPROACH_DIST) — starts firmer (was 30)
SWERVE_CAUTION_MAX  = 80        # max swerve at CAUTION-DANGER boundary — aggressive before DANGER (was 75)
IR_SWERVE_BOOST     = 65        # added degrees when IR fires  (was 55 — increased for harder swerve)
IR_CLOSE_MAX_SWERVE_CM = 25     # cm — full differential (100,0 / 0,100) below this distance
IR_STUCK_REVERSE_DUR   = 0.7    # seconds to reverse when single IR fires (was 0.5 — not enough clearance)
SPEED_FLOOR_FACTOR  = 0.45      # minimum speed fraction near obstacles (was 0.55 — too fast near walls)
INNER_WHEEL_MIN     = 0.05      # inner wheels keep 5% speed — max differential for sharpest turns (was 0.09)
SWERVE_SPEED_FLOOR  = 1.00      # outer wheel stays at full BASE_SPEED during swerve
SWERVE_OUTER_BOOST  = 1.05      # slight boost to outer wheel  (was 1.15 — too aggressive differential)
SWERVE_SMOOTH_ALPHA = 0.75      # EMA smoothing factor — raised from 0.60 for faster swerve response
DANGER_OUTER_FLOOR  = 0.80      # outer wheel never below 75% in DANGER — maintain turn torque

# Heading-rate damping — prevents persistent one-direction swerve (circling)
HEADING_DAMP_START  = 50        # degrees — start damping after this much heading change (was 30 — too early)
HEADING_DAMP_FULL   = 150       # degrees — full damping at this heading change (was 120 — allow sharper turns)
HEADING_DAMP_MAX    = 0.35      # max swerve reduction factor (0–1) — reduced from 0.45 to allow even stronger swerves

# PID heading correction (gyro-Z)
GYRO_KP             = 1.5
GYRO_KI             = 0.0
GYRO_KD             = 0.3

# Crash detection (accelerometer) — DYNAMIC threshold based on speed
# At low speed impacts are softer, so the threshold should be lower.
ACCEL_G             = 9.81      # m/s² per g
CRASH_G_HIGH_SPEED  = 1.5       # g-force threshold at full speed (PWM >= 60)
CRASH_G_LOW_SPEED   = 0.4       # g-force threshold at very low speed / crawl (PWM <= 20)
CRASH_G_RANGE_PWM   = (20, 60)  # PWM range over which to interpolate
MOVING_THRESHOLD_G  = 0.3       # detect if wheels spin but car is stuck
CRASH_COOLDOWN      = 1.5       # seconds between consecutive crash events

# Navigation — proactive "navigate toward open space" approach
NAVIGATE_DIST       = 140        # cm – start looking for best direction when fwd closer (was 100 — match new APPROACH_DIST)
BEST_DIR_ADVANTAGE  = 30        # cm – best direction must have this much more room than fwd
TARGET_HEADING_TOL  = 12        # degrees – turn considered complete within this tolerance
TURN_OUTER_SPEED    = 60        # PWM % for outer wheels during active heading turns (was 50)
TURN_INNER_SPEED    = 10        # PWM % for inner wheels (was 15 — not enough differential to avoid obstacles)
MIN_STEER_ANGLE     = 20        # degrees – below this, motors can't actually turn; drive straight
CRUISE_CLEAR_DIST   = 160       # cm – when forward is this clear, just cruise straight (match new APPROACH_DIST)
NAV_SMOOTH_ALPHA    = 0.3       # EMA smoothing for target heading updates (prevents oscillation)

# Proximity-proportional dodge — direct wheel speed mapping
# Dynamic swerve: SQUARE-ROOT aggression ramp means the car swerves
# strongly from medium distances. At 100cm the car is already curving
# noticeably; by 60cm it's in a hard turn well before reaching the
# obstacle.  Stop/reverse should be extremely rare — only when truly
# boxed in.
# Heading-based attenuation prevents circling: after turning enough
# degrees, the dodge eases off so the car straightens out.
DODGE_START_CM      = 160       # cm — begin dodging very early (was 120 — too late)
DODGE_HARD_CM       = 45        # cm — full hard turn at 45cm, well before tank-turn (was 25)
DODGE_OUTER_MAX     = MAX_PWM_DUTY   # 70 — outer wheel at max aggression
DODGE_OUTER_MIN     = 55        # outer wheel at minimum aggression (was 50 — more differential)
DODGE_INNER_GENTLE  = 20        # inner wheel at gentle dodge (was 28 — sharper curves from the start)
DODGE_INNER_HARD    = 3         # inner wheel at max aggression (was 5 — more differential for hard swerve)
DODGE_CLEAR_DELAY_S = 0.3       # seconds of CLEAR before resetting dodge direction (was 0.5 — quicker reset)

# Heading-based dodge attenuation — prevents circling
# After turning DODGE_HEADING_EASE degrees, dodge starts reducing.
# After DODGE_HEADING_MAX degrees, dodge is fully attenuated (car goes straight).
DODGE_HEADING_EASE  = 55        # degrees — start easing off dodge (was 35 — too early, prevented evasion)
DODGE_HEADING_MAX   = 110       # degrees — force straighten (was 70 — too restrictive for dynamic swerve)
DODGE_STRAIGHT_CYCLES = 8       # ~0.16s of straight driving after heading-based attenuation (was 25 — too long)

# LM393 wheel encoder (rear-right wheel) — now on Pico GPIO 10 (via UART bridge)
# Legacy Pi pin: was BCM GPIO 26 (physical pin 37); now Pico GPIO 10 (pin 14)
ENCODER_HOLES       = 20        # holes per revolution on encoder disc
ENCODER_CALC_HZ     = 10        # RPM calculation rate (Hz)

# Multi-sensor stuck detection (LM393 + sonar + MPU)
# If wheel is spinning but sonar says same distance AND heading isn't
# changing, the car is stuck on a blind-spot obstacle.
BLINDSPOT_RPM_MIN       = 5.0   # RPM — wheel is considered spinning above this
BLINDSPOT_SONAR_DELTA   = 5.0   # cm — sonar change must exceed this to count as moving
BLINDSPOT_HEADING_DELTA = 3.0   # degrees — heading change must exceed this
BLINDSPOT_WINDOW_S      = 1.5   # seconds — observation window
BLINDSPOT_TRIGGER_S     = 1.0   # seconds of continuous stuck before triggering recovery

# ── Stall detection & throttle ramp (carpet / low battery) ───────────
# Detects when motors are commanded but the car isn't physically moving,
# then gradually increases PWM like pressing a gas pedal harder until
# the car overcomes surface friction or low voltage.
# Uses gyro-Z rate, heading change, sonar delta, and lateral-g as
# indicators of actual physical movement.
STALL_GZ_THRESH         = 3.0   # deg/s — gyro-Z below this = not rotating
STALL_LAT_G_THRESH      = 0.10  # g — lateral accel below this = not accelerating
STALL_HDG_DELTA_THRESH  = 1.5   # degrees — heading change below this over window = stuck
STALL_SONAR_DELTA_THRESH = 3.0  # cm — sonar change below this over window = stuck
STALL_MIN_CMD_PWM       = 15    # only detect stall when commanding at least this PWM
STALL_DETECT_CYCLES     = 15    # cycles (~0.3s) — confirm stall before ramping
STALL_RAMP_STEP         = 2.0   # PWM % added per cycle once stall is confirmed
STALL_RAMP_MAX          = 25.0  # max total PWM boost (prevents runaway)
STALL_RAMP_DECAY        = 0.5   # PWM % removed per cycle when car IS moving (gentle ease-off)
STALL_LOG_INTERVAL      = 25    # log stall ramp status every N cycles

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

# Front sonar (HC-SR04 — primary obstacle detector, static mount)
FRONT_SONAR_HZ      = 10        # polling rate (higher than old rear sonar)
FRONT_MIN_VALID_CM  = 3.0       # reject readings shorter than this (sensor noise)
LASER_TRIGGER_DIST  = 120       # cm — sonar distance that activates laser scan (matches APPROACH_DIST)
SCAN_COOLDOWN_S     = 1.5       # minimum seconds between laser scans

# Laser on-demand scan (two-phase)
QUICK_SCAN_ANGLES   = [-60, -30, 0, 30, 60]   # Phase 1: quick 5-angle check
AMBIGUITY_THRESH_CM = 15        # if |left_avg - right_avg| < this → do full sweep

# Blind reverse (no rear sensor)
BLIND_REVERSE_SPEED    = 50     # PWM % — raised from 30, car barely moved at old value
BLIND_REVERSE_DURATION = 0.7    # seconds — raised from 0.5, needs longer burst to gain clearance

# Reverse-before-turn thresholds (boxed-in recovery)
REVERSE_BEFORE_TURN_DIST  = 35  # cm — reverse first if front is closer than this during recovery (was 45 — should rarely reach here now)
REPEATED_STUCK_REVERSE    = True  # always reverse on 2nd+ consecutive boxed-in
REVERSE_ESCALATE_STEP     = 0.5 # extra seconds of reverse per consecutive boxed-in event (was 0.3 — too conservative)
REVERSE_MAX_DURATION      = 2.5 # cap total reverse duration (was 2.0)
REVERSE_CHECK_INTERVAL    = 0.1 # seconds — how often to poll front sonar while reversing
REVERSE_MIN_CLEARANCE_CM  = 50  # cm — keep reversing until front sonar shows at least this

# MPU6050 calibration
CALIBRATION_SAMPLES = 200
CALIBRATION_INTERVAL = 0.010    # seconds between samples

# Timing
LOOP_HZ             = 50
LOOP_PERIOD         = 1.0 / LOOP_HZ
UI_REFRESH_HZ       = 5
SENSOR_LOG_EVERY_N  = 1         # log every Nth control cycle (1 = every)

# ─────────────────────────────────────────────────────────────────────────────
#  GPIO PIN MAP  (BCM numbering)
# ─────────────────────────────────────────────────────────────────────────────

# Front-left motor
FL_IN1 = 17;  FL_IN2 = 27;  FL_ENA = 12
# Front-right motor
FR_IN3 = 23;  FR_IN4 = 22;  FR_ENB = 13
# Rear-left motor
RL_IN1 = 9;   RL_IN2 = 11;  RL_ENA = 26
# Rear-right motor
RR_IN3 = 10;  RR_IN4 = 7;   RR_ENB = 16

# Sensors
# IR_LEFT  — moved to Pico GPIO 8  (was Pi BCM GPIO 5)
# IR_RIGHT — moved to Pico GPIO 9  (was Pi BCM GPIO 6)
# ENCODER  — moved to Pico GPIO 10 (was Pi BCM GPIO 26)
# MPU6050  — moved to Pico I2C     (was Pi I2C 0x68)
# VL53L0X  — moved to Pico I2C     (was Pi I2C 0x29)
# ⚑ All sensors above are read via pico_sensor_reader over UART
SERVO_PIN      = 20
FRONT_TRIG_PIN = 25
FRONT_ECHO_PIN = 24

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
    CLEAR    = auto()   # > APPROACH_DIST (160cm): cruise normally
    APPROACH = auto()   # WARN_DIST .. APPROACH_DIST (110-160cm): pre-scan, gentle swerve
    CAUTION  = auto()   # CRITICAL_DIST .. WARN_DIST (55-110cm): moderate->hard swerve
    DANGER   = auto()   # TANK_TURN_DIST .. CRITICAL_DIST (15-55cm): hard swerve, minimal forward
    CRITICAL = auto()   # ≤ TANK_TURN_DIST (15cm): tank turn — absolute last resort

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING SETUP  →  slalom.logs
# ─────────────────────────────────────────────────────────────────────────────

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_FILE = os.path.join(PROJECT_ROOT, "autonomous_driving_laser.logs")

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

# ── Pico sensor bridge (replaces direct I2C for MPU6050 and VL53L0X) ─────────
# VL53L0X and MPU6050 are now physically connected to the Pico W and their
# data arrives via UART JSON packets.  No Pi-side I2C needed.
_pico = init_pico_reader()

def _pico_ready():
    """True if the Pico bridge is connected and delivering fresh packets."""
    return _pico is not None and _pico.is_fresh(max_age_s=3.0)

# These flags remain True: data comes from Pico, not direct I2C.
# They guard fall-through paths; real availability is checked via _pico_ready().
HAS_TOF = True   # VL53L0X laser  — via Pico UART
HAS_IMU = True   # MPU6050        — via Pico UART
_tof = None      # legacy name — no direct I2C object
_imu = None      # legacy name — no direct I2C object
_log.info("PICO_BRIDGE_INIT port=%s", _pico.port if _pico else "NONE")

# ─────────────────────────────────────────────────────────────────────────────
#  WHEEL ENCODER  (LM393 encoder — now read from Pico sensor bridge)
# ─────────────────────────────────────────────────────────────────────────────

class WheelEncoder:
    """LM393 wheel-speed encoder — data sourced from Pico bridge via UART.

    The encoder is physically wired to Pico GPIO 10 (was Pi BCM 26).
    RPM is computed on the Pico and transmitted in every JSON packet.
    Same public interface as the old GPIO-interrupt version."""

    def __init__(self):
        self._available = _pico is not None
        if self._available:
            _log.info("ENCODER_INIT via Pico bridge  holes=%d", ENCODER_HOLES)
        else:
            _log.warning("ENCODER_INIT_FAIL: Pico bridge unavailable — wheel speed unavailable")

    @property
    def rpm(self):
        """Current wheel RPM from Pico bridge (float)."""
        return pico_get_rpm() or 0.0

    @property
    def is_spinning(self):
        """True if wheel is spinning above BLINDSPOT_RPM_MIN."""
        return self.rpm >= BLINDSPOT_RPM_MIN

    @property
    def available(self):
        """True if Pico bridge is connected and delivering fresh data."""
        return self._available and _pico_ready()


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
#  LASER SCANNER  (servo + VL53L0X — on-demand secondary navigation)
# ─────────────────────────────────────────────────────────────────────────────

class LaserScanner:
    """On-demand directional scanner using a servo-mounted VL53L0X.

    The laser is NOT continuously scanning.  It stays centered (forward)
    while idle and only activates when the front sonar detects an obstacle
    within LASER_TRIGGER_DIST.  Two-phase scan:
      Phase 1 (quick): 5 key angles — ~250 ms
      Phase 2 (full):  13 sectors ±60° — ~650 ms extra (only if ambiguous)
    After scanning, servo returns to center.
    """

    def __init__(self):
        self._full_angles = list(range(-SCAN_RANGE_DEG, SCAN_RANGE_DEG + 1,
                                       SCAN_STEP_DEG))
        self._n_sectors = len(self._full_angles)

        # Last scan result storage
        self._last_scan_result = None   # set by scan_for_direction()
        self._last_scan_time = 0.0
        self._scan_count = 0
        self._scanning = False          # True while a scan is in progress

        # Servo PWM setup
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz standard servo
        self._pwm.start(0)

        _log.info("SCANNER_INIT on-demand mode, quick_angles=%s full_sectors=%d",
                  QUICK_SCAN_ANGLES, self._n_sectors)

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
        """Read VL53L0X distance in cm via Pico bridge, or -1 on error.
        The sensor is physically on the Pico; the servo angles it and the
        Pi reads the result from the latest UART packet."""
        try:
            mm = pico_get_laser_mm()
            if mm is None or mm < 0 or mm > 2000:
                return -1
            cm = round(mm / 10.0, 1)
            if cm < FLOOR_REJECT_CM:
                return -1
            return cm
        except Exception:
            return -1

    # ── on-demand scan ───────────────────────────────────────────────────

    def read_forward(self):
        """Single laser reading at 0° (forward).  Use as backup distance
        confirmation while servo is idle.  Returns cm or -1."""
        self._set_servo_angle(0)
        d = self._read_laser_cm()
        self._pwm.ChangeDutyCycle(0)
        return d

    def scan_for_direction_async(self):
        """Start a scan in a background thread.  Returns immediately.
        The drive loop keeps dodging with the last known direction
        while the scan runs.  Result is picked up via
        `last_scan_result` / `is_scanning`."""
        if self._scanning:
            return  # already in progress
        self._scanning = True
        t = threading.Thread(target=self._do_scan_work, daemon=True)
        t.start()

    def _do_scan_work(self):
        """Actual scan logic — runs in background thread."""
        result = self._scan_for_direction_sync()
        # Store result atomically
        self._last_scan_result = result
        self._last_scan_time = time.time()
        self._scan_count += 1
        self._scanning = False

    def scan_for_direction(self):
        """Synchronous scan (blocking).  Used by recovery routines
        that need an immediate result."""
        return self._scan_for_direction_sync()

    def _scan_for_direction_sync(self):
        """Two-phase on-demand scan to find the clearest escape direction.

        Phase 1 (quick): Read 5 key angles [-60, -30, 0, +30, +60]
        Phase 2 (full):  If left vs right are ambiguous (within
                         AMBIGUITY_THRESH_CM), do a full 13-sector sweep.

        Returns dict:
            direction: 'left' | 'right' | 'forward'
            best_angle: float (-60..+60)
            left_avg:   float (cm)
            right_avg:  float (cm)
            fwd_dist:   float (cm) — distance at 0°
            sector_map: list of (angle, dist, 1.0) tuples
            phase:      1 or 2 — which phase was used for decision
        """
        self._scanning = True
        t_start = time.time()

        # ── Phase 1: quick scan at 5 key angles ─────────────────────
        quick_data = []
        for angle in QUICK_SCAN_ANGLES:
            self._set_servo_angle(angle)
            dist = self._read_laser_cm()
            if dist < 0:
                dist = SECTOR_DEFAULT_CM
            quick_data.append((angle, dist, 1.0))

        # Compute side averages from quick scan
        left_dists  = [d for a, d, _ in quick_data if a < -5]
        right_dists = [d for a, d, _ in quick_data if a > 5]
        fwd_dists   = [d for a, d, _ in quick_data if -5 <= a <= 5]
        left_avg  = sum(left_dists)  / max(1, len(left_dists))
        right_avg = sum(right_dists) / max(1, len(right_dists))
        fwd_dist  = fwd_dists[0] if fwd_dists else SECTOR_DEFAULT_CM

        phase = 1
        sector_map = list(quick_data)

        _log.info("SCAN_PHASE1 L_avg=%.1f R_avg=%.1f fwd=%.1f (%.0fms)",
                  left_avg, right_avg, fwd_dist,
                  (time.time() - t_start) * 1000)

        # ── Phase 2: full sweep if ambiguous ─────────────────────────
        if abs(left_avg - right_avg) < AMBIGUITY_THRESH_CM:
            phase = 2
            full_data = []
            for angle in self._full_angles:
                self._set_servo_angle(angle)
                dist = self._read_laser_cm()
                if dist < 0:
                    dist = SECTOR_DEFAULT_CM
                full_data.append((angle, dist, 1.0))

            # Recompute with full data
            left_dists  = [d for a, d, _ in full_data if a < -5]
            right_dists = [d for a, d, _ in full_data if a > 5]
            fwd_dists   = [d for a, d, _ in full_data if -5 <= a <= 5]
            left_avg  = sum(left_dists)  / max(1, len(left_dists))
            right_avg = sum(right_dists) / max(1, len(right_dists))
            fwd_dist  = min(fwd_dists) if fwd_dists else SECTOR_DEFAULT_CM
            sector_map = list(full_data)

            _log.info("SCAN_PHASE2 L_avg=%.1f R_avg=%.1f fwd=%.1f (%.0fms)",
                      left_avg, right_avg, fwd_dist,
                      (time.time() - t_start) * 1000)

        # Return servo to center
        self.center_servo()

        # Determine best direction
        if fwd_dist > max(left_avg, right_avg) and fwd_dist > LASER_TRIGGER_DIST:
            direction = 'forward'
            best_angle = 0.0
        elif left_avg >= right_avg:
            direction = 'left'
            # Find the exact best angle on the left side
            left_sectors = [(a, d) for a, d, _ in sector_map if a < -5]
            if left_sectors:
                best_angle = max(left_sectors, key=lambda x: x[1])[0]
            else:
                best_angle = -45.0
        else:
            direction = 'right'
            right_sectors = [(a, d) for a, d, _ in sector_map if a > 5]
            if right_sectors:
                best_angle = max(right_sectors, key=lambda x: x[1])[0]
            else:
                best_angle = 45.0

        result = {
            'direction': direction,
            'best_angle': best_angle,
            'left_avg': left_avg,
            'right_avg': right_avg,
            'fwd_dist': fwd_dist,
            'sector_map': sector_map,
            'phase': phase,
        }

        self._scanning = False

        _log.info("SCAN_RESULT dir=%s best_angle=%.0f L=%.1f R=%.1f "
                  "fwd=%.1f phase=%d count=%d (%.0fms total)",
                  direction, best_angle, left_avg, right_avg,
                  fwd_dist, phase, self._scan_count + 1,
                  (time.time() - t_start) * 1000)

        return result

    # ── sector map access (from last scan) ───────────────────────────

    def get_sector_map(self):
        """Return sector map from last scan, or defaults if no scan yet."""
        if self._last_scan_result and self._last_scan_result.get('sector_map'):
            return list(self._last_scan_result['sector_map'])
        # No scan data — return defaults
        return [(a, SECTOR_DEFAULT_CM, 0.0) for a in self._full_angles]

    def get_min_forward_distance(self):
        """Return minimum distance in the forward sectors (±25°) from last scan."""
        smap = self.get_sector_map()
        centre_dists = [d for a, d, c in smap if -25 <= a <= 25]
        if centre_dists:
            return min(centre_dists)
        return SECTOR_DEFAULT_CM

    @property
    def last_scan_result(self):
        """Most recent scan_for_direction() result dict, or None."""
        return self._last_scan_result

    @property
    def last_scan_time(self):
        """Timestamp of most recent scan."""
        return self._last_scan_time

    @property
    def scan_count(self):
        """Total number of completed scans."""
        return self._scan_count

    @property
    def is_scanning(self):
        """True while a scan is in progress."""
        return self._scanning

    def cleanup(self):
        """Release servo PWM."""
        self.center_servo()
        try:
            self._pwm.stop()
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────────────────────
#  IR OBSTACLE SENSORS  (now read from Pico sensor bridge)
# ─────────────────────────────────────────────────────────────────────────────

class IRSensors:
    """IR obstacle sensors — data sourced from Pico bridge via UART.

    Sensors are physically wired to Pico GPIO 8 (left) and GPIO 9 (right).
    Legacy Pi BCM GPIO 5 / 6 are no longer used."""

    def __init__(self):
        _log.info("IR_INIT via Pico bridge (left=Pico_GPIO8, right=Pico_GPIO9, active LOW)")

    def read(self):
        """Returns (left_obstacle: bool, right_obstacle: bool).
        Returns (False, False) if Pico bridge is unavailable."""
        l, r = pico_get_ir()
        return l, r

# ─────────────────────────────────────────────────────────────────────────────
#  FRONT HC-SR04 SONAR  (primary obstacle detector — static mount)
# ─────────────────────────────────────────────────────────────────────────────

class FrontSonar:
    """Threaded front distance sensor with median filtering.
    Runs at FRONT_SONAR_HZ in the background.  This is the PRIMARY
    obstacle detector — always active while driving."""

    def __init__(self):
        GPIO.setup(FRONT_TRIG_PIN, GPIO.OUT)
        GPIO.setup(FRONT_ECHO_PIN, GPIO.IN)
        GPIO.output(FRONT_TRIG_PIN, False)
        time.sleep(0.05)

        self._dist = -1.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._history = deque(maxlen=5)

        _log.info("FRONT_SONAR_INIT trig=GPIO%d echo=GPIO%d",
                  FRONT_TRIG_PIN, FRONT_ECHO_PIN)

    def _measure_once(self):
        """Single HC-SR04 measurement.  Returns distance in cm or -1."""
        try:
            GPIO.output(FRONT_TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(FRONT_TRIG_PIN, False)

            t0 = time.time()
            timeout = t0 + 0.06
            while GPIO.input(FRONT_ECHO_PIN) == 0:
                t0 = time.time()
                if t0 > timeout:
                    return -1

            t1 = time.time()
            timeout = t1 + 0.06
            while GPIO.input(FRONT_ECHO_PIN) == 1:
                t1 = time.time()
                if t1 > timeout:
                    return -1

            duration = t1 - t0
            dist = round(duration * 17150, 1)
            if dist > 400 or dist < FRONT_MIN_VALID_CM:
                return -1
            return dist
        except Exception:
            return -1

    def read(self):
        """Return distance in cm (-1 if no valid reading)."""
        with self._lock:
            return self._dist

    def start(self):
        """Start background measurement thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        _log.info("FRONT_SONAR_START rate=%d Hz", FRONT_SONAR_HZ)

    def stop(self):
        """Stop measurement thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        _log.info("FRONT_SONAR_STOP")

    def _loop(self):
        """Background measurement loop with median filtering."""
        period = 1.0 / FRONT_SONAR_HZ
        while self._running:
            d = self._measure_once()
            if d > 0:
                self._history.append(d)

            # Median of recent readings
            if self._history:
                med = sorted(self._history)[len(self._history) // 2]
            else:
                med = -1.0

            with self._lock:
                self._dist = med

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
        self._crash_threshold_ms2 = CRASH_G_HIGH_SPEED * ACCEL_G
        self._calibrated = False
        self._last_time = time.time()

        # PID state
        self._integral = 0.0
        self._prev_error = 0.0

        # MPU error tracking
        self._consecutive_errors = 0

    def calibrate(self):
        """Collect CALIBRATION_SAMPLES to compute offsets and noise floor.
        Car MUST be stationary during calibration.
        Samples are read from the Pico sensor bridge via UART."""
        if not _pico_ready():
            _log.warning("IMU_CALIBRATE skipped – Pico bridge not available")
            return {"status": "no_imu"}

        _log.info("IMU_CALIBRATE collecting %d samples via Pico bridge …",
                  CALIBRATION_SAMPLES)

        gz_samples = []
        ax_samples = []
        ay_samples = []
        az_samples = []

        for i in range(CALIBRATION_SAMPLES):
            try:
                pkt = pico_get_sensor_packet()
                if pkt is None:
                    time.sleep(CALIBRATION_INTERVAL)
                    continue
                gz_samples.append(pkt.gyro_z)
                ax_samples.append(pkt.accel_x)
                ay_samples.append(pkt.accel_y)
                az_samples.append(pkt.accel_z)
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
            "crash_threshold_g": CRASH_G_HIGH_SPEED,
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
            pkt = pico_get_sensor_packet()
            if pkt is None:
                raise RuntimeError("No Pico packet available")
            accel = {"x": pkt.accel_x, "y": pkt.accel_y, "z": pkt.accel_z}
            gyro  = {"x": pkt.gyro_x,  "y": pkt.gyro_y,  "z": pkt.gyro_z}
        except Exception as exc:
            self._consecutive_errors += 1
            # Print to console so user sees Pico issues in real-time
            if self._consecutive_errors == 1 or self._consecutive_errors % 5 == 0:
                print(f"\n⚠ PICO BRIDGE READ ERROR #{self._consecutive_errors}: {exc}")
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
            "crash_threshold_g": round(self._crash_threshold_ms2 / ACCEL_G, 2),
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
        AvoidanceZone – one of CLEAR, APPROACH, CAUTION, DANGER, CRITICAL
    """
    if min_fwd <= TANK_TURN_DIST:
        return AvoidanceZone.CRITICAL
    elif min_fwd <= CRITICAL_DIST:
        return AvoidanceZone.DANGER
    elif min_fwd <= WARN_DIST:
        return AvoidanceZone.CAUTION
    elif min_fwd <= APPROACH_DIST:
        return AvoidanceZone.APPROACH
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
        self.sonar   = FrontSonar()
        self.imu     = IMU()
        self.encoder = WheelEncoder()

        # Counters
        self._crash_count = 0
        self._ir_left_count = 0
        self._ir_right_count = 0
        self._boxed_in_count = 0
        self._scan_trigger_count = 0
        self._last_crash_time = 0.0
        self._last_boxed_recovery_time = 0.0

        # Heading tracking
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0

        # Stuck detection
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._critical_stuck_cycles = 0  # consecutive CRITICAL zone cycles

        # Laser scan state
        self._last_scan_time = 0.0
        self._steering_from_scan = None   # cached scan result for steering
        self._steer_direction = None      # 'left', 'right', or 'forward'

        # Navigation state — heading-based "navigate toward open space"
        self._nav_target_heading = None   # None = cruise, float = heading to turn toward
        self._nav_smoothed_best = 0.0     # EMA-smoothed best direction angle
        self._prev_zone = AvoidanceZone.CLEAR  # FSM zone tracking
        self._mpu_fallback_mode = False   # True when MPU has too many errors
        self._fallback_cycles = 0         # cycle counter in fallback mode

        # Slalom-style dodge state
        self._dodge_direction = 0         # -1=left, 0=none, 1=right
        self._last_dodge_time = 0.0       # timestamp of last heading increment
        self._dodge_straight_counter = 0  # cycles of forced straight after heading attenuation

        # Blindspot stuck detection (LM393 + sonar + MPU)
        self._blindspot_start_time = 0.0  # when blindspot condition first detected
        self._blindspot_ref_dist = 0.0    # sonar distance when window started
        self._blindspot_ref_heading = 0.0 # heading when window started
        self._blindspot_active = False    # True while blindspot condition holds

        # Stall detection & throttle ramp (carpet / low battery)
        self._stall_cycles = 0            # consecutive cycles car appears stalled
        self._stall_boost = 0.0           # current PWM boost being applied
        self._stall_ref_heading = 0.0     # heading when stall window started
        self._stall_ref_dist = 0.0        # sonar dist when stall window started
        self._stall_log_counter = 0       # throttle logging frequency

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
        """Start front sonar, center laser servo, then begin driving."""
        # Center laser servo (idle position — ready for on-demand scan)
        self.scanner.center_servo()
        _log.info("SCANNER_CENTERED laser idle, ready for on-demand scan")

        # Start front sonar background polling
        self.sonar.start()

        self._running = True
        self._start_time = time.time()
        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._last_scan_time = 0.0
        self._steering_from_scan = None
        self._steer_direction = None
        self._nav_target_heading = None
        self._nav_smoothed_best = 0.0
        self._mpu_fallback_mode = False
        self._fallback_cycles = 0
        self._blindspot_active = False
        self._blindspot_ref_dist = 0.0
        self._blindspot_ref_heading = 0.0
        self._blindspot_start_time = 0.0
        self._stall_cycles = 0
        self._stall_boost = 0.0
        self._stall_ref_heading = 0.0
        self._stall_ref_dist = 0.0
        self._stall_log_counter = 0

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
            self._last_scan_time = 0.0
            self._steering_from_scan = None
            self._steer_direction = None
            self._nav_target_heading = None
            self._nav_smoothed_best = 0.0
            self._mpu_fallback_mode = False
            self._fallback_cycles = 0
            self._blindspot_active = False
            self._blindspot_ref_dist = 0.0
            self._blindspot_ref_heading = 0.0
            self._blindspot_start_time = 0.0
            self._stall_cycles = 0
            self._stall_boost = 0.0
            self._stall_ref_heading = 0.0
            self._stall_ref_dist = 0.0
            self._stall_log_counter = 0
            self._set_state(State.DRIVING)
            self._drive_thread = threading.Thread(target=self._drive_loop,
                                                   daemon=True)
            self._drive_thread.start()

    def stop(self):
        """Stop driving completely."""
        self._running = False
        self.motor.coast()

    # ── MAIN DRIVE LOOP ─────────────────────────────────────────────────
    #
    #  Architecture:
    #    • Front sonar (HC-SR04) is the PRIMARY always-on sensor
    #    • Laser (servo + VL53L0X) is SECONDARY — on-demand only
    #    • CLEAR  (>100cm):  cruise with PID heading
    #    • APPROACH (80-100cm): pre-emptive laser scan, gentle swerve
    #    • CAUTION (40-80cm): dynamic swerve (30°→75°) + speed reduction
    #    • DANGER  (20-40cm): hard dynamic swerve (75°→85°), outer wheel stays strong
    #    • CRITICAL (≤20cm): brake → scan → sustained tank turn (0.6s)
    #    • Swerve intensity is DYNAMIC — scales with proximity.
    #
    # ─────────────────────────────────────────────────────────────────────

    def _drive_loop(self):
        _log.info("DRIVE_LOOP_START hz=%d sonar_primary laser_on_demand", LOOP_HZ)
        prev_ir_l = False
        prev_ir_r = False
        prev_swerve = 0.0          # for swerve smoothing (EMA)
        cycle = 0
        left_spd = BASE_SPEED      # initialise so crash-threshold calc never sees unbound
        right_spd = BASE_SPEED
        IR_GRACE_CYCLES = 10       # ignore IR for first ~0.2 s (hand on car, sensor bounce)

        while self._running and self.state in (State.DRIVING,
                                                 State.BOXED_IN,
                                                 State.CRASH_RECOVERY):
            t_start = time.time()
            cycle += 1
            self._cycle_count = cycle

            # ── 1. read sensors ──────────────────────────────────────
            imu_data   = self.imu.update()
            ir_l, ir_r = self.ir.read()
            front_dist = self.sonar.read()    # primary obstacle distance

            # Use sonar as the forward distance (primary sensor)
            # Fall back to a large value if sonar returns -1 (no echo)
            if front_dist < 0:
                front_dist = SECTOR_DEFAULT_CM

            # IR activation counters
            if ir_l and not prev_ir_l:
                self._ir_left_count += 1
                _log.warning("IR_LEFT_TRIGGERED count=%d", self._ir_left_count)
            if ir_r and not prev_ir_r:
                self._ir_right_count += 1
                _log.warning("IR_RIGHT_TRIGGERED count=%d", self._ir_right_count)
            prev_ir_l = ir_l
            prev_ir_r = ir_r

            # ── 2. dynamic crash threshold based on current speed ─
            #    At low speed, impacts are softer → lower threshold.
            #    Interpolate between CRASH_G_LOW_SPEED and CRASH_G_HIGH_SPEED
            #    based on effective PWM (average of left+right wheel speeds).
            current_pwm = (left_spd + right_spd) / 2.0 if cycle > 1 else BASE_SPEED
            pwm_lo, pwm_hi = CRASH_G_RANGE_PWM
            pwm_ratio = max(0.0, min(1.0,
                (current_pwm - pwm_lo) / max(1, pwm_hi - pwm_lo)))
            dynamic_crash_g = (CRASH_G_LOW_SPEED +
                (CRASH_G_HIGH_SPEED - CRASH_G_LOW_SPEED) * pwm_ratio)
            self.imu._crash_threshold_ms2 = dynamic_crash_g * ACCEL_G

            # ── 3. crash detection ───────────────────────────────────
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
                self._do_crash_recovery()
                continue

            # ── 4. state-dependent behaviour ─────────────────────────
            if self.state == State.DRIVING:

                # MPU status tracking
                mpu_valid = imu_data.get("mpu_valid", True)
                mpu_errors = imu_data.get("mpu_errors", 0)
                imu_heading = imu_data["heading"]

                if mpu_errors >= MPU_MAX_CONSEC_ERRORS and not self._mpu_fallback_mode:
                    self._mpu_fallback_mode = True
                    self._fallback_cycles = 0
                    _log.warning("MPU_FALLBACK_ENABLED errors=%d", mpu_errors)
                elif mpu_valid and self._mpu_fallback_mode:
                    self._mpu_fallback_mode = False
                    self._fallback_cycles = 0
                    _log.info("MPU_FALLBACK_DISABLED – MPU signal restored")

                # ── Branch 1: IR emergency — reverse, scan, swerve ───
                # Skip IR during startup grace period (hand placement,
                # sensor bounce, etc.)
                if cycle <= IR_GRACE_CYCLES:
                    pass   # ignore IR
                elif ir_l and ir_r:
                    _log.warning("IR_BOTH_BLOCKED ir=[%d,%d] – "
                                 "reversing + pivot", ir_l, ir_r)
                    self._do_ir_both_blocked_recovery()
                    continue
                elif ir_l:
                    _log.warning("IR_LEFT_STUCK – reversing + swerve right")
                    self._do_ir_stuck_recovery('left')
                    continue
                elif ir_r:
                    _log.warning("IR_RIGHT_STUCK – reversing + swerve left")
                    self._do_ir_stuck_recovery('right')
                    continue

                # ── Determine FSM zone from front sonar ──────────────
                zone = get_avoidance_zone(front_dist)
                if zone != self._prev_zone:
                    _log.info("FSM_ZONE %s → %s front_dist=%.1f",
                              self._prev_zone.name, zone.name, front_dist)
                    self._prev_zone = zone

                swerve_angle = 0.0
                use_tank = False
                speed_factor = 1.0
                threat_mag = 0.0
                left_avg = SECTOR_DEFAULT_CM
                right_avg = SECTOR_DEFAULT_CM
                best_angle = 0.0
                best_dist = SECTOR_DEFAULT_CM
                scan_result = self._steering_from_scan
                sector_map = self.scanner.get_sector_map()

                # ═══════════════════════════════════════════════════════
                #  SONAR-PRIMARY NAVIGATION with ON-DEMAND LASER
                #  Swerve intensity is DYNAMIC — scales with proximity.
                # ═══════════════════════════════════════════════════════

                # ── CRITICAL (≤20cm): BRAKE → SCAN once → SUSTAINED TANK TURN ─
                if zone == AvoidanceZone.CRITICAL:
                    self._nav_target_heading = None
                    self._critical_stuck_cycles += 1

                    # If stuck in CRITICAL for too many cycles, reverse out
                    if self._critical_stuck_cycles >= CRITICAL_STUCK_LIMIT:
                        self._critical_stuck_cycles = 0
                        self._boxed_in_count += 1
                        _log.warning(
                            "CRITICAL_STUCK count=%d cycles=%d "
                            "front_dist=%.1f – triggering reverse recovery",
                            self._boxed_in_count,
                            CRITICAL_STUCK_LIMIT, front_dist)
                        self._do_boxed_in_recovery()
                        continue

                    # Use cached scan if fresh (< 2s old), else do ONE scan
                    if scan_result and (t_start - self._last_scan_time) < 2.0:
                        # Reuse the scan we already have — don't re-scan every cycle
                        pass
                    else:
                        self.motor.brake()
                        time.sleep(0.1)
                        _log.info("CRITICAL_SCAN front_dist=%.1f – forced scan",
                                  front_dist)
                        scan_result = self.scanner.scan_for_direction()
                        self._steering_from_scan = scan_result
                        self._last_scan_time = time.time()
                        self._scan_trigger_count += 1

                    left_avg = scan_result['left_avg']
                    right_avg = scan_result['right_avg']
                    sector_map = scan_result['sector_map']

                    if left_avg >= right_avg:
                        turn_dir = 'left'
                        swerve_angle = -SWERVE_MAX_ANGLE
                    else:
                        turn_dir = 'right'
                        swerve_angle = SWERVE_MAX_ANGLE

                    # Sustained tank turn — hold for CRITICAL_TANK_DURATION
                    # so the car actually rotates before the next loop cycle
                    self.motor.tank_turn(turn_dir, TANK_TURN_SPEED)
                    time.sleep(CRITICAL_TANK_DURATION)
                    use_tank = True
                    left_spd = TANK_TURN_SPEED
                    right_spd = TANK_TURN_SPEED
                    self._heading_at_swerve_start = imu_heading
                    self._steer_direction = turn_dir

                    # Re-read sonar after the turn to check if we escaped
                    front_dist_new = self.sonar.read()
                    if front_dist_new > 0:
                        front_dist = front_dist_new
                    # If still critical, the next cycle will re-enter here
                    # but will reuse the cached scan (no re-scan delay)
                    if front_dist > TANK_TURN_DIST:
                        # We escaped critical — invalidate scan so next
                        # zone re-evaluates fresh
                        self._steering_from_scan = None
                        self._critical_stuck_cycles = 0
                        _log.info("CRITICAL_ESCAPED front=%.1f after tank turn",
                                  front_dist)

                    _log.info("CRITICAL_TANK_TURN dir=%s front=%.1f "
                              "L=%.1f R=%.1f phase=%d dur=%.1fs",
                              turn_dir, front_dist, left_avg, right_avg,
                              scan_result['phase'], CRITICAL_TANK_DURATION)

                # ─────────────────────────────────────────────────────────────
                # PROXIMITY-PROPORTIONAL DODGE (DANGER / CAUTION / APPROACH)
                # Direct distance→wheel-speed mapping using SQRT aggression.
                # Swerve is strong from medium distances to avoid obstacles
                # dynamically without stopping/reversing.
                # At 120 cm: firm dodge (outer 63, inner 9)
                # At  80 cm: hard swerve (outer 67, inner 5)
                # At  45 cm: FULL hard turn (outer MAX, inner 3)
                # ─────────────────────────────────────────────────────────────
                elif zone in (AvoidanceZone.DANGER,
                              AvoidanceZone.CAUTION,
                              AvoidanceZone.APPROACH):

                    # ── 1. Pick up completed async scan result FIRST ─
                    #    Must happen BEFORE triggering a new scan,
                    #    otherwise the trigger immediately sets
                    #    _scanning=True and the pickup never fires.
                    #    Also reject stale scan data (>SECTOR_STALE_S old)
                    #    to avoid reusing results from a previous obstacle.
                    if (not self.scanner.is_scanning
                            and self.scanner.last_scan_result
                            and self.scanner.last_scan_time > self._last_scan_time
                            and (t_start - self.scanner.last_scan_time) < SECTOR_STALE_S):
                        scan_result = self.scanner.last_scan_result
                        self._steering_from_scan = scan_result
                        self._last_scan_time = self.scanner.last_scan_time
                        _log.info(
                            "SCAN_PICKUP dir=%s L=%.1f R=%.1f",
                            scan_result['direction'],
                            scan_result['left_avg'],
                            scan_result['right_avg'])

                    # ── 1b. Trigger ASYNC laser scan if cooldown elapsed
                    if ((t_start - self._last_scan_time) > SCAN_COOLDOWN_S
                            and not self.scanner.is_scanning):
                        scan_label = zone.name
                        _log.info("%s_SCAN_ASYNC front_dist=%.1f",
                                  scan_label, front_dist)
                        self.scanner.scan_for_direction_async()
                        self._scan_trigger_count += 1

                    # ── 2. Set dodge direction from scan or keep memory ─
                    prev_dodge_dir = self._dodge_direction

                    if scan_result:
                        left_avg = scan_result['left_avg']
                        right_avg = scan_result['right_avg']
                        best_angle = scan_result['best_angle']
                        sector_map = scan_result['sector_map']
                        direction = scan_result['direction']

                        if direction == 'left':
                            self._dodge_direction = -1
                        elif direction == 'right':
                            self._dodge_direction = 1
                        elif direction == 'forward':
                            # Laser says forward is clear, but sonar may
                            # disagree.  If the sonar still shows an
                            # obstacle ahead, pick the side with more room
                            # instead of keeping a potentially stale dodge
                            # direction.
                            if front_dist < WARN_DIST:
                                if left_avg >= right_avg:
                                    self._dodge_direction = -1
                                else:
                                    self._dodge_direction = 1
                                _log.info(
                                    "SCAN_FWD_OVERRIDE front=%.1f "
                                    "L=%.1f R=%.1f → dodge %s",
                                    front_dist, left_avg, right_avg,
                                    'left' if self._dodge_direction == -1
                                    else 'right')

                    # Default dodge direction if never set
                    if self._dodge_direction == 0:
                        self._dodge_direction = 1  # default right

                    # Set heading reference only on dodge ENTRY or
                    # direction change — NOT every cycle (was breaking
                    # circle detection because delta was always ~0).
                    if prev_dodge_dir != self._dodge_direction:
                        self._heading_at_swerve_start = imu_heading
                        _log.info(
                            "DODGE_DIR_CHANGE %d→%d hdg=%.1f",
                            prev_dodge_dir, self._dodge_direction,
                            imu_heading)

                    # ── 3. Proximity-proportional aggression ──────────
                    # SQUARE-ROOT ramp: swerve is STRONG at medium
                    # distances so the car dynamically curves away well
                    # before reaching the obstacle.  At 100cm the car is
                    # already turning noticeably; by 60cm it's in a hard
                    # swerve.  This all-but-eliminates stop-and-reverse.
                    #   100cm: linear=0.52, sqrt=0.72 → strong curve
                    #    80cm: linear=0.70, sqrt=0.83 → hard swerve
                    #    60cm: linear=0.87, sqrt=0.93 → near-max swerve
                    #    45cm: linear=1.00, sqrt=1.00 → max hard turn
                    linear_aggr = ((DODGE_START_CM - front_dist) /
                                   max(1, DODGE_START_CM - DODGE_HARD_CM))
                    linear_aggr = max(0.0, min(1.0, linear_aggr))
                    aggression = math.sqrt(linear_aggr)  # sqrt — much stronger at medium range

                    # ── 3b. Heading-based attenuation ─────────────────
                    # After turning DODGE_HEADING_EASE degrees, start
                    # reducing dodge.  Prevents circling: the car has
                    # already pointed away from the obstacle.
                    heading_turned = abs(
                        imu_heading - self._heading_at_swerve_start)
                    if heading_turned > DODGE_HEADING_MAX:
                        # Force straighten: car has turned too far
                        aggression = 0.0
                        self._dodge_straight_counter = DODGE_STRAIGHT_CYCLES
                        if heading_turned > DODGE_HEADING_MAX + 5:
                            # Reset heading ref so next dodge is fresh
                            self._heading_at_swerve_start = imu_heading
                    elif heading_turned > DODGE_HEADING_EASE:
                        # Ease off proportionally
                        ease = 1.0 - ((heading_turned - DODGE_HEADING_EASE) /
                                      max(1, DODGE_HEADING_MAX - DODGE_HEADING_EASE))
                        ease = max(0.0, min(1.0, ease))
                        aggression *= ease

                    # ── 3c. Forced straight-ahead after attenuation ───
                    if self._dodge_straight_counter > 0:
                        self._dodge_straight_counter -= 1
                        aggression = 0.0

                    # ── 3d. Compute wheel speeds ─────────────────────
                    # Outer wheel ramps up, inner wheel ramps down
                    outer_spd = (DODGE_OUTER_MIN +
                        (DODGE_OUTER_MAX - DODGE_OUTER_MIN) * aggression)
                    inner_spd = (DODGE_INNER_GENTLE *
                        (1.0 - aggression) + DODGE_INNER_HARD * aggression)

                    # Apply dodge direction
                    if self._dodge_direction == 1:   # turn right
                        left_spd = outer_spd
                        right_spd = inner_spd
                    else:                            # turn left
                        left_spd = inner_spd
                        right_spd = outer_spd

                    # Speed factor & threat for telemetry
                    speed_factor = 1.0 - 0.4 * aggression   # 1.0 → 0.6
                    effective_speed = (left_spd + right_spd) / 2.0
                    threat_mag = aggression

                    # Swerve angle for telemetry (positive = swerving right)
                    swerve_angle = SWERVE_MAX_ANGLE * aggression
                    if self._dodge_direction == -1:
                        swerve_angle = -swerve_angle

                    self._last_dodge_time = t_start

                    _log.debug(
                        "FSM_DODGE zone=%s front=%.1f dodge_dir=%d "
                        "aggr=%.2f outer=%.1f inner=%.1f "
                        "spd=[%.1f,%.1f] L=%.1f R=%.1f "
                        "hdg_turn=%.1f",
                        zone.name, front_dist,
                        self._dodge_direction, aggression,
                        outer_spd, inner_spd,
                        left_spd, right_spd,
                        left_avg, right_avg,
                        heading_turned)

                # ── CLEAR (>100cm): cruise straight ──────────────────
                else:
                    # Clear path — laser idle, sonar monitoring
                    self._steering_from_scan = None
                    self._steer_direction = None
                    self._nav_target_heading = None
                    self._dodge_straight_counter = 0

                    # Mark old scanner result as consumed so it won't be
                    # picked up again when approaching a NEW obstacle.
                    if (self.scanner.last_scan_time
                            and self.scanner.last_scan_time > self._last_scan_time):
                        self._last_scan_time = self.scanner.last_scan_time

                    # Reset dodge direction after delay
                    if (self._dodge_direction != 0 and
                            (t_start - self._last_dodge_time) >
                            DODGE_CLEAR_DELAY_S):
                        self._dodge_direction = 0
                        # Adopt current heading as target so PID doesn't
                        # try to steer back toward the old heading
                        # (which would circle back to the obstacle).
                        self._target_heading = imu_heading
                        self.imu.reset_heading()
                        self._target_heading = 0.0

                    dt = imu_data["dt"]

                    if not self._mpu_fallback_mode:
                        # PID heading correction for straight-line cruise
                        correction = self.imu.pid_correction(
                            self._target_heading, dt)
                        swerve_angle = correction * 0.3
                        # Clamp to prevent wild swerve from PID windup
                        swerve_angle = max(-30.0, min(30.0, swerve_angle))
                        left_spd, right_spd = differential_speeds(
                            BASE_SPEED, swerve_angle)
                    else:
                        # MPU fallback — just drive straight
                        self._fallback_cycles += 1
                        swerve_angle = 0.0
                        left_spd = BASE_SPEED
                        right_spd = BASE_SPEED
                        if self._fallback_cycles % MPU_RETRY_INTERVAL == 0:
                            _log.info("MPU_RETRY_ATTEMPT cycle=%d",
                                      self._fallback_cycles)

                    effective_speed = BASE_SPEED
                    self._heading_at_swerve_start = imu_heading

                # Reset critical stuck counter when not in CRITICAL
                if zone != AvoidanceZone.CRITICAL:
                    self._critical_stuck_cycles = 0

                # ── Stuck detection (uses front sonar distance) ──────
                if front_dist < STUCK_MIN_FWD_CM and not use_tank:
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
                            "heading_change=%.1f front_dist=%.1f",
                            self._boxed_in_count, STUCK_CYCLE_LIMIT,
                            heading_change, front_dist)
                        self._do_boxed_in_recovery()
                        continue
                else:
                    self._stuck_cycles = 0
                    self._stuck_heading_ref = imu_heading

                # ── Blindspot stuck detection (LM393+sonar+MPU) ─────
                # Wheel spinning but car not moving (sonar+heading constant)
                if self.encoder.available and self.encoder.is_spinning:
                    dist_delta = abs(front_dist - self._blindspot_ref_dist)
                    hdg_delta = abs(imu_heading - self._blindspot_ref_heading)
                    if (dist_delta < BLINDSPOT_SONAR_DELTA and
                            hdg_delta < BLINDSPOT_HEADING_DELTA):
                        # Condition holds — clock it
                        if not self._blindspot_active:
                            self._blindspot_active = True
                            self._blindspot_start_time = t_start
                            _log.debug(
                                "BLINDSPOT_START rpm=%.1f front=%.1f "
                                "hdg=%.1f",
                                self.encoder.rpm, front_dist,
                                imu_heading)
                        elif (t_start - self._blindspot_start_time >=
                              BLINDSPOT_TRIGGER_S):
                            # Stuck confirmed!
                            self._blindspot_active = False
                            self._boxed_in_count += 1
                            _log.warning(
                                "BLINDSPOT_STUCK rpm=%.1f front=%.1f "
                                "dist_delta=%.1f hdg_delta=%.1f "
                                "count=%d",
                                self.encoder.rpm, front_dist,
                                dist_delta, hdg_delta,
                                self._boxed_in_count)
                            self._do_boxed_in_recovery()
                            continue
                    else:
                        # Car is actually moving — reset window
                        self._blindspot_active = False
                        self._blindspot_ref_dist = front_dist
                        self._blindspot_ref_heading = imu_heading
                else:
                    # Wheel not spinning or encoder unavailable — reset
                    self._blindspot_active = False
                    self._blindspot_ref_dist = front_dist
                    self._blindspot_ref_heading = imu_heading

                # ── Circle detection ─────────────────────────────────
                heading_delta = abs(imu_heading - self._heading_at_swerve_start)
                if heading_delta > CIRCLE_HEADING_LIMIT and front_dist < WARN_DIST:
                    self._boxed_in_count += 1
                    _log.warning(
                        "CIRCLE_DETECTED count=%d heading_delta=%.1f "
                        "front_dist=%.1f – triggering recovery",
                        self._boxed_in_count, heading_delta, front_dist)
                    self._do_boxed_in_recovery()
                    continue

                # ── Stall detection & throttle ramp (carpet / low battery) ─
                # If motors are commanded but IMU/sonar show no movement,
                # gradually ramp up PWM like pressing the gas pedal harder.
                if not use_tank:
                    cmd_avg = (left_spd + right_spd) / 2.0
                    gz_abs = abs(imu_data.get("gyro", {}).get("z", 0))
                    lat_g_val = imu_data.get("lateral_g", 0)
                    hdg_delta_stall = abs(imu_heading - self._stall_ref_heading)
                    dist_delta_stall = abs(front_dist - self._stall_ref_dist)

                    car_is_moving = (
                        gz_abs > STALL_GZ_THRESH or
                        lat_g_val > STALL_LAT_G_THRESH or
                        hdg_delta_stall > STALL_HDG_DELTA_THRESH or
                        dist_delta_stall > STALL_SONAR_DELTA_THRESH or
                        (self.encoder.available and self.encoder.is_spinning)
                    )

                    if cmd_avg >= STALL_MIN_CMD_PWM and not car_is_moving:
                        self._stall_cycles += 1
                        if self._stall_cycles >= STALL_DETECT_CYCLES:
                            # Confirmed stall — ramp up boost
                            self._stall_boost = min(
                                self._stall_boost + STALL_RAMP_STEP,
                                STALL_RAMP_MAX)
                            self._stall_log_counter += 1
                            if self._stall_log_counter >= STALL_LOG_INTERVAL:
                                self._stall_log_counter = 0
                                _log.info(
                                    "STALL_RAMP boost=%.1f cycles=%d "
                                    "gz=%.2f lat_g=%.3f hdg_d=%.1f "
                                    "dist_d=%.1f cmd=[%.1f,%.1f]",
                                    self._stall_boost, self._stall_cycles,
                                    gz_abs, lat_g_val, hdg_delta_stall,
                                    dist_delta_stall, left_spd, right_spd)
                    else:
                        # Car is moving — decay boost gently
                        if self._stall_boost > 0:
                            self._stall_boost = max(
                                0.0,
                                self._stall_boost - STALL_RAMP_DECAY)
                            if self._stall_boost == 0.0:
                                _log.info("STALL_RAMP_CLEARED after movement")
                        # Reset stall window references
                        self._stall_cycles = 0
                        self._stall_ref_heading = imu_heading
                        self._stall_ref_dist = front_dist
                        self._stall_log_counter = 0

                    # Apply boost to both wheels (capped by _set_duty at MAX_PWM_DUTY)
                    boosted_left = left_spd + self._stall_boost
                    boosted_right = right_spd + self._stall_boost
                    self.motor.forward_differential(boosted_left, boosted_right)

                # ── Update telemetry ─────────────────────────────────
                scan_status = ("SCANNING" if self.scanner.is_scanning
                               else "IDLE" if not scan_result
                               else f"DONE:{scan_result['direction'].upper()}")

                self._update_telem(
                    swerve_angle=round(swerve_angle, 1),
                    speed_factor=round(speed_factor, 2),
                    threat_mag=round(threat_mag, 2),
                    left_speed=round(left_spd, 1),
                    right_speed=round(right_spd, 1),
                    ir_left=ir_l,
                    ir_right=ir_r,
                    front_dist=round(front_dist, 1),
                    heading=imu_data["heading"],
                    accel=imu_data["accel"],
                    gyro_z=round(imu_data["gyro"].get("z", 0), 2),
                    lateral_g=imu_data["lateral_g"],
                    sector_map=[(a, round(d, 1), round(c, 2))
                                for a, d, c in sector_map],
                    crash_count=self._crash_count,
                    ir_left_count=self._ir_left_count,
                    ir_right_count=self._ir_right_count,
                    boxed_in_count=self._boxed_in_count,
                    scan_trigger_count=self._scan_trigger_count,
                    cycle_hz=round(1.0 / max(0.001,
                                   time.time() - t_start), 1),
                    uptime=round(time.time() - self._start_time, 1),
                    mpu_fallback=self._mpu_fallback_mode,
                    mpu_errors=mpu_errors,
                    avoidance_zone=zone.name,
                    tank_turn=use_tank,
                    scan_status=scan_status,
                    scan_left_avg=round(left_avg, 1),
                    scan_right_avg=round(right_avg, 1),
                    scan_direction=self._steer_direction or "",
                    best_angle=round(best_angle, 1),
                    best_dist=round(best_dist, 1),
                    encoder_rpm=round(self.encoder.rpm, 1),
                    encoder_spinning=self.encoder.is_spinning,
                    encoder_available=self.encoder.available,
                    blindspot_stuck=self._blindspot_active,
                    crash_threshold_g=round(
                        self.imu._crash_threshold_ms2 / ACCEL_G, 2),
                )

            # ── 4. comprehensive sensor log ──────────────────────────
            if cycle % SENSOR_LOG_EVERY_N == 0:
                _log.debug(
                    "CYCLE=%d st=%s zone=%s spd=[%.1f,%.1f] swerve=%.1f "
                    "sf=%.2f threat=%.2f front=%.1f tank=%s "
                    "ir=[%d,%d] scan=%s "
                    "hdg=%.1f gz=%.2f lat_g=%.3f "
                    "enc_rpm=%.1f enc_spin=%s blind=%s crash_g=%.2f "
                    "stall_boost=%.1f",
                    cycle,
                    self.state.name,
                    self._telem.get("avoidance_zone", "?"),
                    self._telem.get("left_speed", 0),
                    self._telem.get("right_speed", 0),
                    self._telem.get("swerve_angle", 0),
                    self._telem.get("speed_factor", 1),
                    self._telem.get("threat_mag", 0),
                    front_dist,
                    self._telem.get("tank_turn", False),
                    ir_l, ir_r,
                    self._telem.get("scan_status", "?"),
                    imu_data["heading"],
                    imu_data["gyro"].get("z", 0),
                    imu_data["lateral_g"],
                    self.encoder.rpm,
                    self.encoder.is_spinning,
                    self._blindspot_active,
                    self.imu._crash_threshold_ms2 / ACCEL_G,
                    self._stall_boost,
                )

            # ── 5. pace the loop ─────────────────────────────────────
            elapsed = time.time() - t_start
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        _log.info("DRIVE_LOOP_END cycles=%d", cycle)

    # ── BOXED-IN recovery ────────────────────────────────────────────────

    def _do_boxed_in_recovery(self):
        """Recovery from boxed-in / stuck situation.

        Strategy (improved):
          1. Check front sonar distance
          2. If front is close (< REVERSE_BEFORE_TURN_DIST) OR this is a
             repeated stuck event (boxed_in_count > 1), REVERSE first to
             create clearance — polling front sonar every REVERSE_CHECK_INTERVAL
             to stop early if sufficient clearance is gained.
          3. Laser scan for clearest side
          4. Tank turn toward clearest direction
          5. Verify front clearance improved; if not, reverse more

        Reverse duration escalates with consecutive boxed-in events so the
        car doesn't keep doing tiny reverses that don't help.
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.2)

        # Reset stall boost — recovery action will reposition the car
        self._stall_boost = 0.0
        self._stall_cycles = 0
        self._stall_log_counter = 0

        # Read current front distance
        front_dist = self.sonar.read()
        if front_dist < 0:
            front_dist = SECTOR_DEFAULT_CM

        # Determine if we should reverse first
        need_reverse = (
            front_dist < REVERSE_BEFORE_TURN_DIST or
            (REPEATED_STUCK_REVERSE and self._boxed_in_count > 1) or
            (time.time() - self._last_boxed_recovery_time) < 5.0  # recent recovery failed
        )

        # Escalating reverse duration based on consecutive boxed-in count
        reverse_dur = min(
            BLIND_REVERSE_DURATION + (self._boxed_in_count - 1) * REVERSE_ESCALATE_STEP,
            REVERSE_MAX_DURATION
        )

        if need_reverse:
            _log.info("BOXED_IN_REVERSE front=%.1f count=%d dur=%.1fs",
                      front_dist, self._boxed_in_count, reverse_dur)
            self._do_monitored_reverse(reverse_dur)

            # Re-read front distance after reverse
            front_dist_after = self.sonar.read()
            if front_dist_after < 0:
                front_dist_after = front_dist  # fallback
            _log.info("BOXED_IN_POST_REVERSE front_before=%.1f front_after=%.1f",
                      front_dist, front_dist_after)
        else:
            _log.info("BOXED_IN_NO_REVERSE front=%.1f count=%d (skip reverse)",
                      front_dist, self._boxed_in_count)

        # Get fresh laser scan data
        _log.info("BOXED_IN_SCANNING for escape direction")
        scan = self.scanner.scan_for_direction()
        left_avg  = scan['left_avg']
        right_avg = scan['right_avg']
        left_max  = max((d for a, d, _ in scan['sector_map'] if a < -5),
                        default=0)
        right_max = max((d for a, d, _ in scan['sector_map'] if a > 5),
                        default=0)
        fwd_dist  = scan.get('forward', 0)

        _log.info("BOXED_IN_RECOVERY left_avg=%.1f right_avg=%.1f "
                  "left_max=%.1f right_max=%.1f fwd=%.1f",
                  left_avg, right_avg, left_max, right_max, fwd_dist)

        # ── Decide turn direction ────────────────────────────────────
        if left_max >= SIDE_CLEAR_MIN_CM or right_max >= SIDE_CLEAR_MIN_CM:
            if left_avg >= right_avg:
                turn_dir = 'left'
                _log.info("RECOVERY_TANK_TURN LEFT left_avg=%.1f right_avg=%.1f",
                          left_avg, right_avg)
            else:
                turn_dir = 'right'
                _log.info("RECOVERY_TANK_TURN RIGHT left_avg=%.1f right_avg=%.1f",
                          left_avg, right_avg)
        else:
            # All sides blocked — pick less-blocked side
            turn_dir = 'left' if left_avg >= right_avg else 'right'
            _log.info("RECOVERY_ALL_BLOCKED turn=%s left_avg=%.1f right_avg=%.1f",
                      turn_dir, left_avg, right_avg)
            # If we haven't reversed yet, do it now
            if not need_reverse:
                _log.info("BOXED_IN_LATE_REVERSE (all blocked) dur=%.1fs",
                          reverse_dur)
                self._do_monitored_reverse(reverse_dur)

        # ── Tank turn ────────────────────────────────────────────────
        # Longer turn for repeated boxed-in events
        turn_dur = RECOVERY_TURN_DURATION + (
            0.3 * max(0, self._boxed_in_count - 1)
        )
        turn_dur = min(turn_dur, 3.0)  # cap at 3s

        self.motor.tank_turn(turn_dir, TANK_TURN_SPEED)
        time.sleep(turn_dur)
        self.motor.coast()
        time.sleep(0.1)

        # ── Verify front clearance after turn ────────────────────────
        front_after_turn = self.sonar.read()
        if front_after_turn < 0:
            front_after_turn = SECTOR_DEFAULT_CM
        _log.info("BOXED_IN_POST_TURN front=%.1f (was %.1f)",
                  front_after_turn, front_dist)

        # If front is still dangerously close, reverse a bit more
        if front_after_turn < REVERSE_BEFORE_TURN_DIST:
            extra_rev = min(0.5 + 0.2 * self._boxed_in_count, 1.5)
            _log.info("BOXED_IN_EXTRA_REVERSE front=%.1f < %.1f, extra=%.1fs",
                      front_after_turn, REVERSE_BEFORE_TURN_DIST, extra_rev)
            self._do_monitored_reverse(extra_rev)

        self._reset_nav_state()
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    def _do_monitored_reverse(self, duration):
        """Reverse while polling front sonar to stop early if enough
        clearance is gained.  No rear sensor — uses timed burst with
        front sonar monitoring for safety."""
        self.motor.reverse_straight(BLIND_REVERSE_SPEED)
        start = time.time()
        while (time.time() - start) < duration:
            time.sleep(REVERSE_CHECK_INTERVAL)
            d = self.sonar.read()
            if d > 0 and d >= REVERSE_MIN_CLEARANCE_CM:
                _log.info("MONITORED_REVERSE_EARLY_STOP front=%.1f >= %.1f after %.2fs",
                          d, REVERSE_MIN_CLEARANCE_CM, time.time() - start)
                break
        self.motor.brake()
        time.sleep(0.15)

    # ── IR STUCK recovery (single IR) ────────────────────────────────────

    def _do_ir_stuck_recovery(self, stuck_side):
        """Recovery when a single IR sensor fires — car is stuck on one side.

        Strategy:
          1. Brake
          2. Blind reverse briefly to create clearance
          3. Laser scan for best direction
          4. Tank turn AWAY from the stuck side

        stuck_side: 'left' or 'right'
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.15)

        # Reset stall boost
        self._stall_boost = 0.0
        self._stall_cycles = 0

        # Blind reverse briefly to create clearance
        _log.info("IR_STUCK_BLIND_REVERSE side=%s speed=%d dur=%.1fs",
                  stuck_side, BLIND_REVERSE_SPEED, IR_STUCK_REVERSE_DUR)
        self.motor.reverse_straight(BLIND_REVERSE_SPEED)
        time.sleep(IR_STUCK_REVERSE_DUR)
        self.motor.brake()
        time.sleep(0.1)

        # Laser scan to confirm best escape direction
        scan = self.scanner.scan_for_direction()
        _log.info("IR_STUCK_SCAN dir=%s L=%.1f R=%.1f",
                  scan['direction'], scan['left_avg'], scan['right_avg'])

        # Tank turn away from the stuck side
        if stuck_side == 'left':
            _log.info("IR_STUCK_TANK_RIGHT after reverse")
            self.motor.tank_turn('right', TANK_TURN_SPEED)
        else:
            _log.info("IR_STUCK_TANK_LEFT after reverse")
            self.motor.tank_turn('left', TANK_TURN_SPEED)

        time.sleep(RECOVERY_TURN_DURATION)
        self.motor.coast()
        time.sleep(0.1)

        self._reset_nav_state()
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    # ── IR BOTH-BLOCKED recovery ─────────────────────────────────────────

    def _do_ir_both_blocked_recovery(self):
        """Recovery when BOTH IR sensors fire — front is completely blocked.

        Strategy:
          1. Brake
          2. Blind reverse (short timed burst — no rear sensor)
          3. Laser scan for clearest side
          4. Tank turn toward clearest side
        """
        self._set_state(State.BOXED_IN)
        self.motor.brake()
        time.sleep(0.2)

        # Blind reverse
        _log.info("IR_BOTH_BLOCKED_BLIND_REVERSE speed=%d dur=%.1fs",
                  BLIND_REVERSE_SPEED, BLIND_REVERSE_DURATION)
        self.motor.reverse_straight(BLIND_REVERSE_SPEED)
        time.sleep(BLIND_REVERSE_DURATION)
        self.motor.brake()
        time.sleep(0.2)

        # Laser scan for best direction
        scan = self.scanner.scan_for_direction()
        left_avg  = scan['left_avg']
        right_avg = scan['right_avg']

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

        self._reset_nav_state()
        self._last_boxed_recovery_time = time.time()
        self._set_state(State.DRIVING)

    # ── CRASH recovery ───────────────────────────────────────────────────

    def _do_crash_recovery(self):
        """Handle crash event: brake, blind reverse, laser scan, turn."""
        self._set_state(State.CRASH_RECOVERY)

        # Reset stall boost
        self._stall_boost = 0.0
        self._stall_cycles = 0
        self._stall_log_counter = 0

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

        # Blind reverse away from crash site
        _log.info("CRASH_BLIND_REVERSE speed=%d dur=%.1fs",
                  BLIND_REVERSE_SPEED, CRASH_REVERSE_DURATION)
        self.motor.reverse_straight(BLIND_REVERSE_SPEED)
        time.sleep(CRASH_REVERSE_DURATION)
        self.motor.brake()
        time.sleep(0.3)

        # Laser scan for escape direction
        scan = self.scanner.scan_for_direction()
        left_avg  = scan['left_avg']
        right_avg = scan['right_avg']

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

        self._reset_nav_state()
        _log.info("CRASH_RECOVERY_DONE count=%d", self._crash_count)
        self._set_state(State.DRIVING)

    # ── Navigation state reset helper ────────────────────────────────────

    def _reset_nav_state(self):
        """Reset all navigation tracking state after a recovery event."""
        self.imu.reset_heading()
        self._target_heading = 0.0
        self._heading_at_swerve_start = 0.0
        self._stuck_cycles = 0
        self._stuck_heading_ref = 0.0
        self._critical_stuck_cycles = 0
        self._dodge_direction = 0
        self._last_dodge_time = 0.0
        self._dodge_straight_counter = 0
        self._blindspot_active = False
        self._blindspot_ref_dist = 0.0
        self._blindspot_ref_heading = 0.0
        self._blindspot_start_time = 0.0
        self._steering_from_scan = None
        self._steer_direction = None
        self._last_scan_time = 0.0
        self._nav_target_heading = None

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
            "ir_left=%d  ir_right=%d  boxed_in=%d  scans=%d  cycles=%d",
            uptime,
            self._crash_count,
            self._ir_left_count,
            self._ir_right_count,
            self._boxed_in_count,
            self._scan_trigger_count,
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
                controller._reset_nav_state()
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

        header = f"  AUTONOMOUS DRIVE (Sonar+Laser)  |  {controller.state.name}  |  " \
                 f"Cycles: {controller._cycle_count}"
        _safe(row, 0, header, COL_HDR)
        row += 2

        # Front sonar distance (primary sensor)
        fd = t.get("front_dist", -1)
        front_col = (COL_OK if fd > LASER_TRIGGER_DIST
                     else COL_WARN if fd > CRITICAL_DIST
                     else COL_DANG)
        _safe(row, 0, f"Front Sonar: {fd:.1f}cm", front_col)

        # Laser scan status
        scan_st = t.get("scan_status", "IDLE")
        scan_col = COL_WARN if "DONE" in scan_st else COL_INFO
        _safe(row, 28, f"Laser: {scan_st}  "
              f"L={t.get('scan_left_avg', 0):.0f}cm  "
              f"R={t.get('scan_right_avg', 0):.0f}cm  "
              f"Scans={t.get('scan_trigger_count', 0)}", scan_col)
        row += 2

        # Sector map bar chart (from last scan — may be stale)
        sm = t.get("sector_map", [])
        scan_fresh = len(sm) > 0 and any(c > 0 for _, _, c in sm)
        if scan_fresh:
            _safe(row, 0, "Last Laser Scan:", COL_INFO)
            row += 1
            bar_max = 30
            for angle, dist, conf in sm:
                bar_len = int(min(dist, 120) / 120 * bar_max)
                bar_char = "\u2588" * bar_len
                bar_empty = "\u2591" * (bar_max - bar_len)
                col = COL_OK if dist > LASER_TRIGGER_DIST else (COL_WARN if dist > CRITICAL_DIST else COL_DANG)
                label = f"{angle:+4d}\u00b0 {dist:5.0f}cm "
                _safe(row, 0, label + bar_char + bar_empty, col)
                row += 1
        else:
            _safe(row, 0, "Laser: idle (no scan data)", COL_INFO)
            row += 1

        row += 1

        # Speed / swerve / zone
        zone_name = t.get('avoidance_zone', 'CLEAR')
        zone_col = (COL_DANG if zone_name in ('CRITICAL', 'DANGER')
                    else COL_WARN if zone_name in ('CAUTION', 'APPROACH')
                    else COL_OK)
        tank_str = " TANK" if t.get('tank_turn', False) else ""
        _safe(row, 0,
              f"Speed: L={t.get('left_speed', 0):5.1f}"
              f"%  R={t.get('right_speed', 0):5.1f}"
              f"%  |  Swerve: {t.get('swerve_angle', 0):+6.1f}\u00b0"
              f"  SF={t.get('speed_factor', 1):.2f}"
              f"  Threat={t.get('threat_mag', 0):.2f}",
              COL_INFO)
        row += 1
        _safe(row, 0, f"Zone: {zone_name}{tank_str}", zone_col)
        row += 1

        # IR status
        ir_l_str = "\u25a0 LEFT"  if t.get("ir_left", False) else "\u25a1 left"
        ir_r_str = "\u25a0 RIGHT" if t.get("ir_right", False) else "\u25a1 right"
        ir_l_col = COL_DANG if t.get("ir_left", False) else COL_OK
        ir_r_col = COL_DANG if t.get("ir_right", False) else COL_OK
        _safe(row, 0, f"IR: ", COL_INFO)
        _safe(row, 4, ir_l_str, ir_l_col)
        _safe(row, 15, ir_r_str, ir_r_col)
        row += 1

        # Heading
        _safe(row, 0,
              f"Heading: {t.get('heading', 0):.1f}\u00b0  "
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
