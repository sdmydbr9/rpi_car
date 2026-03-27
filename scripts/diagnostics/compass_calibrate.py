#!/usr/bin/env python3
"""
compass_calibrate_and_test.py — Full Auto Compass Calibration + Cardinal Test
==============================================================================
Phase 1 — AUTO CALIBRATION:
    Places the Ackermann rover on open ground, drives alternating
    full-lock circles for 20 seconds, and fits hard-iron (offset) and
    soft-iron (scale) corrections from the min/max envelope of the raw
    magnetometer readings.

Phase 2 — AUTO CARDINAL TEST:
    Uses the computed corrections to drive the car to each of N, E, S, W
    via closed-loop full-lock turning arcs, then measures the heading error.
"""

import os
import sys
import json
import time
import math
from dataclasses import dataclass

# ── path setup ────────────────────────────────────────────────────────────────
_DIAG_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_DIAG_DIR, '..', 'core'))

try:
    from pico_sensor_reader import PicoSensorReader, init_pico_reader
except ImportError as e:
    print(f"❌  Cannot import PicoSensorReader: {e}")
    sys.exit(1)

try:
    from motor import CarSystem
except ImportError as e:
    print(f"❌  Cannot import CarSystem: {e}")
    sys.exit(1)

# ── calibration file ─────────────────────────────────────────────────────────
CAL_FILE = os.path.join(_DIAG_DIR, 'compass_cal.json')


# ── compass manager ───────────────────────────────────────────────────────────
@dataclass
class CompassManager:
    """Encapsulates hard-iron and soft-iron calibration state and heading math."""
    offset_x: float = 0.0
    offset_y: float = 0.0
    scale_x: float = 1.0
    scale_y: float = 1.0

    def compute_heading(self, mag_x: float, mag_y: float) -> float:
        """Return a corrected CW bearing (0-360°) from raw magnetometer data."""
        cx = (mag_x - self.offset_x) * self.scale_x
        cy = (mag_y - self.offset_y) * self.scale_y
        # Negative sign corrects for physical board inversion.
        # +90° corrects for the sensor being mounted 90° CCW on the board.
        return (-math.degrees(math.atan2(cy, cx)) + 90.0) % 360.0

    def save(self, filepath: str):
        data = {
            'offset_x': self.offset_x,
            'offset_y': self.offset_y,
            'scale_x':  self.scale_x,
            'scale_y':  self.scale_y,
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"  💾  Calibration saved to {filepath}")

    @classmethod
    def load(cls, filepath: str):
        if not os.path.exists(filepath):
            return None
        try:
            with open(filepath) as f:
                d = json.load(f)
            return cls(
                offset_x=d['offset_x'],
                offset_y=d['offset_y'],
                scale_x=d['scale_x'],
                scale_y=d['scale_y']
            )
        except Exception as e:
            print(f"  ⚠️   Could not read calibration file: {e}")
            return None


# ── constants ─────────────────────────────────────────────────────────────────
SPIN_TIME          = 20.0   # seconds of motor-driven circle runs for calibration
SPIN_SPEED_START   = 35     # PWM % — initial slow circle speed
SPIN_SPEED_MAX     = 100     # PWM % — ceiling before giving up
SPIN_SPEED_STEP    = 5      # PWM bump each time a stall is detected
SPIN_STALL_WINDOW  = 1.2    # seconds of span-freeze before declaring a stall
SPIN_STALL_THRESH  = 2.0    # min total span growth (raw counts) to count as moving
SPIN_DIR_INTERVAL  = 5.0    # seconds per direction (left → right → left …)
SAMPLE_HZ          = 50     # magnetometer poll rate (Hz)
MOTOR_SAFETY_DWELL = 0.15   # H-bridge direction-change dwell (matches CarSystem)
BATTERY_DIVIDER_RATIO = 5.0  # matches pico_sensor_reader.get_battery_voltage()
CURRENT_SHUNT_OHMS   = 0.1   # matches pico_sensor_reader.get_current_sense()

# Motion-confirmation thresholds (used before the 20 s timer starts)
MOTION_GYRO_THRESH  = 5.0   # deg/s on gyro_z — car is definitely rotating
MOTION_RPM_THRESH   = 2.0   # RPM on any wheel encoder — wheels are turning
MOTION_CONFIRM_S    = 2.5   # seconds to observe movement at each PWM level
MOTION_MAX_ATTEMPTS = 8     # max PWM bumps before aborting

# Forward/backward wiggle — breaks static friction before turn attempts
WIGGLE_SPEED   = 55    # PWM % for the unstick pulses
WIGGLE_PULSE_S = 0.35  # duration of each forward / reverse pulse
WIGGLE_COUNT   = 3     # number of fwd+rev cycles per wiggle

# Ackermann steering geometry for this diagnostic
CAL_STEER_ANGLE_DEG  = 45.0  # fixed circle angle during calibration runs
TURN_STEER_MIN_DEG   = 18.0  # minimum steering angle that reliably changes heading
TURN_STEER_MAX_DEG   = 45.0  # stay just inside the hardware full-lock limit
TURN_STEER_GAIN      = 1.15  # heading error → steering angle gain

# Cardinal facing parameters
FACE_TOLERANCE_DEG = 5.0    # stop turning when within ±5° of target
FACE_SPEED_FAST    = 35     # PWM % while error is still large
FACE_SPEED_SLOW    = 25     # PWM % when trimming in near the target
FACE_SLOWDOWN_DEG  = 20.0   # switch to the slow trim speed inside this window
FACE_TIMEOUT_S      = 24.0  # allow time for multi-segment Ackermann manoeuvres
FACE_HEADING_SAMPLES = 6    # averaged readings for start/end of each manoeuvre
FACE_SEGMENT_SETTLE_S = 0.18  # short settle between arc segments
FACE_SEGMENT_MIN_PROGRESS_DEG = 1.5  # less than this counts as a stalled segment
FACE_LARGE_ERR_DEG = 70.0   # use multi-point turn strategy above this error
FACE_MEDIUM_ERR_DEG = 28.0  # use medium forward arc strategy above this error
SETTLE_S           = 0.4    # seconds to let car settle before reading final heading
VERIFY_SAMPLES     = 25     # readings averaged for final heading measurement


# ── heading math helpers ───────────────────────────────────────────────────────

def circular_mean(angles: list) -> float:
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return (math.degrees(math.atan2(s, c)) + 360.0) % 360.0


def angular_error(measured: float, expected: float) -> float:
    """Signed shortest-path angle difference (−180 ↔ +180)."""
    return ((measured - expected + 180.0) % 360.0) - 180.0


# ── motion detection helper ──────────────────────────────────────────────────

def _packet_value(pkt, *names, default=0.0):
    """Read the first available attribute from a packet, tolerating old schemas."""
    if pkt is None:
        return default
    for name in names:
        if hasattr(pkt, name):
            value = getattr(pkt, name)
            if value is not None:
                return value
    return default


def _packet_motion_snapshot(pkt) -> dict:
    """Return a schema-tolerant motion summary from the latest Pico packet."""
    rpm_left = float(_packet_value(pkt, 'rpm_left', 'rpm_rear_left', default=0.0))
    rpm_right = float(_packet_value(pkt, 'rpm_right', 'rpm_rear_right', default=0.0))
    rpm_candidates = [
        rpm_left,
        rpm_right,
        float(_packet_value(pkt, 'rpm_left_raw', default=0.0)),
        float(_packet_value(pkt, 'rpm_right_raw', default=0.0)),
        float(_packet_value(pkt, 'rpm_front_right', default=0.0)),
    ]
    return {
        'gyro_z': abs(float(_packet_value(pkt, 'gyro_z', default=0.0))),
        'rpm_left': rpm_left,
        'rpm_right': rpm_right,
        'rpm_max': max((abs(v) for v in rpm_candidates), default=0.0),
        'duty_left': float(_packet_value(pkt, 'mot_duty_left', default=0.0)),
        'duty_right': float(_packet_value(pkt, 'mot_duty_right', default=0.0)),
    }


def _sync_power_state(car: CarSystem, pkt):
    """Feed live battery/current telemetry into CarSystem's power limiter."""
    if pkt is None:
        return
    batt_mv = float(_packet_value(pkt, 'adc_a0', default=-1.0))
    if batt_mv < 0:
        return
    current_mv = float(_packet_value(pkt, 'adc_a1', default=-1.0))
    battery_v = (batt_mv / 1000.0) * BATTERY_DIVIDER_RATIO
    current_a = 0.0
    if current_mv >= 0:
        current_a = max(0.0, (current_mv / 1000.0) / CURRENT_SHUNT_OHMS)
    car.update_power_state(battery_v, current_a)


def _packet_power_snapshot(pkt) -> dict:
    """Return a schema-tolerant battery/current snapshot from the latest packet."""
    if pkt is None:
        return {'battery_v': None, 'current_a': None}
    batt_mv = float(_packet_value(pkt, 'adc_a0', default=-1.0))
    current_mv = float(_packet_value(pkt, 'adc_a1', default=-1.0))
    battery_v = None if batt_mv < 0 else (batt_mv / 1000.0) * BATTERY_DIVIDER_RATIO
    current_a = None
    if current_mv >= 0:
        current_a = max(0.0, (current_mv / 1000.0) / CURRENT_SHUNT_OHMS)
    return {
        'battery_v': battery_v,
        'current_a': current_a,
    }


def _motion_diag_string(pkt) -> str:
    """Compact motion/power diagnostics string for no-movement reports."""
    snap = _packet_motion_snapshot(pkt)
    power = _packet_power_snapshot(pkt)
    batt = power['battery_v']
    current = power['current_a']
    batt_txt = f"{batt:.2f}V" if batt is not None else "N/A"
    current_txt = f"{current:.2f}A" if current is not None else "N/A"
    return (
        f"gyro_z={snap['gyro_z']:.1f}°/s  max_rpm={snap['rpm_max']:.1f}  "
        f"duty L/R={snap['duty_left']:.0f}/{snap['duty_right']:.0f}%  "
        f"battery={batt_txt}  current={current_txt}"
    )


def _is_moving(pkt) -> bool:
    """Return True if the sensor packet indicates the car is rotating."""
    if pkt is None:
        return False
    snapshot = _packet_motion_snapshot(pkt)
    # Gyro yaw-rate check (MPU6500 gyro_z, deg/s)
    if snapshot['gyro_z'] >= MOTION_GYRO_THRESH:
        return True
    # Wheel encoder check (current schema: left/right; old schema tolerated)
    if snapshot['rpm_max'] >= MOTION_RPM_THRESH:
        return True
    return False


def _drive_ackermann(car: CarSystem, speed: float, steering_angle_deg: float, forward: bool = True):
    """Drive using Ackermann steering + rear-wheel drive in a single command."""
    # Clamp input degrees to physical limits
    steering_angle_deg = max(-TURN_STEER_MAX_DEG, min(TURN_STEER_MAX_DEG, steering_angle_deg))
    # Convert degrees to CarSystem's -50 to +50 unit range
    # ±45° mechanical maps to ±50 units
    steering_angle_units = (steering_angle_deg / TURN_STEER_MAX_DEG) * 50.0
    if hasattr(car, '_apply_steering'):
        car._apply_steering(speed, steering_angle_units, forward=forward)
        car._steering_angle = steering_angle_units
        car._current_speed = speed
        return
    if forward:
        car.set_steering(steering_angle_units)
        car.set_speed(speed)
    elif hasattr(car, 'reverse_steer'):
        car.reverse_steer(speed, steering_angle_units)
    else:
        car.reverse(speed)


def _drive_turn_arc(car: CarSystem, direction: str, speed: float, steer_angle: float):
    """Turn by steering the front axle and powering the rear wheels forward."""
    signed_angle = -abs(steer_angle) if direction == "left" else abs(steer_angle)
    _drive_ackermann(car, speed, signed_angle, forward=True)


def _alignment_direction(err_deg: float) -> str:
    """Return the turn direction that reduces the current heading error."""
    return "left" if err_deg > 0 else "right"


def _steer_for_heading_error(err_deg: float) -> float:
    """Map heading error magnitude to a smooth Ackermann steering demand."""
    mag = abs(err_deg)
    if mag < FACE_SLOWDOWN_DEG:
        return min(30.0, max(TURN_STEER_MIN_DEG, mag * 0.85))
    return min(TURN_STEER_MAX_DEG, max(TURN_STEER_MIN_DEG, mag * TURN_STEER_GAIN))


def _signed_steer_for_segment(direction: str, steer_abs: float, forward: bool) -> float:
    """Return the signed Ackermann steering angle for a segment.

    When reversing, the steering sign flips so the rover keeps rotating in the
    same yaw direction during the backing-up phase of a multi-point turn.
    """
    signed = -abs(steer_abs) if direction == "left" else abs(steer_abs)
    return signed if forward else -signed


def _yaw_progress_deg(start_heading: float, current_heading: float, direction: str) -> float:
    """Return positive progress in the commanded yaw direction."""
    delta = angular_error(current_heading, start_heading)
    return max(0.0, -delta if direction == "left" else delta)


def _heading_snapshot(pico: PicoSensorReader, compass: CompassManager, n: int = FACE_HEADING_SAMPLES) -> float | None:
    """Short averaged heading snapshot used between Ackermann manoeuvre segments."""
    return averaged_heading(pico, compass, n=n)


def _segment_plan(err_deg: float, forward_floor: float, reverse_floor: float) -> list[dict]:
    """Return the next Ackermann manoeuvre plan for the current heading error."""
    mag = abs(err_deg)
    if mag >= FACE_LARGE_ERR_DEG:
        return [
            {
                'label': 'fwd',
                'forward': True,
                'speed': max(forward_floor, FACE_SPEED_FAST + 5),
                'steer_abs': TURN_STEER_MAX_DEG,
                'goal_deg': min(22.0, max(12.0, mag * 0.18)),
                'max_time_s': 2.4,
            },
            {
                'label': 'rev',
                'forward': False,
                'speed': max(reverse_floor, FACE_SPEED_SLOW + 5),
                'steer_abs': TURN_STEER_MAX_DEG,
                'goal_deg': min(12.0, max(5.0, mag * 0.08)),
                'max_time_s': 1.8,
            },
        ]
    if mag >= FACE_MEDIUM_ERR_DEG:
        steer_abs = min(TURN_STEER_MAX_DEG, max(28.0, mag * 0.55))
        return [
            {
                'label': 'arc',
                'forward': True,
                'speed': max(forward_floor, FACE_SPEED_FAST),
                'steer_abs': steer_abs,
                'goal_deg': min(12.0, max(4.0, mag * 0.22)),
                'max_time_s': 1.8,
            },
        ]
    steer_abs = min(30.0, max(TURN_STEER_MIN_DEG, mag * 0.85))
    return [
        {
            'label': 'trim',
            'forward': True,
            'speed': max(forward_floor, FACE_SPEED_SLOW),
            'steer_abs': steer_abs,
            'goal_deg': min(4.0, max(1.5, mag * 0.35)),
            'max_time_s': 1.2,
        },
    ]


def _run_alignment_segment(car: CarSystem, pico: PicoSensorReader, compass: CompassManager,
                           direction: str, speed: float, steer_abs: float,
                           forward: bool, goal_deg: float, max_time_s: float) -> dict:
    """Execute one Ackermann arc segment and report how much yaw it achieved."""
    steering_angle = _signed_steer_for_segment(direction, steer_abs, forward)
    start_heading = _heading_snapshot(pico, compass, n=4)
    if start_heading is None:
        return {
            'start_heading': None,
            'end_heading': None,
            'progress_deg': 0.0,
            'moving': False,
            'steering_angle': steering_angle,
        }
    deadline = time.monotonic() + max_time_s
    best_progress = 0.0
    moving = False
    last_heading = start_heading

    while time.monotonic() < deadline:
        pkt = pico.get_latest()
        _sync_power_state(car, pkt)
        heading = heading_from_packet(pkt, compass)
        if heading is not None:
            last_heading = heading
            best_progress = max(
                best_progress,
                _yaw_progress_deg(start_heading, heading, direction),
            )
            if best_progress >= goal_deg:
                break

        _drive_ackermann(car, speed, steering_angle, forward=forward)
        if _is_moving(pkt):
            moving = True
        time.sleep(0.02)

    car.stop()
    time.sleep(FACE_SEGMENT_SETTLE_S)
    end_heading = _heading_snapshot(pico, compass, n=4)
    if end_heading is None:
        end_heading = last_heading
    if end_heading is not None:
        best_progress = max(
            best_progress,
            _yaw_progress_deg(start_heading, end_heading, direction),
        )

    return {
        'start_heading': start_heading,
        'end_heading': end_heading,
        'progress_deg': best_progress,
        'moving': moving,
        'steering_angle': steering_angle,
    }


def _break_static_friction(car: CarSystem, speed: int):
    """
    Rock the car forward then backward WIGGLE_COUNT times to break
    static friction before an Ackermann full-lock turn.
    Uses the same *speed* as the upcoming circle so the motors are warm.
    """
    fwd_speed = min(max(speed + 10, WIGGLE_SPEED), 80)
    # Always do the unstick pulses with the wheels centered to reduce scrub.
    _drive_ackermann(car, 0, 0.0, forward=True)
    time.sleep(0.05)
    print(f"  🔧  Wiggling fwd/rev {WIGGLE_COUNT}× at PWM={fwd_speed}% "
          "to break static friction…", end='', flush=True)
    for _ in range(WIGGLE_COUNT):
        # Forward pulse — call set_speed every 20 ms so the PWM ramp reaches target
        deadline = time.monotonic() + WIGGLE_PULSE_S
        while time.monotonic() < deadline:
            car.set_speed(fwd_speed)
            time.sleep(0.02)
        car.stop()
        time.sleep(MOTOR_SAFETY_DWELL)
        # Reverse pulse
        deadline = time.monotonic() + WIGGLE_PULSE_S
        while time.monotonic() < deadline:
            car.reverse(fwd_speed)
            time.sleep(0.02)
        car.stop()
        time.sleep(MOTOR_SAFETY_DWELL)
    print("  done", flush=True)


def _confirm_motion(car: CarSystem, pico: PicoSensorReader,
                    direction: str, speed: int,
                    steer_angle_deg: float = CAL_STEER_ANGLE_DEG,
                    exit_on_failure: bool = True) -> int | None:
    """
    Drive the car in a tight Ackermann arc at *speed* in *direction*
    and wait up to MOTION_CONFIRM_S
    to see movement via gyro_z or wheel encoders.

    If not moving, bumps PWM by SPIN_SPEED_STEP and retries up to
    MOTION_MAX_ATTEMPTS times.

    Returns the confirmed speed. On failure, either returns None or exits.
    """
    print()
    print("  🔍  Confirming the rover is actually moving before starting the timer…")
    steer_angle_deg = max(TURN_STEER_MIN_DEG, min(TURN_STEER_MAX_DEG, abs(steer_angle_deg)))

    for attempt in range(MOTION_MAX_ATTEMPTS):
        if speed > SPIN_SPEED_MAX:
            print()
            print("  ❌  Car did not move even at max PWM.")
            print("      Check: motor connections, battery charge, surface grip.")
            car.stop()
            if exit_on_failure:
                sys.exit(1)
            return None

        # Rock the car to break static friction, then immediately start turning
        _break_static_friction(car, speed)
        car.stop()
        time.sleep(MOTOR_SAFETY_DWELL)

        # Observation window: keep sending front-steer + rear-drive arc commands
        # so the Pico PWM ramp reaches the requested speed.
        deadline = time.monotonic() + MOTION_CONFIRM_S
        confirmed = False
        while time.monotonic() < deadline:
            _drive_turn_arc(car, direction, speed, steer_angle_deg)
            pkt = pico.get_latest()
            _sync_power_state(car, pkt)
            if _is_moving(pkt):
                confirmed = True
                snap = _packet_motion_snapshot(pkt)
                print(f"  ✅  Motion confirmed at PWM={speed}%  "
                      f"(steer={steer_angle_deg:.1f}°  gyro_z={snap['gyro_z']:.1f}°/s  "
                      f"RPM L={snap['rpm_left']:.1f} R={snap['rpm_right']:.1f}  "
                      f"duty L/R={snap['duty_left']:.0f}/{snap['duty_right']:.0f}%)")
                break
            time.sleep(0.02)

        if confirmed:
            return speed

        # Still not moving — report and bump
        pkt = pico.get_latest()
        _sync_power_state(car, pkt)
        print(f"  ⚠️   Not moving at PWM={speed}%  "
              f"(steer={steer_angle_deg:.1f}°  {_motion_diag_string(pkt)}) "
              f"— bumping to {speed + SPIN_SPEED_STEP}%")
        speed = min(speed + SPIN_SPEED_STEP, SPIN_SPEED_MAX)

    print()
    print("  ❌  Motion could not be confirmed after all attempts.")
    car.stop()
    if exit_on_failure:
        sys.exit(1)
    return None


# ── calibration phase ─────────────────────────────────────────────────────────

def run_calibration(pico: PicoSensorReader, car: CarSystem) -> CompassManager:
    """
    Drive alternating Ackermann circles for SPIN_TIME seconds and collect
    magnetometer samples.
    Returns an initialized CompassManager instance.
    """
    print()
    print("=" * 60)
    print("  PHASE 1 — AUTO HARD/SOFT IRON CALIBRATION")
    print("=" * 60)
    print("  The rover will now drive tight left/right circles for "
          f"{SPIN_TIME:.0f} seconds.")
    print("  Give it clear floor space to roll forward while turning,")
    print("  and keep it away from metal objects and power bricks.")
    print()
    input("  Press ENTER when ready to start the circle run… ")
    print()

    # ── Step 1: confirm motion before starting the timed window ──────────────
    direction = "left"   # always start with a left-hand circle
    speed = _confirm_motion(car, pico, direction, SPIN_SPEED_START)

    # ── Step 2: timed calibration circle run ──────────────────────────────────
    print()
    print(f"  🔄  Car is moving — starting {SPIN_TIME:.0f}s calibration!")

    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')
    sample_count = 0

    # Spin state
    dir_start  = time.monotonic()

    # Stall-detection state
    last_stall_check  = time.monotonic()
    last_total_span   = 0.0      # sum of X + Y span at last check

    start_time = time.monotonic()
    last_print = start_time

    try:
        while time.monotonic() - start_time < SPIN_TIME:
            now = time.monotonic()

            # ── direction alternation (left → right → left …) ──────────────
            if now - dir_start >= SPIN_DIR_INTERVAL:
                car.stop()
                time.sleep(MOTOR_SAFETY_DWELL)
                direction = "right" if direction == "left" else "left"
                dir_start = now

            # ── keep sending front-steer + rear-drive arc commands ─────────
            _drive_turn_arc(car, direction, speed, CAL_STEER_ANGLE_DEG)

            # ── collect magnetometer sample ────────────────────────────────
            pkt = pico.get_latest()
            _sync_power_state(car, pkt)
            if pkt:
                if pkt.mag_x < min_x: min_x = pkt.mag_x
                if pkt.mag_x > max_x: max_x = pkt.mag_x
                if pkt.mag_y < min_y: min_y = pkt.mag_y
                if pkt.mag_y > max_y: max_y = pkt.mag_y
                sample_count += 1

            # ── stall detection: bump PWM if span not growing ──────────────
            if now - last_stall_check >= SPIN_STALL_WINDOW:
                span_x = (max_x - min_x) if max_x != float('-inf') else 0.0
                span_y = (max_y - min_y) if max_y != float('-inf') else 0.0
                total_span = span_x + span_y
                moving = _is_moving(pkt)
                if not moving or total_span - last_total_span < SPIN_STALL_THRESH:
                    if speed < SPIN_SPEED_MAX:
                        speed = min(speed + SPIN_SPEED_STEP, SPIN_SPEED_MAX)
                        # Hard-reset applied duty so the ramp restarts from
                        # the new target rather than the old one
                        car.stop()
                        time.sleep(MOTOR_SAFETY_DWELL)
                        print(f"\n  ⚡ Stall — PWM → {speed}%",
                              end='', flush=True)
                last_total_span  = total_span
                last_stall_check = now

            # ── progress bar ───────────────────────────────────────────────
            if now - last_print >= 0.25:
                elapsed   = now - start_time
                progress  = int((elapsed / SPIN_TIME) * 30)
                bar       = '█' * progress + '░' * (30 - progress)
                remaining = SPIN_TIME - elapsed
                dir_sym   = '↺' if direction == 'left' else '↻'
                print(f"\r  [{bar}] {remaining:4.1f}s  "
                      f"{dir_sym} {speed:>2}%  "
                      f"X:[{min_x:6.0f}…{max_x:6.0f}]  "
                      f"Y:[{min_y:6.0f}…{max_y:6.0f}]  "
                      f"n={sample_count}",
                      end='', flush=True)
                last_print = now

            time.sleep(1.0 / SAMPLE_HZ)   # 20 ms — matches PWM ramp tick rate
    finally:
        car.stop()

    print("\n")

    if sample_count < 10 or min_x == float('inf') or min_y == float('inf'):
        print("❌  Insufficient magnetometer data — check UART connection.")
        sys.exit(1)

    # ── compute corrections ────────────────────────────────────────────────────
    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0

    span_x = (max_x - min_x) / 2.0
    span_y = (max_y - min_y) / 2.0
    avg_span = (span_x + span_y) / 2.0

    # Dead Sensor Detection
    if span_x < 1e-6 or span_y < 1e-6:
        print("❌  CRITICAL: Magnetometer span is zero. Dead sensor or frozen I2C bus detected.")
        sys.exit(1)

    scale_x = (avg_span / span_x)
    scale_y = (avg_span / span_y)

    # ── report ─────────────────────────────────────────────────────────────────
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print(f"  Samples collected : {sample_count}")
    print(f"  X range           : {min_x:>8.1f}  →  {max_x:>8.1f}")
    print(f"  Y range           : {min_y:>8.1f}  →  {max_y:>8.1f}")
    print(f"{'─'*60}")
    print(f"  Hard-iron offsets (circle centre):")
    print(f"    MAG_OFFSET_X = {offset_x:>8.2f}")
    print(f"    MAG_OFFSET_Y = {offset_y:>8.2f}")
    print(f"  Soft-iron scales  (ellipse correction):")
    print(f"    MAG_SCALE_X  = {scale_x:>8.4f}")
    print(f"    MAG_SCALE_Y  = {scale_y:>8.4f}")
    print("=" * 60)

    # Sanity check — very wide or very narrow ellipse suggests interference
    aspect = max(span_x, span_y) / max(min(span_x, span_y), 1e-6)
    if aspect > 3.0:
        print(f"  ⚠️   High ellipse aspect ratio ({aspect:.2f}x) — consider")
        print("       re-running away from metal objects or power cables.")
    else:
        print(f"  ✅  Ellipse aspect ratio: {aspect:.2f}x (looks good)")

    return CompassManager(offset_x=offset_x, offset_y=offset_y, scale_x=scale_x, scale_y=scale_y)


# ── heading-read helper ────────────────────────────────────────────────────────

def heading_from_packet(pkt, compass: CompassManager) -> float | None:
    """Return a corrected heading from a packet, or None when unavailable."""
    if pkt is None:
        return None
    return compass.compute_heading(pkt.mag_x, pkt.mag_y)


def read_heading(pico: PicoSensorReader, compass: CompassManager) -> float | None:
    """Return a single corrected heading or None if no data."""
    return heading_from_packet(pico.get_latest(), compass)


def averaged_heading(pico: PicoSensorReader, compass: CompassManager, n: int = VERIFY_SAMPLES) -> float | None:
    """Collect *n* readings and return their circular mean."""
    readings = []
    deadline = time.monotonic() + n * (1.0 / SAMPLE_HZ) * 4 + 1.0
    while len(readings) < n and time.monotonic() < deadline:
        h = read_heading(pico, compass)
        if h is not None:
            readings.append(h)
        time.sleep(1.0 / SAMPLE_HZ)
    return circular_mean(readings) if readings else None


# ── auto-face helper ──────────────────────────────────────────────────────────

def face_heading(car: CarSystem, pico: PicoSensorReader, target_deg: float,
                 compass: CompassManager, label: str) -> bool:
    """
    Turn the Ackermann rover to *target_deg* using continuous Ackermann arcs.
    Drives immediately and adjusts steering every tick based on heading error.
    If the rover stalls, PWM is bumped inline without stopping or wiggling.
    """
    print(f"\n  ➤  Turning to {label} ({target_deg:.0f}°)…", flush=True)

    heading = _heading_snapshot(pico, compass, n=4)
    if heading is None:
        print("  ❌  No sensor data available for heading alignment.")
        return False

    smoothed_err = angular_error(heading, target_deg)
    if abs(smoothed_err) <= FACE_TOLERANCE_DEG:
        print(f"  ✅  Already within {FACE_TOLERANCE_DEG:.0f}° of {label}.")
        return True

    # Start driving immediately — no pre-confirmation or wiggle cycle
    speed = FACE_SPEED_FAST
    stall_accum = 0.0          # seconds of no detected motion
    deadline = time.monotonic() + FACE_TIMEOUT_S

    while time.monotonic() < deadline:
        pkt = pico.get_latest()
        _sync_power_state(car, pkt)

        raw_heading = heading_from_packet(pkt, compass)
        if raw_heading is not None:
            raw_err = angular_error(raw_heading, target_deg)
            smoothed_err = (smoothed_err * 0.7) + (raw_err * 0.3)
            heading = raw_heading

        # 1. Check if we've reached the target tolerance
        if abs(smoothed_err) <= FACE_TOLERANCE_DEG:
            car.stop()
            print(f"\r      current={heading:6.1f}°  error={smoothed_err:+6.1f}°   ", end='', flush=True)
            print(f"\n  ✅  Target reached. Settling…", end='', flush=True)
            time.sleep(SETTLE_S)
            return True

        # 2. Continuously recompute steering from live heading error
        direction = _alignment_direction(smoothed_err)
        steer_abs = _steer_for_heading_error(smoothed_err)
        steer_cmd = _signed_steer_for_segment(direction, steer_abs, forward=True)

        # 3. Inline stall handling — bump PWM without stopping
        if _is_moving(pkt):
            stall_accum = 0.0
        else:
            stall_accum += 0.02
            if stall_accum >= SPIN_STALL_WINDOW:
                if speed >= SPIN_SPEED_MAX:
                    car.stop()
                    fail_pkt = pico.get_latest()
                    _sync_power_state(car, fail_pkt)
                    print(f"\n  ❌  Not moving at max PWM={SPIN_SPEED_MAX}%.")
                    print(f"      telemetry: {_motion_diag_string(fail_pkt)}")
                    return False
                speed = min(speed + SPIN_SPEED_STEP, SPIN_SPEED_MAX)
                stall_accum = 0.0
                print(f"\n  ⚡ PWM → {speed}%", end='', flush=True)

        # 4. Dynamic speed: slow down near target, but never below FACE_SPEED_SLOW
        if abs(smoothed_err) < FACE_SLOWDOWN_DEG:
            current_speed = max(FACE_SPEED_SLOW, speed - 10)
        else:
            current_speed = speed

        # 5. Drive continuously — steering + rear wheels every tick
        _drive_ackermann(car, current_speed, steer_cmd, forward=True)

        print(
            f"\r      current={heading:6.1f}°  error={smoothed_err:+6.1f}°"
            f"  pwm={current_speed:>2.0f}%  steer={steer_cmd:+5.1f}°   ",
            end='',
            flush=True,
        )

        time.sleep(0.02)  # ~50Hz control loop

    car.stop()
    print(f"\n  ⚠️   Timed out aligning to {label}")
    return False

    
# ── interactive heading mode ─────────────────────────────────────────────────

def run_interactive_mode(pico: PicoSensorReader, car: CarSystem, compass: CompassManager):
    """
    Let the user type any heading (0-359) and the car turns to face it.
    Type 'q' or press Ctrl-C to exit.
    """
    print()
    print("=" * 60)
    print("  PHASE 2 — INTERACTIVE HEADING MODE")
    print("=" * 60)
    print("  Enter a heading (0–359°) and the rover will turn to face it.")
    print("  Leave room for it to roll forward while trimming the heading.")
    print("  Type 'q' or press Ctrl-C to quit.")
    print()

    while True:
        # Show live heading while waiting for input
        h = averaged_heading(pico, compass, n=10)
        h_str = f"{h:.1f}°" if h is not None else "N/A"
        try:
            raw = input(f"  Current heading: {h_str:>7}  —  Enter target (0-359) or 'q': ").strip()
        except EOFError:
            break

        if raw.lower() in ('q', 'quit', 'exit', ''):
            print("  🛑  Exiting interactive mode.")
            break

        try:
            target = float(raw)
        except ValueError:
            print(f"  ⚠️   '{raw}' is not a valid number — try again.")
            continue

        if not (0.0 <= target < 360.0):
            print("  ⚠️   Please enter a value between 0 and 359.")
            continue

        label = f"{target:.1f}°"
        face_heading(car, pico, target, compass, label)

        measured = averaged_heading(pico, compass)
        if measured is None:
            print("  ❌  No sensor data — check UART connection.")
            continue

        err = angular_error(measured, target)
        sign = '+' if err >= 0 else ''
        status = "✅" if abs(err) <= FACE_TOLERANCE_DEG else (
                 "⚠️ " if abs(err) <= 15.0 else "❌")
        print(f"  {status} target={target:.1f}°  "
              f"measured={measured:.1f}°  error={sign}{err:.1f}°")
        print()


# ── cardinal test phase ───────────────────────────────────────────────────────

CARDINALS = [
    ("N", 0.0),
    ("E", 90.0),
    ("S", 180.0),
    ("W", 270.0),
]


def run_cardinal_test(pico: PicoSensorReader, car: CarSystem, compass: CompassManager):
    """Face the car to each cardinal point and report heading errors."""
    print()
    print("=" * 60)
    print("  PHASE 2 — AUTO CARDINAL DIRECTION TEST")
    print("=" * 60)
    print("  The rover will drive tight Ackermann arcs to face")
    print("  N → E → S → W automatically.")
    print()
    input("  Press ENTER to begin the cardinal test… ")

    results = {}

    for label, target in CARDINALS:
        face_heading(car, pico, target, compass, label)

        # Take an averaged reading with the car stationary
        measured = averaged_heading(pico, compass)
        if measured is None:
            print(f"  ❌  No sensor data for {label} — skipping.")
            results[label] = (target, None, None)
            continue

        err = angular_error(measured, target)
        results[label] = (target, measured, err)

        status = "✅" if abs(err) <= FACE_TOLERANCE_DEG else (
                 "⚠️ " if abs(err) <= 15.0 else "❌")
        print(f"  {status} {label}: target={target:.0f}°  "
              f"measured={measured:.1f}°  error={err:+.1f}°")

        # Small pause between cardinals so the car isn't lurching
        time.sleep(0.5)

    # ── final report ──────────────────────────────────────────────────────────
    car.stop()
    print()
    print("=" * 60)
    print("  RESULTS SUMMARY")
    print("=" * 60)
    print(f"  {'Dir':<4} {'Target':>7}  {'Measured':>9}  {'Error':>7}")
    print(f"  {'─'*4} {'─'*7}  {'─'*9}  {'─'*7}")

    errors = []
    for label, (target, measured, err) in results.items():
        if measured is None:
            print(f"  {label:<4} {target:>6.1f}°  {'N/A':>9}  {'N/A':>7}")
        else:
            sign = '+' if err >= 0 else ''
            print(f"  {label:<4} {target:>6.1f}°  {measured:>8.1f}°  {sign}{err:.1f}°")
            errors.append(abs(err))

    if errors:
        mean_err = sum(errors) / len(errors)
        max_err  = max(errors)
        print(f"{'─'*60}")
        print(f"  Mean absolute error : {mean_err:.1f}°")
        print(f"  Max  absolute error : {max_err:.1f}°")
        print()

        if max_err <= 5.0:
            quality = "✅  EXCELLENT"
        elif max_err <= 15.0:
            quality = "⚠️   ACCEPTABLE"
        else:
            quality = "❌  POOR — re-run away from metal/interference"

        print(f"  Calibration quality : {quality}")
    print("=" * 60)


# ── entry point ───────────────────────────────────────────────────────────────

def _pick_test_mode() -> str:
    """
    Ask the user which Phase-2 test to run.
    Returns 'cardinal', 'interactive', or 'both'.
    """
    print()
    print("  ┌────────────────────────────────────────────────────────┐")
    print("  │  PHASE 2 — HEADING TEST MODE                           │")
    print("  ├────────────────────────────────────────────────────────┤")
    print("  │  [1] Auto cardinal  — N → E → S → W, error report   │")
    print("  │  [2] Interactive    — type any heading (0–359°)       │")
    print("  │  [3] Both           — cardinal first, then interactive │")
    print("  └────────────────────────────────────────────────────────┘")
    while True:
        choice = input("  Choice [1/2/3]: ").strip()
        if choice == '1': return 'cardinal'
        if choice == '2': return 'interactive'
        if choice == '3': return 'both'
        print("  Please enter 1, 2, or 3.")


def main():
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║    COMPASS AUTO-CALIBRATE & HEADING TEST                 ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║  Phase 1: Full-lock circles collect hard/soft-iron data ║")
    print("║  Phase 2: Auto N/E/S/W  OR  interactive 0–359° target   ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print("  ⚠️   Place the car on a flat, open surface with")
    print("       enough room to roll forward while turning.")
    print("       Keep at least 50 cm from metal objects,")
    print("       power cables, and the laptop/Pi power brick.")
    print()

    print("🔌  Connecting to shared Pico sensor bridge…")
    pico = init_pico_reader('/dev/ttyS0')
    time.sleep(1.2)

    print("⏳  Waiting for first sensor packet…")
    for _ in range(80):
        if pico.is_fresh():
            break
        time.sleep(0.1)
    else:
        print("⚠️   Still no data — continuing anyway (check UART).")

    print("🔧  Initialising motors…")
    car = CarSystem()
    time.sleep(0.3)

    compass = None

    try:
        # ── Phase 1: calibration (load or run) ───────────────────────────────
        saved_compass = CompassManager.load(CAL_FILE)
        if saved_compass is not None:
            print()
            print("  📂  Previous calibration found:")
            print(f"    MAG_OFFSET_X = {saved_compass.offset_x:.2f}")
            print(f"    MAG_OFFSET_Y = {saved_compass.offset_y:.2f}")
            print(f"    MAG_SCALE_X  = {saved_compass.scale_x:.4f}")
            print(f"    MAG_SCALE_Y  = {saved_compass.scale_y:.4f}")
            print()
            recal = input("  Run a fresh calibration? [y/N]: ").strip().lower()
            if recal in ('y', 'yes'):
                compass = run_calibration(pico, car)
                compass.save(CAL_FILE)
            else:
                compass = saved_compass
        else:
            print()
            print("  ℹ️   No saved calibration found — running calibration now.")
            compass = run_calibration(pico, car)
            compass.save(CAL_FILE)

        print()
        print("  Active calibration values:")
        print(f"    MAG_OFFSET_X = {compass.offset_x:.2f}")
        print(f"    MAG_OFFSET_Y = {compass.offset_y:.2f}")
        print(f"    MAG_SCALE_X  = {compass.scale_x:.4f}")
        print(f"    MAG_SCALE_Y  = {compass.scale_y:.4f}")

        # ── Phase 2: heading test ─────────────────────────────────────────────────
        mode = _pick_test_mode()
        if mode in ('cardinal', 'both'):
            run_cardinal_test(pico, car, compass)
        if mode in ('interactive', 'both'):
            run_interactive_mode(pico, car, compass)

    except KeyboardInterrupt:
        print("\n\n  ⛔  Interrupted by user.")
    finally:
        car.stop()
        car.cleanup()
        pico.close()
        print("\n  🔌  Motors and sensor reader shut down. Goodbye.\n")


if __name__ == "__main__":
    main()
