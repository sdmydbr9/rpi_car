#!/usr/bin/env python3
"""
compass_calibrate_and_test.py — Full Auto Compass Calibration + Cardinal Test
==============================================================================
Phase 1 — AUTO CALIBRATION:
    Places the car on open ground, spins it via motors for 20 seconds,
    and fits hard-iron (offset) and soft-iron (scale) corrections from
    the min/max envelope of the raw magnetometer readings.

Phase 2 — AUTO CARDINAL TEST:
    Uses the computed corrections to drive the car to each of N, E, S, W
    via a closed-loop pivot turn, then measures the heading error.
"""

import os
import sys
import json
import time
import math

# ── path setup ────────────────────────────────────────────────────────────────
_DIAG_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_DIAG_DIR, '..', 'core'))

try:
    from pico_sensor_reader import PicoSensorReader
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


def _save_calibration(offset_x, offset_y, scale_x, scale_y):
    data = {
        'offset_x': offset_x,
        'offset_y': offset_y,
        'scale_x':  scale_x,
        'scale_y':  scale_y,
    }
    with open(CAL_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  💾  Calibration saved to {CAL_FILE}")


def _load_calibration():
    """Return (offset_x, offset_y, scale_x, scale_y) or None if not found."""
    if not os.path.exists(CAL_FILE):
        return None
    try:
        with open(CAL_FILE) as f:
            d = json.load(f)
        return d['offset_x'], d['offset_y'], d['scale_x'], d['scale_y']
    except Exception as e:
        print(f"  ⚠️   Could not read calibration file: {e}")
        return None


# ── constants ─────────────────────────────────────────────────────────────────
SPIN_TIME          = 20.0   # seconds of motor-driven spin for calibration
SPIN_SPEED_START   = 35     # PWM % — initial slow spin
SPIN_SPEED_MAX     = 70     # PWM % — ceiling before giving up
SPIN_SPEED_STEP    = 5      # PWM bump each time a stall is detected
SPIN_STALL_WINDOW  = 1.2    # seconds of span-freeze before declaring a stall
SPIN_STALL_THRESH  = 2.0    # min total span growth (raw counts) to count as moving
SPIN_DIR_INTERVAL  = 5.0    # seconds per direction (left → right → left …)
SAMPLE_HZ          = 50     # magnetometer poll rate (Hz)
MOTOR_SAFETY_DWELL = 0.15   # H-bridge direction-change dwell (matches CarSystem)

# Motion-confirmation thresholds (used before the 20 s timer starts)
MOTION_GYRO_THRESH  = 5.0   # deg/s on gyro_z — car is definitely rotating
MOTION_RPM_THRESH   = 2.0   # RPM on any wheel encoder — wheels are turning
MOTION_CONFIRM_S    = 2.5   # seconds to observe movement at each PWM level
MOTION_MAX_ATTEMPTS = 8     # max PWM bumps before aborting

# Forward/backward wiggle — breaks static friction before pivot attempts
WIGGLE_SPEED   = 55    # PWM % for the unstick pulses
WIGGLE_PULSE_S = 0.35  # duration of each forward / reverse pulse
WIGGLE_COUNT   = 3     # number of fwd+rev cycles per wiggle

# Cardinal facing parameters
FACE_TOLERANCE_DEG = 5.0    # stop turning when within ±5° of target
FACE_SPEED         = 35     # PWM % for alignment spin
FACE_TIMEOUT_S     = 8.0    # max seconds to find a heading before giving up
SETTLE_S           = 0.4    # seconds to let car settle before reading final heading
VERIFY_SAMPLES     = 25     # readings averaged for final heading measurement


# ── heading math helpers ───────────────────────────────────────────────────────

def compute_heading(mag_x: float, mag_y: float,
                    offset_x: float, offset_y: float,
                    scale_x: float, scale_y: float) -> float:
    """Return a corrected CW bearing (0-360°) from raw magnetometer data."""
    cx = (mag_x - offset_x) * scale_x
    cy = (mag_y - offset_y) * scale_y
    # Negative sign corrects for physical board inversion.
    # +90° corrects for the sensor being mounted 90° CCW on the board.
    return (-math.degrees(math.atan2(cy, cx)) + 90.0) % 360.0


def circular_mean(angles: list) -> float:
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return (math.degrees(math.atan2(s, c)) + 360.0) % 360.0


def angular_error(measured: float, expected: float) -> float:
    """Signed shortest-path angle difference (−180 ↔ +180)."""
    diff = measured - expected
    while diff >  180.0: diff -= 360.0
    while diff < -180.0: diff += 360.0
    return diff


# ── motion detection helper ──────────────────────────────────────────────────

def _is_moving(pkt) -> bool:
    """Return True if the sensor packet indicates the car is rotating."""
    if pkt is None:
        return False
    # Gyro yaw-rate check (MPU6500 gyro_z, deg/s)
    if abs(pkt.gyro_z) >= MOTION_GYRO_THRESH:
        return True
    # Wheel encoder check (any wheel above threshold)
    if (pkt.rpm_rear_right >= MOTION_RPM_THRESH or
            pkt.rpm_rear_left  >= MOTION_RPM_THRESH or
            pkt.rpm_front_right >= MOTION_RPM_THRESH):
        return True
    return False


def _break_static_friction(car: CarSystem, speed: int):
    """
    Rock the car forward then backward WIGGLE_COUNT times to break
    static friction before a pivot-turn attempt.
    Uses the same *speed* as the upcoming spin so the motors are warm.
    """
    fwd_speed = min(speed + 10, 80)   # slightly higher pulse for unstick
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
                    direction: str, speed: int) -> int:
    """
    Spin the car at *speed* in *direction* and wait up to MOTION_CONFIRM_S
    to see movement via gyro_z or wheel encoders.

    If not moving, bumps PWM by SPIN_SPEED_STEP and retries up to
    MOTION_MAX_ATTEMPTS times.

    Returns the confirmed speed, or calls sys.exit on failure.
    """
    print()
    print("  🔍  Confirming the car is moving before starting the timer…")

    for attempt in range(MOTION_MAX_ATTEMPTS):
        if speed > SPIN_SPEED_MAX:
            print()
            print("  ❌  Car did not move even at max PWM.")
            print("      Check: motor connections, battery charge, surface grip.")
            car.stop()
            sys.exit(1)

        # Rock the car to break static friction, then immediately pivot
        _break_static_friction(car, speed)
        car.stop()
        time.sleep(MOTOR_SAFETY_DWELL)

        # Observation window: call pivot_turn every 20 ms so the PWM
        # ramp (5%/tick) actually reaches the target speed.
        deadline = time.monotonic() + MOTION_CONFIRM_S
        confirmed = False
        while time.monotonic() < deadline:
            car.pivot_turn(direction, speed)   # keeps ramp climbing each tick
            pkt = pico.get_latest()
            if _is_moving(pkt):
                confirmed = True
                gyro = abs(pkt.gyro_z)
                rpm_r = pkt.rpm_rear_right
                rpm_l = pkt.rpm_rear_left
                print(f"  ✅  Motion confirmed at PWM={speed}%  "
                      f"(gyro_z={gyro:.1f}°/s  "
                      f"RPM L={rpm_l:.1f} R={rpm_r:.1f})")
                break
            time.sleep(0.02)

        if confirmed:
            return speed

        # Still not moving — report and bump
        pkt = pico.get_latest()
        gyro = abs(pkt.gyro_z) if pkt else 0.0
        rpm_max = max(
            (pkt.rpm_rear_right, pkt.rpm_rear_left, pkt.rpm_front_right)
            if pkt else (0.0, 0.0, 0.0)
        )
        print(f"  ⚠️   Not moving at PWM={speed}%  "
              f"(gyro_z={gyro:.1f}°/s  max_rpm={rpm_max:.1f}) "
              f"— bumping to {speed + SPIN_SPEED_STEP}%")
        speed = min(speed + SPIN_SPEED_STEP, SPIN_SPEED_MAX)

    print()
    print("  ❌  Motion could not be confirmed after all attempts.")
    car.stop()
    sys.exit(1)


# ── calibration phase ─────────────────────────────────────────────────────────

def run_calibration(pico: PicoSensorReader, car: CarSystem) -> tuple:
    """
    Spin the car in place for SPIN_TIME seconds and collect mag samples.
    Returns (offset_x, offset_y, scale_x, scale_y).
    """
    print()
    print("=" * 60)
    print("  PHASE 1 — AUTO HARD/SOFT IRON CALIBRATION")
    print("=" * 60)
    print("  The car will now spin in slow circles for "
          f"{SPIN_TIME:.0f} seconds.")
    print("  Keep it on a flat, open surface away from metal objects.")
    print()
    input("  Press ENTER when ready to start the spin… ")
    print()

    # ── Step 1: confirm motion before starting the timed window ──────────────
    direction = "left"   # always start spinning left
    speed = _confirm_motion(car, pico, direction, SPIN_SPEED_START)

    # ── Step 2: 20-second timed calibration spin ──────────────────────────────
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

            # ── keep calling pivot_turn every tick so the PWM ramp climbs ───
            car.pivot_turn(direction, speed)

            # ── collect magnetometer sample ────────────────────────────────
            pkt = pico.get_latest()
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
                moving = _is_moving(pico.get_latest())
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

    scale_x = (avg_span / span_x) if span_x > 0 else 1.0
    scale_y = (avg_span / span_y) if span_y > 0 else 1.0

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

    return offset_x, offset_y, scale_x, scale_y


# ── heading-read helper ────────────────────────────────────────────────────────

def read_heading(pico: PicoSensorReader,
                 offset_x: float, offset_y: float,
                 scale_x: float, scale_y: float) -> float | None:
    """Return a single corrected heading or None if no data."""
    pkt = pico.get_latest()
    if pkt:
        return compute_heading(pkt.mag_x, pkt.mag_y,
                               offset_x, offset_y, scale_x, scale_y)
    return None


def averaged_heading(pico: PicoSensorReader,
                     offset_x: float, offset_y: float,
                     scale_x: float, scale_y: float,
                     n: int = VERIFY_SAMPLES) -> float | None:
    """Collect *n* readings and return their circular mean."""
    readings = []
    deadline = time.monotonic() + n * (1.0 / SAMPLE_HZ) * 4 + 1.0
    while len(readings) < n and time.monotonic() < deadline:
        h = read_heading(pico, offset_x, offset_y, scale_x, scale_y)
        if h is not None:
            readings.append(h)
        time.sleep(1.0 / SAMPLE_HZ)
    return circular_mean(readings) if readings else None


# ── auto-face helper ──────────────────────────────────────────────────────────

def face_heading(car: CarSystem, pico: PicoSensorReader,
                 target_deg: float,
                 offset_x: float, offset_y: float,
                 scale_x: float, scale_y: float,
                 label: str) -> bool:
    """
    Spin the car until its heading is within FACE_TOLERANCE_DEG of target_deg.
    Returns True on success, False on timeout.

    Strategy:
      1. Read current heading.
      2. Compute signed error: negative → we're CCW of target → turn right;
         positive → CW of target → turn left.
      3. Stop, settle, re-read for confirmation.
    """
    print(f"\n  ➤  Aligning to {label} ({target_deg:.0f}°)…", flush=True)

    deadline = time.monotonic() + FACE_TIMEOUT_S
    direction = None

    while time.monotonic() < deadline:
        h = read_heading(pico, offset_x, offset_y, scale_x, scale_y)
        if h is None:
            time.sleep(0.02)
            continue

        err = angular_error(h, target_deg)
        print(f"\r     current={h:6.1f}°  error={err:+6.1f}°   ", end='', flush=True)

        if abs(err) <= FACE_TOLERANCE_DEG:
            car.stop()
            break

        # Choose spin direction: positive error means we are CW of target
        new_dir = "left" if err > 0 else "right"
        if new_dir != direction:
            car.stop()
            time.sleep(MOTOR_SAFETY_DWELL)
            direction = new_dir

        car.pivot_turn(direction, FACE_SPEED)
        time.sleep(0.02)
    else:
        # Timed out
        car.stop()
        print(f"\n  ⚠️   Timed out aligning to {label}")
        return False

    car.stop()
    print(f"\n  ⏳  Settling…", end='', flush=True)
    time.sleep(SETTLE_S)
    return True


# ── interactive heading mode ─────────────────────────────────────────────────

def run_interactive_mode(pico: PicoSensorReader, car: CarSystem,
                         offset_x: float, offset_y: float,
                         scale_x: float, scale_y: float):
    """
    Let the user type any heading (0-359) and the car turns to face it.
    Type 'q' or press Ctrl-C to exit.
    """
    print()
    print("=" * 60)
    print("  PHASE 2 — INTERACTIVE HEADING MODE")
    print("=" * 60)
    print("  Enter a heading (0–359°) and the car will turn to face it.")
    print("  Type 'q' or press Ctrl-C to quit.")
    print()

    while True:
        # Show live heading while waiting for input
        h = averaged_heading(pico, offset_x, offset_y, scale_x, scale_y, n=10)
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
        aligned = face_heading(car, pico, target,
                               offset_x, offset_y, scale_x, scale_y,
                               label)

        measured = averaged_heading(pico, offset_x, offset_y, scale_x, scale_y)
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


def run_cardinal_test(pico: PicoSensorReader, car: CarSystem,
                      offset_x: float, offset_y: float,
                      scale_x: float, scale_y: float):
    """Face the car to each cardinal point and report heading errors."""
    print()
    print("=" * 60)
    print("  PHASE 2 — AUTO CARDINAL DIRECTION TEST")
    print("=" * 60)
    print("  The car will spin to face N → E → S → W automatically.")
    print()
    input("  Press ENTER to begin the cardinal test… ")

    results = {}

    for label, target in CARDINALS:
        aligned = face_heading(car, pico, target,
                               offset_x, offset_y, scale_x, scale_y,
                               label)

        # Take an averaged reading with the car stationary
        measured = averaged_heading(pico, offset_x, offset_y, scale_x, scale_y)
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
    print("  │  PHASE 2 — HEADING TEST MODE                          │")
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
    print("║    COMPASS AUTO-CALIBRATE & HEADING TEST                ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║  Phase 1: Motors spin car to collect hard/soft-iron data ║")
    print("║  Phase 2: Auto N/E/S/W  OR  interactive 0–359° target   ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print("  ⚠️   Place the car on a flat, open surface.")
    print("       Keep at least 50 cm from metal objects,")
    print("       power cables, and the laptop/Pi power brick.")
    print()

    print("🔌  Connecting to Pico sensor reader…")
    pico = PicoSensorReader()
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

    try:
        # ── Phase 1: calibration (load or run) ───────────────────────────────
        saved = _load_calibration()
        if saved is not None:
            offset_x, offset_y, scale_x, scale_y = saved
            print()
            print("  📂  Previous calibration found:")
            print(f"    MAG_OFFSET_X = {offset_x:.2f}")
            print(f"    MAG_OFFSET_Y = {offset_y:.2f}")
            print(f"    MAG_SCALE_X  = {scale_x:.4f}")
            print(f"    MAG_SCALE_Y  = {scale_y:.4f}")
            print()
            recal = input("  Run a fresh calibration? [y/N]: ").strip().lower()
            if recal in ('y', 'yes'):
                offset_x, offset_y, scale_x, scale_y = run_calibration(pico, car)
                _save_calibration(offset_x, offset_y, scale_x, scale_y)
        else:
            print()
            print("  ℹ️   No saved calibration found — running calibration now.")
            offset_x, offset_y, scale_x, scale_y = run_calibration(pico, car)
            _save_calibration(offset_x, offset_y, scale_x, scale_y)

        print()
        print("  Active calibration values:")
        print(f"    MAG_OFFSET_X = {offset_x:.2f}")
        print(f"    MAG_OFFSET_Y = {offset_y:.2f}")
        print(f"    MAG_SCALE_X  = {scale_x:.4f}")
        print(f"    MAG_SCALE_Y  = {scale_y:.4f}")

        # ── Phase 2: heading test ─────────────────────────────────────────────────
        mode = _pick_test_mode()
        if mode in ('cardinal', 'both'):
            run_cardinal_test(pico, car, offset_x, offset_y, scale_x, scale_y)
        if mode in ('interactive', 'both'):
            run_interactive_mode(pico, car, offset_x, offset_y, scale_x, scale_y)

    except KeyboardInterrupt:
        print("\n\n  ⛔  Interrupted by user.")
    finally:
        car.stop()
        car.cleanup()
        pico.close()
        print("\n  🔌  Motors and sensor reader shut down. Goodbye.\n")


if __name__ == "__main__":
    main()
