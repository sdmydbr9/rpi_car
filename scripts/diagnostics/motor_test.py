#!/usr/bin/env python3
"""
Ncurses Keyboard motor tester with All-Wheel PI-Loop Synchronization.
Master: Rear-Left
Active Slaves: Rear-Right, Front-Right
Passive Slave (Mirrored): Front-Left
Includes Anti-Stall Deadbands for Mixed Metal/Plastic Gearboxes.
RPM-LOCK mode: interactive RPM target with back-calculation anti-windup PI.
"""

import os
import sys
import time
import curses
import datetime

# Add core/ to path for bare-name imports of core modules (motor, pico_sensor_reader, etc.)
_diag_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_diag_dir, '..', 'core'))

# Try real GPIO first.
try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        class _MockPWM:
            def __init__(self, pin, freq): self.pin, self.freq = pin, freq
            def start(self, value): pass
            def ChangeDutyCycle(self, value): pass
            def stop(self): pass
        def setmode(self, mode): pass
        def setwarnings(self, value): pass
        def setup(self, pins, mode): pass
        def output(self, pins, state): pass
        def cleanup(self): pass
        def PWM(self, pin, freq): return self._MockPWM(pin, freq)
    GPIO = MockGPIO()


# Try to import Pico sensor reader.
try:
    from pico_sensor_reader import init_pico_reader, get_rpm as pico_get_rpm, get_diagnostics as pico_get_diagnostics
    PICO_AVAILABLE = True
except ImportError:
    PICO_AVAILABLE = False
    def pico_get_diagnostics():
        return {'connected': False, 'fresh': False, 'frames': 0, 'errors': 0,
                'packets_received': 0, 'laser_mm': -1, 'age_s': -1}


# Dual L298N, 4 independent wheel channels (BCM numbering).
FL_IN1 = 17
FL_IN2 = 27
FL_ENA = 12

FR_IN3 = 23
FR_IN4 = 22
FR_ENB = 13

RL_IN1 = 9
RL_IN2 = 11
RL_ENA = 26

RR_IN3 = 10
RR_IN4 = 7
RR_ENB = 16

ALL_DIR_PINS = [FL_IN1, FL_IN2, FR_IN3, FR_IN4, RL_IN1, RL_IN2, RR_IN3, RR_IN4]
ALL_EN_PINS = [FL_ENA, FR_ENB, RL_ENA, RR_ENB]

PWM_FREQ = 1000  # Hz

BATTERY_VOLTAGE = 11.1
MOTOR_MAX_VOLTAGE = 7.0
MAX_PWM_DUTY = round((MOTOR_MAX_VOLTAGE / BATTERY_VOLTAGE) * 100)

DEFAULT_SPEED = min(30, MAX_PWM_DUTY)
SPEED_STEP = 5
MIN_SPEED = 5

WHEEL_MAP = {
    "fl": (FL_ENA, FL_IN1, FL_IN2, "Front-Left"),
    "fr": (FR_ENB, FR_IN3, FR_IN4, "Front-Right"),
    "rl": (RL_ENA, RL_IN1, RL_IN2, "Rear-Left"),
    "rr": (RR_ENB, RR_IN3, RR_IN4, "Rear-Right"),
}

COMMAND_MAP = {
    "w": ("fl", "forward"), "s": ("fl", "reverse"),
    "e": ("fr", "forward"), "d": ("fr", "reverse"),
    "r": ("rl", "forward"), "f": ("rl", "reverse"),
    "t": ("rr", "forward"), "g": ("rr", "reverse"),
    "y": ("rear_sync", "forward"), "h": ("rear_sync", "reverse"),
    "a": ("all_sync", "forward"),  "z": ("all_sync", "reverse"),
}

# RPM Target Mode Constants
RPM_TARGET_MIN = 100
RPM_TARGET_MAX = 1000
RPM_TARGET_STEP = 50
RPM_KP = 0.012   # Proportional gain (gentle — feedforward does the heavy lifting)
RPM_KI = 0.004   # Integral gain (trims residual steady-state error)
RPM_KD = 0.012   # Derivative gain (strong damping to kill overshoot oscillation)
RPM_KT = 0.5     # Back-calculation tracking gain (aggressive anti-windup)
RPM_FF_GAIN = 0.065  # Feedforward: PWM% ≈ RPM * FF_GAIN (empirical: 300RPM≈20%PWM)
RPM_FILTER_ALPHA = 0.4   # Low-pass EMA for RPM readings (balance noise vs lag)
RPM_SLEW_UP = 2.0        # Max PWM increase per tick (%/tick, 40%/s — controlled ramp)
RPM_SLEW_DOWN = 4.0      # Max PWM decrease per tick (%/tick, 80%/s — fast over-speed correction)


def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ALL_DIR_PINS + ALL_EN_PINS, GPIO.OUT)
    GPIO.output(ALL_DIR_PINS, False)
    GPIO.output(ALL_EN_PINS, False)

    pwms = {}
    for wheel_id, (pwm_pin, _, _, _) in WHEEL_MAP.items():
        pwm = GPIO.PWM(pwm_pin, PWM_FREQ)
        pwm.start(0)
        pwms[wheel_id] = pwm
    return pwms


def stop_all(pwms):
    for pwm in pwms.values():
        pwm.ChangeDutyCycle(0)
    GPIO.output(ALL_DIR_PINS, False)


def drive_one_wheel(pwms, wheel_id, direction, speed):
    stop_all(pwms)
    _, in_a, in_b, _ = WHEEL_MAP[wheel_id]
    pwm = pwms[wheel_id]

    GPIO.output([in_a, in_b], 0)
    if direction == "forward":
        GPIO.output(in_a, 1)
        GPIO.output(in_b, 0)
    else:
        GPIO.output(in_a, 0)
        GPIO.output(in_b, 1)

    pwm.ChangeDutyCycle(speed)


def drive_rear_sync(pwms, direction, rl_pwm_val, rr_pwm_val):
    pwms["fl"].ChangeDutyCycle(0)
    pwms["fr"].ChangeDutyCycle(0)
    GPIO.output([FL_IN1, FL_IN2, FR_IN3, FR_IN4], 0)

    GPIO.output([RL_IN1, RL_IN2, RR_IN3, RR_IN4], 0)
    if direction == "forward":
        GPIO.output([RL_IN1, RR_IN3], 1)
    else:
        GPIO.output([RL_IN2, RR_IN4], 1)

    pwms["rl"].ChangeDutyCycle(rl_pwm_val)
    pwms["rr"].ChangeDutyCycle(rr_pwm_val)


def drive_all_sync(pwms, direction, rl_pwm, rr_pwm, fl_pwm, fr_pwm):
    """Drives all 4 wheels with independent PWMs for syncing."""
    GPIO.output(ALL_DIR_PINS, 0)
    if direction == "forward":
        GPIO.output([FL_IN1, FR_IN3, RL_IN1, RR_IN3], 1)
    else:
        GPIO.output([FL_IN2, FR_IN4, RL_IN2, RR_IN4], 1)

    pwms["rl"].ChangeDutyCycle(rl_pwm)
    pwms["rr"].ChangeDutyCycle(rr_pwm)
    pwms["fl"].ChangeDutyCycle(fl_pwm)
    pwms["fr"].ChangeDutyCycle(fr_pwm)


def draw_ui(stdscr, speed, status_msg, rpm_data, rr_adj, fr_adj, is_logging, min_rear_pwm, min_front_pwm,
            rpm_target=None, entering_rpm=False, rpm_input_buf="", rpm_pwm_vals=None, diag=None):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    def safe_addstr(row, col, text, attr=curses.A_NORMAL):
        if 0 <= row < max_y - 1:
            stdscr.addnstr(row, col, text, max_x - col - 1, attr)

    safe_addstr(0, 0, "🏎️  Independent Motor Test (4WD Anti-Stall Sync)", curses.A_BOLD)
    safe_addstr(1, 0, "-" * 60)

    safe_addstr(2, 0, "W/S: Front-Left   |  E/D: Front-Right")
    safe_addstr(3, 0, "R/F: Rear-Left    |  T/G: Rear-Right")
    safe_addstr(4, 0, "Y/H: Rear-Sync Drive  (Fwd/Rev)")
    safe_addstr(5, 0, "A/Z: ALL-SYNC 4WD     (Fwd/Rev)")
    safe_addstr(6, 0, f"I: Set target RPM     ({RPM_TARGET_MIN}-{RPM_TARGET_MAX})")
    safe_addstr(7, 0, "Space / X: Stop all motors")
    safe_addstr(8, 0, "+ / -: Change speed / RPM target")
    safe_addstr(9, 0, "Q: Quit")

    safe_addstr(11, 0, "-" * 60)

    # RPM Input prompt overlay
    if entering_rpm:
        safe_addstr(12, 0, "ENTER TARGET RPM:", curses.A_BOLD)
        safe_addstr(13, 0, f">>> {rpm_input_buf}_ ")
        safe_addstr(14, 0, f"Range: {RPM_TARGET_MIN} - {RPM_TARGET_MAX} RPM  |  Enter=Confirm  Esc=Cancel")
        safe_addstr(16, 0, "-" * 60)
        safe_addstr(17, 0, "LIVE RPM SENSORS:", curses.A_BOLD)
        if not PICO_AVAILABLE:
            safe_addstr(18, 0, "⚠️  pico_sensor_reader not available")
        else:
            rr, rl, fr = rpm_data
            safe_addstr(19, 2, f"Front-Right (FR) : {fr:6.1f} RPM")
            safe_addstr(20, 2, f"Rear-Left   (RL) : {rl:6.1f} RPM")
            safe_addstr(21, 2, f"Rear-Right  (RR) : {rr:6.1f} RPM")
        stdscr.refresh()
        return

    safe_addstr(12, 0, "MOTOR STATUS:", curses.A_BOLD)
    safe_addstr(13, 0, status_msg)

    if rpm_target is not None and "RPM-LOCK" in status_msg:
        safe_addstr(14, 0, f"Target RPM: {rpm_target}  (+/- to adjust by {RPM_TARGET_STEP})")
        if rpm_pwm_vals:
            safe_addstr(15, 0, f"PWM -> RL:{rpm_pwm_vals['rl']:5.1f}%  RR:{rpm_pwm_vals['rr']:5.1f}%  "
                                f"FL:{rpm_pwm_vals['fl']:5.1f}%  FR:{rpm_pwm_vals['fr']:5.1f}%", curses.A_DIM)
    else:
        safe_addstr(14, 0, f"Target Base Speed: {int(speed)}% (Max: {MAX_PWM_DUTY}%)")
        if "Sync" in status_msg:
            rr_power = max(min_rear_pwm, min(MAX_PWM_DUTY, speed + rr_adj))
            fr_power = max(min_front_pwm, min(MAX_PWM_DUTY, speed + fr_adj))
            safe_addstr(15, 0, f"RL: {int(speed)}% | RR: {int(rr_power)}% (Adj {rr_adj:+.1f}%)", curses.A_DIM)
            if "ALL-SYNC" in status_msg:
                safe_addstr(16, 0, f"FL: {int(fr_power)}% | FR: {int(fr_power)}% (Adj {fr_adj:+.1f}%)", curses.A_DIM)

    if is_logging:
        safe_addstr(17, 0, "REC: Saving data to CSV...", curses.color_pair(1) | curses.A_BLINK if curses.has_colors() else curses.A_BLINK)

    safe_addstr(19, 0, "-" * 60)
    safe_addstr(20, 0, "LIVE RPM SENSORS:", curses.A_BOLD)

    if not PICO_AVAILABLE:
        safe_addstr(21, 0, "⚠️  pico_sensor_reader not available", curses.color_pair(1) if curses.has_colors() else curses.A_NORMAL)
    else:
        rr, rl, fr = rpm_data
        rl_attr = rr_attr = fr_attr = curses.A_NORMAL

        if "RPM-LOCK" in status_msg and rpm_target:
            if abs(rpm_target - rr) <= 10.0: rr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            if abs(rpm_target - fr) <= 10.0: fr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            if abs(rpm_target - rl) <= 10.0: rl_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
        elif "Sync" in status_msg and rl > 0:
            if abs(rl - rr) <= 5.0: rr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            if abs(rl - fr) <= 5.0: fr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            rl_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD

        safe_addstr(22, 2, f"Front-Right (FR) : {fr:6.1f} RPM", fr_attr)
        safe_addstr(23, 2, f"Rear-Left   (RL) : {rl:6.1f} RPM", rl_attr)
        safe_addstr(24, 2, f"Rear-Right  (RR) : {rr:6.1f} RPM", rr_attr)

    # Pico Connection Diagnostics
    safe_addstr(26, 0, "-" * 60)
    safe_addstr(27, 0, "PICO DIAGNOSTICS:", curses.A_BOLD)
    if diag:
        conn_ok = diag.get('connected', False)
        fresh = diag.get('fresh', False)
        frames = diag.get('frames', 0)
        pkts = diag.get('packets_received', 0)
        pico_errors = diag.get('errors', 0)
        laser = diag.get('laser_mm', -1)
        age = diag.get('age_s', -1)

        if conn_ok and fresh:
            status_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            conn_str = "CONNECTED (live)"
        elif conn_ok:
            status_attr = curses.color_pair(1) if curses.has_colors() else curses.A_DIM
            conn_str = f"CONNECTED (stale — {age}s ago)"
        else:
            status_attr = curses.color_pair(1) if curses.has_colors() else curses.A_DIM
            conn_str = "NO DATA"

        safe_addstr(28, 2, f"Link: {conn_str}", status_attr)
        safe_addstr(29, 2, f"Frames: {frames}  |  Pkts rx: {pkts}  |  Pico errors: {pico_errors}")
        safe_addstr(30, 2, f"Laser: {laser} mm  |  Data age: {age}s")
        if conn_ok and fresh and rr == 0.0 and rl == 0.0 and fr == 0.0:
            safe_addstr(31, 2, "⚠️  Pico data OK but RPM=0 → check LM393 encoder wiring!",
                        curses.color_pair(1) if curses.has_colors() else curses.A_BOLD)
    else:
        safe_addstr(28, 2, "No diagnostics available")

    stdscr.refresh()


def main_loop(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)

    speed = float(DEFAULT_SPEED)
    pwms = setup_gpio()

    pico_reader = None
    if PICO_AVAILABLE:
        try: pico_reader = init_pico_reader()
        except Exception: pass

    active_wheel = None
    active_dir = None
    status_msg = "All motors stopped."

    # 4WD PI Synchronization Variables
    rr_pwm_adj_i = 0.0
    fr_pwm_adj_i = 0.0

    # Tunings - Softened KP to prevent violent jerking
    REAR_KP = 0.04
    REAR_KI = 0.02
    FRONT_KP = 0.04
    FRONT_KI = 0.02

    # Deadband Limits (Minimum power required to prevent stalling)
    MIN_REAR_PWM = 18.0    # Plastic gears need more power just to keep moving
    MIN_FRONT_PWM = 8.0    # Metal gears are efficient and can run at very low power

    MAX_ADJ = float(MAX_PWM_DUTY)
    log_file = None

    # RPM Target Mode Variables
    rpm_target = None
    entering_rpm = False
    rpm_input_buf = ""
    rl_rpm_integral = 0.0
    rr_rpm_integral = 0.0
    fr_rpm_integral = 0.0
    rpm_filt_rl = 0.0   # Low-pass filtered RPM readings
    rpm_filt_rr = 0.0
    rpm_filt_fr = 0.0
    rpm_prev_filt_rl = 0.0  # Previous filtered RPM (for derivative)
    rpm_prev_filt_rr = 0.0
    rpm_prev_filt_fr = 0.0
    rpm_pwm_vals = {"rl": 0.0, "rr": 0.0, "fr": 0.0, "fl": 0.0}

    def close_log():
        nonlocal log_file
        if log_file:
            log_file.close()
            log_file = None

    try:
        while True:
            # 1. Non-blocking keypress check
            k = stdscr.getch()
            if k != -1:
                if entering_rpm:
                    # RPM input mode - collect digits
                    if k == 27:  # Escape - cancel input
                        entering_rpm = False
                        rpm_input_buf = ""
                    elif k in (10, 13):  # Enter - confirm
                        try:
                            val = int(rpm_input_buf)
                            if RPM_TARGET_MIN <= val <= RPM_TARGET_MAX:
                                rpm_target = val
                                active_wheel = "rpm_target"
                                active_dir = "forward"
                                # Zero integrals — live feedforward in _pid_step
                                # handles the initial estimate; integrals only
                                # accumulate to trim residual error.
                                rl_rpm_integral = 0.0
                                rr_rpm_integral = 0.0
                                fr_rpm_integral = 0.0
                                rpm_filt_rl = rpm_filt_rr = rpm_filt_fr = 0.0
                                rpm_prev_filt_rl = rpm_prev_filt_rr = rpm_prev_filt_fr = 0.0
                                rpm_pwm_vals = {"rl": 0.0, "rr": 0.0, "fr": 0.0, "fl": 0.0}
                                status_msg = f"RPM-LOCK: Maintaining {rpm_target} RPM (4WD)"
                                close_log()
                                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                                log_file = open(f"rpm_lock_log_{timestamp}.csv", "w")
                                log_file.write("time_sec,target_rpm,rl_rpm,rr_rpm,fr_rpm,rl_pwm,rr_pwm,fr_pwm,fl_pwm\n")
                        except ValueError:
                            pass
                        entering_rpm = False
                        rpm_input_buf = ""
                    elif k in (8, 127, curses.KEY_BACKSPACE):  # Backspace
                        rpm_input_buf = rpm_input_buf[:-1]
                    elif 48 <= k <= 57 and len(rpm_input_buf) < 4:  # Digits 0-9
                        rpm_input_buf += chr(k)
                else:
                    try: key = chr(k).lower()
                    except ValueError: key = None

                    if key == "q":
                        break

                    elif key in (" ", "x"):
                        stop_all(pwms)
                        active_wheel, active_dir = None, None
                        rr_pwm_adj_i, fr_pwm_adj_i = 0.0, 0.0
                        rpm_target = None
                        rl_rpm_integral = rr_rpm_integral = fr_rpm_integral = 0.0
                        rpm_filt_rl = rpm_filt_rr = rpm_filt_fr = 0.0
                        rpm_prev_filt_rl = rpm_prev_filt_rr = rpm_prev_filt_fr = 0.0
                        rpm_pwm_vals = {"rl": 0.0, "rr": 0.0, "fr": 0.0, "fl": 0.0}
                        status_msg = "All motors stopped."
                        close_log()

                    elif key == "i":
                        entering_rpm = True
                        rpm_input_buf = ""

                    elif key in ("+", "="):
                        if active_wheel == "rpm_target" and rpm_target is not None:
                            rpm_target = min(RPM_TARGET_MAX, rpm_target + RPM_TARGET_STEP)
                            status_msg = f"RPM-LOCK: Maintaining {rpm_target} RPM (4WD)"
                        else:
                            speed = min(float(MAX_PWM_DUTY), speed + SPEED_STEP)
                    elif key in ("-", "_"):
                        if active_wheel == "rpm_target" and rpm_target is not None:
                            rpm_target = max(RPM_TARGET_MIN, rpm_target - RPM_TARGET_STEP)
                            status_msg = f"RPM-LOCK: Maintaining {rpm_target} RPM (4WD)"
                        else:
                            speed = max(float(MIN_SPEED), speed - SPEED_STEP)

                    elif key in COMMAND_MAP:
                        new_wheel, new_dir = COMMAND_MAP[key]

                        if new_wheel in ("rear_sync", "all_sync") and active_wheel not in ("rear_sync", "all_sync"):
                            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                            log_file = open(f"sync_4wd_log_{timestamp}.csv", "w")
                            log_file.write("time_sec,target_speed_pwm,rl_rpm,rr_rpm,fr_rpm,rr_adj,fr_adj,final_rr_pwm,final_fr_pwm\n")
                        elif new_wheel not in ("rear_sync", "all_sync"):
                            close_log()

                        active_wheel, active_dir = new_wheel, new_dir
                        rr_pwm_adj_i, fr_pwm_adj_i = 0.0, 0.0
                        rpm_target = None  # Exit RPM-LOCK if switching modes

                        if active_wheel == "all_sync":
                            status_msg = f"ALL-SYNC 4WD driving {active_dir}"
                        elif active_wheel == "rear_sync":
                            status_msg = f"Rear Sync driving {active_dir}"
                        else:
                            drive_one_wheel(pwms, active_wheel, active_dir, speed)
                            status_msg = f"{WHEEL_MAP[active_wheel][3]} driving {active_dir}"

            # 2. Fetch RPM Data  (rr, rl, fr)
            rpm_data = (0.0, 0.0, 0.0)
            diag = None
            if PICO_AVAILABLE:
                _rpm = pico_get_rpm()  # dict: rear_right, rear_left, front_right
                rpm_data = (
                    _rpm.get('rear_right', 0.0),
                    _rpm.get('rear_left', 0.0),
                    _rpm.get('front_right', 0.0),
                )
                diag = pico_get_diagnostics()

            # 3. Handle Active PI Synchronization Loops
            total_adj_rr = 0.0
            total_adj_fr = 0.0

            if active_wheel in ("rear_sync", "all_sync"):
                rr_rpm, rl_rpm, fr_rpm = rpm_data

                if PICO_AVAILABLE:
                    # REAR LOOP (Slave 1)
                    error_rr = rl_rpm - rr_rpm
                    p_term_rr = error_rr * REAR_KP
                    rr_pwm_adj_i += (error_rr * REAR_KI)
                    rr_pwm_adj_i = max(-MAX_ADJ, min(MAX_ADJ, rr_pwm_adj_i))
                    total_adj_rr = max(-MAX_ADJ, min(MAX_ADJ, p_term_rr + rr_pwm_adj_i))

                    # FRONT LOOP (Slave 2) - Only process if 4WD is active
                    if active_wheel == "all_sync":
                        error_fr = rl_rpm - fr_rpm
                        p_term_fr = error_fr * FRONT_KP
                        fr_pwm_adj_i += (error_fr * FRONT_KI)
                        fr_pwm_adj_i = max(-MAX_ADJ, min(MAX_ADJ, fr_pwm_adj_i))
                        total_adj_fr = max(-MAX_ADJ, min(MAX_ADJ, p_term_fr + fr_pwm_adj_i))

                # Apply calculated powers with Anti-Stall floors
                if speed > 0:
                    rr_target_pwm = max(MIN_REAR_PWM, min(MAX_PWM_DUTY, speed + total_adj_rr))
                    fr_target_pwm = max(MIN_FRONT_PWM, min(MAX_PWM_DUTY, speed + total_adj_fr))
                else:
                    rr_target_pwm = 0.0
                    fr_target_pwm = 0.0

                if active_wheel == "rear_sync":
                    drive_rear_sync(pwms, active_dir, speed, rr_target_pwm)
                elif active_wheel == "all_sync":
                    # FL passively copies FR's power
                    drive_all_sync(pwms, active_dir, speed, rr_target_pwm, fr_target_pwm, fr_target_pwm)

                # Log data
                if log_file:
                    log_file.write(f"{time.time():.3f},{speed},{rl_rpm:.1f},{rr_rpm:.1f},{fr_rpm:.1f},{total_adj_rr:.3f},{total_adj_fr:.3f},{rr_target_pwm:.1f},{fr_target_pwm:.1f}\n")

            # 3b. RPM Target Mode PI Loop
            if active_wheel == "rpm_target" and rpm_target is not None:
                rr_rpm, rl_rpm, fr_rpm = rpm_data

                if PICO_AVAILABLE:
                    # Low-pass filter raw RPM readings to reject encoder noise
                    rpm_prev_filt_rl = rpm_filt_rl
                    rpm_prev_filt_rr = rpm_filt_rr
                    rpm_prev_filt_fr = rpm_filt_fr
                    rpm_filt_rl = RPM_FILTER_ALPHA * rl_rpm + (1.0 - RPM_FILTER_ALPHA) * rpm_filt_rl
                    rpm_filt_rr = RPM_FILTER_ALPHA * rr_rpm + (1.0 - RPM_FILTER_ALPHA) * rpm_filt_rr
                    rpm_filt_fr = RPM_FILTER_ALPHA * fr_rpm + (1.0 - RPM_FILTER_ALPHA) * rpm_filt_fr

                    def _pid_step(filt_rpm, prev_filt_rpm, integral, prev_pwm):
                        """
                        PID tick with feedforward + back-calculation anti-windup.

                        Feedforward: jump-starts the output near the correct PWM
                        so the PI only handles small corrections.

                        Derivative: acts on the *measurement* (not error) to damp
                        oscillations without amplifying setpoint changes.

                        Back-calculation: when the actual applied output differs
                        from the raw PID output (due to clamping or slew limiting),
                        a tracking term pulls the integral back toward reality.
                        """
                        error = rpm_target - filt_rpm
                        # Feedforward: estimated PWM for this RPM target
                        ff = rpm_target * RPM_FF_GAIN
                        # Derivative on measurement (negative because rising RPM
                        # should reduce output).  Divided by dt=0.05s for proper
                        # units, but absorbed into RPM_KD tuning.
                        d_rpm = filt_rpm - prev_filt_rpm
                        d_term = -RPM_KD * d_rpm
                        # Raw (unclamped, un-slewed) PID output
                        raw = ff + integral + error * RPM_KP + d_term
                        # Hard clamp to valid PWM range
                        clamped = max(0.0, min(float(MAX_PWM_DUTY), raw))
                        # Asymmetric slew-rate limiter:
                        #   - moderate ramp UP  (feedforward already near target)
                        #   - fast ramp DOWN (quick over-speed correction)
                        delta = clamped - prev_pwm
                        if delta > 0:
                            output = prev_pwm + min(delta, RPM_SLEW_UP)
                        else:
                            output = prev_pwm + max(delta, -RPM_SLEW_DOWN)
                        output = max(0.0, min(float(MAX_PWM_DUTY), output))
                        # Back-calculation anti-windup:
                        # integral += Ki * error  +  Kt * (actual_output - raw_output)
                        integral += RPM_KI * error + RPM_KT * (output - raw)
                        integral = max(-float(MAX_PWM_DUTY), min(float(MAX_PWM_DUTY), integral))
                        return output, integral

                    rl_pwm_val, rl_rpm_integral = _pid_step(rpm_filt_rl, rpm_prev_filt_rl, rl_rpm_integral, rpm_pwm_vals.get('rl', 0.0))
                    rr_pwm_val, rr_rpm_integral = _pid_step(rpm_filt_rr, rpm_prev_filt_rr, rr_rpm_integral, rpm_pwm_vals.get('rr', 0.0))
                    fr_pwm_val, fr_rpm_integral = _pid_step(rpm_filt_fr, rpm_prev_filt_fr, fr_rpm_integral, rpm_pwm_vals.get('fr', 0.0))
                else:
                    # No sensor feedback — linear PWM estimate, asymmetric slew
                    est_pwm = (rpm_target / RPM_TARGET_MAX) * MAX_PWM_DUTY
                    def _slew(prev, target):
                        delta = target - prev
                        if delta > 0:
                            return max(0.0, min(float(MAX_PWM_DUTY), prev + min(delta, RPM_SLEW_UP)))
                        else:
                            return max(0.0, min(float(MAX_PWM_DUTY), prev + max(delta, -RPM_SLEW_DOWN)))
                    rl_pwm_val = _slew(rpm_pwm_vals.get('rl', 0.0), est_pwm)
                    rr_pwm_val = _slew(rpm_pwm_vals.get('rr', 0.0), est_pwm)
                    fr_pwm_val = _slew(rpm_pwm_vals.get('fr', 0.0), est_pwm)

                fl_pwm_val = fr_pwm_val  # Front-Left mirrors Front-Right (no sensor)
                rpm_pwm_vals = {"rl": rl_pwm_val, "rr": rr_pwm_val, "fr": fr_pwm_val, "fl": fl_pwm_val}

                drive_all_sync(pwms, active_dir, rl_pwm_val, rr_pwm_val, fl_pwm_val, fr_pwm_val)

                if log_file:
                    log_file.write(f"{time.time():.3f},{rpm_target},{rl_rpm:.1f},{rr_rpm:.1f},{fr_rpm:.1f},"
                                   f"{rl_pwm_val:.1f},{rr_pwm_val:.1f},{fr_pwm_val:.1f},{fl_pwm_val:.1f}\n")

            # 4. Draw to screen
            draw_ui(stdscr, speed, status_msg, rpm_data, total_adj_rr, total_adj_fr, bool(log_file), MIN_REAR_PWM, MIN_FRONT_PWM,
                    rpm_target=rpm_target, entering_rpm=entering_rpm, rpm_input_buf=rpm_input_buf, rpm_pwm_vals=rpm_pwm_vals,
                    diag=diag)

            # 5. Loop throttle (20Hz)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        close_log()
        stop_all(pwms)
        for pwm in pwms.values(): pwm.stop()
        GPIO.cleanup()
        if pico_reader: pico_reader.close()

if __name__ == "__main__":
    curses.wrapper(main_loop)
    print("Motor test completed. Hardware cleaned up safely.")
