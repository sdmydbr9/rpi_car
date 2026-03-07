#!/usr/bin/env python3
"""
Ncurses Keyboard motor tester with All-Wheel PI-Loop Synchronization.
Master: Rear-Left
Active Slaves: Rear-Right, Front-Right
Passive Slave (Mirrored): Front-Left
Includes Anti-Stall Deadbands for Mixed Metal/Plastic Gearboxes.
"""

import sys
import time
import curses
import datetime

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
    from pico_sensor_reader import init_pico_reader, get_wheel_rpms
    PICO_AVAILABLE = True
except ImportError:
    PICO_AVAILABLE = False


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


def draw_ui(stdscr, speed, status_msg, rpm_data, rr_adj, fr_adj, is_logging, min_rear_pwm, min_front_pwm):
    stdscr.erase()
    
    stdscr.addstr(0, 0, "🏎️  Independent Motor Test (4WD Anti-Stall Sync)", curses.A_BOLD)
    stdscr.addstr(1, 0, "-" * 60)
    
    stdscr.addstr(2, 0, "W/S: Front-Left   |  E/D: Front-Right")
    stdscr.addstr(3, 0, "R/F: Rear-Left    |  T/G: Rear-Right")
    stdscr.addstr(4, 0, "Y/H: Rear-Sync Drive  (Fwd/Rev)")
    stdscr.addstr(5, 0, "A/Z: ALL-SYNC 4WD     (Fwd/Rev)")
    stdscr.addstr(6, 0, "Space / X: Stop all motors")
    stdscr.addstr(7, 0, "+ / -: Change target speed")
    stdscr.addstr(8, 0, "Q: Quit")
    
    stdscr.addstr(10, 0, "-" * 60)
    
    stdscr.addstr(11, 0, "MOTOR STATUS:", curses.A_BOLD)
    stdscr.addstr(12, 0, status_msg)
    stdscr.addstr(13, 0, f"Target Base Speed: {int(speed)}% (Max: {MAX_PWM_DUTY}%)")

    if "Sync" in status_msg:
        # Show specific power applied to each side, respecting the deadband
        rr_power = max(min_rear_pwm, min(MAX_PWM_DUTY, speed + rr_adj))
        fr_power = max(min_front_pwm, min(MAX_PWM_DUTY, speed + fr_adj))
        stdscr.addstr(14, 0, f"RL: {int(speed)}% | RR: {int(rr_power)}% (Adj {rr_adj:+.1f}%)", curses.A_DIM)
        if "ALL-SYNC" in status_msg:
            stdscr.addstr(15, 0, f"FL: {int(fr_power)}% | FR: {int(fr_power)}% (Adj {fr_adj:+.1f}%)", curses.A_DIM)
        
    if is_logging:
        stdscr.addstr(16, 0, "⏺ REC: Saving 4WD sync data to CSV...", curses.color_pair(1) | curses.A_BLINK if curses.has_colors() else curses.A_BLINK)

    stdscr.addstr(18, 0, "-" * 60)
    stdscr.addstr(19, 0, "LIVE RPM SENSORS:", curses.A_BOLD)
    
    if not PICO_AVAILABLE:
        stdscr.addstr(20, 0, "⚠️  pico_sensor_reader not available", curses.color_pair(1) if curses.has_colors() else curses.A_NORMAL)
    else:
        rr, rl, fr = rpm_data
        rl_attr = rr_attr = fr_attr = curses.A_NORMAL
        
        if "Sync" in status_msg and rl > 0:
            if abs(rl - rr) <= 5.0: rr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            if abs(rl - fr) <= 5.0: fr_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
            rl_attr = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD

        stdscr.addstr(21, 2, f"Front-Right (Slave 2) : {fr:6.1f} RPM", fr_attr)
        stdscr.addstr(22, 2, f"Rear-Left   (MASTER)  : {rl:6.1f} RPM", rl_attr)
        stdscr.addstr(23, 2, f"Rear-Right  (Slave 1) : {rr:6.1f} RPM", rr_attr)

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
    
    # NEW: Deadband Limits (Minimum power required to prevent stalling)
    MIN_REAR_PWM = 18.0    # Plastic gears need more power just to keep moving
    MIN_FRONT_PWM = 8.0    # Metal gears are efficient and can run at very low power
    
    MAX_ADJ = float(MAX_PWM_DUTY)  
    log_file = None      

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
                try: key = chr(k).lower()
                except ValueError: key = None

                if key == "q":
                    break

                elif key in (" ", "x"):
                    stop_all(pwms)
                    active_wheel, active_dir = None, None
                    rr_pwm_adj_i, fr_pwm_adj_i = 0.0, 0.0
                    status_msg = "All motors stopped."
                    close_log()

                elif key in ("+", "="):
                    speed = min(float(MAX_PWM_DUTY), speed + SPEED_STEP)
                elif key in ("-", "_"):
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
                    
                    if active_wheel == "all_sync":
                        status_msg = f"ALL-SYNC 4WD driving {active_dir}"
                    elif active_wheel == "rear_sync":
                        status_msg = f"Rear Sync driving {active_dir}"
                    else:
                        drive_one_wheel(pwms, active_wheel, active_dir, speed)
                        status_msg = f"{WHEEL_MAP[active_wheel][3]} driving {active_dir}"

            # 2. Fetch RPM Data
            rpm_data = (0.0, 0.0, 0.0)
            if PICO_AVAILABLE:
                rpm_data = get_wheel_rpms()
                
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

            # 4. Draw to screen
            draw_ui(stdscr, speed, status_msg, rpm_data, total_adj_rr, total_adj_fr, bool(log_file), MIN_REAR_PWM, MIN_FRONT_PWM)
            
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
