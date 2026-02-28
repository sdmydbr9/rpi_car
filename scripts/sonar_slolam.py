import RPi.GPIO as GPIO
import time
import curses
import threading
import random
import csv
import os
from datetime import datetime

# Import the Pico bridge for ALL sensors
from pico_sensor_reader import (
    init_pico_reader,
    get_ir_sensors,
    get_battery_voltage,
    get_current_sense,
    get_rpm,
    get_gyro_z,
    get_accel_xyz,
    get_laser_distance_mm
)

# =================================================================
# ⚙️ HARDWARE CONFIGURATION
# =================================================================

# --- DRIVER 1: FRONT ---
FL_IN1 = 17; FL_IN2 = 27; FL_ENA = 12
FR_IN3 = 23; FR_IN4 = 22; FR_ENB = 13

# --- DRIVER 2: REAR ---
RL_IN1 = 9;  RL_IN2 = 11; RL_ENA = 26
RR_IN3 = 10; RR_IN4 = 7;  RR_ENB = 16

# --- SENSORS ---
TRIG_F = 25; ECHO_F = 24
SERVO_PIN = 20  

# --- SLALOM TUNING ---
MAX_SPEED = 90      
MIN_SPEED = 35      
RAMP_STEP = 20      

# --- GYRO TUNING ---
GYRO_KP = 0.15      
MAX_YAW_RATE = 120.0 

# Trims
TRIM_FL = 0.6; TRIM_FR = 0.6
TRIM_RL = 1.0; TRIM_RR = 1.0

# =================================================================
# 📊 TELEMETRY LOGGER SETUP
# =================================================================
if not os.path.exists("rover_logs"):
    os.makedirs("rover_logs")

log_filename = datetime.now().strftime("rover_logs/rover_log_%Y%m%d_%H%M%S.csv")
log_file = open(log_filename, mode='w', newline='')
log_writer = csv.writer(log_file)

log_writer.writerow([
    "Timestamp", "Sonar_Dist_cm", "IR_Left", "IR_Right", 
    "Laser_Left_cm", "Laser_Right_cm", "Laser_Active", "Gyro_Z_degs", 
    "Target_Throttle", "Target_Steering", "Actual_PWM_L", "Actual_PWM_R", 
    "Battery_V", "Current_A", "RPM", "Status_Msg"
])

# =================================================================
# 🔧 SETUP
# =================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pico = init_pico_reader()

motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB,
              RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]
GPIO.setup(motor_pins, GPIO.OUT)
GPIO.setup(TRIG_F, GPIO.OUT)
GPIO.setup(ECHO_F, GPIO.IN)

pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)

# STATE VARIABLES
running = False
status_msg = "Ready"
curr_L = 0.0; curr_R = 0.0
global_max_duty = 95.0

# FILTER VARIABLES (The Anti-Wiggle System)
smoothed_steering = 0.0
smoothed_throttle = 0.0

# =================================================================
# 🎯 WIDE-ANGLE LASER TARGETING SYSTEM
# =================================================================

class LaserTargetingSystem:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)

        self.left_clearance = 200.0
        self.right_clearance = 200.0
        self.active = False
        self.running = True

        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()

    def _set_angle(self, angle):
        servo_deg = 90 - angle
        duty = 2.0 + (servo_deg / 18.0)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.15)
        self.pwm.ChangeDutyCycle(0)

    def _get_laser_cm(self):
        mm = get_laser_distance_mm()
        if mm is None or mm < 0 or mm > 2000:
            return 200.0
        return mm / 10.0

    def _scan_loop(self):
        while self.running:
            if self.active:
                self._set_angle(-60)
                self.left_clearance = self._get_laser_cm()
                
                self._set_angle(60)
                self.right_clearance = self._get_laser_cm()
            else:
                self._set_angle(0)
                time.sleep(0.2)

    def stop(self):
        self.running = False
        self.pwm.stop()

laser = LaserTargetingSystem(SERVO_PIN)

# =================================================================
# ⚡ DYNAMIC VOLTAGE COMPENSATION
# =================================================================

def calculate_safe_duty(throttle_percent, trim=1.0):
    global global_max_duty
    v_target_motor = (throttle_percent / 100.0) * 6.0 * trim
    v_batt = get_battery_voltage()
    current_amps = get_current_sense()

    if v_batt < 5.0: v_batt = 12.6
    if current_amps < 0: current_amps = 0.0

    v_drop = max(1.8, min(3.0, 1.8 + 0.6 * current_amps))
    v_effective = max(1.0, v_batt - v_drop)

    duty = (v_target_motor / v_effective) * 100.0
    max_safe_duty = (8.0 / v_effective) * 100.0
    global_max_duty = max(0.0, min(95.0, max_safe_duty))

    return max(0.0, min(global_max_duty, duty))

# =================================================================
# 🧠 SMOOTH MOTOR KERNEL & YAW-ASSISTED CAR MIXER
# =================================================================

def get_sonar():
    GPIO.output(TRIG_F, True); time.sleep(0.00001); GPIO.output(TRIG_F, False)
    start = time.time(); stop = time.time()
    while GPIO.input(ECHO_F) == 0:
        start = time.time()
        if start - stop > 0.02: return 300
    while GPIO.input(ECHO_F) == 1:
        stop = time.time()
        if stop - start > 0.02: return 300
    return (stop - start) * 17150

def ramp_motors(target_L, target_R, dir_L, dir_R):
    global curr_L, curr_R

    if curr_L < target_L: curr_L = min(target_L, curr_L + RAMP_STEP)
    elif curr_L > target_L: curr_L = max(target_L, curr_L - RAMP_STEP)

    if curr_R < target_R: curr_R = min(target_R, curr_R + RAMP_STEP)
    elif curr_R > target_R: curr_R = max(target_R, curr_R - RAMP_STEP)

    pwm_fl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_FL))
    pwm_rl.ChangeDutyCycle(calculate_safe_duty(curr_L, TRIM_RL))
    pwm_fr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_FR))
    pwm_rr.ChangeDutyCycle(calculate_safe_duty(curr_R, TRIM_RR))

    if dir_L == 1: 
        GPIO.output([FL_IN1, RL_IN1], 1); GPIO.output([FL_IN2, RL_IN2], 0)
    elif dir_L == -1: 
        GPIO.output([FL_IN1, RL_IN1], 0); GPIO.output([FL_IN2, RL_IN2], 1)
    else: 
        GPIO.output([FL_IN1, RL_IN1, FL_IN2, RL_IN2], 0)

    if dir_R == 1: 
        GPIO.output([FR_IN3, RR_IN3], 1); GPIO.output([FR_IN4, RR_IN4], 0)
    elif dir_R == -1: 
        GPIO.output([FR_IN3, RR_IN3], 0); GPIO.output([FR_IN4, RR_IN4], 1)
    else: 
        GPIO.output([FR_IN3, RR_IN3, FR_IN4, RR_IN4], 0)

def drive_like_a_car(throttle, steering, current_gz):
    target_yaw_rate = (steering / 100.0) * MAX_YAW_RATE
    yaw_error = target_yaw_rate - current_gz
    corrected_steering = steering + (yaw_error * GYRO_KP)
    corrected_steering = max(-100, min(100, corrected_steering))

    left_target = throttle + corrected_steering
    right_target = throttle - corrected_steering

    max_val = max(abs(left_target), abs(right_target))
    if max_val > 100.0:
        left_target = (left_target / max_val) * 100.0
        right_target = (right_target / max_val) * 100.0

    dir_L = 1 if left_target >= 0 else -1
    dir_R = 1 if right_target >= 0 else -1

    if throttle == 0 and steering == 0:
        dir_L = 0; dir_R = 0

    ramp_motors(abs(left_target), abs(right_target), dir_L, dir_R)

# =================================================================
# 🧠 SMART DRIVER BRAIN
# =================================================================

def calculate_dynamic_speed(dist):
    if dist > 150: return MAX_SPEED
    if dist < 40: return MIN_SPEED
    return int(MIN_SPEED + (dist - 40) * ((MAX_SPEED - MIN_SPEED) / 110.0))

# =================================================================
# 🧠 IMPROVED AUTOPILOT v2 — STATE-MACHINE ARCHITECTURE
# =================================================================
#
# Upgrades over v1:
#   1. Finite state machine with clean transitions
#   2. Predictive steering via closing-speed estimation
#   3. Stuck detection using RPM + throttle correlation
#   4. Multi-strategy escape with retry escalation
#   5. Wall-following mode for corridor navigation
#   6. Per-state adaptive filter weights
#   7. Cooldown logic to prevent state oscillation
# =================================================================

# --- States ---
ST_IDLE     = "IDLE"
ST_CRUISE   = "CRUISE"
ST_SLALOM   = "SLALOM"
ST_CARVE    = "CARVE"
ST_WALL     = "WALL_FOLLOW"
ST_ESCAPE   = "ESCAPE"
ST_STUCK    = "STUCK_RECOVERY"

# --- Autopilot Tuning ---
APPROACH_RATE_ALPHA   = 0.4     # EMA weight for closing-speed estimate
STUCK_RPM_THRESHOLD   = 5.0    # RPM below this while driving = possibly stuck
STUCK_TICKS_TRIGGER   = 25     # ~1.25 s of no movement before recovery
ESCAPE_MAX_STRATEGIES = 4      # Number of distinct escape manoeuvres
CORRIDOR_WIDTH_THRESH = 100.0  # cm — both sides below this = corridor
CARVE_CLEARANCE_MIN   = 80.0   # cm — min open-side room to allow carve
STATE_COOLDOWN_TICKS  = 6      # prevent rapid state flipping (~0.3 s)


class SmartAutoPilot:
    """State-machine autopilot with predictive steering, stuck detection,
    and multi-strategy escape."""

    def __init__(self):
        self.state = ST_IDLE

        # Sensor history for prediction
        self.prev_dist = 300.0
        self.approach_rate = 0.0      # cm/tick closing speed (positive = closing)

        # Stuck detection
        self.stuck_ticks = 0
        self.escape_retries = 0
        self.last_escape_dir = 1      # 1 = right, -1 = left

        # State timers
        self.state_ticks = 0
        self.cooldown_ticks = 0

    # --------------------------------------------------
    # Sensor reading (single call per tick, no duplication)
    # --------------------------------------------------
    def _read_sensors(self):
        dist = get_sonar()
        gz = get_gyro_z()
        try:
            ir_l, ir_r = get_ir_sensors()
        except Exception:
            ir_l, ir_r = False, False
        rpm = get_rpm()
        return dist, gz, ir_l, ir_r, rpm

    # --------------------------------------------------
    # Predictive closing-speed estimator
    # --------------------------------------------------
    def _update_approach_rate(self, dist):
        raw_rate = self.prev_dist - dist          # positive = closing
        self.approach_rate = (APPROACH_RATE_ALPHA * raw_rate +
                              (1.0 - APPROACH_RATE_ALPHA) * self.approach_rate)
        self.prev_dist = dist

    # --------------------------------------------------
    # Stuck detector (RPM vs commanded throttle)
    # --------------------------------------------------
    def _check_stuck(self, rpm, throttle):
        if abs(throttle) > 20 and rpm < STUCK_RPM_THRESHOLD:
            self.stuck_ticks += 1
        else:
            self.stuck_ticks = max(0, self.stuck_ticks - 2)   # decay
        return self.stuck_ticks >= STUCK_TICKS_TRIGGER

    # --------------------------------------------------
    # State transition decision
    # --------------------------------------------------
    def _decide_state(self, dist, ir_l, ir_r):
        # Cooldown prevents rapid state flipping
        if self.cooldown_ticks > 0:
            self.cooldown_ticks -= 1
            return self.state

        # === PANIC: both IR or extremely close ===
        if (ir_l and ir_r) or dist < 15:
            return ST_ESCAPE

        # === Laser-guided states ===
        if laser.active:
            both_close = (laser.left_clearance < CORRIDOR_WIDTH_THRESH and
                          laser.right_clearance < CORRIDOR_WIDTH_THRESH)

            # Corridor: both sides tight but still some room ahead
            if both_close and dist > 50:
                return ST_WALL

            # Close obstacle with one clear side → high-speed carve
            if dist < 70:
                max_cl = max(laser.left_clearance, laser.right_clearance)
                if max_cl > CARVE_CLEARANCE_MIN:
                    return ST_CARVE

            return ST_SLALOM

        return ST_CRUISE

    # --------------------------------------------------
    # Per-state behaviour functions
    # Each returns (raw_throttle, raw_steering, filter_weight)
    # --------------------------------------------------

    def _run_cruise(self, dist, gz):
        speed = calculate_dynamic_speed(dist)
        # Predictive slow-down: reduce speed early when closing fast
        if self.approach_rate > 3.0:
            speed = max(MIN_SPEED, speed - self.approach_rate * 2.5)
        return speed, 0.0, 0.25

    def _run_slalom(self, dist, gz, ir_l, ir_r):
        urgency = max(0.3, min(1.0, (200.0 - dist) / 160.0))
        # Boost urgency when closing fast
        if self.approach_rate > 2.0:
            urgency = min(1.0, urgency + self.approach_rate * 0.05)

        clearance_diff = laser.right_clearance - laser.left_clearance
        if abs(clearance_diff) < 15.0:
            clearance_diff = -30 if laser.left_clearance > laser.right_clearance else 30

        steer = clearance_diff * urgency * 0.55
        speed = calculate_dynamic_speed(dist)
        fw = 0.25

        # IR hard overrides
        if ir_l:
            steer = 90; fw = 0.80
        elif ir_r:
            steer = -90; fw = 0.80

        return speed, max(-100, min(100, steer)), fw

    def _run_carve(self, dist, gz, ir_l, ir_r):
        """High-speed carve around an obstacle into the open side."""
        urgency = max(0.5, min(1.0, (100.0 - dist) / 60.0))
        clearance_diff = laser.right_clearance - laser.left_clearance
        if abs(clearance_diff) < 10.0:
            clearance_diff = -40 if laser.left_clearance > laser.right_clearance else 40

        steer = clearance_diff * urgency * 1.3
        speed = max(MAX_SPEED * 0.75, calculate_dynamic_speed(dist))
        fw = 0.55

        # IR overrides still apply
        if ir_l:
            steer = 95; fw = 0.85
        elif ir_r:
            steer = -95; fw = 0.85

        return speed, max(-100, min(100, steer)), fw

    def _run_wall_follow(self, dist, gz, ir_l, ir_r):
        """Follow corridor: try to stay centred between walls."""
        error = laser.right_clearance - laser.left_clearance   # positive = more room right
        steer = error * 0.6                                     # proportional centering
        speed = min(MAX_SPEED * 0.65, calculate_dynamic_speed(dist))
        fw = 0.30

        if ir_l:
            steer = 70; fw = 0.75
        elif ir_r:
            steer = -70; fw = 0.75

        return speed, max(-100, min(100, steer)), fw

    # --------------------------------------------------
    # Multi-strategy escape manoeuvres
    # Strategies escalate with each retry:
    #   0 → Reverse + arc towards open side
    #   1 → Hard reverse then J-turn (opposite dir)
    #   2 → Long reverse with gentle arc
    #   3 → Full spin (last resort)
    # --------------------------------------------------
    def _execute_escape(self):
        global smoothed_steering, smoothed_throttle, status_msg
        smoothed_steering = 0.0
        smoothed_throttle = 0.0

        # Full stop first
        drive_like_a_car(0, 0, 0)
        time.sleep(0.08)

        strategy = self.escape_retries % ESCAPE_MAX_STRATEGIES

        if strategy == 0:
            status_msg = "🚨 ESCAPE: Reverse-Arc"
            turn_dir = 1 if laser.right_clearance >= laser.left_clearance else -1
            self.last_escape_dir = turn_dir
            for _ in range(14):
                drive_like_a_car(-80, turn_dir * 60, get_gyro_z())
                time.sleep(0.05)

        elif strategy == 1:
            status_msg = "🚨 ESCAPE: J-Turn"
            for _ in range(12):
                drive_like_a_car(-85, 0, get_gyro_z())
                time.sleep(0.05)
            spin = -self.last_escape_dir * 100
            for _ in range(10):
                drive_like_a_car(0, spin, 0)
                time.sleep(0.05)

        elif strategy == 2:
            status_msg = "🚨 ESCAPE: Long Reverse"
            arc = 40 if laser.right_clearance >= laser.left_clearance else -40
            for _ in range(22):
                drive_like_a_car(-75, arc, get_gyro_z())
                time.sleep(0.05)

        else:
            status_msg = "🚨 ESCAPE: Full Spin"
            spin = 100 if laser.right_clearance >= laser.left_clearance else -100
            for _ in range(18):
                drive_like_a_car(0, spin, 0)
                time.sleep(0.05)

        self.escape_retries += 1
        drive_like_a_car(0, 0, 0)
        time.sleep(0.1)

    # --------------------------------------------------
    # Stuck recovery: wiggle free
    # --------------------------------------------------
    def _execute_stuck_recovery(self):
        global smoothed_steering, smoothed_throttle, status_msg
        status_msg = "🔧 STUCK: Wiggle Recovery"
        smoothed_steering = 0.0
        smoothed_throttle = 0.0

        # Alternate short reverse bursts with opposing steering
        for direction in [1, -1, 1]:
            for _ in range(6):
                drive_like_a_car(-70, direction * 80, 0)
                time.sleep(0.05)
            drive_like_a_car(0, 0, 0)
            time.sleep(0.08)

        self.stuck_ticks = 0

    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------
    def run(self):
        global running, status_msg, smoothed_steering, smoothed_throttle

        while True:
            # --- Idle when paused ---
            if not running:
                drive_like_a_car(0, 0, 0)
                laser.active = False
                smoothed_steering = 0.0
                smoothed_throttle = 0.0
                self.state = ST_IDLE
                self.stuck_ticks = 0
                self.escape_retries = 0
                self.approach_rate = 0.0
                time.sleep(0.1)
                continue

            # --- Read all sensors once per tick ---
            dist, gz, ir_l, ir_r, rpm = self._read_sensors()
            self._update_approach_rate(dist)

            # --- Laser activation ---
            laser.active = dist < 180

            # --- Stuck detection (before state logic) ---
            if self._check_stuck(rpm, smoothed_throttle):
                self._execute_stuck_recovery()
                continue

            # --- Decide next state ---
            new_state = self._decide_state(dist, ir_l, ir_r)
            if new_state != self.state:
                self.state_ticks = 0
                self.cooldown_ticks = STATE_COOLDOWN_TICKS
                # Clear escape counter when we cleanly leave escape
                if self.state == ST_ESCAPE and new_state != ST_ESCAPE:
                    self.escape_retries = 0
            self.state = new_state
            self.state_ticks += 1

            # --- Execute current state ---
            if self.state == ST_ESCAPE:
                self._execute_escape()
                continue

            if self.state == ST_CRUISE:
                raw_throttle, raw_steer, fw = self._run_cruise(dist, gz)
                status_msg = (f"🛣️ CRUISE [{int(smoothed_throttle)}%] "
                              f"Δd {self.approach_rate:+.1f}cm/t")

            elif self.state == ST_SLALOM:
                raw_throttle, raw_steer, fw = self._run_slalom(dist, gz, ir_l, ir_r)
                status_msg = (f"🐍 SLALOM d={dist:.0f}cm "
                              f"L:{laser.left_clearance:.0f} R:{laser.right_clearance:.0f}")

            elif self.state == ST_CARVE:
                raw_throttle, raw_steer, fw = self._run_carve(dist, gz, ir_l, ir_r)
                status_msg = (f"🔥 CARVE d={dist:.0f}cm "
                              f"L:{laser.left_clearance:.0f} R:{laser.right_clearance:.0f}")

            elif self.state == ST_WALL:
                raw_throttle, raw_steer, fw = self._run_wall_follow(dist, gz, ir_l, ir_r)
                status_msg = (f"🧱 WALL d={dist:.0f}cm "
                              f"L:{laser.left_clearance:.0f} R:{laser.right_clearance:.0f}")

            else:
                raw_throttle, raw_steer, fw = 0, 0, 0.25

            # --- Dynamic low-pass filters (weight adapts per state) ---
            smoothed_steering = ((1.0 - fw) * smoothed_steering) + (fw * raw_steer)
            smoothed_throttle = (0.70 * smoothed_throttle) + (0.30 * raw_throttle)

            # --- Drive! ---
            drive_like_a_car(smoothed_throttle, smoothed_steering, gz)

            # --- Telemetry logging ---
            try:
                v_batt = get_battery_voltage()
                amps = get_current_sense()

                log_writer.writerow([
                    f"{time.time():.3f}",
                    f"{dist:.1f}",
                    int(ir_l), int(ir_r),
                    f"{laser.left_clearance:.1f}",
                    f"{laser.right_clearance:.1f}",
                    int(laser.active),
                    f"{gz:.2f}",
                    f"{smoothed_throttle:.1f}",
                    f"{smoothed_steering:.1f}",
                    f"{curr_L:.1f}", f"{curr_R:.1f}",
                    f"{v_batt:.2f}", f"{amps:.2f}",
                    f"{rpm:.1f}",
                    status_msg
                ])
                log_file.flush()
            except Exception:
                pass

            time.sleep(0.05)


# Instantiate and start
_pilot = SmartAutoPilot()
t = threading.Thread(target=_pilot.run, daemon=True)
t.start()

# =================================================================
# 🖥️ TELEMETRY UI
# =================================================================

def main(stdscr):
    global running, global_max_duty
    stdscr.nodelay(True)
    curses.curs_set(0)

    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "--- 🏎️  PROACTIVE SLALOM ROVER (CARVE OVERRIDE) ---", curses.A_BOLD)

        v_batt = get_battery_voltage()
        amps = get_current_sense()
        rpm = get_rpm()
        gz = get_gyro_z()

        batt_display = v_batt if v_batt > 0 else 0.0
        amps_display = amps if amps > 0 else 0.0

        stdscr.addstr(2, 0, f"MOTORS:   Left: {int(curr_L)}% | Right: {int(curr_R)}%")
        stdscr.addstr(3, 0, f"POWER:    {batt_display:.1f}V @ {amps_display:.1f}A | Max Safe PWM: {global_max_duty:.1f}%")
        stdscr.addstr(4, 0, f"SPEED:    {rpm:.1f} RPM")
        
        drift_dir = ">>" if gz > 5 else "<<" if gz < -5 else "=="
        stdscr.addstr(5, 0, f"YAW GYRO: {drift_dir} {gz:>5.1f} °/s")

        scan_state = "SCANNING..." if laser.active else "IDLE"
        stdscr.addstr(7, 0, f"LASER:    {scan_state} | L: {laser.left_clearance:.1f}cm | R: {laser.right_clearance:.1f}cm")

        stdscr.addstr(9, 0, f"STATUS:   {status_msg}")
        stdscr.addstr(11, 0, "[S] Start  [SPACE] Pause  [Q] Quit")
        stdscr.addstr(12, 0, f"LOGGING TO: {log_filename}")

        if running: stdscr.addstr(11, 40, "● ON AIR", curses.A_BLINK)

        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord('s'): running = True
        elif key == ord(' '): running = False
        time.sleep(0.1)

    running = False
    laser.stop()
    stop_all()

def stop_all():
    pwm_fl.stop(); pwm_fr.stop(); pwm_rl.stop(); pwm_rr.stop()
    GPIO.cleanup()
    
    try:
        log_file.close()
    except:
        pass

if __name__ == "__main__":
    curses.wrapper(main)
