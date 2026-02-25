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

def execute_soft_escape():
    global status_msg, smoothed_steering, smoothed_throttle
    status_msg = "🚨 ESCAPING..."
    
    # Reset filters during a panic maneuver so it doesn't drag
    smoothed_steering = 0.0
    smoothed_throttle = 0.0
    
    drive_like_a_car(0, 0, 0)
    time.sleep(0.1)

    status_msg = "🔙 REVERSING..."
    for _ in range(12):
        gz = get_gyro_z() 
        drive_like_a_car(-85, 0, gz) 
        time.sleep(0.05)

    status_msg = "🔄 J-TURN..."
    spin_steer = 100 if laser.right_clearance > laser.left_clearance else -100
    for _ in range(8):
        drive_like_a_car(0, spin_steer, 0) 
        time.sleep(0.05)

def auto_pilot():
    global running, status_msg, curr_L, curr_R
    global smoothed_steering, smoothed_throttle

    while True:
        if not running:
            drive_like_a_car(0, 0, 0)
            laser.active = False
            smoothed_steering = 0.0
            smoothed_throttle = 0.0
            time.sleep(0.1)
            continue

        dist = get_sonar()
        gz = get_gyro_z() 

        try:
            ir_left, ir_right = get_ir_sensors()
        except:
            ir_left, ir_right = False, False

        if dist < 180:
            laser.active = True
        else:
            laser.active = False

        # 2. PANIC ZONE (Safety Net)
        if (ir_left and ir_right) or (dist < 20):
            execute_soft_escape()
            continue

        # Calculate base raw targets
        raw_throttle = calculate_dynamic_speed(dist)
        raw_steering_pull = 0.0
        steer_filter_weight = 0.20  # Default: 20% snappy, 80% smooth gliding
        
        # 3. PROACTIVE ARC STEERING
        if laser.active:
            urgency = max(0.3, min(1.0, (200.0 - dist) / 160.0))
            clearance_diff = laser.right_clearance - laser.left_clearance
            
            if abs(clearance_diff) < 20.0:
                clearance_diff = -35 if laser.left_clearance > laser.right_clearance else 35
            
            steer_mult = 0.55 # Default gentle slalom
            
            # 🚀 THE "CARVE, DON'T BRAKE" OVERRIDE 🚀
            # If an obstacle is close, but we have a clear escape route...
            if dist < 70.0:
                max_clearance = max(laser.left_clearance, laser.right_clearance)
                
                # If the most open side has more than 80cm of room...
                if max_clearance > 80.0:
                    # 1. Keep the speed up! Don't let it drop below 80% of MAX_SPEED
                    raw_throttle = max(raw_throttle, MAX_SPEED * 0.80) 
                    
                    # 2. Crank up the steering multiplier to take a sharp curve
                    steer_mult = 1.3 
                    
                    # 3. Make the filter react 3x faster to snap into the turn
                    steer_filter_weight = 0.60 
                    
                    status_msg = f"🔥 HIGH-SPEED CARVE!"
                else:
                    status_msg = f"🐍 SLALOM (Urg: {urgency:.2f})"
            else:
                status_msg = f"🐍 SLALOM (Urg: {urgency:.2f})"
            
            raw_steering_pull = clearance_diff * urgency * steer_mult 
        else:
            status_msg = f"🛣️ CRUISING (Throt {int(smoothed_throttle)}%)"

        # 4. HARD OVERRIDES FOR IR SENSORS
        if ir_left:
            status_msg = "⚡ SWERVE RIGHT (IR)"
            raw_steering_pull = 90
            steer_filter_weight = 0.80 # Snap away instantly
        elif ir_right:
            status_msg = "⚡ SWERVE LEFT (IR)"
            raw_steering_pull = -90
            steer_filter_weight = 0.80 # Snap away instantly

        raw_steering_pull = max(-100, min(100, raw_steering_pull))

        # --- THE DYNAMIC LOW-PASS FILTERS ---
        # The steer_filter_weight changes depending on if we are cruising or carving!
        smoothed_steering = ((1.0 - steer_filter_weight) * smoothed_steering) + (steer_filter_weight * raw_steering_pull)
        smoothed_throttle = (0.70 * smoothed_throttle) + (0.30 * raw_throttle)

        # Drive!
        drive_like_a_car(smoothed_throttle, smoothed_steering, gz)

        # Log Data
        try:
            timestamp = time.time()
            v_batt = get_battery_voltage()
            amps = get_current_sense()
            rpm = get_rpm()

            log_writer.writerow([
                f"{timestamp:.3f}", 
                f"{dist:.1f}", 
                int(ir_left), 
                int(ir_right), 
                f"{laser.left_clearance:.1f}", 
                f"{laser.right_clearance:.1f}", 
                int(laser.active), 
                f"{gz:.2f}", 
                f"{smoothed_throttle:.1f}", 
                f"{smoothed_steering:.1f}", 
                f"{curr_L:.1f}", 
                f"{curr_R:.1f}", 
                f"{v_batt:.2f}", 
                f"{amps:.2f}", 
                f"{rpm:.1f}", 
                status_msg
            ])
            log_file.flush() 
        except Exception:
            pass 

        time.sleep(0.05)

# Start Thread
t = threading.Thread(target=auto_pilot)
t.daemon = True
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
