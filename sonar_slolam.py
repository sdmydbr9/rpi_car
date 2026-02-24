import RPi.GPIO as GPIO
import time
import curses
import threading
import random

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

# --- AGGRESSIVE TUNING ---
MAX_SPEED = 100
MIN_SPEED = 20
RAMP_STEP = 35

# --- GYRO TUNING ---
GYRO_KP = 0.5        # Proportional gain for drift correction
MAX_YAW_RATE = 150.0 # Expected degrees/sec at 100% steering

# Trims
TRIM_FL = 0.6; TRIM_FR = 0.6
TRIM_RL = 1.0; TRIM_RR = 1.0

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
                # WIDER FOV: -60 (Left) and +60 (Right)
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
    """
    throttle: -100 to 100
    steering: -100 (Hard Left) to +100 (Hard Right)
    current_gz: The Z-axis gyro reading in deg/s
    """
    # 1. Active Yaw Rate Control
    target_yaw_rate = (steering / 100.0) * MAX_YAW_RATE
    yaw_error = target_yaw_rate - current_gz
    corrected_steering = steering + (yaw_error * GYRO_KP)
    corrected_steering = max(-100, min(100, corrected_steering))

    # 2. Car Steering Mixer
    left_target = throttle + corrected_steering
    right_target = throttle - corrected_steering

    # Normalize limits
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
    global status_msg
    status_msg = "🚨 ESCAPING..."
    
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
    global running, status_msg

    while True:
        if not running:
            drive_like_a_car(0, 0, 0)
            laser.active = False
            time.sleep(0.1)
            continue

        dist = get_sonar()
        gz = get_gyro_z() 

        try:
            ir_left, ir_right = get_ir_sensors()
        except:
            ir_left, ir_right = False, False

        # 1. Wake up the laser scanner EARLY (Proactive mapping)
        if dist < 180:
            laser.active = True
        else:
            laser.active = False

        # 2. PANIC ZONE (Imminent collision)
        if (ir_left and ir_right) or (dist < 20):
            execute_soft_escape()
            continue

        throttle = calculate_dynamic_speed(dist)
        
        # 3. PROACTIVE ARC STEERING (The "Car" Logic)
        if laser.active:
            # Calculate urgency: 180cm = 0.0 (straight), 40cm = 1.0 (max turn)
            urgency = max(0.0, min(1.0, (180.0 - dist) / 140.0))
            
            clearance_diff = laser.right_clearance - laser.left_clearance
            
            # THE FLAT WALL FIX: If the wall is flat, force a decision!
            if abs(clearance_diff) < 20.0:
                if laser.left_clearance > 150: 
                    clearance_diff = -60 # Pass on the left if it's wide open
                else:
                    clearance_diff = 60  # Default to passing on the right
            
            # Apply the continuous curve. 
            steering_pull = clearance_diff * urgency * 1.5 
            status_msg = f"🚙 CARVING ARC (Urg: {urgency:.2f})"
            
        else:
            steering_pull = 0 
            status_msg = f"🛣️ CRUISING (Throt {throttle}%)"

        # 4. HARD OVERRIDES FOR IR SENSORS
        if ir_left:
            status_msg = "⚡ SWERVE RIGHT (IR)"
            steering_pull = 90
        elif ir_right:
            status_msg = "⚡ SWERVE LEFT (IR)"
            steering_pull = -90

        # Clamp steering to valid mixer range
        steering_pull = max(-100, min(100, steering_pull))

        # Drive!
        drive_like_a_car(throttle, steering_pull, gz)
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
        stdscr.addstr(0, 0, "--- 🏎️  PROACTIVE ARC-CARVING ROVER (WITH ACTIVE YAW) ---", curses.A_BOLD)

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

if __name__ == "__main__":
    curses.wrapper(main)
