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
    get_accel_xyz
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

# --- AGGRESSIVE TUNING ---
MAX_SPEED = 100        # Turbo Speed 
MIN_SPEED = 20         # Crawl Speed 
RAMP_STEP = 35         # Very snappy motor response.

# Trims
TRIM_FL = 0.6; TRIM_FR = 0.6
TRIM_RL = 1.0; TRIM_RR = 1.0

# =================================================================
# 🔧 SETUP
# =================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize Pico UART Bridge
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
curr_L = 0; curr_R = 0
last_turn_dir = 1
current_tr = 0.5  
global_max_duty = 95.0 

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
# 🧠 SMOOTH MOTOR KERNEL
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

def ramp_motors(target_L, target_R, direction):
    global curr_L, curr_R

    if curr_L < target_L: curr_L += RAMP_STEP
    elif curr_L > target_L: curr_L -= RAMP_STEP

    if curr_R < target_R: curr_R += RAMP_STEP
    elif curr_R > target_R: curr_R -= RAMP_STEP

    if abs(curr_L - target_L) < RAMP_STEP: curr_L = target_L
    if abs(curr_R - target_R) < RAMP_STEP: curr_R = target_R

    duty_fl = calculate_safe_duty(curr_L, TRIM_FL)
    duty_fr = calculate_safe_duty(curr_R, TRIM_FR)
    duty_rl = calculate_safe_duty(curr_L, TRIM_RL)
    duty_rr = calculate_safe_duty(curr_R, TRIM_RR)

    pwm_fl.ChangeDutyCycle(duty_fl)
    pwm_fr.ChangeDutyCycle(duty_fr)
    pwm_rl.ChangeDutyCycle(duty_rl)
    pwm_rr.ChangeDutyCycle(duty_rr)

    if direction == 1:   
        GPIO.output([FL_IN1, RL_IN1, FR_IN3, RR_IN3], 1)
        GPIO.output([FL_IN2, RL_IN2, FR_IN4, RR_IN4], 0)
    elif direction == -1: 
        GPIO.output([FL_IN1, RL_IN1, FR_IN3, RR_IN3], 0)
        GPIO.output([FL_IN2, RL_IN2, FR_IN4, RR_IN4], 1)
    elif direction == 0:  
        GPIO.output([FL_IN1, RL_IN1], 1); GPIO.output([FL_IN2, RL_IN2], 0)
        GPIO.output([FR_IN3, RR_IN3], 0); GPIO.output([FR_IN4, RR_IN4], 1)
    elif direction == 2:  
        GPIO.output([FL_IN1, RL_IN1], 0); GPIO.output([FL_IN2, RL_IN2], 1)
        GPIO.output([FR_IN3, RR_IN3], 1); GPIO.output([FR_IN4, RR_IN4], 0)

# =================================================================
# 🧠 SMART DRIVER BRAIN
# =================================================================

def calculate_dynamic_speed(dist):
    """ Brakes much harder and earlier to kill momentum. """
    if dist > 150: return MAX_SPEED
    if dist < 40: return MIN_SPEED
    # Linear slope between 40cm (20%) and 150cm (100%)
    return int(MIN_SPEED + (dist - 40) * ((MAX_SPEED - MIN_SPEED) / 110.0))

def calculate_dynamic_turn_ratio(dist):
    """ Anchor the inner wheel completely at close range! """
    if dist <= 45:
        return 0.0   # Dead stop the inner wheel for a harsh pivot
    elif dist <= 90:
        return 0.0 + (dist - 45) * (0.05 / 45.0) # 0.0 to 0.05
    elif dist <= 150:
        return 0.05 + (dist - 90) * (0.35 / 60.0) # 0.05 to 0.40
    else:
        return 0.6 

def execute_soft_escape():
    global curr_L, curr_R, status_msg
    status_msg = "🚨 ESCAPING..."

    while curr_L > 0 or curr_R > 0:
        ramp_motors(0, 0, 1)
        time.sleep(0.05)

    status_msg = "🔙 REVERSING..."
    for _ in range(12): 
        ramp_motors(85, 85, -1)
        time.sleep(0.05)

    status_msg = "🔄 SPINNING..."
    ramp_motors(0, 0, 1) 

    target_spin = 95 
    spin_dir = 0 if random.choice([True, False]) else 2

    for _ in range(8):
        ramp_motors(target_spin, target_spin, spin_dir)
        time.sleep(0.05)

    while curr_L > 0:
        ramp_motors(0, 0, spin_dir)
        time.sleep(0.05)

    curr_L = 0; curr_R = 0 

def auto_pilot():
    global running, status_msg, last_turn_dir, current_tr

    while True:
        if not running:
            ramp_motors(0, 0, 1)
            time.sleep(0.1)
            continue

        dist = get_sonar()
        
        try:
            ir_left, ir_right = get_ir_sensors()
        except:
            ir_left, ir_right = False, False 

        l_obs = ir_left
        r_obs = ir_right

        # 1. PANIC ZONE (Increased to 20cm)
        if (l_obs and r_obs) or (dist < 20):
            execute_soft_escape()
            continue

        # 2. CALCULATE BASE THROTTLE & DYNAMIC TURN RATIO
        throttle = calculate_dynamic_speed(dist)
        current_tr = calculate_dynamic_turn_ratio(dist)

        # 3. AGGRESSIVE STEERING MODIFIERS
        if l_obs:
            status_msg = "⚡ SWERVE RIGHT"
            ramp_motors(MAX_SPEED, 0, 1)
            last_turn_dir = 1

        elif r_obs:
            status_msg = "⚡ SWERVE LEFT"
            ramp_motors(0, MAX_SPEED, 1)
            last_turn_dir = -1
            
        elif dist < 90: # Hard dodge starts way earlier!
            status_msg = f"⚡ HARD DODGE! (TR: {current_tr:.2f})"
            if last_turn_dir == 1: 
                ramp_motors(MAX_SPEED, throttle * current_tr, 1)
            else: 
                ramp_motors(throttle * current_tr, MAX_SPEED, 1)

        elif dist < 150: # Pre-emptive steering starts very early
            status_msg = f"⤵️ STEERING (TR: {current_tr:.2f})"
            if last_turn_dir == 1: 
                ramp_motors(throttle, throttle * current_tr, 1)
            else: 
                ramp_motors(throttle * current_tr, throttle, 1)

        else:
            status_msg = f"🚀 TURBO (Spd {throttle}%)"
            ramp_motors(throttle, throttle, 1)

        time.sleep(0.05)

# Start Thread
t = threading.Thread(target=auto_pilot)
t.daemon = True
t.start()

# UI Loop
def main(stdscr):
    global running, current_tr, global_max_duty
    stdscr.nodelay(True)
    curses.curs_set(0)

    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "--- 🏎️  AGGRESSIVE SLALOM ROVER ---", curses.A_BOLD)

        # Telemetry variables
        v_batt = get_battery_voltage()
        amps = get_current_sense()
        rpm = get_rpm()
        gz = get_gyro_z()
        ax, ay, az = get_accel_xyz()

        batt_display = v_batt if v_batt > 0 else 0.0
        amps_display = amps if amps > 0 else 0.0

        bar_len = int(curr_L / 5)
        bar = "█" * bar_len
        stdscr.addstr(2, 0, f"THROTTLE: |{bar:<20}| {int(curr_L)}%")
        stdscr.addstr(3, 0, f"POWER:    {batt_display:.1f}V @ {amps_display:.1f}A | Hard Limit: {global_max_duty:.1f}% PWM")
        
        # New Telemetry Data
        stdscr.addstr(4, 0, f"SPEED:    {rpm:.1f} RPM")
        stdscr.addstr(5, 0, f"MPU6050:  Gyro-Z: {gz:>5.1f} °/s | Accel: X:{ax:>5.2f} Y:{ay:>5.2f} Z:{az:>5.2f} g")

        stdscr.addstr(7, 0, f"STATUS:   {status_msg}")
        stdscr.addstr(9, 0, "[S] Start  [SPACE] Pause  [Q] Quit")

        if running: stdscr.addstr(9, 40, "● ON AIR", curses.A_BLINK)

        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord('s'): running = True
        elif key == ord(' '): running = False
        time.sleep(0.1)

    running = False
    stop_all()

def stop_all():
    pwm_fl.stop(); pwm_fr.stop(); pwm_rl.stop(); pwm_rr.stop()
    GPIO.cleanup()

curses.wrapper(main)
