import RPi.GPIO as GPIO
import time
import curses
import threading
import random
from mpu6050 import mpu6050

# =================================================================
# ⚙️ HARDWARE & TUNING
# =================================================================
try:
    sensor = mpu6050(0x68)
    gyro_available = True
except:
    gyro_available = False

# Motor Pins
FL_IN1 = 17; FL_IN2 = 27; FL_ENA = 12
FR_IN3 = 23; FR_IN4 = 22; FR_ENB = 13
RL_IN1 = 10; RL_IN2 = 7;  RL_ENA = 19
RR_IN3 = 9;  RR_IN4 = 11; RR_ENB = 18

# Sensors
TRIG_F = 25; ECHO_F = 24
IR_L = 5; IR_R = 6 

# --- TUNING (ULTRA AGGRESSIVE) ---
BASE_SPEED = 50       
ESCAPE_SPEED = 65     # Slightly faster reverse
GYRO_KP = 1.5         # INCREASED to 1.5 (Very snappy)
CRITICAL_DIST = 20    
WARN_DIST = 100       

# Trims
TRIM_FL = 0.6; TRIM_FR = 0.6
TRIM_RL = 1.0; TRIM_RR = 1.0

# =================================================================
# 🔧 SETUP
# =================================================================
GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)

motor_pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN3, FR_IN4, FR_ENB,
              RL_IN1, RL_IN2, RL_ENA, RR_IN3, RR_IN4, RR_ENB]
GPIO.setup(motor_pins, GPIO.OUT)
GPIO.setup(TRIG_F, GPIO.OUT); GPIO.setup(ECHO_F, GPIO.IN)
GPIO.setup([IR_L, IR_R], GPIO.IN)

pwm_fl = GPIO.PWM(FL_ENA, 1000); pwm_fl.start(0)
pwm_fr = GPIO.PWM(FR_ENB, 1000); pwm_fr.start(0)
pwm_rl = GPIO.PWM(RL_ENA, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(RR_ENB, 1000); pwm_rr.start(0)

# Global State
current_yaw = 0.0
target_yaw = 0.0
gyro_drift = 0.0
last_time = time.time()
running = False
status_msg = "Booting..."
dodge_direction = 0  
last_dodge_time = 0

# =================================================================
# 🧠 CORE FUNCTIONS
# =================================================================

def calibrate_gyro():
    global gyro_drift, status_msg
    if not gyro_available: return
    status_msg = "Calibrating..."
    total_z = 0
    for _ in range(100):
        total_z += sensor.get_gyro_data()['z']
        time.sleep(0.01)
    gyro_drift = total_z / 100
    status_msg = "Ready."

calibrate_gyro()

def get_yaw():
    global current_yaw, last_time
    if not gyro_available: return 0
    gyro_z = sensor.get_gyro_data()['z'] - gyro_drift
    now = time.time()
    dt = now - last_time
    last_time = now
    current_yaw += gyro_z * dt
    return current_yaw

def set_motors(left_speed, right_speed, direction):
    ls = max(0, min(100, abs(left_speed))) * TRIM_FL
    rs = max(0, min(100, abs(right_speed))) * TRIM_FR
    rl_s = max(0, min(100, abs(left_speed))) * TRIM_RL
    rr_s = max(0, min(100, abs(right_speed))) * TRIM_RR

    pwm_fl.ChangeDutyCycle(ls); pwm_fr.ChangeDutyCycle(rs)
    pwm_rl.ChangeDutyCycle(rl_s); pwm_rr.ChangeDutyCycle(rr_s)

    if direction == 1: # FWD
        GPIO.output([FL_IN1, RL_IN1, FR_IN3, RR_IN3], 1)
        GPIO.output([FL_IN2, RL_IN2, FR_IN4, RR_IN4], 0)
    elif direction == -1: # REV
        GPIO.output([FL_IN1, RL_IN1, FR_IN3, RR_IN3], 0)
        GPIO.output([FL_IN2, RL_IN2, FR_IN4, RR_IN4], 1)
    elif direction == 0: # SPIN RIGHT
        GPIO.output([FL_IN1, RL_IN1], 1); GPIO.output([FL_IN2, RL_IN2], 0)
        GPIO.output([FR_IN3, RR_IN3], 0); GPIO.output([FR_IN4, RR_IN4], 1)
    elif direction == 2: # SPIN LEFT
        GPIO.output([FL_IN1, RL_IN1], 0); GPIO.output([FL_IN2, RL_IN2], 1)
        GPIO.output([FR_IN3, RR_IN3], 1); GPIO.output([FR_IN4, RR_IN4], 0)

def get_sonar():
    GPIO.output(TRIG_F, True); time.sleep(0.00001); GPIO.output(TRIG_F, False)
    start = stop = time.time()
    while GPIO.input(ECHO_F) == 0:
        start = time.time()
        if start - stop > 0.02: return 300 
    while GPIO.input(ECHO_F) == 1:
        stop = time.time()
        if stop - start > 0.02: return 300 
    return (stop - start) * 17150

def auto_pilot_thread():
    global running, status_msg, target_yaw, current_yaw, dodge_direction, last_dodge_time
    
    while True:
        if not running:
            set_motors(0, 0, 1)
            target_yaw = get_yaw()
            time.sleep(0.1)
            continue
            
        dist = get_sonar()
        left_stuck = GPIO.input(IR_L) == 0 
        right_stuck = GPIO.input(IR_R) == 0 
        
        # ---------------------------------------------------------
        # 🚨 PRIORITY 1: STUCK / TOO CLOSE (ESCAPE LOGIC)
        # ---------------------------------------------------------
        if left_stuck or right_stuck or dist < CRITICAL_DIST:
            status_msg = "🚨 OBSTACLE! ESCAPING..."
            
            # Stop & Reverse
            set_motors(0, 0, 1); time.sleep(0.1)
            set_motors(ESCAPE_SPEED, ESCAPE_SPEED, -1)
            time.sleep(0.8) 
            
            # Spin Decision
            if left_stuck: spin_mode = 0 
            elif right_stuck: spin_mode = 2 
            else: spin_mode = 0 if random.choice([True, False]) else 2
            
            set_motors(ESCAPE_SPEED, ESCAPE_SPEED, spin_mode)
            time.sleep(0.5) 
            
            # Reset Headings
            set_motors(0, 0, 1); time.sleep(0.2)
            current_yaw = 0.0   
            target_yaw = 0.0    
            dodge_direction = 0 
            continue 

        # ---------------------------------------------------------
        # 🧠 PRIORITY 2: SLALOM MEMORY (ULTRA AGGRESSIVE)
        # ---------------------------------------------------------
        elif dist < WARN_DIST:
            
            if dodge_direction == 0:
                dodge_direction = 1 # Default Right
            
            # --- AGGRESSIVE CALCULATION ---
            # Base 15 degrees (was 12)
            # Multiplier 0.30 (was 0.25)
            # Result: Much sharper turns much earlier
            dynamic_increment = 15 + int((100 - dist) * 0.30)
            
            heading_error = abs(target_yaw - current_yaw)
            
            if heading_error < 10: 
                if time.time() - last_dodge_time > 0.5: 
                    status_msg = f"⚠️ OBS PERSIST! +{dynamic_increment}°"
                    target_yaw += (dynamic_increment * dodge_direction)
                    last_dodge_time = time.time()
            else:
                status_msg = f"⚠️ TURNING {dynamic_increment}°..."

        # ---------------------------------------------------------
        # 🟢 PRIORITY 3: CLEAR PATH
        # ---------------------------------------------------------
        else:
            status_msg = "🚀 CLEAR"
            if time.time() - last_dodge_time > 1.0:
                dodge_direction = 0 
            
            # Return to center 
            if target_yaw > 2: target_yaw -= 0.2
            elif target_yaw < -2: target_yaw += 0.2

        # ---------------------------------------------------------
        # 🏎️ EXECUTION (PID)
        # ---------------------------------------------------------
        heading = get_yaw()
        error = target_yaw - heading
        correction = error * GYRO_KP # Now 1.5
        
        l_speed = BASE_SPEED - correction
        r_speed = BASE_SPEED + correction
        
        set_motors(l_speed, r_speed, 1)
        time.sleep(0.02)

# Start Thread
t = threading.Thread(target=auto_pilot_thread)
t.daemon = True
t.start()

# =================================================================
# 🖥️ UI
# =================================================================
def main(stdscr):
    global running
    stdscr.nodelay(True); curses.curs_set(0)
    
    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "--- 🧠 ULTRA AGGRESSIVE ROVER ---", curses.A_BOLD)
        stdscr.addstr(2, 0, "[S] START  [SPACE] PAUSE  [Q] QUIT")
        
        stdscr.addstr(5, 0, f"STATUS:   {status_msg}")
        stdscr.addstr(6, 0, f"HEADING:  {current_yaw:.1f}°")
        stdscr.addstr(7, 0, f"TARGET:   {target_yaw:.1f}°")
        
        if running: stdscr.addstr(5, 40, "● RUNNING")
        else: stdscr.addstr(5, 40, "○ PAUSED")

        key = stdscr.getch()
        if key == ord('q'): break
        elif key == ord('s'): running = True
        elif key == ord(' '): running = False
        time.sleep(0.1)

    GPIO.cleanup()

curses.wrapper(main)
