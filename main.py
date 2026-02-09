import RPi.GPIO as GPIO
from flask import Flask, render_template, send_from_directory, jsonify, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit, join_room, leave_room
import os
import subprocess
import sys
import time
import threading
import socket
import re
from enum import Enum
from collections import deque
from sensors import SensorSystem
from motor import CarSystem
from autopilot import AutoPilot, State

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================

# --- PROJECT PATHS ---
# The folder where the built website ends up
DIST_DIR = "dist"

# --- MOTOR PINS (BCM Numbering) ---
# Check your wiring! 
IN1 = 17  # Left Motor Direction A
IN2 = 27  # Left Motor Direction B
IN3 = 22  # Right Motor Direction A
IN4 = 23  # Right Motor Direction B
ENA = 18  # Left Motor Speed (PWM)
ENB = 19  # Right Motor Speed (PWM)

# --- SONAR SENSOR PINS (BCM Numbering) ---
# Front Sonar: TRIG → Pin 22 (GPIO 25), ECHO → Pin 18 (GPIO 24)
SONAR_TRIG = 25  # GPIO 25 - Sonar Trigger
SONAR_ECHO = 24  # GPIO 24 - Sonar Echo

# --- IR OBSTACLE SENSORS (BCM Numbering) ---
# Left Front IR: VCC → Pin 1, GND → Pin 9, OUT → Pin 29 (GPIO 5)
# Right Front IR: VCC → Pin 17, GND → Pin 25, OUT → Pin 31 (GPIO 6)
LEFT_IR = 5   # GPIO 5 - Left Front Obstacle Detection
RIGHT_IR = 6  # GPIO 6 - Right Front Obstacle Detection
AVOID_SWERVE_ANGLE = 85  # Degrees to swerve when obstacle detected (increased for aggressive steering)

# --- PHYSICS TUNING ---
ACCEL_RATE = 2.0    # How snappy the car speeds up (Higher = Faster)
COAST_RATE = 0.5    # How slowly it stops when you let go of gas (Coasting)
BRAKE_RATE = 1.5    # Brake deceleration rate (stops from 100 in ~3-4 seconds)
EMERGENCY_BRAKE_RATE = 5.0  # Emergency brake deceleration (faster stop on obstacle)
FREQ = 1000         # PWM Frequency (Hz)

# --- SONAR DISTANCE THRESHOLDS (cm) ---
SONAR_STOP_DISTANCE = 15      # Emergency stop below this distance
SONAR_CRAWL_DISTANCE = 25      # Very slow movement (5% speed) at this distance
SONAR_SLOW_DISTANCE = 40       # Start slowing down at this distance
SONAR_CAUTION_DISTANCE = 60     # Reduce speed moderately at this distance
SONAR_MAX_DISTANCE = 400        # Maximum reliable distance

# --- OBSTACLE AVOIDANCE POWER SETTINGS ---
REVERSE_POWER = 40            # Power level (%) for obstacle avoidance reverse (20-60 recommended)
MIN_REVERSE_POWER = 15         # Minimum power needed to move motors (adjust if motors don't start)

# ==========================================
# 🛠️ AUTO-BUILDER
# ==========================================
def check_and_build():
    """Checks if the React UI is built. If not, runs 'npm run build'."""
    if not os.path.exists(DIST_DIR):
        print("⚠️  UI build not found. Building now...")
        print("⏳ This might take 2-3 minutes on a Raspberry Pi...")
        try:
            subprocess.run(["npm", "run", "build"], check=True)
            print("✅ Build Complete!")
        except Exception as e:
            print(f"❌ Build failed: {e}")
            print("   Make sure you are in the 'rpi_car' folder and npm dependencies are installed.")
            sys.exit(1)
    else:
        print("✅ UI found. Starting engine...")

# Run the check immediately
check_and_build()

# ==========================================
# 🚗 HARDWARE SETUP
# ==========================================

# Clean up any existing GPIO first
try:
    GPIO.cleanup()
except:
    pass

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup Motor Pins
try:
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
except Exception as e:
    print(f"⚠️  GPIO setup error: {e}")
    print("   Retrying with force_cleanup...")
    try:
        GPIO.cleanup()
        GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
    except Exception as e2:
        print(f"❌ GPIO setup failed: {e2}")
        print("   Running without physical GPIO (simulation mode)")

# Setup IR Sensor Pins (Input)
try:
    GPIO.setup([LEFT_IR, RIGHT_IR], GPIO.IN)
except Exception as e:
    print(f"⚠️  IR sensor setup error: {e}")

# ── CarSystem owns the ONLY PWM channels on ENA / ENB ────────────
# Creating module-level PWMs here and again inside CarSystem caused
# RPi.GPIO to raise RuntimeError ("A PWM object already exists"),
# forcing CarSystem to fall back to DummyPWM (silent no-ops).
# Fix: let CarSystem create them first; physics_loop reuses them.
car_system = CarSystem()
pwm_a = car_system.pwm_a
pwm_b = car_system.pwm_b

# Initialize Sensor System
try:
    sensor_system = SensorSystem()
    print("✅ Sensor system initialized (Sonar + IR)")
except Exception as e:
    print(f"⚠️  Sensor system initialization error: {e}")
    # Create dummy sensor system for testing
    class DummySensorSystem:
        def get_sonar_distance(self): return 100
        def get_rear_sonar_distance(self): return 100
        def get_ir_status(self): return False, False
    sensor_system = DummySensorSystem()

# ==========================================
# 🏎️ AUTOPILOT INSTANTIATION
# ==========================================
# CarSystem was already created above (before sensor init) to avoid
# duplicate-PWM conflicts.  Only the autopilot wiring remains here.

def _get_sonar_for_autopilot():
    return get_smoothed_sonar()

def _get_rear_sonar_for_autopilot():
    """Read rear sonar distance via sensor_system."""
    try:
        return sensor_system.get_rear_sonar_distance()
    except Exception:
        return -1

def _get_ir_for_autopilot():
    """Read IR sensors: True = obstacle detected (active LOW, inverted)."""
    try:
        left  = not GPIO.input(LEFT_IR)
        right = not GPIO.input(RIGHT_IR)
    except Exception:
        left = False
        right = False
    return left, right

autopilot = AutoPilot(car_system, _get_sonar_for_autopilot, _get_ir_for_autopilot, _get_rear_sonar_for_autopilot)

# Store a copy of the original class-level defaults (immutable reference for reset)
_AUTOPILOT_DEFAULT_TUNING = AutoPilot.get_default_tuning()

# Active tuning state — persists across UI reconnections
_active_tuning = dict(_AUTOPILOT_DEFAULT_TUNING)

# ==========================================
# 🧠 PHYSICS ENGINE (Background Thread)
# ==========================================

# Shared Memory (The "Brain" of the car)
car_state = {
    "gear": "1",          # Current Gear: R, N, 1, 2, 3, S
    "gas_pressed": False, # Is throttle held?
    "brake_pressed": False, # Is brake held?
    "direction": "stop",  # stop, forward, backward
    "turning": "straight",# straight, left, right
    "current_pwm": 0.0,   # The actual speed (0-100) right now
    "steer_angle": 0,      # Current steering angle
    "left_obstacle": False, # IR sensor: obstacle detected on left
    "right_obstacle": False, # IR sensor: obstacle detected on right
    "ir_enabled": True,    # IR sensor toggle
    "user_steer_angle": 0, # Store user's manual steering (preserved when avoiding)
    "obstacle_avoidance_active": False,  # Track if currently avoiding
    "speed_limit": 100,    # Manual speed limit (5-100), overrides gear speeds
    "speed_limit_enabled": False,  # Whether to use speed limit instead of gear
    "auto_accel_enabled": False,  # Auto-acceleration mode (client-side for throttle)
    "emergency_brake_active": False,  # Emergency brake: when ON, car cannot move
    "obstacle_state": "IDLE",  # Obstacle avoidance state: IDLE, STOPPED, STEERING
    "is_braking": False,  # Flag to indicate normal brake is applied for motor control
    "heartbeat_active": True,  # Heartbeat status: True = client responding, False = lost connection
    "sonar_distance": 100,  # Distance from front sonar sensor in cm
    "sonar_enabled": True,   # Sonar sensor toggle
    # 🤖 AUTONOMOUS DRIVING MODE
    "autonomous_mode": False,  # Smart Driver autonomous mode toggle
    "autonomous_state": State.CRUISING.value,  # Current autonomous driving state (read from AutoPilot)
    "last_obstacle_side": "none",  # Track which side detected obstacle for escape logic
    # 🚨 SENSOR HEALTH STATUS
    "sensor_status": {
        "front_sonar": "OK",  # OK, WARNING, FAILED
        "rear_sonar": "OK",
        "left_ir": "OK",
        "right_ir": "OK",
    },
    "service_light_active": False,  # True if any sensor has error/warning
}

# ==========================================
# 💗 HEARTBEAT / PING TRACKING
# ==========================================
# Dictionary to track last ping time from each connected client
client_last_pong = {}  # {client_id: timestamp}
has_had_client = False  # Track if a client has ever connected (to avoid false emergency brakes on startup)

# Heartbeat configuration
HEARTBEAT_INTERVAL = 2.0  # Send ping every 2 seconds
HEARTBEAT_TIMEOUT = 5.0   # Declare client dead if no pong for 5 seconds

# ==========================================
# 📡 SONAR MOVING-AVERAGE FILTER
# ==========================================
# Circular buffer of last 5 sonar readings for noise rejection.
# At 50Hz read rate, this spans ~100ms — fast enough for real-time,
# smooth enough to reject single bad readings.
sonar_buffer = deque(maxlen=5)

def get_smoothed_sonar():
    """Read sonar and return a moving-average smoothed distance."""
    try:
        raw = sensor_system.get_sonar_distance()
    except:
        raw = -1
    # Only add valid readings to the buffer
    if raw > 0:
        sonar_buffer.append(raw)
    # Return average of buffer, or safe default if empty
    if sonar_buffer:
        return sum(sonar_buffer) / len(sonar_buffer)
    return 100  # Safe default

# ==========================================
# 🚨 SENSOR HEALTH CHECK
# ==========================================

def check_sensor_health():
    """
    Periodically check sensor health and update car_state["sensor_status"]
    Returns a dict with health status for each sensor
    """
    sensor_status = {
        "front_sonar": "OK",
        "rear_sonar": "OK",
        "left_ir": "OK",
        "right_ir": "OK",
    }
    
    has_error = False
    
    try:
        # Check Front Sonar
        dist_front = sensor_system.get_sonar_distance()
        if dist_front == -1:
            sensor_status["front_sonar"] = "FAILED"
            has_error = True
        elif dist_front < 5:
            sensor_status["front_sonar"] = "WARNING"
            has_error = True
        else:
            sensor_status["front_sonar"] = "OK"
    except Exception as e:
        sensor_status["front_sonar"] = "FAILED"
        has_error = True
    
    try:
        # Check Rear Sonar
        dist_rear = sensor_system.get_rear_sonar_distance()
        if dist_rear == -1:
            sensor_status["rear_sonar"] = "FAILED"
            has_error = True
        elif dist_rear < 5:
            sensor_status["rear_sonar"] = "WARNING"
            has_error = True
        else:
            sensor_status["rear_sonar"] = "OK"
    except Exception as e:
        sensor_status["rear_sonar"] = "FAILED"
        has_error = True
    
    try:
        # Check IR Sensors
        left_blocked, right_blocked = sensor_system.get_ir_status()
        # IR sensors are working if they return a boolean value
        sensor_status["left_ir"] = "OK"
        sensor_status["right_ir"] = "OK"
    except Exception as e:
        sensor_status["left_ir"] = "FAILED"
        sensor_status["right_ir"] = "FAILED"
        has_error = True
    
    # Update car_state with sensor status
    car_state["sensor_status"] = sensor_status
    car_state["service_light_active"] = has_error
    
    return sensor_status

def sensor_monitor():
    """
    Background thread that periodically checks sensor health every 5 seconds
    """
    while True:
        try:
            check_sensor_health()
            time.sleep(5)  # Check every 5 seconds
        except Exception as e:
            print(f"❌ Sensor monitor error: {e}")
            time.sleep(5)

def physics_loop():
    # Note: obstacle_state is now in car_state, not a local variable
    stop_timer = 0           # Timer for 1-second stop (50 cycles = 1 second at 20ms)
    steering_timer = 0       # Timer for steering duration
    last_left_obstacle = False
    last_right_obstacle = False
    target_steer_angle = 0   # Target angle during steering phase
    was_autonomous = False   # Track autonomous → manual transition
    
    while True:
        # --- CHECK IR SENSORS ---
        # IR sensors are active LOW: 0 = obstacle detected, 1 = no obstacle
        try:
            left_obstacle = not GPIO.input(LEFT_IR)   # Invert so True = obstacle
            right_obstacle = not GPIO.input(RIGHT_IR) # Invert so True = obstacle
        except:
            left_obstacle = False
            right_obstacle = False
        
        car_state["left_obstacle"] = left_obstacle
        car_state["right_obstacle"] = right_obstacle
        
        # --- CHECK SONAR SENSOR (Moving-Average Filtered) ---
        if car_state["sonar_enabled"]:
            sonar_distance = get_smoothed_sonar()
        else:
            sonar_distance = 100  # Default safe distance when disabled
        
        car_state["sonar_distance"] = round(sonar_distance, 1)
        
        # Debug: Print only on state change (less spam)
        if left_obstacle and not last_left_obstacle:
            print("⚠️  LEFT OBSTACLE DETECTED!")
        if right_obstacle and not last_right_obstacle:
            print("⚠️  RIGHT OBSTACLE DETECTED!")
        if not left_obstacle and last_left_obstacle:
            print("✅ LEFT CLEAR")
        if not right_obstacle and last_right_obstacle:
            print("✅ RIGHT CLEAR")
        
        # Sonar distance warnings (only print when crossing thresholds)
        if sonar_distance < SONAR_STOP_DISTANCE:
            print(f"🚨 SONAR: {sonar_distance}cm - EMERGENCY STOP!")
        elif sonar_distance < SONAR_CRAWL_DISTANCE:
            print(f"🐌 SONAR: {sonar_distance}cm - CRAWL SPEED")
        elif sonar_distance < SONAR_SLOW_DISTANCE:
            print(f"⚠️  SONAR: {sonar_distance}cm - SLOWING DOWN")
        elif sonar_distance < SONAR_CAUTION_DISTANCE:
            print(f"📍 SONAR: {sonar_distance}cm - CAUTION")
        
        last_left_obstacle = left_obstacle
        last_right_obstacle = right_obstacle
        
        gear = car_state["gear"]
        gas = car_state["gas_pressed"]
        brake = car_state["brake_pressed"]
        current = car_state["current_pwm"]
        user_angle = car_state["user_steer_angle"]
        obstacle_state = car_state["obstacle_state"]
        
        # --- OBSTACLE AVOIDANCE STATE MACHINE (Manual Mode Only) ---
        # When autonomous mode is active, drive_autonomous() owns all obstacle logic.
        # physics_loop only handles sensor reads, smooth ramping, steering mixer, and GPIO.
        # 
        # HOWEVER: Emergency Safety Override - even in autonomous mode, if sensors detect
        # imminent collision, immediately cut power and brake to prevent damage.
        emergency_stop_distance = 8   # Emergency stop if sonar < 8cm (immediate danger)
        emergency_ir_override = car_state["ir_enabled"] and (left_obstacle or right_obstacle)
        emergency_sonar_override = car_state["sonar_enabled"] and sonar_distance < emergency_stop_distance
        
        if emergency_ir_override or emergency_sonar_override:
            # EMERGENCY OVERRIDE: Immediate stop regardless of mode
            print(f"🚨 EMERGENCY OVERRIDE! IR: {emergency_ir_override}, Sonar: {sonar_distance:.1f}cm - CUTTING POWER")
            car_state["current_pwm"] = 0
            car_state["is_braking"] = True
            car_state["gas_pressed"] = False
            car_state["brake_pressed"] = True
            # In autonomous mode, AutoPilot owns its own FSM —
            # the emergency brake GPIO is applied below by the motor-output
            # section; AutoPilot will detect the obstacle on its next tick.
        elif not car_state["autonomous_mode"]:
            # Enhanced: Combine IR sensors with sonar for better obstacle detection
            ir_obstacle_detected = car_state["ir_enabled"] and (left_obstacle or right_obstacle)
            sonar_obstacle_detected = car_state["sonar_enabled"] and sonar_distance < SONAR_STOP_DISTANCE
            sonar_too_close = car_state["sonar_enabled"] and sonar_distance < SONAR_CRAWL_DISTANCE
            any_obstacle = ir_obstacle_detected or sonar_obstacle_detected
            
            if any_obstacle and gas:
                if obstacle_state == "IDLE":
                    # Trigger: Obstacle detected, enter appropriate avoidance state
                    if sonar_too_close:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                        print(f"🔄 TOO CLOSE ({sonar_distance}cm) - REVERSING FIRST")
                    else:
                        obstacle_state = "STEERING"
                        car_state["obstacle_state"] = "STEERING"
                        
                        if sonar_obstacle_detected and not ir_obstacle_detected:
                            if left_obstacle:
                                target_steer_angle = 60
                                print("🚨 SONAR OBSTACLE AHEAD + LEFT IR - GENTLE RIGHT")
                            elif right_obstacle:
                                target_steer_angle = -60
                                print("🚨 SONAR OBSTACLE AHEAD + RIGHT IR - GENTLE LEFT")
                            else:
                                target_steer_angle = 60
                                print("🚨 SONAR OBSTACLE AHEAD - GENTLE RIGHT")
                        elif left_obstacle and right_obstacle:
                            obstacle_state = "REVERSING"
                            car_state["obstacle_state"] = "REVERSING"
                            target_steer_angle = 0
                            car_state["dual_obstacle_escape"] = True
                            print("🚨 BOTH IR OBSTACLES - REVERSING FIRST")
                        elif left_obstacle:
                            target_steer_angle = 60
                            print("🚨 LEFT IR OBSTACLE - GENTLE RIGHT 60°")
                        else:
                            target_steer_angle = -60
                            print("🚨 RIGHT IR OBSTACLE - GENTLE LEFT 60°")
                
                elif obstacle_state == "REVERSING":
                    car_state["steer_angle"] = 0
                    steering_timer += 1
                    
                    if steering_timer > 25:
                        steering_timer = 0
                        
                        if car_state.get("dual_obstacle_escape", False):
                            car_state["dual_obstacle_escape"] = False
                            if left_obstacle and not right_obstacle:
                                target_steer_angle = 60
                                print("🔄 DUAL ESCAPE: Left cleared - TURNING RIGHT")
                            elif right_obstacle and not left_obstacle:
                                target_steer_angle = -60
                                print("🔄 DUAL ESCAPE: Right cleared - TURNING LEFT")
                            else:
                                import random
                                target_steer_angle = 60 if random.random() > 0.5 else -60
                                direction = "RIGHT" if target_steer_angle > 0 else "LEFT"
                                print(f"🔄 DUAL ESCAPE: Both still present - RANDOM {direction}")
                        else:
                            if left_obstacle:
                                target_steer_angle = 60
                                print("🔄 REVERSE COMPLETE - NOW STEERING RIGHT")
                            elif right_obstacle:
                                target_steer_angle = -60
                                print("🔄 REVERSE COMPLETE - NOW STEERING LEFT")
                            else:
                                target_steer_angle = 60
                                print("🔄 REVERSE COMPLETE - NOW STEERING RIGHT")
                        
                        obstacle_state = "STEERING"
                        car_state["obstacle_state"] = "STEERING"
                
                elif obstacle_state == "STEERING":
                    car_state["steer_angle"] = target_steer_angle
                    steering_timer += 1
                    
                    if not any_obstacle:
                        obstacle_state = "IDLE"
                        car_state["obstacle_state"] = "IDLE"
                        car_state["steer_angle"] = user_angle
                        print("✅ ALL OBSTACLES CLEARED - RESUMING NORMAL CONTROL")
                    elif left_obstacle and right_obstacle and steering_timer > 75:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                        car_state["dual_obstacle_escape"] = True
                        steering_timer = 0
                        print("🔄 BOTH OBSTACLES PERSIST - REVERSING AGAIN")
                    elif steering_timer > 100:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                        steering_timer = 0
                        print("⏰ STEERING TIMEOUT - TRYING REVERSE")
            else:
                # No obstacles detected
                if obstacle_state != "IDLE":
                    obstacle_state = "IDLE"
                    car_state["obstacle_state"] = "IDLE"
                car_state["steer_angle"] = user_angle
        else:
            # Autonomous mode active — AutoPilot drives motors directly
            # via CarSystem; physics_loop only reads sensors for telemetry.
            obstacle_state = "IDLE"
            car_state["obstacle_state"] = "IDLE"
        
        # Get the current steering angle (either from avoidance or user input)
        angle = car_state["steer_angle"]
        
        # --- NORMAL THROTTLE PHYSICS (when not in obstacle avoidance) ---
        if obstacle_state == "IDLE":
            # Check if autonomous mode is active
            if car_state["autonomous_mode"]:
                # Even in autopilot, the emergency brake must override everything
                if car_state["emergency_brake_active"]:
                    car_state["current_pwm"] = 0
                    car_state["is_braking"] = True
                    car_state["gear"] = "N"
                    car_system.stop()  # Immediately cut motors
                else:
                    # AutoPilot controls motors directly via CarSystem.
                    # We still need to update car_state["current_pwm"] for
                    # telemetry, but we do NOT write to GPIO here.
                    car_state["current_pwm"] = car_system._current_speed
                    car_state["is_braking"] = False
                # Skip the rest of throttle physics — fall through to
                # the motor-output section which will also be skipped.
            else:
                # Manual mode - existing logic
                # Check if emergency brake is active - if so, apply brakes and set to Neutral
                if car_state["emergency_brake_active"]:
                    current = 0
                    car_state["current_pwm"] = 0
                    car_state["is_braking"] = True  # Apply brake signals
                    car_state["gear"] = "N"  # Set gear to Neutral
                # BRAKE PEDAL PRESSED - Real car braking behavior
                elif brake:
                    # When brake is pressed, FORCE speed to 0 immediately and hold it there
                    # Aggressive braking - reduce speed rapidly until completely stopped
                    if current > 0.5:  # Still moving
                        current -= BRAKE_RATE * 5  # Even faster braking
                    else:
                        current = 0  # Force to exactly 0 when almost stopped
                        
                    current = max(0, current)  # Ensure zero cannot go negative
                    car_state["current_pwm"] = 0  # Always set to 0 when braking
                    car_state["is_braking"] = True  # Apply magnetic braking force to hold
                else:
                    # Brakes released - normal gas/coast logic
                    car_state["is_braking"] = False
                    # Check if speed limit is being used instead of gear
                    if car_state["speed_limit_enabled"]:
                        # Use speed limit (5-100%)
                        target = car_state["speed_limit"] if (gas and gear != "N") else 0
                    else:
                        # Use gear-based speeds (original behavior)
                        ranges = {"N": (0,0), "R": (0,80), "1": (0,40), "2": (40,60), "3": (60,80), "S": (80,100)}
                        min_s, max_s = ranges.get(gear, (0,40))
                        target = max_s if (gas and gear != "N") else 0
                        if gas and current < min_s: current = min_s
                    
                    # --- SONAR-BASED SPEED REDUCTION ---
                    if car_state["sonar_enabled"] and target > 0:
                        if sonar_distance < SONAR_STOP_DISTANCE:
                            # Emergency stop - too close!
                            target = 0
                            print(f"🚨 SONAR EMERGENCY STOP: {sonar_distance}cm")
                        elif sonar_distance < SONAR_CRAWL_DISTANCE:
                            # Very close - crawl speed (5% max)
                            target = min(target, 5)
                            print(f"🐌 SONAR CRAWLING: {sonar_distance}cm -> {target}%")
                        elif sonar_distance < SONAR_SLOW_DISTANCE:
                            # Close - very slow movement (15% max)
                            target = min(target, 15)
                            print(f"⚠️  SONAR SLOWING: {sonar_distance}cm -> {target}%")
                        elif sonar_distance < SONAR_CAUTION_DISTANCE:
                            # Getting close - moderate speed (40% max)
                            target = min(target, 40)
                            print(f"📍 SONAR CAUTION: {sonar_distance}cm -> {target}%")
                    
                    # Smooth Ramping
                    if current < target: current += ACCEL_RATE
                    elif current > target: current -= COAST_RATE
                    current = max(0, current)  # Ensure speed never goes negative
                    
                    car_state["current_pwm"] = current
        
        # --- OBSTACLE AVOIDANCE PHYSICS ---
        elif obstacle_state == "REVERSING":
            # Reverse with configurable power to overcome inertia and move away from obstacle
            current = REVERSE_POWER  # Use configurable reverse power
            car_state["current_pwm"] = current
            car_state["gear"] = "R"  # Force reverse gear
            print(f"🔄 REVERSING: {current}% speed (configurable power)")
        
        elif obstacle_state == "STEERING":
            # Use sonar-reduced speeds during steering
            if car_state["speed_limit_enabled"]:
                target = car_state["speed_limit"] if gas else 0
            else:
                ranges = {"N": (0,0), "R": (0,80), "1": (0,40), "2": (40,60), "3": (60,80), "S": (80,100)}
                min_s, max_s = ranges.get(gear, (0,40))
                target = max_s if gas else 0
                if gas and current < min_s: current = min_s
            
            # Apply sonar speed reduction even during steering
            if car_state["sonar_enabled"] and target > 0:
                if sonar_distance < SONAR_STOP_DISTANCE:
                    target = 0
                elif sonar_distance < SONAR_CRAWL_DISTANCE:
                    target = min(target, 5)
                elif sonar_distance < SONAR_SLOW_DISTANCE:
                    target = min(target, 15)
                elif sonar_distance < SONAR_CAUTION_DISTANCE:
                    target = min(target, 40)
            
            # Smooth Ramping
            if current < target: current += ACCEL_RATE
            elif current > target: current -= COAST_RATE
            current = max(0, current)
            
            car_state["current_pwm"] = current

        # --- 3. STEERING MIXER (The Magic) ---
        # In autonomous mode, CarSystem handles GPIO directly — skip mixer.
        if car_state["autonomous_mode"]:
            was_autonomous = True
            time.sleep(0.02)
            continue

        # ── Autonomous → Manual transition guard ──────────────────────
        # When we just left autonomous mode, current_pwm may still hold
        # the autopilot's last speed (mirrored above).  Force it to zero
        # so physics_loop doesn't coast the car at the old speed.
        if was_autonomous:
            current = 0
            car_state["current_pwm"] = 0
            car_state["is_braking"] = False
            was_autonomous = False

        # We calculate separate speeds for Left and Right motors
        left_motor_speed = current
        right_motor_speed = current

        # Calculate "Turn Factor" (0.0 to 1.0) based on angle
        # 90 degrees = 1.0 (Full turn), 0 degrees = 0.0 (Straight)
        turn_factor = abs(angle) / 90.0
        
        # ARC TURN LOGIC:
        if angle < -5: # TURNING LEFT
            # Left motor gets slower. At max turn, it drops to 10% speed
            left_motor_speed = current * (1.0 - (turn_factor * 0.9))
            right_motor_speed = current # Outer wheel stays fast
            
        elif angle > 5: # TURNING RIGHT
            # Right motor gets slower
            right_motor_speed = current * (1.0 - (turn_factor * 0.9))
            left_motor_speed = current # Outer wheel stays fast

        # --- 4. APPLY TO MOTORS ---
        try:
            # If normal brake is active, apply maximum magnetic braking force to lock the car
            if car_state["is_braking"]:
                # HARD BRAKE: Set both motor pins HIGH (creates short circuit for max resistance)
                # This prevents the car from moving even under external force
                GPIO.output(IN1, True); GPIO.output(IN2, True)
                GPIO.output(IN3, True); GPIO.output(IN4, True)
                
                # Keep PWM at 100% for absolute maximum holding force
                pwm_a.ChangeDutyCycle(100)
                pwm_b.ChangeDutyCycle(100)
            else:
                # Normal operation: apply calculated motor speeds
                pwm_a.ChangeDutyCycle(int(left_motor_speed))
                pwm_b.ChangeDutyCycle(int(right_motor_speed))
                
                # Direction Logic
                l_a, l_b = False, True # Default Forward
                r_a, r_b = False, True # Default Forward

                if gear == "R": # Reverse Logic
                    l_a, l_b = True, False
                    r_a, r_b = True, False
                    # Debug output for reverse gear
                    if obstacle_state == "REVERSING":
                        print(f"🔧 REVERSE MOTORS: L={int(left_motor_speed)}% R={int(right_motor_speed)}% IN1={l_a} IN2={l_b} IN3={r_a} IN4={r_b}")

                # NOTE: We removed the "Spin" override. 
                # Now we only steer by changing speed, not direction.
                
                GPIO.output(IN1, l_a); GPIO.output(IN2, l_b)
                GPIO.output(IN3, r_a); GPIO.output(IN4, r_b)
        except Exception as e:
            pass  # GPIO not available, skip

        time.sleep(0.02)  # 20ms loop = 50Hz (2.5x faster reaction than 50ms)

# ==========================================
# 🤖 SMART DRIVER AUTONOMOUS DRIVING
# ==========================================

def drive_autonomous():
    """
    Thin wrapper that ticks the AutoPilot FSM at 20 Hz.
    All navigation logic lives in autopilot.py.

    IMPORTANT – only the SocketIO / HTTP handlers call autopilot.start().
    This thread only *ticks* the FSM and handles emergency-brake pausing.
    Previously an unconditional ``if not is_active: start()`` here caused
    a race on disable: the handler called stop() but this thread restarted
    the autopilot before autonomous_mode was set to False, producing a
    ~0.5 s forward surge.
    """
    emergency_stopped = False        # track if WE paused the autopilot

    while True:
        if not car_state["autonomous_mode"]:
            if autopilot.is_active:
                autopilot.stop()
            emergency_stopped = False
            time.sleep(0.05)
            continue

        # Emergency brake overrides autopilot — pause the FSM and stop motors
        if car_state["emergency_brake_active"]:
            if autopilot.is_active:
                autopilot.stop()  # Cuts motors immediately
                emergency_stopped = True
            time.sleep(0.05)
            continue

        # Resume ONLY after emergency brake release (not after a mode-disable stop)
        if not autopilot.is_active:
            if emergency_stopped:
                autopilot.start()
                emergency_stopped = False
            else:
                # Autopilot was stopped by the disable handler; don't restart.
                time.sleep(0.05)
                continue

        autopilot.update()

        # Mirror state into car_state for telemetry & UI
        car_state["autonomous_state"] = autopilot.state.value
        car_state["last_obstacle_side"] = autopilot.turn_direction or "none"

        time.sleep(0.05)  # 20 Hz

# Start the Engine (Thread)
engine_thread = threading.Thread(target=physics_loop, daemon=True)
engine_thread.start()

# Start the Smart Driver Autonomous Thread
autonomous_thread = threading.Thread(target=drive_autonomous, daemon=True)
autonomous_thread.start()
print("🤖 Smart Driver autonomous system initialized")

# Start the Sensor Monitor Thread
sensor_monitor_thread = threading.Thread(target=sensor_monitor, daemon=True)
sensor_monitor_thread.start()
print("🚨 Sensor health monitor initialized")

# ==========================================
# 🌐 WEB SERVER (Flask + SocketIO)
# ==========================================
app = Flask(__name__, 
            static_folder=os.path.join(DIST_DIR, 'assets'), 
            static_url_path='/assets',
            template_folder=DIST_DIR)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Helper function to get local network IP
def get_local_ip():
    """Get the Raspberry Pi's local network IP address"""
    try:
        # Create a socket to determine which IP is used to reach external networks
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Google DNS, no internet connection needed
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        # Fallback: try to get hostname IP
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"

@app.route("/")
def index():
    return render_template('index.html')

# API endpoint for getting server IP (used by mobile/remote clients)
@app.route("/api/server-ip")
def api_server_ip():
    """Return the server's network IP address for remote connections"""
    return jsonify({"ip": get_local_ip(), "port": 5000})

# Catch-all for React files (vite.svg, etc.)
@app.route('/<path:filename>')
def serve_root_files(filename):
    return send_from_directory(DIST_DIR, filename)

# Print server info on startup
print(f"🌐 Server IP Address: {get_local_ip()}")
print(f"📱 Mobile devices can connect via: http://{get_local_ip()}:5000")

# --- CONTROLS ---

@app.route("/gear/<gear>")
def set_gear(gear):
    """Sets the Gear (R, N, 1, 2, 3, S)"""
    if gear in ["R", "N", "1", "2", "3", "S"]:
        car_state["gear"] = gear
        return f"GEAR_{gear}"
    return "INVALID"

@app.route("/steer/<angle>")
def steer(angle):
    try:
        angle = int(angle)
        # Store user's steering input (preserved when avoiding obstacles)
        car_state["user_steer_angle"] = angle
        # If angle is big enough, mark as turning
        if abs(angle) > 10:
            car_state["turning"] = "left" if angle < 0 else "right"
        else:
            car_state["turning"] = "straight"
        
        # Log only when steering is commanded
        turn_factor = abs(angle) / 90.0
        print(f"🎯 Steering: {angle:+d}° | Turn Factor: {turn_factor:.2f} | Direction: {car_state['turning'].upper()}")
        
        return "OK"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit/<speed>")
def set_speed_limit(speed):
    """Sets the manual speed limit (5-100%)"""
    try:
        speed = int(speed)
        if 5 <= speed <= 100:
            car_state["speed_limit"] = speed
            car_state["speed_limit_enabled"] = True  # Enable speed limit mode
            print(f"⚡ Speed Limit: {speed}% | Mode: SPEED LIMIT (Gear ignored)")
            return f"SPEED_LIMIT_{speed}"
        else:
            return "INVALID (must be 5-100)"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit_enable/<speed>")
def enable_speed_limit(speed):
    """Enables speed limit mode with specified speed"""
    try:
        speed = int(speed)
        if 5 <= speed <= 100:
            car_state["speed_limit"] = speed
            car_state["speed_limit_enabled"] = True
            print(f"✅ Speed Limit ENABLED: {speed}% | Mode: SPEED LIMIT (Gear ignored)")
            return f"SPEED_LIMIT_ENABLED_{speed}"
        else:
            return "INVALID (must be 5-100)"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit_disable")
def disable_speed_limit():
    """Disables speed limit mode, reverts to gear-based speeds"""
    car_state["speed_limit_enabled"] = False
    car_state["speed_limit"] = 100  # Reset to max
    print(f"❌ Speed Limit DISABLED | Mode: GEAR (Normal gear-based speeds)")
    return "SPEED_LIMIT_DISABLED"

@app.route("/<action>")
def handle_action(action):
    """Handles Press/Release events from the UI"""
    
    # THROTTLE
    if action == 'forward':
        car_state["gas_pressed"] = True
        car_state["brake_pressed"] = False
        car_state["direction"] = "forward"
        car_state["turning"] = "straight"
    
    # COAST (Gas Released)
    elif action == 'stop':
        car_state["gas_pressed"] = False
        car_state["brake_pressed"] = False
        # Note: We keep direction as-is so it rolls naturally
    
    # BRAKE PEDAL
    elif action == 'brake' or action == 'backward':
        car_state["gas_pressed"] = False
        car_state["brake_pressed"] = True
    
    # STEERING
    elif action == 'left':
        car_state["gas_pressed"] = True
        car_state["turning"] = "left"
    elif action == 'right':
        car_state["gas_pressed"] = True
        car_state["turning"] = "right"

    return "OK"

@app.route("/emergency_stop")
def emergency_stop():
    """Toggle emergency brake - when ON, apply brakes and set gear to Neutral"""
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    
    # Always cut power immediately
    car_state["current_pwm"] = 0
    
    # Reset obstacle avoidance state to prevent stuck state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Note: is_braking flag is managed by the physics loop based on emergency_brake_active
    # Don't touch brake_pressed - it should only reflect actual brake pedal state
    # This prevents state desync between UI and server
    
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"🚨 EMERGENCY BRAKE: {state}")
    return "EMERGENCY_BRAKE"

@app.route("/autonomous_enable")
def enable_autonomous():
    """Enable Smart Driver autonomous mode"""
    car_state["autonomous_mode"] = True
    car_state["autonomous_state"] = State.CRUISING.value
    car_state["last_obstacle_side"] = "none"
    # Force IR and Sonar sensors enabled in autonomous mode for safety
    car_state["ir_enabled"] = True
    car_state["sonar_enabled"] = True
    autopilot.start()
    print(f"🤖 SMART DRIVER: ENABLED - Autonomous driving active")
    print(f"📡 SAFETY: IR and Sonar sensors force-enabled in autonomous mode")
    return "AUTONOMOUS_ENABLED"

@app.route("/autonomous_disable")
def disable_autonomous():
    """Disable Smart Driver autonomous mode and return to manual control"""
    # ── FIX: set mode flag FIRST so drive_autonomous cannot restart
    car_state["autonomous_mode"] = False
    # Now stop hardware
    autopilot.stop()
    car_system.stop()
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    car_state["gear"] = "N"
    car_state["steer_angle"] = 0
    car_state["user_steer_angle"] = 0
    print(f"🤖 SMART DRIVER: DISABLED - Returned to manual control")
    return "AUTONOMOUS_DISABLED"

@app.route("/autonomous_toggle")
def toggle_autonomous():
    """Toggle Smart Driver autonomous mode on/off"""
    if car_state["autonomous_mode"]:
        return disable_autonomous()
    else:
        return enable_autonomous()

# --- SYSTEM MONITORING FUNCTIONS ---

def get_wifi_status():
    """Get WiFi connection status and SSID"""
    try:
        result = subprocess.run(['iwconfig', 'wlan0'], capture_output=True, text=True, timeout=2)
        output = result.stdout
        
        # Check if connected
        if 'Not-Associated' in output or 'off/any' in output:
            return False, 'Disconnected'
        
        # Extract SSID
        ssid_match = re.search(r'ESSID:"([^"]*)"', output)
        if ssid_match:
            ssid = ssid_match.group(1)
            return True, ssid if ssid else 'Connected'
        
        return True, 'Connected'
    except Exception as e:
        print(f"⚠️ WiFi check failed: {e}")
        return False, 'Unknown'

def get_hotspot_status():
    """Check if hotspot is active"""
    try:
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True, timeout=2)
        # Check for dnsmasq or hostapd which indicate hotspot is running
        is_active = 'dnsmasq' in result.stdout or 'hostapd' in result.stdout
        return is_active
    except Exception as e:
        print(f"⚠️ Hotspot check failed: {e}")
        return False

def get_cpu_temperature():
    """Get CPU temperature in Celsius"""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = int(f.read().strip()) / 1000.0
            return round(temp, 1)
    except Exception as e:
        print(f"⚠️ Temperature read failed: {e}")
        return 45.0

def get_cpu_clock():
    """Get CPU clock speed in MHz"""
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 'r') as f:
            freq_khz = int(f.read().strip())
            freq_mhz = freq_khz / 1000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Fallback: Try alternative sysfs path
    try:
        with open('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq', 'r') as f:
            freq_khz = int(f.read().strip())
            freq_mhz = freq_khz / 1000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Final fallback: return a reasonable default
    return 1200

def get_gpu_clock():
    """Get GPU clock speed in MHz"""
    try:
        result = subprocess.run(['vcgencmd', 'measure_clock', 'gpu'], 
                              capture_output=True, text=True, timeout=2)
        # Output format: frequency(45)=XXXXX
        match = re.search(r'=(\d+)', result.stdout)
        if match:
            freq_hz = int(match.group(1))
            freq_mhz = freq_hz / 1_000_000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception as e:
        pass
    
    # Fallback: Try reading from sysfs
    try:
        with open('/sys/kernel/debug/clk/vec/clk_rate', 'r') as f:
            freq_hz = int(f.read().strip())
            freq_mhz = freq_hz / 1_000_000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Final fallback: return a reasonable default for Raspberry Pi
    return 400

# --- TELEMETRY ---

# ==========================================
# 🌐 WEBSOCKET EVENT HANDLERS
# ==========================================

@socketio.on('connect')
def on_connect():
    """Handle client connection"""
    global has_had_client
    print(f"\n🔗 [Socket] ✅ CLIENT CONNECTED")
    print(f"   Client ID: {request.sid}")
    print(f"   Remote Address: {request.remote_addr}")
    # Initialize ping tracking for this client
    has_had_client = True
    client_last_pong[request.sid] = time.time()
    car_state["heartbeat_active"] = True
    # Ensure emergency brakes are OFF when client connects (client is now responsible for safety)
    car_state["emergency_brake_active"] = False
    print(f"   Emergency brakes reset to OFF")
    emit('connection_response', {'data': 'Connected to RC Car'})
    # Send current tuning state so UI stays in sync after refresh
    emit('tuning_sync', {
        'tuning': _active_tuning,
        'defaults': _AUTOPILOT_DEFAULT_TUNING,
    })

@socketio.on('disconnect')
def on_disconnect():
    """Handle client disconnection"""
    global has_had_client
    print(f"\n🔓 [Socket] ❌ CLIENT DISCONNECTED")
    print(f"   Client ID: {request.sid}")
    # Clean up ping tracking for this client
    if request.sid in client_last_pong:
        del client_last_pong[request.sid]
    # If all clients disconnected AND we've had clients before, activate emergency brakes
    if not client_last_pong and has_had_client:
        car_state["emergency_brake_active"] = True
        car_state["heartbeat_active"] = False
        car_state["current_pwm"] = 0
        print(f"🚨 [Heartbeat] ❌ ALL CLIENTS DISCONNECTED - EMERGENCY BRAKES ACTIVATED!")


@socketio.on('heartbeat_pong')
def on_heartbeat_pong(data):
    """Handle heartbeat_pong response from client (heartbeat check)"""
    client_id = request.sid
    client_last_pong[client_id] = time.time()
    # Signal that heartbeat is active
    car_state["heartbeat_active"] = True
    # Debug: Only print occasionally to avoid spam
    # print(f"💗 [Heartbeat] ✅ Heartbeat pong received from client {client_id}")


@socketio.on('throttle')
def on_throttle(data):
    """Handle throttle control"""
    value = data.get('value', False)
    if value:
        car_state["gas_pressed"] = True
        car_state["brake_pressed"] = False
        car_state["direction"] = "forward"
        print(f"\n⚙️ [UI Control] 🚀 THROTTLE PRESSED (Forward)")
    else:
        car_state["gas_pressed"] = False
        print(f"\n⚙️ [UI Control] ⏸️ THROTTLE RELEASED")
    emit('throttle_response', {'status': 'ok', 'value': value})

@socketio.on('brake')
def on_brake(data):
    """Handle brake control"""
    value = data.get('value', False)
    car_state["brake_pressed"] = value
    if value:
        car_state["gas_pressed"] = False
        print(f"\n⚙️ [UI Control] 🛑 BRAKE ENGAGED")
    else:
        # When brake is released, restore gas if auto-accel is still enabled
        if car_state["auto_accel_enabled"]:
            car_state["gas_pressed"] = True
        print(f"\n⚙️ [UI Control] 🩹 BRAKE RELEASED")
    emit('brake_response', {'status': 'ok', 'value': value})

@socketio.on('steering')
def on_steering(data):
    """Handle steering control (angle -90 to +90)"""
    try:
        angle = int(data.get('angle', 0))
        angle = max(-90, min(90, angle))  # Clamp to -90 to 90
        car_state["user_steer_angle"] = angle
        if abs(angle) > 10:
            car_state["turning"] = "left" if angle < 0 else "right"
            direction = "⬅️ LEFT" if angle < 0 else "➡️ RIGHT"
            print(f"\n⚙️ [UI Control] 🎡 STEERING: {direction} ({abs(angle)}°)")
        else:
            car_state["turning"] = "straight"
            print(f"\n⚙️ [UI Control] 🎡 STEERING: ⬆️ CENTER (0°)")
        emit('steering_response', {'status': 'ok', 'angle': angle})
    except (ValueError, TypeError) as e:
        print(f"\n⚠️ [UI Control] ❌ Invalid steering angle: {e}")
        emit('steering_response', {'status': 'error', 'message': 'Invalid angle'})

@socketio.on('gear_change')
def on_gear_change(data):
    """Handle gear selection"""
    gear = data.get('gear', 'N').upper()
    if gear in ["R", "N", "1", "2", "3", "S"]:
        car_state["gear"] = gear
        gear_names = {'R': '🔙 REVERSE', 'N': '⏸️ NEUTRAL', '1': '1️⃣ 1st', '2': '2️⃣ 2nd', '3': '3️⃣ 3rd', 'S': '⚡ SPORT'}
        print(f"\n⚙️ [UI Control] 🔧 GEAR: {gear_names.get(gear, gear)}")
        emit('gear_response', {'status': 'ok', 'gear': gear})
    else:
        print(f"\n⚠️ [UI Control] ❌ Invalid gear: {gear}")
        emit('gear_response', {'status': 'error', 'message': 'Invalid gear'})

@socketio.on('emergency_stop')
def on_emergency_stop(data):
    """Handle emergency brake toggle - when ON, apply brakes and set gear to Neutral"""
    # Toggle the emergency brake state
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    
    # Always cut power immediately
    car_state["current_pwm"] = 0
    
    # If in autopilot mode and e-brake activated, stop motors immediately
    if car_state["emergency_brake_active"] and car_state["autonomous_mode"]:
        car_system.stop()
        if autopilot.is_active:
            autopilot.stop()
    
    # Reset obstacle avoidance state to prevent stuck state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Note: is_braking flag is managed by the physics loop based on emergency_brake_active
    # Don't touch brake_pressed - it should only reflect actual brake pedal state
    # This prevents state desync between UI and server
    
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"\n⚙️ [UI Control] 🚨 EMERGENCY BRAKE: {state}")
    emit('emergency_stop_response', {'status': 'ok', 'emergency_brake_active': car_state["emergency_brake_active"]})

@socketio.on('emergency_stop_release')
def on_emergency_stop_release(data):
    """Handle emergency brake release - explicitly set brake to OFF"""
    car_state["emergency_brake_active"] = False
    
    # Reset obstacle avoidance state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    print(f"\n⚙️ [UI Control] 🚨 EMERGENCY BRAKE: 🟢 RELEASED (explicit release)")
    emit('emergency_stop_response', {'status': 'ok', 'emergency_brake_active': False})

@socketio.on('auto_accel_enable')
def on_auto_accel_enable(data):
    """Handle auto-acceleration enable (server-side auto-throttle)"""
    car_state["auto_accel_enabled"] = True
    car_state["gas_pressed"] = True  # Actually engage the throttle
    print(f"\n⚙️ [UI Control] 🚀 AUTO-ACCEL: ENABLED (auto-throttle mode)")
    emit('auto_accel_response', {'status': 'ok', 'enabled': True})

@socketio.on('auto_accel_disable')
def on_auto_accel_disable(data):
    """Handle auto-acceleration disable"""
    car_state["auto_accel_enabled"] = False
    car_state["gas_pressed"] = False
    print(f"\n⚙️ [UI Control] 🚫 AUTO-ACCEL: DISABLED")
    emit('auto_accel_response', {'status': 'ok', 'enabled': False})

@socketio.on('ir_toggle')
def on_ir_toggle(data):
    """Handle IR sensor toggle"""
    # Block IR toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        print(f"\n⚙️ [UI Control] 📡 IR SENSORS: BLOCKED - Cannot toggle in autonomous mode")
        emit('ir_response', {'status': 'blocked', 'ir_enabled': car_state["ir_enabled"], 'message': 'Cannot toggle IR sensors in autonomous mode'})
        return
    
    car_state["ir_enabled"] = not car_state["ir_enabled"]
    state = '✅ ON' if car_state["ir_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📡 IR SENSORS: {state}")
    emit('ir_response', {'status': 'ok', 'ir_enabled': car_state["ir_enabled"]})

@socketio.on('sonar_toggle')
def on_sonar_toggle(data):
    """Handle sonar sensor toggle"""
    # Block Sonar toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        print(f"\n⚙️ [UI Control] 📡 SONAR SENSOR: BLOCKED - Cannot toggle in autonomous mode")
        emit('sonar_response', {'status': 'blocked', 'sonar_enabled': car_state["sonar_enabled"], 'message': 'Cannot toggle Sonar sensor in autonomous mode'})
        return
    
    car_state["sonar_enabled"] = not car_state["sonar_enabled"]
    state = '✅ ON' if car_state["sonar_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📡 SONAR SENSOR: {state}")
    emit('sonar_response', {'status': 'ok', 'sonar_enabled': car_state["sonar_enabled"]})

@socketio.on('autonomous_enable')
def on_autonomous_enable(data):
    """Handle Smart Driver autonomous mode enable"""
    car_state["autonomous_mode"] = True
    car_state["autonomous_state"] = State.CRUISING.value
    car_state["last_obstacle_side"] = "none"
    # Force IR and Sonar sensors enabled in autonomous mode for safety
    car_state["ir_enabled"] = True
    car_state["sonar_enabled"] = True
    autopilot.start()
    print(f"\n⚙️ [UI Control] 🤖 SMART DRIVER: ENABLED - Autonomous driving active")
    print(f"📡 SAFETY: IR and Sonar sensors force-enabled in autonomous mode")
    emit('autonomous_response', {'status': 'ok', 'autonomous_enabled': True})

@socketio.on('autonomous_disable')
def on_autonomous_disable(data):
    """Handle Smart Driver autonomous mode disable"""
    # ── FIX: set mode flag FIRST so drive_autonomous cannot restart
    # the autopilot between stop() and the flag change (race condition
    # that caused ~0.5 s forward surge on disable).
    car_state["autonomous_mode"] = False
    # Now stop hardware — drive_autonomous sees False and won't restart
    autopilot.stop()
    car_system.stop()
    # Reset to safe manual state
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    car_state["gear"] = "N"
    car_state["steer_angle"] = 0
    car_state["user_steer_angle"] = 0
    print(f"\n⚙️ [UI Control] 🤖 SMART DRIVER: DISABLED - Returned to manual control")
    emit('autonomous_response', {'status': 'ok', 'autonomous_enabled': False})

@socketio.on('autonomous_toggle')
def on_autonomous_toggle(data):
    """Handle Smart Driver autonomous mode toggle"""
    if car_state["autonomous_mode"]:
        on_autonomous_disable(data)
    else:
        on_autonomous_enable(data)

@socketio.on('autopilot_toggle')
def on_autopilot_toggle(data):
    """Alias for autonomous_toggle — frontend emits this event name"""
    on_autonomous_toggle(data)

@socketio.on('tuning_update')
def on_tuning_update(data):
    """Apply tuning constants from the UI to the running AutoPilot instance
    and persist in _active_tuning so new clients pick them up."""
    tuning = data.get('tuning', {})
    applied = []
    for key, value in tuning.items():
        attr = key.upper()
        if hasattr(autopilot, attr):
            try:
                cast_value = type(getattr(AutoPilot, attr))(value)  # cast to original type
                setattr(autopilot, attr, cast_value)
                _active_tuning[attr] = cast_value
                applied.append(attr)
            except Exception as e:
                print(f"⚠️ [Tuning] Failed to set {attr}={value}: {e}")
    # If SONAR_HISTORY_LEN changed, resize the deques
    if 'SONAR_HISTORY_LEN' in tuning:
        new_len = int(tuning['SONAR_HISTORY_LEN'])
        autopilot._sonar_history = deque(autopilot._sonar_history, maxlen=new_len)
        autopilot._rear_sonar_history = deque(autopilot._rear_sonar_history, maxlen=new_len)
    print(f"\n⚙️ [Tuning] Applied {len(applied)} constants from UI: {applied}")
    emit('tuning_response', {'status': 'ok', 'applied': applied})

@socketio.on('tuning_request')
def on_tuning_request(data):
    """Send current active tuning and defaults to the requesting client."""
    emit('tuning_sync', {
        'tuning': _active_tuning,
        'defaults': _AUTOPILOT_DEFAULT_TUNING,
    })

@socketio.on('state_request')
def on_state_request(data):
    """Send current car state to client"""
    emit('state_response', car_state)

# Heartbeat monitoring thread
def heartbeat_monitor():
    """Monitor client heartbeat - ping clients and check for pong responses"""
    last_ping_time = time.time()
    
    while True:
        try:
            current_time = time.time()
            
            # Send heartbeat_ping to all connected clients periodically
            if current_time - last_ping_time >= HEARTBEAT_INTERVAL:
                if client_last_pong:  # Only ping if clients are connected
                    # Need app context for socketio.emit in background thread
                    with app.app_context():
                        socketio.emit('heartbeat_ping', {})
                    print(f"💗 [Heartbeat] 🔔 Heartbeat ping sent to all clients")
                    last_ping_time = current_time
            
            # Check for dead clients (no pong response within timeout)
            dead_clients = []
            for client_id, last_pong_time in client_last_pong.items():
                if current_time - last_pong_time > HEARTBEAT_TIMEOUT:
                    dead_clients.append(client_id)
            
            # Handle dead clients
            if dead_clients:
                for client_id in dead_clients:
                    print(f"🚨 [Heartbeat] ❌ HEARTBEAT LOST from client {client_id}")
                    print(f"🚨 [Heartbeat] 🛑 ACTIVATING EMERGENCY BRAKES!")
                    car_state["emergency_brake_active"] = True
                    car_state["heartbeat_active"] = False
                    car_state["current_pwm"] = 0
                    car_state["gas_pressed"] = False
                    car_state["brake_pressed"] = False
                
                # Remove dead clients from tracking
                for client_id in dead_clients:
                    del client_last_pong[client_id]
            
            time.sleep(0.1)  # Check every 100ms
        except Exception as e:
            print(f"❌ Heartbeat monitor error: {e}")
            time.sleep(0.1)

# Start heartbeat monitor thread
heartbeat_thread = threading.Thread(target=heartbeat_monitor, daemon=True)
heartbeat_thread.start()
print("💗 [Heartbeat] ✅ Heartbeat monitor started (interval: {:.1f}s, timeout: {:.1f}s)".format(HEARTBEAT_INTERVAL, HEARTBEAT_TIMEOUT))

# Telemetry broadcast thread
def telemetry_broadcast():
    """Broadcast telemetry data to all connected clients at 20Hz"""
    while True:
        try:
            pwm = car_state["current_pwm"]
            fake_rpm = int(pwm * 2.2)
            fake_speed = round(pwm * 0.025, 1)
            
            telemetry_data = {
                "rpm": fake_rpm,
                "speed": fake_speed,
                "current_pwm": pwm,
                "gear": car_state["gear"],
                "steer_angle": car_state["steer_angle"],
                "direction": car_state["direction"],
                "turning": car_state["turning"],
                "left_obstacle": car_state["left_obstacle"],
                "right_obstacle": car_state["right_obstacle"],
                "gas_pressed": car_state["gas_pressed"],
                "brake_pressed": car_state["brake_pressed"],
                "ir_enabled": car_state["ir_enabled"],
                "heartbeat_active": car_state["heartbeat_active"],
                "emergency_brake_active": car_state["emergency_brake_active"],
                "temperature": get_cpu_temperature(),
                "cpu_clock": get_cpu_clock(),
                "gpu_clock": get_gpu_clock(),
                # 🤖 Autonomous driving telemetry
                "autonomous_mode": car_state["autonomous_mode"],
                "autonomous_state": car_state["autonomous_state"],
                "autonomous_target_speed": car_system._current_speed if car_state["autonomous_mode"] else 0,
                "sonar_distance": car_state["sonar_distance"],
                "sonar_enabled": car_state["sonar_enabled"],
                # 🚨 Sensor health status
                "sensor_status": car_state["sensor_status"],
                "service_light_active": car_state["service_light_active"]
            }
            
            socketio.emit('telemetry_update', telemetry_data)
            time.sleep(0.05)  # 20Hz = 50ms
        except Exception as e:
            print(f"❌ Telemetry broadcast error: {e}")
            time.sleep(0.05)

# Start telemetry broadcaster thread
telemetry_thread = threading.Thread(target=telemetry_broadcast, daemon=True)
telemetry_thread.start()

# ==========================================
# 🌐 HTTP ENDPOINTS (kept for backwards compatibility)
# ==========================================
@app.route("/telemetry")
def telemetry():
    """Returns current car state for the dashboard"""
    pwm = car_state["current_pwm"]
    
    # Math to fake RPM and KM/H based on power
    fake_rpm = int(pwm * 2.2)      # Max ~220 RPM
    fake_speed = round(pwm * 0.025, 1) # Max ~2.5 km/h
    
    return jsonify({
        "rpm": fake_rpm,
        "speed": fake_speed,
        "current_pwm": pwm,
        "gear": car_state["gear"],
        "steer_angle": car_state["steer_angle"],
        "direction": car_state["direction"],
        "turning": car_state["turning"],
        "left_obstacle": car_state["left_obstacle"],
        "right_obstacle": car_state["right_obstacle"],
        "temperature": get_cpu_temperature(),
        "cpu_clock": get_cpu_clock(),
        "gpu_clock": get_gpu_clock(),
        # 🤖 Autonomous driving telemetry
        "autonomous_mode": car_state["autonomous_mode"],
        "autonomous_state": car_state["autonomous_state"],
        "autonomous_target_speed": car_state["autonomous_target_speed"],
        "sonar_distance": car_state["sonar_distance"],
        "sonar_enabled": car_state["sonar_enabled"]
    })

@app.route("/system/status")
def system_status():
    """Returns system status: WiFi, hotspot, temperature, clock speeds"""
    wifi_connected, ssid = get_wifi_status()
    hotspot_active = get_hotspot_status()
    cpu_temp = get_cpu_temperature()
    cpu_clock = get_cpu_clock()
    gpu_clock = get_gpu_clock()
    
    return jsonify({
        "wifi_connected": wifi_connected,
        "ssid": ssid,
        "hotspot_active": hotspot_active,
        "cpu_temp": cpu_temp,
        "cpu_clock": cpu_clock,
        "gpu_clock": gpu_clock,
        "ir_enabled": car_state["ir_enabled"]
    })

@app.route("/ir/<state>")
def set_ir(state):
    """Toggle IR sensor on/off"""
    # Block IR toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        print("❌ IR SENSORS: BLOCKED - Cannot toggle in autonomous mode")
        return "BLOCKED_AUTONOMOUS_MODE"
    
    if state.lower() == "on":
        car_state["ir_enabled"] = True
        print("✅ IR Sensors ENABLED")
        return "IR_ON"
    elif state.lower() == "off":
        car_state["ir_enabled"] = False
        print("❌ IR Sensors DISABLED")
        return "IR_OFF"
    return "INVALID"

# --- SYSTEM SETTINGS ---

@app.route("/system/reboot")
def system_reboot():
    """Reboots the Raspberry Pi system"""
    print("🔄 SYSTEM REBOOT initiated!")
    try:
        subprocess.Popen(['sudo', 'reboot'])
        return "REBOOT_INITIATED"
    except Exception as e:
        print(f"❌ Reboot failed: {e}")
        return "REBOOT_FAILED"

@app.route("/system/configure_hotspot", methods=['POST'])
def configure_hotspot():
    """Configures hotspot mode with SSID, password, and IP"""
    try:
        data = request.get_json()
        ssid = data.get('ssid', 'RC-Car-Connect')
        password = data.get('password', '')
        ip = data.get('ip', '192.168.4.1')
        
        if not password:
            return "HOTSPOT_CONFIG_FAILED: Password required"
        
        print(f"📡 Configuring hotspot...")
        print(f"   SSID: {ssid}")
        print(f"   IP: {ip}")
        
        # Create a shell script to configure hotspot
        hotspot_script = f"""#!/bin/bash
set -e

echo "🔧 Setting up WiFi Hotspot..."

# Disable WiFi
sudo ifconfig wlan0 down 2>/dev/null || true

# Install hostapd and dnsmasq if not already installed
echo "📦 Installing hostapd and dnsmasq..."
sudo apt-get update -qq
sudo apt-get install -y -qq hostapd dnsmasq

# Configure hostapd
echo "⚙️ Configuring hostapd..."
sudo tee /etc/hostapd/hostapd.conf > /dev/null <<'HOSTAPD'
interface=wlan0
driver=nl80211
ssid={ssid}
hw_mode=g
channel=6
wmm_enabled=0
wpa=2
wpa_passphrase={password}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
HOSTAPD

# Configure dnsmasq
echo "⚙️ Configuring dnsmasq..."
sudo tee /etc/dnsmasq.conf > /dev/null <<'DNSMASQ'
interface=wlan0
dhcp-range={ip.rsplit('.', 1)[0]}.100,{ip.rsplit('.', 1)[0]}.200,12h
address=/#/{ip}
DNSMASQ

# Bring interface up and configure IP
echo "🔌 Bringing up wlan0..."
sudo ifconfig wlan0 up
sudo ifconfig wlan0 {ip} netmask 255.255.255.0

# Start services
echo "🚀 Starting hostapd and dnsmasq..."
sudo systemctl start hostapd
sudo systemctl start dnsmasq

# Enable IP forwarding
echo "🌉 Enabling IP forwarding..."
sudo sysctl -w net.ipv4.ip_forward=1 > /dev/null

echo "✅ Hotspot configured successfully!"
echo "   Connect to: {ssid}"
echo "   IP Address: {ip}"
"""
        
        # Write the script
        script_path = '/tmp/setup_hotspot.sh'
        with open(script_path, 'w') as f:
            f.write(hotspot_script)
        
        # Make it executable
        os.chmod(script_path, 0o755)
        
        # Run the script in background
        subprocess.Popen(['bash', script_path], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL)
        
        print("✅ Hotspot configuration initiated")
        return "HOTSPOT_CONFIG_INITIATED"
    except Exception as e:
        print(f"❌ Hotspot configuration failed: {e}")
        return f"HOTSPOT_CONFIG_FAILED: {str(e)}"

if __name__ == "__main__":
    try:
        # Host 0.0.0.0 makes it available on your Wi-Fi
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    finally:
        # Safety cleanup
        autopilot.stop()
        car_system.cleanup()
        try:
            pwm_a.stop()
            pwm_b.stop()
        except Exception:
            pass
        GPIO.cleanup()