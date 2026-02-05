import RPi.GPIO as GPIO
from flask import Flask, render_template, send_from_directory, jsonify
from flask_cors import CORS
import os
import subprocess
import sys
import time
import threading
import socket
import re

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================

# --- PROJECT PATHS ---
# Folder name where your Loveable/React code lives
PROJECT_DIR = "f1-race-control"
# The folder where the built website ends up
DIST_DIR = os.path.join(PROJECT_DIR, "dist")

# --- MOTOR PINS (BCM Numbering) ---
# Check your wiring! 
IN1 = 17  # Left Motor Direction A
IN2 = 27  # Left Motor Direction B
IN3 = 22  # Right Motor Direction A
IN4 = 23  # Right Motor Direction B
ENA = 18  # Left Motor Speed (PWM)
ENB = 19  # Right Motor Speed (PWM)

# --- PHYSICS TUNING ---
ACCEL_RATE = 2.0    # How snappy the car speeds up (Higher = Faster)
COAST_RATE = 0.5    # How slowly it stops when you let go of gas (Coasting)
BRAKE_RATE = 1.5    # Brake deceleration rate (stops from 100 in ~3-4 seconds)
FREQ = 1000         # PWM Frequency (Hz)

# ==========================================
# 🛠️ AUTO-BUILDER
# ==========================================
def check_and_build():
    """Checks if the React UI is built. If not, runs 'npm run build'."""
    if not os.path.exists(DIST_DIR):
        print("⚠️  UI build not found. Building now...")
        print("⏳ This might take 2-3 minutes on a Raspberry Pi...")
        try:
            subprocess.run(["npm", "run", "build"], cwd=PROJECT_DIR, check=True)
            print("✅ Build Complete!")
        except Exception as e:
            print(f"❌ Build failed: {e}")
            print("   Make sure you are in the 'rpi_car' folder and 'f1-race-control' exists.")
            sys.exit(1)
    else:
        print("✅ UI found. Starting engine...")

# Run the check immediately
check_and_build()

# ==========================================
# 🚗 HARDWARE SETUP
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup Pins
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

# Setup PWM
pwm_a = GPIO.PWM(ENA, FREQ)
pwm_b = GPIO.PWM(ENB, FREQ)
pwm_a.start(0)
pwm_b.start(0)

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
    "steer_angle": 0      # NEW: Defaults to 0 (Straight)
}

def physics_loop():
    while True:
        gear = car_state["gear"]
        gas = car_state["gas_pressed"]
        brake = car_state["brake_pressed"]
        current = car_state["current_pwm"]
        angle = car_state["steer_angle"] # -90 to 90

        # --- 1. BRAKING (Priority) ---
        if brake:
            # Brake: Quickly reduce speed to stop
            target = 0
            if current > target:
                current -= BRAKE_RATE
            if current < 0:
                current = 0
            car_state["current_pwm"] = current
        else:
            # --- 2. THROTTLE PHYSICS ---
            ranges = {"N": (0,0), "R": (0,80), "1": (0,40), "2": (40,60), "3": (60,80), "S": (80,100)}
            min_s, max_s = ranges.get(gear, (0,40))
            
            target = max_s if (gas and gear != "N") else 0
            if gas and current < min_s: current = min_s
            
            # Smooth Ramping
            if current < target: current += ACCEL_RATE
            elif current > target: current -= COAST_RATE
            if current < 0: current = 0
            
            car_state["current_pwm"] = current

        # --- 3. STEERING MIXER (The Magic) ---
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
        pwm_a.ChangeDutyCycle(int(left_motor_speed))
        pwm_b.ChangeDutyCycle(int(right_motor_speed))

        # Direction Logic
        l_a, l_b = False, True # Default Forward
        r_a, r_b = False, True # Default Forward

        if gear == "R": # Reverse Logic
            l_a, l_b = True, False
            r_a, r_b = True, False

        # NOTE: We removed the "Spin" override. 
        # Now we only steer by changing speed, not direction.
        
        GPIO.output(IN1, l_a); GPIO.output(IN2, l_b)
        GPIO.output(IN3, r_a); GPIO.output(IN4, r_b)

        time.sleep(0.05)

# Start the Engine (Thread)
engine_thread = threading.Thread(target=physics_loop, daemon=True)
engine_thread.start()

# ==========================================
# 🌐 WEB SERVER (Flask)
# ==========================================
app = Flask(__name__, 
            static_folder=os.path.join(DIST_DIR, 'assets'), 
            static_url_path='/assets',
            template_folder=DIST_DIR)
CORS(app)

@app.route("/")
def index():
    return render_template('index.html')

# Catch-all for React files (vite.svg, etc.)
@app.route('/<path:filename>')
def serve_root_files(filename):
    return send_from_directory(DIST_DIR, filename)

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
        # Store the exact angle (-90 left to +90 right)
        car_state["steer_angle"] = angle
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
    """Immediately locks motors with magnetic braking (instant stop)"""
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    # Magnetic lock: all pins to False
    GPIO.output(IN1, False); GPIO.output(IN2, False)
    GPIO.output(IN3, False); GPIO.output(IN4, False)
    pwm_a.ChangeDutyCycle(100); pwm_b.ChangeDutyCycle(100)
    print("🚨 EMERGENCY STOP: Motors locked!")
    return "EMERGENCY_STOP"

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
@app.route("/telemetry")
def telemetry():
    """Returns fake data for the dashboard gauges"""
    pwm = car_state["current_pwm"]
    
    # Math to fake RPM and KM/H based on power
    fake_rpm = int(pwm * 2.2)      # Max ~220 RPM
    fake_speed = round(pwm * 0.025, 1) # Max ~2.5 km/h
    
    return jsonify({
        "rpm": fake_rpm,
        "speed": fake_speed,
        "gear": car_state["gear"]
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
        "gpu_clock": gpu_clock
    })

if __name__ == "__main__":
    try:
        # Host 0.0.0.0 makes it available on your Wi-Fi
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        # Safety cleanup
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()