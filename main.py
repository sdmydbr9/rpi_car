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

# Setup PWM
try:
    pwm_a = GPIO.PWM(ENA, FREQ)
    pwm_b = GPIO.PWM(ENB, FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
except Exception as e:
    print(f"⚠️  PWM setup error: {e}")
    # Create dummy PWM objects for testing
    class DummyPWM:
        def ChangeDutyCycle(self, val): pass
        def stop(self): pass
    pwm_a = DummyPWM()
    pwm_b = DummyPWM()

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
    "obstacle_state": "IDLE"  # Obstacle avoidance state: IDLE, STOPPED, STEERING
}

def physics_loop():
    # Note: obstacle_state is now in car_state, not a local variable
    stop_timer = 0           # Timer for 1-second stop (50 cycles = 1 second at 20ms)
    steering_timer = 0       # Timer for steering duration
    last_left_obstacle = False
    last_right_obstacle = False
    target_steer_angle = 0   # Target angle during steering phase
    
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
        
        # Debug: Print only on state change (less spam)
        if left_obstacle and not last_left_obstacle:
            print("⚠️  LEFT OBSTACLE DETECTED!")
        if right_obstacle and not last_right_obstacle:
            print("⚠️  RIGHT OBSTACLE DETECTED!")
        if not left_obstacle and last_left_obstacle:
            print("✅ LEFT CLEAR")
        if not right_obstacle and last_right_obstacle:
            print("✅ RIGHT CLEAR")
        
        last_left_obstacle = left_obstacle
        last_right_obstacle = right_obstacle
        
        gear = car_state["gear"]
        gas = car_state["gas_pressed"]
        brake = car_state["brake_pressed"]
        current = car_state["current_pwm"]
        user_angle = car_state["user_steer_angle"]
        obstacle_state = car_state["obstacle_state"]
        
        # --- OBSTACLE AVOIDANCE STATE MACHINE ---
        if car_state["ir_enabled"] and gas and (left_obstacle or right_obstacle):
            if obstacle_state == "IDLE":
                # Trigger: Obstacle detected, enter STOPPED state
                obstacle_state = "STOPPED"
                car_state["obstacle_state"] = "STOPPED"
                stop_timer = 50  # 1 second (50 cycles × 20ms)
                print("🚨 OBSTACLE(S) DETECTED - EMERGENCY STOP FOR 1 SECOND")
                current = 0  # Immediately cut power
                car_state["current_pwm"] = current
            
            elif obstacle_state == "STOPPED":
                # In stopped state: keep speed at 0
                current = 0
                car_state["current_pwm"] = current
                stop_timer -= 1
                
                if stop_timer <= 0:
                    # Stop period complete, transition to STEERING
                    obstacle_state = "STEERING"
                    car_state["obstacle_state"] = "STEERING"
                    steering_timer = 0
                    
                    # Determine steering direction based on obstacles
                    if left_obstacle and right_obstacle:
                        # Both obstacles: alternate steering left/right
                        target_steer_angle = 90  # Start steering right
                        print("🚨 BOTH OBSTACLES - AUTO-STEERING RIGHT")
                    elif left_obstacle:
                        # Left obstacle: steer right 90°
                        target_steer_angle = 90
                        print("🚨 LEFT OBSTACLE - STEERING RIGHT 90°")
                    else:  # right_obstacle
                        # Right obstacle: steer left 90°
                        target_steer_angle = -90
                        print("🚨 RIGHT OBSTACLE - STEERING LEFT 90°")
            
            elif obstacle_state == "STEERING":
                # In steering state: apply target angle
                car_state["steer_angle"] = target_steer_angle
                steering_timer += 1
                
                # Continue steering while obstacles present
                if not (left_obstacle or right_obstacle):
                    # All obstacles cleared, return to normal
                    obstacle_state = "IDLE"
                    car_state["obstacle_state"] = "IDLE"
                    car_state["steer_angle"] = user_angle
                    print("✅ ALL OBSTACLES CLEARED - RESUMING NORMAL CONTROL")
                elif left_obstacle and right_obstacle:
                    # Both still present, auto-steer
                    if steering_timer > 50:  # Switch direction every 1 second
                        target_steer_angle = -target_steer_angle  # Toggle direction
                        steering_timer = 0
                        direction = "LEFT" if target_steer_angle < 0 else "RIGHT"
                        print(f"🔄 BOTH OBSTACLES STILL PRESENT - STEERING {direction}")
        else:
            # No obstacles detected
            if obstacle_state != "IDLE":
                obstacle_state = "IDLE"
                car_state["obstacle_state"] = "IDLE"
            car_state["steer_angle"] = user_angle
        
        # Get the current steering angle (either from avoidance or user input)
        angle = car_state["steer_angle"]
        
        # --- NORMAL THROTTLE PHYSICS (when not in obstacle avoidance) ---
        if obstacle_state == "IDLE":
            # Check if emergency brake is active - if so, force speed to 0
            if car_state["emergency_brake_active"]:
                current = 0
                car_state["current_pwm"] = 0
            # Normal operation
            elif brake:
                # Normal brake
                target = 0
                if current > target:
                    current -= BRAKE_RATE
                if current < 0:
                    current = 0
                car_state["current_pwm"] = current
            else:
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
        try:
            pwm_a.ChangeDutyCycle(int(left_motor_speed))
            pwm_b.ChangeDutyCycle(int(right_motor_speed))
        except Exception as e:
            pass  # GPIO not available, skip

        # Direction Logic
        l_a, l_b = False, True # Default Forward
        r_a, r_b = False, True # Default Forward

        if gear == "R": # Reverse Logic
            l_a, l_b = True, False
            r_a, r_b = True, False

        # NOTE: We removed the "Spin" override. 
        # Now we only steer by changing speed, not direction.
        
        try:
            GPIO.output(IN1, l_a); GPIO.output(IN2, l_b)
            GPIO.output(IN3, r_a); GPIO.output(IN4, r_b)
        except Exception as e:
            pass  # GPIO not available, skip

        time.sleep(0.02)  # 20ms loop = 50Hz (2.5x faster reaction than 50ms)

# Start the Engine (Thread)
engine_thread = threading.Thread(target=physics_loop, daemon=True)
engine_thread.start()

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
    """Toggle emergency brake - when ON, car cannot move"""
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    
    # CRITICAL: Reset obstacle avoidance state when emergency brake is toggled
    # This prevents the car from being stuck in STOPPED or STEERING mode after releasing the brake
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Magnetic lock: all pins to False
    try:
        GPIO.output(IN1, False); GPIO.output(IN2, False)
        GPIO.output(IN3, False); GPIO.output(IN4, False)
        pwm_a.ChangeDutyCycle(100); pwm_b.ChangeDutyCycle(100)
    except Exception as e:
        pass  # GPIO not available
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"🚨 EMERGENCY BRAKE: {state}")
    return "EMERGENCY_BRAKE"

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
    print(f"\n🔗 [Socket] ✅ CLIENT CONNECTED")
    print(f"   Client ID: {request.sid}")
    print(f"   Remote Address: {request.remote_addr}")
    emit('connection_response', {'data': 'Connected to RC Car'})

@socketio.on('disconnect')
def on_disconnect():
    """Handle client disconnection"""
    print(f"\n🔓 [Socket] ❌ CLIENT DISCONNECTED")
    print(f"   Client ID: {request.sid}")

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
    """Handle emergency brake toggle - when ON, car cannot move"""
    # Toggle the emergency brake state
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    
    # Immediately cut power
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    
    # CRITICAL: Reset obstacle avoidance state when emergency brake is toggled
    # This prevents the car from being stuck in STOPPED or STEERING mode after releasing the brake
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Lock the motors physically
    try:
        GPIO.output(18, False); GPIO.output(19, False)
        GPIO.output(17, False); GPIO.output(27, False)
        GPIO.output(22, False); GPIO.output(23, False)
        pwm_a.ChangeDutyCycle(100); pwm_b.ChangeDutyCycle(100)
    except Exception as e:
        pass  # GPIO not available
    
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"\n⚙️ [UI Control] 🚨 EMERGENCY BRAKE: {state}")
    emit('emergency_stop_response', {'status': 'ok', 'emergency_brake_active': car_state["emergency_brake_active"]})

@socketio.on('auto_accel_enable')
def on_auto_accel_enable(data):
    """Handle auto-acceleration enable (server-side auto-throttle)"""
    car_state["auto_accel_enabled"] = True
    print(f"\n⚙️ [UI Control] 🚀 AUTO-ACCEL: ENABLED (auto-throttle mode)")
    emit('auto_accel_response', {'status': 'ok', 'enabled': True})

@socketio.on('auto_accel_disable')
def on_auto_accel_disable(data):
    """Handle auto-acceleration disable"""
    car_state["auto_accel_enabled"] = False
    car_state["gas_pressed"] = False
    print(f"\n⚙️ [UI Control] 🚫 AUTO-ACCEL: DISABLED")
    emit('auto_accel_response', {'status': 'ok', 'enabled': False})

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
                "brake_pressed": car_state["brake_pressed"]
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
        "right_obstacle": car_state["right_obstacle"]
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
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()