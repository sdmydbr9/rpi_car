import time
import os

# Create a mock GPIO class for testing
class MockGPIO:
    BCM = "BCM"
    IN = "IN"
    OUT = "OUT"
    
    def __init__(self):
        self.pin_states = {}  # For testing: store pin values
    
    def setmode(self, mode): pass
    def setwarnings(self, val): pass
    def setup(self, pins, mode): pass
    
    def input(self, pin):
        """Return the current pin state (default 1 if not set)"""
        return self.pin_states.get(pin, 1)
    
    def output(self, pins, state): pass
    def cleanup(self): pass
    
    class PWM:
        def __init__(self, pin, freq): pass
        def start(self, val): pass
        def stop(self): pass
        def ChangeDutyCycle(self, val): pass
    
    def PWM(self, pin, freq):
        return self.PWM(pin, freq)
    
    # Test helper: set pin state directly
    def set_pin(self, pin, value):
        """For testing: manually set a pin's input state"""
        self.pin_states[pin] = value

# Create a GPIO wrapper that can handle both real and mock GPIO
class GPIOWrapper:
    def __init__(self, real_gpio=None):
        self.real_gpio = real_gpio
        self.pin_states = {}  # For testing with real GPIO
        # Use real GPIO constants if available, otherwise use strings
        if real_gpio:
            self.BCM = real_gpio.BCM
            self.IN = real_gpio.IN
            self.OUT = real_gpio.OUT
        else:
            self.BCM = "BCM"
            self.IN = "IN"
            self.OUT = "OUT"
    
    def setmode(self, mode):
        if self.real_gpio:
            self.real_gpio.setmode(mode)
    
    def setwarnings(self, val):
        if self.real_gpio:
            self.real_gpio.setwarnings(val)
    
    def setup(self, pins, mode):
        if self.real_gpio:
            self.real_gpio.setup(pins, mode)
    
    def input(self, pin):
        """Read from pin - check override first, then real GPIO"""
        if pin in self.pin_states:
            return self.pin_states[pin]
        if self.real_gpio:
            return self.real_gpio.input(pin)
        return 1
    
    def output(self, pins, state):
        if self.real_gpio:
            self.real_gpio.output(pins, state)
    
    def cleanup(self):
        if self.real_gpio:
            self.real_gpio.cleanup()
    
    def PWM(self, pin, freq):
        if self.real_gpio:
            return self.real_gpio.PWM(pin, freq)
        return MockGPIO.PWM(pin, freq)
    
    def set_pin(self, pin, value):
        """For testing: override a pin's input state"""
        self.pin_states[pin] = value

# Try to use real GPIO, fall back to mock for testing
GPIO_AVAILABLE = False
real_gpio = None

# Check if we're on a real Raspberry Pi
if os.path.exists('/proc/device-tree/model'):  # Running on RPi
    try:
        import RPi.GPIO as real_gpio
    except Exception as e:
        print(f"⚠️  GPIO import failed: {e}")

# Create the GPIO wrapper
GPIO = GPIOWrapper(real_gpio)

# Try to initialize real GPIO if available
if real_gpio:
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO_AVAILABLE = True
    except Exception as e:
        print(f"⚠️  GPIO initialization failed: {e}")

class CarSystem:
    def __init__(self):
        # --- HARDWARE CONFIGURATION ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pins (User Verified)
        self.IN1 = 17; self.IN2 = 27  # Left
        self.IN3 = 22; self.IN4 = 23  # Right
        self.ENA = 18; self.ENB = 19  # Speed
        
        # IR Obstacle Sensors
        self.LEFT_IR = 5   # GPIO 5 - Left Front Obstacle Detection
        self.RIGHT_IR = 6  # GPIO 6 - Right Front Obstacle Detection
        self.AVOID_SWERVE_ANGLE = 85  # Degrees to swerve when obstacle detected (increased for aggressive steering)

        # Setup Motor Pins
        try:
            GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        except Exception as e:
            print(f"⚠️  Motor pin setup failed: {e}")
        
        # Setup IR Sensor Pins (Input)
        try:
            GPIO.setup([self.LEFT_IR, self.RIGHT_IR], GPIO.IN)
        except Exception as e:
            print(f"⚠️  IR sensor pin setup failed: {e}")

        # PWM Init
        try:
            self.pwm_a = GPIO.PWM(self.ENA, 1000)
            self.pwm_b = GPIO.PWM(self.ENB, 1000)
            self.pwm_a.start(0)
            self.pwm_b.start(0)
        except Exception as e:
            print(f"⚠️  PWM setup failed: {e}")
            # Create dummy PWM objects
            class DummyPWM:
                def start(self, val): pass
                def stop(self): pass
                def ChangeDutyCycle(self, val): pass
            self.pwm_a = DummyPWM()
            self.pwm_b = DummyPWM()

        # State Variables
        self.current_gear = 1
        self.steering_angle = 0
        self.user_steering_angle = 0  # Manual input (preserved during avoidance)
        self.is_gas_pressed = False
        self.current_speed = 0  # Track current motor speed for acceleration
        self.acceleration_rate = 5  # Speed increase per update cycle (0-100 scale)
        self.last_update_time = time.time()
        self.left_obstacle = False   # IR sensor state
        self.right_obstacle = False  # IR sensor state
        self.obstacle_avoidance_active = False  # Track if currently avoiding
        self.last_left_obstacle = False  # Track state changes
        self.last_right_obstacle = False
        
        # Obstacle Avoidance State Machine
        self.avoidance_state = "IDLE"  # IDLE, STEERING
        self.obstacle_clear_timer = 0  # Timer to confirm obstacle is cleared
        self.obstacle_clear_threshold = 10  # Cycles to confirm obstacle is gone (0.2s typical)
        self.safe_steer_angle = 75     # Smart steering angle (less aggressive than 90°)
        
        # Max Speed per Gear (0-100)
        self.GEAR_SPEEDS = {0: 0, 1: 30, 2: 50, 3: 75, 4: 100, -1: 60}  # -1 is reverse gear at 60%
        
        # Emergency Brake Settings
        self.EMERGENCY_BRAKE_RATE = 5.0  # Fast deceleration on obstacle detection

    def check_obstacles(self):
        """Read IR sensors. IR sensors are active LOW: 0 = obstacle, 1 = no obstacle"""
        # Invert the reading so True = obstacle detected
        self.left_obstacle = not GPIO.input(self.LEFT_IR)
        self.right_obstacle = not GPIO.input(self.RIGHT_IR)
        
        # Print only on state change
        if self.left_obstacle and not self.last_left_obstacle:
            print("⚠️  LEFT OBSTACLE DETECTED!")
        if self.right_obstacle and not self.last_right_obstacle:
            print("⚠️  RIGHT OBSTACLE DETECTED!")
        if not self.left_obstacle and self.last_left_obstacle:
            print("✅ LEFT CLEAR")
        if not self.right_obstacle and self.last_right_obstacle:
            print("✅ RIGHT CLEAR")
        
        self.last_left_obstacle = self.left_obstacle
        self.last_right_obstacle = self.right_obstacle

    def update(self):
        """Calculates and applies motor speeds based on state"""
        # Check obstacles first
        self.check_obstacles()
        
        # --- SMART OBSTACLE AVOIDANCE STATE MACHINE ---
        obstacle_detected = self.left_obstacle or self.right_obstacle
        
        if self.is_gas_pressed and obstacle_detected:
            # Only avoidance while actively moving (gas pressed)
            if self.avoidance_state == "IDLE":
                # Trigger: Obstacle detected, start STEERING immediately
                self.avoidance_state = "STEERING"
                self.obstacle_clear_timer = 0
                
                # Smart steering: Determine direction to steer AWAY from obstacle
                if self.left_obstacle and self.right_obstacle:
                    # Both obstacles: steer right (away from both directions)
                    self.steering_angle = self.safe_steer_angle
                    print("🚨 BOTH OBSTACLES DETECTED - AUTO-STEERING RIGHT 75°")
                elif self.left_obstacle:
                    # Left obstacle: steer RIGHT (away from left)
                    self.steering_angle = self.safe_steer_angle
                    print("⚠️  LEFT OBSTACLE DETECTED - STEERING RIGHT 75°")
                else:  # right_obstacle
                    # Right obstacle: steer LEFT (away from right)
                    self.steering_angle = -self.safe_steer_angle
                    print("⚠️  RIGHT OBSTACLE DETECTED - STEERING LEFT 75°")
            
            elif self.avoidance_state == "STEERING":
                # Continue steering: apply smart steering angle based on current obstacles
                # This allows dynamic adjustment if situation changes
                if self.left_obstacle and self.right_obstacle:
                    # Both obstacles: maintain right steer
                    self.steering_angle = self.safe_steer_angle
                elif self.left_obstacle:
                    # Left obstacle: maintain right steer
                    self.steering_angle = self.safe_steer_angle
                else:  # right_obstacle
                    # Right obstacle: maintain left steer
                    self.steering_angle = -self.safe_steer_angle
                
                # Obstacle gone? Start confirmation timer
                self.obstacle_clear_timer += 1
        else:
            # No obstacles detected or gas released
            if self.avoidance_state != "IDLE":
                # Try to exit avoidance mode
                if obstacle_detected:
                    # Obstacle came back but gas released, just stop steering
                    self.avoidance_state = "IDLE"
                    self.steering_angle = self.user_steering_angle
                else:
                    # Obstacle truly clear - reset steering
                    self.avoidance_state = "IDLE"
                    self.steering_angle = self.user_steering_angle
                    if self.obstacle_clear_timer > 0:
                        print("✅ OBSTACLE CLEARED - RESUMING NORMAL CONTROL")
                    self.obstacle_clear_timer = 0
        
        # --- SPEED HANDLING ---
        if self.avoidance_state == "IDLE":
            # Normal operation: target gear speed
            base_speed = self.GEAR_SPEEDS.get(self.current_gear, 0)
            
            # Deadman Switch: Stop if Gas released or Neutral
            if not self.is_gas_pressed or self.current_gear == 0:
                self.current_speed = 0
                self._set_raw_motors(0, 0, False, False, False, False)
                return

            # Normal acceleration logic
            if self.current_speed < base_speed:
                self.current_speed += self.acceleration_rate
                if self.current_speed > base_speed:
                    self.current_speed = base_speed
            elif self.current_speed > base_speed:
                self.current_speed -= self.acceleration_rate
                if self.current_speed < base_speed:
                    self.current_speed = base_speed
        else:
            # During obstacle avoidance (STEERING): reduce speed for stability while steering
            # Reduce to 60% of current speed for smoother steering
            avoidance_speed = self.current_speed * 0.6
            if self.current_speed > avoidance_speed:
                self.current_speed -= self.acceleration_rate * 0.5  # Gradual reduction
                if self.current_speed < avoidance_speed:
                    self.current_speed = avoidance_speed

        # Steering Physics (Mixing)
        # Intensity 0.0 to 1.0
        intensity = abs(self.steering_angle) / 90.0 
        inner_wheel_factor = 1.0 - (intensity * 0.9) # Inner wheel drops to 10% at max turn
        
        left_speed = self.current_speed
        right_speed = self.current_speed
        
        # Note: Using your "Inverted Logic" (False = Forward)
        # Left Fwd: (False, True) | Right Fwd: (False, True)
        l_a = False; l_b = True
        r_a = False; r_b = True

        if self.steering_angle < -5: # LEFT TURN
            left_speed = self.current_speed * inner_wheel_factor
        
        elif self.steering_angle > 5: # RIGHT TURN
            right_speed = self.current_speed * inner_wheel_factor

        self._set_raw_motors(left_speed, right_speed, l_a, l_b, r_a, r_b)

    def _set_raw_motors(self, speed_l, speed_r, la, lb, ra, rb):
        GPIO.output(self.IN1, la)
        GPIO.output(self.IN2, lb)
        GPIO.output(self.IN3, ra)
        GPIO.output(self.IN4, rb)
        self.pwm_a.ChangeDutyCycle(int(speed_l))
        self.pwm_b.ChangeDutyCycle(int(speed_r))

    def set_steering(self, angle):
        self.user_steering_angle = angle
        self.update()

    def set_gear(self, gear):
        self.current_gear = gear
        self.current_speed = 0  # Reset acceleration when changing gears
        self.update()

    def set_gas(self, pressed):
        self.is_gas_pressed = pressed
        self.update()

    def emergency_brake(self):
        self.is_gas_pressed = False
        # Magnetic Lock
        GPIO.output([self.IN1, self.IN2, self.IN3, self.IN4], False)
        self.pwm_a.ChangeDutyCycle(100)
        self.pwm_b.ChangeDutyCycle(100)
        time.sleep(0.5)
        self._set_raw_motors(0, 0, False, False, False, False)

    def cleanup(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
