import RPi.GPIO as GPIO
import time

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
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        # Setup IR Sensor Pins (Input)
        GPIO.setup([self.LEFT_IR, self.RIGHT_IR], GPIO.IN)

        # PWM Init
        self.pwm_a = GPIO.PWM(self.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

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
        self.steering_timer = 0        # Timer for steering duration
        self.target_steer_angle = 0    # Target angle during steering phase
        
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
        
        # --- OBSTACLE AVOIDANCE STATE MACHINE ---
        if self.is_gas_pressed and (self.left_obstacle or self.right_obstacle):
            if self.avoidance_state == "IDLE":
                # Trigger: Obstacle detected, immediately start STEERING
                self.avoidance_state = "STEERING"
                self.steering_timer = 0
                
                # Determine steering direction based on obstacles
                if self.left_obstacle and self.right_obstacle:
                    # Both obstacles: steer right
                    self.target_steer_angle = 90
                    print("🚨 BOTH OBSTACLES - AUTO-STEERING RIGHT")
                elif self.left_obstacle:
                    # Left obstacle: steer right 90°
                    self.target_steer_angle = 90
                    print("🚨 LEFT OBSTACLE - STEERING RIGHT 90°")
                else:  # right_obstacle
                    # Right obstacle: steer left 90°
                    self.target_steer_angle = -90
                    print("🚨 RIGHT OBSTACLE - STEERING LEFT 90°")
            
            elif self.avoidance_state == "STEERING":
                # In steering state: apply target angle
                self.steering_angle = self.target_steer_angle
                self.steering_timer += 1
                
                # Continue steering while obstacles present
                if not (self.left_obstacle or self.right_obstacle):
                    # All obstacles cleared, return to normal
                    self.avoidance_state = "IDLE"
                    self.steering_angle = self.user_steering_angle
                    print("✅ ALL OBSTACLES CLEARED - RESUMING NORMAL CONTROL")
                elif self.left_obstacle and self.right_obstacle:
                    # Both still present, auto-steer
                    if self.steering_timer > 50:  # Switch direction every 1 second
                        self.target_steer_angle = -self.target_steer_angle  # Toggle direction
                        self.steering_timer = 0
                        direction = "LEFT" if self.target_steer_angle < 0 else "RIGHT"
                        print(f"🔄 BOTH OBSTACLES STILL PRESENT - STEERING {direction}")
        else:
            # No obstacles detected
            if self.avoidance_state != "IDLE":
                self.avoidance_state = "IDLE"
                self.steering_angle = self.user_steering_angle
        
        # --- NORMAL OPERATION (when not in obstacle avoidance) ---
        if self.avoidance_state == "IDLE":
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
            # During obstacle avoidance (STEERING): continue at current speed while steering
            pass

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
