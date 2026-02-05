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

        # Setup
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)

        # PWM Init
        self.pwm_a = GPIO.PWM(self.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        # State Variables
        self.current_gear = 1
        self.steering_angle = 0
        self.is_gas_pressed = False
        self.current_speed = 0  # Track current motor speed for acceleration
        self.acceleration_rate = 5  # Speed increase per update cycle (0-100 scale)
        self.last_update_time = time.time()
        
        # Max Speed per Gear (0-100)
        self.GEAR_SPEEDS = {0: 0, 1: 30, 2: 50, 3: 75, 4: 100, -1: 60}  # -1 is reverse gear at 60%

    def update(self):
        """Calculates and applies motor speeds based on state"""
        base_speed = self.GEAR_SPEEDS.get(self.current_gear, 0)
        
        # Deadman Switch: Stop if Gas released or Neutral
        if not self.is_gas_pressed or self.current_gear == 0:
            self.current_speed = 0
            self._set_raw_motors(0, 0, False, False, False, False)
            return

        # Acceleration logic: gradually increase speed from 0 to target
        if self.current_speed < base_speed:
            self.current_speed += self.acceleration_rate
            if self.current_speed > base_speed:
                self.current_speed = base_speed
        elif self.current_speed > base_speed:
            self.current_speed -= self.acceleration_rate
            if self.current_speed < base_speed:
                self.current_speed = base_speed

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
        self.steering_angle = angle
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
