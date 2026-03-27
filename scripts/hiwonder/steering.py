from machine import Pin, PWM
import sys
from time import sleep

# -----------------------------
# Servo configuration
# -----------------------------
# You plugged into Physical Pin 20, which is GP15
SERVO_PIN = 15 

# Pulse widths in microseconds (exactly as you had them)
CENTER_PW = 1440   
LEFT_PW   = 940    
RIGHT_PW  = 2150   

STEP = 20          # pulse width change per keypress

# Initialize PWM on GP15
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)     # Standard servos run at 50Hz

def set_servo_us(pulsewidth_us):
    """Converts microseconds to nanoseconds for the Pico's hardware PWM"""
    servo.duty_ns(pulsewidth_us * 1000)

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

# Start centered
current_pw = CENTER_PW
set_servo_us(current_pw)

print("--- Steering Calibration Test (Pico) ---")
print("Click into this console window to send keys.")
print("Press 'L' to steer Left")
print("Press 'R' to steer Right")
print("Press 'C' to center the wheels")
print("Press 'Q' to quit and relax servo")
print("----------------------------------------")
print(f"Centered. Current pulse width: {current_pw} us")

try:
    while True:
        # Read exactly 1 character from the Thonny console
        char = sys.stdin.read(1).lower()

        if char == 'q':
            print("\nQuitting and centering wheels...")
            break

        elif char == 'l':
            current_pw = clamp(current_pw - STEP, LEFT_PW, RIGHT_PW)
            set_servo_us(current_pw)
            print(f"\rSteering Left.   Current pulse width: {current_pw} us   ", end="")

        elif char == 'r':
            current_pw = clamp(current_pw + STEP, LEFT_PW, RIGHT_PW)
            set_servo_us(current_pw)
            print(f"\rSteering Right.  Current pulse width: {current_pw} us   ", end="")

        elif char == 'c':
            current_pw = CENTER_PW
            set_servo_us(current_pw)
            print(f"\rWheels Centered. Current pulse width: {current_pw} us   ", end="")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Re-center, then detach
    set_servo_us(CENTER_PW)
    sleep(0.4)
    servo.deinit() # This kills the PWM signal so the servo fully relaxes
    print("\nServo detached.")
