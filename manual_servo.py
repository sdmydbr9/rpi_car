import RPi.GPIO as GPIO
import time
import sys
import tty
import termios

# Setup GPIO
SERVO_PIN = 20
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

# State
current_angle = 90
step = 10

def getch():
    """Reads a single character from stdin without needing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def set_servo(angle):
    global current_angle
    current_angle = max(0, min(180, angle))
    
    # Standard formula: 2.5 to 12.5 often covers full 180 range better
    duty = 2 + (current_angle / 18)
    pwm.ChangeDutyCycle(duty)
    
    # Allow time to move, then cut signal to stop jitter/heat
    time.sleep(0.2)
    pwm.ChangeDutyCycle(0)
    print(f"\rAngle: {current_angle} | Press 'L' (Left), 'R' (Right), or 'Q' (Quit)", end="", flush=True)

try:
    print("Direct SSH Control Active.")
    set_servo(current_angle)

    while True:
        char = getch().lower() # Handle both 'L' and 'l'
        
        if char == 'l':
            set_servo(current_angle - step)
        elif char == 'r':
            set_servo(current_angle + step)
        elif char == 'q':
            print("\nExiting...")
            break

except Exception as e:
    print(f"\nError: {e}")
finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO Cleaned up.")
