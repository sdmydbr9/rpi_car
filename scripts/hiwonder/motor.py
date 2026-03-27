from machine import Pin, PWM
import sys
from time import sleep

# --- 1. Setup the Pins ---
# Left Motor
ena = PWM(Pin(10))      # GP10 -> Physical Pin 14
in1 = Pin(17, Pin.OUT)  # GP17 -> Physical Pin 22
in2 = Pin(12, Pin.OUT)  # GP12 -> Physical Pin 16

# Right Motor
in3 = Pin(13, Pin.OUT)  # GP13 -> Physical Pin 17
in4 = Pin(14, Pin.OUT)  # GP14 -> Physical Pin 19
enb = PWM(Pin(16))      # GP16 -> Physical Pin 21

# Set frequency and a default speed
ena.freq(1000)
enb.freq(1000)
SPEED = 40000  # Roughly 60% throttle
ena.duty_u16(SPEED)
enb.duty_u16(SPEED)

# --- 2. Directional Logic ---
def set_left(state):
    if state == 1:     # Forward
        in1.value(0)
        in2.value(1)
    elif state == -1:  # Backward
        in1.value(1)
        in2.value(0)
    else:              # Stop
        in1.value(0)
        in2.value(0)

def set_right(state):
    if state == 1:     # Forward
        in3.value(0)
        in4.value(1)
    elif state == -1:  # Backward
        in3.value(1)
        in4.value(0)
    else:              # Stop
        in3.value(0)
        in4.value(0)

def stop_all():
    set_left(0)
    set_right(0)

# --- 3. The Interactive Loop ---
stop_all()

print("--- INTERACTIVE ROVER CONTROL ---")
print("Click into this console window to drive!")
print(" e = Left Fwd      w = Right Fwd")
print(" d = Left Back     s = Right Back")
print(" a = All Fwd       b = All Back")
print(" [SPACE] = Stop    q = Quit")
print("---------------------------------")

try:
    while True:
        char = sys.stdin.read(1).lower()

        if char == 'q':
            print("\nQuitting...")
            break

        elif char == ' ':
            stop_all()
            print("\r[STOPPED]                     ", end="")

        elif char == 'e':
            set_left(1)
            print("\rLeft Side -> FORWARD          ", end="")

        elif char == 'd':
            set_left(-1)
            print("\rLeft Side -> BACKWARD         ", end="")

        elif char == 'w':
            set_right(1)
            print("\rRight Side -> FORWARD         ", end="")

        elif char == 's':
            set_right(-1)
            print("\rRight Side -> BACKWARD        ", end="")

        elif char == 'a':
            set_left(1)
            set_right(1)
            print("\rALL -> FORWARD                ", end="")

        elif char == 'b':
            set_left(-1)
            set_right(-1)
            print("\rALL -> BACKWARD               ", end="")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    stop_all()
    ena.duty_u16(0)
    enb.duty_u16(0)
    print("\nMotors parked safely.")
