"""Manual pan/tilt servo control via Pico UART bridge.

Replaces the old GPIO 20 PWM servo test. Now sends PT:<pan>,<tilt>
commands over UART to the Pico, which drives the camera pan/tilt gimbal.
"""
import sys
import tty
import termios
from pico_sensor_reader import init_pico_reader, send_pan_tilt, send_center

# State
PAN_CENTER = 90
TILT_CENTER = 90
PAN_MIN, PAN_MAX = 60, 120
TILT_MIN, TILT_MAX = 60, 120

current_pan = PAN_CENTER
current_tilt = TILT_CENTER
step = 5


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


def set_pan_tilt(pan, tilt):
    global current_pan, current_tilt
    current_pan = max(PAN_MIN, min(PAN_MAX, pan))
    current_tilt = max(TILT_MIN, min(TILT_MAX, tilt))
    send_pan_tilt(current_pan, current_tilt)
    print(f"\rPan: {current_pan}  Tilt: {current_tilt} | L/R=pan  U/D=tilt  C=center  Q=quit", end="", flush=True)


try:
    init_pico_reader()
    print("Manual Pan/Tilt Control (via Pico UART)")
    set_pan_tilt(current_pan, current_tilt)

    while True:
        char = getch().lower()

        if char == 'l':
            set_pan_tilt(current_pan - step, current_tilt)
        elif char == 'r':
            set_pan_tilt(current_pan + step, current_tilt)
        elif char == 'u':
            set_pan_tilt(current_pan, current_tilt - step)
        elif char == 'd':
            set_pan_tilt(current_pan, current_tilt + step)
        elif char == 'c':
            send_center()
            current_pan = PAN_CENTER
            current_tilt = TILT_CENTER
            print(f"\rCentered ({PAN_CENTER}, {TILT_CENTER})                              ", end="", flush=True)
        elif char == 'q':
            print("\nExiting...")
            break

except Exception as e:
    print(f"\nError: {e}")
finally:
    send_center()
    print("Servo centered. Done.")
