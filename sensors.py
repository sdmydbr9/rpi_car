import RPi.GPIO as GPIO
import time

# --- PIN CONFIGURATION ---
# Front Sonar
TRIG = 25  # Output (Sends sound)
ECHO = 24  # Input (Receives sound - voltage divided)

# Rear Sonar
REAR_TRIG = 26  # Output (Sends sound)
REAR_ECHO = 16  # Input (Receives sound - voltage divided)

# IR Obstacle Sensors
IR_LEFT = 5   # Front Left
IR_RIGHT = 6  # Front Right

class SensorSystem:
    def __init__(self):
        # 1. Setup GPIO Mode
        # We check if mode is set; if not, set it to BCM
        try:
            mode = GPIO.getmode()
            if mode is None:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
        except Exception as e:
            print(f"⚠️ GPIO Config Error: {e}")

        # 2. Setup Pins — Front Sonar
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        # 3. Setup Pins — Rear Sonar
        GPIO.setup(REAR_TRIG, GPIO.OUT)
        GPIO.setup(REAR_ECHO, GPIO.IN)

        # 4. Setup Pins — IR Sensors
        GPIO.setup(IR_LEFT, GPIO.IN)
        GPIO.setup(IR_RIGHT, GPIO.IN)

        # 5. Initialize Outputs
        GPIO.output(TRIG, False)       # Ensure Front Trigger starts LOW
        GPIO.output(REAR_TRIG, False)  # Ensure Rear Trigger starts LOW
        
        # 6. Allow sensors to settle
        time.sleep(0.5)

    def _read_sonar(self, trig_pin, echo_pin):
        """
        Generic sonar reading for any TRIG/ECHO pair.
        Returns distance in cm, or -1 on timeout/error.
        """
        try:
            # 1. Send 10us Pulse
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)

            # 2. Wait for Echo Start (Timeout 0.1s to prevent freezing)
            pulse_start = time.time()
            timeout = pulse_start + 0.1
            while GPIO.input(echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return -1

            # 3. Wait for Echo End (Timeout 0.1s)
            pulse_end = time.time()
            timeout = pulse_end + 0.1
            while GPIO.input(echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return -1

            # 4. Calculate Distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # (Speed of sound 34300 cm/s) / 2
            distance = round(distance, 1)

            # Filter out crazy values (Sensor max is ~400cm)
            if distance > 400:
                return -1

            return distance

        except Exception as e:
            # print(f"Sonar Error: {e}") # Uncomment for debugging
            return -1

    def get_sonar_distance(self):
        """Returns front sonar distance in cm. Returns -1 on error."""
        return self._read_sonar(TRIG, ECHO)

    def get_rear_sonar_distance(self):
        """Returns rear sonar distance in cm. Returns -1 on error."""
        return self._read_sonar(REAR_TRIG, REAR_ECHO)

    def get_ir_status(self):
        """
        Returns (Left_Obstacle, Right_Obstacle) as Booleans.
        True = Obstacle Detected
        False = Path Clear
        """
        # Note: IR sensors usually send LOW (0) when they see an object.
        # We invert this so logic is easier to read (True = Obstacle).
        left_detect = not GPIO.input(IR_LEFT)
        right_detect = not GPIO.input(IR_RIGHT)
        
        return left_detect, right_detect
