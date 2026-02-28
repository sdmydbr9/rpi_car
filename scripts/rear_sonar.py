import time
import sys
import RPi.GPIO as GPIO

# --- CONFIGURATION ---
TRIG_FRONT = 25
ECHO_FRONT = 24

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(TRIG_FRONT, GPIO.OUT)
    GPIO.setup(ECHO_FRONT, GPIO.IN)
    GPIO.output(TRIG_FRONT, False)
    time.sleep(0.5)

    print("✅ Debug Mode: Angle your sensor UP slightly if stuck at 16cm.")
    
    try:
        while True:
            # 1. Trigger
            GPIO.output(TRIG_FRONT, True)
            time.sleep(0.00001)
            GPIO.output(TRIG_FRONT, False)

            # 2. Watch for Echo (With longer timeout)
            pulse_start = time.time()
            timeout = pulse_start + 0.1  # Increased to 100ms (17 meters)
            
            # Wait for signal HIGH
            while GPIO.input(ECHO_FRONT) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    break

            # Wait for signal LOW
            pulse_end = time.time()
            timeout = pulse_end + 0.1
            
            while GPIO.input(ECHO_FRONT) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    break

            # 3. Calculate
            if pulse_end > pulse_start:
                duration = pulse_end - pulse_start
                distance = duration * 17150
                distance = round(distance, 1)
                
                # Check for "Floor Detection"
                note = ""
                if 14 < distance < 18:
                    note = " <-- LIKELY THE FLOOR"
                
                print(f"Raw Dist: {distance} cm {note}")
            else:
                print("--- Timeout (No Echo) ---")

            time.sleep(0.2)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
