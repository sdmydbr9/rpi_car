import time
import sys
import RPi.GPIO as GPIO

# Import your driver
from sensors import SensorSystem

def main():
    # 1. Initialize
    print("⏳ Initializing Sonar...")
    try:
        # This will set GPIO mode and setup pins automatically
        car_eyes = SensorSystem()
        print("✅ Sonar Active! Press CTRL+C to stop.")
        print("-" * 40)
        print(f"{'DISTANCE (cm)':<20} | {'STATUS'}")
        print("-" * 40)

    except Exception as e:
        print(f"\n❌ Setup Failed: {e}")
        return

    # 2. Test Loop
    try:
        while True:
            # Get distance from your driver
            dist = car_eyes.get_sonar_distance()
            
            # Interpret the data
            status = "CLEAR"
            if dist == -1:
                status = "⏱️ TIMEOUT / ERROR"
            elif dist < 5:
                status = "🚨 CRASH IMMINENT"
            elif dist < 20:
                status = "⚠️ CLOSE"
            
            # Print cleanly (overwrite line for animation effect if supported, else just print)
            # using \r to stay on same line is cleaner for terminals
            sys.stdout.write(f"\r📏 {dist} cm \t\t| {status}          ")
            sys.stdout.flush()
            
            time.sleep(0.1) # Fast refresh

    except KeyboardInterrupt:
        print("\n\n👋 Stopping test...")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
