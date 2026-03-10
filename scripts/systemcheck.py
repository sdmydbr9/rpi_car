import time
import sys
import RPi.GPIO as GPIO
# Import the class we defined in sensors.py
from sensors import SensorSystem

def main():
    print("🏥 STARTING SYSTEM HEALTH CHECK...")
    print("-" * 50)
    print(f"{'COMPONENT':<15} | {'READING':<15} | {'STATUS'}")
    print("-" * 50)

    # 1. Initialize the Sensor System
    # This automatically sets up all pins (Laser via Pico)
    try:
        car = SensorSystem()
        print(f"{'GPIO Setup':<15} | {'Done':<15} | [OK] ✅")
    except Exception as e:
        print(f"{'GPIO Setup':<15} | {'Failed':<15} | [CRITICAL] ❌")
        print(f"\nError details: {e}")
        return

    # 1b. TEST MPU6050 (via Pico sensor bridge)
    try:
        from pico_sensor_reader import init_pico_reader, get_gyro_z
        pico = init_pico_reader('/dev/ttyS0')
        time.sleep(1)  # wait for first packet
        if pico.is_connected():
            gz = get_gyro_z()
            print(f"{'MPU6050':<15} | {'gz=' + format(gz, '.1f') + '°/s':<15} | [OK] ✅ (Pico bridge)")
        else:
            print(f"{'MPU6050':<15} | {'No Pico data':<15} | [FAILED] ❌")
    except Exception as e:
        print(f"{'MPU6050':<15} | {'Pico error':<15} | [FAILED] ❌  {e}")

    # 2. TEST LASER (via Pico bridge → SensorSystem.get_forward_distance)
    dist_front = car.get_forward_distance()
    status_front = "UNKNOWN"
    if dist_front == -1:
        status_front = "[FAILED] ❌ (Timeout)"
    elif dist_front < 5:
        status_front = "[WARNING] ⚠️ (Too Close/Noise)"
    else:
        status_front = "[OK] ✅"
    
    print(f"{'Laser':<15} | {str(dist_front) + ' cm':<15} | {status_front}")
    time.sleep(0.1)

    print("-" * 50)
    print("Diagnostic Complete.")
    
    # Cleanup only if we are done, though SensorSystem usually keeps running.
    # GPIO.cleanup() # Optional: keeping it commented so you can run other scripts immediately

if __name__ == "__main__":
    main()
