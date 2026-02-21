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
    # This automatically sets up all pins (Front, Rear, IR)
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

    # 2. TEST FRONT SONAR
    dist_front = car.get_sonar_distance()
    status_front = "UNKNOWN"
    if dist_front == -1:
        status_front = "[FAILED] ❌ (Timeout)"
    elif dist_front < 5:
        status_front = "[WARNING] ⚠️ (Too Close/Noise)"
    else:
        status_front = "[OK] ✅"
    
    print(f"{'Front Sonar':<15} | {str(dist_front) + ' cm':<15} | {status_front}")
    time.sleep(0.1)

    # 3. TEST REAR SONAR
    dist_rear = car.get_rear_sonar_distance()
    status_rear = "UNKNOWN"
    if dist_rear == -1:
        status_rear = "[FAILED] ❌ (Timeout)"
    elif dist_rear < 5:
        status_rear = "[WARNING] ⚠️ (Too Close/Noise)"
    else:
        status_rear = "[OK] ✅"

    print(f"{'Rear Sonar':<15} | {str(dist_rear) + ' cm':<15} | {status_rear}")
    time.sleep(0.1)

    # 4. TEST IR SENSORS
    # get_ir_status returns (Left_Is_Blocked, Right_Is_Blocked)
    left_blocked, right_blocked = car.get_ir_status()
    
    # Left IR
    l_val = "OBSTACLE" if left_blocked else "CLEAR"
    l_stat = "[OK] ✅" # We assume it works if it returns True/False
    print(f"{'Left IR':<15} | {l_val:<15} | {l_stat}")

    # Right IR
    r_val = "OBSTACLE" if right_blocked else "CLEAR"
    r_stat = "[OK] ✅"
    print(f"{'Right IR':<15} | {r_val:<15} | {r_stat}")

    print("-" * 50)
    print("Diagnostic Complete.")
    
    # Cleanup only if we are done, though SensorSystem usually keeps running.
    # GPIO.cleanup() # Optional: keeping it commented so you can run other scripts immediately

if __name__ == "__main__":
    main()
