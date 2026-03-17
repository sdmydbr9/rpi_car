#!/usr/bin/env python3
import time
import csv
import sys
import os
from datetime import datetime

# Resolve project paths so imports work just like in your main script
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)
for p in (SCRIPT_DIR, SCRIPTS_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

# Import your existing hardware reader
try:
    from pico_sensor_reader import init_pico_reader, get_gyro_z
except ImportError:
    print("Error: Could not import pico_sensor_reader. Make sure you run this from the right folder.")
    sys.exit(1)

def record_phase(phase_name, duration_sec, writer):
    print(f"\n--- Recording: {phase_name} ({duration_sec}s) ---")
    start_time = time.time()
    
    while time.time() - start_time < duration_sec:
        current_time = time.time()
        gz = get_gyro_z()
        
        # Simple terminal visualization to spot the sign instantly
        direction = "<< LEFT" if gz > 5.0 else "RIGHT >>" if gz < -5.0 else "  STILL  "
        print(f"Yaw (Z): {gz:+7.2f} °/s | {direction}")
        
        writer.writerow([current_time, phase_name, gz])
        time.sleep(0.1)  # 10Hz sampling rate

def main():
    print("Initializing Pico sensor bridge...")
    init_pico_reader()
    time.sleep(1)  # Give sensors a moment to stabilize

    log_dir = os.path.join(PROJECT_ROOT, "rover_logs")
    os.makedirs(log_dir, exist_ok=True)
    filename = os.path.join(log_dir, f"gyro_manual_test_{datetime.now():%Y%m%d_%H%M%S}.csv")
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Phase", "Gyro_Z"])
        
        print("\n*** MPU6050 GYRO DIRECTION TEST ***")
        print("Let's figure out which way your Z-axis is pointing.\n")
        
        input("1. Place the rover flat on the floor and DON'T TOUCH IT. Press [Enter] to start...")
        record_phase("STILL", 3, writer)
        
        input("\n2. Get ready to spin the rover smoothly to the LEFT (Counter-Clockwise). Press [Enter], then turn it...")
        record_phase("TURN_LEFT", 4, writer)
        
        input("\n3. Get ready to spin the rover smoothly to the RIGHT (Clockwise). Press [Enter], then turn it...")
        record_phase("TURN_RIGHT", 4, writer)
        
    print(f"\nTest complete! Data saved to {filename}")
    
    print("\n--- HOW TO READ THIS ---")
    print("Standard robotics convention dictates:")
    print(" • Turning LEFT (Counter-Clockwise) = POSITIVE Z")
    print(" • Turning RIGHT (Clockwise)        = NEGATIVE Z")
    print("If your terminal output above showed the exact opposite, your gyro is inverted relative to your math.")

if __name__ == "__main__":
    main()
