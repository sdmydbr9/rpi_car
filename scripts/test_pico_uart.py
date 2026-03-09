#!/usr/bin/env python3
"""
pico_uart_test.py — Test Pico↔Pi UART connection
Reads JSON sensor packets from Pico and validates the link
"""

import serial
import json
import time
import sys

def test_pico_connection():
    """Test UART connection to Pico and read sensor data."""
    
    print("\n" + "="*60)
    print("  PICO ↔ PI UART CONNECTION TEST")
    print("="*60 + "\n")
    
    # ── Check for available serial ports ──
    import os
    uart_path = None
    
    for port in ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/serial0']:
        if os.path.exists(port):
            uart_path = port
            print(f"✅ Found UART: {port}\n")
            break
    
    if not uart_path:
        print("❌ ERROR: No UART device found")
        print("   Expected: /dev/ttyAMA0 (or /dev/ttyS0 / /dev/serial0)")
        print("\n   Fix: Edit /boot/firmware/config.txt and add:")
        print("       enable_uart=1")
        print("       dtoverlay=uart0")
        print("   Then reboot: sudo reboot")
        return False
    
    # ── Open serial connection ──
    try:
        ser = serial.Serial(uart_path, 115200, timeout=2)
        print(f"✅ Serial port opened: {ser.port} @ {ser.baudrate} baud\n")
    except PermissionError:
        print(f"❌ Permission denied on {uart_path}")
        print("   Fix: sudo usermod -a -G dialout $USER")
        print("   Then logout/login completely")
        return False
    except Exception as e:
        print(f"❌ Failed to open serial: {e}")
        return False
    
    # ── Read packets ──
    print("Waiting for data from Pico (10 second timeout)...\n")
    
    valid_packets = 0
    bad_packets = 0
    start_time = time.time()
    
    while time.time() - start_time < 10:
        try:
            line = ser.readline().decode().strip()
            
            if not line:
                continue
            
            # Try to parse as JSON
            if line.startswith('{'):
                try:
                    packet = json.loads(line)
                    valid_packets += 1
                    
                    # Pretty print first packet
                    if valid_packets == 1:
                        print("🎉 FIRST VALID PACKET RECEIVED:\n")
                        print(json.dumps(packet, indent=2))
                        print()
                    
                    # Status line for subsequent packets
                    if valid_packets > 1 and valid_packets % 5 == 0:
                        print(f"   ✅ Frame #{packet.get('frame', '?')} | "
                              f"Accel: ({packet['accel']['x']:.2f}, "
                              f"{packet['accel']['y']:.2f}, "
                              f"{packet['accel']['z']:.2f}) | "
                              f"Laser: {packet.get('laser_mm', -1)}mm")
                
                except json.JSONDecodeError:
                    bad_packets += 1
                    print(f"❌ Invalid JSON: {line}")
            else:
                print(f"📝 Status: {line}")
        
        except Exception as e:
            print(f"⚠️  Read error: {e}")
    
    ser.close()
    
    # ── Summary ──
    print("\n" + "="*60)
    print("  CONNECTION TEST SUMMARY")
    print("="*60)
    print(f"Valid packets received:  {valid_packets} ✅")
    print(f"Invalid packets:         {bad_packets} ❌")
    print(f"Total frames:            {valid_packets + bad_packets}")
    
    if valid_packets > 0:
        print(f"\n✅ SUCCESS! Pico is transmitting correctly.")
        print(f"   Expected ~50 fps = ~50 frames in 10 seconds")
        print(f"   Received: {valid_packets} frames")
        return True
    else:
        print(f"\n❌ FAILURE! No valid packets received.")
        print("   Checklist:")
        print("   1. Pico GPIO0 (pin 1) TX → Pi pin 10 (GPIO15 RX)")
        print("   2. Pico GPIO1 (pin 2) RX → Pi pin 8  (GPIO14 TX)")
        print("   3. Pico GND → Pi GND")
        print("   4. Pico running pico_sensor_bridge.py")
        print("   5. Pi UART enabled: raspi-config → Interface → Serial")
        return False

if __name__ == "__main__":
    try:
        success = test_pico_connection()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n⛔ Test interrupted by user")
        sys.exit(1)
