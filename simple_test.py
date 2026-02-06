#!/usr/bin/env python3
"""Simple test: Turn on, Gear 2, Accelerate"""

import socketio
import time

sio = socketio.Client(reconnection=True)

@sio.event
def connect():
    print("✅ Connected")
    time.sleep(0.5)
    
    # Shift to Gear 2
    print("\n→ Shifting to Gear 2")
    sio.emit('gear_change', {'gear': '2'})
    time.sleep(1)
    
    # Accelerate
    print("→ Accelerating (throttle on)")
    sio.emit('throttle', {'value': True})
    time.sleep(2)
    
    print("→ Requesting state...")
    sio.emit('state_request', {})
    time.sleep(0.5)
    
    sio.disconnect()

@sio.on('state_response')
def on_state(data):
    pwm = data.get('current_pwm', 0)
    gas = data.get('gas_pressed', False)
    gear = data.get('gear', 'N')
    print(f"\n📊 STATE: Speed={pwm:.0f}%, Gas={gas}, Gear={gear}")
    if pwm > 0 and gas:
        print("✅ SUCCESS: Car is accelerating!")
    else:
        print("❌ PROBLEM: Car is not accelerating!")

try:
    print("🔌 Connecting...")
    sio.connect('http://localhost:5000', wait_timeout=5)
    time.sleep(1)
except Exception as e:
    print(f"❌ Error: {e}")
