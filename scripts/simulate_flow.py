#!/usr/bin/env python3
"""
Simulate car control flow: 
1. Turn on car (shift to Gear 1)
2. Shift gear  
3. Accelerate
4. Apply emergency brakes
5. Deactivate emergency brakes
6. Accelerate again (fails)
"""

import socketio
import time
import sys
import threading

# Create a socket.io client
sio = socketio.Client(reconnection=True, reconnection_attempts=5, reconnection_delay=1)

# Track state
connected = False
flow_step = 0

@sio.event
def connect():
    global connected
    connected = True
    print("\n✅ Connected to server\n")
    
    # Start the flow
    time.sleep(0.5)
    run_flow()

@sio.event
def disconnect():
    print("\n❌ Disconnected from server")

@sio.on('gear_response')
def on_gear_response(data):
    print(f"📡 Gear Response: {data}")

@sio.on('throttle_response')
def on_throttle_response(data):
    print(f"📡 Throttle Response: {data}")

@sio.on('brake_response')
def on_brake_response(data):
    print(f"📡 Brake Response: {data}")

@sio.on('emergency_stop_response')
def on_emergency_stop_response(data):
    print(f"📡 Emergency Stop Response: {data}")

@sio.on('state_update')
def on_state_update(data):
    if 'current_pwm' in data:
        print(f"  State: Speed={data['current_pwm']:.1f}%, Gear={data['gear']}, Gas={data['gas_pressed']}, EmgBrake={data['emergency_brake_active']}")

def run_flow():
    global flow_step
    
    print("\n" + "="*60)
    print("🚗 SIMULATING CAR CONTROL FLOW")
    print("="*60)
    
    # Step 1: Turn on car (shift to Gear 1)
    print("\n[Step 1] Turn on car (shift to Gear 1)")
    print("-" * 40)
    sio.emit('gear_change', {'gear': '1'})
    time.sleep(1)
    
    # Step 2: Shift gear
    print("\n[Step 2] Shift gear to 2")
    print("-" * 40)
    sio.emit('gear_change', {'gear': '2'})
    time.sleep(1)
    
    # Step 3: Accelerate
    print("\n[Step 3] Accelerate (press throttle)")
    print("-" * 40)
    sio.emit('throttle', {'value': True})
    print("Waiting for acceleration...")
    time.sleep(3)  # Wait 3 seconds for acceleration
    
    # Step 4: Apply emergency brake
    print("\n[Step 4] Apply emergency brake")
    print("-" * 40)
    sio.emit('emergency_stop', {})
    time.sleep(1)
    
    # Step 5: Deactivate emergency brake
    print("\n[Step 5] Deactivate emergency brake")
    print("-" * 40)
    sio.emit('emergency_stop', {})
    time.sleep(1)
    
    # Step 6: Accelerate again
    print("\n[Step 6] Accelerate again (throttle already pressed?)")
    print("-" * 40)
    print("Sending throttle pressed again...")
    sio.emit('throttle', {'value': True})
    time.sleep(3)  # Wait 3 seconds to see if it accelerates
    
    print("\n" + "="*60)
    print("🏁 FLOW COMPLETE")
    print("="*60)
    
    # Disconnect after flow
    time.sleep(1)
    sio.disconnect()
    sys.exit(0)

# Connect to the server
try:
    print("🔌 Connecting to socket.io server at http://localhost:5000...")
    sio.connect('http://localhost:5000', transports=['websocket'], wait_timeout=10)
    
    # Wait for flow to complete
    while connected:
        time.sleep(0.1)
        
except Exception as e:
    print(f"❌ Connection failed: {e}")
    sys.exit(1)
