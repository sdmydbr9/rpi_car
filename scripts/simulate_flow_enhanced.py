#!/usr/bin/env python3
"""
Enhanced simulation with state monitoring
"""

import socketio
import time
import sys

sio = socketio.Client(reconnection=True, reconnection_attempts=5, reconnection_delay=1)

connected = False
current_state = {}

@sio.event
def connect():
    global connected
    connected = True
    print("\n✅ Connected to server\n")
    time.sleep(0.5)
    run_flow()

@sio.event
def disconnect():
    print("\n❌ Disconnected from server")

@sio.on('state_response')
def on_state_response(data):
    global current_state
    current_state = data
    print(f"\n📊 STATE UPDATE:")
    print(f"   Gear: {data.get('gear', 'N/A')}")
    print(f"   Speed (PWM): {data.get('current_pwm', 0):.1f}%")
    print(f"   Gas Pressed: {data.get('gas_pressed', False)}")
    print(f"   Brake Pressed: {data.get('brake_pressed', False)}")
    print(f"   Emergency Brake Active: {data.get('emergency_brake_active', False)}")
    print(f"   Obstacle State: {data.get('obstacle_state', 'N/A')}")

@sio.on('gear_response')
def on_gear_response(data):
    print(f"   → Gear Response: {data}")

@sio.on('throttle_response')
def on_throttle_response(data):
    print(f"   → Throttle Response: {data}")

@sio.on('brake_response')
def on_brake_response(data):
    print(f"   → Brake Response: {data}")

@sio.on('emergency_stop_response')
def on_emergency_stop_response(data):
    print(f"   → Emergency Stop Response: {data}")

def request_state():
    """Request current car state"""
    sio.emit('state_request', {})
    time.sleep(0.2)

def run_flow():
    global connected
    
    print("\n" + "="*70)
    print("🚗 SIMULATING CAR CONTROL FLOW WITH STATE MONITORING")
    print("="*70)
    
    # Step 1: Turn on car
    print("\n[Step 1] Turn on car (shift to Gear 1)")
    print("-" * 70)
    sio.emit('gear_change', {'gear': '1'})
    time.sleep(0.5)
    request_state()
    
    # Step 2: Shift gear  
    print("\n[Step 2] Shift gear to 2")
    print("-" * 70)
    sio.emit('gear_change', {'gear': '2'})
    time.sleep(0.5)
    request_state()
    
    # Step 3: Accelerate
    print("\n[Step 3] Accelerate (press throttle)")
    print("-" * 70)
    sio.emit('throttle', {'value': True})
    time.sleep(0.5)
    print("   Waiting for acceleration (2 seconds)...")
    for i in range(4):  # Request state 4 times over 2 seconds
        time.sleep(0.5)
        request_state()
    
    # Step 4: Apply emergency brake
    print("\n[Step 4] Apply emergency brake")
    print("-" * 70)
    sio.emit('emergency_stop', {})
    time.sleep(0.5)
    request_state()
    
    # Step 5: Deactivate emergency brake
    print("\n[Step 5] Deactivate emergency brake")
    print("-" * 70)
    sio.emit('emergency_stop', {})
    time.sleep(0.5)
    request_state()
    
    # Step 6: Accelerate again
    print("\n[Step 6] Accelerate again (check if throttle still works)")
    print("-" * 70)
    sio.emit('throttle', {'value': True})
    time.sleep(0.5)
    print("   Waiting for acceleration (2 seconds)...")
    for i in range(4):  # Request state 4 times over 2 seconds
        time.sleep(0.5)
        request_state()
    
    print("\n" + "="*70)
    print("🏁 FLOW COMPLETE - ANALYSIS")
    print("="*70)
    print(f"\nFinal State:")
    print(f"  - Speed: {current_state.get('current_pwm', 0):.1f}%")
    print(f"  - Emergency Brake: {current_state.get('emergency_brake_active', False)}")
    print(f"  - Throttle: {'PRESSED' if current_state.get('gas_pressed') else 'NOT PRESSED'}")
    
    if current_state.get('current_pwm', 0) == 0 and current_state.get('gas_pressed'):
        print(f"\n❌ BUG FOUND: Throttle is pressed but speed is 0%!")
        print(f"   Emergency brake off? {not current_state.get('emergency_brake_active', False)}")
    
    time.sleep(1)
    sio.disconnect()
    sys.exit(0)

try:
    print("🔌 Connecting to socket.io server at http://localhost:5000...")
    sio.connect('http://localhost:5000', transports=['websocket'], wait_timeout=10)
    
    while connected:
        time.sleep(0.1)
        
except Exception as e:
    print(f"❌ Connection failed: {e}")
    sys.exit(1)
