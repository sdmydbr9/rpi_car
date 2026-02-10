#!/usr/bin/env python3
"""
Test: Verify the gear shifting bug with emergency brake is fixed.

Bug description:
  When the car is started and gears are shifted, the gear falls back to 
  neutral after using the emergency brake. The issue should be resolved 
  automatically when disengaging the emergency brake.

Expected behavior:
  1. Shift to gear 2
  2. Engage emergency brake - gear should go to N (neutral)
  3. Disengage emergency brake - gear should restore to 2
  4. Car should be able to accelerate in gear 2 without manual re-shifting

This test connects via SocketIO and verifies the gear restoration logic.
"""

import socketio
import time
import sys

sio = socketio.Client(reconnection=True, reconnection_attempts=3, reconnection_delay=1)

# Test state tracking
test_passed = True
current_state = {}

@sio.event
def connect():
    print("✅ Connected to server")

@sio.event
def disconnect():
    print("❌ Disconnected from server")

@sio.on('telemetry_update')
def on_telemetry(data):
    global current_state
    current_state = {
        'gear': data.get('gear', 'N'),
        'current_pwm': data.get('current_pwm', 0),
        'emergency_brake_active': data.get('emergency_brake_active', False),
        'gas_pressed': data.get('gas_pressed', False),
    }

@sio.on('gear_response')
def on_gear_response(data):
    print(f"   → Gear response: {data}")

@sio.on('emergency_stop_response')
def on_emergency_stop_response(data):
    print(f"   → Emergency brake response: {data}")

@sio.on('throttle_response')
def on_throttle_response(data):
    print(f"   → Throttle response: {data}")

def wait_and_get_state(delay=0.5):
    """Wait a bit and return current state"""
    time.sleep(delay)
    return current_state.copy()

def run_test():
    global test_passed
    
    print("\n" + "=" * 70)
    print("🧪 EMERGENCY BRAKE GEAR RESTORATION TEST")
    print("=" * 70)
    
    # Step 1: Shift to gear 2
    print("\n[Step 1] Shift to gear 2")
    print("-" * 70)
    sio.emit('gear_change', {'gear': '2'})
    state = wait_and_get_state()
    
    if state.get('gear') == '2':
        print(f"   ✅ PASS — Gear successfully changed to 2")
    else:
        print(f"   ❌ FAIL — Gear is {state.get('gear')}, expected 2")
        test_passed = False
    
    # Step 2: Engage emergency brake
    print("\n[Step 2] Engage emergency brake")
    print("-" * 70)
    sio.emit('emergency_stop', {})
    state = wait_and_get_state()
    
    if state.get('emergency_brake_active') and state.get('gear') == 'N':
        print(f"   ✅ PASS — Emergency brake ON, gear set to N (neutral)")
    else:
        print(f"   ❌ FAIL — E-brake: {state.get('emergency_brake_active')}, Gear: {state.get('gear')}")
        test_passed = False
    
    # Step 3: Disengage emergency brake - THE CRITICAL TEST
    print("\n[Step 3] Disengage emergency brake (CRITICAL TEST)")
    print("-" * 70)
    sio.emit('emergency_stop', {})
    state = wait_and_get_state()
    
    if not state.get('emergency_brake_active') and state.get('gear') == '2':
        print(f"   ✅ PASS — Emergency brake OFF, gear RESTORED to 2")
        print(f"   🎉 BUG FIX VERIFIED! Gear automatically restored after e-brake release")
    else:
        print(f"   ❌ FAIL — E-brake: {state.get('emergency_brake_active')}, Gear: {state.get('gear')} (expected 2)")
        print(f"   🐛 BUG STILL EXISTS! Gear should have been restored to 2")
        test_passed = False
    
    # Step 4: Verify car can accelerate in restored gear
    print("\n[Step 4] Verify acceleration works in restored gear")
    print("-" * 70)
    sio.emit('throttle', {'value': True})
    time.sleep(1.5)  # Wait for acceleration
    state = wait_and_get_state()
    
    if state.get('gas_pressed') and state.get('gear') == '2':
        print(f"   ✅ PASS — Throttle active in gear 2")
        print(f"   Current speed: {state.get('current_pwm', 0):.1f}%")
    else:
        print(f"   ❌ FAIL — Gas: {state.get('gas_pressed')}, Gear: {state.get('gear')}")
        test_passed = False
    
    # Clean up - release throttle
    sio.emit('throttle', {'value': False})
    time.sleep(0.3)
    
    # Summary
    print("\n" + "=" * 70)
    print("📊 TEST SUMMARY")
    print("=" * 70)
    
    if test_passed:
        print("   ✅ ALL TESTS PASSED")
        print("   🎉 Emergency brake gear restoration bug is FIXED!")
    else:
        print("   ❌ SOME TESTS FAILED")
        print("   🐛 Emergency brake gear restoration bug may still exist")
    
    print("=" * 70)
    
    return test_passed

# Main
if __name__ == '__main__':
    try:
        print("🔌 Connecting to server at localhost:5000 ...")
        sio.connect('http://localhost:5000', wait_timeout=5)
        time.sleep(1)  # Let telemetry start flowing
        
        passed = run_test()
        
        sio.disconnect()
        sys.exit(0 if passed else 1)
    except Exception as e:
        print(f"❌ Connection error: {e}")
        sys.exit(1)
