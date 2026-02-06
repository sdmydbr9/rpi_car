#!/usr/bin/env python3
"""
Test script: Auto-accelerate in gear 3, then test braking
Uses HTTP API instead of WebSockets
"""

import requests
import time
import sys

BASE_URL = "http://localhost:5000"

def api_call(endpoint, data=None):
    """Make an API call to the backend"""
    try:
        if data:
            response = requests.post(f"{BASE_URL}{endpoint}", json=data, timeout=5)
        else:
            response = requests.get(f"{BASE_URL}{endpoint}", timeout=5)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"❌ API call failed: {e}")
        return None

def get_state():
    """Request current car state"""
    return api_call("/api/state")

def print_state_update(data):
    """Print state in a formatted way"""
    if not data:
        return
    print(f"\n📊 STATE UPDATE:")
    print(f"   Gear: {data.get('gear', 'N/A')}")
    print(f"   Speed (PWM): {data.get('current_pwm', 0):.1f}%")
    print(f"   Gas Pressed: {data.get('gas_pressed', False)}")
    print(f"   Brake Pressed: {data.get('brake_pressed', False)}")
    print(f"   Auto-Accel Enabled: {data.get('auto_accel_enabled', False)}")
    print(f"   Is Braking: {data.get('is_braking', False)}")

def run_test():
    print("\n" + "="*70)
    print("🧪 BRAKE TEST: Auto-Accel Gear 3")
    print("="*70)
    
    # Test connection
    print("\n🔌 Testing connection to HTTP server at http://localhost:5000...")
    state = get_state()
    if state is None:
        print("❌ Could not connect to server. Make sure the backend is running.")
        sys.exit(1)
    print("✅ Connected to server\n")
    
    # Step 1: Shift to Gear 3
    print("\n[Step 1] Shift to Gear 3")
    print("-" * 70)
    api_call("/api/control/gear", {'gear': '3'})
    time.sleep(0.5)
    state = get_state()
    print_state_update(state)
    
    # Step 2: Enable auto-acceleration
    print("\n[Step 2] Enable auto-acceleration (auto-throttle)")
    print("-" * 70)
    api_call("/api/control/auto_accel_enable", {})
    time.sleep(0.5)
    state = get_state()
    print_state_update(state)
    
    # Step 3: Wait for acceleration
    print("\n[Step 3] Accelerating... waiting for speed to build up (5 seconds)")
    print("-" * 70)
    for i in range(10):  # Request state 10 times over 5 seconds
        time.sleep(0.5)
        state = get_state()
        if state:
            speed = state.get('current_pwm', 0)
            if speed > 50:  # If we've reached a good speed, we can test brakes
                print(f"\n✅ Good speed reached ({speed:.1f}%) - Ready to test brakes!")
                break
    
    # Step 4: Press the brakes
    print("\n[Step 4] PRESSING BRAKES - Testing brake functionality")
    print("-" * 70)
    api_call("/api/control/brake", {'value': True})
    time.sleep(0.3)
    state = get_state()
    print_state_update(state)
    
    if state:
        speed_before_brake = state.get('current_pwm', 0)
        print(f"   Speed before brake: {speed_before_brake:.1f}%")
    
    # Step 5: Monitor speed while brakes applied
    print("\n[Step 5] Monitoring speed while brakes applied (3 seconds)")
    print("-" * 70)
    for i in range(6):
        time.sleep(0.5)
        state = get_state()
        if state:
            current_speed = state.get('current_pwm', 0)
            is_braking = state.get('is_braking', False)
            print(f"   Iteration {i+1}: Speed = {current_speed:.1f}%, Is Braking = {is_braking}")
    
    # Step 6: Release brakes
    print("\n[Step 6] RELEASING BRAKES")
    print("-" * 70)
    api_call("/api/control/brake", {'value': False})
    time.sleep(0.3)
    state = get_state()
    print_state_update(state)
    
    # Final analysis
    print("\n" + "="*70)
    print("🔍 BRAKE TEST ANALYSIS")
    print("="*70)
    
    if state:
        final_speed = state.get('current_pwm', 0)
        brake_pressed = state.get('brake_pressed', False)
        is_braking = state.get('is_braking', False)
        
        print(f"\nFinal State:")
        print(f"  - Speed: {final_speed:.1f}%")
        print(f"  - Brake Pressed: {brake_pressed}")
        print(f"  - Is Braking: {is_braking}")
        
        if final_speed == 0:
            print(f"\n✅ SUCCESS: Brakes are working! Speed reduced to 0%")
        elif final_speed < speed_before_brake:
            print(f"\n⚠️ PARTIAL: Brakes reduced speed from {speed_before_brake:.1f}% to {final_speed:.1f}%")
            print(f"   But didn't fully stop. Check BRAKE_RATE in main.py")
        else:
            print(f"\n❌ FAILURE: Brakes are NOT WORKING! Speed stayed at {final_speed:.1f}%")
            print(f"   Auto-accel still active: {state.get('auto_accel_enabled', False)}")
            print(f"   Gas pressed: {state.get('gas_pressed', False)}")
    
    time.sleep(1)

try:
    run_test()
    print("\n✅ Test completed successfully")
except Exception as e:
    print(f"❌ Test failed: {e}")
    sys.exit(1)
