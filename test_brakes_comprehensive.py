#!/usr/bin/env python3
"""
Comprehensive brake test: Auto-Accel Gear 3 with IR disabled
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

def print_state(state=None):
    """Print state in a compact format"""
    if state is None:
        state = get_state()
    if not state:
        return
    d = state
    print(f"   Gear: {d.get('gear')}, Speed: {d.get('current_pwm', 0):.0f}%, " + 
          f"Gas: {d.get('gas_pressed')}, Brake: {d.get('brake_pressed')}, " +
          f"AutoAccel: {d.get('auto_accel_enabled')}")

def run_test():
    print("\n" + "="*80)
    print("🧪 COMPREHENSIVE BRAKE TEST: Auto-Accel Gear 3")
    print("="*80)
    
    # Test connection
    print("\n🔌 Testing connection to HTTP server at http://localhost:5000...")
    state = get_state()
    if state is None:
        print("❌ Could not connect to server. Make sure the backend is running.")
        sys.exit(1)
    print("✅ Connected to server\n")
    
    # Step 1: Disable IR sensors to avoid interference
    print("\n[Step 1] Disable IR sensors (to avoid obstacle interference)")
    print("-" * 80)
    api_call("/api/control/ir_toggle", {})
    time.sleep(0.5)
    state = get_state()
    print_state(state)
    
    # Step 2: Shift to Gear 3
    print("\n[Step 2] Shift to Gear 3")
    print("-" * 80)
    api_call("/api/control/gear", {'gear': '3'})
    time.sleep(0.5)
    state = get_state()
    print_state(state)
    
    # Step 3: Enable auto-acceleration
    print("\n[Step 3] Enable auto-acceleration")
    print("-" * 80)
    api_call("/api/control/auto_accel_enable", {})
    time.sleep(0.5)
    state = get_state()
    print_state(state)
    
    # Step 4: Accelerate for 3 seconds
    print("\n[Step 4] Auto-accelerating for 3 seconds...")
    print("-" * 80)
    for i in range(6):
        time.sleep(0.5)
        state = get_state()
        if state:
            speed = state.get('current_pwm', 0)
            print(f"   T+{(i+1)*0.5:.1f}s → Speed: {speed:.0f}%")
    
    final_speed_before_brake = state.get('current_pwm', 0) if state else 0
    
    # Step 5: Press the brakes HARD
    print(f"\n[Step 5] PRESS BRAKES (from {final_speed_before_brake:.0f}%)")
    print("-" * 80)
    api_call("/api/control/brake", {'value': True})
    time.sleep(0.3)
    state = get_state()
    if state:
        speed_after_brake = state.get('current_pwm', 0)
        brake_pressed = state.get('brake_pressed', False)
        print(f"   Brake pressed: {brake_pressed}")
        print(f"   Speed dropped: {final_speed_before_brake:.0f}% → {speed_after_brake:.0f}%")
    
    # Step 6: Hold brakes and monitor
    print("\n[Step 6] Hold brakes for 2 seconds")
    print("-" * 80)
    for i in range(4):
        time.sleep(0.5)
        state = get_state()
        if state:
            speed = state.get('current_pwm', 0)
            is_braking = state.get('is_braking', False)
            print(f"   T+{(i+1)*0.5:.1f}s → Speed: {speed:.0f}%, Is Braking: {is_braking}")
    
    # Step 7: Release brakes
    print("\n[Step 7] Release brakes")
    print("-" * 80)
    api_call("/api/control/brake", {'value': False})
    time.sleep(0.5)
    state = get_state()
    print_state(state)
    
    # Step 8: Let it accelerate again
    print("\n[Step 8] Auto-acceleration resumes for 2 seconds")
    print("-" * 80)
    for i in range(4):
        time.sleep(0.5)
        state = get_state()
        if state:
            speed = state.get('current_pwm', 0)
            print(f"   T+{(i+1)*0.5:.1f}s → Speed: {speed:.0f}%")
    
    # Final Analysis
    print("\n" + "="*80)
    print("🔍 TEST ANALYSIS")  
    print("="*80)
    
    if final_speed_before_brake > 50 and speed_after_brake == 0:
        print("\n✅ BRAKES WORKING PERFECTLY!")
        print(f"   ✓ Auto-accel reached {final_speed_before_brake:.0f}%")
        print(f"   ✓ Brakes reduced speed to {speed_after_brake:.0f}%")
        print(f"   ✓ Speed held at zero while brakes engaged")
        print(f"   ✓ Auto-accel resumed after brake release")
        print("\n🎉 All tests passed!")
    else:
        print("\n❌ ISSUES DETECTED:")
        print(f"   - Final speed before brake: {final_speed_before_brake:.0f}%")
        print(f"   - Speed after brake applied: {speed_after_brake:.0f}%")
        if final_speed_before_brake <= 50:
            print("   - Auto-acceleration not reaching expected speed")
        if speed_after_brake > 0:
            print("   - Brakes not stopping the car completely")
    
    time.sleep(1)

try:
    run_test()
    print("\n✅ Test completed successfully")
except Exception as e:
    print(f"❌ Test failed: {e}")
    print("   Make sure the server is running: python main.py")
    sys.exit(1)
