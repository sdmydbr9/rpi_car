#!/usr/bin/env python3
"""
Test that simulates realistic UI behavior: press brake, hold for a bit, then release
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

def print_state(current_state, label=""):
    """Print state in a compact format"""
    if not current_state:
        return
    d = current_state
    msg = f"   Speed: {d.get('current_pwm', 0):.0f}%, Gas: {d.get('gas_pressed')}, Brake: {d.get('brake_pressed')}, IsBraking: {d.get('is_braking')}"
    if label:
        print(f"{label} {msg}")
    else:
        print(f"   {msg}")

def run_test():
    print("\n" + "="*80)
    print("🧪 UI BEHAVIOR SIMULATION: Brake Press & Hold Test")
    print("="*80)
    
    # Test connection
    print("\n🔌 Testing connection to HTTP server at http://localhost:5000...")
    state = get_state()
    if state is None:
        print("❌ Could not connect to server. Make sure the backend is running.")
        sys.exit(1)
    print("✅ Connected to server\n")
    
    # Step 1: Set up gear 3
    print("\n[Step 1] Set up: Shift to Gear 3")
    api_call("/api/control/gear", {'gear': '3'})
    time.sleep(0.3)
    state = get_state()
    print_state(state, "Before acceleration:")
    
    # Step 2: Press throttle (like holding the gas button)
    print("\n[Step 2] PRESS throttle (simulating button press & hold)")
    print("-" * 80)
    api_call("/api/control/throttle", {'value': True})
    
    # Step 3: Let it accelerate for 2 seconds while throttle is held
    print("Throttle pressed - waiting for acceleration (2 seconds)...")
    for i in range(4):
        time.sleep(0.5)
        state = get_state()
        print_state(state, f"  {(i+1)*0.5:.1f}s:")
    
    # Step 4: RELEASE throttle
    print("\n[Step 3] RELEASE throttle")
    api_call("/api/control/throttle", {'value': False})
    state = get_state()
    print_state(state, "After throttle release:")
    
    # Step 5: Now simulate brake press and hold
    print("\n[Step 4] PRESS brake (simulating button press & hold)")
    print("-" * 80)
    print(f"Speed before brake: {state.get('current_pwm', 0):.0f}%" if state else "Unknown")
    
    api_call("/api/control/brake", {'value': True})
    state = get_state()
    print_state(state, "  Brake just pressed:")
    
    # Step 6: HOLD brake for 3 seconds
    print("Brake held - monitoring speed over 3 seconds...")
    for i in range(6):
        time.sleep(0.5)
        state = get_state()
        if state:
            speed = state.get('current_pwm', 0)
            is_braking = state.get('is_braking', False)
            print(f"    {(i+1)*0.5:.1f}s: Speed = {speed:.0f}%, Is Braking = {is_braking}")
    
    speed_while_braking = state.get('current_pwm', 0) if state else 0
    
    # Step 7: RELEASE brake
    print("\n[Step 5] RELEASE brake")
    api_call("/api/control/brake", {'value': False})
    state = get_state()
    print_state(state, "After brake release:")
    
    # Final analysis
    print("\n" + "="*80)
    print("🔍 ANALYSIS")  
    print("="*80)
    
    if speed_while_braking == 0:
        print("\n✅ BRAKE WORKS CORRECTLY!")
        print("   ✓ Speed reduced to 0% while brake was held")
        print("   ✓ Indicates continuous brake is being maintained by the server")
    else:
        print(f"\n⚠️ BRAKE BEHAVIOR:")
        print(f"   Speed while brake held: {speed_while_braking:.0f}%")
        if speed_while_braking > 0:
            print("   The brake is NOT fully applied - car still has speed")
    
    time.sleep(1)

try:
    run_test()
    print("\n✅ Test completed successfully")
except Exception as e:
    print(f"❌ Test failed: {e}")
    sys.exit(1)
