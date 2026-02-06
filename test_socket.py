#!/usr/bin/env python3
"""Test script to verify HTTP API connection and commands"""

import requests
import time
import sys

BASE_URL = "http://127.0.0.1:5000"

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

def test_api():
    print(f"Attempting to connect to {BASE_URL}...")
    
    # Test connection
    response = api_call("/api/state")
    if response is None:
        print(f"❌ Failed to connect to {BASE_URL}")
        sys.exit(1)
    
    print("✅ Connected to server\n")
    
    # Test throttle command
    print("   Sending throttle command...")
    result = api_call("/api/control/throttle", {'value': True})
    if result:
        print(f"📡 Throttle Response: {result}")
    time.sleep(0.5)
    
    # Test steering command
    print("   Sending steering command...")
    result = api_call("/api/control/steering", {'angle': 45})
    if result:
        print(f"📡 Steering Response: {result}")
    time.sleep(0.5)
    
    # Test brake command
    print("   Sending brake command...")
    result = api_call("/api/control/brake", {'value': True})
    if result:
        print(f"📡 Brake Response: {result}")
    time.sleep(0.5)
    
    # Test gear command
    print("   Sending gear command...")
    result = api_call("/api/control/gear", {'gear': '3'})
    if result:
        print(f"📡 Gear Response: {result}")
    time.sleep(0.5)
    
    # Get telemetry
    print("   Getting telemetry...")
    result = api_call("/api/telemetry")
    if result:
        print(f"📡 Telemetry: {result}")
    time.sleep(0.5)
    
    print("\n✅ All tests completed successfully")

try:
    test_api()
except Exception as e:
    print(f"❌ Test failed: {e}")
    sys.exit(1)
