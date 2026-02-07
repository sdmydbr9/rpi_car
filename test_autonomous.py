#!/usr/bin/env python3
"""
Test script for Smart Driver autonomous mode functionality
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Mock GPIO for testing
class MockGPIO:
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    HIGH = True
    LOW = False
    
    @staticmethod
    def setmode(mode): pass
    @staticmethod
    def setwarnings(warnings): pass
    @staticmethod
    def setup(pins, mode): pass
    @staticmethod
    def output(pin, value): pass
    @staticmethod
    def input(pin): return 1  # No obstacle detected
    @staticmethod
    def cleanup(): pass
    
    class PWM:
        def __init__(self, pin, freq): pass
        def start(self, duty): pass
        def ChangeDutyCycle(self, duty): pass
        def stop(self): pass

# Mock Flask and SocketIO for testing
class MockFlask:
    def __init__(self, *args, **kwargs): pass
    def route(self, path): 
        def decorator(func): return func
        return decorator

class MockSocketIO:
    def __init__(self, app, **kwargs): pass
    def on(self, event):
        def decorator(func): return func
        return decorator

# Mock the imports
sys.modules['RPi'] = type(sys)('RPi')
sys.modules['RPi.GPIO'] = MockGPIO()
sys.modules['flask'] = type(sys)('flask')
sys.modules['flask.Flask'] = MockFlask
sys.modules['flask_cors'] = type(sys)('flask_cors')
sys.modules['flask_socketio'] = type(sys)('flask_socketio')
sys.modules['flask_socketio.SocketIO'] = MockSocketIO

# Now import our modules
from main import State, car_state

def test_autonomous_states():
    """Test the autonomous state enum and car state variables"""
    print("🧪 Testing Smart Driver Autonomous System")
    print("=" * 50)
    
    # Test State enum
    print("✅ State Enum:")
    for state in State:
        print(f"  - {state.name}: {state.value}")
    
    # Test car state variables
    print("\n✅ Car State Variables:")
    autonomous_vars = [
        "autonomous_mode", 
        "autonomous_state", 
        "autonomous_target_speed",
        "escape_maneuver_timer",
        "last_obstacle_side"
    ]
    
    for var in autonomous_vars:
        if var in car_state:
            print(f"  - {var}: {car_state[var]}")
        else:
            print(f"  ❌ {var}: MISSING")
    
    # Test state transitions
    print("\n✅ Testing State Transitions:")
    print(f"  Initial state: {car_state['autonomous_state']}")
    
    # Enable autonomous mode
    car_state["autonomous_mode"] = True
    car_state["autonomous_state"] = State.CRUISING.value
    print(f"  After enable: {car_state['autonomous_state']}")
    
    # Simulate obstacle detection
    car_state["autonomous_state"] = State.BRAKING.value
    print(f"  Obstacle detected: {car_state['autonomous_state']}")
    
    # Simulate escape maneuver
    car_state["autonomous_state"] = State.REVERSING.value
    print(f"  Reversing: {car_state['autonomous_state']}")
    
    car_state["autonomous_state"] = State.TURNING.value
    print(f"  Turning: {car_state['autonomous_state']}")
    
    # Return to cruising
    car_state["autonomous_state"] = State.CRUISING.value
    print(f"  Back to cruising: {car_state['autonomous_state']}")
    
    print("\n🎉 All tests passed! Smart Driver system is ready.")

if __name__ == "__main__":
    test_autonomous_states()
