#!/usr/bin/env python3
"""
Motor Power Test Script
Tests different power levels to find the minimum needed to move the car in reverse.
Run this to diagnose if the issue is power-related.
"""

import time
import sys
import os

# Add the current directory to Python path to import main modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("❌ RPi.GPIO not available - running in simulation mode")
    GPIO_AVAILABLE = False
else:
    GPIO_AVAILABLE = True

# Motor pin configuration (same as main.py)
IN1 = 17; IN2 = 27  # Left Motor
IN3 = 22; IN4 = 23  # Right Motor  
ENA = 18; ENB = 19  # Speed PWM

def setup_gpio():
    """Initialize GPIO pins"""
    if not GPIO_AVAILABLE:
        print("🔧 Simulation mode - no actual GPIO control")
        return None, None
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
        
        # Setup PWM
        pwm_a = GPIO.PWM(ENA, 1000)
        pwm_b = GPIO.PWM(ENB, 1000)
        pwm_a.start(0)
        pwm_b.start(0)
        
        print("✅ GPIO setup successful")
        return pwm_a, pwm_b
    except Exception as e:
        print(f"❌ GPIO setup failed: {e}")
        return None, None

def test_reverse_power(pwm_a, pwm_b, power_level, duration=2.0):
    """Test reverse movement at specific power level"""
    print(f"\n🔧 Testing {power_level}% power in reverse for {duration}s...")
    
    if not GPIO_AVAILABLE:
        print(f"   (Simulation: Would set reverse motors to {power_level}% power)")
        return
    
    try:
        # Set reverse direction (IN1=IN3=True, IN2=IN4=False for reverse)
        GPIO.output(IN1, True)
        GPIO.output(IN2, False) 
        GPIO.output(IN3, True)
        GPIO.output(IN4, False)
        
        # Apply power
        pwm_a.ChangeDutyCycle(power_level)
        pwm_b.ChangeDutyCycle(power_level)
        
        print(f"   📊 Motors: L={power_level}% R={power_level}% REVERSE")
        print(f"   ⏱️  Running for {duration} seconds...")
        
        time.sleep(duration)
        
        # Stop motors
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        GPIO.output(IN1, False)
        GPIO.output(IN2, False)
        GPIO.output(IN3, False) 
        GPIO.output(IN4, False)
        
        print(f"   ⏹️  Stopped")
        
    except Exception as e:
        print(f"   ❌ Error: {e}")

def cleanup(pwm_a, pwm_b):
    """Clean up GPIO resources"""
    if GPIO_AVAILABLE and pwm_a and pwm_b:
        try:
            pwm_a.stop()
            pwm_b.stop()
            GPIO.cleanup()
            print("✅ GPIO cleanup complete")
        except:
            pass

def main():
    print("🚗 RPi Car Motor Power Test")
    print("=" * 40)
    print("This script tests different power levels to find")
    print("the minimum needed for reverse movement.")
    print()
    
    # Setup GPIO
    pwm_a, pwm_b = setup_gpio()
    
    try:
        print("⚠️  Make sure the car is on a clear surface!")
        print("⚠️  Remove any obstacles around the car!")
        input("\nPress Enter to start testing...")
        
        # Test power levels from lowest to highest
        test_levels = [10, 15, 20, 25, 30, 35, 40, 45, 50, 60]
        
        for power in test_levels:
            test_reverse_power(pwm_a, pwm_b, power, 1.5)
            
            response = input(f"   Did the car move at {power}%? (y/n): ").lower()
            if response == 'y':
                print(f"✅ SUCCESS: Car moves at {power}% power!")
                print(f"📝 Recommended REVERSE_POWER setting: {power}")
                break
            elif response == 'n':
                print(f"   ❌ No movement at {power}%")
            else:
                print("   ⏭️  Skipping to next power level")
        
        print("\n🧪 Additional test: Try current obstacle avoidance setting")
        current_setting = 40  # This should match REVERSE_POWER in main.py
        test_reverse_power(pwm_a, pwm_b, current_setting, 2.0)
        print(f"📊 Current obstacle avoidance uses {current_setting}% power")
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test error: {e}")
    finally:
        cleanup(pwm_a, pwm_b)
        print("\n🏁 Test complete!")

if __name__ == "__main__":
    main()
