"""
test_motors.py — Quick motor spin test with detailed GPIO logging.

Tests ENA (BCM 12, Physical Pin 32) and ENB (BCM 13, Physical Pin 33)
along with direction pins IN1-IN4.
"""

import time
import sys

print("=" * 60)
print("🔧 MOTOR PIN TEST")
print("=" * 60)

# ── GPIO Setup ──────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    print("✅ RPi.GPIO imported successfully")
    print(f"   RPi.GPIO version: {GPIO.VERSION}")
except ImportError as e:
    print(f"❌ RPi.GPIO import failed: {e}")
    sys.exit(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
print(f"✅ GPIO mode set to BCM")

# ── Pin Definitions ─────────────────────────────────────────
IN1 = 17   # Left Motor Direction A
IN2 = 27   # Left Motor Direction B
IN3 = 22   # Right Motor Direction A
IN4 = 23   # Right Motor Direction B
ENA = 12   # Left Motor Speed (PWM)  — Physical Pin 32
ENB = 13   # Right Motor Speed (PWM) — Physical Pin 33

PWM_FREQ = 1000  # Hz

pins = {
    "IN1 (Left Dir A)":  IN1,
    "IN2 (Left Dir B)":  IN2,
    "IN3 (Right Dir A)": IN3,
    "IN4 (Right Dir B)": IN4,
    "ENA (Left PWM)":    ENA,
    "ENB (Right PWM)":   ENB,
}

print("\n📌 Pin Configuration:")
for label, pin in pins.items():
    print(f"   {label:<22} → BCM {pin}")

# ── Setup all pins as OUTPUT ────────────────────────────────
print("\n⚙️  Setting up GPIO pins...")
try:
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
    print("✅ All motor pins configured as OUTPUT")
except Exception as e:
    print(f"❌ Pin setup failed: {e}")
    GPIO.cleanup()
    sys.exit(1)

# ── Create PWM channels ────────────────────────────────────
print(f"\n⚙️  Creating PWM on ENA (BCM {ENA}) and ENB (BCM {ENB}) at {PWM_FREQ} Hz...")
try:
    pwm_a = GPIO.PWM(ENA, PWM_FREQ)
    pwm_b = GPIO.PWM(ENB, PWM_FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
    print("✅ PWM channels created and started at 0% duty cycle")
except Exception as e:
    print(f"❌ PWM creation failed: {e}")
    GPIO.cleanup()
    sys.exit(1)

# ── Helper ──────────────────────────────────────────────────
def set_forward():
    """Set direction pins for forward motion."""
    GPIO.output([IN1, IN3], False)   # IN_A = LOW
    GPIO.output([IN2, IN4], True)    # IN_B = HIGH
    print("   Direction: FORWARD (IN1=LOW, IN2=HIGH, IN3=LOW, IN4=HIGH)")

def set_reverse():
    """Set direction pins for reverse motion."""
    GPIO.output([IN1, IN3], True)    # IN_A = HIGH
    GPIO.output([IN2, IN4], False)   # IN_B = LOW
    print("   Direction: REVERSE (IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW)")

def stop_motors():
    """Stop both motors."""
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], False)
    print("   ⏹  Motors stopped")

# ── Test Sequence ───────────────────────────────────────────
print("\n" + "=" * 60)
print("🏁 STARTING MOTOR TEST SEQUENCE")
print("=" * 60)

try:
    # Test 1: Left motor only — forward
    print("\n── Test 1: LEFT motor forward (50% → 100%) ──")
    set_forward()
    for duty in [50, 75, 100]:
        pwm_a.ChangeDutyCycle(duty)
        pwm_b.ChangeDutyCycle(0)
        print(f"   ENA={duty}%, ENB=0%  — Left motor should spin")
        time.sleep(1.5)
    stop_motors()
    time.sleep(0.5)

    # Test 2: Right motor only — forward
    print("\n── Test 2: RIGHT motor forward (50% → 100%) ──")
    set_forward()
    for duty in [50, 75, 100]:
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(duty)
        print(f"   ENA=0%, ENB={duty}%  — Right motor should spin")
        time.sleep(1.5)
    stop_motors()
    time.sleep(0.5)

    # Test 3: Both motors — forward at increasing speed
    print("\n── Test 3: BOTH motors forward (25% → 50% → 75% → 100%) ──")
    set_forward()
    for duty in [25, 50, 75, 100]:
        pwm_a.ChangeDutyCycle(duty)
        pwm_b.ChangeDutyCycle(duty)
        print(f"   ENA={duty}%, ENB={duty}%  — Both motors should spin")
        time.sleep(2)
    stop_motors()
    time.sleep(0.5)

    # Test 4: Both motors — reverse
    print("\n── Test 4: BOTH motors reverse at 75% ──")
    set_reverse()
    pwm_a.ChangeDutyCycle(75)
    pwm_b.ChangeDutyCycle(75)
    print(f"   ENA=75%, ENB=75%  — Both motors should spin in reverse")
    time.sleep(2)
    stop_motors()

    print("\n" + "=" * 60)
    print("✅ MOTOR TEST COMPLETE")
    print("=" * 60)

except KeyboardInterrupt:
    print("\n\n⚠️  Test interrupted by user")

except Exception as e:
    print(f"\n❌ Error during test: {e}")
    import traceback
    traceback.print_exc()

finally:
    print("\n🧹 Cleaning up GPIO...")
    stop_motors()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("✅ GPIO cleaned up. Done.")
