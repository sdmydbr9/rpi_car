#!/usr/bin/env python3
"""
Quick test to verify Kokoro client timeout fixes
"""
import sys
sys.path.insert(0, '/home/pi/rpi_car')

from kokoro_client import get_kokoro_client
import time

print("=" * 60)
print("🎤 Kokoro Client Timeout Test")
print("=" * 60)

client = get_kokoro_client()

# Test 1: Invalid IP (should timeout quickly, not hang)
print("\n[Test 1] Invalid IP — should fail quickly (not hang forever)...")
start = time.time()
result = client.validate_api("192.0.2.1:8880")  # TEST-NET-1 (unreachable)
elapsed = time.time() - start
print(f"  Result: {result}")
print(f"  Time: {elapsed:.2f}s (should be < 10s)")
if elapsed < 10:
    print("  ✅ Timeout working correctly")
else:
    print("  ❌ Took too long!")

# Test 2: Localhost (will fail but quickly)
print("\n[Test 2] Localhost (should timeout quickly)...")
start = time.time()
result = client.validate_api("127.0.0.1:8880")
elapsed = time.time() - start
print(f"  Result: {result}")
print(f"  Time: {elapsed:.2f}s (should be < 10s)")
if elapsed < 10:
    print("  ✅ Timeout working correctly")
else:
    print("  ❌ Took too long!")

print("\n" + "=" * 60)
print("✅ Timeout Tests Complete")
print("=" * 60)
print("\n✨ Fixes Applied:")
print("  • Separate connect (5s) and read (15s) timeouts")
print("  • JSON parsing with error handling")
print("  • Streaming chunk iteration with proper error handling")
print("  • stdin flush for reliable MP3 streaming")
print("  • Proper subprocess termination on stop()")
