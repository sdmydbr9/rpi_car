#!/usr/bin/env python3
"""
Kokoro TTS Integration Test Script
Validates that all components are properly integrated
"""

import sys
import os

# Add the project root to the path
sys.path.insert(0, '/home/pi/rpi_car')

print("=" * 60)
print("🎤 Kokoro TTS Integration Test")
print("=" * 60)

# Test 1: Import kokoro_client
print("\n[1/6] Testing kokoro_client import...")
try:
    from kokoro_client import get_kokoro_client, KokoroTTSClient
    print("✅ kokoro_client imported successfully")
except Exception as e:
    print(f"❌ Failed to import kokoro_client: {e}")
    sys.exit(1)

# Test 2: Import narration with Kokoro support
print("[2/6] Testing narration.py with Kokoro support...")
try:
    from narration import NarrationEngine, get_tts_synthesizer
    print("✅ narration.py imports successfully")
except Exception as e:
    print(f"❌ Failed to import narration: {e}")
    sys.exit(1)

# Test 3: Check NarrationEngine has Kokoro methods
print("[3/6] Checking NarrationEngine has Kokoro methods...")
try:
    engine = NarrationEngine()
    assert hasattr(engine, 'set_kokoro_config'), "Missing set_kokoro_config method"
    assert hasattr(engine, '_kokoro_enabled'), "Missing _kokoro_enabled attribute"
    assert hasattr(engine, '_kokoro_ip'), "Missing _kokoro_ip attribute"
    assert hasattr(engine, '_kokoro_voice'), "Missing _kokoro_voice attribute"
    print("✅ NarrationEngine has Kokoro support")
except Exception as e:
    print(f"❌ NarrationEngine validation failed: {e}")
    sys.exit(1)

# Test 4: Test KokoroTTSClient initialization
print("[4/6] Testing KokoroTTSClient initialization...")
try:
    kokoro = get_kokoro_client()
    assert hasattr(kokoro, 'validate_api'), "Missing validate_api method"
    assert hasattr(kokoro, 'synthesize_and_stream'), "Missing synthesize_and_stream method"
    print("✅ KokoroTTSClient initialized and has required methods")
except Exception as e:
    print(f"❌ KokoroTTSClient test failed: {e}")
    sys.exit(1)

# Test 5: Verify main.py has Kokoro imports
print("[5/6] Checking main.py has Kokoro imports and handlers...")
try:
    with open('/home/pi/rpi_car/main.py', 'r') as f:
        content = f.read()
        assert 'from kokoro_client import get_kokoro_client' in content, "Missing kokoro_client import"
        assert "def on_kokoro_validate_api" in content, "Missing kokoro_validate_api handler"
        assert "def on_kokoro_config_update" in content, "Missing kokoro_config_update handler"
        assert "'kokoro_enabled'" in content, "Missing kokoro_enabled config"
        assert "'kokoro_ip'" in content, "Missing kokoro_ip config"
        assert "'kokoro_voice'" in content, "Missing kokoro_voice config"
    print("✅ main.py has Kokoro imports and Socket.IO handlers")
except Exception as e:
    print(f"❌ main.py validation failed: {e}")
    sys.exit(1)

# Test 6: Verify socketClient.ts has Kokoro events
print("[6/6] Checking socketClient.ts has Kokoro Socket.IO events...")
try:
    with open('/home/pi/rpi_car/src/lib/socketClient.ts', 'r') as f:
        content = f.read()
        assert 'emitKokoroValidateApi' in content, "Missing emitKokoroValidateApi function"
        assert 'emitKokoroConfigUpdate' in content, "Missing emitKokoroConfigUpdate function"
        assert 'onKokoroValidationResult' in content, "Missing onKokoroValidationResult listener"
        assert 'onKokoroConfigResponse' in content, "Missing onKokoroConfigResponse listener"
        assert 'kokoro_validate_api' in content, "Missing kokoro_validate_api event"
        assert 'kokoro_config_update' in content, "Missing kokoro_config_update event"
    print("✅ socketClient.ts has Kokoro Socket.IO events")
except Exception as e:
    print(f"❌ socketClient.ts validation failed: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("✅ All Integration Tests Passed!")
print("=" * 60)
print("\n📋 Summary of Kokoro TTS Integration:")
print("  • Backend: Flask/Socket.IO handlers for Kokoro validation and config")
print("  • Frontend: TypeScript Socket.IO client for Kokoro events")
print("  • UI: Settings panel with IP input, voice validation, and voice dropdown")
print("  • Narration Engine: Integrated Kokoro with fallback to local TTS")
print("  • Audio Streaming: Kokoro audio streams directly to Pi headphone jack")
print("\n🎯 Next Steps:")
print("  1. Deploy the changes to your Raspberry Pi")
print("  2. Start the backend server: python3 main.py")
print("  3. Open the web UI and navigate to Settings → AI NARRATION")
print("  4. Enter your Kokoro API address (e.g., 192.169.29.105:8880)")
print("  5. Click 'Validate' to fetch available voices")
print("  6. Select a voice and toggle Kokoro TTS ON")
print("  7. Enable Image Analysis to start using Kokoro TTS")
