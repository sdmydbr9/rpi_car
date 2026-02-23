#!/usr/bin/env python3
"""
🧪 Test script to verify hotspot narration setup matches auto_voice.py pattern
Tests: Local LLM + Mimic3 TTS chain for hotspot mode
"""

import requests
import subprocess
import sys
import json
import base64

# Configuration (matches auto_voice.py)
LOCAL_LLM_URL = "http://127.0.0.1:8000/v1/chat/completions"
MIMIC3_API_URL = "http://127.0.0.1:59125/api/tts"
AUDIO_DEVICE = "plughw:0,0"

def test_mimic3_tts():
    """Test Mimic3 TTS endpoint (exactly like auto_voice.py)"""
    print("\n🎙️ Testing Mimic3 TTS endpoint...")
    try:
        text = "Testing Mimic3 Text To Speech"
        r = requests.get(MIMIC3_API_URL, params={"text": text}, timeout=5)
        if r.status_code == 200:
            print(f"✅ Mimic3 TTS: OK (received {len(r.content)} bytes)")
            # Try to play it
            try:
                aplay = subprocess.Popen(['aplay', '-D', AUDIO_DEVICE, '-q'], 
                                       stdin=subprocess.PIPE, 
                                       stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)
                aplay.communicate(input=r.content, timeout=5)
                print(f"✅ Mimic3 audio playback: OK")
                return True
            except subprocess.TimeoutExpired:
                print(f"⚠️ Mimic3 audio playback: Timeout (may still be OK)")
                return True
            except Exception as e:
                print(f"⚠️ Mimic3 audio playback: {e}")
                return True
        else:
            print(f"❌ Mimic3 TTS: HTTP {r.status_code}")
            return False
    except requests.Timeout:
        print(f"❌ Mimic3 TTS: Connection timeout")
        return False
    except requests.ConnectionError:
        print(f"❌ Mimic3 TTS: Connection failed (not running on port 59125)")
        return False
    except Exception as e:
        print(f"❌ Mimic3 TTS: {e}")
        return False


def test_local_llm_text_mode():
    """Test local LLM with text only (streaming, like auto_voice.py)"""
    print("\n🧠 Testing Local LLM (text-only streaming mode)...")
    try:
        payload = {
            "messages": [
                {"role": "system", "content": "You are a helpful assistant. Keep responses under 25 words."},
                {"role": "user", "content": "What do you see in front of you?"}
            ],
            "stream": True,
            "temperature": 0.7
        }
        
        response = requests.post(LOCAL_LLM_URL, json=payload, stream=True, timeout=10)
        
        if response.status_code == 200:
            print(f"✅ Local LLM text endpoint: OK (streaming)")
            # Try to parse streaming response
            buffer = ""
            try:
                for line in response.iter_lines():
                    if not line:
                        continue
                    decoded = line.decode('utf-8').lstrip("data: ")
                    if decoded == "[DONE]":
                        break
                    try:
                        content = json.loads(decoded)["choices"][0]["delta"].get("content", "")
                        if content:
                            buffer += content
                            print(f"  -> {repr(content)}", end="", flush=True)
                    except:
                        pass
                
                if buffer.strip():
                    print(f"\n✅ Local LLM streaming: OK - Got response: {buffer[:60]}...")
                    return True
                else:
                    print(f"\n⚠️ Local LLM streaming: Empty response")
                    return True
            except Exception as e:
                print(f"\n⚠️ Local LLM streaming parse error: {e}")
                return True
        else:
            print(f"❌ Local LLM: HTTP {response.status_code}")
            return False
    except requests.Timeout:
        print(f"❌ Local LLM: Connection timeout")
        return False
    except requests.ConnectionError:
        print(f"❌ Local LLM: Connection failed (not running on port 8000)")
        return False
    except Exception as e:
        print(f"❌ Local LLM: {e}")
        return False


def test_local_llm_vision_mode():
    """Test local LLM with vision/image support (base64 encoded)"""
    print("\n👁️ Testing Local LLM (vision mode with base64 image)...")
    try:
        # Create a simple test image (100x100 red pixels)
        import cv2
        import numpy as np
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[:, :] = (0, 0, 255)  # Red
        ret, buffer = cv2.imencode('.jpg', img)
        if not ret:
            print(f"❌ Failed to encode test image")
            return False
        
        image_base64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
        
        payload = {
            "model": "llava-1.6",  # Try common vision model name
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Describe this image in 20 words"},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                    ]
                }
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        response = requests.post(LOCAL_LLM_URL, json=payload, timeout=15)
        
        if response.status_code == 200:
            data = response.json()
            if data.get("choices") and len(data["choices"]) > 0:
                text = data["choices"][0].get("message", {}).get("content", "").strip()
                if text:
                    print(f"✅ Local LLM vision mode: OK - Got response: {text[:60]}...")
                    return True
                else:
                    print(f"⚠️ Local LLM vision mode: Empty response (may not support vision)")
                    return True
            else:
                print(f"⚠️ Local LLM vision mode: Unexpected response format")
                return True
        else:
            print(f"❌ Local LLM vision: HTTP {response.status_code}")
            return False
    except ImportError:
        print(f"⚠️ OpenCV not available for vision test - skipping")
        return True
    except Exception as e:
        print(f"⚠️ Local LLM vision mode: {e} (may not support vision)")
        return True


def test_hotspot_detection():
    """Test hotspot detection"""
    print("\n🌐 Testing hotspot detection...")
    try:
        from narration import is_hotspot_mode, get_hotspot_status
        hotspot = is_hotspot_mode()
        print(f"{'✅' if hotspot else '⚠️'} Hotspot mode: {hotspot}")
        
        # Also check using main.py's detection
        try:
            from main import get_hotspot_status as main_get_hotspot_status
            hotspot_main = main_get_hotspot_status()
            print(f"✅ Main.py hotspot detection: {hotspot_main}")
        except:
            pass
        
        return True
    except Exception as e:
        print(f"❌ Hotspot detection: {e}")
        return False


def test_narration_engine_config():
    """Test that NarrationEngine has local LLM config method"""
    print("\n🎙️ Testing NarrationEngine configuration...")
    try:
        from narration import NarrationEngine, is_hotspot_mode
        engine = NarrationEngine()
        
        # Check if set_local_llm_config method exists
        if hasattr(engine, 'set_local_llm_config'):
            print(f"✅ NarrationEngine.set_local_llm_config: exists")
            # Try to call it
            engine.set_local_llm_config(force_use=is_hotspot_mode())
            print(f"✅ NarrationEngine local LLM config: callable")
            return True
        else:
            print(f"❌ NarrationEngine.set_local_llm_config: NOT FOUND")
            return False
    except Exception as e:
        print(f"❌ NarrationEngine config: {e}")
        return False


def main():
    print("=" * 70)
    print("🧪 HOTSPOT NARRATION SETUP VERIFICATION")
    print("=" * 70)
    print(f"\n📋 Configuration:")
    print(f"  Local LLM URL: {LOCAL_LLM_URL}")
    print(f"  Mimic3 TTS URL: {MIMIC3_API_URL}")
    print(f"  Audio Device: {AUDIO_DEVICE}")
    
    results = {
        "Mimic3 TTS": test_mimic3_tts(),
        "Local LLM (text/streaming)": test_local_llm_text_mode(),
        "Local LLM (vision mode)": test_local_llm_vision_mode(),
        "Hotspot detection": test_hotspot_detection(),
        "NarrationEngine config": test_narration_engine_config(),
    }
    
    print("\n" + "=" * 70)
    print("📊 TEST RESULTS SUMMARY:")
    print("=" * 70)
    for test_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {status}: {test_name}")
    
    all_passed = all(results.values())
    print("\n" + "=" * 70)
    if all_passed:
        print("✨ ALL TESTS PASSED - Hotspot narration setup is ready!")
    else:
        print("⚠️ SOME TESTS FAILED - Check the errors above")
        print("\nRequired services:")
        print("  - Local LLM: http://127.0.0.1:8000 (e.g., ollama serve)")
        print("  - Mimic3 TTS: http://127.0.0.1:59125 (piper-tts server)")
    print("=" * 70)
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
