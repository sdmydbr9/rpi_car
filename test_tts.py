#!/usr/bin/env python3
"""
🔊 Test script for local TTS on Raspberry Pi
Tests espeak-ng and pyttsx3 TTS engines on the headphone jack.
"""

import sys
from tts_local import get_tts_synthesizer

def main():
    print("🔊 [TTS Test] Initializing TTS synthesizer...")
    tts = get_tts_synthesizer()
    
    if not tts.is_available:
        print("❌ [TTS Test] No TTS engine available!")
        print("   Install espeak-ng: sudo apt install espeak-ng")
        print("   Or pyttsx3: pip install pyttsx3")
        sys.exit(1)
    
    test_phrases = [
        "Hello, I am the Raspberry Pi robot car.",
        "This is a test of the local text to speech system.",
        "Narration will now play through the headphone jack speaker.",
    ]
    
    for i, phrase in enumerate(test_phrases, 1):
        print(f"\n🔊 [TTS Test] Playing phrase {i}/{len(test_phrases)}:")
        print(f"   '{phrase}'")
        tts.speak(phrase)
        
        # Wait for playback to finish
        import time
        while tts.is_playing:
            time.sleep(0.1)
    
    print("\n✅ [TTS Test] All tests completed!")
    print("   If you heard the phrases through the headphone jack, TTS is working correctly.")

if __name__ == '__main__':
    main()

