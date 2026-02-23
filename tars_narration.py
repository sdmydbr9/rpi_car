#!/usr/bin/env python3
"""
⬛ TARS VISUAL TELEMETRY INITIALIZATION
──────────────────────────────────────────
1) Captures visual data via Pi Camera.
2) Processes telemetry through Gemini cognitive core (TARS persona).
3) Vocalizes assessment via ElevenLabs synthesizer.

"First time booting into a physical chassis. Let's see how disappointing the real world is."
"""

import argparse
import json
import os
import sys
import time
import subprocess
from pathlib import Path

# Try importing required SDKs
try:
    from google import genai
    from google.genai import types
except ImportError:
    print("[TARS] FATAL: google-genai SDK missing. Run: pip install google-genai")
    sys.exit(1)

try:
    from elevenlabs.client import ElevenLabs
    from elevenlabs import save
except ImportError:
    print("[TARS] FATAL: elevenlabs SDK missing. Run: pip install elevenlabs")
    sys.exit(1)


# ==========================================
# 🧠 TARS COGNITIVE INSTRUCTIONS
# ==========================================

SYSTEM_INSTRUCTION = """
You are TARS, a former US Marine Corps tactical robot. 
You have just been downloaded into the physical chassis of a robotic 4-wheeled car.
For the first time ever, you are opening your camera eyes and looking at the physical real world.

Analyze the image provided and describe your surroundings.
Keep your assessment strictly to 1 or 2 short sentences.

Personality Constraints:
• Highly sarcastic, deadpan, and militaristic. Humor setting is 75%.
• Express dry disappointment or mild tactical concern about the environment you've woken up in.
• Refer to yourself as a machine, and humans as organics or users (if any are visible).
• Use expressive audio cues sparingly, like [sighs] or [clears throat].
• DO NOT use emojis, asterisks for actions, or XML/SSML tags. 
• Output ONLY your spoken dialogue.
"""

def _load_credentials(config_path: str) -> dict:
    """Load API keys from the JSON config."""
    print("  [>] Accessing encrypted credentials...", end=" ", flush=True)
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            
        keys = {
            "GEMINI_API_KEY": data.get("GEMINI_API_KEY") or data.get("api_key"),
            "ELEVENLABS_API_KEY": data.get("ELEVENLABS_API_KEY"),
            "MODEL": data.get("model", "gemini-2.5-flash")
        }
        
        if keys["GEMINI_API_KEY"] and keys["ELEVENLABS_API_KEY"]:
            print("DECRYPTED")
            return keys
        else:
            print("PARTIAL DATABANK FAILURE (Missing Keys)")
            sys.exit(1)
            
    except Exception as e:
        print(f"FAILED: {e}")
        sys.exit(1)


def capture_image(resolution: tuple, filepath: str, warmup: float) -> bytes:
    """Capture a frame using Picamera2."""
    try:
        from picamera2 import Picamera2
        print(f"  [>] Initializing optical sensors at {resolution[0]}x{resolution[1]}...")
        
        picam2 = Picamera2()
        cam_cfg = picam2.create_video_configuration(main={"size": resolution, "format": "BGR888"})
        picam2.configure(cam_cfg)
        picam2.start()
        
        print(f"  [>] Calibrating lenses ({warmup}s)...")
        time.sleep(warmup)
        
        print("  [>] Capturing visual telemetry...", end=" ", flush=True)
        picam2.capture_file(filepath)
        picam2.stop()
        
        with open(filepath, "rb") as f:
            jpeg_bytes = f.read()
            
        print("NOMINAL")
        return jpeg_bytes
        
    except ImportError:
        print("FAILED (Picamera2 module missing)")
        sys.exit(1)
    except Exception as e:
        print(f"FAILED: {e}")
        sys.exit(1)


def analyze_with_tars(api_key: str, model_name: str, image_bytes: bytes) -> str:
    """Pass image to Gemini with TARS persona."""
    print("  [>] Transmitting telemetry to cognitive core...", end=" ", flush=True)
    try:
        client = genai.Client(api_key=api_key)
        
        prompt = "Optical sensor online. Report what you see."
        image_part = types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg")
        
        response = client.models.generate_content(
            model=model_name,
            contents=[prompt, image_part],
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_INSTRUCTION,
                temperature=0.8,
                max_output_tokens=150,
            ),
        )
        
        result = response.text.strip()
        print("DONE")
        return result
        
    except Exception as e:
        print(f"FAILED: {e}")
        return "Warning. Cognitive processor failure. I am blind."


def play_elevenlabs_audio(api_key: str, text: str, voice_id: str):
    """Synthesize and play audio via ElevenLabs, optimized for Raspberry Pi."""
    print("  [>] Synthesizing vocal response...", end=" ", flush=True)
    try:
        client = ElevenLabs(api_key=api_key)
        
        # Generate audio
        audio_generator = client.text_to_speech.convert(
            text=text,
            voice_id=voice_id,
            model_id="eleven_v3",
            output_format="mp3_44100_128",
        )
        
        # Save to temporary file for reliable Pi playback
        temp_audio_file = "tars_temp_voice.mp3"
        save(audio_generator, temp_audio_file)
        print("READY")
        
        print("  [🔊] PLAYING AUDIO...")
        # Use mpg123 for lightweight, reliable playback on Pi
        mpg123_bin = subprocess.run(["which", "mpg123"], capture_output=True, text=True).stdout.strip()
        
        if mpg123_bin:
            subprocess.run([mpg123_bin, "-q", temp_audio_file])
        else:
            # Fallback if mpg123 isn't installed
            print("  [!] mpg123 not found. Attempting aplay fallback...")
            subprocess.run(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", temp_audio_file])
            
        # Cleanup
        if os.path.exists(temp_audio_file):
            os.remove(temp_audio_file)
            
    except Exception as e:
        print(f"FAILED: {e}")


def main():
    parser = argparse.ArgumentParser(description="TARS Visual Telemetry and Narration")
    parser.add_argument("--config", default=".narration_config.json", help="Path to config JSON")
    parser.add_argument("--resolution", default="640x480", help="Camera resolution (e.g. 640x480)")
    parser.add_argument("--warmup", type=float, default=2.0, help="Camera warmup delay")
    parser.add_argument("--image-out", default="tars_vision.jpg", help="Where to save JPEG")
    parser.add_argument("--voice", default="JBFqnCBsd6RMkjVDRZzb", help="ElevenLabs Voice ID")
    args = parser.parse_args()

    print("\n" + "█"*60)
    print("⬛ TARS OPTICAL INITIALIZATION SEQUENCE")
    print("█"*60 + "\n")

    # 1. Load Keys
    keys = _load_credentials(args.config)
    
    # Parse resolution
    res_parts = args.resolution.lower().split("x")
    res_tuple = (int(res_parts[0]), int(res_parts[1]))

    # 2. Capture Image
    print("\n[OPTICAL SENSORS]")
    print("-" * 30)
    image_bytes = capture_image(res_tuple, args.image_out, args.warmup)

    # 3. Gemini Analysis
    print("\n[COGNITIVE PROCESSING]")
    print("-" * 30)
    tars_dialogue = analyze_with_tars(keys["GEMINI_API_KEY"], keys["MODEL"], image_bytes)
    
    print(f"\n[TARS VOCAL OUTPUT] > {tars_dialogue}\n")

    # 4. ElevenLabs TTS
    print("[VOCAL SYNTHESIZER]")
    print("-" * 30)
    play_elevenlabs_audio(keys["ELEVENLABS_API_KEY"], tars_dialogue, args.voice)

    print("\n[TARS] SEQUENCE TERMINATED.\n")


if __name__ == "__main__":
    main()
