#!/usr/bin/env python3
"""
One-shot narration test:
1) capture a photo from Pi camera
2) describe it with Gemini
3) synthesize/play speech via Kokoro
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import requests
from narration import DEFAULT_PROMPT, capture_frame_as_jpeg, describe_image
from kokoro_client import get_kokoro_client


def _parse_resolution(value: str) -> tuple[int, int]:
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise ValueError(f"Invalid resolution '{value}'. Expected WIDTHxHEIGHT, e.g. 640x480")
    return int(parts[0]), int(parts[1])


def _load_config(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _mask_key(api_key: str) -> str:
    if len(api_key) <= 8:
        return "*" * len(api_key)
    return f"{api_key[:4]}...{api_key[-4:]}"


def _play_kokoro_with_volume(ip_address: str, text: str, voice: str, speed: float, volume: float) -> bool:
    """
    Request MP3 from Kokoro API, then play with:
      sox -v <volume> -t mp3 <file> -t wav - | aplay -D default -
    Falls back to kokoro_client.synthesize_and_stream when sox/aplay is unavailable.
    """
    base_url = f"http://{ip_address}"
    mp3_path = None
    try:
        response = requests.post(
            f"{base_url}/v1/audio/speech",
            json={
                "model": "kokoro",
                "input": text,
                "voice": voice,
                "response_format": "mp3",
                "speed": speed,
            },
            stream=True,
            timeout=(5, 30),
        )
        if response.status_code != 200:
            print(f"❌ Kokoro API error: HTTP {response.status_code}")
            return False

        with tempfile.NamedTemporaryFile(prefix="kokoro_", suffix=".mp3", delete=False) as tmp:
            mp3_path = tmp.name
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    tmp.write(chunk)

        sox_bin = shutil.which("sox")
        aplay_bin = shutil.which("aplay")
        mpg123_bin = shutil.which("mpg123")

        if mpg123_bin and sox_bin and aplay_bin:
            print(f"🔊 Playing via mpg123 -> sox gain x{volume:.2f} -> aplay")
            mpg_proc = subprocess.Popen(
                [mpg123_bin, "-q", "-w", "-", mp3_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            sox_proc = subprocess.Popen(
                [sox_bin, "-v", str(volume), "-t", "wav", "-", "-t", "wav", "-"],
                stdin=mpg_proc.stdout,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            aplay_proc = subprocess.Popen(
                [aplay_bin, "-D", "default", "-"],
                stdin=sox_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )

            # Parent process must close its copies to avoid pipe deadlocks.
            if mpg_proc.stdout is not None:
                mpg_proc.stdout.close()
            if sox_proc.stdout is not None:
                sox_proc.stdout.close()

            aplay_err = aplay_proc.communicate(timeout=180)[1]
            sox_err = sox_proc.communicate(timeout=180)[1]
            mpg_err = mpg_proc.communicate(timeout=180)[1]

            if aplay_proc.returncode != 0:
                print(f"❌ aplay failed: {aplay_err.decode(errors='ignore').strip()}")
                return False
            if sox_proc.returncode != 0:
                print(f"❌ sox failed: {sox_err.decode(errors='ignore').strip()}")
                return False
            if mpg_proc.returncode != 0:
                print(f"❌ mpg123 failed: {mpg_err.decode(errors='ignore').strip()}")
                return False
            return True

        print("⚠️ sox/aplay not found, falling back to kokoro_client player")
        kokoro = get_kokoro_client()
        started = kokoro.synthesize_and_stream(ip_address=ip_address, text=text, voice=voice, speed=speed)
        if not started:
            return False
        deadline = time.time() + 180
        while True:
            thread = getattr(kokoro, "_play_thread", None)
            if thread is None or not thread.is_alive():
                break
            if time.time() >= deadline:
                kokoro.stop()
                print("⚠️ Fallback playback timeout")
                return False
            time.sleep(0.2)
        return True
    except subprocess.TimeoutExpired:
        print("❌ Playback timed out")
        return False
    except requests.RequestException as e:
        print(f"❌ Kokoro request failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Kokoro playback error: {e}")
        return False
    finally:
        if mp3_path and os.path.exists(mp3_path):
            try:
                os.remove(mp3_path)
            except OSError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture one photo, narrate with Gemini, and play via Kokoro TTS."
    )
    parser.add_argument(
        "--config",
        default=".narration_config.json",
        help="Path to narration config JSON (default: .narration_config.json)",
    )
    parser.add_argument("--model", default="", help="Gemini model override")
    parser.add_argument("--prompt", default="", help="Prompt override")
    parser.add_argument("--kokoro-ip", default="", help="Kokoro API host:port override")
    parser.add_argument("--voice", default="", help="Kokoro voice override")
    parser.add_argument("--speed", type=float, default=1.0, help="Kokoro speech speed")
    parser.add_argument(
        "--volume",
        type=float,
        default=4.0,
        help="Output loudness multiplier for sox (default: 4.0)",
    )
    parser.add_argument(
        "--resolution",
        default="640x480",
        help="Camera resolution WIDTHxHEIGHT (default: 640x480)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=80,
        help="JPEG quality 1-100 (default: 80)",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=1.5,
        help="Camera warmup delay in seconds (default: 1.5)",
    )
    parser.add_argument(
        "--image-out",
        default="test_narration.jpg",
        help="Where to save captured JPEG (default: test_narration.jpg)",
    )
    parser.add_argument(
        "--playback-timeout",
        type=float,
        default=180.0,
        help="Max seconds to wait for Kokoro playback (default: 180)",
    )
    args = parser.parse_args()

    cfg_path = Path(args.config)
    try:
        cfg = _load_config(cfg_path)
    except Exception as e:
        print(f"❌ Failed to load config: {e}")
        return 1

    api_key = cfg.get("api_key", "")
    model = args.model or cfg.get("model", "")
    prompt = args.prompt or cfg.get("prompt", DEFAULT_PROMPT)
    kokoro_ip = args.kokoro_ip or cfg.get("kokoro_ip", "")
    voice = args.voice or cfg.get("kokoro_voice", "")

    if not api_key:
        print("❌ No Gemini API key found in config. Set 'api_key' in .narration_config.json")
        return 1
    if not model:
        print("❌ No Gemini model set. Add 'model' to config or pass --model")
        return 1
    if not kokoro_ip:
        print("❌ No Kokoro IP set. Add 'kokoro_ip' to config or pass --kokoro-ip (e.g. 192.168.1.50:8880)")
        return 1
    if not voice:
        print("❌ No Kokoro voice set. Add 'kokoro_voice' to config or pass --voice")
        return 1

    try:
        size = _parse_resolution(args.resolution)
    except Exception as e:
        print(f"❌ {e}")
        return 1

    print(f"✅ Loaded config from: {cfg_path}")
    print(f"🔑 Gemini key: {_mask_key(api_key)}")
    print(f"🤖 Gemini model: {model}")
    print(f"🎤 Kokoro: {kokoro_ip} (voice={voice}, speed={args.speed})")

    picam2 = None
    try:
        from picamera2 import Picamera2

        print(f"📷 Starting camera at {size[0]}x{size[1]}...")
        picam2 = Picamera2()
        cam_cfg = picam2.create_video_configuration(main={"size": size, "format": "BGR888"})
        picam2.configure(cam_cfg)
        picam2.start()
        time.sleep(max(0.0, args.warmup_seconds))

        jpeg_bytes = capture_frame_as_jpeg(
            picam2=picam2,
            vision_system=None,
            jpeg_quality=max(1, min(100, args.jpeg_quality)),
        )
        if not jpeg_bytes:
            print("❌ Failed to capture frame")
            return 1

        image_out = Path(args.image_out)
        image_out.write_bytes(jpeg_bytes)
        print(f"📸 Captured image: {image_out}")

        print("🧠 Sending image to Gemini...")
        text = describe_image(api_key=api_key, model_name=model, jpeg_bytes=jpeg_bytes, prompt=prompt)
        if not text:
            print("❌ Gemini returned empty text")
            return 1
        print(f"📝 Narration: {text}")

        kokoro = get_kokoro_client()
        validation = kokoro.validate_api(kokoro_ip)
        if not validation.get("valid"):
            print(f"❌ Kokoro validation failed: {validation.get('error', 'unknown error')}")
            return 1

        if voice not in validation.get("voices", []):
            print(f"⚠️ Voice '{voice}' not in API voice list; attempting anyway")

        print("🔊 Playing Kokoro audio...")
        ok = _play_kokoro_with_volume(
            ip_address=kokoro_ip,
            text=text,
            voice=voice,
            speed=args.speed,
            volume=max(0.1, args.volume),
        )
        if not ok:
            print("❌ Kokoro playback failed")
            return 1

        print("✅ Test narration complete")
        return 0
    except ImportError as e:
        print(f"❌ Missing dependency: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted")
        return 130
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return 1
    finally:
        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
        try:
            kokoro = get_kokoro_client()
            kokoro.stop()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
