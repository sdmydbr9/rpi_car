"""
🎙️ AI Narration Module
Provides Gemini-powered image description for the robot's camera feed.
Supports key validation, multimodal model discovery, and image analysis.
Falls back to local Llama LLM when in hotspot mode or when Gemini fails.
"""

import google.generativeai as genai
import threading
import time
import cv2
import numpy as np
import os
import shutil
import subprocess
import tempfile
import requests
import json
import base64
from tts_local import get_tts_synthesizer
from kokoro_client import get_kokoro_client

# ==========================================
# 🌐 HOTSPOT DETECTION & LOCAL LLM
# ==========================================

# Default narration prompt
DEFAULT_PROMPT = (
    "you are a robot who just got vision for the first time, "
    "describe what do you see in less than 20 words, "
    "be narrative and interesting and use human like expressions"
)

# Configuration for hotspot mode (matches auto_voice.py exactly)
LOCAL_LLM_URL = os.getenv("RC_LOCAL_LLM_URL", "http://127.0.0.1:8000/v1/chat/completions")
MIMIC3_TTS_URL = os.getenv("RC_MIMIC3_TTS_URL", "http://127.0.0.1:59125/api/tts")
HOTSPOT_AUDIO_DEVICE = os.getenv("RC_HOTSPOT_AUDIO_DEVICE", "plughw:0,0")
ELEVENLABS_TTS_URL = os.getenv("RC_ELEVENLABS_TTS_URL", "https://api.elevenlabs.io/v1/text-to-speech")
ELEVENLABS_DEFAULT_VOICE_ID = os.getenv("RC_ELEVENLABS_VOICE_ID", "JBFqnCBsd6RMkjVDRZzb")
ELEVENLABS_DEFAULT_MODEL_ID = os.getenv("RC_ELEVENLABS_MODEL_ID", "eleven_v3")

def is_hotspot_mode() -> bool:
    """Check if device is in hotspot mode (no internet connectivity).
    Returns True if wlan0 IP is in hotspot range (192.168.4.x or 10.42.0.x)"""
    try:
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'], 
                              capture_output=True, text=True, timeout=2)
        for line in result.stdout.split('\n'):
            if 'inet ' in line and 'scope' in line:
                parts = line.strip().split()
                if len(parts) >= 2:
                    ip_with_mask = parts[1]
                    ip = ip_with_mask.split('/')[0]
                    # Hotspot range check
                    if ip.startswith('192.168.4.') or ip.startswith('10.42.0.'):
                        return True
                    # Has regular WiFi IP, not hotspot
                    return False
        return False
    except Exception as e:
        print(f"⚠️ [Narration] Failed to check hotspot status: {e}")
        return False


def has_internet_connectivity(timeout: float = 5.0) -> bool:
    """Check if device has internet connectivity by attempting DNS resolution."""
    try:
        requests.head("http://8.8.8.8", timeout=timeout)
        return True
    except (requests.RequestException, Exception):
        return False


def describe_image_local_llm(jpeg_bytes: bytes, prompt: str = DEFAULT_PROMPT,
                             local_llm_url: str = None) -> str:
    """Send a JPEG image to a local Llama LLM for description via base64 encoding.
    
    Exactly like auto_voice.py but with vision support.
    
    Args:
        jpeg_bytes: Raw JPEG image data
        prompt: Text prompt for the AI
        local_llm_url: URL of the local LLM API endpoint
    
    Returns:
        The AI's text response, or an empty string on failure.
    """
    if local_llm_url is None:
        local_llm_url = LOCAL_LLM_URL
    
    try:
        # Encode image as base64
        image_base64 = base64.b64encode(jpeg_bytes).decode('utf-8')
        
        # Prepare payload for local LLM with vision capability
        payload = {
            "model": "llama-2-vision",  # or whatever vision model is available
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                    ]
                }
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        response = requests.post(
            local_llm_url,
            json=payload,
            timeout=20
        )
        
        if response.status_code == 200:
            data = response.json()
            if data.get("choices") and len(data["choices"]) > 0:
                text = data["choices"][0].get("message", {}).get("content", "").strip()
                if text:
                    print(f"👁️ [Narration] Local LLM says: {text}")
                    return text
        
        print(f"❌ [Narration] Local LLM returned status {response.status_code}")
        return ""
        
    except requests.Timeout:
        print(f"❌ [Narration] Local LLM request timed out")
        return ""
    except Exception as e:
        print(f"❌ [Narration] Local LLM error: {e}")
        return ""


def _play_mimic3_tts(text: str, mimic3_url: str = None, audio_device: str = None,
                     on_playback_start=None, on_playback_end=None) -> bool:
    """Play audio via Mimic3 TTS exactly like auto_voice.py.
    
    Fetches WAV audio from Mimic3, then plays via aplay.
    
    Args:
        text: Text to synthesize
        mimic3_url: Mimic3 API endpoint
        audio_device: ALSA device for playback (e.g., 'plughw:0,0')
        on_playback_start: Optional callback fired immediately before audio playback.
        on_playback_end: Optional callback fired immediately after audio playback.
    
    Returns:
        True if playback succeeded, False otherwise
    """
    if mimic3_url is None:
        mimic3_url = MIMIC3_TTS_URL
    if audio_device is None:
        audio_device = HOTSPOT_AUDIO_DEVICE
    
    if not text.strip():
        return False
    
    try:
        print(f"🎙️ [Narration] Mimic3: synthesizing...")
        # GET request to Mimic3 (no streaming needed — we consume r.content)
        r = requests.get(mimic3_url, params={"text": text}, timeout=(5, 90))
        
        if r.status_code != 200:
            print(f"❌ [Narration] Mimic3 HTTP {r.status_code}")
            return False
        
        audio_bytes = r.content
        if not audio_bytes:
            print("❌ [Narration] Mimic3 returned empty audio")
            return False
        
        # Play content via aplay (exactly like startup_systems_check.py)
        print(f"🎙️ [Narration] Mimic3: playing to {audio_device}...")
        playback_started = False
        try:
            if callable(on_playback_start):
                try:
                    on_playback_start()
                    playback_started = True
                except Exception as cb_err:
                    print(f"⚠️ [Narration] Playback start callback failed: {cb_err}")
            
            primary = subprocess.run(
                ['aplay', '-D', audio_device, '-q'],
                input=audio_bytes,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=180,
            )
            if primary.returncode == 0:
                print(f"✅ [Narration] Mimic3 playback: OK")
                return True
            
            primary_err = primary.stderr.decode(errors='ignore').strip()
            print(f"❌ [Narration] Mimic3 playback failed on {audio_device}: {primary_err}")
            
            # Fallback to default audio device (like startup_systems_check.py)
            if audio_device != "default":
                fallback = subprocess.run(
                    ['aplay', '-D', 'default', '-q'],
                    input=audio_bytes,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=180,
                )
                if fallback.returncode == 0:
                    print("✅ [Narration] Mimic3 fallback played on system default audio device")
                    return True
                fallback_err = fallback.stderr.decode(errors='ignore').strip()
                print(f"❌ [Narration] Mimic3 fallback default playback failed: {fallback_err}")
            
            return False
        finally:
            if playback_started and callable(on_playback_end):
                try:
                    on_playback_end()
                except Exception as cb_err:
                    print(f"⚠️ [Narration] Playback end callback failed: {cb_err}")
        
    except requests.Timeout:
        print(f"❌ [Narration] Mimic3 request timeout")
        return False
    except requests.ConnectionError:
        print(f"❌ [Narration] Mimic3 connection failed (not running at {mimic3_url})")
        return False
    except Exception as e:
        print(f"❌ [Narration] Mimic3 error: {e}")
        return False


# ==========================================
# 🔊 SHARED MP3 PLAYBACK HELPER
# ==========================================

def _play_mp3_with_loudness(mp3_path: str, volume: float = 8.0, audio_device: str = "default",
                           on_playback_start=None, on_playback_end=None) -> bool:
    """Decode MP3 and play with compression/normalization via sox -> aplay.
    
    Args:
        on_playback_start: Optional callback fired immediately before audio playback.
        on_playback_end: Optional callback fired immediately after audio playback.
    """
    sox_bin = shutil.which("sox")
    aplay_bin = shutil.which("aplay")
    mpg123_bin = shutil.which("mpg123")
    if not (mpg123_bin and sox_bin and aplay_bin):
        print("⚠️ [Narration] Missing mpg123/sox/aplay for MP3 playback")
        return False

    wav_path = f"{mp3_path}.wav"
    sox_proc = None
    aplay_proc = None
    playback_started = False
    try:
        decode = subprocess.run(
            [mpg123_bin, "-q", "-w", wav_path, mp3_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=60,
        )
        if decode.returncode != 0 or not os.path.exists(wav_path):
            err = decode.stderr.decode(errors="ignore").strip()
            print(f"❌ [Narration] mpg123 decode failed: {err}")
            return False

        # Fire playback start callback AFTER decode but BEFORE audio pipeline
        if callable(on_playback_start):
            try:
                on_playback_start()
                playback_started = True
            except Exception as cb_err:
                print(f"⚠️ [Narration] Playback start callback failed: {cb_err}")

        sox_cmd = [
            sox_bin,
            "-v", str(max(0.1, volume)),
            wav_path,
            "-t", "wav", "-",
            "highpass", "100",
            "compand", "0.01,0.20",
            "6:-90,-70,-40,-18,-20,-8,-5,-2",
            "-2", "-90", "0.05",
            "gain", "-n", "-0.1",
        ]
        sox_proc = subprocess.Popen(
            sox_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        aplay_proc = subprocess.Popen(
            [aplay_bin, "-D", audio_device, "-"],
            stdin=sox_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        if sox_proc.stdout is not None:
            sox_proc.stdout.close()

        aplay_err = aplay_proc.communicate(timeout=180)[1]
        sox_err = sox_proc.communicate(timeout=180)[1]

        if aplay_proc.returncode != 0:
            print(f"❌ [Narration] aplay failed: {aplay_err.decode(errors='ignore').strip()}")
            return False
        if sox_proc.returncode != 0:
            print(f"❌ [Narration] sox failed: {sox_err.decode(errors='ignore').strip()}")
            return False
        return True
    except subprocess.TimeoutExpired:
        print("❌ [Narration] Playback timed out")
        if aplay_proc and aplay_proc.poll() is None:
            aplay_proc.kill()
        if sox_proc and sox_proc.poll() is None:
            sox_proc.kill()
        return False
    except Exception as e:
        print(f"❌ [Narration] MP3 playback error: {e}")
        if aplay_proc and aplay_proc.poll() is None:
            aplay_proc.kill()
        if sox_proc and sox_proc.poll() is None:
            sox_proc.kill()
        return False
    finally:
        if playback_started and callable(on_playback_end):
            try:
                on_playback_end()
            except Exception as cb_err:
                print(f"⚠️ [Narration] Playback end callback failed: {cb_err}")
        if os.path.exists(wav_path):
            try:
                os.remove(wav_path)
            except OSError:
                pass


# ==========================================
# 🎤 ELEVENLABS PLAYBACK HELPER
# ==========================================

def _play_elevenlabs_tts(
    text: str,
    api_key: str,
    voice_id: str = ELEVENLABS_DEFAULT_VOICE_ID,
    model_id: str = ELEVENLABS_DEFAULT_MODEL_ID,
    audio_device: str = "default",
    volume: float = 8.0,
    on_playback_start=None,
    on_playback_end=None,
) -> bool:
    """Synthesize speech with ElevenLabs and play it locally."""
    if not api_key or not text or not text.strip():
        return False

    mp3_path = None
    try:
        headers = {
            "xi-api-key": api_key,
            "Content-Type": "application/json",
            "Accept": "audio/mpeg",
        }
        payload = {
            "text": text,
            "model_id": model_id or ELEVENLABS_DEFAULT_MODEL_ID,
        }
        response = requests.post(
            f"{ELEVENLABS_TTS_URL}/{voice_id}",
            headers=headers,
            json=payload,
            timeout=(5, 30),
        )
        if response.status_code != 200:
            body = response.text[:160].strip().replace("\n", " ")
            print(f"❌ [Narration] ElevenLabs API error: HTTP {response.status_code} - {body}")
            return False
        if not response.content:
            print("❌ [Narration] ElevenLabs returned empty audio")
            return False

        with tempfile.NamedTemporaryFile(prefix="eleven_narr_", suffix=".mp3", delete=False) as tmp:
            mp3_path = tmp.name
            tmp.write(response.content)

        return _play_mp3_with_loudness(
            mp3_path=mp3_path,
            volume=volume,
            audio_device=audio_device,
            on_playback_start=on_playback_start,
            on_playback_end=on_playback_end,
        )
    except requests.RequestException as e:
        print(f"❌ [Narration] ElevenLabs request failed: {e}")
        return False
    except Exception as e:
        print(f"❌ [Narration] ElevenLabs playback error: {e}")
        return False
    finally:
        if mp3_path and os.path.exists(mp3_path):
            try:
                os.remove(mp3_path)
            except OSError:
                pass


# ==========================================# � KOKORO LOUDNESS PLAYBACK HELPER
# ==========================================

def _play_kokoro_with_volume(
    ip_address: str,
    text: str,
    voice: str,
    speed: float = 1.0,
    volume: float = 8.0,
    audio_device: str = 'default',
    on_playback_start=None,
    on_playback_end=None,
) -> bool:
    """
    Request MP3 from Kokoro API, then decode and play with an aggressive
    loudness chain (compression + normalisation) via sox -> aplay.
    Falls back to kokoro_client.synthesize_and_stream when sox/aplay is unavailable.
    
    Args:
        audio_device: ALSA device name (e.g. 'default', 'hw:1,0') for output
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
            print(f"❌ [Narration] Kokoro API error: HTTP {response.status_code}")
            return False

        with tempfile.NamedTemporaryFile(prefix="kokoro_narr_", suffix=".mp3", delete=False) as tmp:
            mp3_path = tmp.name
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    tmp.write(chunk)

        if _play_mp3_with_loudness(
            mp3_path=mp3_path,
            volume=volume,
            audio_device=audio_device,
            on_playback_start=on_playback_start,
            on_playback_end=on_playback_end,
        ):
            return True

        # Fallback: kokoro_client streaming when local MP3 playback pipeline fails
        print("⚠️ [Narration] Kokoro loudness pipeline failed — falling back to kokoro_client streaming")
        kokoro = get_kokoro_client()
        return kokoro.synthesize_and_stream(ip_address=ip_address, text=text, voice=voice, speed=speed)

    except subprocess.TimeoutExpired:
        print("❌ [Narration] Playback timed out")
        return False
    except requests.RequestException as e:
        print(f"❌ [Narration] Kokoro request failed: {e}")
        return False
    except Exception as e:
        print(f"❌ [Narration] Kokoro playback error: {e}")
        return False
    finally:
        if mp3_path and os.path.exists(mp3_path):
            try:
                os.remove(mp3_path)
            except OSError:
                pass


# ==========================================
# �🔑 API KEY VALIDATION
# ==========================================

def validate_key(api_key: str) -> bool:
    """Validate a Gemini API key by attempting to list models.
    Returns True if the key is valid and can access the API."""
    try:
        genai.configure(api_key=api_key)
        # Try to list models — this will fail with an invalid key
        models = list(genai.list_models())
        return len(models) > 0
    except Exception as e:
        print(f"⚠️  [Narration] API key validation failed: {e}")
        return False


# ==========================================
# 📋 MODEL DISCOVERY
# ==========================================

# Models known to support multimodal (vision) input
# We filter by checking if the model supports 'generateContent'
# and has input token limits suggesting multimodal capability
_KNOWN_VISION_MODEL_PREFIXES = [
    "gemini-2",
    "gemini-1.5",
    "gemini-1.0-pro-vision",
    "gemini-pro-vision",
    "gemini-3",
]

def list_multimodal_models(api_key: str) -> list[dict]:
    """List available Gemini models that support multimodal (vision) input.
    Returns a list of dicts with 'name' and 'display_name' keys.
    Only includes models that support generateContent with image inputs."""
    try:
        genai.configure(api_key=api_key)
        models = []
        for m in genai.list_models():
            # Must support generateContent
            if 'generateContent' not in m.supported_generation_methods:
                continue
            
            # Check if model name suggests vision/multimodal capability
            model_name = m.name.replace("models/", "")
            is_multimodal = any(
                model_name.startswith(prefix)
                for prefix in _KNOWN_VISION_MODEL_PREFIXES
            )
            
            if is_multimodal:
                models.append({
                    "name": m.name,
                    "display_name": m.display_name or model_name,
                })
        
        print(f"✅ [Narration] Found {len(models)} multimodal models")
        return models
    except Exception as e:
        print(f"❌ [Narration] Failed to list models: {e}")
        return []


# ==========================================
# 🖼️ IMAGE ANALYSIS
# ==========================================

def describe_image(api_key: str, model_name: str, jpeg_bytes: bytes,
                   prompt: str = DEFAULT_PROMPT) -> str:
    """Send a JPEG image to a Gemini model for description.
    
    Args:
        api_key: Valid Gemini API key
        model_name: Full model name (e.g. 'models/gemini-2.0-flash')
        jpeg_bytes: Raw JPEG image data
        prompt: Text prompt for the AI
    
    Returns:
        The AI's text response, or an error message string.
    """
    try:
        print(f"🔬 [Gemini] Configuring API key and model '{model_name}' ({len(jpeg_bytes)} bytes image)...")
        genai.configure(api_key=api_key)
        # Normalize model name — strip 'models/' prefix if present
        clean_name = model_name
        if clean_name.startswith('models/'):
            clean_name = clean_name[7:]
        model = genai.GenerativeModel(clean_name)
        
        # Use relaxed safety settings to avoid blocked responses
        safety_settings = [
            {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_NONE"},
            {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_NONE"},
            {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_NONE"},
            {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_NONE"},
        ]
        
        print(f"🔬 [Gemini] Sending generate_content() request...")
        t0 = time.time()
        response = model.generate_content(
            [prompt, {'mime_type': 'image/jpeg', 'data': jpeg_bytes}],
            safety_settings=safety_settings,
        )
        t1 = time.time()
        print(f"🔬 [Gemini] generate_content() returned in {t1-t0:.1f}s")
        
        # Handle blocked / empty responses
        if not response.candidates:
            print(f"⚠️ [Narration] Response blocked (no candidates)")
            return ""
        
        try:
            text = response.text.strip()
        except ValueError as e:
            print(f"⚠️ [Narration] Response blocked by safety filter: {e}")
            return ""
        
        print(f"👁️ [Narration] AI says ({len(text)} chars): {text[:500]}")
        return text
    except Exception as e:
        error_msg = f"Narration error: {e}"
        print(f"❌ [Narration] {error_msg}")
        import traceback; traceback.print_exc()
        return ""


# ==========================================
# 📸 FRAME CAPTURE HELPER
# ==========================================

def _capture_frame_from_rtsp(rtsp_url: str, jpeg_quality: int = 70,
                             timeout_s: float = 8.0) -> bytes | None:
    """Grab a single JPEG frame from an RTSP stream (e.g. MediaMTX).

    Opens the stream, reads one frame, then releases.  Uses a worker
    thread with a timeout so a stuck RTSP connection cannot block forever.
    """
    result: list[bytes | None] = [None]

    def _grab():
        cap = None
        try:
            # Set FFMPEG timeouts (in microseconds) to avoid indefinite hangs
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                "timeout;5000000|stimeout;5000000|rtsp_transport;tcp"
            )
            print(f"📸 [RTSP] Opening stream: {rtsp_url}")
            t0 = time.time()
            cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
            t1 = time.time()
            print(f"📸 [RTSP] VideoCapture created in {t1-t0:.1f}s, isOpened={cap.isOpened()}")
            if not cap.isOpened():
                print(f"⚠️  [Narration] Could not open RTSP stream: {rtsp_url}")
                return
            print(f"📸 [RTSP] Reading frame...")
            ret, frame = cap.read()
            t2 = time.time()
            print(f"📸 [RTSP] cap.read() returned in {t2-t1:.1f}s, ret={ret}, frame={'None' if frame is None else frame.shape}")
            if not ret or frame is None:
                print("⚠️  [Narration] Failed to read frame from RTSP stream")
                return
            ok, buffer = cv2.imencode('.jpg', frame,
                                      [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if ok:
                result[0] = buffer.tobytes()
                print(f"📸 [Narration] Captured frame from RTSP ({frame.shape[1]}x{frame.shape[0]}, {len(result[0])} bytes)")
        except Exception as e:
            print(f"❌ [Narration] RTSP frame capture failed: {e}")
            import traceback; traceback.print_exc()
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    t = threading.Thread(target=_grab, daemon=True)
    t.start()
    t.join(timeout=timeout_s)
    if t.is_alive():
        print(f"⚠️  [Narration] RTSP capture timed out after {timeout_s}s")
        # Thread is daemon — it will die when the main process exits
    return result[0]


def capture_frame_as_jpeg(picam2=None, vision_system=None,
                          jpeg_quality: int = 70,
                          rtsp_url: str | None = None) -> bytes | None:
    """Capture a single frame from the camera pipeline and encode as JPEG.
    
    Tries vision system first (to get annotated frame), falls back to
    raw picam2 capture, then falls back to grabbing a frame from an
    RTSP stream (MediaMTX) if provided. Returns JPEG bytes or None.
    """
    print(f"📸 [Frame Capture] Starting: picam2={'SET' if picam2 else 'None'}, "
          f"vision={'SET' if vision_system else 'None'}, "
          f"rtsp_url={rtsp_url or 'None'}")
    try:
        frame = None
        
        # Try vision system annotated frame first
        if vision_system is not None:
            print("📸 [Frame Capture] Trying vision_system.get_frame()...")
            frame = vision_system.get_frame()
            if frame is not None:
                print(f"📸 [Frame Capture] ✅ Got frame from vision_system: {frame.shape}")
            else:
                print("📸 [Frame Capture] ⚠️ vision_system.get_frame() returned None")
        
        # Fall back to raw picam2 — BUT only when there is no RTSP URL.
        # When an RTSP URL is set we are in MediaMTX-only mode and picam2
        # has been closed/released; calling capture_array() would block
        # forever waiting on a dead device.
        if frame is None and picam2 is not None and not rtsp_url:
            print("📸 [Frame Capture] Trying picam2.capture_array()...")
            frame = picam2.capture_array()
            if frame is not None:
                print(f"📸 [Frame Capture] ✅ Got frame from picam2: {frame.shape}")
            else:
                print("📸 [Frame Capture] ⚠️ picam2.capture_array() returned None")
        elif frame is None and picam2 is not None and rtsp_url:
            print("📸 [Frame Capture] Skipping picam2 (MediaMTX mode — device released)")
        
        if frame is not None:
            # Convert BGR to RGB (picam2 captures BGR888)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame_rgb,
                                       [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if not ret:
                print("📸 [Frame Capture] ❌ cv2.imencode failed")
                return None
            jpeg_bytes = buffer.tobytes()
            print(f"📸 [Frame Capture] ✅ JPEG encoded: {len(jpeg_bytes)} bytes")
            return jpeg_bytes

        # Fall back to RTSP snapshot (MediaMTX stream)
        if rtsp_url:
            print(f"📸 [Frame Capture] Trying RTSP fallback: {rtsp_url}")
            result = _capture_frame_from_rtsp(rtsp_url, jpeg_quality)
            if result:
                print(f"📸 [Frame Capture] ✅ RTSP captured: {len(result)} bytes")
            else:
                print("📸 [Frame Capture] ❌ RTSP capture returned None")
            return result

        print("📸 [Frame Capture] ❌ No frame source available (all None)")
        return None
    except Exception as e:
        print(f"❌ [Narration] Frame capture failed: {e}")
        import traceback; traceback.print_exc()
        return None


# ==========================================
# 🔄 NARRATION LOOP (Background Thread)
# ==========================================

class NarrationEngine:
    """Manages the periodic narration loop.
    
    When enabled, periodically captures a frame, sends it to Gemini,
    and calls the callback with the response text.
    """
    
    def __init__(self):
        self._thread: threading.Thread | None = None
        self._running = False
        self._api_key: str = ""
        self._model_name: str = ""
        self._interval: float = 30.0  # seconds between narrations
        self._prompt: str = DEFAULT_PROMPT
        self._picam2 = None
        self._vision_system = None
        self._rtsp_url: str = ""  # RTSP fallback for MediaMTX-only mode
        self._on_narration_text = None  # callback(text: str)
        self._on_narration_error = None  # callback(error: str)
        self._on_narration_done = None   # callback() — fired after audio playback finishes
        self._lock = threading.Lock()
        self._analysis_lock = threading.Lock()  # Prevent overlap between timer/manual analysis
        self._play_local_tts = True  # Fallback to espeak-ng when Kokoro is unavailable
        self._audio_device = 'default'  # ALSA device for both Kokoro and TTS playback
        self._audio_manager = None  # Reference to CarAudioManager for volume ducking
        # ElevenLabs TTS configuration (primary narration voice)
        self._elevenlabs_api_key: str = ""
        self._elevenlabs_voice_id: str = ELEVENLABS_DEFAULT_VOICE_ID
        self._elevenlabs_model_id: str = ELEVENLABS_DEFAULT_MODEL_ID
        self._elevenlabs_volume: float = 8.0
        # Kokoro TTS configuration
        self._kokoro_enabled = False
        self._kokoro_ip: str = ""
        self._kokoro_voice: str = ""
        self._kokoro_speed: float = 1.0
        self._kokoro_volume: float = 8.0  # Aggressive loudness multiplier
        # Local LLM configuration
        self._local_llm_url: str = LOCAL_LLM_URL
        self._use_local_llm: bool = False  # Set to True to force local LLM in hotspot mode
        # Mimic3 TTS configuration (tertiary fallback voice path)
        self._mimic3_enabled: bool = True
        self._mimic3_url: str = MIMIC3_TTS_URL
        self._mimic3_audio_device: str = HOTSPOT_AUDIO_DEVICE
    
    def configure(self, api_key: str, model_name: str,
                  interval: float = 90.0, prompt: str = DEFAULT_PROMPT):
        """Configure narration parameters."""
        with self._lock:
            self._api_key = api_key
            self._model_name = model_name
            self._interval = max(10.0, min(300.0, interval))
            self._prompt = prompt
    
    def set_local_tts_enabled(self, enabled: bool):
        """Enable/disable local TTS (espeak-ng) fallback when Kokoro is unavailable."""
        with self._lock:
            self._play_local_tts = enabled
        status = "enabled" if enabled else "disabled"
        print(f"🔊 [Narration] Local TTS fallback {status}")
    
    def set_tts_audio_device(self, device: str):
        """Configure the audio device for all playback (Kokoro and espeak-ng).
        
        Args:
            device: ALSA device name for GPIO or other audio output (e.g., 'default', 'hw:1,0')
        """
        with self._lock:
            self._audio_device = device
        tts = get_tts_synthesizer()
        tts.set_audio_device(device)
        print(f"🔊 [Narration] Audio device configured for all playback: {device}")

    def set_elevenlabs_config(
        self,
        api_key: str | None,
        voice_id: str | None = None,
        model_id: str | None = None,
        volume: float | None = None,
    ):
        """Configure ElevenLabs credentials used as the first playback option."""
        with self._lock:
            self._elevenlabs_api_key = (api_key or "").strip()
            if voice_id:
                self._elevenlabs_voice_id = voice_id
            if model_id:
                self._elevenlabs_model_id = model_id
            if volume is not None:
                self._elevenlabs_volume = max(0.1, float(volume))

            key_set = bool(self._elevenlabs_api_key)
            voice = self._elevenlabs_voice_id
            model = self._elevenlabs_model_id
            masked = ('*' * 8 + self._elevenlabs_api_key[-4:]) if len(self._elevenlabs_api_key) > 4 else ''

        if key_set:
            print(f"🎤 [Narration] ElevenLabs enabled - key={masked}, voice={voice}, model={model}")
        else:
            print("🎤 [Narration] ElevenLabs disabled (no API key)")
    
    
    def set_kokoro_config(
        self,
        ip_address: str | None,
        voice: str | None,
        speed: float = 1.0,
        volume: float = 8.0,
    ):
        """Configure Kokoro TTS (remote FastAPI server).

        Pass None for either parameter to disable Kokoro.
        speed:  speech rate multiplier (default 1.0)
        volume: sox loudness multiplier — higher = louder (default 8.0)
        """
        with self._lock:
            if ip_address and voice:
                self._kokoro_enabled = True
                self._kokoro_ip = ip_address
                self._kokoro_voice = voice
                self._kokoro_speed = max(0.1, speed)
                self._kokoro_volume = max(0.1, volume)
                print(f"🎤 [Narration] Kokoro enabled - IP: {ip_address}, Voice: {voice}, speed={speed}, volume={volume}")
            else:
                self._kokoro_enabled = False
                self._kokoro_ip = ""
                self._kokoro_voice = ""
                print(f"🎤 [Narration] Kokoro disabled")
    
    def set_local_llm_config(self, url: str | None = None, force_use: bool = False):
        """Configure local LLM for hotspot mode fallback.
        
        Mimic3 remains enabled as a tertiary voice fallback regardless of network mode.
        
        Args:
            url: URL of the local LLM API endpoint (e.g., http://127.0.0.1:8000/v1/chat/completions)
            force_use: If True, always use local LLM instead of Gemini
        """
        with self._lock:
            if url:
                self._local_llm_url = url
            self._use_local_llm = force_use or is_hotspot_mode()
            self._mimic3_enabled = True
            if self._use_local_llm:
                print("🧠 [Narration] Hotspot mode detected - local LLM preferred")
        
        mode = "always" if force_use else ("hotspot" if self._use_local_llm else "if-available")
        print(f"🧠 [Narration] Local LLM configured - URL: {self._local_llm_url}, Mode: {mode}")
        print("🎙️ [Narration] Mimic3 TTS fallback enabled")
    
    def set_camera(self, picam2, vision_system, rtsp_url: str | None = None):
        """Set camera references for frame capture.

        Args:
            picam2: Picamera2 instance (may be None in MediaMTX-only mode)
            vision_system: VisionSystem instance (may be None)
            rtsp_url: RTSP URL for MediaMTX snapshot fallback
        """
        with self._lock:
            self._picam2 = picam2
            self._vision_system = vision_system
            if rtsp_url is not None:
                self._rtsp_url = rtsp_url
    
    def set_audio_manager(self, audio_manager):
        """Set the audio manager reference for volume ducking during narration.
        
        Args:
            audio_manager: CarAudioManager instance for volume control
        """
        with self._lock:
            self._audio_manager = audio_manager
        print("🔊 [Narration] Audio manager configured for volume ducking")
    
    def set_callback(self, callback):
        """Set the callback for narration text output."""
        self._on_narration_text = callback
    
    def set_error_callback(self, callback):
        """Set the callback for narration error output."""
        self._on_narration_error = callback

    def set_done_callback(self, callback):
        """Set the callback fired after each audio playback completes."""
        self._on_narration_done = callback

    @property
    def is_running(self) -> bool:
        return self._running
    
    def start(self):
        """Start the narration loop."""
        if self._running:
            return
        
        with self._lock:
            if not self._api_key or not self._model_name:
                print("❌ [Narration] Cannot start - no API key or model configured")
                return
        
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"🎙️ [Narration] Started (model={self._model_name}, interval={self._interval}s)")
    
    def stop(self):
        """Stop the narration loop."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        print("🎙️ [Narration] Stopped")

    def _run_single_pass(self) -> tuple[bool, str, bool]:
        """Run one capture -> describe -> speak pass.

        Returns:
            (success, message, empty_response)
            - success: True when narration text was generated and playback attempted
            - message: success text or error detail
            - empty_response: True when both Gemini and local LLM returned empty text
        """
        with self._lock:
            api_key = self._api_key
            model_name = self._model_name
            prompt = self._prompt
            picam2 = self._picam2
            vision_system = self._vision_system
            rtsp_url = self._rtsp_url
            elevenlabs_api_key = self._elevenlabs_api_key
            elevenlabs_voice_id = self._elevenlabs_voice_id
            elevenlabs_model_id = self._elevenlabs_model_id
            elevenlabs_volume = self._elevenlabs_volume
            kokoro_enabled = self._kokoro_enabled
            kokoro_ip = self._kokoro_ip
            kokoro_voice = self._kokoro_voice
            kokoro_speed = self._kokoro_speed
            kokoro_volume = self._kokoro_volume
            play_local_tts = self._play_local_tts
            audio_device = self._audio_device
            local_llm_url = self._local_llm_url
            use_local_llm = self._use_local_llm
            mimic3_enabled = self._mimic3_enabled
            mimic3_url = self._mimic3_url
            mimic3_audio_device = self._mimic3_audio_device
            audio_manager = self._audio_manager

        if not api_key or not model_name:
            msg = "Narration is not configured with API key and model"
            print(f"⚠️  [Narration] {msg}")
            return False, msg, False

        # Capture frame (with RTSP fallback for MediaMTX-only mode)
        print(f"\n🔬 [Narration] === SINGLE PASS START ===")
        print(f"🔬 [Narration] model={model_name}, rtsp_url={rtsp_url}")
        print(f"🔬 [Narration] picam2={'SET' if picam2 else 'None'}, vision={'SET' if vision_system else 'None'}")
        capture_start = time.time()
        jpeg_bytes = capture_frame_as_jpeg(picam2, vision_system, rtsp_url=rtsp_url)
        capture_elapsed = time.time() - capture_start
        if jpeg_bytes is None:
            msg = f"No camera frame available after {capture_elapsed:.1f}s — is the camera enabled?"
            print(f"⚠️  [Narration] {msg}")
            return False, msg, False
        print(f"🔬 [Narration] Frame captured: {len(jpeg_bytes)} bytes in {capture_elapsed:.1f}s")

        # Determine which AI to use: check hotspot mode and internet connectivity
        in_hotspot = is_hotspot_mode()
        if in_hotspot or use_local_llm:
            print("🌐 [Narration] Hotspot mode detected or local LLM forced - using local LLM")
            text = describe_image_local_llm(jpeg_bytes, prompt, local_llm_url)
        else:
            # Try Gemini first if we have API key and internet
            print(f"🔬 [Narration] Sending {len(jpeg_bytes)} bytes to Gemini ({model_name})...")
            start_time = time.time()
            text = describe_image(api_key, model_name, jpeg_bytes, prompt)
            elapsed = time.time() - start_time

            # If Gemini fails, fall back to local LLM
            if not text:
                print(f"⚠️  [Narration] Gemini failed or returned empty after {elapsed:.1f}s - falling back to local LLM")
                text = describe_image_local_llm(jpeg_bytes, prompt, local_llm_url)
            else:
                print(f"⏱️ [Narration] Gemini response in {elapsed:.1f}s")
                print(f"🔬 [Narration] AI RESPONSE ({len(text)} chars): {text[:300]}")

        if not text:
            msg = "AI returned empty responses from both Gemini and local LLM"
            print(f"🔬 [Narration] === SINGLE PASS FAILED (empty AI response) ===")
            return False, msg, True

        if self._on_narration_text:
            self._on_narration_text(text)

        # Audio playback priority (matches startup_systems_check.py pattern):
        # 1. ElevenLabs
        # 2. Kokoro
        # 3. Mimic3
        # 4. Local TTS (espeak-ng)
        #
        # Engine volume ducking is handled via on_playback_start / on_playback_end
        # callbacks passed into each TTS helper, so the engine only ducks while
        # actual audio is playing (not during synthesis/download).
        playback_success = False

        def _on_playback_start():
            if audio_manager:
                audio_manager.duck_engine_volume(True)

        def _on_playback_end():
            if audio_manager:
                audio_manager.duck_engine_volume(False)

        # PRIMARY: ElevenLabs
        if elevenlabs_api_key and elevenlabs_voice_id:
            print(f"🎤 [Narration] ElevenLabs: voice={elevenlabs_voice_id}, model={elevenlabs_model_id}...")
            playback_success = _play_elevenlabs_tts(
                text=text,
                api_key=elevenlabs_api_key,
                voice_id=elevenlabs_voice_id,
                model_id=elevenlabs_model_id,
                audio_device=audio_device,
                volume=elevenlabs_volume,
                on_playback_start=_on_playback_start,
                on_playback_end=_on_playback_end,
            )
            if playback_success:
                print("✅ [Narration] ElevenLabs playback: SUCCESS")
            else:
                print("⚠️  [Narration] ElevenLabs playback failed - trying Kokoro...")
        else:
            print("⚠️  [Narration] ElevenLabs not configured")

        # SECONDARY: Kokoro TTS
        if not playback_success and kokoro_enabled and kokoro_ip and kokoro_voice:
            print(f"🎤 [Narration] Kokoro: IP={kokoro_ip}, Voice={kokoro_voice}...")
            playback_success = _play_kokoro_with_volume(
                ip_address=kokoro_ip,
                text=text,
                voice=kokoro_voice,
                speed=kokoro_speed,
                volume=kokoro_volume,
                audio_device=audio_device,
                on_playback_start=_on_playback_start,
                on_playback_end=_on_playback_end,
            )
            if playback_success:
                print("✅ [Narration] Kokoro playback: SUCCESS")
            else:
                print("⚠️  [Narration] Kokoro playback failed - trying Mimic3...")
        elif not playback_success and not kokoro_enabled:
            print("⚠️  [Narration] Kokoro not configured")

        # TERTIARY: Mimic3 TTS
        if not playback_success and mimic3_enabled and mimic3_url:
            print("🎙️ [Narration] Trying Mimic3 TTS fallback...")
            playback_success = _play_mimic3_tts(
                text=text,
                mimic3_url=mimic3_url,
                audio_device=mimic3_audio_device,
                on_playback_start=_on_playback_start,
                on_playback_end=_on_playback_end,
            )
            if playback_success:
                print("✅ [Narration] Mimic3 playback: SUCCESS")
            else:
                print("⚠️  [Narration] Mimic3 playback failed - trying local TTS...")
        elif not playback_success:
            print("⚠️  [Narration] Mimic3 not configured")

        # QUATERNARY: Local TTS fallback (espeak-ng)
        # Uses synchronous playback to ensure ducking remains active
        # until speech finishes (espeak blocks on subprocess).
        if not playback_success and play_local_tts:
            print("🔊 [Narration] Using local TTS fallback (espeak-ng)...")
            _on_playback_start()
            try:
                tts = get_tts_synthesizer()
                tts.speak(text)
                # Wait for the async speak thread to finish so we don't
                # release the duck before audio completes.
                if tts._play_thread is not None:
                    tts._play_thread.join(timeout=60)
                playback_success = True
            finally:
                _on_playback_end()
        elif not playback_success:
            print("❌ [Narration] All TTS backends failed")

        # Signal that audio playback is done
        if self._on_narration_done:
            self._on_narration_done()

        return True, text, False

    def analyze_once(self) -> tuple[bool, str]:
        """Run one immediate narration pass (for manual UI-triggered analysis)."""
        print(f"🔬 [Narration] analyze_once() called, acquiring lock (timeout=10s)...")
        if not self._analysis_lock.acquire(timeout=10):
            msg = "Analysis already in progress"
            print(f"⚠️  [Narration] {msg}")
            return False, msg

        print(f"🔬 [Narration] analyze_once() lock acquired, running single pass...")
        try:
            success, message, _empty_response = self._run_single_pass()
            print(f"🔬 [Narration] analyze_once() result: success={success}, msg={message[:200] if message else 'None'}")
            if not success and self._on_narration_error:
                self._on_narration_error(message)
            return success, message
        except Exception as e:
            msg = str(e)
            print(f"❌ [Narration] One-shot analysis error: {msg}")
            import traceback; traceback.print_exc()
            if self._on_narration_error:
                self._on_narration_error(msg)
            return False, msg
        finally:
            print(f"🔬 [Narration] analyze_once() releasing lock")
            self._analysis_lock.release()
    
    def _loop(self):
        """Background narration loop."""
        consecutive_errors = 0
        while self._running:
            try:
                with self._analysis_lock:
                    success, message, empty_response = self._run_single_pass()

                if success:
                    consecutive_errors = 0
                elif empty_response:
                    consecutive_errors += 1
                    print(f"⚠️  [Narration] Empty response from both Gemini and local LLM (attempt {consecutive_errors})")
                    if consecutive_errors >= 3 and self._on_narration_error:
                        self._on_narration_error(
                            "AI returned empty responses from both Gemini and local LLM — check connectivity and LLM availability"
                        )
                        consecutive_errors = 0
                else:
                    consecutive_errors = 0
                    if message and self._on_narration_error:
                        self._on_narration_error(message)

            except Exception as e:
                print(f"❌ [Narration] Loop error: {e}")
                if self._on_narration_error:
                    self._on_narration_error(str(e))

            with self._lock:
                interval = self._interval

            # Wait for the configured interval
            # Use small sleep increments so we can stop quickly
            wait_end = time.time() + interval
            while self._running and time.time() < wait_end:
                time.sleep(0.5)
