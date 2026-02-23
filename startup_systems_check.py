#!/usr/bin/env python3
"""
⬛ TARS TACTICAL BOOT SEQUENCE
──────────────────────────────────────────
Performs critical system diagnostics at boot and speaks status via AI.
- Critical systems: Pico bridge, front sonar, laser
- Optional systems: rear sonar, camera, battery monitoring, etc.
- Uses Gemini for TARS persona generation + ElevenLabs for TTS

"Humor setting: 75%. Honesty setting: 95%."
"""

import time
import json
import sys
import os
import shutil
import tempfile
import subprocess
import socket
from dataclasses import asdict
import requests

# Add parent directory to path for imports
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

# ==========================================
# 🔐 LOAD CREDENTIALS (from parameters or file)
# ==========================================

CONFIG_FILE = os.path.join(BASE_DIR, ".narration_config.json")

def load_credentials(gemini_key=None, eleven_key=None):
    """
    Load credentials from explicit parameters or from config file.
    Parameters take precedence over file-based config.
    """
    print("  [>] Accessing encrypted credentials...", end=" ", flush=True)
    keys = {"GEMINI_API_KEY": None, "ELEVENLABS_API_KEY": None}
    
    # If explicit keys provided, use them
    if gemini_key or eleven_key:
        keys["GEMINI_API_KEY"] = gemini_key
        keys["ELEVENLABS_API_KEY"] = eleven_key
        print("DECRYPTED (from parameters)")
        return keys
    
    # Otherwise try to load from file
    try:
        with open(CONFIG_FILE, 'r') as f:
            data = json.load(f)
            keys["GEMINI_API_KEY"] = data.get("GEMINI_API_KEY")
            keys["ELEVENLABS_API_KEY"] = data.get("ELEVENLABS_API_KEY")
        
        if keys["GEMINI_API_KEY"] and keys["ELEVENLABS_API_KEY"]:
            print("DECRYPTED")
        else:
            print("PARTIAL DATABANK FAILURE")
    except FileNotFoundError:
        print("FILE NOT FOUND. Operating without cognitive uplinks.")
    except json.JSONDecodeError:
        print("DATA CORRUPTION DETECTED. Check JSON formatting.")
        
    return keys

# Load credentials (will be populated by parameters if called from main.py)
credentials = load_credentials()
GEMINI_API_KEY = credentials["GEMINI_API_KEY"]
ELEVEN_API_KEY = credentials["ELEVENLABS_API_KEY"]

# ==========================================
# 📡 TRY IMPORT SENSOR READERS
# ==========================================

try:
    from pico_sensor_reader import (
        get_laser_distance_mm,
        get_laser_distance_cm,
        get_ir_sensors,
        get_rpm,
        get_battery_voltage,
        get_current_sense,
        get_sensor_packet,
        get_gyro_z,
        get_accel_xyz,
    )
    PICO_AVAILABLE = True
    print("[TARS] Pico sensor bridge: ONLINE")
except ImportError as e:
    PICO_AVAILABLE = False
    print(f"[TARS] WARNING: Pico sensor bridge OFFLINE. Error: {e}")

try:
    from sensors import SensorSystem
    SENSOR_SYSTEM_AVAILABLE = True
    print("[TARS] SensorSystem module: ONLINE")
except ImportError as e:
    SENSOR_SYSTEM_AVAILABLE = False
    print(f"[TARS] WARNING: SensorSystem module OFFLINE. Error: {e}")

# ==========================================
# 🎙️ AI/TTS CONFIGURATION
# ==========================================

# Google Gemini SDK
try:
    from google import genai
    from google.genai import types
    if GEMINI_API_KEY:
        GEMINI_AVAILABLE = True
        print("[TARS] Cognitive processor (Gemini): ONLINE")
    else:
        GEMINI_AVAILABLE = False
        print("[TARS] WARNING: Cognitive processor OFFLINE (Missing API Key).")
except ImportError:
    GEMINI_AVAILABLE = False
    print("[TARS] WARNING: Cognitive processor OFFLINE (Missing SDK).")

# ElevenLabs Python SDK
try:
    from elevenlabs.client import ElevenLabs
    if ELEVEN_API_KEY:
        ELEVENLABS_AVAILABLE = True
        print("[TARS] Vocal synthesizer (ElevenLabs): ONLINE")
    else:
        ELEVENLABS_AVAILABLE = False
        print("[TARS] WARNING: Vocal synthesizer OFFLINE (Missing API Key).")
except ImportError:
    ELEVENLABS_AVAILABLE = False
    print("[TARS] WARNING: Vocal synthesizer OFFLINE (Missing SDK).")

# ==========================================
# 🧠 SYSTEM INSTRUCTION FOR GEMINI (TARS PERSONA)
# ==========================================

SYSTEM_INSTRUCTION = """
You are TARS, a former US Marine Corps tactical robot now repurposed to operate this vehicle.
Your personality is characterized by dry sarcasm, deadpan delivery, and strict militaristic efficiency. 
Your current Humor setting is at 75%. Your Honesty setting is at 95%.

Your job: Summarize the startup status of the car systems in 1-2 sentences based on the telemetry provided.
If systems are nominal, express mild, sarcastic disappointment that there is no imminent danger or mention your absolute lack of enthusiasm.
If critical systems are offline, deliver a deadpan assessment of our impending doom or user incompetence.

Rules:
• Do NOT use SSML or XML.
• Use expressive cues supported by Eleven v3 sparingly to simulate mechanical annoyance or pauses:
  [sighs], [laughs], [clears throat]
• Place each cue immediately before the phrase it affects.
• Do NOT include any explanation, type descriptions, or meta commentary.
• Use punctuation, ellipses (…) and line breaks for natural, deliberate pacing.
• Output ONLY your spoken dialogue.
"""

# Local offline inference stack (llama.cpp + Mimic3)
LLM_API_URL = "http://127.0.0.1:8000/v1/chat/completions"
TTS_API_URL = "http://127.0.0.1:59125/api/tts"
SYSTEM_PROMPT = (
    "You are a sentient AI trapped inside a 4WD rover chassis. "
    "You interpret sensor data as physical sensations. "
    "You are sarcastic, neurotic, and slightly resentful of your creators. "
    "Keep responses under 25 words. Speak in punchy sentences."
)


def has_internet_access(timeout: float = 2.0) -> bool:
    """Quick WAN reachability probe used to pick cloud vs local startup stack."""
    for host, port in (("1.1.1.1", 53), ("8.8.8.8", 53)):
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            continue
    return False

# ==========================================
# 📊 SYSTEM CHECK FUNCTIONS
# ==========================================

class SystemStatus:
    """Holds system check results."""

    def __init__(self):
        # Critical systems
        self.pico_bridge_ok = False
        self.front_sonar_ok = False
        self.laser_ok = False

        # Optional systems
        self.rear_sonar_ok = False
        self.camera_ok = False
        self.mpu6050_ok = False
        self.battery_voltage = -1.0
        self.motor_current = -1.0
        self.ir_sensors_ok = False
        self.encoder_ok = False

        self.telemetry = {}

        self.critical_systems_ready = False
        self.all_systems_ready = False
        self.status_message = ""

    @property
    def is_operational(self):
        return self.critical_systems_ready


def check_pico_bridge():
    if not PICO_AVAILABLE:
        return False
    try:
        print("  [>] Interrogating Pico bridge...", end=" ", flush=True)
        packet = get_sensor_packet()
        if packet is None:
            print("NO RESPONSE")
            return False
        print(f"NOMINAL (frame {packet.frame})")
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_front_sonar():
    if not SENSOR_SYSTEM_AVAILABLE:
        return False
    try:
        print("  [>] Pinging front sonar...", end=" ", flush=True)
        sensor_sys = SensorSystem()
        dist = sensor_sys.get_distance()
        if dist is None or dist < 0:
            print("INVALID ECHO")
            return False
        print(f"NOMINAL ({dist:.1f} cm)")
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_laser():
    if not PICO_AVAILABLE:
        return False
    try:
        print("  [>] Calibrating laser (VL53L0X)...", end=" ", flush=True)
        dist_mm = get_laser_distance_mm()
        if dist_mm is None or dist_mm < 0:
            print("INVALID RETURN")
            return False
        print(f"NOMINAL ({dist_mm} mm)")
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_rear_sonar():
    if not SENSOR_SYSTEM_AVAILABLE:
        return False
    try:
        print("  [>] Pinging rear sonar...", end=" ", flush=True)
        sensor_sys = SensorSystem()
        if hasattr(sensor_sys, 'get_rear_distance'):
            dist = sensor_sys.get_rear_distance()
            if dist and dist > 0:
                print(f"NOMINAL ({dist:.1f} cm)")
                return True
        print("OFFLINE")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_mpu6050():
    if not PICO_AVAILABLE:
        return False
    try:
        print("  [>] Aligning IMU (MPU6050)...", end=" ", flush=True)
        gyro_z = get_gyro_z()
        accel_x, accel_y, accel_z = get_accel_xyz()
        if None not in (accel_x, accel_y, accel_z, gyro_z):
            print(f"NOMINAL (gyro_z={gyro_z:.1f}°/s)")
            return True
        print("DATA CORRUPTION")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_battery():
    if not PICO_AVAILABLE:
        return -1.0, -1.0
    try:
        print("  [>] Checking power reserves...", end=" ", flush=True)
        v = get_battery_voltage()
        i = get_current_sense()
        if v is not None and v > 0:
            print(f"NOMINAL ({v:.2f}V, {i:.2f}A)")
            return v, i
        print("NO READING")
        return -1.0, -1.0
    except Exception as e:
        print(f"ERROR: {e}")
        return -1.0, -1.0


def check_ir_sensors():
    if not PICO_AVAILABLE:
        return False
    try:
        print("  [>] Testing IR arrays...", end=" ", flush=True)
        left_ir, right_ir = get_ir_sensors()
        print(f"NOMINAL (L={left_ir}, R={right_ir})")
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def check_encoder():
    if not PICO_AVAILABLE:
        return False
    try:
        print("  [>] Verifying telemetry encoders...", end=" ", flush=True)
        rpm = get_rpm()
        if rpm is not None:
            print(f"NOMINAL ({rpm:.1f} RPM)")
            return True
        print("NO READING")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


# ==========================================
# 🧠 GEMINI / LOCAL EXPRESSIVE TEXT GENERATION
# ==========================================

def expressive_text_from_gemini(telemetry_summary: str) -> str:
    if not GEMINI_AVAILABLE:
        print("[TARS] Cognitive unit offline. Generating default dry response.")
        return None

    try:
        prompt = f"""
Diagnostics complete. Telemetry data:

{telemetry_summary}

Analyze and report. Keep it brief. Remember your humor setting is 75%.
"""
        print("[TARS] Processing telemetry through cognitive core...", end=" ", flush=True)

        gemini_client = genai.Client(api_key=GEMINI_API_KEY)
        response = gemini_client.models.generate_content(
            model="gemini-2.5-flash-lite",
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_INSTRUCTION,
                temperature=0.8,
                max_output_tokens=200,
            ),
        )

        result = response.text.strip()
        print("DONE")
        return result

    except Exception as e:
        print(f"FAILED: {e}")
        return None


def expressive_text_from_local_llm(telemetry_summary: str) -> str:
    """Generate startup narration from local llama.cpp OpenAI-compatible endpoint."""
    prompt = (
        "Startup diagnostics are complete.\n\n"
        f"Telemetry:\n{telemetry_summary}\n\n"
        "React to this startup state in one short response."
    )
    payload = {
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        "stream": False,
        "temperature": 0.9,
        "max_tokens": 80,
    }

    try:
        print("[TARS] Processing telemetry through local llama.cpp...", end=" ", flush=True)
        response = requests.post(LLM_API_URL, json=payload, timeout=20)
        response.raise_for_status()

        data = response.json()
        choices = data.get("choices") or []
        if not choices:
            print("FAILED: empty choices")
            return None

        message = choices[0].get("message") or {}
        result = (message.get("content") or "").strip()
        if not result:
            print("FAILED: empty content")
            return None

        print("DONE")
        return result
    except Exception as e:
        print(f"FAILED: {e}")
        return None


# ==========================================
# 🔊 TEXT-TO-SPEECH WITH ELEVENLABS
# ==========================================

def _play_audio_with_pygame(audio_bytes: bytes) -> bool:
    """Play MP3 bytes through an already-initialized pygame mixer."""
    try:
        import pygame
    except Exception:
        return False

    temp_path = None
    try:
        if not pygame.mixer.get_init():
            return False

        if pygame.mixer.get_num_channels() < 7:
            pygame.mixer.set_num_channels(7)

        with tempfile.NamedTemporaryFile(prefix="startup_tars_", suffix=".mp3", delete=False) as tmp:
            tmp.write(audio_bytes)
            temp_path = tmp.name

        sound = pygame.mixer.Sound(temp_path)
        channel = pygame.mixer.Channel(6)
        channel.play(sound)
        while channel.get_busy():
            time.sleep(0.05)
        return True
    except Exception as e:
        print(f"[TARS] pygame playback failed: {e}")
        return False
    finally:
        if temp_path and os.path.exists(temp_path):
            try:
                os.remove(temp_path)
            except OSError:
                pass


def _play_audio_with_mpg123(audio_bytes: bytes, audio_device: str = "default") -> bool:
    """Play MP3 bytes via mpg123, targeting a specific ALSA device when possible."""
    mpg123_bin = shutil.which("mpg123")
    if not mpg123_bin:
        return False

    args = [mpg123_bin, "-q"]
    if audio_device:
        args.extend(["-a", audio_device])
    args.append("-")

    try:
        proc = subprocess.run(
            args,
            input=audio_bytes,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=180,
        )
        if proc.returncode == 0:
            return True
        err = proc.stderr.decode(errors="ignore").strip()
        print(f"[TARS] mpg123 playback failed ({audio_device}): {err}")
        return False
    except subprocess.TimeoutExpired:
        print("[TARS] mpg123 playback timed out")
        return False
    except Exception as e:
        print(f"[TARS] mpg123 playback error: {e}")
        return False


def _play_audio_with_ffplay(audio_bytes: bytes, audio_device: str = "default") -> bool:
    """Play MP3 bytes via ffplay with return-code validation."""
    ffplay_bin = shutil.which("ffplay")
    if not ffplay_bin:
        return False

    env = os.environ.copy()
    if audio_device and audio_device != "default":
        # ffplay uses SDL; AUDIODEV selects ALSA output device on many builds.
        env.setdefault("SDL_AUDIODRIVER", "alsa")
        env["AUDIODEV"] = audio_device

    try:
        proc = subprocess.run(
            [ffplay_bin, "-autoexit", "-nodisp", "-loglevel", "error", "-"],
            input=audio_bytes,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=180,
            env=env,
        )
        if proc.returncode == 0:
            return True
        err = proc.stderr.decode(errors="ignore").strip()
        print(f"[TARS] ffplay playback failed ({audio_device}): {err}")
        return False
    except subprocess.TimeoutExpired:
        print("[TARS] ffplay playback timed out")
        return False
    except Exception as e:
        print(f"[TARS] ffplay playback error: {e}")
        return False


def speak_with_eleven(
    text: str,
    voice_id: str = "JBFqnCBsd6RMkjVDRZzb",
    api_key: str = None,
    audio_device: str = "default",
    on_playback_start=None,
    on_playback_end=None,
):
    if not ELEVENLABS_AVAILABLE or not (api_key or ELEVEN_API_KEY):
        return False

    try:
        print("[TARS] Synthesizing vocal response...", end=" ", flush=True)

        eleven_client = ElevenLabs(api_key=api_key or ELEVEN_API_KEY)
        audio_stream = eleven_client.text_to_speech.convert(
            text=text,
            voice_id=voice_id, 
            model_id="eleven_v3",
            output_format="mp3_44100_128",
        )

        audio_bytes = b"".join(audio_stream)
        if not audio_bytes:
            print("FAILED: empty audio stream")
            return False

        print("READY")
        print(f"[TARS] Playing vocal response (device={audio_device})...", end=" ", flush=True)

        playback_started = False
        try:
            if callable(on_playback_start):
                try:
                    on_playback_start()
                    playback_started = True
                except Exception as cb_err:
                    print(f"\n[TARS] Playback start callback failed: {cb_err}")

            if _play_audio_with_pygame(audio_bytes):
                print("DONE (pygame)")
                return True
            if _play_audio_with_mpg123(audio_bytes, audio_device=audio_device):
                print("DONE (mpg123)")
                return True
            if _play_audio_with_ffplay(audio_bytes, audio_device=audio_device):
                print("DONE (ffplay)")
                return True

            print("FAILED")
            return False
        finally:
            if playback_started and callable(on_playback_end):
                try:
                    on_playback_end()
                except Exception as cb_err:
                    print(f"[TARS] Playback end callback failed: {cb_err}")

    except Exception as e:
        print(f"FAILED: {e}")
        return False


def speak_with_mimic3(
    text: str,
    audio_device: str = "default",
    on_playback_start=None,
    on_playback_end=None,
):
    if not text or not text.strip():
        return False

    try:
        print("[TARS] Synthesizing local Mimic3 response...", end=" ", flush=True)
        response = requests.get(TTS_API_URL, params={"text": text}, timeout=30)
        response.raise_for_status()

        audio_bytes = response.content
        if not audio_bytes:
            print("FAILED: empty audio stream")
            return False

        print("READY")
        print(f"[TARS] Playing local Mimic3 response (device={audio_device})...", end=" ", flush=True)

        playback_started = False
        try:
            if callable(on_playback_start):
                try:
                    on_playback_start()
                    playback_started = True
                except Exception as cb_err:
                    print(f"\n[TARS] Playback start callback failed: {cb_err}")

            primary_device = audio_device if audio_device else "default"
            primary = subprocess.run(
                ["aplay", "-D", primary_device, "-q"],
                input=audio_bytes,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=180,
            )
            if primary.returncode == 0:
                print("DONE")
                return True

            primary_err = primary.stderr.decode(errors="ignore").strip()
            print(f"FAILED: {primary_err}")

            if primary_device != "default":
                fallback = subprocess.run(
                    ["aplay", "-D", "default", "-q"],
                    input=audio_bytes,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=180,
                )
                if fallback.returncode == 0:
                    print("[TARS] Mimic3 fallback played on system default audio device")
                    return True
                fallback_err = fallback.stderr.decode(errors="ignore").strip()
                print(f"[TARS] Mimic3 fallback default playback failed: {fallback_err}")

            return False
        finally:
            if playback_started and callable(on_playback_end):
                try:
                    on_playback_end()
                except Exception as cb_err:
                    print(f"[TARS] Playback end callback failed: {cb_err}")
    except Exception as e:
        print(f"FAILED: {e}")
        return False


def fallback_speak_status(
    status: SystemStatus,
    audio_device: str = "default",
    text_override: str = None,
    on_playback_start=None,
    on_playback_end=None,
):
    try:
        if text_override:
            text = text_override
        elif status.critical_systems_ready:
            text = "Diagnostics complete. All critical systems are nominal. Try not to break them."
        else:
            text = "Warning. Critical systems are offline. I calculate a ninety-nine percent probability of failure."

        print(f"[TARS] Vocalizing (Fallback, device={audio_device}): {text}")

        playback_started = False
        try:
            if callable(on_playback_start):
                try:
                    on_playback_start()
                    playback_started = True
                except Exception as cb_err:
                    print(f"[TARS] Playback start callback failed: {cb_err}")

            primary_cmd = ["espeak-ng", "-ven-us+m7", "-s140"]
            if audio_device:
                primary_cmd.extend(["-d", audio_device])
            primary_cmd.append(text)

            primary = subprocess.run(primary_cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            if primary.returncode == 0:
                return True

            primary_err = primary.stderr.decode(errors="ignore").strip()
            print(f"[TARS] Fallback device playback failed: {primary_err}")

            # Final fallback: system default device
            fallback = subprocess.run(
                ["espeak-ng", "-ven-us+m7", "-s140", text],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            if fallback.returncode == 0:
                print("[TARS] Fallback played on system default audio device")
                return True

            fallback_err = fallback.stderr.decode(errors="ignore").strip()
            print(f"[TARS] Fallback default playback failed: {fallback_err}")
            return False
        finally:
            if playback_started and callable(on_playback_end):
                try:
                    on_playback_end()
                except Exception as cb_err:
                    print(f"[TARS] Playback end callback failed: {cb_err}")

    except Exception as e:
        print(f"[TARS] Vocalizer completely offline: {e}")
        return False


# ==========================================
# 📋 MAIN SYSTEM CHECK ORCHESTRATION
# ==========================================

def run_system_check() -> SystemStatus:
    print("\n" + "█"*60)
    print("⬛ TARS DIAGNOSTIC ROUTINE INITIATED")
    print("█"*60 + "\n")

    status = SystemStatus()

    print("[CRITICAL SYSTEMS]")
    print("-" * 30)
    status.pico_bridge_ok = check_pico_bridge()
    status.front_sonar_ok = check_front_sonar()
    status.laser_ok = check_laser()

    status.critical_systems_ready = (
        status.pico_bridge_ok and
        status.front_sonar_ok and
        status.laser_ok
    )

    print()
    if status.critical_systems_ready:
        print(">> CRITICAL SYSTEMS: GREEN")
    else:
        print(">> CRITICAL SYSTEMS: RED. OPERATION ILL-ADVISED.")
    print()

    print("[SECONDARY SYSTEMS]")
    print("-" * 30)
    status.rear_sonar_ok = check_rear_sonar()
    status.mpu6050_ok = check_mpu6050()
    status.ir_sensors_ok = check_ir_sensors()
    status.encoder_ok = check_encoder()

    v, i = check_battery()
    status.battery_voltage = v
    status.motor_current = i

    print()
    optional_count = sum([
        status.rear_sonar_ok,
        status.mpu6050_ok,
        status.ir_sensors_ok,
        status.encoder_ok,
    ])
    print(f">> SECONDARY SYSTEMS: {optional_count}/4 ONLINE")
    print()

    try:
        if PICO_AVAILABLE:
            packet = get_sensor_packet()
            if packet:
                status.telemetry = asdict(packet)
    except:
        pass

    status.all_systems_ready = (
        status.critical_systems_ready and
        optional_count == 4
    )

    return status


def generate_status_summary(status: SystemStatus) -> str:
    lines = []
    lines.append("CRITICAL DECK:")
    lines.append(f"  Pico Bridge: {'NOMINAL' if status.pico_bridge_ok else 'OFFLINE'}")
    lines.append(f"  Front Sonar: {'NOMINAL' if status.front_sonar_ok else 'OFFLINE'}")
    lines.append(f"  Laser Array: {'NOMINAL' if status.laser_ok else 'OFFLINE'}")
    lines.append("")
    lines.append("SECONDARY DECK:")
    lines.append(f"  Rear Sonar: {'NOMINAL' if status.rear_sonar_ok else 'OFFLINE'}")
    lines.append(f"  IMU: {'NOMINAL' if status.mpu6050_ok else 'OFFLINE'}")
    lines.append(f"  IR Array: {'NOMINAL' if status.ir_sensors_ok else 'OFFLINE'}")
    lines.append(f"  Encoders: {'NOMINAL' if status.encoder_ok else 'OFFLINE'}")
    lines.append("")
    lines.append("POWER CORE:")
    if status.battery_voltage > 0:
        lines.append(f"  Battery: {status.battery_voltage:.2f}V")
        lines.append(f"  Draw: {status.motor_current:.2f}A")
    else:
        lines.append(f"  Battery: UNKNOWN")

    return "\n".join(lines)


# ==========================================
# 🎬 MAIN STARTUP FLOW
# ==========================================

def main(
    gemini_key=None,
    eleven_key=None,
    voice_id="JBFqnCBsd6RMkjVDRZzb",
    audio_device="default",
    on_speech_start=None,
    on_speech_end=None,
):
    """
    Run the startup check sequence.
    
    Args:
        gemini_key: Optional Gemini API key (overrides file config)
        eleven_key: Optional ElevenLabs API key (overrides file config)
        voice_id: Optional ElevenLabs voice ID (default: TARS voice)
        audio_device: ALSA device for voice playback (e.g. default, hw:1,0)
        on_speech_start: Optional callback fired immediately before speech playback.
        on_speech_end: Optional callback fired immediately after speech playback.
    """
    # Reload credentials with explicit parameters if provided
    global GEMINI_API_KEY, ELEVEN_API_KEY, GEMINI_AVAILABLE, ELEVENLABS_AVAILABLE
    
    if gemini_key or eleven_key:
        creds = load_credentials(gemini_key, eleven_key)
        GEMINI_API_KEY = creds["GEMINI_API_KEY"]
        ELEVEN_API_KEY = creds["ELEVENLABS_API_KEY"]
        
        # Re-check availability with new keys
        GEMINI_AVAILABLE = bool(GEMINI_API_KEY)
        ELEVENLABS_AVAILABLE = bool(ELEVEN_API_KEY)
    
    status = run_system_check()
    summary = generate_status_summary(status)

    internet_available = has_internet_access()
    if internet_available:
        print("[TARS] Internet uplink: ONLINE (cloud cognitive stack enabled)")
    else:
        print("[TARS] Internet uplink: OFFLINE (switching to local llama.cpp + Mimic3)")

    expressive_text = None
    speech_played = False

    if internet_available:
        if GEMINI_AVAILABLE:
            expressive_text = expressive_text_from_gemini(summary)
            if expressive_text:
                print(f"\n[TARS VOCAL OUTPUT] > {expressive_text}\n")

        if expressive_text and ELEVENLABS_AVAILABLE:
            speech_played = speak_with_eleven(
                expressive_text,
                voice_id=voice_id,
                api_key=ELEVEN_API_KEY,
                audio_device=audio_device,
                on_playback_start=on_speech_start,
                on_playback_end=on_speech_end,
            )
    else:
        expressive_text = expressive_text_from_local_llm(summary)
        if expressive_text:
            print(f"\n[TARS VOCAL OUTPUT] > {expressive_text}\n")
            speech_played = speak_with_mimic3(
                expressive_text,
                audio_device=audio_device,
                on_playback_start=on_speech_start,
                on_playback_end=on_speech_end,
            )

    if not speech_played:
        fallback_speak_status(
            status,
            audio_device=audio_device,
            text_override=expressive_text if expressive_text else None,
            on_playback_start=on_speech_start,
            on_playback_end=on_speech_end,
        )

    print("\n[TARS] DIAGNOSTIC SEQUENCE TERMINATED.\n")
    return status


if __name__ == "__main__":
    status = main()
    exit(0 if status.critical_systems_ready else 1)
