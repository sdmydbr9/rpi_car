import os
# Prefer PulseAudio by default for Shairport Sync mixing, but allow
# systemd/service mode to override via CAR_AUDIO_DRIVER (e.g. "alsa").
_car_audio_driver_override = os.environ.get("CAR_AUDIO_DRIVER")
if _car_audio_driver_override:
    os.environ["SDL_AUDIODRIVER"] = _car_audio_driver_override
else:
    os.environ.setdefault("SDL_AUDIODRIVER", "pulseaudio")

# When started as a system service, PulseAudio socket vars may be missing.
# Auto-populate them from the current UID runtime dir if available.
_runtime_dir = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
_pulse_native = os.path.join(_runtime_dir, "pulse", "native")
if os.path.exists(_pulse_native):
    os.environ.setdefault("XDG_RUNTIME_DIR", _runtime_dir)
    os.environ.setdefault("PULSE_SERVER", f"unix:{_pulse_native}")

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    # Use mock GPIO when not on Raspberry Pi
    from motor import MockGPIO
    GPIO = MockGPIO()
    print("⚠️  RPi.GPIO not available - using mock GPIO for testing")
    
from flask import Flask, render_template, send_from_directory, jsonify, request, Response, make_response
from flask_cors import CORS
from flask_socketio import SocketIO, emit, join_room, leave_room
import os
import subprocess
import sys
import time
import threading
import socket
import re
import cv2
import json
import base64
import xml.etree.ElementTree as ET
from io import BytesIO
from enum import Enum
from collections import deque
from sensors import SensorSystem
from motor import CarSystem
from autopilot_pid import AutoPilot, State
from vision import VisionSystem
from narration import NarrationEngine, validate_key as narration_validate_key, list_multimodal_models
from kokoro_client import get_kokoro_client
from audio_manager import CarAudioManager
from network_core import PiCarNetworkManager
from pico_sensor_reader import (
    init_pico_reader, get_gyro_z as pico_get_gyro_z,
    get_accel_xyz as pico_get_accel_xyz,
    get_laser_distance_cm as pico_get_laser_cm,
    get_laser_distance_mm as pico_get_laser_mm,
    get_ir_sensors as pico_get_ir,
    get_rpm as pico_get_rpm,
    get_battery_voltage as pico_get_battery_voltage,
    get_current_sense as pico_get_current_sense,
    get_sensor_packet as pico_get_sensor_packet,
)

try:
    from PIL import Image
except Exception:
    Image = None

try:
    from colorthief import ColorThief
except Exception:
    ColorThief = None

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================

# --- PROJECT PATHS ---
# The folder where the built website ends up
DIST_DIR = "dist"
SOUNDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")

# --- MOTOR PINS (BCM Numbering) — Dual L298N, 4WD ---
# Managed by CarSystem in motor.py — listed here for wiring reference only.
# Driver 1 (Front): FL_IN1=17, FL_IN2=27, FL_ENA=12, FR_IN3=23, FR_IN4=22, FR_ENB=13
# Driver 2 (Rear):  RL_IN1=9,  RL_IN2=11, RL_ENA=26, RR_IN3=10, RR_IN4=7,  RR_ENB=16

# --- SONAR SENSOR PINS (BCM Numbering) ---
# Front Sonar: TRIG → Pin 22 (GPIO 25), ECHO → Pin 18 (GPIO 24)
SONAR_TRIG = 25  # GPIO 25 - Sonar Trigger
SONAR_ECHO = 24  # GPIO 24 - Sonar Echo

# --- IR OBSTACLE SENSORS ---
# IR sensors are now read from Pico sensor bridge (via UART)
# Legacy Pi GPIO pins kept as comments for wiring reference:
# Left Front IR: was GPIO 5, Right Front IR: was GPIO 6
AVOID_SWERVE_ANGLE = 85  # Degrees to swerve when obstacle detected (increased for aggressive steering)

# --- SPEED ENCODER (Rear-Right Wheel) ---
# Encoder now read from Pico sensor bridge (via UART)
# Legacy: was GPIO 26 on Pi, now on Pico GPIO 10
ENCODER_HOLES = 20      # Holes per revolution on encoder disc
WHEEL_DIAMETER_M = 0.065  # Wheel diameter in meters (65mm)
import math
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M  # ~0.2042m per revolution

# --- PHYSICS TUNING ---
ACCEL_RATE = 2.0    # How snappy the car speeds up (Higher = Faster)
COAST_RATE = 0.5    # How slowly it stops when you let go of gas (Coasting)
BRAKE_RATE = 1.5    # Brake deceleration rate (stops from 100 in ~3-4 seconds)
EMERGENCY_BRAKE_RATE = 5.0  # Emergency brake deceleration (faster stop on obstacle)
FREQ = 1000         # PWM Frequency (Hz)

# --- MOTOR PROTECTION ---
from motor import MAX_PWM_DUTY, PowerLimiter  # Voltage-based PWM hard limit
STEER_RATE_LIMIT_PER_TICK = 10   # Max degrees steering change per 20 ms tick (500°/s)
MIN_SPEED_FOR_GEAR_CHANGE = 5.0  # Max current_pwm to allow forward↔reverse gear switch
BRAKE_DECEL_RATE = 3.0           # Brake ramp-down rate (%/tick) — ~150 %/s, ~670 ms to stop from 100

# --- SONAR DISTANCE THRESHOLDS (cm) ---
SONAR_STOP_DISTANCE = 15      # Emergency stop below this distance
SONAR_CRAWL_DISTANCE = 25      # Very slow movement (5% speed) at this distance
SONAR_SLOW_DISTANCE = 40       # Start slowing down at this distance
SONAR_CAUTION_DISTANCE = 60     # Reduce speed moderately at this distance
SONAR_MAX_DISTANCE = 400        # Maximum reliable distance

# --- OBSTACLE AVOIDANCE POWER SETTINGS ---
REVERSE_POWER = 40            # Power level (%) for obstacle avoidance reverse (20-60 recommended)
MIN_REVERSE_POWER = 15         # Minimum power needed to move motors (adjust if motors don't start)

# --- CAMERA SETTINGS ---
# Legacy resolution names mapped to WxH for backward compatibility
_LEGACY_RESOLUTION_MAP = {
    "low": (640, 480),
    "medium": (1280, 720),
    "high": (1920, 1080),
}

def _parse_resolution(value: str):
    """Parse a resolution string into (width, height) tuple.
    Accepts WxH format (e.g. '1920x1080') or legacy keys ('low', 'medium', 'high').
    Returns None if the value is invalid."""
    if not value or not isinstance(value, str):
        return None
    # Check legacy keys first
    if value in _LEGACY_RESOLUTION_MAP:
        return _LEGACY_RESOLUTION_MAP[value]
    # Try WxH format
    match = re.match(r'^(\d+)x(\d+)$', value.strip())
    if match:
        return (int(match.group(1)), int(match.group(2)))
    return None

def _normalize_resolution(value: str) -> str:
    """Normalize a resolution string to WxH format.
    Converts legacy keys ('low') to WxH ('640x480'). Returns as-is if already WxH.
    Returns '640x480' as fallback for invalid values."""
    parsed = _parse_resolution(value)
    if parsed is None:
        print(f"⚠️  [Camera Config] Invalid resolution '{value}', falling back to 640x480")
        return "640x480"
    return f"{parsed[0]}x{parsed[1]}"

CAMERA_DEFAULT_RESOLUTION = "640x480"  # Default to 640x480 for best streaming performance
CAMERA_JPEG_QUALITY = 70           # JPEG compression quality (1-100, higher = better quality)
CAMERA_FRAMERATE = 30              # Camera framerate (FPS)

# --- CAMERA CONFIG PERSISTENCE ---
CAMERA_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.camera_config.json')

def _load_camera_config():
    """Load persisted camera configuration from JSON file."""
    try:
        if os.path.exists(CAMERA_CONFIG_FILE):
            with open(CAMERA_CONFIG_FILE, 'r') as f:
                config = json.load(f)
            print(f"✅ [Camera Config] Loaded persisted config: {config}")
            return config
    except Exception as e:
        print(f"⚠️  [Camera Config] Failed to load persisted config: {e}")
    return None

def _save_camera_config(config):
    """Save camera configuration to JSON file for persistence."""
    try:
        with open(CAMERA_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"💾 [Camera Config] Saved config to {CAMERA_CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️  [Camera Config] Failed to save config: {e}")

# --- NARRATION CONFIG PERSISTENCE ---
NARRATION_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.narration_config.json')

def _load_narration_config():
    """Load persisted narration configuration from JSON file."""
    try:
        if os.path.exists(NARRATION_CONFIG_FILE):
            with open(NARRATION_CONFIG_FILE, 'r') as f:
                config = json.load(f)
            print(f"\u2705 [Narration Config] Loaded persisted config")
            return config
    except Exception as e:
        print(f"\u26a0\ufe0f  [Narration Config] Failed to load: {e}")
    return {
        'provider': 'gemini',
        'api_key': '',
        'model': '',
        'interval': 30,
        'enabled': False,
        'models': [],
        'kokoro_enabled': False,
        'kokoro_ip': '',
        'kokoro_voice': '',
        'kokoro_voices': [],
        'elevenlabs_api_key': '',
        'elevenlabs_voice_id': 'JBFqnCBsd6RMkjVDRZzb',
    }

def _save_narration_config(config):
    """Save narration configuration to JSON file for persistence."""
    try:
        with open(NARRATION_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"\U0001f4be [Narration Config] Saved")
    except Exception as e:
        print(f"\u26a0\ufe0f  [Narration Config] Failed to save: {e}")

_narration_config = _load_narration_config()

# --- SENSOR CONFIG PERSISTENCE ---
SENSOR_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.sensor_config.json')

_SENSOR_CONFIG_DEFAULTS = {
    'ir_enabled': True,
    'sonar_enabled': True,
    'mpu6050_enabled': True,
    'rear_sonar_enabled': True,
}

def _load_sensor_config():
    """Load persisted sensor toggle states from JSON file."""
    try:
        if os.path.exists(SENSOR_CONFIG_FILE):
            with open(SENSOR_CONFIG_FILE, 'r') as f:
                config = json.load(f)
            # Merge with defaults so new keys get a value
            merged = {**_SENSOR_CONFIG_DEFAULTS, **config}
            print(f"✅ [Sensor Config] Loaded persisted config: {merged}")
            return merged
    except Exception as e:
        print(f"⚠️  [Sensor Config] Failed to load: {e}")
    return dict(_SENSOR_CONFIG_DEFAULTS)

def _save_sensor_config(config):
    """Save sensor toggle states to JSON file for persistence."""
    try:
        with open(SENSOR_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"💾 [Sensor Config] Saved to {SENSOR_CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️  [Sensor Config] Failed to save: {e}")

_sensor_config = _load_sensor_config()

# --- STARTUP CHECK CONFIG PERSISTENCE ---
STARTUP_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.startup_config.json')

_STARTUP_CONFIG_DEFAULTS = {
    'startup_check_enabled': True,
    'elevenlabs_api_key': '',
    'elevenlabs_voice_id': 'JBFqnCBsd6RMkjVDRZzb',  # Default TARS voice
}

def _load_startup_config():
    """Load persisted startup check configuration from JSON file."""
    try:
        if os.path.exists(STARTUP_CONFIG_FILE):
            with open(STARTUP_CONFIG_FILE, 'r') as f:
                config = json.load(f)
            # Merge with defaults so new keys get a value
            merged = {**_STARTUP_CONFIG_DEFAULTS, **config}
            print(f"✅ [Startup Config] Loaded persisted config")
            return merged
    except Exception as e:
        print(f"⚠️  [Startup Config] Failed to load: {e}")
    return dict(_STARTUP_CONFIG_DEFAULTS)

def _save_startup_config(config):
    """Save startup check configuration to JSON file for persistence."""
    try:
        with open(STARTUP_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"💾 [Startup Config] Saved to {STARTUP_CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️  [Startup Config] Failed to save: {e}")

_startup_config = _load_startup_config()

ELEVENLABS_DEFAULT_VOICE_ID = 'JBFqnCBsd6RMkjVDRZzb'

def _effective_elevenlabs_api_key() -> str:
    """Return the stored ElevenLabs key from startup or narration config."""
    return _startup_config.get('elevenlabs_api_key', '') or _narration_config.get('elevenlabs_api_key', '')

def _effective_elevenlabs_voice_id() -> str:
    """Return the stored ElevenLabs voice ID from startup or narration config."""
    return (
        _startup_config.get('elevenlabs_voice_id')
        or _narration_config.get('elevenlabs_voice_id')
        or ELEVENLABS_DEFAULT_VOICE_ID
    )

def _build_narration_config_sync_payload() -> dict:
    """Build narration config payload shared by connect/update events."""
    api_key = _narration_config.get('api_key', '')
    eleven_key = _effective_elevenlabs_api_key()
    return {
        'provider': _narration_config.get('provider', 'gemini'),
        'api_key_set': bool(api_key),
        'api_key_masked': ('*' * 8 + api_key[-4:]) if len(api_key) > 4 else '',
        'model': _narration_config.get('model', ''),
        'interval': _narration_config.get('interval', 30),
        'enabled': car_state.get('narration_enabled', False),
        'models': _narration_config.get('models', []),
        'kokoro_enabled': _narration_config.get('kokoro_enabled', False),
        'kokoro_ip': _narration_config.get('kokoro_ip', ''),
        'kokoro_voice': _narration_config.get('kokoro_voice', ''),
        'kokoro_voices': _narration_config.get('kokoro_voices', []),
        'elevenlabs_api_key_set': bool(eleven_key),
        'elevenlabs_api_key': eleven_key,
        'elevenlabs_api_key_masked': ('*' * 8 + eleven_key[-4:]) if len(eleven_key) > 4 else '',
        'elevenlabs_voice_id': _effective_elevenlabs_voice_id(),
    }

def _build_startup_config_sync_payload() -> dict:
    """Build startup config payload shared by connect/request/update events."""
    eleven_key = _effective_elevenlabs_api_key()
    return {
        'startup_check_enabled': _startup_config.get('startup_check_enabled', True),
        'elevenlabs_api_key_set': bool(eleven_key),
        'elevenlabs_api_key': eleven_key,
        'elevenlabs_voice_id': _effective_elevenlabs_voice_id(),
    }

def _sync_elevenlabs_config_between_files() -> None:
    """Keep startup/narration ElevenLabs values aligned across config files."""
    effective_key = _effective_elevenlabs_api_key()
    effective_voice_id = _effective_elevenlabs_voice_id()
    changed = False

    if _startup_config.get('elevenlabs_api_key', '') != effective_key:
        _startup_config['elevenlabs_api_key'] = effective_key
        changed = True
    if _narration_config.get('elevenlabs_api_key', '') != effective_key:
        _narration_config['elevenlabs_api_key'] = effective_key
        changed = True
    if _startup_config.get('elevenlabs_voice_id', ELEVENLABS_DEFAULT_VOICE_ID) != effective_voice_id:
        _startup_config['elevenlabs_voice_id'] = effective_voice_id
        changed = True
    if _narration_config.get('elevenlabs_voice_id', ELEVENLABS_DEFAULT_VOICE_ID) != effective_voice_id:
        _narration_config['elevenlabs_voice_id'] = effective_voice_id
        changed = True

    if changed:
        _save_startup_config(_startup_config)
        _save_narration_config(_narration_config)
        print("💾 [ElevenLabs] Synced startup and narration config values")

_sync_elevenlabs_config_between_files()

# Load persisted camera config (overrides defaults)
_persisted_camera_config = _load_camera_config()
if _persisted_camera_config:
    if 'resolution' in _persisted_camera_config and _parse_resolution(_persisted_camera_config['resolution']) is not None:
        CAMERA_DEFAULT_RESOLUTION = _normalize_resolution(_persisted_camera_config['resolution'])
        print(f"📷 [Camera Config] Loaded persisted resolution: '{_persisted_camera_config['resolution']}' → {CAMERA_DEFAULT_RESOLUTION}")
    else:
        print(f"⚠️  [Camera Config] Persisted resolution '{_persisted_camera_config.get('resolution', 'N/A')}' is invalid, using default {CAMERA_DEFAULT_RESOLUTION}")
    if 'jpeg_quality' in _persisted_camera_config:
        CAMERA_JPEG_QUALITY = int(_persisted_camera_config['jpeg_quality'])
    if 'framerate' in _persisted_camera_config:
        CAMERA_FRAMERATE = int(_persisted_camera_config['framerate'])
    print(f"📷 [Camera Config] Active config: res={CAMERA_DEFAULT_RESOLUTION}, quality={CAMERA_JPEG_QUALITY}, fps={CAMERA_FRAMERATE}")

# ==========================================
# 🛠️ AUTO-BUILDER
# ==========================================
def check_and_build():
    """Checks if the React UI is built. If not, runs 'npm run build'."""
    if not os.path.exists(DIST_DIR):
        print("⚠️  UI build not found. Building now...")
        print("⏳ This might take 2-3 minutes on a Raspberry Pi...")
        try:
            subprocess.run(["npm", "run", "build"], check=True)
            print("✅ Build Complete!")
        except Exception as e:
            print(f"❌ Build failed: {e}")
            print("   Make sure you are in the 'rpi_car' folder and npm dependencies are installed.")
            sys.exit(1)
    else:
        print("✅ UI found. Starting engine...")

# Run the check immediately
check_and_build()

# ==========================================
# 🔊 CAR AUDIO SETUP
# ==========================================
CAR_AUDIO_DEVICE = os.environ.get("CAR_AUDIO_DEVICE", "default")
try:
    CAR_IDLE_START_DELAY_S = float(os.environ.get("CAR_IDLE_START_DELAY_S", "3.5"))
except (TypeError, ValueError):
    CAR_IDLE_START_DELAY_S = 3.5
try:
    CAR_IDLE_TRIM_END_S = float(os.environ.get("CAR_IDLE_TRIM_END_S", "2.5"))
except (TypeError, ValueError):
    CAR_IDLE_TRIM_END_S = 2.5
try:
    CAR_IDLE_CROSSFADE_S = float(os.environ.get("CAR_IDLE_CROSSFADE_S", "0.12"))
except (TypeError, ValueError):
    CAR_IDLE_CROSSFADE_S = 0.12
try:
    CAR_IDLE_HANDOFF_OVERLAP_S = float(os.environ.get("CAR_IDLE_HANDOFF_OVERLAP_S", "0.18"))
except (TypeError, ValueError):
    CAR_IDLE_HANDOFF_OVERLAP_S = 0.18
car_audio = CarAudioManager(
    SOUNDS_DIR,
    audio_device=CAR_AUDIO_DEVICE,
    idle_start_delay_s=CAR_IDLE_START_DELAY_S,
    idle_trim_end_s=CAR_IDLE_TRIM_END_S,
    idle_crossfade_s=CAR_IDLE_CROSSFADE_S,
    idle_handoff_overlap_s=CAR_IDLE_HANDOFF_OVERLAP_S,
)
print(
    f"🔊 [Audio] Initialized (ready={car_audio.is_available()}, backend={car_audio.backend()}, device={CAR_AUDIO_DEVICE}, "
    f"idle_lead_in={CAR_IDLE_START_DELAY_S:.2f}s, "
    f"idle_trim_end={CAR_IDLE_TRIM_END_S:.2f}s, "
    f"idle_xfade={CAR_IDLE_CROSSFADE_S:.3f}s, "
    f"idle_handoff_overlap={CAR_IDLE_HANDOFF_OVERLAP_S:.3f}s, sounds={SOUNDS_DIR})"
)

# ==========================================
# 📷 CAMERA DETECTION
# ==========================================
def detect_camera_specs():
    """Detect installed camera model and supported resolutions using rpicam-hello.
    Falls back to OV5647 defaults if detection fails.
    Returns dict with model, max_resolution, and supported_modes."""
    try:
        result = subprocess.run(
            ["rpicam-hello", "--list-cameras"],
            capture_output=True,
            text=True,
            timeout=5
        )
        output = result.stdout
        
        # Parse camera model from output (e.g., "0 : ov5647 [2592x1944 ...")
        model_match = re.search(r':\s+(\w+)\s+\[', output)
        model = model_match.group(1).lower() if model_match else "ov5647"
        
        # Parse supported modes from "Modes:" section
        # Extract resolution pairs from mode lines (e.g., "640x480 [30.00 fps")
        modes_match = re.search(r"Modes:\s*([^\n]*(?:\n\s+[^\n]+)*)", output)
        supported_modes = []
        if modes_match:
            modes_text = modes_match.group(1)
            # Find all "WIDTHxHEIGHT" patterns
            for match in re.finditer(r'(\d+)x(\d+)', modes_text):
                res_str = f"{match.group(1)}x{match.group(2)}"
                if res_str not in supported_modes:
                    supported_modes.append(res_str)
        
        # Determine max resolution based on model
        max_res_map = {
            "ov5647": (2592, 1944),
            "imx219": (3280, 2464),
            "imx477": (4056, 3040),
            "ov64a40": (9152, 6944),
        }
        max_resolution = max_res_map.get(model, (2592, 1944))
        
        # If no modes detected, use defaults for the model
        if not supported_modes:
            supported_modes = {
                "ov5647": ["640x480", "1296x972", "1920x1080", "2592x1944"],
                "imx219": ["640x480", "1296x972", "1920x1080", "3280x2464"],
            }.get(model, ["640x480", "1296x972", "1920x1080", "2592x1944"])
        
        camera_info = {
            "model": model,
            "max_resolution": max_resolution,
            "supported_modes": supported_modes,
        }
        print(f"✅ [Camera] Detected {model.upper()} with max resolution {max_resolution[0]}x{max_resolution[1]}")
        print(f"   Available modes: {', '.join(supported_modes)}")
        return camera_info
    except Exception as e:
        print(f"⚠️  [Camera] Detection failed ({e}), using OV5647 defaults")
        return {
            "model": "ov5647",
            "max_resolution": (2592, 1944),
            "supported_modes": ["640x480", "1296x972", "1920x1080", "2592x1944"],
        }

camera_specs = detect_camera_specs()

# ==========================================
# 📷 CAMERA SETUP
# ==========================================
# Lock for camera reconfiguration
_camera_lock = threading.Lock()
_camera_restarting = False

try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    _init_resolution = _parse_resolution(CAMERA_DEFAULT_RESOLUTION) or (640, 480)
    print(f"📷 [Camera Init] Starting with resolution {CAMERA_DEFAULT_RESOLUTION} → {_init_resolution[0]}x{_init_resolution[1]}")
    cam_config = picam2.create_video_configuration(
        main={"size": _init_resolution, "format": "BGR888"}
    )
    picam2.configure(cam_config)
    picam2.start()
    CAMERA_AVAILABLE = True
    print(f"✅ Camera initialized successfully at {_init_resolution[0]}x{_init_resolution[1]}!")
except Exception as e:
    CAMERA_AVAILABLE = False
    picam2 = None
    print(f"⚠️  Camera not available: {e}")
    print("   Video feed will be disabled.")

# ==========================================
# 🧠 VISION / OBJECT DETECTION SETUP
# ==========================================
try:
    if CAMERA_AVAILABLE and picam2 is not None:
        vision_system = VisionSystem(picam2)
        vision_system.start()
        VISION_AVAILABLE = True
        print("✅ Vision system initialized (MobileNetSSD object detection)")
    else:
        vision_system = None
        VISION_AVAILABLE = False
        print("⚠️  Vision system disabled (camera not available)")
except Exception as e:
    vision_system = None
    VISION_AVAILABLE = False
    print(f"⚠️  Vision system initialization failed: {e}")

# ==========================================
# 🎙️ NARRATION ENGINE SETUP
# ==========================================
narration_engine = NarrationEngine()
narration_engine.set_camera(picam2, vision_system)
narration_engine.set_audio_manager(car_audio)

# Configure from persisted settings if API key exists
if _narration_config.get('api_key'):
    narration_engine.configure(
        api_key=_narration_config['api_key'],
        model_name=_narration_config.get('model', ''),
        interval=_narration_config.get('interval', 30),
    )
    print(f"\U0001f399\ufe0f [Narration] Engine configured from persisted config")

# Apply persisted ElevenLabs config to the engine (primary voice path)
narration_engine.set_elevenlabs_config(
    api_key=_effective_elevenlabs_api_key(),
    voice_id=_effective_elevenlabs_voice_id(),
)

# Apply persisted Kokoro TTS config to the engine
if _narration_config.get('kokoro_enabled') and _narration_config.get('kokoro_ip') and _narration_config.get('kokoro_voice'):
    narration_engine.set_kokoro_config(
        _narration_config['kokoro_ip'],
        _narration_config['kokoro_voice']
    )
    print(f"\U0001f3a4 [Kokoro] Restored persisted config: {_narration_config['kokoro_ip']} / {_narration_config['kokoro_voice']}")

# Configure local LLM fallback for hotspot mode
narration_engine.set_local_llm_config()

def _on_narration_text(text: str):
    """Callback when narration text is received from AI."""
    car_state["narration_speaking"] = True
    with app.app_context():
        socketio.emit('narration_text', {'text': text, 'timestamp': time.time()})
    print(f"\U0001f399\ufe0f [Narration] Broadcast: {text[:60]}...")

narration_engine.set_callback(_on_narration_text)

def _on_narration_done():
    """Callback when Pi-side audio playback finishes."""
    car_state["narration_speaking"] = False
    with app.app_context():
        socketio.emit('narration_speaking_done', {'timestamp': time.time()})
    print("🎙️ [Narration] Playback done — speaking state cleared")

narration_engine.set_done_callback(_on_narration_done)

def _on_narration_error(error_msg: str):
    """Callback when narration encounters an error."""
    with app.app_context():
        socketio.emit('narration_error', {'error': error_msg, 'timestamp': time.time()})
    print(f"\U0001f399\ufe0f [Narration] Error broadcast: {error_msg[:80]}")

narration_engine.set_error_callback(_on_narration_error)

# Auto-start narration engine if it was previously enabled (persist across restarts)
if _narration_config.get('enabled') and _narration_config.get('api_key') and _narration_config.get('model'):
    narration_engine.start()
    print(f"\U0001f399\ufe0f [Narration] Auto-started from persisted enabled state")

def _reconfigure_camera(resolution_str, framerate):
    """Reconfigure picam2 with new resolution and framerate. Must hold _camera_lock.
    resolution_str: WxH format string (e.g. '1920x1080') or legacy key ('low', 'medium', 'high')."""
    global _camera_restarting, CAMERA_AVAILABLE
    if picam2 is None:
        print(f"❌ [Camera] Cannot reconfigure - picam2 is None")
        return False
    _camera_restarting = True
    try:
        new_size = _parse_resolution(resolution_str)
        if new_size is None:
            print(f"❌ [Camera] Invalid resolution '{resolution_str}', cannot reconfigure")
            return False
        print(f"📷 [Camera] Reconfiguring: '{resolution_str}' → {new_size[0]}x{new_size[1]} @ {framerate}fps...")
        picam2.stop()
        cam_cfg = picam2.create_video_configuration(
            main={"size": new_size, "format": "BGR888"}
        )
        picam2.configure(cam_cfg)
        picam2.start()
        CAMERA_AVAILABLE = True
        print(f"✅ [Camera] Reconfigured successfully to {new_size[0]}x{new_size[1]} @ {framerate}fps")
        return True
    except Exception as e:
        print(f"❌ [Camera] Reconfiguration failed: {e}")
        # Try to recover with previous settings
        try:
            picam2.start()
            CAMERA_AVAILABLE = True
        except:
            CAMERA_AVAILABLE = False
        return False
    finally:
        _camera_restarting = False

class CameraFrameBroadcaster:
    """Shared camera frame broadcaster — one producer thread encodes frames,
    all /video_feed clients consume from a shared buffer.
    This prevents per-client encoding overhead and ensures all clients get frames."""

    def __init__(self):
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._frame = None          # Latest JPEG bytes
        self._frame_id = 0          # Monotonic frame counter
        self._running = False
        self._thread = None
        self._fps_frame_count = 0
        self._fps_last_time = time.time()

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._producer_loop, daemon=True)
        self._thread.start()
        print("📷 [CameraBroadcaster] ✅ Producer thread started")

    def stop(self):
        self._running = False

    def _producer_loop(self):
        """Continuously capture, encode, and store the latest JPEG frame."""
        while self._running:
            try:
                if not CAMERA_AVAILABLE or picam2 is None:
                    car_state["camera_actual_fps"] = 0.0
                    time.sleep(1)
                    continue

                # Wait during camera reconfiguration
                if _camera_restarting:
                    time.sleep(0.05)
                    continue

                # Check if camera is enabled
                if not car_state["camera_enabled"]:
                    car_state["camera_actual_fps"] = 0.0
                    self._fps_frame_count = 0
                    self._fps_last_time = time.time()
                    time.sleep(0.1)
                    continue

                # Get annotated frame from vision system if available
                frame = None
                if VISION_AVAILABLE and vision_system is not None:
                    frame = vision_system.get_frame()

                # Fallback to raw capture if vision not ready
                if frame is None:
                    frame = picam2.capture_array()

                # Convert BGR to RGB for standard color representation
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Get current JPEG quality from car_state
                jpeg_quality = car_state.get("camera_jpeg_quality", CAMERA_JPEG_QUALITY)
                ret, buffer = cv2.imencode('.jpg', frame_rgb, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()

                # Measure actual streaming FPS
                self._fps_frame_count += 1
                now = time.time()
                elapsed = now - self._fps_last_time
                if elapsed >= 1.0:
                    car_state["camera_actual_fps"] = round(self._fps_frame_count / elapsed, 1)
                    self._fps_frame_count = 0
                    self._fps_last_time = now

                # Publish frame to all waiting consumers
                with self._condition:
                    self._frame = frame_bytes
                    self._frame_id += 1
                    self._condition.notify_all()

            except Exception as e:
                print(f"❌ [CameraBroadcaster] Frame error: {e}")
                time.sleep(0.1)

    def get_frame(self, timeout=2.0):
        """Block until a new frame is available. Returns JPEG bytes or None on timeout."""
        with self._condition:
            current_id = self._frame_id
            # Wait for a new frame (frame_id to change)
            self._condition.wait_for(
                lambda: self._frame_id != current_id or not self._running,
                timeout=timeout
            )
            if not self._running:
                return None
            return self._frame


# Create global broadcaster instance
_camera_broadcaster = CameraFrameBroadcaster()


def generate_camera_frames():
    """Generator that yields MJPEG frames from the shared broadcaster.
    Each client gets its own generator but all consume from the same
    pre-encoded frame — no per-client capture or JPEG encoding."""
    while True:
        try:
            frame_bytes = _camera_broadcaster.get_frame(timeout=2.0)
            if frame_bytes is None:
                # Timeout or stopped — yield nothing, keep connection alive
                time.sleep(0.1)
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except GeneratorExit:
            return
        except Exception as e:
            print(f"❌ Camera frame error: {e}")
            time.sleep(0.1)

# ==========================================
# 🚗 HARDWARE SETUP
# ==========================================

# CarSystem (motor.py) owns ALL motor GPIO: direction pins, PWM
# channels, and the lgpio chip handle.  Do NOT call GPIO.setmode()
# or GPIO.setup() on motor pins here — it opens a second lgpio
# handle that becomes stale once CarSystem re-opens the chip,
# causing silent "unknown handle" errors in the physics_loop.

car_system = CarSystem()

# GPIO mode is now set by CarSystem; sensor pins are set up below.
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# IR sensors are now on the Pico — no GPIO.setup needed on the Pi.

# Initialize Pico Sensor Bridge (UART)
try:
    pico_reader = init_pico_reader('/dev/ttyS0')
    print("✅ Pico sensor bridge initialized (UART)")
except Exception as e:
    pico_reader = None
    print(f"⚠️  Pico sensor bridge initialization error: {e}")

# Initialize Sensor System (servo + rear sonar stay on Pi)
try:
    sensor_system = SensorSystem()
    print("✅ Sensor system initialized (Servo + Rear Sonar)")
except Exception as e:
    print(f"⚠️  Sensor system initialization error: {e}")
    # Create dummy sensor system for testing
    class DummySensorSystem:
        def get_sonar_distance(self): return 100
        def get_rear_sonar_distance(self): return 100
        def get_ir_status(self): return False, False
        def set_servo_angle(self, a): pass
        def center_servo(self): pass
        def scan_sweep(self, **kw): return []
        def cleanup(self): pass
    sensor_system = DummySensorSystem()

# ==========================================
# 🏎️ AUTOPILOT INSTANTIATION
# ==========================================
# CarSystem was already created above (before sensor init) to avoid
# duplicate-PWM conflicts.  Only the autopilot wiring remains here.

def _get_laser_for_autopilot():
    """Read forward laser distance (smoothed) for the autopilot."""
    return get_smoothed_laser()

def _get_rear_for_autopilot():
    """Read rear HC-SR04 sonar distance for the autopilot."""
    return sensor_system.get_rear_sonar_distance()

def _get_ir_for_autopilot():
    """Read IR sensors from Pico bridge: True = obstacle detected."""
    try:
        left, right = pico_get_ir()
    except Exception:
        left = False
        right = False
    return left, right

autopilot = AutoPilot(
    car_system,
    _get_laser_for_autopilot,
    _get_ir_for_autopilot,
    sensor_system=sensor_system,
    get_rear_distance=_get_rear_for_autopilot,
)

# Store a copy of the original class-level defaults (immutable reference for reset)
_AUTOPILOT_DEFAULT_TUNING = AutoPilot.get_default_tuning()

# Active tuning state — persists across UI reconnections
_active_tuning = dict(_AUTOPILOT_DEFAULT_TUNING)

# ==========================================
# 🧠 PHYSICS ENGINE (Background Thread)
# ==========================================

# Shared Memory (The "Brain" of the car)
car_state = {
    "gear": "1",          # Current Gear: R, N, 1, 2, 3, S
    "gas_pressed": False, # Is throttle held?
    "brake_pressed": False, # Is brake held?
    "direction": "stop",  # stop, forward, backward
    "turning": "straight",# straight, left, right
    "current_pwm": 0.0,   # The actual speed (0-100) right now
    "steer_angle": 0,      # Current steering angle
    "left_obstacle": False, # IR sensor: obstacle detected on left
    "right_obstacle": False, # IR sensor: obstacle detected on right
    "ir_enabled": _sensor_config.get('ir_enabled', True),    # IR sensor toggle
    "mpu6050_enabled": _sensor_config.get('mpu6050_enabled', True),  # MPU6050 gyroscope toggle
    "rear_sonar_enabled": _sensor_config.get('rear_sonar_enabled', True),  # Rear sonar sensor toggle
    "user_steer_angle": 0, # Store user's manual steering (preserved when avoiding)
    "obstacle_avoidance_active": False,  # Track if currently avoiding
    "speed_limit": 100,    # Manual speed limit (5-100), overrides gear speeds
    "speed_limit_enabled": False,  # Whether to use speed limit instead of gear
    "auto_accel_enabled": False,  # Auto-acceleration mode (client-side for throttle)
    "engine_running": False,  # Engine start/stop state from UI power button
    "emergency_brake_active": False,  # Emergency brake: when ON, car cannot move
    "gear_before_ebrake": None,  # Store gear before emergency brake was activated
    "obstacle_state": "IDLE",  # Obstacle avoidance state: IDLE, STOPPED, STEERING
    "is_braking": False,  # Flag to indicate normal brake is applied for motor control
    "heartbeat_active": True,  # Heartbeat status: True = client responding, False = lost connection
    "sonar_distance": 100,  # Distance from front sonar sensor in cm
    "sonar_enabled": _sensor_config.get('sonar_enabled', True),   # Sonar sensor toggle
    # 🤖 AUTONOMOUS DRIVING MODE
    "autonomous_mode": False,  # Smart Driver autonomous mode toggle
    "autonomous_state": State.CRUISING.value,  # Current autonomous driving state (read from AutoPilot)
    "laser_scan_data": [],                        # Latest sweep scan data [{angle, distance}, ...]
    "last_obstacle_side": "none",  # Track which side detected obstacle for escape logic
    # 🧭 Slalom yaw-tracking telemetry
    "target_yaw": 0.0,              # Target yaw heading (degrees)
    "current_heading": 0.0,         # Current integrated yaw heading (degrees)
    "slalom_sign": 0,               # Dodge direction: -1=left, 0=none, 1=right
    # 🚨 SENSOR HEALTH STATUS
    "sensor_status": {
        "front_sonar": "OK",    # HC-SR04 ultrasonic on Pi GPIO 25/24
        "laser": "OK",          # VL53L0X laser rangefinder via Pico
        "left_ir": "OK",
        "right_ir": "OK",
        "mpu6050": "OK",
        "pico_bridge": "OK",
        "camera": "OK",
        "voltage_sensor": "OK",  # ADS1115 battery voltage ADC
        "speaker_amp": "OK",     # MAX98357 I2S amplifier + speaker
    },
    "service_light_active": False,  # True if any sensor has error/warning
    # 📷 CAMERA / VISION / OBJECT DETECTION
    "camera_enabled": False,         # Camera master toggle (disabled by default)
    "user_wants_vision": False,      # User's explicit CV toggle preference
    "vision_active": False,          # True if vision DNN is running
    "camera_obstacle_distance": 999.0,  # Virtual distance from camera (cm)
    "camera_detections_count": 0,     # Number of detected objects
    "camera_in_path_count": 0,        # Objects in driving path
    "camera_closest_object": "",      # Closest in-path object class
    "camera_closest_confidence": 0,   # Confidence % of closest detection
    "vision_fps": 0.0,               # DNN inference FPS
    "camera_actual_fps": 0.0,        # Measured streaming FPS (actual frames/sec)
    # 🧭 MPU6050 Gyro telemetry
    "gyro_z": 0.0,                     # Current Z-axis yaw rate (°/s)
    "pid_correction": 0.0,             # Current PID heading correction
    "gyro_available": False,           # MPU6050 hardware detected
    "gyro_calibrated": False,          # Gyro has been calibrated
    # 🔄 Speed Encoder (Rear-Right Wheel)
    "encoder_rpm": 0.0,                  # Real RPM from encoder
    "encoder_speed_mpm": 0.0,            # Speed in meters per minute
    "encoder_available": False,          # True if encoder GPIO setup succeeded
    # 🔋 Battery / Current (from Pico ADC)
    "battery_voltage": -1,                   # Battery voltage in V (from ADS1115 A0)
    "current_amps": -1,                      # Current draw in A (from ADS1115 A1)
    # ⚡ Dynamic power limiter telemetry
    "effective_max_duty": MAX_PWM_DUTY,       # Current dynamic duty cap (%)
    "l298n_voltage_drop": 1.5,               # Estimated L298N driver drop (V)
    "effective_motor_voltage": 0.0,           # Estimated motor-terminal voltage (V)
    # Camera configuration settings
    "camera_resolution": CAMERA_DEFAULT_RESOLUTION,  # Current resolution setting (WxH format, e.g. '640x480')
    "camera_jpeg_quality": CAMERA_JPEG_QUALITY,      # JPEG quality (1-100)
    "camera_framerate": CAMERA_FRAMERATE,            # Camera framerate (FPS)
    # 🎙️ AI NARRATION
    "narration_enabled": _narration_config.get('enabled', False),  # Restore from persisted config
    "narration_provider": _narration_config.get('provider', 'gemini'),
    "narration_model": _narration_config.get('model', ''),
    "narration_interval": _narration_config.get('interval', 8),
    "narration_speaking": False,  # True when TTS is playing on client
}

# ==========================================
# 🎵 AIRPLAY NOW-PLAYING METADATA
# ==========================================
AIRPLAY_METADATA_PIPE_PATH = os.getenv("RC_AIRPLAY_METADATA_PIPE", "/tmp/shairport-sync-metadata")

_now_playing_lock = threading.Lock()
_now_playing_state = {
    "status": "Waiting for AirPlay...",
    "volume": "--",
    "track": "--",
    "artist": "--",
    "album": "--",
    "image": "",
    "bg_color": "#121212",
    "progress": None,  # {"current_sec": float, "total_sec": float, "sync_time": float}
}

def _decode_airplay_text_payload(data_text: str):
    if not data_text:
        return None
    try:
        return base64.b64decode(data_text).decode("utf-8")
    except Exception:
        return None

def _extract_album_bg_color(clean_b64: str) -> str:
    if not clean_b64 or Image is None or ColorThief is None:
        return "#121212"
    try:
        image_data = base64.b64decode(clean_b64)
        with BytesIO(image_data) as img_io:
            with Image.open(img_io) as img:
                img = img.convert("RGB")
                img.thumbnail((150, 150))
                with BytesIO() as thumb_io:
                    img.save(thumb_io, format="JPEG")
                    thumb_io.seek(0)
                    color_thief = ColorThief(thumb_io)
                    dominant_color = color_thief.get_color(quality=1)
                    r, g, b = [int(c * 0.6) for c in dominant_color]
                    return f"rgb({r}, {g}, {b})"
    except Exception:
        return "#121212"

def _process_airplay_metadata_item(xml_string: str):
    try:
        root = ET.fromstring(xml_string)
        type_hex = root.findtext("type") or ""
        code_hex = root.findtext("code") or ""
        type_tag = bytes.fromhex(type_hex).decode("utf-8")
        code_tag = bytes.fromhex(code_hex).decode("utf-8")
        data_node = root.find("data")
        data_text = data_node.text if data_node is not None else None
    except Exception:
        return

    if type_tag == "ssnc":
        if code_tag in ("pbeg", "prsm"):
            with _now_playing_lock:
                _now_playing_state["status"] = "Playing"
                progress = _now_playing_state.get("progress")
                if isinstance(progress, dict):
                    progress["sync_time"] = time.time()
        elif code_tag == "paus":
            with _now_playing_lock:
                _now_playing_state["status"] = "Paused"
                progress = _now_playing_state.get("progress")
                if isinstance(progress, dict):
                    elapsed = time.time() - progress.get("sync_time", time.time())
                    progress["current_sec"] = progress.get("current_sec", 0.0) + elapsed
                    progress["sync_time"] = time.time()
        elif code_tag == "pend":
            with _now_playing_lock:
                _now_playing_state["status"] = "Stopped"
                _now_playing_state["progress"] = None
        elif code_tag == "pvol":
            vol_data = _decode_airplay_text_payload(data_text)
            if vol_data:
                parts = vol_data.split(",")
                if len(parts) >= 4:
                    try:
                        current_vol = float(parts[0])
                        low_vol = float(parts[2])
                        high_vol = float(parts[3])
                        pct = int(((current_vol - low_vol) / (high_vol - low_vol)) * 100) if high_vol != low_vol else 100
                        pct = max(0, min(100, pct))
                        with _now_playing_lock:
                            _now_playing_state["volume"] = f"{pct}%"
                    except Exception:
                        with _now_playing_lock:
                            _now_playing_state["volume"] = "--"
        elif code_tag == "prgr":
            prgr_data = _decode_airplay_text_payload(data_text)
            if prgr_data:
                parts = prgr_data.split("/")
                if len(parts) == 3:
                    try:
                        start_rtp, current_rtp, end_rtp = [int(p) for p in parts]
                        total_frames = (end_rtp - start_rtp) % (2 ** 32)
                        current_frames = (current_rtp - start_rtp) % (2 ** 32)
                        with _now_playing_lock:
                            _now_playing_state["progress"] = {
                                "current_sec": current_frames / 44100.0,
                                "total_sec": total_frames / 44100.0,
                                "sync_time": time.time(),
                            }
                    except Exception:
                        pass
        elif code_tag == "PICT" and data_text:
            clean_b64 = data_text.replace("\n", "").replace("\r", "")
            bg_color = _extract_album_bg_color(clean_b64)
            with _now_playing_lock:
                _now_playing_state["image"] = f"data:image/jpeg;base64,{clean_b64}"
                _now_playing_state["bg_color"] = bg_color
    elif type_tag == "core":
        value = _decode_airplay_text_payload(data_text) or "--"
        with _now_playing_lock:
            if code_tag == "minm":
                _now_playing_state["track"] = value
            elif code_tag == "asar":
                _now_playing_state["artist"] = value
            elif code_tag == "asal":
                _now_playing_state["album"] = value

def _format_now_playing_time(seconds: float) -> str:
    if seconds < 0:
        seconds = 0
    minutes = int(seconds) // 60
    remainder = int(seconds) % 60
    return f"{minutes}:{remainder:02d}"

def _build_now_playing_response():
    with _now_playing_lock:
        snapshot = _now_playing_state.copy()
        progress = snapshot.get("progress")
        if isinstance(progress, dict):
            progress = progress.copy()
        snapshot["progress"] = progress

    if isinstance(progress, dict):
        current_sec = float(progress.get("current_sec", 0.0))
        total_sec = float(progress.get("total_sec", 0.0))
        if snapshot.get("status") == "Playing":
            elapsed = time.time() - float(progress.get("sync_time", time.time()))
            current_sec = min(current_sec + elapsed, total_sec)
        snapshot["time_str"] = f"{_format_now_playing_time(current_sec)} / {_format_now_playing_time(total_sec)}"
        snapshot["progress_pct"] = (current_sec / total_sec * 100.0) if total_sec > 0 else 0.0
    else:
        snapshot["time_str"] = "0:00 / 0:00"
        snapshot["progress_pct"] = 0.0

    snapshot.pop("progress", None)
    return snapshot

def _airplay_metadata_reader():
    while True:
        if not os.path.exists(AIRPLAY_METADATA_PIPE_PATH):
            time.sleep(1.0)
            continue
        try:
            with open(AIRPLAY_METADATA_PIPE_PATH, "r") as pipe:
                item_xml = ""
                in_item = False
                for line in pipe:
                    if "<item>" in line:
                        in_item = True
                        item_xml = line
                    elif in_item:
                        item_xml += line
                    if "</item>" in line and in_item:
                        _process_airplay_metadata_item(item_xml)
                        in_item = False
        except Exception:
            time.sleep(0.2)

# ==========================================
# 💗 HEARTBEAT / PING TRACKING
# ==========================================
# Dictionary to track last ping time from each connected client
client_last_pong = {}  # {client_id: timestamp}
has_had_client = False  # Track if a client has ever connected (to avoid false emergency brakes on startup)

# Heartbeat configuration
HEARTBEAT_INTERVAL = 2.0  # Send ping every 2 seconds
HEARTBEAT_TIMEOUT = 5.0   # Declare client dead if no pong for 5 seconds

# Startup-check runtime guard (prevents overlapping checks from auto/manual triggers)
_startup_check_lock = threading.Lock()
_startup_check_running = False

# ==========================================
# 📡 LASER MOVING-AVERAGE FILTER
# ==========================================
# Circular buffer of last 5 laser readings for noise rejection.
# At 20Hz autopilot rate, this spans ~250ms — fast enough for
# real-time, smooth enough to reject single bad readings.
laser_buffer = deque(maxlen=5)

def get_smoothed_laser():
    """Read forward laser from Pico bridge and return a moving-average smoothed distance."""
    try:
        raw = pico_get_laser_cm()
    except:
        raw = -1
    if raw > 0:
        laser_buffer.append(raw)
    if laser_buffer:
        return sum(laser_buffer) / len(laser_buffer)
    return 100  # Safe default

# Backward-compat alias used by physics_loop
def get_smoothed_sonar():
    return get_smoothed_laser()

# ==========================================
# 🚨 SENSOR HEALTH CHECK
# ==========================================

def check_sensor_health():
    """
    Periodically check sensor health and update car_state["sensor_status"].
    Uses Pico bridge packet freshness to detect stale/dead sensors.
    Returns a dict with health status for each sensor.
    """
    sensor_status = {
        "front_sonar": "OK",
        "laser": "OK",
        "left_ir": "OK",
        "right_ir": "OK",
        "mpu6050": "OK",
        "pico_bridge": "OK",
        "camera": "OK",
        "voltage_sensor": "OK",
        "speaker_amp": "OK",
    }
    
    has_error = False

    # ── 1. Check Pico Bridge connection & freshness ──
    pico_fresh = False
    try:
        if pico_reader is not None and pico_reader.is_connected():
            if pico_reader.is_fresh(max_age_s=3.0):
                sensor_status["pico_bridge"] = "OK"
                pico_fresh = True
            else:
                sensor_status["pico_bridge"] = "WARNING"
                has_error = True
        else:
            sensor_status["pico_bridge"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["pico_bridge"] = "FAILED"
        has_error = True

    # ── 2. Check Pico packet-level sensor data ──
    pico_packet = None
    if pico_fresh:
        try:
            pico_packet = pico_get_sensor_packet()
        except Exception:
            pico_packet = None

    # ── 3. Check Laser Rangefinder (VL53L0X via Pico) ──
    try:
        if pico_fresh and pico_packet is not None:
            if pico_packet.laser_mm <= 0:
                sensor_status["laser"] = "FAILED"
                has_error = True
            else:
                sensor_status["laser"] = "OK"
        else:
            # Pico not fresh — laser unavailable
            sensor_status["laser"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["laser"] = "FAILED"
        has_error = True

    # ── 3b. Check Front Sonar (HC-SR04 on Pi GPIO 25/24) ──
    try:
        if hasattr(sensor_system, '_rear_sonar_available') and sensor_system._rear_sonar_available:
            # HC-SR04 GPIO init succeeded — try a quick ping
            dist = sensor_system.get_rear_sonar_distance()
            if dist > 0:
                sensor_status["front_sonar"] = "OK"
            else:
                # Hardware present but reading failed (no echo)
                sensor_status["front_sonar"] = "WARNING"
                has_error = True
        else:
            sensor_status["front_sonar"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["front_sonar"] = "FAILED"
        has_error = True

    # ── 4. Check IR Sensors (via Pico) ──
    try:
        if pico_fresh and pico_packet is not None:
            # IR sensors are on the Pico — if we have fresh data they're working
            sensor_status["left_ir"] = "OK"
            sensor_status["right_ir"] = "OK"
        else:
            # Pico stale/down — IR sensors unavailable
            sensor_status["left_ir"] = "FAILED"
            sensor_status["right_ir"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["left_ir"] = "FAILED"
        sensor_status["right_ir"] = "FAILED"
        has_error = True

    # ── 5. Check MPU6050 Gyroscope (via Pico bridge) ──
    try:
        if pico_fresh and pico_packet is not None:
            # Check if gyro values are plausible (not NaN/extreme)
            if abs(pico_packet.gyro_z) < 2000:
                sensor_status["mpu6050"] = "OK"
            else:
                sensor_status["mpu6050"] = "WARNING"
                has_error = True
        elif autopilot._gyro.available:
            try:
                autopilot._gyro.read_gyro_z()
                sensor_status["mpu6050"] = "OK"
            except Exception:
                sensor_status["mpu6050"] = "WARNING"
                has_error = True
        else:
            sensor_status["mpu6050"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["mpu6050"] = "FAILED"
        has_error = True

    # ── 6. Check Pico error counter ──
    if pico_fresh and pico_packet is not None and pico_packet.errors > 50:
        # High error count on Pico indicates hardware issues
        if sensor_status["pico_bridge"] == "OK":
            sensor_status["pico_bridge"] = "WARNING"
            has_error = True

    # ── 7. Check Camera ──
    try:
        if not CAMERA_AVAILABLE or picam2 is None:
            sensor_status["camera"] = "FAILED"
            has_error = True
        else:
            sensor_status["camera"] = "OK"
    except Exception:
        sensor_status["camera"] = "FAILED"
        has_error = True
    
    # ── 8. Check Voltage Sensor (ADS1115 ADC via Pico) ──
    try:
        batt_v = car_state.get("battery_voltage", -1)
        if batt_v > 0:
            sensor_status["voltage_sensor"] = "OK"
        elif pico_fresh:
            # Pico alive but no valid voltage reading
            sensor_status["voltage_sensor"] = "WARNING"
            has_error = True
        else:
            sensor_status["voltage_sensor"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["voltage_sensor"] = "FAILED"
        has_error = True

    # ── 9. Check Speaker + Amplifier (MAX98357 I2S) ──
    try:
        import subprocess
        # Check if ALSA audio output device is available
        result = subprocess.run(
            ['aplay', '-l'],
            capture_output=True, timeout=3
        )
        if result.returncode == 0 and len(result.stdout) > 0:
            # Check for any playback device in the output
            output = result.stdout.decode(errors='ignore')
            if 'card' in output.lower():
                sensor_status["speaker_amp"] = "OK"
            else:
                sensor_status["speaker_amp"] = "FAILED"
                has_error = True
        else:
            sensor_status["speaker_amp"] = "FAILED"
            has_error = True
    except Exception:
        sensor_status["speaker_amp"] = "FAILED"
        has_error = True

    # Update car_state with sensor status
    car_state["sensor_status"] = sensor_status
    car_state["service_light_active"] = has_error
    
    return sensor_status

def sensor_monitor():
    """
    Background thread that periodically checks sensor health every 5 seconds
    """
    while True:
        try:
            check_sensor_health()
            time.sleep(5)  # Check every 5 seconds
        except Exception as e:
            print(f"❌ Sensor monitor error: {e}")
            time.sleep(5)

def save_gear_before_ebrake():
    """Save current gear before emergency brake sets it to Neutral
    
    Only saves if:
    1. Current gear is not already "N" (no need to save neutral)
    2. No gear is already saved (prevents overwriting during same e-brake activation)
    
    This ensures we save the gear once when e-brake is first activated,
    and preserve that gear throughout the e-brake duration.
    """
    if car_state["gear"] != "N" and car_state["gear_before_ebrake"] is None:
        car_state["gear_before_ebrake"] = car_state["gear"]

def restore_gear_after_ebrake():
    """Restore gear after emergency brake is released"""
    if car_state["gear_before_ebrake"] is not None:
        car_state["gear"] = car_state["gear_before_ebrake"]
        car_state["gear_before_ebrake"] = None

def physics_loop():
    # Note: obstacle_state is now in car_state, not a local variable
    stop_timer = 0           # Timer for 1-second stop (50 cycles = 1 second at 20ms)
    steering_timer = 0       # Timer for steering duration
    last_left_obstacle = False
    last_right_obstacle = False
    target_steer_angle = 0   # Target angle during steering phase
    was_autonomous = False   # Track autonomous → manual transition
    smoothed_steer = 0.0     # Rate-limited steering angle (motor protection)
    
    while True:
        # --- CHECK IR SENSORS (from Pico bridge) ---
        try:
            left_obstacle, right_obstacle = pico_get_ir()
        except:
            left_obstacle = False
            right_obstacle = False
        
        car_state["left_obstacle"] = left_obstacle
        car_state["right_obstacle"] = right_obstacle
        
        # --- CHECK LASER SENSOR (Moving-Average Filtered) ---
        if car_state["sonar_enabled"]:
            sonar_distance = get_smoothed_laser()
        else:
            sonar_distance = 100  # Default safe distance when disabled
        
        car_state["sonar_distance"] = round(sonar_distance, 1)
        
        # Debug: Print only on state change (less spam)
        if left_obstacle and not last_left_obstacle:
            print("⚠️  LEFT OBSTACLE DETECTED!")
        if right_obstacle and not last_right_obstacle:
            print("⚠️  RIGHT OBSTACLE DETECTED!")
        if not left_obstacle and last_left_obstacle:
            print("✅ LEFT CLEAR")
        if not right_obstacle and last_right_obstacle:
            print("✅ RIGHT CLEAR")
        
        # Sonar distance warnings (only print when crossing thresholds)
        # [Sonar logs removed from terminal output]
        
        last_left_obstacle = left_obstacle
        last_right_obstacle = right_obstacle
        
        # --- VISION ACTIVATION FOR MANUAL MODE ---
        # Activate vision DNN when driving forward in manual mode (only if camera + CV enabled)
        if VISION_AVAILABLE and vision_system is not None and not car_state["autonomous_mode"]:
            gear_val = car_state["gear"]
            is_forward_gear = gear_val in ("1", "2", "3", "S")
            is_moving = car_state["gas_pressed"] and car_state["current_pwm"] > 0
            vision_system.active = car_state["user_wants_vision"] and is_forward_gear and is_moving
            # Mirror vision data for manual mode telemetry
            if vision_system.active:
                summary = vision_system.get_detections_summary()
                car_state["vision_active"] = summary["vision_active"]
                car_state["camera_obstacle_distance"] = summary["camera_obstacle_distance"]
                car_state["camera_detections_count"] = summary["count"]
                car_state["camera_in_path_count"] = summary["in_path_count"]
                car_state["camera_closest_object"] = summary["closest_class"]
                car_state["camera_closest_confidence"] = summary["closest_confidence"]
                car_state["vision_fps"] = summary["vision_fps"]
            else:
                car_state["vision_active"] = False
                car_state["camera_obstacle_distance"] = 999.0
                car_state["camera_detections_count"] = 0
                car_state["camera_in_path_count"] = 0
                car_state["camera_closest_object"] = ""
                car_state["camera_closest_confidence"] = 0
                car_state["vision_fps"] = 0.0
        elif not car_state["camera_enabled"]:
            # Reset vision metrics when camera is disabled
            car_state["vision_active"] = False
            car_state["camera_obstacle_distance"] = 999.0
            car_state["camera_detections_count"] = 0
            car_state["camera_in_path_count"] = 0
            car_state["camera_closest_object"] = ""
            car_state["camera_closest_confidence"] = 0
            car_state["vision_fps"] = 0.0

        gear = car_state["gear"]
        gas = car_state["gas_pressed"]
        brake = car_state["brake_pressed"]
        current = car_state["current_pwm"]
        user_angle = car_state["user_steer_angle"]
        obstacle_state = car_state["obstacle_state"]
        
        # --- STEERING SMOOTHING (motor protection) ---
        # Rate-limit the steering angle change to prevent instant
        # differential speed swings that stress motors.
        # user_angle is the raw target from socket; smoothed_steer is
        # the rate-limited value actually sent to the steering mixer.
        steer_delta = user_angle - smoothed_steer
        if abs(steer_delta) > STEER_RATE_LIMIT_PER_TICK:
            steer_delta = STEER_RATE_LIMIT_PER_TICK if steer_delta > 0 else -STEER_RATE_LIMIT_PER_TICK
        smoothed_steer += steer_delta
        # Use the smoothed value for the rest of this tick
        user_angle = int(smoothed_steer)
        
        # --- OBSTACLE AVOIDANCE STATE MACHINE (Manual Mode Only) ---
        # When autonomous mode is active, drive_autonomous() owns all obstacle logic.
        # physics_loop only handles sensor reads, smooth ramping, steering mixer, and GPIO.
        # 
        # HOWEVER: Emergency Safety Override - even in autonomous mode, if sensors detect
        # imminent collision, immediately cut power and brake to prevent damage.
        emergency_stop_distance = 8   # Emergency stop if sonar < 8cm (immediate danger)
        # IR and sonar sensors face FORWARD.  When the car is in Reverse
        # gear it is driving AWAY from whatever the sensors see, so the
        # emergency override must not fire — otherwise the car can never
        # back away from a front obstacle.
        is_forward_gear = car_state["gear"] != "R"
        emergency_ir_override = car_state["ir_enabled"] and (left_obstacle or right_obstacle) and is_forward_gear
        emergency_sonar_override = car_state["sonar_enabled"] and sonar_distance < emergency_stop_distance and is_forward_gear
        
        if emergency_ir_override or emergency_sonar_override:
            # EMERGENCY OVERRIDE: Immediate stop regardless of mode
            car_state["current_pwm"] = 0
            car_state["is_braking"] = True
            # NOTE: Do NOT overwrite gas_pressed / brake_pressed here.
            # Those flags represent the user's physical input on the UI.
            # Overwriting them causes a latching bug: when the obstacle
            # clears, brake_pressed stays True and the car remains
            # braked until the user manually releases and re-presses
            # the throttle — the UI thinks throttle is held but the
            # server ignores it.  is_braking alone is enough; the
            # motor-output section already checks it to apply brakes.
            # In autonomous mode, AutoPilot owns its own FSM —
            # the emergency brake GPIO is applied below by the motor-output
            # section; AutoPilot will detect the obstacle on its next tick.
        elif not car_state["autonomous_mode"]:
            # Enhanced: Combine IR sensors with sonar for better obstacle detection
            ir_obstacle_detected = car_state["ir_enabled"] and (left_obstacle or right_obstacle)
            sonar_obstacle_detected = car_state["sonar_enabled"] and sonar_distance < SONAR_STOP_DISTANCE
            sonar_too_close = car_state["sonar_enabled"] and sonar_distance < SONAR_CRAWL_DISTANCE
            any_obstacle = ir_obstacle_detected or sonar_obstacle_detected
            
            if any_obstacle and gas:
                if obstacle_state == "IDLE":
                    # Trigger: Obstacle detected, enter appropriate avoidance state
                    if sonar_too_close:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                    else:
                        obstacle_state = "STEERING"
                        car_state["obstacle_state"] = "STEERING"
                        
                        if sonar_obstacle_detected and not ir_obstacle_detected:
                            if left_obstacle:
                                target_steer_angle = 60
                            elif right_obstacle:
                                target_steer_angle = -60
                            else:
                                target_steer_angle = 60
                        elif left_obstacle and right_obstacle:
                            obstacle_state = "REVERSING"
                            car_state["obstacle_state"] = "REVERSING"
                            target_steer_angle = 0
                            car_state["dual_obstacle_escape"] = True
                            print("🚨 BOTH IR OBSTACLES - REVERSING FIRST")
                        elif left_obstacle:
                            target_steer_angle = 60
                            print("🚨 LEFT IR OBSTACLE - GENTLE RIGHT 60°")
                        else:
                            target_steer_angle = -60
                            print("🚨 RIGHT IR OBSTACLE - GENTLE LEFT 60°")
                
                elif obstacle_state == "REVERSING":
                    car_state["steer_angle"] = 0
                    steering_timer += 1
                    
                    if steering_timer > 25:
                        steering_timer = 0
                        
                        if car_state.get("dual_obstacle_escape", False):
                            car_state["dual_obstacle_escape"] = False
                            if left_obstacle and not right_obstacle:
                                target_steer_angle = 60
                                print("🔄 DUAL ESCAPE: Left cleared - TURNING RIGHT")
                            elif right_obstacle and not left_obstacle:
                                target_steer_angle = -60
                                print("🔄 DUAL ESCAPE: Right cleared - TURNING LEFT")
                            else:
                                import random
                                target_steer_angle = 60 if random.random() > 0.5 else -60
                                direction = "RIGHT" if target_steer_angle > 0 else "LEFT"
                                print(f"🔄 DUAL ESCAPE: Both still present - RANDOM {direction}")
                        else:
                            if left_obstacle:
                                target_steer_angle = 60
                                print("🔄 REVERSE COMPLETE - NOW STEERING RIGHT")
                            elif right_obstacle:
                                target_steer_angle = -60
                                print("🔄 REVERSE COMPLETE - NOW STEERING LEFT")
                            else:
                                target_steer_angle = 60
                                print("🔄 REVERSE COMPLETE - NOW STEERING RIGHT")
                        
                        obstacle_state = "STEERING"
                        car_state["obstacle_state"] = "STEERING"
                
                elif obstacle_state == "STEERING":
                    car_state["steer_angle"] = target_steer_angle
                    steering_timer += 1
                    
                    if not any_obstacle:
                        obstacle_state = "IDLE"
                        car_state["obstacle_state"] = "IDLE"
                        car_state["steer_angle"] = user_angle
                        print("✅ ALL OBSTACLES CLEARED - RESUMING NORMAL CONTROL")
                    elif left_obstacle and right_obstacle and steering_timer > 75:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                        car_state["dual_obstacle_escape"] = True
                        steering_timer = 0
                        print("🔄 BOTH OBSTACLES PERSIST - REVERSING AGAIN")
                    elif steering_timer > 100:
                        obstacle_state = "REVERSING"
                        car_state["obstacle_state"] = "REVERSING"
                        steering_timer = 0
                        print("⏰ STEERING TIMEOUT - TRYING REVERSE")
            else:
                # No obstacles detected
                if obstacle_state != "IDLE":
                    obstacle_state = "IDLE"
                    car_state["obstacle_state"] = "IDLE"
                car_state["steer_angle"] = user_angle
        else:
            # Autonomous mode active — AutoPilot drives motors directly
            # via CarSystem; physics_loop only reads sensors for telemetry.
            obstacle_state = "IDLE"
            car_state["obstacle_state"] = "IDLE"
        
        # Get the current steering angle (either from avoidance or user input)
        angle = car_state["steer_angle"]
        
        # --- NORMAL THROTTLE PHYSICS (when not in obstacle avoidance) ---
        if obstacle_state == "IDLE":
            # Check if autonomous mode is active
            if car_state["autonomous_mode"]:
                # Even in autopilot, the emergency brake must override everything
                if car_state["emergency_brake_active"]:
                    car_state["current_pwm"] = 0
                    car_state["is_braking"] = True
                    # Save current gear before setting to Neutral
                    save_gear_before_ebrake()
                    car_state["gear"] = "N"
                    car_system.stop()  # Immediately cut motors
                else:
                    # AutoPilot controls motors directly via CarSystem.
                    # We still need to update car_state["current_pwm"] for
                    # telemetry, but we do NOT write to GPIO here.
                    car_state["current_pwm"] = car_system._current_speed
                    car_state["is_braking"] = False
                # Skip the rest of throttle physics — fall through to
                # the motor-output section which will also be skipped.
            else:
                # Manual mode - existing logic
                # Check if emergency brake is active - if so, apply brakes and set to Neutral
                if car_state["emergency_brake_active"]:
                    current = 0
                    car_state["current_pwm"] = 0
                    car_state["is_braking"] = True  # Apply brake signals
                    # Save current gear before setting to Neutral
                    save_gear_before_ebrake()
                    car_state["gear"] = "N"  # Set gear to Neutral
                # BRAKE PEDAL PRESSED - Real car braking behavior
                elif brake:
                    # Ramp speed down smoothly instead of instant zero.
                    # Magnetic brake engages only once speed reaches near-zero.
                    if current > 1.0:
                        current -= BRAKE_DECEL_RATE
                        current = max(0, current)
                        car_state["current_pwm"] = current
                        car_state["is_braking"] = False  # Still decelerating, no magnetic lock yet
                    else:
                        current = 0
                        car_state["current_pwm"] = 0
                        car_state["is_braking"] = True  # Stopped — apply magnetic braking to hold
                else:
                    # Brakes released - normal gas/coast logic
                    car_state["is_braking"] = False
                    # Check if speed limit is being used instead of gear
                    if car_state["speed_limit_enabled"]:
                        # Use speed limit (5-100%)
                        target = car_state["speed_limit"] if (gas and gear != "N") else 0
                    else:
                        # Use gear-based speeds (voltage-aware dynamic ranges)
                        ranges = car_system.power_limiter.gear_duty_ranges()
                        min_s, max_s = ranges.get(gear, (0, 40))
                        target = max_s if (gas and gear != "N") else 0
                        if gas and current < min_s: current = min_s
                    
                    # --- SONAR-BASED SPEED REDUCTION ---
                    if car_state["sonar_enabled"] and target > 0:
                        if sonar_distance < SONAR_STOP_DISTANCE:
                            # Emergency stop - too close!
                            target = 0
                        elif sonar_distance < SONAR_CRAWL_DISTANCE:
                            # Very close - crawl speed (5% max)
                            target = min(target, 5)
                        elif sonar_distance < SONAR_SLOW_DISTANCE:
                            # Close - very slow movement (15% max)
                            target = min(target, 15)
                        elif sonar_distance < SONAR_CAUTION_DISTANCE:
                            # Getting close - moderate speed (40% max)
                            target = min(target, 40)
                    
                    # Hard dynamic voltage cap on any target
                    target = min(target, car_system.power_limiter.max_safe_duty)
                    
                    # Smooth Ramping
                    if current < target: current += ACCEL_RATE
                    elif current > target: current -= COAST_RATE
                    current = max(0, current)  # Ensure speed never goes negative
                    
                    car_state["current_pwm"] = current
        
        # --- OBSTACLE AVOIDANCE PHYSICS ---
        elif obstacle_state == "REVERSING":
            # Brake-before-reverse: ensure the car is stopped before reversing.
            # The physics loop's braking logic will have ramped current
            # toward 0.  Only engage reverse once speed is low enough.
            if current > MIN_SPEED_FOR_GEAR_CHANGE:
                # Still decelerating — apply brakes, don't reverse yet
                current -= BRAKE_DECEL_RATE
                current = max(0, current)
                car_state["current_pwm"] = current
                car_state["is_braking"] = current < 1.0
            else:
                # Stopped — safe to reverse with configurable power
                current = REVERSE_POWER
                car_state["current_pwm"] = current
                car_state["gear"] = "R"  # Force reverse gear
                car_state["is_braking"] = False
        
        elif obstacle_state == "STEERING":
            # Use sonar-reduced speeds during steering
            if car_state["speed_limit_enabled"]:
                target = car_state["speed_limit"] if gas else 0
            else:
                ranges = car_system.power_limiter.gear_duty_ranges()
                min_s, max_s = ranges.get(gear, (0, 40))
                target = max_s if gas else 0
                if gas and current < min_s: current = min_s
            
            # Apply sonar speed reduction even during steering
            if car_state["sonar_enabled"] and target > 0:
                if sonar_distance < SONAR_STOP_DISTANCE:
                    target = 0
                elif sonar_distance < SONAR_CRAWL_DISTANCE:
                    target = min(target, 5)
                elif sonar_distance < SONAR_SLOW_DISTANCE:
                    target = min(target, 15)
                elif sonar_distance < SONAR_CAUTION_DISTANCE:
                    target = min(target, 40)
            
            # Hard dynamic voltage cap on any target
            target = min(target, car_system.power_limiter.max_safe_duty)
            
            # Smooth Ramping
            if current < target: current += ACCEL_RATE
            elif current > target: current -= COAST_RATE
            current = max(0, current)
            
            car_state["current_pwm"] = current

        # Re-read gear — obstacle avoidance (REVERSING) may have
        # changed car_state["gear"] to "R" after we read it at the
        # top of this iteration.  Using the stale value would apply
        # the wrong direction for one full 20 ms frame.
        gear = car_state["gear"]

        # --- AUDIO STATE ---
        # Drive sound effects from real runtime state:
        # - acceleration while throttle is pressed in forward gears
        # - beeper while moving in reverse
        # - horn warning during autopilot when obstacle is closer than 10cm
        engine_running = car_state.get("engine_running", False)
        accelerating_audio = (
            engine_running
            and gas
            and not brake
            and gear != "R"
            and obstacle_state != "REVERSING"
            and not car_state["is_braking"]
        )
        reversing_audio = (
            engine_running
            and gear == "R"
            and car_state["current_pwm"] > 1.0
            and not car_state["is_braking"]
        )
        # Horn warning: Only during autopilot when obstacle is very close (< 10cm)
        horn_audio = (
            car_state.get("autonomous_mode", False)
            and car_state["sonar_enabled"]
            and sonar_distance < 10
        )
        car_audio.update_runtime_state(
            accelerating=accelerating_audio,
            reversing=reversing_audio,
            horn_warning=horn_audio,
        )

        # --- 3. STEERING MIXER (The Magic) ---
        # In autonomous mode, CarSystem handles GPIO directly — skip mixer.
        if car_state["autonomous_mode"]:
            was_autonomous = True
            time.sleep(0.02)
            continue

        # ── Autonomous → Manual transition guard ──────────────────────
        # When we just left autonomous mode, current_pwm may still hold
        # the autopilot's last speed (mirrored above).  Force it to zero
        # so physics_loop doesn't coast the car at the old speed.
        if was_autonomous:
            current = 0
            car_state["current_pwm"] = 0
            car_state["is_braking"] = False
            was_autonomous = False

        # We calculate separate speeds for Left and Right motors
        left_motor_speed = current
        right_motor_speed = current

        # Calculate "Turn Factor" (0.0 to 1.0) based on angle
        # 90 degrees = 1.0 (Full turn), 0 degrees = 0.0 (Straight)
        turn_factor = abs(angle) / 90.0
        
        # ARC TURN LOGIC:
        if angle < -5: # TURNING LEFT
            # Left motor gets slower. At max turn, it drops to 10% speed
            left_motor_speed = current * (1.0 - (turn_factor * 0.9))
            right_motor_speed = current # Outer wheel stays fast
            
        elif angle > 5: # TURNING RIGHT
            # Right motor gets slower
            right_motor_speed = current * (1.0 - (turn_factor * 0.9))
            left_motor_speed = current # Outer wheel stays fast

        # --- 4. APPLY TO MOTORS ---
        # Use car_system methods (which go through motor.py's GPIO
        # wrapper) instead of raw GPIO calls.  RPi.GPIO 0.7.2 uses
        # lgpio chip handles internally; the physics_loop's direct
        # RPi.GPIO calls see a stale handle after CarSystem.__init__
        # re-opens the chip — causing silent 'unknown handle' errors.
        try:
            if car_state["is_braking"]:
                car_system.brake()
            else:
                is_forward = (gear != "R")
                if obstacle_state == "REVERSING":
                    print(f"🔧 REVERSE MOTORS: L={int(left_motor_speed)}% R={int(right_motor_speed)}%")
                car_system._set_raw_motors(
                    left_motor_speed, right_motor_speed,
                    is_forward, is_forward
                )
        except Exception as e:
            print(f"❌ [Physics] Motor GPIO error: {e}")

        time.sleep(0.02)  # 20ms loop = 50Hz (2.5x faster reaction than 50ms)

# ==========================================
# 🤖 SMART DRIVER AUTONOMOUS DRIVING
# ==========================================

def drive_autonomous():
    """
    Thin wrapper that ticks the AutoPilot FSM at 20 Hz.
    All navigation logic lives in autopilot.py.

    IMPORTANT – only the SocketIO / HTTP handlers call autopilot.start().
    This thread only *ticks* the FSM and handles emergency-brake pausing.
    Previously an unconditional ``if not is_active: start()`` here caused
    a race on disable: the handler called stop() but this thread restarted
    the autopilot before autonomous_mode was set to False, producing a
    ~0.5 s forward surge.
    """
    emergency_stopped = False        # track if WE paused the autopilot

    while True:
        if not car_state["autonomous_mode"]:
            if autopilot.is_active:
                autopilot.stop()
            emergency_stopped = False
            time.sleep(0.05)
            continue

        # Emergency brake overrides autopilot — pause the FSM and stop motors
        if car_state["emergency_brake_active"]:
            if autopilot.is_active:
                autopilot.stop()  # Cuts motors immediately
                emergency_stopped = True
            time.sleep(0.05)
            continue

        # Resume ONLY after emergency brake release (not after a mode-disable stop)
        if not autopilot.is_active:
            if emergency_stopped:
                autopilot.start()
                emergency_stopped = False
            else:
                # Autopilot was stopped by the disable handler; don't restart.
                time.sleep(0.05)
                continue

        autopilot.update()

        # Mirror state into car_state for telemetry & UI
        car_state["autonomous_state"] = autopilot.state.value
        car_state["last_obstacle_side"] = autopilot.turn_direction or "none"

        # Mirror MPU6050 / PID telemetry
        car_state["gyro_z"] = round(autopilot.gyro_z, 2)
        car_state["pid_correction"] = round(autopilot.pid_correction, 1)
        car_state["gyro_available"] = autopilot.gyro_available
        car_state["gyro_calibrated"] = autopilot.gyro_calibrated

        # Mirror yaw-tracking telemetry
        car_state["target_yaw"] = round(autopilot.target_yaw, 1)
        car_state["current_heading"] = round(autopilot.current_heading, 1)
        car_state["slalom_sign"] = autopilot.slalom_sign

        # Emit fresh laser scan data to UI clients
        if autopilot.scan_data_fresh:
            scan_payload = [
                {"angle": a, "distance": round(d, 1)}
                for a, d in autopilot.last_scan_data
            ]
            car_state["laser_scan_data"] = scan_payload
            try:
                socketio.emit('laser_scan', {'scan': scan_payload})
            except Exception:
                pass

        time.sleep(0.05)  # 20 Hz

# Start the Engine (Thread)
engine_thread = threading.Thread(target=physics_loop, daemon=True)
engine_thread.start()

# Start the Smart Driver Autonomous Thread
autonomous_thread = threading.Thread(target=drive_autonomous, daemon=True)
autonomous_thread.start()
print("🤖 Smart Driver autonomous system initialized")

# Start the Sensor Monitor Thread
sensor_monitor_thread = threading.Thread(target=sensor_monitor, daemon=True)
sensor_monitor_thread.start()
print("🚨 Sensor health monitor initialized")

# Start AirPlay metadata reader thread (integrated from metadata.py)
airplay_metadata_thread = threading.Thread(target=_airplay_metadata_reader, daemon=True)
airplay_metadata_thread.start()
print(f"🎵 AirPlay metadata reader initialized (pipe: {AIRPLAY_METADATA_PIPE_PATH})")

# ==========================================
# 🌐 WEB SERVER (Flask + SocketIO)
# ==========================================
app = Flask(__name__, 
            static_folder=os.path.join(DIST_DIR, 'assets'), 
            static_url_path='/assets',
            template_folder=DIST_DIR)
CORS(app)
socketio = SocketIO(app, 
                   cors_allowed_origins="*",
                   async_mode='threading',
                   ping_timeout=60,
                   ping_interval=25,
                   max_http_buffer_size=1e8)

DEFAULT_HOTSPOT_IP = os.getenv("RC_HOTSPOT_IP", "192.168.4.1")
DEFAULT_WIFI_REDIRECT_HOST = os.getenv("RC_WIFI_REDIRECT_HOST", "raspberrypi.local")
network_manager = PiCarNetworkManager()

# Helper function to check hotspot status
def get_hotspot_status():
    """Check if hotspot is active by checking wlan0 IP address"""
    try:
        # Get the IP address of wlan0
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'], 
                              capture_output=True, text=True, timeout=2)
        # Check if wlan0 has an IP in the 192.168.4.x range (standard hotspot range)
        # or 10.42.0.x range (alternative hotspot range)
        for line in result.stdout.split('\n'):
            if 'inet ' in line and 'scope' in line:
                parts = line.strip().split()
                if len(parts) >= 2:
                    ip_with_mask = parts[1]
                    ip = ip_with_mask.split('/')[0]
                    # Check if it's in hotspot range (192.168.4.x or 10.42.0.x)
                    if ip.startswith('192.168.4.') or ip.startswith('10.42.0.'):
                        return True
                    # If it has any IP, it's not in hotspot mode (it's connected to WiFi)
                    return False
        # No IP found on wlan0, so not in hotspot mode
        return False
    except Exception as e:
        print(f"⚠️ Hotspot check failed: {e}")
        return False

# Helper function to get local network IP
def get_wlan0_ip():
    """Get the IP address of wlan0 interface (used for hotspot)"""
    try:
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'], 
                              capture_output=True, text=True, timeout=2)
        # Parse output like: inet 192.168.4.1/24 brd 192.168.4.255 scope global wlan0
        for line in result.stdout.split('\n'):
            if 'inet ' in line and 'scope' in line:
                parts = line.strip().split()
                if len(parts) >= 2:
                    ip_with_mask = parts[1]
                    ip = ip_with_mask.split('/')[0]
                    return ip
        return None
    except Exception:
        return None

def get_local_ip():
    """Get the Raspberry Pi's local network IP address"""
    # First, check if hotspot is active and get its IP
    if get_hotspot_status():
        print("📡 [Network] Hotspot mode detected")
        wlan0_ip = get_wlan0_ip()
        if wlan0_ip:
            print(f"📡 [Network] Using hotspot IP from wlan0: {wlan0_ip}")
            return wlan0_ip
        # Hotspot active but couldn't get wlan0 IP, fallback below
        print("⚠️  [Network] Hotspot active but couldn't determine wlan0 IP, falling back...")
    
    try:
        # Create a socket to determine which IP is used to reach external networks
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Google DNS, no internet connection needed
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        # Fallback: try to get hostname IP
        try:
            hostname_ip = socket.gethostbyname(socket.gethostname())
            print(f"📡 [Network] Using hostname IP: {hostname_ip}")
            return hostname_ip
        except Exception:
            print("⚠️  [Network] Could not determine IP, using loopback")
            return "127.0.0.1"

@app.route("/")
def index():
    """Serve the main index.html with no-cache headers to prevent stale HTML."""
    response = render_template('index.html')
    # Add headers to prevent caching of index.html (so updates are always fetched)
    # Assets (JS/CSS) have content hashes and are cached by the browser indefinitely
    resp = make_response(response)
    resp.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate, public, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    resp.headers['Expires'] = '0'
    return resp

# API endpoint for getting server IP (used by mobile/remote clients)
@app.route("/api/server-ip")
def api_server_ip():
    """Return the server's network IP address for remote connections"""
    ip = get_local_ip()
    hotspot_active = get_hotspot_status()
    return jsonify({
        "ip": ip, 
        "port": 5000,
        "hotspot_active": hotspot_active,
        "transport": "hotspot" if hotspot_active else "wifi"
    })

@app.route("/api/now-playing")
def api_now_playing():
    """Return integrated AirPlay now-playing metadata."""
    return jsonify(_build_now_playing_response())

# Catch-all for React files (vite.svg, etc.)
@app.route('/<path:filename>')
def serve_root_files(filename):
    return send_from_directory(DIST_DIR, filename)

# --- CAMERA VIDEO FEED ---
@app.route('/video_feed')
def video_feed():
    """MJPEG streaming route for live camera feed."""
    if not CAMERA_AVAILABLE:
        return "Camera not available", 503
    return Response(generate_camera_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Print server info on startup
print(f"🌐 Server IP Address: {get_local_ip()}")
print(f"📱 Mobile devices can connect via: http://{get_local_ip()}:5000")
if CAMERA_AVAILABLE:
    print(f"📷 Camera feed available at: http://{get_local_ip()}:5000/video_feed")

# --- CONTROLS ---

@app.route("/gear/<gear>")
def set_gear(gear):
    """Sets the Gear (R, N, 1, 2, 3, S)"""
    if gear in ["R", "N", "1", "2", "3", "S"]:
        car_state["gear"] = gear
        return f"GEAR_{gear}"
    return "INVALID"

@app.route("/steer/<angle>")
def steer(angle):
    try:
        angle = int(angle)
        # Store user's steering input (preserved when avoiding obstacles)
        car_state["user_steer_angle"] = angle
        # If angle is big enough, mark as turning
        if abs(angle) > 10:
            car_state["turning"] = "left" if angle < 0 else "right"
        else:
            car_state["turning"] = "straight"
        
        # Log only when steering is commanded
        turn_factor = abs(angle) / 90.0
        print(f"🎯 Steering: {angle:+d}° | Turn Factor: {turn_factor:.2f} | Direction: {car_state['turning'].upper()}")
        
        return "OK"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit/<speed>")
def set_speed_limit(speed):
    """Sets the manual speed limit (5-100%)"""
    try:
        speed = int(speed)
        if 5 <= speed <= 100:
            car_state["speed_limit"] = speed
            car_state["speed_limit_enabled"] = True  # Enable speed limit mode
            print(f"⚡ Speed Limit: {speed}% | Mode: SPEED LIMIT (Gear ignored)")
            return f"SPEED_LIMIT_{speed}"
        else:
            return "INVALID (must be 5-100)"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit_enable/<speed>")
def enable_speed_limit(speed):
    """Enables speed limit mode with specified speed"""
    try:
        speed = int(speed)
        if 5 <= speed <= 100:
            car_state["speed_limit"] = speed
            car_state["speed_limit_enabled"] = True
            print(f"✅ Speed Limit ENABLED: {speed}% | Mode: SPEED LIMIT (Gear ignored)")
            return f"SPEED_LIMIT_ENABLED_{speed}"
        else:
            return "INVALID (must be 5-100)"
    except ValueError:
        return "INVALID"

@app.route("/speed_limit_disable")
def disable_speed_limit():
    """Disables speed limit mode, reverts to gear-based speeds"""
    car_state["speed_limit_enabled"] = False
    car_state["speed_limit"] = 100  # Reset to max
    print(f"❌ Speed Limit DISABLED | Mode: GEAR (Normal gear-based speeds)")
    return "SPEED_LIMIT_DISABLED"

@app.route("/<action>")
def handle_action(action):
    """Handles Press/Release events from the UI"""
    
    # THROTTLE
    if action == 'forward':
        car_state["gas_pressed"] = True
        car_state["brake_pressed"] = False
        car_state["direction"] = "forward"
        car_state["turning"] = "straight"
    
    # COAST (Gas Released)
    elif action == 'stop':
        car_state["gas_pressed"] = False
        car_state["brake_pressed"] = False
        # Note: We keep direction as-is so it rolls naturally
    
    # BRAKE PEDAL
    elif action == 'brake' or action == 'backward':
        car_state["gas_pressed"] = False
        car_state["brake_pressed"] = True
    
    # STEERING
    elif action == 'left':
        car_state["gas_pressed"] = True
        car_state["turning"] = "left"
    elif action == 'right':
        car_state["gas_pressed"] = True
        car_state["turning"] = "right"

    return "OK"

@app.route("/emergency_stop")
def emergency_stop():
    """Toggle emergency brake - when ON, apply brakes and set gear to Neutral"""
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    
    # Always cut power immediately
    car_state["current_pwm"] = 0
    
    # Reset obstacle avoidance state to prevent stuck state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Note: is_braking flag is managed by the physics loop based on emergency_brake_active
    # Don't touch brake_pressed - it should only reflect actual brake pedal state
    # This prevents state desync between UI and server
    
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"🚨 EMERGENCY BRAKE: {state}")
    return "EMERGENCY_BRAKE"

@app.route("/engine/start")
def http_engine_start():
    """HTTP engine start endpoint for ROS / API clients."""
    car_state["engine_running"] = True
    car_state["brake_pressed"] = False
    car_state["is_braking"] = False
    car_audio.set_engine_running(True)
    print("⚙️ [HTTP Control] 🔊 ENGINE: STARTED")
    return "ENGINE_STARTED"

@app.route("/engine/stop")
def http_engine_stop():
    """HTTP engine stop endpoint for ROS / API clients."""
    car_state["engine_running"] = False
    car_state["auto_accel_enabled"] = False
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = True
    car_state["is_braking"] = True
    car_state["current_pwm"] = 0
    car_state["obstacle_state"] = "IDLE"
    try:
        car_system.brake()
    except Exception:
        pass
    car_audio.set_engine_running(False)
    print("⚙️ [HTTP Control] 🔊 ENGINE: STOPPED")
    return "ENGINE_STOPPED"

@app.route("/emergency_brake/on")
def http_emergency_brake_on():
    """Explicitly enable emergency brake (idempotent)."""
    car_state["emergency_brake_active"] = True
    car_state["current_pwm"] = 0
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    save_gear_before_ebrake()
    car_state["gear"] = "N"
    print("⚙️ [HTTP Control] 🚨 EMERGENCY BRAKE: ON")
    return "EMERGENCY_BRAKE_ON"

@app.route("/emergency_brake/off")
def http_emergency_brake_off():
    """Explicitly disable emergency brake (idempotent)."""
    car_state["emergency_brake_active"] = False
    restore_gear_after_ebrake()
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    print("⚙️ [HTTP Control] 🚨 EMERGENCY BRAKE: OFF")
    return "EMERGENCY_BRAKE_OFF"

@app.route("/autonomous_enable")
def enable_autonomous():
    """Enable Smart Driver autonomous mode"""
    car_state["autonomous_mode"] = True
    car_state["autonomous_state"] = State.CRUISING.value
    car_state["last_obstacle_side"] = "none"
    # Force IR sensors enabled in autonomous mode for safety
    car_state["ir_enabled"] = True
    car_state["sonar_enabled"] = True
    autopilot.start()
    print(f"🤖 SMART DRIVER: ENABLED - Laser-scanner autonomous active")
    return "AUTONOMOUS_ENABLED"

@app.route("/autonomous_disable")
def disable_autonomous():
    """Disable Smart Driver autonomous mode and return to manual control"""
    # ── FIX: set mode flag FIRST so drive_autonomous cannot restart
    car_state["autonomous_mode"] = False
    # Now stop hardware
    autopilot.stop()
    car_system.stop()
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    car_state["gear"] = "N"
    car_state["steer_angle"] = 0
    car_state["user_steer_angle"] = 0
    print(f"🤖 SMART DRIVER: DISABLED - Returned to manual control")
    return "AUTONOMOUS_DISABLED"

@app.route("/autonomous_toggle")
def toggle_autonomous():
    """Toggle Smart Driver autonomous mode on/off"""
    if car_state["autonomous_mode"]:
        return disable_autonomous()
    else:
        return enable_autonomous()

# --- SYSTEM MONITORING FUNCTIONS ---

def get_wifi_status():
    """Get WiFi connection status and SSID"""
    try:
        result = subprocess.run(['iwconfig', 'wlan0'], capture_output=True, text=True, timeout=2)
        output = result.stdout
        
        # Check if connected
        if 'Not-Associated' in output or 'off/any' in output:
            return False, 'Disconnected'
        
        # Extract SSID
        ssid_match = re.search(r'ESSID:"([^"]*)"', output)
        if ssid_match:
            ssid = ssid_match.group(1)
            return True, ssid if ssid else 'Connected'
        
        return True, 'Connected'
    except Exception as e:
        print(f"⚠️ WiFi check failed: {e}")
        return False, 'Unknown'

def get_cpu_temperature():
    """Get CPU temperature in Celsius"""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = int(f.read().strip()) / 1000.0
            return round(temp, 1)
    except Exception as e:
        print(f"⚠️ Temperature read failed: {e}")
        return 45.0

def get_cpu_clock():
    """Get CPU clock speed in MHz"""
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 'r') as f:
            freq_khz = int(f.read().strip())
            freq_mhz = freq_khz / 1000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Fallback: Try alternative sysfs path
    try:
        with open('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq', 'r') as f:
            freq_khz = int(f.read().strip())
            freq_mhz = freq_khz / 1000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Final fallback: return a reasonable default
    return 1200

def get_gpu_clock():
    """Get GPU clock speed in MHz"""
    try:
        result = subprocess.run(['vcgencmd', 'measure_clock', 'gpu'], 
                              capture_output=True, text=True, timeout=2)
        # Output format: frequency(45)=XXXXX
        match = re.search(r'=(\d+)', result.stdout)
        if match:
            freq_hz = int(match.group(1))
            freq_mhz = freq_hz / 1_000_000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception as e:
        pass
    
    # Fallback: Try reading from sysfs
    try:
        with open('/sys/kernel/debug/clk/vec/clk_rate', 'r') as f:
            freq_hz = int(f.read().strip())
            freq_mhz = freq_hz / 1_000_000
            if freq_mhz > 0:
                return int(freq_mhz)
    except Exception:
        pass
    
    # Final fallback: return a reasonable default for Raspberry Pi
    return 400

# --- TELEMETRY ---

# ==========================================
# 🌐 WEBSOCKET EVENT HANDLERS
# ==========================================

@socketio.on('connect')
def on_connect():
    """Handle client connection"""
    global has_had_client
    print(f"\n🔗 [Socket] ✅ CLIENT CONNECTED")
    print(f"   Client ID: {request.sid}")
    print(f"   Remote Address: {request.remote_addr}")
    # Initialize ping tracking for this client
    has_had_client = True
    client_last_pong[request.sid] = time.time()
    car_state["heartbeat_active"] = True
    # Ensure emergency brakes are OFF when client connects (client is now responsible for safety)
    car_state["emergency_brake_active"] = False
    restore_gear_after_ebrake()  # Restore saved gear if e-brake was previously active
    print(f"   Emergency brakes reset to OFF")
    emit('connection_response', {'data': 'Connected to RC Car'})
    # Send current tuning state so UI stays in sync after refresh
    emit('tuning_sync', {
        'tuning': _active_tuning,
        'defaults': _AUTOPILOT_DEFAULT_TUNING,
    })
    # Send camera specs for dynamic resolution dropdown
    emit('camera_specs_sync', camera_specs)
    # Send narration config
    # Always include cached models from the config file to avoid "revalidate" prompts on browser refresh
    api_key = _narration_config.get('api_key', '')
    cached_models = _narration_config.get('models', [])
    _masked_narration = _build_narration_config_sync_payload()
    _masked_narration['models'] = cached_models  # Use cached models from config file on initial sync
    emit('narration_config_sync', _masked_narration)
    print(f"✅ [Narration] Sent config sync with {len(cached_models)} cached models (api_key_set={bool(api_key)})")
    # Send persisted sensor toggle states so client syncs immediately
    emit('sensor_config_sync', {
        'ir_enabled': car_state['ir_enabled'],
        'sonar_enabled': car_state['sonar_enabled'],
        'mpu6050_enabled': car_state['mpu6050_enabled'],
        'rear_sonar_enabled': car_state['rear_sonar_enabled'],
    })
    # Send startup check config
    _masked_startup = _build_startup_config_sync_payload()
    emit('startup_config_sync', _masked_startup)
    # Refresh models list in background if key is set and we have cached models
    _client_sid = request.sid
    if api_key:
        def _refresh_models_async(client_id):
            try:
                fresh_models = list_multimodal_models(api_key)
                if fresh_models:
                    # Only send update if models actually changed
                    if fresh_models != cached_models:
                        _narration_config['models'] = fresh_models
                        _save_narration_config(_narration_config)
                        _updated = _build_narration_config_sync_payload()
                        _updated['models'] = fresh_models
                        socketio.emit('narration_config_sync', _updated, to=client_id)
                        print(f"✅ [Narration] Refreshed models from API: {len(fresh_models)} models available")
                    else:
                        print(f"✅ [Narration] Cached models are current ({len(fresh_models)} models)")
            except Exception as e:
                print(f"⚠️  [Narration] Background model refresh failed: {e}")
        threading.Thread(target=_refresh_models_async, args=(_client_sid,), daemon=True).start()

@socketio.on('disconnect')
def on_disconnect():
    """Handle client disconnection"""
    global has_had_client
    print(f"\n🔓 [Socket] ❌ CLIENT DISCONNECTED")
    print(f"   Client ID: {request.sid}")
    # Clean up ping tracking for this client
    if request.sid in client_last_pong:
        del client_last_pong[request.sid]
    # If all clients disconnected AND we've had clients before, activate emergency brakes
    if not client_last_pong and has_had_client:
        car_state["emergency_brake_active"] = True
        car_state["heartbeat_active"] = False
        car_state["current_pwm"] = 0
        car_state["engine_running"] = False
        car_audio.set_engine_running(False)
        print(f"🚨 [Heartbeat] ❌ ALL CLIENTS DISCONNECTED - EMERGENCY BRAKES ACTIVATED!")


@socketio.on('heartbeat_pong')
def on_heartbeat_pong(data):
    """Handle heartbeat_pong response from client (heartbeat check)"""
    client_id = request.sid
    client_last_pong[client_id] = time.time()
    # Signal that heartbeat is active
    car_state["heartbeat_active"] = True
    # Debug: Only print occasionally to avoid spam
    # print(f"💗 [Heartbeat] ✅ Heartbeat pong received from client {client_id}")


@socketio.on('engine_start')
def on_engine_start(data=None):
    """Handle UI engine start button."""
    was_running = bool(car_state.get("engine_running", False))
    car_state["engine_running"] = True
    car_state["brake_pressed"] = False
    car_state["is_braking"] = False
    car_audio.set_engine_running(True)
    print(f"\n⚙️ [UI Control] 🔊 ENGINE: STARTED")

    # Auto-run startup AI check when engine transitions OFF -> ON.
    if not was_running:
        started, reason = _trigger_startup_check_async(
            client_id=request.sid,
            source="engine_start_auto",
            require_enabled=True,
        )
        if started:
            print("⬛ [Startup] Auto diagnostic check triggered by engine start")
        elif reason == "disabled":
            print("⬛ [Startup] Auto diagnostic check skipped (disabled in config)")
        elif reason == "already_running":
            print("⬛ [Startup] Auto diagnostic check skipped (already running)")

    emit('engine_response', {'status': 'ok', 'running': True})


@socketio.on('engine_stop')
def on_engine_stop(data=None):
    """Handle UI engine stop button."""
    car_state["engine_running"] = False
    car_state["auto_accel_enabled"] = False
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = True
    car_state["is_braking"] = True
    car_state["current_pwm"] = 0
    car_state["obstacle_state"] = "IDLE"
    try:
        car_system.brake()
    except Exception:
        pass
    car_audio.set_engine_running(False)
    print(f"\n⚙️ [UI Control] 🔊 ENGINE: STOPPED")
    emit('engine_response', {'status': 'ok', 'running': False})


@socketio.on('horn')
def on_horn(data=None):
    """Handle horn button press - stateless, just plays horn sound."""
    print(f"\n🔔 [UI Control] 📯 HORN: HONK!")
    car_audio.play_horn()
    emit('horn_response', {'status': 'ok'})


@socketio.on('throttle')
def on_throttle(data):
    """Handle throttle control"""
    value = data.get('value', False)
    if value:
        car_state["gas_pressed"] = True
        car_state["brake_pressed"] = False
        car_state["direction"] = "forward"
        print(f"\n⚙️ [UI Control] 🚀 THROTTLE PRESSED (Forward)")
    else:
        car_state["gas_pressed"] = False
        print(f"\n⚙️ [UI Control] ⏸️ THROTTLE RELEASED")
    emit('throttle_response', {'status': 'ok', 'value': value})

@socketio.on('brake')
def on_brake(data):
    """Handle brake control"""
    value = data.get('value', False)
    car_state["brake_pressed"] = value
    if value:
        car_state["gas_pressed"] = False
        print(f"\n⚙️ [UI Control] 🛑 BRAKE ENGAGED")
    else:
        # When brake is released, restore gas if auto-accel is still enabled
        if car_state["auto_accel_enabled"]:
            car_state["gas_pressed"] = True
        print(f"\n⚙️ [UI Control] 🩹 BRAKE RELEASED")
    emit('brake_response', {'status': 'ok', 'value': value})

@socketio.on('steering')
def on_steering(data):
    """Handle steering control (angle -90 to +90)"""
    try:
        angle = int(data.get('angle', 0))
        angle = max(-90, min(90, angle))  # Clamp to -90 to 90
        car_state["user_steer_angle"] = angle
        if abs(angle) > 10:
            car_state["turning"] = "left" if angle < 0 else "right"
            direction = "⬅️ LEFT" if angle < 0 else "➡️ RIGHT"
            print(f"\n⚙️ [UI Control] 🎡 STEERING: {direction} ({abs(angle)}°)")
        else:
            car_state["turning"] = "straight"
            print(f"\n⚙️ [UI Control] 🎡 STEERING: ⬆️ CENTER (0°)")
        emit('steering_response', {'status': 'ok', 'angle': angle})
    except (ValueError, TypeError) as e:
        print(f"\n⚠️ [UI Control] ❌ Invalid steering angle: {e}")
        emit('steering_response', {'status': 'error', 'message': 'Invalid angle'})

@socketio.on('gear_change')
def on_gear_change(data):
    """Handle gear selection with forward↔reverse safety guard.

    If the car is still moving (current_pwm > MIN_SPEED_FOR_GEAR_CHANGE)
    and the driver requests a forward↔reverse transition, auto-brake to
    a stop first, then apply the new gear.  This prevents shoot-through
    current in the H-bridge from an instant direction reversal.
    """
    gear = data.get('gear', 'N').upper()
    if gear in ["R", "N", "1", "2", "3", "S"]:
        current_gear = car_state["gear"]
        current_speed = car_state["current_pwm"]
        forward_gears = {"1", "2", "3", "S"}

        # Detect forward↔reverse transition while moving
        switching_direction = (
            (current_gear in forward_gears and gear == "R") or
            (current_gear == "R" and gear in forward_gears)
        )

        if switching_direction and current_speed > MIN_SPEED_FOR_GEAR_CHANGE:
            # Auto-brake first, then apply gear once stopped
            car_state["brake_pressed"] = True
            car_state["gas_pressed"] = False
            car_state["is_braking"] = True
            car_state["current_pwm"] = 0
            car_system.brake()
            # Small delay to let the motor settle
            import time as _time
            _time.sleep(0.15)
            car_state["is_braking"] = False
            car_state["brake_pressed"] = False
            print(f"\n⚙️ [Motor Protection] 🛡️ Auto-braked before {current_gear}→{gear} at PWM={current_speed:.0f}%")

        car_state["gear"] = gear
        # If emergency brake is active, update the saved gear state
        if car_state["emergency_brake_active"]:
            if gear != "N":
                # User selected a non-neutral gear while e-brake is active
                # Update saved gear so it restores to this new gear when e-brake is released
                car_state["gear_before_ebrake"] = gear
            else:
                # User explicitly selected neutral while e-brake is active
                # Clear saved gear to respect the user's explicit choice
                # When e-brake is released, gear will remain in neutral
                car_state["gear_before_ebrake"] = None
        gear_names = {'R': '🔙 REVERSE', 'N': '⏸️ NEUTRAL', '1': '1️⃣ 1st', '2': '2️⃣ 2nd', '3': '3️⃣ 3rd', 'S': '⚡ SPORT'}
        print(f"\n⚙️ [UI Control] 🔧 GEAR: {gear_names.get(gear, gear)}")
        emit('gear_response', {'status': 'ok', 'gear': gear})
    else:
        print(f"\n⚠️ [UI Control] ❌ Invalid gear: {gear}")
        emit('gear_response', {'status': 'error', 'message': 'Invalid gear'})

@socketio.on('emergency_stop')
def on_emergency_stop(data):
    """Handle emergency brake toggle - when ON, apply brakes and set gear to Neutral"""
    # Toggle the emergency brake state
    car_state["emergency_brake_active"] = not car_state["emergency_brake_active"]
    
    # Always cut power immediately
    car_state["current_pwm"] = 0
    
    # Handle gear state when emergency brake is toggled
    if car_state["emergency_brake_active"]:
        # Activating emergency brake - save current gear and set to Neutral
        save_gear_before_ebrake()
        car_state["gear"] = "N"
    else:
        # Deactivating emergency brake - restore previous gear
        restore_gear_after_ebrake()
    
    # If in autopilot mode and e-brake activated, stop motors immediately
    if car_state["emergency_brake_active"] and car_state["autonomous_mode"]:
        car_system.stop()
        if autopilot.is_active:
            autopilot.stop()
    
    # Reset obstacle avoidance state to prevent stuck state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    # Note: is_braking flag is managed by the physics loop based on emergency_brake_active
    # Don't touch brake_pressed - it should only reflect actual brake pedal state
    # This prevents state desync between UI and server
    
    state = '🔴 ON' if car_state["emergency_brake_active"] else '🟢 OFF'
    print(f"\n⚙️ [UI Control] 🚨 EMERGENCY BRAKE: {state}")
    emit('emergency_stop_response', {'status': 'ok', 'emergency_brake_active': car_state["emergency_brake_active"]})

@socketio.on('emergency_stop_release')
def on_emergency_stop_release(data):
    """Handle emergency brake release - explicitly set brake to OFF"""
    car_state["emergency_brake_active"] = False
    
    # Restore previous gear
    restore_gear_after_ebrake()
    
    # Reset obstacle avoidance state
    car_state["obstacle_state"] = "IDLE"
    car_state["steer_angle"] = 0
    
    print(f"\n⚙️ [UI Control] 🚨 EMERGENCY BRAKE: 🟢 RELEASED (explicit release)")
    emit('emergency_stop_response', {'status': 'ok', 'emergency_brake_active': False})

@socketio.on('auto_accel_enable')
def on_auto_accel_enable(data):
    """Handle auto-acceleration enable (server-side auto-throttle)"""
    car_state["auto_accel_enabled"] = True
    car_state["gas_pressed"] = True  # Actually engage the throttle
    print(f"\n⚙️ [UI Control] 🚀 AUTO-ACCEL: ENABLED (auto-throttle mode)")
    emit('auto_accel_response', {'status': 'ok', 'enabled': True})

@socketio.on('auto_accel_disable')
def on_auto_accel_disable(data):
    """Handle auto-acceleration disable"""
    car_state["auto_accel_enabled"] = False
    car_state["gas_pressed"] = False
    print(f"\n⚙️ [UI Control] 🚫 AUTO-ACCEL: DISABLED")
    emit('auto_accel_response', {'status': 'ok', 'enabled': False})

@socketio.on('ir_toggle')
def on_ir_toggle(data):
    """Handle IR sensor toggle"""
    # Block IR toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        print(f"\n⚙️ [UI Control] 📡 IR SENSORS: BLOCKED - Cannot toggle in autonomous mode")
        emit('ir_response', {'status': 'blocked', 'ir_enabled': car_state["ir_enabled"], 'message': 'Cannot toggle IR sensors in autonomous mode'})
        return
    
    car_state["ir_enabled"] = not car_state["ir_enabled"]
    _sensor_config['ir_enabled'] = car_state["ir_enabled"]
    _save_sensor_config(_sensor_config)
    state = '✅ ON' if car_state["ir_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📡 IR SENSORS: {state}")
    emit('ir_response', {'status': 'ok', 'ir_enabled': car_state["ir_enabled"]})

@socketio.on('sonar_toggle')
def on_sonar_toggle(data):
    """Handle sonar sensor toggle"""
    # Block Sonar toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        emit('sonar_response', {'status': 'blocked', 'sonar_enabled': car_state["sonar_enabled"], 'message': 'Cannot toggle Sonar sensor in autonomous mode'})
        return
    
    car_state["sonar_enabled"] = not car_state["sonar_enabled"]
    _sensor_config['sonar_enabled'] = car_state["sonar_enabled"]
    _save_sensor_config(_sensor_config)
    emit('sonar_response', {'status': 'ok', 'sonar_enabled': car_state["sonar_enabled"]})

@socketio.on('rear_sonar_toggle')
def on_rear_sonar_toggle(data):
    """Handle rear sonar sensor toggle"""
    if car_state["autonomous_mode"]:
        emit('rear_sonar_response', {'status': 'blocked', 'rear_sonar_enabled': car_state["rear_sonar_enabled"], 'message': 'Cannot toggle Rear Sonar in autonomous mode'})
        return
    car_state["rear_sonar_enabled"] = not car_state["rear_sonar_enabled"]
    _sensor_config['rear_sonar_enabled'] = car_state["rear_sonar_enabled"]
    _save_sensor_config(_sensor_config)
    state = '✅ ON' if car_state["rear_sonar_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📡 REAR SONAR: {state}")
    emit('rear_sonar_response', {'status': 'ok', 'rear_sonar_enabled': car_state["rear_sonar_enabled"]})

@socketio.on('mpu6050_toggle')
def on_mpu6050_toggle(data):
    """Handle MPU6050 gyroscope toggle"""
    if car_state["autonomous_mode"]:
        emit('mpu6050_response', {'status': 'blocked', 'mpu6050_enabled': car_state["mpu6050_enabled"], 'message': 'Cannot toggle MPU6050 in autonomous mode'})
        return
    car_state["mpu6050_enabled"] = not car_state["mpu6050_enabled"]
    _sensor_config['mpu6050_enabled'] = car_state["mpu6050_enabled"]
    _save_sensor_config(_sensor_config)
    state = '✅ ON' if car_state["mpu6050_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 🧭 MPU6050: {state}")
    emit('mpu6050_response', {'status': 'ok', 'mpu6050_enabled': car_state["mpu6050_enabled"]})

@socketio.on('camera_toggle')
def on_camera_toggle(data):
    """Toggle camera and all vision-related functions on/off."""
    car_state["camera_enabled"] = not car_state["camera_enabled"]
    state = '✅ ON' if car_state["camera_enabled"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📷 CAMERA: {state}")
    
    # When camera is disabled, turn off vision system and reset user CV preference
    if not car_state["camera_enabled"]:
        car_state["user_wants_vision"] = False
        if VISION_AVAILABLE and vision_system is not None:
            vision_system.active = False
            print(f"📷 VISION: Disabled (camera off)")
        # Auto-disable narration when camera is off
        if car_state["narration_enabled"]:
            narration_engine.stop()
            car_state["narration_enabled"] = False
            car_state["narration_speaking"] = False
            _narration_config['enabled'] = False
            _save_narration_config(_narration_config)
            print(f"\U0001f399\ufe0f NARRATION: Disabled (camera off)")
            emit('narration_toggle_response', {'status': 'ok', 'enabled': False})
    
    emit('camera_response', {'status': 'ok', 'camera_enabled': car_state["camera_enabled"]})

@socketio.on('vision_toggle')
def on_vision_toggle(data):
    """Toggle vision/object detection system on/off (only if camera is enabled).
    Sets the user_wants_vision flag which is respected by physics_loop and
    autonomous loop to decide whether to activate the DNN."""
    if not car_state["camera_enabled"]:
        emit('vision_response', {'status': 'error', 'message': 'Camera must be enabled first. Please enable the camera before activating vision detection.'})
        return
    if not VISION_AVAILABLE or vision_system is None:
        emit('vision_response', {'status': 'error', 'message': 'Vision system not available'})
        return
    car_state["user_wants_vision"] = not car_state["user_wants_vision"]
    # If user is disabling CV, immediately deactivate vision system
    if not car_state["user_wants_vision"]:
        vision_system.active = False
    state = '✅ ON' if car_state["user_wants_vision"] else '❌ OFF'
    print(f"\n⚙️ [UI Control] 📷 VISION: {state}")
    emit('vision_response', {'status': 'ok', 'vision_active': car_state["user_wants_vision"]})

@socketio.on('camera_config_update')
def on_camera_config_update(data):
    """Update camera configuration (resolution, quality, framerate).
    Accepts resolution in WxH format (e.g. '1920x1080') or legacy keys ('low', 'medium', 'high').
    JPEG quality changes are applied immediately.
    Resolution/framerate changes trigger a live camera reconfiguration."""
    print(f"\n📷 [Camera Config] Received update request: {data}")
    updated = []
    needs_restart = False
    old_resolution = car_state["camera_resolution"]
    old_framerate = car_state["camera_framerate"]
    
    # Update JPEG quality (applied immediately - no restart needed)
    if 'jpeg_quality' in data:
        quality = int(data['jpeg_quality'])
        if 1 <= quality <= 100:
            car_state["camera_jpeg_quality"] = quality
            updated.append(f"jpeg_quality={quality}")
            print(f"   ✅ JPEG quality set to {quality}")
    
    # Update resolution (accepts WxH format or legacy low/medium/high)
    if 'resolution' in data:
        resolution = data['resolution']
        # Normalize to WxH format (handles legacy 'low'/'medium'/'high' keys)
        normalized = _normalize_resolution(resolution)
        parsed = _parse_resolution(normalized)
        if parsed is not None:
            if normalized != old_resolution:
                needs_restart = True
                print(f"   📷 Resolution change: '{old_resolution}' → '{normalized}' (input: '{resolution}', parsed: {parsed[0]}x{parsed[1]})")
            else:
                print(f"   📷 Resolution unchanged: '{normalized}'")
            car_state["camera_resolution"] = normalized
            updated.append(f"resolution={normalized}")
        else:
            print(f"   ❌ Invalid resolution rejected: '{resolution}' (supported modes: {camera_specs.get('supported_modes', [])})")
    
    # Update framerate
    if 'framerate' in data:
        framerate = int(data['framerate'])
        if 1 <= framerate <= 120:
            if framerate != old_framerate:
                needs_restart = True
            car_state["camera_framerate"] = framerate
            updated.append(f"framerate={framerate}")
            print(f"   ✅ Framerate set to {framerate}")
    
    # Live reconfigure camera if resolution or framerate changed
    restart_ok = True
    if needs_restart and CAMERA_AVAILABLE and picam2 is not None:
        print(f"   🔄 Reconfiguring camera (resolution={car_state['camera_resolution']}, fps={car_state['camera_framerate']})...")
        with _camera_lock:
            restart_ok = _reconfigure_camera(
                car_state["camera_resolution"],
                car_state["camera_framerate"]
            )
            if not restart_ok:
                # Revert to previous settings on failure
                car_state["camera_resolution"] = old_resolution
                car_state["camera_framerate"] = old_framerate
                updated.append("restart_failed (reverted)")
                print(f"   ❌ Camera reconfiguration FAILED, reverted to {old_resolution}")
            else:
                print(f"   ✅ Camera reconfiguration SUCCESS")
    elif needs_restart:
        print(f"   ⚠️ Camera restart needed but camera not available (CAMERA_AVAILABLE={CAMERA_AVAILABLE})")
    
    # Persist camera config to disk
    _save_camera_config({
        'resolution': car_state["camera_resolution"],
        'jpeg_quality': car_state["camera_jpeg_quality"],
        'framerate': car_state["camera_framerate"],
    })
    
    result_status = 'ok' if restart_ok else 'partial'
    print(f"   📷 [Camera Config] Result: status={result_status}, applied={', '.join(updated) if updated else 'none'}, current_resolution={car_state['camera_resolution']}")
    emit('camera_config_response', {
        'status': result_status, 
        'updated': updated,
        'current_config': {
            'resolution': car_state["camera_resolution"],
            'jpeg_quality': car_state["camera_jpeg_quality"],
            'framerate': car_state["camera_framerate"],
        }
    })

@socketio.on('autonomous_enable')
def on_autonomous_enable(data):
    """Handle Smart Driver autonomous mode enable"""
    # Check required sensors are enabled before starting autopilot
    disabled_sensors = []
    if not car_state["ir_enabled"]:
        disabled_sensors.append("Left IR")
        disabled_sensors.append("Right IR")
    if not car_state["mpu6050_enabled"]:
        disabled_sensors.append("MPU6050")
    
    if disabled_sensors:
        print(f"\n⚙️ [UI Control] 🤖 SMART DRIVER: BLOCKED - Required sensors disabled: {', '.join(disabled_sensors)}")
        emit('autonomous_response', {'status': 'blocked', 'autonomous_enabled': False, 'disabled_sensors': disabled_sensors})
        return
    
    car_state["autonomous_mode"] = True
    car_state["autonomous_state"] = State.CRUISING.value
    car_state["last_obstacle_side"] = "none"
    # Force IR sensors enabled in autonomous mode for safety
    car_state["ir_enabled"] = True
    car_state["sonar_enabled"] = True  # keep flag for UI compat (now reads laser)
    autopilot.start()
    print(f"\n⚙️ [UI Control] 🤖 SMART DRIVER: ENABLED - Laser-scanner autonomous active")
    print(f"📡 SAFETY: IR sensors force-enabled in autonomous mode")
    emit('autonomous_response', {'status': 'ok', 'autonomous_enabled': True})

@socketio.on('autonomous_disable')
def on_autonomous_disable(data):
    """Handle Smart Driver autonomous mode disable"""
    # ── FIX: set mode flag FIRST so drive_autonomous cannot restart
    # the autopilot between stop() and the flag change (race condition
    # that caused ~0.5 s forward surge on disable).
    car_state["autonomous_mode"] = False
    # Now stop hardware — drive_autonomous sees False and won't restart
    autopilot.stop()
    car_system.stop()
    # Reset to safe manual state
    car_state["current_pwm"] = 0
    car_state["gas_pressed"] = False
    car_state["brake_pressed"] = False
    car_state["gear"] = "N"
    car_state["steer_angle"] = 0
    car_state["user_steer_angle"] = 0
    print(f"\n⚙️ [UI Control] 🤖 SMART DRIVER: DISABLED - Returned to manual control")
    emit('autonomous_response', {'status': 'ok', 'autonomous_enabled': False})

@socketio.on('autonomous_toggle')
def on_autonomous_toggle(data):
    """Handle Smart Driver autonomous mode toggle"""
    if car_state["autonomous_mode"]:
        on_autonomous_disable(data)
    else:
        on_autonomous_enable(data)

@socketio.on('autopilot_toggle')
def on_autopilot_toggle(data):
    """Alias for autonomous_toggle — frontend emits this event name"""
    on_autonomous_toggle(data)

@socketio.on('tuning_update')
def on_tuning_update(data):
    """Apply tuning constants from the UI to the running AutoPilot instance
    and persist in _active_tuning so new clients pick them up."""
    tuning = data.get('tuning', {})
    applied = []
    for key, value in tuning.items():
        attr = key.upper()
        if hasattr(autopilot, attr):
            try:
                cast_value = type(getattr(AutoPilot, attr))(value)  # cast to original type
                setattr(autopilot, attr, cast_value)
                _active_tuning[attr] = cast_value
                applied.append(attr)
            except Exception as e:
                print(f"⚠️ [Tuning] Failed to set {attr}={value}: {e}")
    print(f"\n⚙️ [Tuning] Applied {len(applied)} constants from UI: {applied}")
    emit('tuning_response', {'status': 'ok', 'applied': applied})

@socketio.on('tuning_request')
def on_tuning_request(data):
    """Send current active tuning and defaults to the requesting client."""
    emit('tuning_sync', {
        'tuning': _active_tuning,
        'defaults': _AUTOPILOT_DEFAULT_TUNING,
    })

@socketio.on('state_request')
def on_state_request(data):
    """Send current car state to client"""
    emit('state_response', car_state)

# ==========================================
# 🎙️ NARRATION SOCKET EVENTS
# ==========================================

@socketio.on('narration_validate_key')
def on_narration_validate_key(data):
    """Validate an AI provider API key and return available models.
    Runs in background thread to avoid blocking the SocketIO thread."""
    provider = data.get('provider', 'gemini')
    api_key = data.get('api_key', '')
    client_id = request.sid  # Get the client ID for this connection
    
    def validate_async():
        """Background validation task."""
        print(f"\n\U0001f399\ufe0f [Narration] Validating {provider} API key (client: {client_id[:8]}...)")
        
        if provider != 'gemini':
            socketio.emit('narration_key_result', {'valid': False, 'error': f'{provider} is not supported yet', 'models': []}, to=client_id)
            return
        
        if not api_key:
            socketio.emit('narration_key_result', {'valid': False, 'error': 'API key is empty', 'models': []}, to=client_id)
            return
        
        # Validate (blocking, but in background thread)
        try:
            is_valid = narration_validate_key(api_key)
            if is_valid:
                print(f"✓ [Narration] Key valid, fetching multimodal models...")
                models = list_multimodal_models(api_key)
                # Save API key and models to config immediately on successful validation
                _narration_config['api_key'] = api_key
                _narration_config['provider'] = provider
                _narration_config['models'] = models
                _save_narration_config(_narration_config)
                print(f"\u2705 [Narration] Key valid, {len(models)} multimodal models available")
                socketio.emit('narration_key_result', {'valid': True, 'models': models, 'error': ''}, to=client_id)
                # Emit updated config sync so UI reflects saved state immediately
                socketio.emit('narration_config_sync', _build_narration_config_sync_payload(), to=client_id)
            else:
                print(f"❌ [Narration] Key validation failed")
                socketio.emit('narration_key_result', {'valid': False, 'models': [], 'error': 'Invalid API key'}, to=client_id)
        except Exception as e:
            print(f"❌ [Narration] Validation error: {e}")
            socketio.emit('narration_key_result', {'valid': False, 'models': [], 'error': f'Validation failed: {str(e)}'}, to=client_id)
    
    # Run validation in background thread
    validation_thread = threading.Thread(target=validate_async, daemon=True)
    validation_thread.start()

@socketio.on('narration_config_update')
def on_narration_config_update(data):
    """Update narration configuration (model, interval, etc)."""
    print(f"\n\U0001f399\ufe0f [Narration] Config update: {data}")
    
    if 'model' in data:
        _narration_config['model'] = data['model']
        car_state['narration_model'] = data['model']
    if 'interval' in data:
        _narration_config['interval'] = max(10, min(300, int(data['interval'])))
        car_state['narration_interval'] = _narration_config['interval']
    if 'provider' in data:
        _narration_config['provider'] = data['provider']
        car_state['narration_provider'] = data['provider']
    
    _save_narration_config(_narration_config)
    
    # Reconfigure the engine
    if _narration_config.get('api_key') and _narration_config.get('model'):
        narration_engine.configure(
            api_key=_narration_config['api_key'],
            model_name=_narration_config['model'],
            interval=_narration_config.get('interval', 30),
        )
    narration_engine.set_elevenlabs_config(
        api_key=_effective_elevenlabs_api_key(),
        voice_id=_effective_elevenlabs_voice_id(),
    )
    
    emit('narration_config_response', {'status': 'ok'})

@socketio.on('narration_toggle')
def on_narration_toggle(data):
    """Toggle AI narration on/off."""
    desired = data.get('enabled', not car_state['narration_enabled'])
    print(f"\n🎙️ [Narration] Toggle: {desired}")
    
    if desired:
        # Check prerequisites
        if not _narration_config.get('api_key'):
            emit('narration_toggle_response', {'status': 'error', 'message': 'No API key configured', 'enabled': False})
            return
        if not _narration_config.get('model'):
            emit('narration_toggle_response', {'status': 'error', 'message': 'No model selected', 'enabled': False})
            return
        if not car_state.get('camera_enabled'):
            emit('narration_toggle_response', {'status': 'error', 'message': 'Camera must be enabled first', 'enabled': False})
            return
        
        # Configure and start
        narration_engine.configure(
            api_key=_narration_config['api_key'],
            model_name=_narration_config['model'],
            interval=_narration_config.get('interval', 30),
        )
        narration_engine.set_camera(picam2, vision_system)
        narration_engine.set_elevenlabs_config(
            api_key=_effective_elevenlabs_api_key(),
            voice_id=_effective_elevenlabs_voice_id(),
        )
        
        # Apply Kokoro TTS config if available
        if _narration_config.get('kokoro_enabled') and _narration_config.get('kokoro_ip') and _narration_config.get('kokoro_voice'):
            narration_engine.set_kokoro_config(
                _narration_config.get('kokoro_ip'),
                _narration_config.get('kokoro_voice')
            )
            print(f"🎤 [Narration] Kokoro enabled for this session: {_narration_config.get('kokoro_ip')} / {_narration_config.get('kokoro_voice')}")
        
        narration_engine.start()
        car_state['narration_enabled'] = True
        _narration_config['enabled'] = True
        _save_narration_config(_narration_config)
        emit('narration_toggle_response', {'status': 'ok', 'enabled': True})
    else:
        narration_engine.stop()
        car_state['narration_enabled'] = False
        car_state['narration_speaking'] = False
        _narration_config['enabled'] = False
        _save_narration_config(_narration_config)
        emit('narration_toggle_response', {'status': 'ok', 'enabled': False})

@socketio.on('narration_analyze_once')
def on_narration_analyze_once(data):
    """Trigger one immediate AI analysis + narration pass."""
    client_id = request.sid
    print(f"\n🎙️ [Narration] Analyze-once requested by client {client_id[:8]}...")

    # Check prerequisites
    if not _narration_config.get('api_key'):
        emit('narration_analyze_once_response', {'status': 'error', 'message': 'No API key configured'})
        return
    if not _narration_config.get('model'):
        emit('narration_analyze_once_response', {'status': 'error', 'message': 'No model selected'})
        return
    if not car_state.get('camera_enabled'):
        emit('narration_analyze_once_response', {'status': 'error', 'message': 'Camera must be enabled first'})
        return

    def analyze_once_async():
        try:
            # Re-apply latest persisted config before the one-shot run
            narration_engine.configure(
                api_key=_narration_config['api_key'],
                model_name=_narration_config['model'],
                interval=_narration_config.get('interval', 30),
            )
            narration_engine.set_camera(picam2, vision_system)
            narration_engine.set_elevenlabs_config(
                api_key=_effective_elevenlabs_api_key(),
                voice_id=_effective_elevenlabs_voice_id(),
            )

            # Apply Kokoro config if available (otherwise disable for this run)
            if _narration_config.get('kokoro_enabled') and _narration_config.get('kokoro_ip') and _narration_config.get('kokoro_voice'):
                narration_engine.set_kokoro_config(
                    _narration_config.get('kokoro_ip'),
                    _narration_config.get('kokoro_voice')
                )
            else:
                narration_engine.set_kokoro_config(None, None)

            success, message = narration_engine.analyze_once()
            if success:
                socketio.emit(
                    'narration_analyze_once_response',
                    {'status': 'ok', 'message': 'Analysis complete'},
                    to=client_id,
                )
            else:
                socketio.emit(
                    'narration_analyze_once_response',
                    {'status': 'error', 'message': message or 'Analysis failed'},
                    to=client_id,
                )
        except Exception as e:
            print(f"❌ [Narration] Analyze-once failed: {e}")
            socketio.emit(
                'narration_analyze_once_response',
                {'status': 'error', 'message': str(e)},
                to=client_id,
            )

    threading.Thread(target=analyze_once_async, daemon=True).start()

@socketio.on('narration_speaking_done')
def on_narration_speaking_done(data):
    """Client reports TTS playback finished."""
    car_state['narration_speaking'] = False

@socketio.on('narration_key_clear')
def on_narration_key_clear(data):
    """Clear the stored API key and disable narration."""
    print(f"\n\U0001f399\ufe0f [Narration] API key clear requested")
    # Stop narration if running
    if car_state.get('narration_enabled'):
        narration_engine.stop()
        car_state['narration_enabled'] = False
        car_state['narration_speaking'] = False
    # Clear config
    _narration_config['api_key'] = ''
    _narration_config['model'] = ''
    _narration_config['models'] = []
    _narration_config['enabled'] = False
    _save_narration_config(_narration_config)
    # Broadcast cleared config to all clients
    _cleared = _build_narration_config_sync_payload()
    _cleared['api_key_set'] = False
    _cleared['api_key_masked'] = ''
    _cleared['model'] = ''
    _cleared['enabled'] = False
    _cleared['models'] = []
    socketio.emit('narration_config_sync', _cleared)
    emit('narration_key_clear_response', {'status': 'ok'})
    print(f"\u2705 [Narration] API key cleared and narration disabled")

@socketio.on('kokoro_validate_api')
def on_kokoro_validate_api(data):
    """Validate Kokoro TTS API connection and fetch available voices.
    Runs in background thread to avoid blocking the SocketIO thread."""
    ip_address = data.get('ip', '')
    client_id = request.sid
    
    def validate_async():
        """Background validation task."""
        print(f"\n\U0001f3a4 [Kokoro] Validating API at {ip_address} (client: {client_id[:8]}...)")
        
        if not ip_address:
            socketio.emit('kokoro_validation_result', {'valid': False, 'voices': [], 'error': 'IP address is empty'}, to=client_id)
            return
        
        # Validate Kokoro API
        try:
            kokoro = get_kokoro_client()
            result = kokoro.validate_api(ip_address)
            # Persist voices list and IP on successful validation
            if result.get('valid') and result.get('voices'):
                _narration_config['kokoro_voices'] = result['voices']
                _narration_config['kokoro_ip'] = ip_address
                _save_narration_config(_narration_config)
                print(f"✅ [Kokoro] Persisted {len(result['voices'])} voices for {ip_address}")
            socketio.emit('kokoro_validation_result', result, to=client_id)
        except Exception as e:
            print(f"❌ [Kokoro] Validation error: {e}")
            socketio.emit('kokoro_validation_result', {'valid': False, 'voices': [], 'error': f'Validation failed: {str(e)}'}, to=client_id)
    
    # Run validation in background thread
    validation_thread = threading.Thread(target=validate_async, daemon=True)
    validation_thread.start()

@socketio.on('kokoro_config_update')
def on_kokoro_config_update(data):
    """Update Kokoro TTS configuration (enable/disable, IP, voice)."""
    print(f"\n🎤 [Kokoro] Config update: {data}")
    
    if 'kokoro_enabled' in data:
        _narration_config['kokoro_enabled'] = data['kokoro_enabled']
        car_state['kokoro_enabled'] = data['kokoro_enabled']
    
    if 'kokoro_ip' in data:
        _narration_config['kokoro_ip'] = data['kokoro_ip']
        car_state['kokoro_ip'] = data['kokoro_ip']
    
    if 'kokoro_voice' in data:
        _narration_config['kokoro_voice'] = data['kokoro_voice']
        car_state['kokoro_voice'] = data['kokoro_voice']
    
    _save_narration_config(_narration_config)
    
    # Always apply configuration to narration engine (whether it's running or not)
    if _narration_config.get('kokoro_enabled') and _narration_config.get('kokoro_ip') and _narration_config.get('kokoro_voice'):
        narration_engine.set_kokoro_config(
            _narration_config.get('kokoro_ip'),
            _narration_config.get('kokoro_voice')
        )
        print(f"✅ [Kokoro] Configuration applied to narration engine: {_narration_config.get('kokoro_ip')} / {_narration_config.get('kokoro_voice')}")
    else:
        narration_engine.set_kokoro_config(None, None)
        print(f"⚠️  [Kokoro] Kokoro disabled or incomplete configuration")
    
    emit('kokoro_config_response', {'status': 'ok'})

# ==========================================
# ⬛ STARTUP CHECK RUNTIME
# ==========================================

def _trigger_startup_check_async(client_id=None, source="manual", require_enabled=False):
    """Launch startup diagnostics in a background thread with overlap protection."""
    global _startup_check_running

    if require_enabled and not _startup_config.get('startup_check_enabled', True):
        return False, "disabled"

    with _startup_check_lock:
        if _startup_check_running:
            return False, "already_running"
        _startup_check_running = True

    def run_check_async():
        global _startup_check_running
        try:
            from startup_systems_check import main as run_startup_check

            gemini_key = _narration_config.get('api_key')  # Use narration API key for Gemini
            eleven_key = _startup_config.get('elevenlabs_api_key')
            voice_id = _startup_config.get('elevenlabs_voice_id', 'JBFqnCBsd6RMkjVDRZzb')
            kokoro_enabled = _narration_config.get('kokoro_enabled', False)
            kokoro_ip = _narration_config.get('kokoro_ip')
            kokoro_voice = _narration_config.get('kokoro_voice')

            print(f"⬛ [Startup] Running diagnostics ({source}) with provided credentials...")

            def _on_speech_start():
                try:
                    car_audio.duck_engine_volume(True)
                except Exception as duck_err:
                    print(f"⚠️  [Startup] Could not duck engine audio: {duck_err}")

            def _on_speech_end():
                try:
                    car_audio.duck_engine_volume(False)
                except Exception as unduck_err:
                    print(f"⚠️  [Startup] Could not restore engine audio: {unduck_err}")

            status = run_startup_check(
                gemini_key=gemini_key,
                eleven_key=eleven_key,
                voice_id=voice_id,
                kokoro_enabled=kokoro_enabled,
                kokoro_ip=kokoro_ip,
                kokoro_voice=kokoro_voice,
                audio_device=CAR_AUDIO_DEVICE,
                on_speech_start=_on_speech_start,
                on_speech_end=_on_speech_end,
            )

            result_data = {
                'status': 'complete',
                'critical_ok': status.critical_systems_ready,
                'all_ok': status.all_systems_ready,
                'pico_bridge': status.pico_bridge_ok,
                'front_sonar': status.front_sonar_ok,
                'laser': status.laser_ok,
                'rear_sonar': status.rear_sonar_ok,
                'mpu6050': status.mpu6050_ok,
                'ir_sensors': status.ir_sensors_ok,
                'encoder': status.encoder_ok,
                'battery_voltage': status.battery_voltage,
                'motor_current': status.motor_current,
            }

            if client_id:
                print(f"⬛ [Startup] Check complete ({source}) - sending results to client {client_id}")
                socketio.emit('startup_check_result', result_data, to=client_id)
            else:
                print(f"⬛ [Startup] Check complete ({source}) - broadcasting results")
                socketio.emit('startup_check_result', result_data)

        except Exception as e:
            print(f"❌ [Startup] Check failed ({source}): {e}")
            error_payload = {'status': 'error', 'error': str(e)}
            if client_id:
                socketio.emit('startup_check_result', error_payload, to=client_id)
            else:
                socketio.emit('startup_check_result', error_payload)
        finally:
            with _startup_check_lock:
                _startup_check_running = False

    check_thread = threading.Thread(target=run_check_async, daemon=True)
    check_thread.start()
    return True, "running"

# ==========================================
# ⬛ STARTUP CHECK SOCKET EVENTS
# ==========================================

@socketio.on('startup_config_request')
def on_startup_config_request(data):
    """Handle request for current startup config."""
    print(f"⬛ [Startup] Config request received")
    emit('startup_config_sync', _build_startup_config_sync_payload())

@socketio.on('startup_config_update')
def on_startup_config_update(data):
    """Update startup check configuration (enabled/disabled, API key, voice ID)."""
    print(f"\n⬛ [Startup] Config update requested")
    
    if 'startup_check_enabled' in data:
        _startup_config['startup_check_enabled'] = data['startup_check_enabled']
        print(f"   Startup check: {'ENABLED' if data['startup_check_enabled'] else 'DISABLED'}")
    
    if 'elevenlabs_voice_id' in data:
        _startup_config['elevenlabs_voice_id'] = data['elevenlabs_voice_id']
        # Also update narration config for consistency
        _narration_config['elevenlabs_voice_id'] = data['elevenlabs_voice_id']
        print(f"   ElevenLabs voice ID: {data['elevenlabs_voice_id']}")
    
    _sync_elevenlabs_config_between_files()
    _save_startup_config(_startup_config)
    _save_narration_config(_narration_config)
    narration_engine.set_elevenlabs_config(
        api_key=_effective_elevenlabs_api_key(),
        voice_id=_effective_elevenlabs_voice_id(),
    )
    
    socketio.emit('startup_config_sync', _build_startup_config_sync_payload())
    emit('startup_config_response', {'status': 'ok'})
    print(f"✅ [Startup] Configuration saved and synced")

@socketio.on('elevenlabs_validate_key')
def on_elevenlabs_validate_key(data):
    """Validate ElevenLabs API key and fetch available voices."""
    api_key = data.get('api_key', '')
    client_id = request.sid
    
    if not api_key:
        emit('elevenlabs_validation_result', {'valid': False, 'voices': [], 'error': 'API key is empty'})
        return
    
    def validate_async():
        """Background validation task."""
        print(f"\n🎤 [ElevenLabs] Validating API key (client: {client_id[:8]}...)")
        
        try:
            from elevenlabs.client import ElevenLabs
            from elevenlabs.play import play
            client = ElevenLabs(api_key=api_key)
            
            voices = []
            permission_limited = False
            
            try:
                # Try to get available voices
                voices_response = client.voices.get_all()
                
                # Extract voice names and IDs
                if hasattr(voices_response, 'voices'):
                    for voice in voices_response.voices:
                        if hasattr(voice, 'name') and hasattr(voice, 'voice_id'):
                            voices.append({
                                'name': voice.name,
                                'voice_id': voice.voice_id
                            })
            except Exception as voices_error:
                # Check if it's a permission error
                error_str = str(voices_error)
                if 'missing_permissions' in error_str or 'voices_read' in error_str or 'permission' in error_str.lower():
                    print(f"⚠️  [ElevenLabs] API key lacks voices_read permission - using default voices")
                    permission_limited = True
                    # Use default premium voice if not permissioned to list
                    voices = [
                        {'name': 'Rachel (Default)', 'voice_id': ELEVENLABS_DEFAULT_VOICE_ID},
                        {'name': 'Bella', 'voice_id': '2EiwWnXFnvU5JabPnP5r'},
                        {'name': 'Antoni', 'voice_id': 'ErXwobaYp0DKrXfsKm7U'},
                        {'name': 'Josh', 'voice_id': 'nPczCjzI2InUD7YtIKWO'},
                        {'name': 'Arnold', 'voice_id': 'VR6AewLHs0MzaIknZaEp'},
                    ]
                else:
                    raise
            
            if voices:
                # Persist the API key and default voice in both config files
                _startup_config['elevenlabs_api_key'] = api_key
                _startup_config['elevenlabs_voice_id'] = voices[0]['voice_id']
                _narration_config['elevenlabs_api_key'] = api_key
                _narration_config['elevenlabs_voice_id'] = voices[0]['voice_id']
                _save_startup_config(_startup_config)
                _save_narration_config(_narration_config)
                narration_engine.set_elevenlabs_config(
                    api_key=_effective_elevenlabs_api_key(),
                    voice_id=_effective_elevenlabs_voice_id(),
                )
                
                result = {
                    'valid': True,
                    'voices': voices,
                    'error': 'limited_permissions' if permission_limited else ''
                }
                message = f"✅ [ElevenLabs] API key valid - using {len(voices)} voices"
                if permission_limited:
                    message += " (note: API key has limited permissions, using default voice list)"
                print(message)
                
                # Send updated narration config sync so UI reflects saved state
                socketio.emit('narration_config_sync', _build_narration_config_sync_payload(), to=client_id)
                socketio.emit('startup_config_sync', _build_startup_config_sync_payload())
            else:
                result = {
                    'valid': False,
                    'voices': [],
                    'error': 'No voices found in account'
                }
                print(f"⚠️  [ElevenLabs] API key valid but no voices found")
            
            socketio.emit('elevenlabs_validation_result', result, to=client_id)
            
        except Exception as e:
            error_msg = str(e)
            print(f"❌ [ElevenLabs] Validation error: {error_msg}")
            
            # Provide more helpful error messages
            if '401' in error_msg or 'unauthorized' in error_msg.lower():
                user_error = "Invalid or expired API key. Check your ElevenLabs account settings."
            elif 'permission' in error_msg.lower():
                user_error = "API key lacks required permissions. Ensure it has 'voices_read' permission in ElevenLabs account."
            elif 'network' in error_msg.lower() or 'connection' in error_msg.lower():
                user_error = "Network error connecting to ElevenLabs. Check your internet connection."
            else:
                user_error = f"Validation failed: {error_msg[:100]}"
            
            socketio.emit('elevenlabs_validation_result', {
                'valid': False,
                'voices': [],
                'error': user_error
            }, to=client_id)
    
    # Run validation in background thread
    validation_thread = threading.Thread(target=validate_async, daemon=True)
    validation_thread.start()

@socketio.on('elevenlabs_key_clear')
def on_elevenlabs_key_clear(data):
    """Clear the stored ElevenLabs API key."""
    print(f"\n🎤 [ElevenLabs] API key clear requested")
    _startup_config['elevenlabs_api_key'] = ''
    _narration_config['elevenlabs_api_key'] = ''
    _save_startup_config(_startup_config)
    _save_narration_config(_narration_config)
    narration_engine.set_elevenlabs_config(
        api_key='',
        voice_id=_effective_elevenlabs_voice_id(),
    )
    
    # Broadcast cleared config to all clients
    socketio.emit('startup_config_sync', _build_startup_config_sync_payload())
    socketio.emit('narration_config_sync', _build_narration_config_sync_payload())
    emit('elevenlabs_key_clear_response', {'status': 'ok'})
    print(f"✅ [ElevenLabs] API key cleared")

@socketio.on('startup_check_run')
def on_startup_check_run(data):
    """Trigger manual startup check execution."""
    print(f"\n⬛ [Startup] Manual check requested from client")
    client_id = request.sid

    started, reason = _trigger_startup_check_async(
        client_id=client_id,
        source="manual",
        require_enabled=False,
    )

    if started:
        emit('startup_check_response', {'status': 'running', 'message': 'Diagnostic check started in background'})
    elif reason == "already_running":
        emit('startup_check_response', {'status': 'busy', 'message': 'Diagnostic check is already running'})
    else:
        emit('startup_check_response', {'status': 'skipped', 'message': 'Diagnostic check is disabled in config'})

# Heartbeat monitoring thread
def heartbeat_monitor():
    """Monitor client heartbeat - ping clients and check for pong responses"""
    last_ping_time = time.time()
    
    while True:
        try:
            current_time = time.time()
            
            # Send heartbeat_ping to all connected clients periodically
            if current_time - last_ping_time >= HEARTBEAT_INTERVAL:
                if client_last_pong:  # Only ping if clients are connected
                    # Need app context for socketio.emit in background thread
                    with app.app_context():
                        socketio.emit('heartbeat_ping', {})
                    print(f"💗 [Heartbeat] 🔔 Heartbeat ping sent to all clients")
                    last_ping_time = current_time
            
            # Check for dead clients (no pong response within timeout)
            dead_clients = []
            for client_id, last_pong_time in client_last_pong.items():
                if current_time - last_pong_time > HEARTBEAT_TIMEOUT:
                    dead_clients.append(client_id)
            
            # Handle dead clients
            if dead_clients:
                for client_id in dead_clients:
                    print(f"🚨 [Heartbeat] ❌ HEARTBEAT LOST from client {client_id}")
                    print(f"🚨 [Heartbeat] 🛑 ACTIVATING EMERGENCY BRAKES!")
                    car_state["emergency_brake_active"] = True
                    car_state["heartbeat_active"] = False
                    car_state["current_pwm"] = 0
                    car_state["gas_pressed"] = False
                    car_state["brake_pressed"] = False
                
                # Remove dead clients from tracking
                for client_id in dead_clients:
                    del client_last_pong[client_id]
            
            time.sleep(0.1)  # Check every 100ms
        except Exception as e:
            print(f"❌ Heartbeat monitor error: {e}")
            time.sleep(0.1)

# Start heartbeat monitor thread
heartbeat_thread = threading.Thread(target=heartbeat_monitor, daemon=True)
heartbeat_thread.start()
print("💗 [Heartbeat] ✅ Heartbeat monitor started (interval: {:.1f}s, timeout: {:.1f}s)".format(HEARTBEAT_INTERVAL, HEARTBEAT_TIMEOUT))

# ==========================================
# 🔄 SPEED ENCODER + POWER TELEMETRY (from Pico bridge)
# ==========================================
# Encoder and ADC are now on the Pico — read via UART bridge.
car_state["encoder_available"] = True  # Pico always provides RPM
print("🔄 [Encoder] ✅ Speed encoder reading from Pico bridge")

def encoder_and_power_thread():
    """Read RPM from Pico bridge every 200ms, battery/current every 1s."""
    interval = 0.2  # 200ms for RPM
    power_counter = 0  # count intervals; read power every 5th (= 1s)
    while True:
        try:
            time.sleep(interval)

            # --- RPM from Pico ---
            rpm = pico_get_rpm()
            speed_mpm = rpm * WHEEL_CIRCUMFERENCE_M
            car_state["encoder_rpm"] = round(rpm, 1)
            car_state["encoder_speed_mpm"] = round(speed_mpm, 2)

            # --- Battery / Current from Pico ADC (every ~1s) ---
            power_counter += 1
            if power_counter >= 5:
                power_counter = 0
                try:
                    batt_v = pico_get_battery_voltage()
                    car_state["battery_voltage"] = round(batt_v, 2) if batt_v >= 0 else -1
                except Exception:
                    car_state["battery_voltage"] = -1
                try:
                    curr_a = pico_get_current_sense()
                    car_state["current_amps"] = round(curr_a, 2) if curr_a >= 0 else -1
                except Exception:
                    car_state["current_amps"] = -1

                # --- Update dynamic power limiter with fresh sensor data ---
                bv = car_state["battery_voltage"]
                ca = car_state["current_amps"]
                car_system.update_power_state(
                    bv if bv > 0 else -1,
                    ca if ca >= 0 else 0.0
                )
                pl = car_system.power_limiter
                car_state["effective_max_duty"] = round(pl.max_safe_duty, 1)
                car_state["l298n_voltage_drop"] = round(pl.l298n_drop, 2)
                car_state["effective_motor_voltage"] = round(
                    pl.effective_motor_voltage(car_state["current_pwm"]), 2
                )
        except Exception as e:
            print(f"❌ [Encoder/Power] Read error: {e}")
            time.sleep(interval)

# Start encoder + power thread
encoder_thread = threading.Thread(target=encoder_and_power_thread, daemon=True)
encoder_thread.start()
print("🔄 [Encoder] ✅ Encoder + power telemetry thread started (200ms interval)")

# Start camera frame broadcaster (single producer, multi-consumer)
if CAMERA_AVAILABLE:
    _camera_broadcaster.start()

# Telemetry broadcast thread
def telemetry_broadcast():
    """Broadcast telemetry data to all connected clients at 20Hz"""
    while True:
        try:
            pwm = car_state["current_pwm"]
            
            # Use real encoder data when available, fall back to PWM estimates
            if car_state["encoder_available"]:
                real_rpm = car_state["encoder_rpm"]
                real_speed_mpm = car_state["encoder_speed_mpm"]
            else:
                # Fallback: estimate from PWM duty cycle
                real_rpm = round(pwm * 2.2, 1)
                real_speed_mpm = round(pwm * 2.2 * WHEEL_CIRCUMFERENCE_M, 2)
            
            # 📡 Get fresh accelerometer data from Pico
            accel_x, accel_y, accel_z = pico_get_accel_xyz()
            
            telemetry_data = {
                "rpm": real_rpm,
                "speed_mpm": real_speed_mpm,
                "encoder_available": car_state["encoder_available"],
                "current_pwm": pwm,
                "gear": car_state["gear"],
                "steer_angle": car_state["steer_angle"],
                "direction": car_state["direction"],
                "turning": car_state["turning"],
                "left_obstacle": car_state["left_obstacle"],
                "right_obstacle": car_state["right_obstacle"],
                "gas_pressed": car_state["gas_pressed"],
                "brake_pressed": car_state["brake_pressed"],
                "engine_running": car_state["engine_running"],
                "ir_enabled": car_state["ir_enabled"],
                "heartbeat_active": car_state["heartbeat_active"],
                "emergency_brake_active": car_state["emergency_brake_active"],
                "temperature": get_cpu_temperature(),
                "cpu_clock": get_cpu_clock(),
                "gpu_clock": get_gpu_clock(),
                # 🤖 Autonomous driving telemetry
                "autonomous_mode": car_state["autonomous_mode"],
                "autonomous_state": car_state["autonomous_state"],
                "autonomous_target_speed": car_system._current_speed if car_state["autonomous_mode"] else 0,
                "sonar_distance": car_state["sonar_distance"],
                "sonar_enabled": car_state["sonar_enabled"],
                "mpu6050_enabled": car_state["mpu6050_enabled"],
                "rear_sonar_enabled": car_state["rear_sonar_enabled"],
                # 🧭 MPU6050 Accelerometer telemetry
                "accel_x": accel_x,
                "accel_y": accel_y,
                "accel_z": accel_z,
                # 🧭 MPU6050 Gyro telemetry
                "gyro_z": car_state["gyro_z"],
                "pid_correction": car_state["pid_correction"],
                "gyro_available": car_state["gyro_available"],
                "gyro_calibrated": car_state["gyro_calibrated"],
                # 🧭 Slalom yaw-tracking telemetry
                "target_yaw": car_state.get("target_yaw", 0.0),
                "current_heading": car_state.get("current_heading", 0.0),
                "slalom_sign": car_state.get("slalom_sign", 0),
                # � Battery / power telemetry
                "battery_voltage": car_state.get("battery_voltage", -1),
                "current_amps": car_state.get("current_amps", -1),
                # �🚨 Sensor health status
                "sensor_status": car_state["sensor_status"],
                "service_light_active": car_state["service_light_active"],
                # 📷 Camera status
                "camera_enabled": car_state["camera_enabled"],
                "user_wants_vision": car_state["user_wants_vision"],
                # 📷 Vision / Object Detection telemetry
                "vision_active": car_state["vision_active"],
                "camera_obstacle_distance": car_state["camera_obstacle_distance"],
                "camera_detections_count": car_state["camera_detections_count"],
                "camera_in_path_count": car_state["camera_in_path_count"],
                "camera_closest_object": car_state["camera_closest_object"],
                "camera_closest_confidence": car_state["camera_closest_confidence"],
                "vision_fps": car_state["vision_fps"],
                # 📷 Camera configuration (for HUD badge)
                "camera_resolution": car_state["camera_resolution"],
                "camera_jpeg_quality": car_state["camera_jpeg_quality"],
                "camera_framerate": car_state["camera_framerate"],
                "camera_actual_fps": car_state["camera_actual_fps"],
                # 🎙️ Narration telemetry
                "narration_enabled": car_state["narration_enabled"],
                "narration_speaking": car_state["narration_speaking"],
            }
            
            socketio.emit('telemetry_update', telemetry_data)
            time.sleep(0.05)  # 20Hz = 50ms
        except Exception as e:
            print(f"❌ Telemetry broadcast error: {e}")
            time.sleep(0.05)

# Start telemetry broadcaster thread
telemetry_thread = threading.Thread(target=telemetry_broadcast, daemon=True)
telemetry_thread.start()

# ==========================================
# 🌐 HTTP ENDPOINTS (kept for backwards compatibility)
# ==========================================
@app.route("/telemetry")
def telemetry():
    """Returns current car state for the dashboard"""
    pwm = car_state["current_pwm"]
    
    # Use real encoder data when available, fall back to PWM estimates
    if car_state["encoder_available"]:
        real_rpm = car_state["encoder_rpm"]
        real_speed_mpm = car_state["encoder_speed_mpm"]
    else:
        real_rpm = round(pwm * 2.2, 1)
        real_speed_mpm = round(pwm * 2.2 * WHEEL_CIRCUMFERENCE_M, 2)
    
    # 📡 Get fresh accelerometer data from Pico
    accel_x, accel_y, accel_z = pico_get_accel_xyz()
    
    return jsonify({
        "rpm": real_rpm,
        "speed_mpm": real_speed_mpm,
        "encoder_available": car_state["encoder_available"],
        "current_pwm": pwm,
        "gear": car_state["gear"],
        "steer_angle": car_state["steer_angle"],
        "direction": car_state["direction"],
        "turning": car_state["turning"],
        "left_obstacle": car_state["left_obstacle"],
        "right_obstacle": car_state["right_obstacle"],
        "engine_running": car_state["engine_running"],
        "temperature": get_cpu_temperature(),
        "cpu_clock": get_cpu_clock(),
        "gpu_clock": get_gpu_clock(),
        # 🧭 MPU6050 Accelerometer telemetry
        "accel_x": accel_x,
        "accel_y": accel_y,
        "accel_z": accel_z,
        # 🤖 Autonomous driving telemetry
        "autonomous_mode": car_state["autonomous_mode"],
        "autonomous_state": car_state["autonomous_state"],
        "autonomous_target_speed": car_state["autonomous_target_speed"],
        "sonar_distance": car_state["sonar_distance"],
        "sonar_enabled": car_state["sonar_enabled"],
        # 🔋 Battery / power telemetry
        "battery_voltage": car_state.get("battery_voltage", -1),
        "current_amps": car_state.get("current_amps", -1),
        # ⚡ Dynamic power limiter telemetry
        "effective_max_duty": car_state.get("effective_max_duty", MAX_PWM_DUTY),
        "l298n_voltage_drop": car_state.get("l298n_voltage_drop", 1.5),
        "effective_motor_voltage": car_state.get("effective_motor_voltage", 0.0),
    })

@app.route("/system/status")
def system_status():
    """Returns system status: WiFi, hotspot, temperature, clock speeds"""
    wifi_connected, ssid = get_wifi_status()
    hotspot_active = get_hotspot_status()
    cpu_temp = get_cpu_temperature()
    cpu_clock = get_cpu_clock()
    gpu_clock = get_gpu_clock()
    
    return jsonify({
        "wifi_connected": wifi_connected,
        "ssid": ssid,
        "hotspot_active": hotspot_active,
        "cpu_temp": cpu_temp,
        "cpu_clock": cpu_clock,
        "gpu_clock": gpu_clock,
        "ir_enabled": car_state["ir_enabled"]
    })

@app.route("/ir/<state>")
def set_ir(state):
    """Toggle IR sensor on/off"""
    # Block IR toggle when in autonomous mode for safety
    if car_state["autonomous_mode"]:
        print("❌ IR SENSORS: BLOCKED - Cannot toggle in autonomous mode")
        return "BLOCKED_AUTONOMOUS_MODE"
    
    if state.lower() == "on":
        car_state["ir_enabled"] = True
        print("✅ IR Sensors ENABLED")
        return "IR_ON"
    elif state.lower() == "off":
        car_state["ir_enabled"] = False
        print("❌ IR Sensors DISABLED")
        return "IR_OFF"
    return "INVALID"

# --- SYSTEM SETTINGS ---

@app.route("/system/reboot")
def system_reboot():
    """Reboots the Raspberry Pi system"""
    print("🔄 SYSTEM REBOOT initiated!")
    try:
        subprocess.Popen(['sudo', 'reboot'])
        return "REBOOT_INITIATED"
    except Exception as e:
        print(f"❌ Reboot failed: {e}")
        return "REBOOT_FAILED"

@app.route("/system/switch_network_mode", methods=['POST'])
def switch_network_mode():
    """Switch between WiFi client mode and hotspot mode for wlan0."""
    try:
        data = request.get_json(silent=True) or {}
        requested_mode = str(data.get('mode', '')).strip().lower()

        if requested_mode not in {"wifi", "hotspot"}:
            return jsonify({
                "status": "error",
                "message": "Invalid mode. Use 'wifi' or 'hotspot'.",
            }), 400

        current_mode = "hotspot" if get_hotspot_status() else "wifi"
        if requested_mode == current_mode:
            current_ip = get_local_ip()
            return jsonify({
                "status": "ok",
                "mode": current_mode,
                "message": f"Already in {current_mode} mode.",
                "redirect_url": f"http://{current_ip}:5000",
            })

        if requested_mode == "hotspot":
            activated = network_manager.enable_hotspot()
            if not activated:
                return jsonify({
                    "status": "error",
                    "message": "Failed to issue hotspot activation command.",
                }), 500

            redirect_url = f"http://{DEFAULT_HOTSPOT_IP}:5000"
            print(f"📡 Network switch requested via network_core.py: WIFI → HOTSPOT ({DEFAULT_HOTSPOT_IP})")
        else:
            activated = network_manager.enable_wifi()
            if not activated:
                return jsonify({
                    "status": "error",
                    "message": "Failed to issue WiFi activation command.",
                }), 500

            redirect_host = DEFAULT_WIFI_REDIRECT_HOST.strip() or "raspberrypi.local"
            redirect_url = f"http://{redirect_host}:5000"
            print(f"📡 Network switch requested via network_core.py: HOTSPOT → WIFI ({redirect_host})")

        return jsonify({
            "status": "ok",
            "mode": requested_mode,
            "message": "Network switch started. Connection interruption expected.",
            "redirect_url": redirect_url,
            "disrupts_connection": True,
        })
    except Exception as e:
        print(f"❌ Network mode switch failed: {e}")
        return jsonify({
            "status": "error",
            "message": f"Network switch failed: {str(e)}",
        }), 500

@app.route("/system/configure_hotspot", methods=['POST'])
def configure_hotspot():
    """Configures hotspot mode with SSID, password, and IP"""
    try:
        data = request.get_json()
        ssid = data.get('ssid', 'RC-Car-Connect')
        password = data.get('password', '')
        ip = data.get('ip', '192.168.4.1')
        
        if not password:
            return "HOTSPOT_CONFIG_FAILED: Password required"
        
        print(f"📡 Configuring hotspot...")
        print(f"   SSID: {ssid}")
        print(f"   IP: {ip}")
        
        # Create a shell script to configure hotspot
        hotspot_script = f"""#!/bin/bash
set -e

echo "🔧 Setting up WiFi Hotspot..."

# Disable WiFi
sudo ifconfig wlan0 down 2>/dev/null || true

# Install hostapd and dnsmasq if not already installed
echo "📦 Installing hostapd and dnsmasq..."
sudo apt-get update -qq
sudo apt-get install -y -qq hostapd dnsmasq

# Configure hostapd
echo "⚙️ Configuring hostapd..."
sudo tee /etc/hostapd/hostapd.conf > /dev/null <<'HOSTAPD'
interface=wlan0
driver=nl80211
ssid={ssid}
hw_mode=g
channel=6
wmm_enabled=0
wpa=2
wpa_passphrase={password}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
HOSTAPD

# Configure dnsmasq
echo "⚙️ Configuring dnsmasq..."
sudo tee /etc/dnsmasq.conf > /dev/null <<'DNSMASQ'
interface=wlan0
dhcp-range={ip.rsplit('.', 1)[0]}.100,{ip.rsplit('.', 1)[0]}.200,12h
address=/#/{ip}
DNSMASQ

# Bring interface up and configure IP
echo "🔌 Bringing up wlan0..."
sudo ifconfig wlan0 up
sudo ifconfig wlan0 {ip} netmask 255.255.255.0

# Start services
echo "🚀 Starting hostapd and dnsmasq..."
sudo systemctl start hostapd
sudo systemctl start dnsmasq

# Enable IP forwarding
echo "🌉 Enabling IP forwarding..."
sudo sysctl -w net.ipv4.ip_forward=1 > /dev/null

echo "✅ Hotspot configured successfully!"
echo "   Connect to: {ssid}"
echo "   IP Address: {ip}"
"""
        
        # Write the script
        script_path = '/tmp/setup_hotspot.sh'
        with open(script_path, 'w') as f:
            f.write(hotspot_script)
        
        # Make it executable
        os.chmod(script_path, 0o755)
        
        # Run the script in background
        subprocess.Popen(['bash', script_path], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL)
        
        print("✅ Hotspot configuration initiated")
        return "HOTSPOT_CONFIG_INITIATED"
    except Exception as e:
        print(f"❌ Hotspot configuration failed: {e}")
        return f"HOTSPOT_CONFIG_FAILED: {str(e)}"

if __name__ == "__main__":
    try:
        # Host 0.0.0.0 makes it available on your Wi-Fi
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    finally:
        # Safety cleanup — car_system.cleanup() stops PWMs + releases GPIO
        _camera_broadcaster.stop()
        car_audio.shutdown()
        narration_engine.stop()
        autopilot.stop()
        car_system.cleanup()
