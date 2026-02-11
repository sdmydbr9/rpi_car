"""
🎙️ AI Narration Module
Provides Gemini-powered image description for the robot's camera feed.
Supports key validation, multimodal model discovery, and image analysis.
"""

import google.generativeai as genai
import threading
import time
import cv2
import numpy as np
from tts_local import get_tts_synthesizer
from kokoro_client import get_kokoro_client

# ==========================================
# 🔑 API KEY VALIDATION
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

# Default narration prompt
DEFAULT_PROMPT = (
    "you are a robot who just got vision for the first time, "
    "describe what do you see in less than 20 words, "
    "be narrative and interesting and use human like expressions"
)

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
        
        response = model.generate_content(
            [prompt, {'mime_type': 'image/jpeg', 'data': jpeg_bytes}],
            safety_settings=safety_settings,
        )
        
        # Handle blocked / empty responses
        if not response.candidates:
            print(f"⚠️ [Narration] Response blocked (no candidates)")
            return ""
        
        try:
            text = response.text.strip()
        except ValueError as e:
            print(f"⚠️ [Narration] Response blocked by safety filter: {e}")
            return ""
        
        print(f"👁️ [Narration] AI says: {text}")
        return text
    except Exception as e:
        error_msg = f"Narration error: {e}"
        print(f"❌ [Narration] {error_msg}")
        return ""


# ==========================================
# 📸 FRAME CAPTURE HELPER
# ==========================================

def capture_frame_as_jpeg(picam2=None, vision_system=None,
                          jpeg_quality: int = 70) -> bytes | None:
    """Capture a single frame from the camera pipeline and encode as JPEG.
    
    Tries vision system first (to get annotated frame), falls back to
    raw picam2 capture. Returns JPEG bytes or None on failure.
    """
    try:
        frame = None
        
        # Try vision system annotated frame first
        if vision_system is not None:
            frame = vision_system.get_frame()
        
        # Fall back to raw picam2
        if frame is None and picam2 is not None:
            frame = picam2.capture_array()
        
        if frame is None:
            return None
        
        # Convert BGR to RGB (picam2 captures BGR888)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', frame_rgb,
                                   [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ret:
            return None
        
        return buffer.tobytes()
    except Exception as e:
        print(f"❌ [Narration] Frame capture failed: {e}")
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
        self._interval: float = 90.0  # seconds between narrations
        self._prompt: str = DEFAULT_PROMPT
        self._picam2 = None
        self._vision_system = None
        self._on_narration_text = None  # callback(text: str)
        self._on_narration_error = None  # callback(error: str)
        self._lock = threading.Lock()
        self._play_local_tts = True  # Always play audio on Pi's headphone jack by default
        # Kokoro TTS configuration
        self._kokoro_enabled = False
        self._kokoro_ip: str = ""
        self._kokoro_voice: str = ""
    
    def configure(self, api_key: str, model_name: str,
                  interval: float = 90.0, prompt: str = DEFAULT_PROMPT):
        """Configure narration parameters."""
        with self._lock:
            self._api_key = api_key
            self._model_name = model_name
            self._interval = max(90.0, min(120.0, interval))
            self._prompt = prompt
    
    def set_local_tts_enabled(self, enabled: bool):
        """Enable/disable local TTS playback on Pi's headphone jack."""
        with self._lock:
            self._play_local_tts = enabled
        status = "enabled" if enabled else "disabled"
        print(f"🔊 [Narration] Local TTS playback {status}")
    
    def set_kokoro_config(self, ip_address: str | None, voice: str | None):
        """Configure Kokoro TTS (remote FastAPI server).
        
        Pass None for either parameter to disable Kokoro.
        """
        with self._lock:
            if ip_address and voice:
                self._kokoro_enabled = True
                self._kokoro_ip = ip_address
                self._kokoro_voice = voice
                print(f"🎤 [Narration] Kokoro enabled - IP: {ip_address}, Voice: {voice}")
            else:
                self._kokoro_enabled = False
                self._kokoro_ip = ""
                self._kokoro_voice = ""
                print(f"🎤 [Narration] Kokoro disabled - will fallback to local TTS")
    
    def set_camera(self, picam2, vision_system):
        """Set camera references for frame capture."""
        with self._lock:
            self._picam2 = picam2
            self._vision_system = vision_system
    
    def set_callback(self, callback):
        """Set the callback for narration text output."""
        self._on_narration_text = callback
    
    def set_error_callback(self, callback):
        """Set the callback for narration error output."""
        self._on_narration_error = callback
    
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
    
    def _loop(self):
        """Background narration loop."""
        consecutive_errors = 0
        while self._running:
            try:
                with self._lock:
                    api_key = self._api_key
                    model_name = self._model_name
                    interval = self._interval
                    prompt = self._prompt
                    picam2 = self._picam2
                    vision_system = self._vision_system
                    kokoro_enabled = self._kokoro_enabled
                    kokoro_ip = self._kokoro_ip
                    kokoro_voice = self._kokoro_voice
                    play_local_tts = self._play_local_tts
                
                # Capture frame
                jpeg_bytes = capture_frame_as_jpeg(picam2, vision_system)
                if jpeg_bytes is None:
                    msg = "No camera frame available — is the camera enabled?"
                    print(f"⚠️  [Narration] {msg}")
                    if self._on_narration_error:
                        self._on_narration_error(msg)
                    time.sleep(interval)
                    continue
                
                # Send to AI
                start_time = time.time()
                text = describe_image(api_key, model_name, jpeg_bytes, prompt)
                elapsed = time.time() - start_time
                
                if text and self._on_narration_text:
                    self._on_narration_text(text)
                    consecutive_errors = 0
                    print(f"⏱️ [Narration] Response in {elapsed:.1f}s")
                    
                    # Play audio - try Kokoro first, fallback to local TTS
                    kokoro_success = False
                    if kokoro_enabled and kokoro_ip and kokoro_voice:
                        # Try Kokoro first
                        print(f"🎤 [Narration] Attempting Kokoro TTS at {kokoro_ip}...")
                        kokoro = get_kokoro_client()
                        kokoro_success = kokoro.synthesize_and_stream(
                            kokoro_ip, text, kokoro_voice
                        )
                        if not kokoro_success:
                            print(f"⚠️  [Narration] Kokoro synthesis failed, falling back to local TTS")
                    
                    # Fallback to local TTS if Kokoro not enabled or failed
                    if not kokoro_success and play_local_tts:
                        print(f"🔊 [Narration] Using local TTS...")
                        tts = get_tts_synthesizer()
                        tts.speak(text)
                elif not text:
                    consecutive_errors += 1
                    print(f"⚠️  [Narration] Empty response (attempt {consecutive_errors})")
                    if consecutive_errors >= 3 and self._on_narration_error:
                        self._on_narration_error(
                            "AI returned empty responses — check API key, model, or try a different model"
                        )
                        consecutive_errors = 0
                
            except Exception as e:
                print(f"❌ [Narration] Loop error: {e}")
                if self._on_narration_error:
                    self._on_narration_error(str(e))
            
            # Wait for the configured interval
            # Use small sleep increments so we can stop quickly
            wait_end = time.time() + interval
            while self._running and time.time() < wait_end:
                time.sleep(0.5)
