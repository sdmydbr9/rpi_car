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
        model = genai.GenerativeModel(model_name)
        
        response = model.generate_content([
            prompt,
            {'mime_type': 'image/jpeg', 'data': jpeg_bytes}
        ])
        
        text = response.text.strip()
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
        self._interval: float = 8.0  # seconds between narrations
        self._prompt: str = DEFAULT_PROMPT
        self._picam2 = None
        self._vision_system = None
        self._on_narration_text = None  # callback(text: str)
        self._lock = threading.Lock()
    
    def configure(self, api_key: str, model_name: str,
                  interval: float = 8.0, prompt: str = DEFAULT_PROMPT):
        """Configure narration parameters."""
        with self._lock:
            self._api_key = api_key
            self._model_name = model_name
            self._interval = max(3.0, min(30.0, interval))
            self._prompt = prompt
    
    def set_camera(self, picam2, vision_system):
        """Set camera references for frame capture."""
        with self._lock:
            self._picam2 = picam2
            self._vision_system = vision_system
    
    def set_callback(self, callback):
        """Set the callback for narration text output."""
        self._on_narration_text = callback
    
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
        while self._running:
            try:
                with self._lock:
                    api_key = self._api_key
                    model_name = self._model_name
                    interval = self._interval
                    prompt = self._prompt
                    picam2 = self._picam2
                    vision_system = self._vision_system
                
                # Capture frame
                jpeg_bytes = capture_frame_as_jpeg(picam2, vision_system)
                if jpeg_bytes is None:
                    print("⚠️  [Narration] No frame available, skipping...")
                    time.sleep(interval)
                    continue
                
                # Send to AI
                start_time = time.time()
                text = describe_image(api_key, model_name, jpeg_bytes, prompt)
                elapsed = time.time() - start_time
                
                if text and self._on_narration_text:
                    self._on_narration_text(text)
                    print(f"⏱️ [Narration] Response in {elapsed:.1f}s")
                
            except Exception as e:
                print(f"❌ [Narration] Loop error: {e}")
            
            # Wait for the configured interval
            # Use small sleep increments so we can stop quickly
            wait_end = time.time() + interval
            while self._running and time.time() < wait_end:
                time.sleep(0.5)
