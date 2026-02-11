"""
🔊 Local TTS Synthesizer for Raspberry Pi
Uses espeak-ng or pyttsx3 to generate speech and play via ALSA/PulseAudio.
Falls back gracefully if TTS is not available.
"""

import subprocess
import os
import threading
import time
from pathlib import Path

# ==========================================
# 🔊 TTS PROVIDER DETECTION
# ==========================================

def _check_espeak() -> bool:
    """Check if espeak-ng is installed."""
    try:
        result = subprocess.run(['which', 'espeak-ng'], 
                              capture_output=True, timeout=2)
        return result.returncode == 0
    except Exception:
        return False

def _check_pyttsx3() -> bool:
    """Check if pyttsx3 is installed."""
    try:
        import pyttsx3
        return True
    except ImportError:
        return False

# Detect available TTS providers
HAS_ESPEAK = _check_espeak()
HAS_PYTTSX3 = _check_pyttsx3()

if HAS_PYTTSX3:
    import pyttsx3

# ==========================================
# 🔊 LOCAL TTS SYNTHESIZER
# ==========================================

class LocalTTSynthesizer:
    """Synthesizes and plays speech locally on the Pi via headphone jack."""
    
    def __init__(self):
        self._playing = False
        self._play_thread: threading.Thread | None = None
        self._tts_engine = None
        self._lock = threading.Lock()
        
        # Initialize pyttsx3 if available
        if HAS_PYTTSX3:
            try:
                self._tts_engine = pyttsx3.init()
                # Configure for Pi (headphone jack output)
                self._tts_engine.setProperty('rate', 150)  # Speed (slower for clarity)
                self._tts_engine.setProperty('volume', 1.0)  # Max volume
                print("🔊 [TTS] Using pyttsx3 engine")
            except Exception as e:
                print(f"⚠️  [TTS] pyttsx3 initialization failed: {e}")
                self._tts_engine = None
    
    @property
    def is_available(self) -> bool:
        """Check if TTS is available (espeak or pyttsx3)."""
        return HAS_ESPEAK or HAS_PYTTSX3
    
    @property
    def is_playing(self) -> bool:
        """Check if audio is currently playing."""
        with self._lock:
            return self._playing
    
    def speak(self, text: str) -> bool:
        """
        Synthesize and play text immediately.
        Returns True if playback started, False otherwise.
        """
        if not self.is_available:
            print("❌ [TTS] No TTS engine available (espeak-ng or pyttsx3 required)")
            return False
        
        if not text or not text.strip():
            print("⚠️  [TTS] Empty text, skipping")
            return False
        
        # Stop any in-progress playback
        self.stop()
        
        with self._lock:
            self._playing = True
        
        # Play in background thread
        self._play_thread = threading.Thread(
            target=self._speak_sync,
            args=(text,),
            daemon=True
        )
        self._play_thread.start()
        return True
    
    def _speak_sync(self, text: str):
        """Synchronous speech synthesis and playback."""
        try:
            print(f"🔊 [TTS] Speaking: {text[:60]}...")
            
            # Try pyttsx3 first (more reliable on Pi)
            if self._tts_engine:
                self._speak_pyttsx3(text)
            # Fall back to espeak-ng
            elif HAS_ESPEAK:
                self._speak_espeak(text)
            
            print(f"🔊 [TTS] ✅ Speech complete")
        except Exception as e:
            print(f"❌ [TTS] Playback error: {e}")
        finally:
            with self._lock:
                self._playing = False
    
    def _speak_pyttsx3(self, text: str):
        """Synthesize and play using pyttsx3."""
        try:
            # Save to temporary WAV file
            temp_wav = '/tmp/tts_output.wav'
            self._tts_engine.save_to_file(text, temp_wav)
            self._tts_engine.runAndWait()
            
            # Play the WAV file via aplay (ALSA player)
            if os.path.exists(temp_wav):
                result = subprocess.run(
                    ['aplay', '-D', 'default', temp_wav],
                    capture_output=True,
                    timeout=30
                )
                if result.returncode == 0:
                    print(f"🔊 [TTS] Played via pyttsx3 + aplay")
                else:
                    print(f"⚠️  [TTS] aplay failed: {result.stderr.decode()}")
                # Clean up
                try:
                    os.remove(temp_wav)
                except:
                    pass
        except Exception as e:
            print(f"❌ [TTS] pyttsx3 playback failed: {e}")
    
    def _speak_espeak(self, text: str):
        """Synthesize and play using espeak-ng."""
        try:
            # Use espeak-ng to synthesize and play directly
            # -a = amplitude (0-200), default 100
            # -s = speed (in words per minute, 80-500)
            # -p = pitch (0-99)
            # -d = audio device (hw:0,0 = card 0 device 0 = headphone jack)
            result = subprocess.run(
                [
                    'espeak-ng',
                    '-a', '100',      # amplitude
                    '-s', '120',      # speed (words per minute)
                    '-p', '50',       # pitch
                    '-d', 'hw:0,0',   # output to card 0 device 0 (headphone jack)
                    '-m',             # no markup interpretation
                    text
                ],
                capture_output=True,
                timeout=30
            )
            if result.returncode == 0:
                print(f"🔊 [TTS] Played via espeak-ng to hw:0,0 (headphone jack)")
            else:
                stderr = result.stderr.decode()
                print(f"⚠️  [TTS] espeak-ng failed: {stderr}")
                
                # Try without specifying device (use system default)
                result = subprocess.run(
                    ['espeak-ng', '-s', '120', '-p', '50', text],
                    capture_output=True,
                    timeout=30
                )
                if result.returncode == 0:
                    print(f"🔊 [TTS] Played via espeak-ng (system default)")
        except Exception as e:
            print(f"❌ [TTS] espeak-ng playback failed: {e}")
    
    def stop(self):
        """Stop current playback."""
        with self._lock:
            if self._playing:
                self._playing = False
                if self._play_thread:
                    self._play_thread.join(timeout=1.0)
                    self._play_thread = None
                print("🔊 [TTS] Stopped")

# ==========================================
# 🔊 SINGLETON INSTANCE
# ==========================================

_tts_synthesizer: LocalTTSynthesizer | None = None

def get_tts_synthesizer() -> LocalTTSynthesizer:
    """Get the global TTS synthesizer instance."""
    global _tts_synthesizer
    if _tts_synthesizer is None:
        _tts_synthesizer = LocalTTSynthesizer()
    return _tts_synthesizer

