"""
🎤 Kokoro TTS Client for Kokoro-FastAPI (remsky/Kokoro-FastAPI)
OpenAI-compatible Speech API — validates connection, lists voices,
and streams MP3 audio to the Pi's audio output via mpg123.

API reference (OpenAI-compatible):
  GET  /v1/audio/voices             → list available voices
  POST /v1/audio/speech             → synthesize speech (streaming)
       {"model":"kokoro","input":"...","voice":"...","response_format":"mp3","speed":1.0}
"""

import requests
import subprocess
import threading
import shutil
import os
from typing import Optional

# ==========================================
# 🎤 KOKORO TTS CLIENT
# ==========================================

def _find_player() -> list[str]:
    """Find a suitable CLI audio player for MP3 streaming on the Pi.
    Returns the command list for Popen, e.g. ['mpg123', '-q', '-'].
    """
    # mpg123 -q -  → read MP3 from stdin, quiet mode
    if shutil.which('mpg123'):
        return ['mpg123', '-q', '-']
    # ffplay fallback (part of ffmpeg)
    if shutil.which('ffplay'):
        return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', '-i', 'pipe:0']
    # aplay can only handle WAV; we'll save to temp file
    return []


class KokoroTTSClient:
    """Client for remote Kokoro-FastAPI server (OpenAI-compatible Speech API)."""

    def __init__(self, timeout: int = 30):
        self.timeout = timeout
        self._playing = False
        self._play_thread: Optional[threading.Thread] = None
        self._play_proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Validation — called from background thread via Socket.IO handler
    # ------------------------------------------------------------------
    def validate_api(self, ip_address: str) -> dict:
        """
        Validate Kokoro API connection and fetch available voices.

        Args:
            ip_address: host:port (e.g. "192.168.29.105:8880")

        Returns:
            {'valid': bool, 'voices': list[str], 'error': str|None}
        """
        base_url = f"http://{ip_address}"
        try:
            print(f"🎤 [Kokoro] Validating API at {base_url}...")
            
            # Use tuple timeout: (connect_timeout, read_timeout)
            # connect_timeout: 5s, read_timeout: 15s
            timeout = (5, 15)
            
            # ------- fetch voices via OpenAI-compatible endpoint -------
            try:
                resp = requests.get(
                    f"{base_url}/v1/audio/voices",
                    timeout=timeout,
                )
            except (requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout) as e:
                return {'valid': False, 'voices': [], 'error': f"Timeout during connection: {e}"}
            except requests.exceptions.ConnectionError as e:
                return {'valid': False, 'voices': [], 'error': f"Cannot connect to {base_url}"}
            except requests.exceptions.RequestException as e:
                return {'valid': False, 'voices': [], 'error': f"Request failed: {e}"}
            
            if resp.status_code != 200:
                return {
                    'valid': False,
                    'voices': [],
                    'error': f"Voices endpoint returned HTTP {resp.status_code}",
                }

            # Parse JSON with timeout
            try:
                data = resp.json()
            except Exception as e:
                return {'valid': False, 'voices': [], 'error': f"Invalid JSON response: {e}"}

            # The response is typically {"voices": [{"name": "af_sky", ...}, ...]}
            # or could be a flat list of voice objects / strings.
            voices: list[str] = []
            try:
                if isinstance(data, dict):
                    raw = data.get('voices', data.get('data', []))
                    if isinstance(raw, dict):
                        voices = sorted(raw.keys())
                    elif isinstance(raw, list):
                        for v in raw:
                            if isinstance(v, str):
                                voices.append(v)
                            elif isinstance(v, dict) and isinstance(v.get('name'), str):
                                voices.append(v['name'])
                elif isinstance(data, list):
                    for v in data:
                        if isinstance(v, str):
                            voices.append(v)
                        elif isinstance(v, dict) and isinstance(v.get('name'), str):
                            voices.append(v['name'])
            except Exception as e:
                return {'valid': False, 'voices': [], 'error': f"Error parsing voices: {e}"}

            if not voices:
                return {
                    'valid': False,
                    'voices': [],
                    'error': "API responded but no voices found",
                }

            print(f"🎤 [Kokoro] ✅ API validated at {ip_address} — {len(voices)} voices: {voices[:5]}{'...' if len(voices) > 5 else ''}")
            return {'valid': True, 'voices': voices, 'error': None}

        except Exception as e:
            return {'valid': False, 'voices': [], 'error': f"Unexpected error: {e}"}

    # ------------------------------------------------------------------
    # Synthesis + playback — fire-and-forget (background thread)
    # ------------------------------------------------------------------
    def synthesize_and_stream(
        self,
        ip_address: str,
        text: str,
        voice: str,
        speed: float = 1.0,
    ) -> bool:
        """
        Synthesize text via Kokoro API and stream MP3 to speakers.
        Non-blocking — launches a background thread.
        Returns True if the request was dispatched, False on immediate error.
        """
        if not text or not text.strip():
            print("⚠️  [Kokoro] Empty text, skipping")
            return False

        self.stop()

        with self._lock:
            self._playing = True

        self._play_thread = threading.Thread(
            target=self._synthesize_sync,
            args=(ip_address, text, voice, speed),
            daemon=True,
        )
        self._play_thread.start()
        return True

    def _synthesize_sync(
        self,
        ip_address: str,
        text: str,
        voice: str,
        speed: float,
    ):
        """Stream Kokoro MP3 response directly into an audio player."""
        proc = None
        try:
            base_url = f"http://{ip_address}"
            print(f"🎤 [Kokoro] Synthesizing: {text[:80]}… (voice={voice}, speed={speed})")

            # Use tuple timeout: (connect_timeout, read_timeout)
            timeout = (5, 15)
            
            try:
                # ------ request speech from OpenAI-compatible endpoint ------
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
                    timeout=timeout,
                )
            except (requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout):
                print(f"❌ [Kokoro] Request timeout → fallback to local TTS")
                return False
            except requests.exceptions.ConnectionError:
                print(f"❌ [Kokoro] Connection failed → fallback to local TTS")
                return False
            except requests.exceptions.RequestException as e:
                print(f"❌ [Kokoro] Request error: {e} → fallback to local TTS")
                return False

            if response.status_code != 200:
                # Don't try to read full response body (could hang)
                error_msg = f"HTTP {response.status_code}"
                print(f"⚠️  [Kokoro] API error {error_msg}")
                return False

            # ------ choose playback method ------
            player_cmd = _find_player()

            if player_cmd:
                # Stream MP3 data into mpg123 / ffplay via stdin pipe
                try:
                    proc = subprocess.Popen(
                        player_cmd,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                    )
                except Exception as e:
                    print(f"❌ [Kokoro] Failed to start player: {e}")
                    return False
                    
                with self._lock:
                    self._play_proc = proc

                # Stream chunks with timeout handling
                try:
                    for chunk in response.iter_content(chunk_size=4096):
                        if not self._playing:
                            proc.terminate()
                            break
                        if chunk:
                            try:
                                proc.stdin.write(chunk)
                                proc.stdin.flush()
                            except BrokenPipeError:
                                # Player closed stdin (e.g., finished playing)
                                break
                except Exception as e:
                    print(f"⚠️  [Kokoro] Error streaming chunks: {e}")
                    return False
                finally:
                    try:
                        proc.stdin.close()
                    except Exception:
                        pass

                try:
                    proc.wait(timeout=120)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    print(f"⚠️  [Kokoro] Playback timeout")
                    return False
                    
                print("🎤 [Kokoro] ✅ Streaming playback complete")
                return True
            else:
                # Fallback: save to temp file and play with aplay after conversion
                print("🎤 [Kokoro] No streaming player found, using file-based playback")
                tmp_mp3 = '/tmp/kokoro_tts.mp3'
                tmp_wav = '/tmp/kokoro_tts.wav'
                try:
                    with open(tmp_mp3, 'wb') as f:
                        for chunk in response.iter_content(chunk_size=4096):
                            if not self._playing:
                                break
                            if chunk:
                                f.write(chunk)

                    if not self._playing:
                        return False

                    # Convert mp3 → wav with ffmpeg (if available) then play
                    if shutil.which('ffmpeg'):
                        result = subprocess.run(
                            ['ffmpeg', '-y', '-i', tmp_mp3, tmp_wav],
                            capture_output=True, timeout=30,
                        )
                        if result.returncode != 0:
                            print(f"⚠️  [Kokoro] ffmpeg conversion failed")
                            return False
                            
                        proc = subprocess.Popen(
                            ['aplay', '-D', 'default', tmp_wav],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE,
                        )
                        with self._lock:
                            self._play_proc = proc
                        proc.wait(timeout=120)
                        print("🎤 [Kokoro] ✅ File-based playback complete")
                        # Cleanup
                        for f in (tmp_mp3, tmp_wav):
                            try:
                                os.remove(f)
                            except OSError:
                                pass
                        return True
                    else:
                        print("❌ [Kokoro] No suitable audio player (install mpg123 or ffmpeg)")
                        return False
                except subprocess.TimeoutExpired:
                    if proc:
                        proc.kill()
                    print("⚠️  [Kokoro] Conversion/playback timeout")
                    return False
                except Exception as e:
                    print(f"❌ [Kokoro] File-based playback error: {e}")
                    return False

        except Exception as e:
            print(f"❌ [Kokoro] Unexpected error: {e} → fallback to local TTS")
            return False
        finally:
            with self._lock:
                self._playing = False
                self._play_proc = None

    # ------------------------------------------------------------------
    def stop(self):
        """Stop any in-progress playback."""
        with self._lock:
            if self._playing:
                self._playing = False
                # Kill the audio player process if running
                if self._play_proc and self._play_proc.poll() is None:
                    try:
                        self._play_proc.terminate()
                    except Exception:
                        pass
                if self._play_thread:
                    self._play_thread.join(timeout=2.0)
                    self._play_thread = None
                self._play_proc = None
                print("🎤 [Kokoro] Stopped")


# ==========================================
# 🎤 SINGLETON
# ==========================================

_kokoro_client: Optional[KokoroTTSClient] = None


def get_kokoro_client() -> KokoroTTSClient:
    """Get the global Kokoro TTS client instance."""
    global _kokoro_client
    if _kokoro_client is None:
        _kokoro_client = KokoroTTSClient()
    return _kokoro_client
