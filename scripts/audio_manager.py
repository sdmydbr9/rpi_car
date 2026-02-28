import os
import time
import threading

import pygame
from pydub import AudioSegment
from pydub.silence import detect_nonsilent


# ==========================================
# 🛠️ ENGINE TUNING VARIABLES
# ==========================================
IGNITION_CHOP_MS = 1500  
CROSSFADE_MS = 800       
IDLE_OVERLAP_MS = 350    
IDLE_VOLUME_OFFSET_DB = -4.0 

# ACCELERATION & EFFECTS
ACCEL_VOLUME_OFFSET_DB = -2.0  
HORN_VOLUME_OFFSET_DB = 2.0    # Horns should be loud!
BEEPER_VOLUME_OFFSET_DB = 0.0  
THROTTLE_RAMP_UP = 0.04        
THROTTLE_RAMP_DOWN = 0.02      
NARRATION_DUCK_FACTOR = 0.10   # 10% volume while narration/startup speech plays
NARRATION_DUCK_ATTACK_PER_SEC = 2.5   # 1.0 -> 0.1 in ~360ms
NARRATION_DUCK_RELEASE_PER_SEC = 1.8  # 0.1 -> 1.0 in ~500ms
# ==========================================

ENGINE_SAMPLE_RATE = 44100 
AUDIO_BACKEND_FALLBACK_ORDER = ("pulseaudio", "alsa")


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _wait_for_pulse_socket() -> bool:
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    pulse_native = os.path.join(runtime_dir, "pulse", "native")
    wait_s = max(0.0, float(os.environ.get("CAR_AUDIO_PULSE_WAIT_SECS", "12")))
    poll_s = max(0.05, float(os.environ.get("CAR_AUDIO_PULSE_POLL_SECS", "0.25")))

    deadline = time.monotonic() + wait_s
    while True:
        if os.path.exists(pulse_native):
            os.environ.setdefault("XDG_RUNTIME_DIR", runtime_dir)
            os.environ.setdefault("PULSE_SERVER", f"unix:{pulse_native}")
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_s)


def process_track(filename, target_dbfs, target_channels, offset_db=0.0):
    """Helper function to normalize and trim any audio file."""
    base_name = filename.split('.')[0]
    out_file = f"{base_name}_trimmed.wav"
    
    if not os.path.exists(out_file):
        print(f"⚙️  Processing {filename}...")
        try:
            audio = AudioSegment.from_file(filename)
            audio = audio.set_frame_rate(ENGINE_SAMPLE_RATE).set_channels(target_channels)
            
            vol_diff = (target_dbfs - audio.dBFS) + offset_db
            audio = audio + vol_diff

            nonsilent = detect_nonsilent(audio, min_silence_len=50, silence_thresh=-50)
            if nonsilent:
                audio = audio[nonsilent[0][0]:nonsilent[-1][1]]
                
            audio.export(out_file, format="wav")
            print(f"✅ Created: {out_file}")
        except FileNotFoundError:
            print(f"❌ Error: Could not find '{filename}'")
            return False
    return True


def prepare_gapless_audio(sounds_dir):
    """Prepare gapless audio files in specified directory."""
    global ENGINE_SAMPLE_RATE
    
    # Save current directory and change to sounds directory
    original_cwd = os.getcwd()
    os.chdir(sounds_dir)
    
    try:
        ign_path = "ignition.wav"
        if not os.path.exists(ign_path):
            print(f"❌ Error: Could not find '{ign_path}'")
            return False
            
        ign_reference = AudioSegment.from_file(ign_path)
        target_dbfs = ign_reference.dBFS
        ENGINE_SAMPLE_RATE = ign_reference.frame_rate
        target_channels = ign_reference.channels

        # 1. IGNITION (Custom tail chop)
        if not os.path.exists("ignition_trimmed.wav"):
            if len(ign_reference) > IGNITION_CHOP_MS:
                chopped_ign = ign_reference[:-IGNITION_CHOP_MS]
            else:
                chopped_ign = ign_reference
            chopped_ign.export("ignition_trimmed.wav", format="wav")

        # 2. Process all other files dynamically
        process_track("idle.mp3", target_dbfs, target_channels, IDLE_VOLUME_OFFSET_DB)
        process_track("acceleration.mp3", target_dbfs, target_channels, ACCEL_VOLUME_OFFSET_DB)
        process_track("horn.mp3", target_dbfs, target_channels, HORN_VOLUME_OFFSET_DB)
        process_track("beeper.mp3", target_dbfs, target_channels, BEEPER_VOLUME_OFFSET_DB)
        print("")  # Empty line for clean terminal
        return True
    finally:
        os.chdir(original_cwd)


class CarAudioManager:
    """Advanced pygame-based state-driven audio controller for engine, reverse beeper, and horn warning."""

    def __init__(
        self,
        sounds_dir: str,
        audio_device: str = "default",
        idle_start_delay_s: float = 3.5,
        idle_trim_end_s: float = 2.5,
        idle_crossfade_s: float = 0.12,
        idle_handoff_overlap_s: float = 0.18,
    ):
        self._lock = threading.Lock()
        self._sounds_dir = sounds_dir
        self._audio_device = audio_device or "default"
        self._audio_ready = False
        self._active_backend = None
        self._warned_audio_unavailable = False

        # State is initialized before mixer setup so service-mode failures
        # never leave a partially-constructed object.
        self._sounds = {}
        self._ign_channel = None
        self._idle_channel_a = None
        self._idle_channel_b = None
        self._accel_channel = None
        self._horn_channel = None
        self._beeper_channel = None
        
        self._paths = {
            "ignition": os.path.join(sounds_dir, "ignition_trimmed.wav"),
            "idle": os.path.join(sounds_dir, "idle_trimmed.wav"),
            "acceleration": os.path.join(sounds_dir, "acceleration_trimmed.wav"),
            "beeper": os.path.join(sounds_dir, "beeper_trimmed.wav"),
            "horn": os.path.join(sounds_dir, "horn_trimmed.wav"),
        }

        # Prepare audio files
        if not prepare_gapless_audio(sounds_dir):
            print("⚠️  Audio preparation failed")

        self._engine_running = False
        self._accelerating = False
        self._reversing = False
        self._horn_warning = False

        # Timing state
        self._ign_started = False
        self._idle_channel_active = "A"
        self._next_idle_trigger = 0.0
        self._throttle = 0.0
        self._last_gas_time = 0.0
        self._last_reverse_time = 0.0

        # Volume ducking state for narration
        self._narration_ducking = False
        self._duck_factor = 1.0
        self._last_mix_update_ts = time.monotonic()

        if not self._init_mixer_with_fallback():
            print("⚠️  Engine sound is unavailable; continuing without car audio.")
            return

        self._activate_audio_pipeline_locked()

    def is_available(self) -> bool:
        return self._audio_ready

    def backend(self) -> str:
        return self._active_backend or "unavailable"

    def _alsa_device_candidates(self):
        configured_device = (self._audio_device or "").strip()
        if configured_device and configured_device.lower() != "default":
            return [configured_device]

        # Default fallback order prioritizes shared mixers when Shairport is active.
        candidates = [
            "pulse",
            "dmix:CARD=sndrpihifiberry,DEV=0",
            "dmix:CARD=vc4hdmi0,DEV=0",
            "default",
        ]

        extra_candidates = os.environ.get("CAR_AUDIO_DEVICE_FALLBACKS", "")
        if extra_candidates:
            for item in extra_candidates.split(","):
                value = item.strip()
                if value:
                    candidates.insert(0, value)

        deduped = []
        seen = set()
        for candidate in candidates:
            key = candidate.lower()
            if key in seen:
                continue
            seen.add(key)
            deduped.append(candidate)
        return deduped

    def _activate_audio_pipeline_locked(self):
        self._sounds = {}
        self._load_sounds()

        self._ign_channel = pygame.mixer.Channel(0)
        self._idle_channel_a = pygame.mixer.Channel(1)
        self._idle_channel_b = pygame.mixer.Channel(2)
        self._accel_channel = pygame.mixer.Channel(3)
        self._horn_channel = pygame.mixer.Channel(4)
        self._beeper_channel = pygame.mixer.Channel(5)

        if "acceleration" in self._sounds:
            self._accel_channel.play(self._sounds["acceleration"], loops=-1)
            self._accel_channel.set_volume(0.0)

        if "beeper" in self._sounds:
            self._beeper_channel.play(self._sounds["beeper"], loops=-1)
            self._beeper_channel.set_volume(0.0)

        self._audio_ready = True
        self._warned_audio_unavailable = False
        self._last_mix_update_ts = time.monotonic()

    def _init_mixer_with_fallback(self) -> bool:
        preferred_driver = (os.environ.get("SDL_AUDIODRIVER") or "").strip()
        fallback_order = []
        if preferred_driver:
            fallback_order.append(preferred_driver)
        for candidate in AUDIO_BACKEND_FALLBACK_ORDER:
            if candidate not in fallback_order:
                fallback_order.append(candidate)

        # For mixed playback with Shairport Sync, require PulseAudio when asked.
        require_pulse = _env_bool("CAR_AUDIO_REQUIRE_PULSE", False)
        if require_pulse:
            fallback_order = ["pulseaudio"]

        if "pulseaudio" in fallback_order:
            pulse_ready = _wait_for_pulse_socket()
            if not pulse_ready:
                print("⚠️  PulseAudio socket not available before mixer init timeout.")

        self._active_backend = None
        init_errors = []
        for driver in fallback_order:
            try:
                pygame.mixer.quit()
            except Exception:
                pass

            os.environ["SDL_AUDIODRIVER"] = driver

            if driver == "alsa":
                for alsa_device in self._alsa_device_candidates():
                    if alsa_device.lower() == "default":
                        os.environ.pop("AUDIODEV", None)
                    else:
                        os.environ["AUDIODEV"] = alsa_device
                    try:
                        pygame.mixer.init(
                            frequency=ENGINE_SAMPLE_RATE,
                            size=-16,
                            channels=2,
                            buffer=512,
                            devicename=None,
                        )
                        self._active_backend = f"alsa:{alsa_device}"
                        print(
                            f"🔊 Pygame mixer initialized "
                            f"(rate={ENGINE_SAMPLE_RATE}Hz, backend={self._active_backend})"
                        )
                        return True
                    except Exception as exc:
                        init_errors.append(f"{driver} ({alsa_device}): {exc}")
                continue

            os.environ.pop("AUDIODEV", None)
            try:
                pygame.mixer.init(
                    frequency=ENGINE_SAMPLE_RATE,
                    size=-16,
                    channels=2,
                    buffer=512,
                    devicename=None,
                )
                self._active_backend = driver
                print(
                    f"🔊 Pygame mixer initialized "
                    f"(rate={ENGINE_SAMPLE_RATE}Hz, backend={self._active_backend})"
                )
                return True
            except Exception as exc:
                init_errors.append(f"{driver}: {exc}")

        print(
            "⚠️  Failed to initialize pygame mixer on any backend: "
            + " | ".join(init_errors)
        )
        return False

    def _load_sounds(self):
        """Load all sound files into pygame mixer."""
        for name, path in self._paths.items():
            if os.path.exists(path):
                try:
                    self._sounds[name] = pygame.mixer.Sound(path)
                    print(f"✅ Loaded: {name} from {path}")
                except Exception as e:
                    print(f"⚠️  Failed to load {name}: {e}")
            else:
                print(f"⚠️  Sound file not found: {path}")

    def set_engine_running(self, enabled: bool):
        """Start or stop the engine audio sequence."""
        with self._lock:
            enabled = bool(enabled)
            if enabled == self._engine_running:
                return

            self._engine_running = enabled
            if not self._audio_ready:
                if enabled:
                    if self._init_mixer_with_fallback():
                        self._activate_audio_pipeline_locked()
                        print("🔊 Audio backend recovered on engine start.")
                    elif not self._warned_audio_unavailable:
                        print("⚠️  Engine start requested but audio backend is unavailable.")
                        self._warned_audio_unavailable = True
                return

            if enabled:
                self._accelerating = False
                self._reversing = False
                self._horn_warning = False
                self._start_engine_sequence_locked()
            else:
                self._accelerating = False
                self._reversing = False
                self._horn_warning = False
                self._stop_all_locked()

    def update_runtime_state(self, accelerating: bool, reversing: bool, horn_warning: bool):
        """Update audio state based on vehicle state."""
        with self._lock:
            self._accelerating = bool(accelerating)
            self._reversing = bool(reversing)
            self._horn_warning = bool(horn_warning)
            if not self._audio_ready:
                return
            self._update_audio_mix_locked()

    def play_horn(self):
        """Play the horn sound once (stateless, manual trigger)."""
        with self._lock:
            if not self._audio_ready:
                return
            if "horn" in self._sounds:
                # Ensure horn channel volume is set for manual play
                self._horn_channel.set_volume(1.0)
                if not self._horn_channel.get_busy():
                    self._horn_channel.play(self._sounds["horn"])
                    print("🔔 Horn played!")

    def duck_engine_volume(self, enable: bool = True):
        """Reduce engine volume to 10% for narration playback.
        
        Args:
            enable: True to duck (lower) volume, False to restore
        """
        with self._lock:
            if not self._audio_ready:
                return
            if enable and not self._narration_ducking:
                self._narration_ducking = True
                print("🔊 Engine ducking requested (smooth ramp to 10%)")
                self._update_audio_mix_locked()
            
            elif not enable and self._narration_ducking:
                self._narration_ducking = False
                print("🔊 Engine unduck requested (smooth restore)")
                self._update_audio_mix_locked()

    def shutdown(self):
        """Stop all audio and clean up."""
        with self._lock:
            self._engine_running = False
            self._accelerating = False
            self._reversing = False
            self._horn_warning = False
            self._stop_all_locked()
            if not self._audio_ready:
                return
            try:
                pygame.mixer.quit()
            except Exception:
                pass

    def _update_audio_mix_locked(self):
        """Update throttle and audio mixing based on current state."""
        if not self._audio_ready:
            return
        current_time = time.monotonic()
        dt = current_time - self._last_mix_update_ts
        self._last_mix_update_ts = current_time
        # Clamp dt to avoid large one-shot jumps after scheduler stalls.
        dt = max(0.0, min(dt, 0.1))

        target_duck = NARRATION_DUCK_FACTOR if self._narration_ducking else 1.0
        if self._duck_factor > target_duck:
            self._duck_factor = max(target_duck, self._duck_factor - (NARRATION_DUCK_ATTACK_PER_SEC * dt))
        elif self._duck_factor < target_duck:
            self._duck_factor = min(target_duck, self._duck_factor + (NARRATION_DUCK_RELEASE_PER_SEC * dt))

        if not self._engine_running:
            self._duck_factor = 1.0
            self._accel_channel.set_volume(0.0)
            self._idle_channel_a.set_volume(0.0)
            self._idle_channel_b.set_volume(0.0)
            self._beeper_channel.set_volume(0.0)
            self._horn_channel.set_volume(0.0)
            self._ign_channel.set_volume(0.0)
            return

        # --- Twin Turntable Idle Logic ---
        if "idle" in self._sounds and self._ign_started:
            idle_length_sec = self._sounds["idle"].get_length()
            overlap_sec = IDLE_OVERLAP_MS / 1000.0
            if current_time >= self._next_idle_trigger:
                if self._idle_channel_active == "A":
                    self._idle_channel_b.play(self._sounds["idle"], fade_ms=100)
                    self._idle_channel_active = "B"
                else:
                    self._idle_channel_a.play(self._sounds["idle"], fade_ms=100)
                    self._idle_channel_active = "A"
                self._next_idle_trigger = current_time + (idle_length_sec - overlap_sec)

        # --- Throttle Logic ---
        if self._accelerating:
            self._last_gas_time = current_time
        if self._reversing:
            self._last_reverse_time = current_time
            self._last_gas_time = current_time  # Auto-rev when reversing

        # Throttle ramping
        if current_time - self._last_gas_time < 0.25:
            self._throttle = min(1.0, self._throttle + THROTTLE_RAMP_UP)
        else:
            self._throttle = max(0.0, self._throttle - THROTTLE_RAMP_DOWN)

        # --- Beeper Logic (Reverse) ---
        is_reversing = current_time - self._last_reverse_time < 0.25
        if is_reversing:
            self._beeper_channel.set_volume(1.0)
        else:
            self._beeper_channel.set_volume(0.0)

        # --- Audio Ducking Mix ---
        accel_vol = self._throttle * 0.70
        idle_vol = 1.0 - (self._throttle * 0.70)
        self._ign_channel.set_volume(self._duck_factor)
        self._accel_channel.set_volume(accel_vol * self._duck_factor)
        self._idle_channel_a.set_volume(idle_vol * self._duck_factor)
        self._idle_channel_b.set_volume(idle_vol * self._duck_factor)

        # --- Horn Logic (Autopilot Warning) ---
        if self._horn_warning:
            if not self._horn_channel.get_busy() and "horn" in self._sounds:
                self._horn_channel.play(self._sounds["horn"])

    def _start_engine_sequence_locked(self):
        """Start the engine ignition sequence with crossfade to idle."""
        if not self._audio_ready:
            return
        self._throttle = 0.0
        self._last_gas_time = 0.0
        self._last_reverse_time = 0.0

        if "ignition" not in self._sounds:
            print("⚠️  Ignition sound not loaded, playing idle only")
            self._ign_started = True
            if "idle" in self._sounds:
                self._idle_channel_a.play(self._sounds["idle"], loops=-1, fade_ms=CROSSFADE_MS)
                ign_length_sec = 0.0
                idle_length_sec = self._sounds["idle"].get_length()
                overlap_sec = IDLE_OVERLAP_MS / 1000.0
                self._next_idle_trigger = time.monotonic() + (idle_length_sec - overlap_sec)
            return

        # Play ignition
        print("🔊 IGNITION...")
        self._ign_channel.play(self._sounds["ignition"])
        ign_length_sec = self._sounds["ignition"].get_length()

        # Schedule idle crossfade
        crossfade_sec = CROSSFADE_MS / 1000.0
        idle_start_delay = max(0.0, ign_length_sec - crossfade_sec)

        if "idle" in self._sounds:
            idle_length_sec = self._sounds["idle"].get_length()
            overlap_sec = IDLE_OVERLAP_MS / 1000.0

            def start_idle():
                with self._lock:
                    if self._engine_running:
                        print("🔄 Crossfading to idle...")
                        self._ign_channel.fadeout(CROSSFADE_MS)
                        self._idle_channel_a.play(self._sounds["idle"], fade_ms=CROSSFADE_MS, loops=-1)
                        self._ign_started = True
                        self._idle_channel_active = "A"
                        self._next_idle_trigger = time.monotonic() + (idle_length_sec - overlap_sec)

            timer = threading.Timer(idle_start_delay, start_idle)
            timer.daemon = True
            timer.start()

    def _stop_all_locked(self):
        """Stop all audio channels with fadeout."""
        if not self._audio_ready:
            self._ign_started = False
            self._throttle = 0.0
            self._duck_factor = 1.0
            return
        # Fade out all audio like in sound_sequence.py when 'q' is pressed
        try:
            pygame.mixer.fadeout(1000)
        except Exception:
            # Fallback: fade out individual channels if mixer fadeout fails
            if self._ign_channel is not None:
                self._ign_channel.fadeout(200)
            if self._idle_channel_a is not None:
                self._idle_channel_a.fadeout(200)
            if self._idle_channel_b is not None:
                self._idle_channel_b.fadeout(200)
            if self._accel_channel is not None:
                self._accel_channel.set_volume(0.0)
            if self._beeper_channel is not None:
                self._beeper_channel.set_volume(0.0)
            if self._horn_channel is not None:
                self._horn_channel.fadeout(200)
        self._ign_started = False
        self._throttle = 0.0
        self._duck_factor = 1.0
