import os
import shutil
import subprocess
import threading
import time
import wave
from typing import Optional


class CarAudioManager:
    """State-driven audio controller for engine, reverse beeper, and horn warning."""

    def __init__(
        self,
        sounds_dir: str,
        audio_device: str = "default",
        ignition_idle_overlap_s: float = 0.7,
    ):
        self._lock = threading.Lock()
        self._sounds_dir = sounds_dir
        self._audio_device = audio_device or "default"
        self._ignition_idle_overlap_s = max(0.0, ignition_idle_overlap_s)

        self._mpg123_bin = shutil.which("mpg123")
        self._aplay_bin = shutil.which("aplay")

        self._paths = {
            "ignition": os.path.join(sounds_dir, "ignition.wav"),
            "idle": os.path.join(sounds_dir, "idle.mp3"),
            "acceleration": os.path.join(sounds_dir, "acceleration.mp3"),
            "beeper": os.path.join(sounds_dir, "beeper.mp3"),
            "horn": os.path.join(sounds_dir, "horn.mp3"),
        }

        self._ignition_duration_s = self._wav_duration(self._paths["ignition"])

        self._engine_running = False
        self._accelerating = False
        self._reversing = False
        self._horn_warning = False

        self._ignition_proc: Optional[subprocess.Popen] = None
        self._idle_proc: Optional[subprocess.Popen] = None
        self._accel_proc: Optional[subprocess.Popen] = None
        self._beeper_proc: Optional[subprocess.Popen] = None
        self._horn_proc: Optional[subprocess.Popen] = None
        self._idle_timer: Optional[threading.Timer] = None

        # Avoid tight respawn loops if ALSA is busy or a command fails.
        self._last_start_attempt = {
            "idle": 0.0,
            "acceleration": 0.0,
            "beeper": 0.0,
            "horn": 0.0,
        }

        missing = [name for name, path in self._paths.items() if not os.path.exists(path)]
        if missing:
            print(f"[Audio] Missing sound files: {', '.join(missing)}")
        if not self._mpg123_bin:
            print("[Audio] mpg123 not found; MP3 sounds will be disabled.")
        if not self._aplay_bin:
            print("[Audio] aplay not found; ignition WAV sound will be disabled.")

    def set_engine_running(self, enabled: bool):
        with self._lock:
            enabled = bool(enabled)
            if enabled == self._engine_running:
                return

            self._engine_running = enabled
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
        with self._lock:
            self._accelerating = bool(accelerating)
            self._reversing = bool(reversing)
            self._horn_warning = bool(horn_warning)

            if not self._engine_running:
                # Ensure anything left over is stopped even if state updates arrive late.
                self._stop_proc_locked("_accel_proc")
                self._stop_proc_locked("_idle_proc")
                self._stop_proc_locked("_beeper_proc")
                self._stop_proc_locked("_horn_proc")
                self._cancel_idle_timer_locked()
                return

            if self._accelerating:
                self._stop_proc_locked("_idle_proc")
                self._ensure_loop_locked(
                    name="acceleration",
                    proc_attr="_accel_proc",
                    path=self._paths["acceleration"],
                )
            else:
                self._stop_proc_locked("_accel_proc")
                self._ensure_loop_locked(
                    name="idle",
                    proc_attr="_idle_proc",
                    path=self._paths["idle"],
                )

            if self._reversing:
                self._ensure_loop_locked(
                    name="beeper",
                    proc_attr="_beeper_proc",
                    path=self._paths["beeper"],
                )
            else:
                self._stop_proc_locked("_beeper_proc")

            if self._horn_warning:
                self._ensure_loop_locked(
                    name="horn",
                    proc_attr="_horn_proc",
                    path=self._paths["horn"],
                )
            else:
                self._stop_proc_locked("_horn_proc")

    def shutdown(self):
        with self._lock:
            self._engine_running = False
            self._accelerating = False
            self._reversing = False
            self._horn_warning = False
            self._stop_all_locked()

    def _start_engine_sequence_locked(self):
        self._stop_proc_locked("_accel_proc")
        self._stop_proc_locked("_idle_proc")
        self._cancel_idle_timer_locked()

        ignition_path = self._paths["ignition"]
        if self._aplay_bin and os.path.exists(ignition_path):
            cmd = [self._aplay_bin, "-q", "-D", self._audio_device, ignition_path]
            self._ignition_proc = self._spawn(cmd)

            lead_in = max(0.0, self._ignition_duration_s - self._ignition_idle_overlap_s)
            self._idle_timer = threading.Timer(lead_in, self._idle_timer_fired)
            self._idle_timer.daemon = True
            self._idle_timer.start()
        else:
            self._ensure_loop_locked(
                name="idle",
                proc_attr="_idle_proc",
                path=self._paths["idle"],
            )

    def _idle_timer_fired(self):
        with self._lock:
            self._idle_timer = None
            if not self._engine_running or self._accelerating:
                return
            self._ensure_loop_locked(
                name="idle",
                proc_attr="_idle_proc",
                path=self._paths["idle"],
            )

    def _ensure_loop_locked(self, name: str, proc_attr: str, path: str):
        if not self._mpg123_bin or not os.path.exists(path):
            return

        proc: Optional[subprocess.Popen] = getattr(self, proc_attr)
        if proc and proc.poll() is None:
            return
        if proc and proc.poll() is not None:
            setattr(self, proc_attr, None)

        now = time.time()
        last = self._last_start_attempt.get(name, 0.0)
        if now - last < 0.5:
            return
        self._last_start_attempt[name] = now

        cmd = [self._mpg123_bin, "-q"]
        # mpg123 supports explicit device selection via -a.
        if self._audio_device != "default":
            cmd.extend(["-a", self._audio_device])
        cmd.extend(["--loop", "-1", path])
        spawned = self._spawn(cmd)
        setattr(self, proc_attr, spawned)

    def _stop_all_locked(self):
        self._cancel_idle_timer_locked()
        self._stop_proc_locked("_horn_proc")
        self._stop_proc_locked("_beeper_proc")
        self._stop_proc_locked("_accel_proc")
        self._stop_proc_locked("_idle_proc")
        self._stop_proc_locked("_ignition_proc")

    def _cancel_idle_timer_locked(self):
        if self._idle_timer:
            try:
                self._idle_timer.cancel()
            except Exception:
                pass
            self._idle_timer = None

    def _stop_proc_locked(self, proc_attr: str):
        proc: Optional[subprocess.Popen] = getattr(self, proc_attr)
        if not proc:
            return

        setattr(self, proc_attr, None)
        if proc.poll() is not None:
            return

        try:
            proc.terminate()
            proc.wait(timeout=0.5)
        except Exception:
            try:
                proc.kill()
                proc.wait(timeout=0.2)
            except Exception:
                pass

    @staticmethod
    def _spawn(cmd):
        try:
            return subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            print(f"[Audio] Failed to start process: {' '.join(cmd)} ({exc})")
            return None

    @staticmethod
    def _wav_duration(path: str) -> float:
        try:
            with wave.open(path, "rb") as wf:
                frames = wf.getnframes()
                rate = wf.getframerate()
                if rate > 0:
                    return frames / float(rate)
        except Exception:
            pass
        return 0.0
