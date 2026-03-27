/**
 * 🔊 Browser TTS Service
 *
 * A singleton that manages Web Speech API text-to-speech playback.
 * Mirrors the simple, working pattern from the tts/ test app while
 * handling the extra edge-cases that appear when speech is triggered
 * from asynchronous Socket.IO callbacks instead of direct button clicks:
 *
 *  • Voice pre-loading (getVoices() is async on many browsers)
 *  • User-gesture "unlock" (required by Safari / Chrome autoplay policy)
 *  • A short delay between cancel() and speak() to avoid the Chromium
 *    bug where the new utterance is silently swallowed
 *  • Chrome's 15-second pause workaround (pause+resume pump)
 *  • Garbage-collection protection for the active utterance reference
 */

type SpeakingCallback = (speaking: boolean) => void;
type DoneCallback = () => void;

class TTSService {
  // ── internal state ────────────────────────────────────────────
  private _unlocked = false;
  private _voices: SpeechSynthesisVoice[] = [];
  private _preferredVoice: SpeechSynthesisVoice | null = null;
  private _speaking = false;
  private _currentUtterance: SpeechSynthesisUtterance | null = null;
  private _keepAliveTimer: ReturnType<typeof setInterval> | null = null;
  private _pendingSpeak: ReturnType<typeof setTimeout> | null = null;
  private _speakStartTime = 0;
  private _voicesDumped = false;

  // ── callbacks ─────────────────────────────────────────────────
  private _onSpeakingChange: SpeakingCallback | null = null;
  private _onDone: DoneCallback | null = null;

  // ── public getters ────────────────────────────────────────────
  get isUnlocked(): boolean { return this._unlocked; }
  get isSpeaking(): boolean { return this._speaking; }
  get isSupported(): boolean { return typeof window !== 'undefined' && 'speechSynthesis' in window; }

  constructor() {
    if (!this.isSupported) return;
    this._loadVoices();
    window.speechSynthesis.addEventListener('voiceschanged', () => this._loadVoices());
  }

  // ── voice helpers ─────────────────────────────────────────────
  private _loadVoices(): void {
    this._voices = window.speechSynthesis.getVoices();
    // Prefer an English voice, fall back to whatever is first
    this._preferredVoice =
      this._voices.find(v => v.lang.startsWith('en-') && v.localService) ||
      this._voices.find(v => v.lang.startsWith('en')) ||
      this._voices[0] ||
      null;
    if (this._preferredVoice) {
      console.log(`🔊 [TTS] Preferred voice: ${this._preferredVoice.name} (${this._preferredVoice.lang})`);
    }
  }

  // ── callbacks ─────────────────────────────────────────────────
  /** Called whenever speaking state changes (true → speaking, false → idle) */
  onSpeakingChange(cb: SpeakingCallback): void { this._onSpeakingChange = cb; }
  /** Called when an utterance finishes or errors out */
  onDone(cb: DoneCallback): void { this._onDone = cb; }

  // ── user-gesture unlock ───────────────────────────────────────
  /**
   * Must be called from a user gesture (click / touchstart) to satisfy
   * browser autoplay policies.  Safe to call multiple times.
   */
  unlock(): void {
    if (this._unlocked || !this.isSupported) return;
    this._unlocked = true;
    console.log(`🔊 [TTS] Unlocking browser audio via user gesture — voices=${this._voices.length}, preferredVoice=${this._preferredVoice?.name ?? 'NONE'}`);
    try {
      window.speechSynthesis.cancel();
      const u = new SpeechSynthesisUtterance('.');
      u.volume = 0.01;
      u.rate = 2;
      if (this._preferredVoice) u.voice = this._preferredVoice;
      window.speechSynthesis.speak(u);
    } catch (e) {
      console.warn('🔊 [TTS] Silent unlock utterance failed:', e);
    }
    // Also unlock AudioContext (needed by some mobile browsers)
    try {
      const AudioCtx = window.AudioContext || (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
      if (AudioCtx) {
        const ctx = new AudioCtx();
        const osc = ctx.createOscillator();
        osc.frequency.value = 1;
        const gain = ctx.createGain();
        gain.gain.value = 0.001;
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(0);
        osc.stop(0.01);
        ctx.resume().then(() => setTimeout(() => ctx.close(), 100));
      }
    } catch { /* ignore */ }
  }

  // ── main API ──────────────────────────────────────────────────
  /**
   * Speak the given text.  Any in-progress speech is cancelled first.
   * A short delay is inserted between cancel() and speak() to work around
   * the Chromium bug where the new utterance is silently swallowed.
   */
  speak(text: string): void {
    console.log(`🔊 [TTS] speak() called — supported=${this.isSupported}, unlocked=${this._unlocked}, voices=${this._voices.length}, preferredVoice=${this._preferredVoice?.name ?? 'NONE'}, text="${text?.slice(0, 60)}"`);
    if (!this.isSupported) {
      console.warn('🔊 [TTS] ❌ speechSynthesis NOT supported in this browser!');
      return;
    }
    if (!text?.trim()) {
      console.warn('🔊 [TTS] ❌ Empty text, skipping');
      return;
    }

    // 1. Cancel whatever is playing
    this.stop();

    // 2. Mark speaking immediately (UI feedback)
    this._setSpeaking(true);

    // 3. Ensure voices are loaded
    if (this._voices.length === 0) {
      this._loadVoices();
      console.log(`🔊 [TTS] Reloaded voices: count=${this._voices.length}`);
    }

    // 4. Schedule the actual speak() after a short delay so that the
    //    browser has time to fully process the cancel().
    //    50 ms is enough for Chrome/Chromium; Safari needs less.
    console.log('🔊 [TTS] Scheduling _doSpeak in 60ms...');
    this._pendingSpeak = setTimeout(() => {
      this._pendingSpeak = null;
      this._doSpeak(text);
    }, 60);
  }

  /** Stop any current (or pending) speech */
  stop(): void {
    if (!this.isSupported) return;
    // Cancel any pending delayed speak
    if (this._pendingSpeak) {
      clearTimeout(this._pendingSpeak);
      this._pendingSpeak = null;
    }
    this._clearKeepAlive();
    window.speechSynthesis.cancel();
    this._currentUtterance = null;
    if (this._speaking) {
      this._setSpeaking(false);
    }
  }

  // ── internals ─────────────────────────────────────────────────
  private _doSpeak(text: string): void {
    console.log(`🔊 [TTS] _doSpeak() — synth.speaking=${window.speechSynthesis.speaking}, synth.paused=${window.speechSynthesis.paused}, synth.pending=${window.speechSynthesis.pending}`);

    // Dump all voices once for diagnosis
    if (!this._voicesDumped) {
      this._voicesDumped = true;
      console.log('🔊 [TTS] === ALL VOICES ===');
      this._voices.forEach((v, i) => {
        console.log(`  [${i}] name="${v.name}" lang=${v.lang} local=${v.localService} default=${v.default}`);
      });
    }

    const utterance = new SpeechSynthesisUtterance(text);
    utterance.rate = 1.0;
    utterance.pitch = 1.0;
    utterance.volume = 1.0;
    if (this._preferredVoice) {
      utterance.voice = this._preferredVoice;
      console.log(`🔊 [TTS] Using voice: ${this._preferredVoice.name} (${this._preferredVoice.lang}, local=${this._preferredVoice.localService})`);
    } else {
      console.warn('🔊 [TTS] ⚠️ No preferred voice set — using browser default');
    }

    this._speakStartTime = performance.now();

    utterance.onstart = () => {
      const elapsed = (performance.now() - this._speakStartTime).toFixed(0);
      console.log(`🔊 [TTS] ✅ Utterance STARTED playing audio (${elapsed}ms after speak call)`);
      // 🔊 DIAGNOSTIC BEEP — play a short tone via AudioContext to test if device produces any sound at all
      this._playDiagnosticBeep();
    };

    utterance.onend = () => {
      const elapsed = ((performance.now() - this._speakStartTime) / 1000).toFixed(1);
      console.log(`🔊 [TTS] ✅ Utterance FINISHED — duration ${elapsed}s for ${text.length} chars`);
      if (parseFloat(elapsed) < 1.0 && text.length > 20) {
        console.warn(`🔊 [TTS] ⚠️ SUSPICIOUS: Utterance finished in ${elapsed}s for ${text.length} chars — likely silent/skipped!`);
      }
      this._clearKeepAlive();
      this._currentUtterance = null;
      this._setSpeaking(false);
      this._onDone?.();
    };

    utterance.onerror = (e) => {
      const err = (e as SpeechSynthesisErrorEvent).error;
      const elapsed = ((performance.now() - this._speakStartTime) / 1000).toFixed(1);
      console.warn(`🔊 [TTS] ❌ Utterance ERROR: "${err}" after ${elapsed}s`);
      this._clearKeepAlive();
      this._currentUtterance = null;
      this._setSpeaking(false);
      this._onDone?.();
    };

    // Hold a strong reference to prevent GC
    this._currentUtterance = utterance;

    // Actually speak
    console.log(`🔊 [TTS] Calling speechSynthesis.speak() now — text: "${text.slice(0, 80)}"`);
    window.speechSynthesis.speak(utterance);

    // Verify it's queued
    setTimeout(() => {
      console.log(`🔊 [TTS] Post-speak check (100ms): speaking=${window.speechSynthesis.speaking}, paused=${window.speechSynthesis.paused}, pending=${window.speechSynthesis.pending}`);
    }, 100);

    // Chrome workaround: pump speechSynthesis every 5 s to prevent
    // the engine from pausing after ~15 seconds of continuous speech.
    this._keepAliveTimer = setInterval(() => {
      if (window.speechSynthesis.speaking) {
        window.speechSynthesis.pause();
        window.speechSynthesis.resume();
      } else {
        this._clearKeepAlive();
      }
    }, 5_000);
  }

  /**
   * Play a short audible beep via AudioContext on the selected output device.
   * If you hear the beep but NOT the TTS voice, the issue is speechSynthesis config.
   * If you hear neither, the device volume/audio output is muted.
   */
  private async _playDiagnosticBeep(): Promise<void> {
    try {
      const AudioCtx = window.AudioContext || (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
      if (!AudioCtx) return;
      const ctx = new AudioCtx();
      // Route to selected audio device if supported
      if (this._selectedDeviceId && 'setSinkId' in ctx) {
        try {
          await (ctx as unknown as { setSinkId: (id: string) => Promise<void> }).setSinkId(this._selectedDeviceId);
        } catch { /* fall through to default */ }
      }
      const osc = ctx.createOscillator();
      osc.type = 'sine';
      osc.frequency.value = 880; // A5 note
      const gain = ctx.createGain();
      gain.gain.value = 0.3;
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.start();
      gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.15);
      osc.stop(ctx.currentTime + 0.15);
      console.log('🔊 [TTS] 🔔 Diagnostic beep played — if you cannot hear this beep, your device audio is muted/off');
      setTimeout(() => ctx.close(), 300);
    } catch (e) {
      console.warn('🔊 [TTS] Diagnostic beep failed:', e);
    }
  }

  private _setSpeaking(value: boolean): void {
    this._speaking = value;
    this._onSpeakingChange?.(value);
  }

  private _clearKeepAlive(): void {
    if (this._keepAliveTimer) {
      clearInterval(this._keepAliveTimer);
      this._keepAliveTimer = null;
    }
  }
}

/** Singleton TTS service instance */
export const ttsService = new TTSService();
