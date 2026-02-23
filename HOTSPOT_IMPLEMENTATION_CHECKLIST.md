# ✅ HOTSPOT NARRATION IMPLEMENTATION CHECKLIST

## Overview
Verifies that the hotspot narration system is implemented exactly like `auto_voice.py` with proper fallback chain.

---

## 📋 COMPONENT CHECKLIST

### ✅ 1. Local LLM Configuration
**File**: `narration.py` (Lines 26-32)
```python
LOCAL_LLM_URL = os.getenv("RC_LOCAL_LLM_URL", "http://127.0.0.1:8000/v1/chat/completions")
MIMIC3_TTS_URL = os.getenv("RC_MIMIC3_TTS_URL", "http://127.0.0.1:59125/api/tts")
HOTSPOT_AUDIO_DEVICE = os.getenv("RC_HOTSPOT_AUDIO_DEVICE", "plughw:0,0")
```
**Status**: ✅ Configured  
**Match to auto_voice.py**: Yes - Uses same URLs and ALSA device pattern

---

### ✅ 2. Hotspot Detection
**Function**: `is_hotspot_mode()` (Lines 35-50)
- Checks wlan0 IP range: `192.168.4.x` or `10.42.0.x`
- Returns True if in hotspot mode, False otherwise

**Status**: ✅ Implemented  
**Notes**: Used to auto-switch to local LLM + Mimic3 when no internet

---

### ✅ 3. Internet Connectivity Check
**Function**: `has_internet_connectivity()` (Lines 53-61)
- Optional DNS test for explicit internet verification
- Currently not used but available for future enhancements

**Status**: ✅ Implemented  
**Usage**: Can be called for explicit connectivity checks

---

### ✅ 4. Local LLM Integration
**Function**: `describe_image_local_llm()` (Lines 64-118)
- Accepts JPEG bytes (camera frame)
- Encodes as base64 for transmission
- Sends to local LLM endpoint
- Supports model selection (default: `llama-2-vision`)

**Status**: ✅ Implemented  
**Match to auto_voice.py**: Uses same HTTP POST pattern with JSON payload

**Implementation Details**:
```
POST http://127.0.0.1:8000/v1/chat/completions
{
    "model": "llama-2-vision",
    "messages": [{"role": "user", "content": [...image_content...]}],
    "temperature": 0.7,
    "max_tokens": 100
}
```

---

### ✅ 5. Mimic3 TTS Integration (CRITICAL - Matches auto_voice.py EXACTLY)
**Function**: `_play_mimic3_tts()` (Lines 121-171)

**Implementation EXACTLY like auto_voice.py**:
```python
# Step 1: GET request to Mimic3 API (streaming)
r = requests.get(MIMIC3_TTS_URL, params={"text": text}, stream=True, timeout=15)

# Step 2: Pipe directly to aplay ALSA device
aplay = subprocess.Popen(
    ['aplay', '-D', audio_device, '-q'],
    stdin=subprocess.PIPE
)
aplay.communicate(input=r.content)
```

**Status**: ✅ Implemented  
**Match to auto_voice.py**: ✅ EXACT - Uses streaming GET + aplay pattern

---

### ✅ 6. NarrationEngine Configuration
**Class**: `NarrationEngine` (narration.py)

#### Init Variables (Lines 486-497):
- `_local_llm_url`: URL for local LLM endpoint
- `_use_local_llm`: Flag to enable local LLM mode
- `_mimic3_enabled`: Flag to enable Mimic3 TTS for hotspot
- `_mimic3_url`: Mimic3 API endpoint
- `_mimic3_audio_device`: ALSA device for Mimic3 output

**Status**: ✅ All variables added

#### Configuration Method (Lines 565-585):
**Function**: `set_local_llm_config(url=None, force_use=False)`
- Sets local LLM URL
- Enables hotspot mode detection
- **Auto-enables Mimic3 TTS when in hotspot mode** ✅

**Status**: ✅ Implemented with Mimic3 auto-enable

---

### ✅ 7. Main Narration Loop Changes
**Method**: `_loop()` (Lines 690-760)

#### New Logic Flow for Hotspot Mode:
```
1. Detect hotspot mode via is_hotspot_mode()
2. IF hotspot OR forced: Use LOCAL LLM (not Gemini)
3. ELSE: Try Gemini first, fall back to LOCAL LLM if fails
4. IF got text: Play audio via priority chain:
   - PRIMARY: Mimic3 TTS (if hotspot mode enabled) ← NEW EXACT auto_voice.py
   - SECONDARY: Kokoro TTS (online mode)
   - TERTIARY: Local TTS (espeak-ng fallback)
```

**Status**: ✅ Implemented in _loop()

---

### ✅ 8. Audio Playback Priority (HOTSPOT MODE - Matches auto_voice.py)
**File**: `narration.py` (Lines 730-760)

**Playback Chain**:
1. **Mimic3 TTS** (hotspot mode) - Uses `_play_mimic3_tts()`
   - HTTP GET to Mimic3 API
   - Stream to aplay
   - EXACTLY like auto_voice.py
2. **Kokoro TTS** (online mode) - Uses `_play_kokoro_with_volume()`
3. **Local TTS** (fallback) - Uses tts_local (espeak-ng)

**Status**: ✅ Chain properly ordered

---

### ✅ 9. LLM Selection Logic (Gemini vs Local LLM)
**File**: `narration.py` (Lines 705-720)

```python
# If hotspot or forced: Use LOCAL LLM only
if in_hotspot or use_local_llm:
    text = describe_image_local_llm(...)
    
# Else: Try Gemini first
else:
    text = describe_image(api_key, model_name, jpeg_bytes, prompt)
    
    # If Gemini fails: Fall back to LOCAL LLM
    if not text:
        text = describe_image_local_llm(...)
```

**Status**: ✅ Implemented with proper fallback

---

## 🔗 Integration with main.py
**File**: `main.py` (Line 545)
```python
narration_engine.set_local_llm_config(force_use=get_hotspot_status())
```

**Status**: ✅ Auto-detection on startup

---

## 📊 Configuration Summary

### Environment Variables Available:
```bash
# Local LLM endpoint
export RC_LOCAL_LLM_URL="http://127.0.0.1:8000/v1/chat/completions"

# Mimic3 TTS endpoint
export RC_MIMIC3_TTS_URL="http://127.0.0.1:59125/api/tts"

# ALSA audio device for Mimic3
export RC_HOTSPOT_AUDIO_DEVICE="plughw:0,0"
```

### Default Values:
- Local LLM: `http://127.0.0.1:8000/v1/chat/completions`
- Mimic3 TTS: `http://127.0.0.1:59125/api/tts`
- Audio Device: `plughw:0,0` (HiFiBerry DAC)

---

## 🧪 Testing Checklist

### Quick Verification:
```bash
# Run the test script
python3 test_hotspot_narration.py
```

**Expected Output**:
- ✅ Mimic3 TTS: OK
- ✅ Local LLM (text-only): OK
- ✅ Local LLM (vision mode): OK (or appropriate fallback)
- ✅ Hotspot detection: Working
- ✅ NarrationEngine config: OK

---

## 📝 Logs to Expect

### When Hotspot Mode is Detected:
```
🌐 [Narration] Hotspot mode detected or local LLM forced - using local LLM
🎙️ [Narration] Hotspot mode detected - enabling Mimic3 TTS
🧠 [Narration] Local LLM configured - URL: ..., Mode: hotspot
🎙️ [Narration] Mimic3 TTS: enabled
```

### When Narration Occurs (Hotspot Mode):
```
👁️ [Narration] Local LLM says: <narration text>
🎙️ [Narration] Mimic3 enabled (hotspot mode) - using Mimic3 TTS...
🎙️ [Narration] Mimic3: synthesizing...
🎙️ [Narration] Mimic3: playing to plughw:0,0...
✅ [Narration] Mimic3 playback: SUCCESS
```

### When Gemini Fails (Online Mode):
```
❌ Gemini error: 503 DNS resolution failed...
⚠️ [Narration] Gemini failed or returned empty - falling back to local LLM
👁️ [Narration] Local LLM says: <narration text>
🎤 [Narration] Kokoro: IP=..., Voice=...
✅ [Narration] Kokoro playback: SUCCESS
```

---

## ✨ Key Implementation Details - Matches auto_voice.py EXACTLY

### 1. Mimic3 Request Pattern ✅
```python
# EXACT MATCH to auto_voice.py lines 44-47
r = requests.get(TTS_API_URL, params={"text": text}, stream=True)
aplay = subprocess.Popen(['aplay', '-D', AUDIO_DEVICE, '-q'], stdin=subprocess.PIPE)
aplay.communicate(input=r.content)
```

### 2. Local LLM Pattern ✅
Uses HTTP POST with streaming (compatible with ollama/llama.cpp endpoints)

### 3. Error Handling ✅
- Mimic3 connection failure -> Try next method
- Mimic3 timeout -> Try next method
- Local LLM failure -> Try next method
- All failures -> Signal error callback

### 4. Audio Device Management ✅
- Uses ALSA device names (e.g., `plughw:0,0`)
- Configurable via environment variables
- Consistent with HiFiBerry DAC setup

---

## 🎯 Fallback Chain Summary

### ONLINE MODE (With Internet):
```
Camera Frame
    ↓
Try Gemini API
    ├─ SUCCESS → Play via Kokoro TTS
    └─ FAIL → Try Local LLM
              ├─ SUCCESS → Play via Kokoro
              └─ FAIL → Play via espeak-ng (fallback)
```

### HOTSPOT MODE (No Internet):
```
Camera Frame
    ↓
Use Local LLM (skip Gemini)
    ├─ SUCCESS → Play via Mimic3 TTS ← AUTO_VOICE.PY PATTERN
    └─ FAIL → Play via Kokoro or espeak-ng
```

---

## 📌 Summary Status
- ✅ Hotspot detection: READY
- ✅ Local LLM integration: READY
- ✅ Mimic3 TTS (exact auto_voice.py pattern): READY
- ✅ Fallback chain: READY
- ✅ NarrationEngine config: READY
- ✅ main.py integration: READY
- ✅ Error handling: READY
- ✅ Testing script: READY

**Status: FULLY IMPLEMENTED AND READY FOR TESTING** 💗
