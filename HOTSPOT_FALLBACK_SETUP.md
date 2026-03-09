# 🌐 Hotspot Mode AI Fallback Configuration

## Overview
When the Raspberry Pi is in **WiFi hotspot mode** (no internet connection), the narration system automatically falls back from Google Gemini to a local Llama LLM to avoid DNS failures.

## Changes Made

### 1. **narration.py** - Core Fallback Logic
Added hotspot detection and local LLM support:

#### New Functions:
- **`is_hotspot_mode()`** - Detects if device is in hotspot mode by checking wlan0 IP range (192.168.4.x or 10.42.0.x)
- **`has_internet_connectivity()`** - Tests internet availability via DNS requests
- **`describe_image_local_llm()`** - Sends image to local Llama LLM using base64 encoding

#### NarrationEngine Updates:
- **`set_local_llm_config(url=None, force_use=False)`** - Configure local LLM endpoint
  - `url`: Custom LLM API URL (default: http://127.0.0.1:8000/v1/chat/completions)
  - `force_use`: Force local LLM even with internet connection

#### Modified Loop Logic in `_loop()`:
```
1. Detect hotspot mode via is_hotspot_mode()
2. IF in hotspot OR forced: Use local LLM only
3. ELSE: Try Gemini first
4. IF Gemini fails: Fall back to local LLM
5. Final fallback: Local TTS (Mimic3/espeak-ng)
```

### 2. **main.py** - Automatic Hotspot Detection
Added automatic configuration on startup:
```python
narration_engine.set_local_llm_config(force_use=get_hotspot_status())
```

This ensures local LLM is always used when in hotspot mode.

## How It Works

### Hotspot Mode Detection
```python
# Checks if wlan0 has hotspot IP range
192.168.4.0/24  ← Standard hotspot range
10.42.0.0/24    ← Alternative hotspot range
```

### Fallback Chain
1. **Primary**: Google Gemini (if internet + API key available)
2. **Secondary**: Local Llama LLM at http://127.0.0.1:8000
3. **Tertiary**: Local TTS (Mimic3/espeak-ng for text-to-speech)

### Error Handling
- **Gemini timeouts** → Automatically try local LLM
- **DNS failures** (like in your logs) → Detected and handled gracefully
- **Local LLM timeout (20s)** → Falls back to next method
- **Empty responses** → Tracks consecutive errors, uses local LLM after failures

## Configuration

### Environment Variables
```bash
# Override local LLM endpoint (default: http://127.0.0.1:8000/v1/chat/completions)
export RC_LOCAL_LLM_URL="http://192.168.1.100:8000/v1/chat/completions"
```

### Runtime Configuration
```python
# Force local LLM always
narration_engine.set_local_llm_config(force_use=True)

# Custom LLM endpoint
narration_engine.set_local_llm_config(
    url="http://custom-llm-server:8000/v1/chat/completions",
    force_use=False  # Only use if hotspot or Gemini fails
)
```

## API Compatibility

### Local LLM Endpoint
Expects OpenAI-compatible API format:
```json
{
    "model": "llama-2-vision",
    "messages": [
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "describe image"},
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}
            ]
        }
    ],
    "temperature": 0.7,
    "max_tokens": 100
}
```

## Logs

### Hotspot Mode Detection
```
🌐 [Narration] Hotspot mode detected or local LLM forced - using local LLM
```

### Fallback Triggered
```
⚠️ [Narration] Gemini failed or returned empty - falling back to local LLM
👁️ [Narration] Local LLM says: <narration text>
```

### Hotspot Status at Startup
```
📡 [Network] Hotspot mode detected
📡 [Network] Using hotspot IP from wlan0: 192.168.4.1
```

## Troubleshooting

### Local LLM Not Responding
```
❌ [Narration] Local LLM request timed out
```
**Solution**: Ensure Llama/local LLM server is running at http://127.0.0.1:8000

### DNS Resolution Errors (Original Issue)
```
❌ DNS resolution failed for generativelanguage.googleapis.com
```
**Now Fixed**: Automatically uses local LLM when in hotspot mode

### Empty Responses
```
⚠️ [Narration] Empty response from both Gemini and local LLM (attempt 1)
```
**Solution**: Check that both APIs are responding, or use forced local LLM mode

## Testing

### Verify Hotspot Detection
```bash
# Check IP configuration
ip addr show wlan0

# Manually test hotspot status
python3 -c "from narration import is_hotspot_mode; print(is_hotspot_mode())"
```

### Test Local LLM Endpoint
```bash
# Make test request
curl -X POST http://127.0.0.1:8000/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "llama-2-vision", "messages": [{"role": "user", "content": "test"}]}'
```

## Performance Impact

- **Hotspot Mode**: Local LLM only (optimized for no-internet scenarios)
- **WiFi Mode**: Gemini preferred, transparent fallback to local LLM if needed
- **Response Time**: Local LLM typically 2-5s vs Gemini 1-3s
- **Bandwidth**: Zero internet bandwidth in hotspot mode

## Future Enhancements

1. ✅ Hotspot detection implemented
2. ✅ Local LLM fallback implemented
3. ⏳ Configurable retry logic
4. ⏳ Performance metrics collection
5. ⏳ Multi-LLM support (Ollama, LMStudio, etc.)
