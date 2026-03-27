# 🔊 Local TTS Setup for Raspberry Pi Headphone Jack

## What's New

The robot can now narrate AI descriptions directly through the Raspberry Pi's 3.5mm headphone jack, instead of only through the browser's speakers. This is useful when:
- Running the remote client on a device without speakers
- You want the robot to "speak" autonomously regardless of client device
- The Pi is connected to a speaker/audio system via the headphone jack

## Hardware Requirements

- **Raspberry Pi** with 3.5mm headphone jack (all Pi models)
- **Speaker or headphones** connected to the 3.5mm jack
- **USB power** to the Pi (TTS synthesis uses CPU)

## Software Setup

### 1. Install espeak-ng (Recommended)

Espeak-ng is usually pre-installed on Raspberry Pi OS. To verify/install:

```bash
sudo apt update
sudo apt install espeak-ng
```

### 2. Install pyttsx3 (Optional Fallback)

```bash
pip install pyttsx3
```

The system tries pyttsx3 first, then falls back to espeak-ng.

### 3. Update Python Dependencies

```bash
pip install -r requirements.txt
```

This installs pyttsx3>=2.90

### 4. Check ALSA Audio Configuration

To ensure the headphone jack is the default output:

```bash
# List available audio devices
aplay -l

# You should see: card 0: Headphones [bcm2835 Headphones]
# This is the 3.5mm headphone jack

# Test audio playback
speaker-test -t sine -f 440 -l 1
```

If audio doesn't play, check volume levels:

```bash
amixer sset PCM 100% unmute
```

## Usage

### Browser Settings

1. Open the **Settings** dialog (gear icon in header)
2. Scroll to **🎙️ AI NARRATION** section
3. Validate API key and select a model
4. Enable **Analyse Image** 
5. Scroll to **Pi Headphone Output** under the narration section
6. Toggle **ON** to enable local TTS playback on the Pi

Now when the AI narrates, the text will be synthesized and played through the Pi's headphone jack speaker, **in addition to** the browser TTS (if enabled).

### Testing

Run the test script to verify TTS works:

```bash
python3 test_tts.py
```

You should hear three test phrases through the headphone jack speaker.

## Backend API

When `narration_local_tts_toggle` is enabled on the client, the server automatically:
1. Receives the toggle command via Socket.IO
2. Calls `narration_engine.set_local_tts_enabled(True)`
3. On each narration, synthesizes and plays the text on the Pi

## Troubleshooting

### No Audio Output

1. **Check ALSA levels:**
   ```bash
   amixer sget PCM
   ```
   Should show `[100%] [on]`

2. **Check if espeak-ng is available:**
   ```bash
   which espeak-ng
   ```

3. **Test espeak directly:**
   ```bash
   espeak-ng "Hello world" -d hw:0,0
   ```
   (Note: `-d hw:0,0` routes to card 0 device 0 = headphone jack)

4. **Check Pi logs:**
   ```bash
   journalctl -u rpi-car -n 50 -f
   ```
   Look for `🔊 [TTS]` messages

### Slow Speech Synthesis

- TTS synthesis takes CPU time. If the Pi is busy with vision processing, synthesis will be delayed.
- You can reduce narration interval to allow more time between analyses.

### Switch Between Audio Outputs

The system always tries to use the headphone jack (`hw:0,0`). To switch to HDMI, edit `tts_local.py`:

```python
# Change this line in _speak_espeak():
'-d', 'hw:0,0',   # headphone jack
# To:
'-d', 'hw:1,0',   # HDMI 0
```

## Performance Notes

- **pyttsx3** uses system TTS engines (faster on Pi with hardware acceleration)
- **espeak-ng** is lightweight but uses more CPU
- Both are non-blocking (synthesis runs in background thread)
- Max 1 utterance plays at a time (newer speech interrupts previous)

