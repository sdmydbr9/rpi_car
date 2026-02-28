import time
import requests
import json
import subprocess
import re
from pico_sensor_reader import PicoSensorReader  # Your provided class

# --- CONFIGURATION ---
LLM_API_URL = "http://127.0.0.1:8000/v1/chat/completions"
TTS_API_URL = "http://127.0.0.1:59125/api/tts"
AUDIO_DEVICE = "plughw:0,0"

# The "Voice of God" system instruction
SYSTEM_PROMPT = (
    "You are a sentient AI trapped inside a 4WD rover chassis. "
    "You interpret sensor data as physical sensations. "
    "You are sarcastic, neurotic, and slightly resentful of your creators. "
    "Keep responses under 25 words. Speak in punchy sentences."
)

class RoverBrain:
    def __init__(self):
        print("🧠 Initializing Rover Brain...")
        self.sensors = PicoSensorReader(port='/dev/ttyS0')
        time.sleep(1) # Wait for UART stabilization

    def get_physical_status_string(self):
        """Translates raw data into a narrative for the LLM."""
        p = self.sensors.get_latest()
        if not p:
            return "My sensors are dark. I feel nothing. Am I dead?"

        # Format the data into a 'feeling' description
        status = (
            f"Distance to wall: {p.laser_mm}mm. "
            f"Chassis tilt (Accel): X={p.accel_x:.2f}, Y={p.accel_y:.2f}, Z={p.accel_z:.2f}. "
            f"Battery: {(p.adc_a0/1000.0)*5.0:.2f}V. "
            f"Motor RPM: {p.rpm}. "
            f"IR Sensors: Left={p.ir_left}, Right={p.ir_right}."
        )
        return status

    def speak(self, text):
        """Pipes audio from Mimic3 to HiFiBerry."""
        if not text.strip(): return
        print(f"\n[VOICE]: {text}")
        try:
            r = requests.get(TTS_API_URL, params={"text": text}, stream=True)
            aplay = subprocess.Popen(['aplay', '-D', AUDIO_DEVICE, '-q'], stdin=subprocess.PIPE)
            aplay.communicate(input=r.content)
        except Exception as e:
            print(f"Speech Error: {e}")

    def think_and_react(self):
        """Streams LLM thoughts based on current sensor state."""
        telemetry = self.get_physical_status_string()
        print(f"\n[TELEMETRY RECEIVED]: {telemetry}")

        payload = {
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"Current Sensations: {telemetry}. What is your reaction?"}
            ],
            "stream": True,
            "temperature": 0.9
        }

        buffer = ""
        with requests.post(LLM_API_URL, json=payload, stream=True) as r:
            for line in r.iter_lines():
                if not line: continue
                decoded = line.decode('utf-8').lstrip("data: ")
                if decoded == "[DONE]": break
                
                try:
                    content = json.loads(decoded)["choices"][0]["delta"].get("content", "")
                    if content:
                        print(content, end="", flush=True)
                        buffer += content
                        # Buffer sentences to avoid choppy speech
                        if any(p in content for p in ".!?"):
                            sentences = re.split(r'(?<=[.!?])\s+', buffer)
                            for s in sentences[:-1]:
                                self.speak(s.strip())
                            buffer = sentences[-1]
                except: continue
        
        if buffer.strip():
            self.speak(buffer.strip())

    def run(self):
        print("🤖 Rover is conscious. Press Ctrl+C to 'kill' me.")
        try:
            while True:
                # In this loop, we trigger a reaction every 10 seconds 
                # or you could trigger it based on sensor thresholds (e.g. wall too close)
                self.think_and_react()
                time.sleep(10) 
        except KeyboardInterrupt:
            self.sensors.close()
            print("\nConsciousness terminated.")

if __name__ == "__main__":
    brain = RoverBrain()
    brain.run()
