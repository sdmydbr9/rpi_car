#!/bin/bash
# Raspberry Pi Car Control Server Startup Script
set -e

cd /home/pi/rpi_car

# Activate virtual environment if present
if [ -f venv/bin/activate ]; then
  # shellcheck source=/dev/null
  source venv/bin/activate
fi

# Ensure service-mode process can discover the user PulseAudio socket.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
PULSE_NATIVE="${XDG_RUNTIME_DIR}/pulse/native"
for _ in $(seq 1 20); do
  if [ -S "$PULSE_NATIVE" ]; then
    export PULSE_SERVER="unix:${PULSE_NATIVE}"
    break
  fi
  sleep 0.5
done

# Prefer PulseAudio for shared playback with shairport-sync.
export CAR_AUDIO_DRIVER="${CAR_AUDIO_DRIVER:-pulseaudio}"
export CAR_AUDIO_REQUIRE_PULSE="${CAR_AUDIO_REQUIRE_PULSE:-1}"

exec python3 main.py
