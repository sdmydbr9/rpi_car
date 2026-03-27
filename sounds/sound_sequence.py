#!/usr/bin/env python3
"""
Interactive Engine Audio Sequencer (The Full Vehicle Edition)
Includes Engine Idle, Dynamic Acceleration, Reverse Beeper, and a Horn.
"""

import os
import sys
import time
import select
import termios
import tty
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
# ==========================================

ENGINE_SAMPLE_RATE = 44100 

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
            exit(1)

def prepare_gapless_audio():
    global ENGINE_SAMPLE_RATE
    
    try:
        ign_reference = AudioSegment.from_file("ignition.wav")
        target_dbfs = ign_reference.dBFS
        ENGINE_SAMPLE_RATE = ign_reference.frame_rate
        target_channels = ign_reference.channels
    except FileNotFoundError:
        print("❌ Error: Could not find 'ignition.wav'")
        exit(1)

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
    print("") # Empty line for clean terminal


def play_engine_sequence():
    pygame.mixer.init(frequency=ENGINE_SAMPLE_RATE, size=-16, channels=2, buffer=512)

    print("📦 Loading processed engine sounds into RAM...")
    ignition_sound = pygame.mixer.Sound("ignition_trimmed.wav")
    idle_sound = pygame.mixer.Sound("idle_trimmed.wav")
    accel_sound = pygame.mixer.Sound("acceleration_trimmed.wav")
    horn_sound = pygame.mixer.Sound("horn_trimmed.wav")
    beeper_sound = pygame.mixer.Sound("beeper_trimmed.wav")

    # We now need 6 software channels!
    ign_channel = pygame.mixer.Channel(0)
    idle_channel_a = pygame.mixer.Channel(1)
    idle_channel_b = pygame.mixer.Channel(2)
    accel_channel = pygame.mixer.Channel(3)
    horn_channel = pygame.mixer.Channel(4)
    beeper_channel = pygame.mixer.Channel(5)

    # Start continuous muted tracks
    accel_channel.play(accel_sound, loops=-1)
    accel_channel.set_volume(0.0)
    
    beeper_channel.play(beeper_sound, loops=-1)
    beeper_channel.set_volume(0.0)

    ign_length_sec = ignition_sound.get_length()
    idle_length_sec = idle_sound.get_length()
    
    crossfade_sec = CROSSFADE_MS / 1000.0
    overlap_sec = IDLE_OVERLAP_MS / 1000.0
    
    print("🔊 IGNITION...")
    ign_channel.play(ignition_sound)
    time.sleep(max(0, ign_length_sec - crossfade_sec))

    print("🔄 Crossfading to idle...")
    idle_channel_a.play(idle_sound, fade_ms=CROSSFADE_MS)
    ign_channel.fadeout(CROSSFADE_MS)

    next_trigger_time = time.monotonic() + idle_length_sec - overlap_sec
    active_turntable = "A"

    print("\n==================================")
    print(" 🏎️  VEHICLE AUDIO SYSTEM ONLINE")
    print(" -> Hold 'S' to drive forward")
    print(" -> Hold 'R' to reverse (beeper + gas)")
    print(" -> Press 'H' to honk the horn")
    print(" -> Press 'Q' to shut down engine")
    print("==================================\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())

    throttle = 0.0
    last_gas_time = 0.0  
    last_reverse_time = 0.0

    try:
        while True:
            current_time = time.monotonic()
            
            # --- 1. TWIN TURNTABLE IDLE LOGIC ---
            if current_time >= next_trigger_time:
                if active_turntable == "A":
                    idle_channel_b.play(idle_sound, fade_ms=100)
                    active_turntable = "B"
                else:
                    idle_channel_a.play(idle_sound, fade_ms=100)
                    active_turntable = "A"
                next_trigger_time += (idle_length_sec - overlap_sec)
                
            # --- 2. KEYBOARD LOGIC ---
            key = None
            while select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                
                if key == 's':
                    last_gas_time = current_time 
                elif key == 'r':
                    last_reverse_time = current_time
                    last_gas_time = current_time # Automatically rev the engine when reversing!
                elif key == 'h':
                    # Only play the horn if it isn't already blasting
                    if not horn_channel.get_busy():
                        horn_channel.play(horn_sound)

            if key == 'q':
                break
                
            # --- 3. DYNAMIC MIXING ---
            
            # Throttle Logic
            if current_time - last_gas_time < 0.25:
                throttle = min(1.0, throttle + THROTTLE_RAMP_UP)
            else:
                throttle = max(0.0, throttle - THROTTLE_RAMP_DOWN)

            # Beeper Logic (Unmute if R was held recently)
            is_reversing = current_time - last_reverse_time < 0.25
            if is_reversing:
                beeper_channel.set_volume(1.0)
            else:
                beeper_channel.set_volume(0.0)

            # Audio Ducking Mix
            accel_vol = throttle * 0.70
            idle_vol = 1.0 - (throttle * 0.70)

            accel_channel.set_volume(accel_vol)
            idle_channel_a.set_volume(idle_vol)
            idle_channel_b.set_volume(idle_vol)
            
            # UI Update
            rev_indicator = "REVERSE BEEP" if is_reversing else "            "
            sys.stdout.write(f"\rRPM: [{'#' * int(throttle * 20):<20}] {int(throttle*100)}%  | {rev_indicator}   ")
            sys.stdout.flush()

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\n🛑 Shutting down engine...")
        pygame.mixer.fadeout(1000)
        time.sleep(1)
        pygame.mixer.quit()
        print("Engine off.")


if __name__ == "__main__":
    prepare_gapless_audio()
    play_engine_sequence()
