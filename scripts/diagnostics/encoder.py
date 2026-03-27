from gpiozero import RotaryEncoder
import time

# --- CONFIGURATION ---
# IMPORTANT: Change this to match your specific encoder!
PPR = 20

# How often the screen updates and calculates speed (in seconds)
SAMPLE_TIME = 0.5

# --- INITIALIZE ENCODERS ---
# SWAPPED PINS to reverse the counting direction
# Left Encoder: Phase B to GPIO 27, Phase A to GPIO 17
encoder_left = RotaryEncoder(27, 17, max_steps=0)

# Right Encoder: Phase B to GPIO 23, Phase A to GPIO 22
encoder_right = RotaryEncoder(23, 22, max_steps=0)

print(f"Encoders ready! Assuming {PPR} Pulses Per Revolution.")
print("LEFT: Reading from GPIO 27 and 17 (Reversed).")
print("RIGHT: Reading from GPIO 23 and 22 (Reversed).")
print("Spin the motor shafts forward...")
print("-" * 60)

# Track the previous positions to calculate the change over time
prev_steps_left = 0
prev_steps_right = 0

try:
    while True:
        # Wait for our sample time
        time.sleep(SAMPLE_TIME)

        # Grab the current step counts
        curr_steps_left = encoder_left.steps
        curr_steps_right = encoder_right.steps

        # How many steps happened in the last SAMPLE_TIME?
        delta_left = curr_steps_left - prev_steps_left
        delta_right = curr_steps_right - prev_steps_right

        # --- The RPM Math ---
        rpm_multiplier = 60.0 / (PPR * SAMPLE_TIME) 
        
        rpm_left = delta_left * rpm_multiplier
        rpm_right = delta_right * rpm_multiplier

        # Print the results side-by-side
        print(f"LEFT  -> Pos: {curr_steps_left:5} | RPM: {rpm_left:6.2f}   ||   RIGHT -> Pos: {curr_steps_right:5} | RPM: {rpm_right:6.2f}")

        # Update the previous_steps for the next loop
        prev_steps_left = curr_steps_left
        prev_steps_right = curr_steps_right

except KeyboardInterrupt:
    print("\nExiting...")
    # gpiozero automatically cleans up the GPIO pins when the script stops!
