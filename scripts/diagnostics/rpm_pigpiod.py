#!/usr/bin/env python3
"""
RPM Test — Enter a target RPM and the motors will hold that speed
using a simple PID loop with encoder feedback. Saves a plot on exit.

Uses lgpio (kernel character device interface) for reliable GPIO edge
detection on modern kernels (6.x+). pigpio's DMA-based sampling is
broken on kernel 6.12+ for the Pi 4.

Encoders: Left (GPIO 27/17), Right (GPIO 23/22)
Motors:   Controlled via Pico UART (pico_sensor_reader)
"""

import os
import sys
import time
import signal
import matplotlib.pyplot as plt
import lgpio

# Add core/ to path
_diag_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_diag_dir, '..', 'core'))

from pico_sensor_reader import (
    init_pico_reader,
    send_motor_command,
    send_stop,
)
from lgpio_encoder import LgpioEncoder

# ── Encoder config ──
# gpiozero counts 1 step per full quadrature cycle (4 edges).
# Our state machine counts every edge, so multiply by 4.
PPR = 330 * 4
SAMPLE_TIME = 0.05  

# ── Steering (straight ahead) ──
STEER_CENTER = 1440

# ── Feedforward & Deadband ──
DEADBAND = 15.0  # The minimum PWM needed just to overcome gear friction
Kf = 0.6         # Feedforward gain: estimated extra PWM needed per RPM

# ── PID gains ──
Kp = 0.3
Ki = 0.1
Kd = 0.0

# ── Limits ──
MIN_PWM = 0
MAX_PWM = 100  


def compute_rpm(delta_steps, dt):
    """Convert encoder step delta to RPM."""
    if dt <= 0:
        return 0.0
    revolutions = delta_steps / PPR
    return (revolutions / dt) * 60.0


def generate_plot(log_data):
    """Generates and saves a PNG plot of the run."""
    print("\nGenerating plot in the background... please wait.")
    
    plt.figure(figsize=(10, 8))

    # Top Subplot: RPMs
    plt.subplot(2, 1, 1)
    plt.plot(log_data['time'], log_data['target'], 'k--', label='Target RPM', linewidth=2)
    plt.plot(log_data['time'], log_data['l_rpm'], label='Left RPM', alpha=0.8)
    plt.plot(log_data['time'], log_data['r_rpm'], label='Right RPM', alpha=0.8)
    plt.plot(log_data['time'], log_data['avg_rpm'], label='Filtered Avg RPM', linewidth=2)
    plt.title('Motor RPM over Time')
    plt.ylabel('RPM')
    plt.legend()
    plt.grid(True)

    # Bottom Subplot: PWM and Error
    plt.subplot(2, 1, 2)
    plt.plot(log_data['time'], log_data['pwm'], label='PWM %', color='orange', linewidth=2)
    plt.plot(log_data['time'], log_data['error'], label='Error', color='red', alpha=0.6)
    plt.title('Control Signals')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Value')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    
    timestamp = int(time.time())
    filename = f"pid_log_{timestamp}.png"
    plt.savefig(filename)
    print(f"Plot successfully saved as: {filename}\n")


def main():
    try:
        target_rpm = float(input("Enter target RPM: "))
    except (ValueError, EOFError):
        print("Invalid input.")
        return

    if target_rpm <= 0:
        print("RPM must be positive.")
        return

    if target_rpm > 110:
        print("Warning: Target exceeds the physical 110 RPM limit of the motors.")

    # ── Initialize lgpio ──
    try:
        chip = lgpio.gpiochip_open(0)
    except lgpio.error as e:
        print(f"Error: Could not open GPIO chip: {e}")
        return

    print(f"\nTarget RPM: {target_rpm:.1f}")
    print("Starting motors... Press Ctrl+C to stop.\n")
    print(f"{'Time':>6s}  {'L_RPM':>7s}  {'R_RPM':>7s}  {'Avg_RPM':>8s}  {'PWM%':>5s}  {'Error':>7s}")
    print("-" * 52)

    init_pico_reader()
    time.sleep(0.5)  

    # Instantiate the state machine encoder classes
    encoder_left = LgpioEncoder(chip, 27, 17)
    encoder_right = LgpioEncoder(chip, 23, 22)

    integral = 0.0
    prev_error = 0.0
    filtered_rpm = 0.0
    ALPHA = 0.3

    log_data = {
        'time': [], 'l_rpm': [], 'r_rpm': [], 
        'avg_rpm': [], 'target': [], 'pwm': [], 'error': []
    }

    prev_steps_l = encoder_left.steps
    prev_steps_r = encoder_right.steps
    t_start = time.monotonic()
    t_prev = t_start

    running = True

    def stop_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    try:
        while running:
            time.sleep(SAMPLE_TIME)
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now
            elapsed = t_now - t_start

            curr_l = encoder_left.steps
            curr_r = encoder_right.steps
            delta_l = curr_l - prev_steps_l
            delta_r = curr_r - prev_steps_r
            prev_steps_l = curr_l
            prev_steps_r = curr_r

            rpm_l = abs(compute_rpm(delta_l, dt))
            rpm_r = abs(compute_rpm(delta_r, dt))
            raw_avg_rpm = (rpm_l + rpm_r) / 2.0

            filtered_rpm = (ALPHA * raw_avg_rpm) + ((1.0 - ALPHA) * filtered_rpm)

            # ── Base Power (Feedforward) ──
            if target_rpm > 0:
                base_pwm = DEADBAND + (target_rpm * Kf)
            else:
                base_pwm = 0.0

            # ── PID ──
            error = target_rpm - filtered_rpm
            integral += error * dt
            integral = max(-1000, min(1000, integral))
            derivative = (error - prev_error) / dt if dt > 0 else 0.0
            prev_error = error

            # Add PID corrections to our base power prediction
            pwm_out = base_pwm + (Kp * error) + (Ki * integral) + (Kd * derivative)
            pwm_out = max(MIN_PWM, min(MAX_PWM, pwm_out))

            send_motor_command(int(pwm_out), STEER_CENTER)

            print(f"{elapsed:6.1f}  {rpm_l:7.1f}  {rpm_r:7.1f}  {filtered_rpm:8.1f}  {pwm_out:5.1f}  {error:+7.1f}")

            log_data['time'].append(elapsed)
            log_data['l_rpm'].append(rpm_l)
            log_data['r_rpm'].append(rpm_r)
            log_data['avg_rpm'].append(filtered_rpm)
            log_data['target'].append(target_rpm)
            log_data['pwm'].append(pwm_out)
            log_data['error'].append(error)

    finally:
        # Graceful hardware and motor shutdown
        send_stop()
        encoder_left.cancel()
        encoder_right.cancel()
        lgpio.gpiochip_close(chip)
        print("\nMotors stopped. Hardware callbacks cleaned up.")
        
        if len(log_data['time']) > 0:
            generate_plot(log_data)

if __name__ == "__main__":
    main()
