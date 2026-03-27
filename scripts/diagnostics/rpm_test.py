#!/usr/bin/env python3
"""
Dual PID RPM Test — Enter a target RPM and the motors will hold that speed
using INDEPENDENT PID loops for the left and right wheels. 
Saves a plot on exit.
"""

import os
import sys
import time
import signal
import matplotlib.pyplot as plt

# Add core/ to path
_diag_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_diag_dir, '..', 'core'))

from pico_sensor_reader import (
    init_pico_reader,
    get_pico_rpm,
    get_pico_motor_duty,
    send_encoder_reset,
    send_stop,
    send_brake,
    is_pico_fresh,
    send_lr_pwm,
)

# ── Config ──
SAMPLE_TIME = 0.05  # 50ms poll interval

# ── Feedforward & Deadband ──
DEADBAND = 15.0  # The minimum PWM needed just to overcome gear friction
Kf = 0.6         # Feedforward gain: estimated extra PWM needed per RPM

# ── PID gains ──
Kp = 0.3
Ki = 0.1
Kd = 0.05

# ── Limits ──
MIN_PWM = 0
MAX_PWM = 100

def generate_plot(log_data):
    """Generates and saves a PNG plot of the run."""
    print("\nGenerating plot in the background... please wait.")

    plt.figure(figsize=(10, 8))

    # Top Subplot: RPMs
    plt.subplot(2, 1, 1)
    plt.plot(log_data['time'], log_data['target'], 'k--', label='Target RPM', linewidth=2)
    plt.plot(log_data['time'], log_data['l_rpm'], label='Left RPM', alpha=0.8, color='blue')
    plt.plot(log_data['time'], log_data['r_rpm'], label='Right RPM', alpha=0.8, color='green')
    plt.title('Independent Motor RPM over Time')
    plt.ylabel('RPM')
    plt.legend()
    plt.grid(True)

    # Bottom Subplot: PWM
    plt.subplot(2, 1, 2)
    plt.plot(log_data['time'], log_data['l_pwm'], label='Left PWM %', color='cyan', linewidth=2)
    plt.plot(log_data['time'], log_data['r_pwm'], label='Right PWM %', color='lightgreen', linewidth=2)
    plt.title('Independent Control Signals (PWM)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('PWM %')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()

    timestamp = int(time.time())
    filename = f"dual_pid_log_{timestamp}.png"
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

    print(f"\nTarget RPM: {target_rpm:.1f}")
    print("Starting motors... Press Ctrl+C to stop.\n")
    print(f"{'Time':>6s} | {'L_RPM':>6s} {'R_RPM':>6s} | {'L_PWM':>5s} {'R_PWM':>5s} | {'PicoL':>5s} {'PicoR':>5s} | {'L_Err':>6s} {'R_Err':>6s}")
    print("-" * 75)

    init_pico_reader()
    time.sleep(0.5)

    if not is_pico_fresh():
        print("Waiting for Pico data...")
        for _ in range(20):
            time.sleep(0.25)
            if is_pico_fresh():
                break
        else:
            print("Failed to receive data from Pico. Check UART connection.")
            return

    send_encoder_reset()
    time.sleep(0.1)

    # Independent State Variables
    integral_l, integral_r = 0.0, 0.0
    prev_error_l, prev_error_r = 0.0, 0.0
    filtered_rpm_l, filtered_rpm_r = 0.0, 0.0
    ALPHA = 0.3

    log_data = {
        'time': [], 'target': [], 
        'l_rpm': [], 'r_rpm': [],
        'l_pwm': [], 'r_pwm': [], 
        'l_error': [], 'r_error': []
    }

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

            # Read RPM
            rpm_l, rpm_r = get_pico_rpm()

            # Filter independently
            filtered_rpm_l = (ALPHA * rpm_l) + ((1.0 - ALPHA) * filtered_rpm_l)
            filtered_rpm_r = (ALPHA * rpm_r) + ((1.0 - ALPHA) * filtered_rpm_r)

            # ── Base Power (Feedforward) ──
            base_pwm = DEADBAND + (target_rpm * Kf) if target_rpm > 0 else 0.0

            # ── PID LEFT ──
            error_l = target_rpm - filtered_rpm_l
            integral_l += error_l * dt
            integral_l = max(-1000, min(1000, integral_l))
            derivative_l = (error_l - prev_error_l) / dt if dt > 0 else 0.0
            prev_error_l = error_l
            
            pwm_l = base_pwm + (Kp * error_l) + (Ki * integral_l) + (Kd * derivative_l)
            floor_l = DEADBAND if target_rpm > 0 else MIN_PWM
            pwm_l = max(floor_l, min(MAX_PWM, pwm_l))
            # Anti-windup: if output is clamped at the floor, don't let integral wind down
            if pwm_l == floor_l and error_l < 0:
                integral_l -= error_l * dt

            # ── PID RIGHT ──
            error_r = target_rpm - filtered_rpm_r
            integral_r += error_r * dt
            integral_r = max(-1000, min(1000, integral_r))
            derivative_r = (error_r - prev_error_r) / dt if dt > 0 else 0.0
            prev_error_r = error_r

            pwm_r = base_pwm + (Kp * error_r) + (Ki * integral_r) + (Kd * derivative_r)
            floor_r = DEADBAND if target_rpm > 0 else MIN_PWM
            pwm_r = max(floor_r, min(MAX_PWM, pwm_r))
            # Anti-windup: if output is clamped at the floor, don't let integral wind down
            if pwm_r == floor_r and error_r < 0:
                integral_r -= error_r * dt

            # Send independent commands
            send_lr_pwm(int(pwm_l), int(pwm_r))

            # Read Pico-reported applied motor duties
            pico_l, pico_r = get_pico_motor_duty()

            # Logging & Printing
            print(f"{elapsed:6.1f} | {rpm_l:6.1f} {rpm_r:6.1f} | {pwm_l:5.1f} {pwm_r:5.1f} | {pico_l:5.1f} {pico_r:5.1f} | {error_l:+6.1f} {error_r:+6.1f}")

            log_data['time'].append(elapsed)
            log_data['target'].append(target_rpm)
            log_data['l_rpm'].append(rpm_l)
            log_data['r_rpm'].append(rpm_r)
            log_data['l_pwm'].append(pwm_l)
            log_data['r_pwm'].append(pwm_r)
            log_data['l_error'].append(error_l)
            log_data['r_error'].append(error_r)

    finally:
        send_lr_pwm(0, 0)
        send_brake()
        time.sleep(0.2)
        send_stop()
        print("\nMotors stopped.")
        if len(log_data['time']) > 0:
            generate_plot(log_data)

if __name__ == "__main__":
    main()
