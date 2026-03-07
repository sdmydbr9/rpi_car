#!/usr/bin/env python3
"""
Plot motor synchronization logs from CSV (Updated for 4WD PI Control).
Requires pandas and matplotlib:
pip install pandas matplotlib
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_log(csv_filename):
    print(f"Loading data from {csv_filename}...")
    try:
        df = pd.read_csv(csv_filename)
    except FileNotFoundError:
        print(f"Error: Could not find file '{csv_filename}'")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    # Convert absolute timestamp to relative time (starting at T=0)
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
    
    # Auto-detect if this is a 4WD log or an older 2WD log
    is_4wd = 'fr_rpm' in df.columns

    # Make the plot slightly taller to accommodate extra data lines
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    title_prefix = "4WD " if is_4wd else ""
    fig.suptitle(f'{title_prefix}Motor PI Synchronization Analysis\n{csv_filename}', fontsize=16)

    # --- Subplot 1: Wheel Speeds (RPM) ---
    ax1.plot(df['time_sec'], df['rl_rpm'], label='RL RPM (Master)', color='blue', linewidth=2.5)
    ax1.plot(df['time_sec'], df['rr_rpm'], label='RR RPM (Slave 1)', color='orange', linewidth=2, alpha=0.8)
    if is_4wd:
        ax1.plot(df['time_sec'], df['fr_rpm'], label='FR RPM (Slave 2)', color='green', linewidth=2, alpha=0.8)
        
    ax1.set_ylabel('Speed (RPM)')
    ax1.set_title('Wheel Speeds')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper right')

    # --- Subplot 2: Control Loop Effort (Final PWM) ---
    ax2.plot(df['time_sec'], df['target_speed_pwm'], label='Target Base PWM', color='black', linestyle='--', linewidth=2)
    
    if is_4wd:
        ax2.plot(df['time_sec'], df['final_rr_pwm'], label='Final RR PWM', color='red', alpha=0.8)
        ax2.plot(df['time_sec'], df['final_fr_pwm'], label='Final FR PWM', color='purple', alpha=0.8)
    else:
        ax2.plot(df['time_sec'], df['final_rr_pwm'], label='Final RR PWM', color='red', alpha=0.8)
        # Handle older adjustment column names
        adj_col = 'total_adj' if 'total_adj' in df.columns else 'rr_pwm_adj'
        if adj_col in df.columns:
            ax2.plot(df['time_sec'], df[adj_col], label=f'PI Adjustment ({adj_col})', color='green', alpha=0.5)

    ax2.set_ylabel('Power / PWM (%)')
    ax2.set_title('Control Loop Effort (Final Applied Power)')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper right')

    # --- Subplot 3: Synchronization Error ---
    ax3.axhline(0, color='black', linewidth=1.5, linestyle='--')
    
    # Calculate error dynamically since our new script doesn't save it directly
    rr_error = df['rl_rpm'] - df['rr_rpm']
    ax3.plot(df['time_sec'], rr_error, label='RR Error (RL - RR)', color='red', alpha=0.8)
    
    if is_4wd:
        fr_error = df['rl_rpm'] - df['fr_rpm']
        ax3.plot(df['time_sec'], fr_error, label='FR Error (RL - FR)', color='purple', alpha=0.8)

    ax3.set_xlabel('Time (Seconds)')
    ax3.set_ylabel('RPM Error')
    ax3.set_title('Synchronization Error (Distance from Master)')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    
    # Save the plot as an image
    output_img = csv_filename.replace('.csv', '.png')
    plt.savefig(output_img)
    print(f"✅ Plot saved successfully as: {output_img}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_sync.py <your_log_file.csv>")
        sys.exit(1)
        
    log_file = sys.argv[1]
    plot_log(log_file)
