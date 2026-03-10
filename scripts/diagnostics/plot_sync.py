#!/usr/bin/env python3
"""
Plot motor synchronization / RPM-lock logs from CSV.
Supports three log formats produced by motor_test.py:
  1. rpm_lock_log_*.csv  - 4WD independent RPM-LOCK PI control
  2. sync_4wd_log_*.csv  - 4WD sync PI control (master/slave)
  3. legacy 2WD sync     - older rear-only sync logs

Requires pandas and matplotlib:
pip install pandas matplotlib
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Log-format detection
# ---------------------------------------------------------------------------
def detect_log_type(df):
    """Return 'rpm_lock', '4wd_sync', or '2wd_sync'."""
    if 'target_rpm' in df.columns and 'rl_pwm' in df.columns:
        return 'rpm_lock'
    if 'fr_rpm' in df.columns:
        return '4wd_sync'
    return '2wd_sync'


# ---------------------------------------------------------------------------
# RPM-LOCK plot  (rpm_lock_log_*.csv)
# Columns: time_sec, target_rpm, rl_rpm, rr_rpm, fr_rpm,
#          rl_pwm, rr_pwm, fr_pwm, fl_pwm
# ---------------------------------------------------------------------------
def plot_rpm_lock(df, csv_filename):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 13), sharex=True)
    fig.suptitle(f'RPM-LOCK 4WD Analysis\n{csv_filename}', fontsize=16)

    # --- Subplot 1: Wheel RPMs vs Target ---
    ax1.axhline(df['target_rpm'].iloc[0], color='black', linestyle='--', linewidth=2,
                label=f"Target RPM")
    # Draw a shaded band if target changes over time
    if df['target_rpm'].nunique() > 1:
        ax1.step(df['time_sec'], df['target_rpm'], color='black', linestyle='--',
                 linewidth=2, where='post')
    ax1.plot(df['time_sec'], df['rl_rpm'], label='RL RPM', color='blue', linewidth=2)
    ax1.plot(df['time_sec'], df['rr_rpm'], label='RR RPM', color='orange', linewidth=2, alpha=0.85)
    ax1.plot(df['time_sec'], df['fr_rpm'], label='FR RPM', color='green', linewidth=2, alpha=0.85)
    ax1.set_ylabel('Speed (RPM)')
    ax1.set_title('Wheel Speeds vs Target RPM')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper right')

    # --- Subplot 2: PWM applied to each wheel ---
    ax2.plot(df['time_sec'], df['rl_pwm'], label='RL PWM', color='blue', linewidth=2)
    ax2.plot(df['time_sec'], df['rr_pwm'], label='RR PWM', color='orange', alpha=0.85)
    ax2.plot(df['time_sec'], df['fr_pwm'], label='FR PWM', color='green', alpha=0.85)
    ax2.plot(df['time_sec'], df['fl_pwm'], label='FL PWM (mirror)', color='purple',
             linestyle=':', alpha=0.7)
    ax2.set_ylabel('PWM Duty Cycle (%)')
    ax2.set_title('PI Controller Output — Applied PWM per Wheel')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper right')

    # --- Subplot 3: RPM error from target for each wheel ---
    ax3.axhline(0, color='black', linewidth=1.5, linestyle='--')
    ax3.fill_between(df['time_sec'], -10, 10, alpha=0.08, color='green',
                     label='±10 RPM band')
    ax3.plot(df['time_sec'], df['target_rpm'] - df['rl_rpm'],
             label='RL Error', color='blue', alpha=0.85)
    ax3.plot(df['time_sec'], df['target_rpm'] - df['rr_rpm'],
             label='RR Error', color='orange', alpha=0.85)
    ax3.plot(df['time_sec'], df['target_rpm'] - df['fr_rpm'],
             label='FR Error', color='green', alpha=0.85)
    ax3.set_xlabel('Time (Seconds)')
    ax3.set_ylabel('RPM Error (Target − Actual)')
    ax3.set_title('Tracking Error per Wheel')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right')

    return fig


# ---------------------------------------------------------------------------
# 4WD / 2WD SYNC plot  (sync_4wd_log_*.csv or legacy)
# Columns: time_sec, target_speed_pwm, rl_rpm, rr_rpm, fr_rpm (4wd only),
#          rr_adj, fr_adj (4wd only), final_rr_pwm, final_fr_pwm (4wd only)
# ---------------------------------------------------------------------------
def plot_sync(df, csv_filename, is_4wd):
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
    ax2.plot(df['time_sec'], df['target_speed_pwm'], label='Target Base PWM',
             color='black', linestyle='--', linewidth=2)
    if is_4wd:
        ax2.plot(df['time_sec'], df['final_rr_pwm'], label='Final RR PWM', color='red', alpha=0.8)
        ax2.plot(df['time_sec'], df['final_fr_pwm'], label='Final FR PWM', color='purple', alpha=0.8)
    else:
        ax2.plot(df['time_sec'], df['final_rr_pwm'], label='Final RR PWM', color='red', alpha=0.8)
        adj_col = 'total_adj' if 'total_adj' in df.columns else 'rr_pwm_adj'
        if adj_col in df.columns:
            ax2.plot(df['time_sec'], df[adj_col], label=f'PI Adjustment ({adj_col})',
                     color='green', alpha=0.5)
    ax2.set_ylabel('Power / PWM (%)')
    ax2.set_title('Control Loop Effort (Final Applied Power)')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper right')

    # --- Subplot 3: Synchronization Error ---
    ax3.axhline(0, color='black', linewidth=1.5, linestyle='--')
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

    return fig


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
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

    # Normalise time to start at T=0
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

    log_type = detect_log_type(df)
    print(f"Detected log type: {log_type}")

    if log_type == 'rpm_lock':
        fig = plot_rpm_lock(df, csv_filename)
    else:
        fig = plot_sync(df, csv_filename, is_4wd=(log_type == '4wd_sync'))

    plt.tight_layout()
    output_img = csv_filename.replace('.csv', '.png')
    fig.savefig(output_img, dpi=150)
    print(f"Plot saved successfully as: {output_img}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_sync.py <your_log_file.csv>")
        sys.exit(1)

    plot_log(sys.argv[1])
