#!/usr/bin/env python3
"""
DRIVE LOG VISUALISER
====================
Reads a drive telemetry CSV from rover_logs/ and produces a multi-panel
performance dashboard focused on RPM synchronization, sensor data, and
drift analysis.

Usage:
    python3 plot_drive_log.py                          # auto-picks latest log
    python3 plot_drive_log.py rover_logs/drive_log_*.csv  # specific file
    python3 plot_drive_log.py --all                    # overlay all drive logs
"""

import sys, os, glob
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless — saves PNG, no display needed
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path
from datetime import datetime

# ── Resolve input file(s) ──────────────────────────────────────────
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, 'rover_logs')


def find_logs(args):
    if '--all' in args:
        return sorted(glob.glob(os.path.join(LOG_DIR, 'drive_log_*.csv')))
    for a in args[1:]:
        if os.path.isfile(a):
            return [a]
        cand = os.path.join(LOG_DIR, a)
        if os.path.isfile(cand):
            return [cand]
    # default: latest
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'drive_log_*.csv')))
    return files[-1:] if files else []


paths = find_logs(sys.argv)
if not paths:
    print("No drive logs found in rover_logs/")
    print("  Expected format: rover_logs/drive_log_YYYYMMDD_HHMMSS.csv")
    print("  Start logging via the web UI (drive_log_start) or gamepad.py")
    sys.exit(1)

print(f"Loading {len(paths)} log(s)...")

# ── Load & merge ───────────────────────────────────────────────────
frames = []
for p in paths:
    df = pd.read_csv(p)
    df['_src'] = Path(p).stem
    frames.append(df)
raw = pd.concat(frames, ignore_index=True)

# Time axis
raw['timestamp'] = pd.to_numeric(raw['timestamp'], errors='coerce')
raw = raw.dropna(subset=['timestamp']).sort_values('timestamp').reset_index(drop=True)
t0 = raw['timestamp'].iloc[0]
raw['t'] = raw['timestamp'] - t0

# Coerce numerics
num_cols = [
    'laser_front_mm', 'sonar_front_cm',
    'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'temp_c',
    'battery_mv', 'current_mv',
    'rpm_rear_left', 'rpm_rear_right', 'rpm_front_right',
    'cmd_pwm_left', 'cmd_pwm_right',
    'applied_pwm_fl', 'applied_pwm_fr', 'applied_pwm_rl', 'applied_pwm_rr',
    'throttle_input', 'steering_input',
    'target_rpm_left', 'target_rpm_right',
]
for c in num_cols:
    if c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors='coerce')

duration = raw['t'].iloc[-1]
total_rows = len(raw)

# Derived metrics
raw['accel_mag'] = np.sqrt(raw['accel_x']**2 + raw['accel_y']**2 + raw['accel_z']**2)
raw['rpm_drift'] = raw['rpm_rear_right'] - raw['rpm_rear_left']
raw['pwm_diff'] = raw['applied_pwm_rr'] - raw['applied_pwm_rl']

# Battery voltage from raw mV (ADS1115 A0 with 5:1 divider)
raw['battery_v'] = (raw['battery_mv'] / 1000.0) * 5.0
# Current from shunt mV (ADS1115 A1, 0.1Ω shunt)
raw['current_a'] = (raw['current_mv'] / 1000.0) / 0.1

# Filter to rows where car is moving
moving = raw[(raw['cmd_pwm_left'] > 1) | (raw['cmd_pwm_right'] > 1)]

print(f"  Duration: {duration:.1f}s  |  {total_rows} rows  |  "
      f"Moving: {len(moving)}  ({100*len(moving)/max(total_rows,1):.0f}%)")

# ── PERFORMANCE SUMMARY ───────────────────────────────────────────
print("\n" + "=" * 64)
print("  DRIVE LOG SUMMARY")
print("=" * 64)

# Source
sources = raw['source'].value_counts()
for src, cnt in sources.items():
    print(f"  Source: {src} ({cnt} rows)")

# Drive time
drive_time = len(moving) * 0.02  # ~20ms per cycle (50Hz)
print(f"  Active driving time: {drive_time:.1f}s / {duration:.1f}s")

# RPM sync quality (when moving)
if len(moving) > 0:
    drift = moving['rpm_drift'].dropna()
    if len(drift) > 0:
        print(f"  RPM drift (RR-RL): mean={drift.mean():.1f}  std={drift.std():.1f}  "
              f"max={drift.abs().max():.1f}")
        within_10 = (drift.abs() <= 10).sum()
        print(f"  RPM within ±10 band: {100*within_10/len(drift):.0f}%")
        bias = "RIGHT (RR faster)" if drift.mean() > 5 else \
               "LEFT (RL faster)" if drift.mean() < -5 else "NONE"
        print(f"  Drift bias: {bias}")

# Sync status distribution
if 'sync_status' in raw.columns:
    sync_dist = raw['sync_status'].value_counts()
    print(f"  Sync states: {dict(sync_dist)}")

# Distance sensors
if 'laser_front_mm' in raw.columns:
    laser = raw['laser_front_mm'].dropna()
    laser_valid = laser[laser > 0]
    if len(laser_valid) > 0:
        print(f"  Laser: min={laser_valid.min():.0f}mm  mean={laser_valid.mean():.0f}mm  "
              f"max={laser_valid.max():.0f}mm")

# Battery
bv = raw['battery_v'].dropna()
bv_valid = bv[bv > 0]
if len(bv_valid) > 0:
    print(f"  Battery: start={bv_valid.iloc[0]:.2f}V  end={bv_valid.iloc[-1]:.2f}V  "
          f"drop={bv_valid.iloc[0]-bv_valid.iloc[-1]:.3f}V")

# Current
ca = raw['current_a'].dropna()
ca_valid = ca[ca >= 0]
if len(ca_valid) > 0:
    print(f"  Current: mean={ca_valid.mean():.2f}A  peak={ca_valid.max():.2f}A")

# IMU
am = raw['accel_mag'].dropna()
if len(am) > 0:
    print(f"  Accel magnitude: mean={am.mean():.3f}g  max={am.max():.3f}g")

# Gear distribution
if 'gear' in raw.columns:
    gear_dist = raw['gear'].value_counts()
    print(f"  Gears used: {dict(gear_dist)}")

# Loop rate
if len(raw) > 1:
    dts = raw['t'].diff().dropna()
    dts = dts[dts > 0]
    if len(dts) > 0:
        hz = 1.0 / dts.mean()
        print(f"  Sample rate: {hz:.1f} Hz  (dt mean={dts.mean()*1000:.1f}ms  "
              f"p95={dts.quantile(0.95)*1000:.1f}ms)")

print("=" * 64)

# ── THEME ──────────────────────────────────────────────────────────
DARK = '#0d1117'
GRID = '#21262d'
TXT = '#c9d1d9'
BLUE = '#58a6ff'
GREEN = '#3fb950'
ORANGE = '#d29922'
RED = '#f85149'
PURPLE = '#bc8cff'
CYAN = '#39d2c0'


def style_ax(ax, title, ylabel=''):
    ax.set_facecolor(DARK)
    ax.set_title(title, color=BLUE, fontsize=10, fontweight='bold', pad=6)
    ax.set_ylabel(ylabel, color=TXT, fontsize=8)
    ax.tick_params(colors=TXT, labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(True, color=GRID, alpha=0.4, linewidth=0.5)


# ── FIGURE LAYOUT ─────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 28), facecolor=DARK)
fig.suptitle(
    f'DRIVE TELEMETRY — {Path(paths[-1]).stem}  ({duration:.1f}s, {total_rows} rows)',
    fontsize=14, color=BLUE, fontweight='bold', y=0.998
)
gs = GridSpec(10, 2, figure=fig, hspace=0.42, wspace=0.25,
              left=0.06, right=0.97, top=0.977, bottom=0.02)

t = raw['t']

# ──────────────────────────────────────────────────────────────────
# 1. DISTANCE SENSORS (laser + sonar)
# ──────────────────────────────────────────────────────────────────
ax1 = fig.add_subplot(gs[0, :])
style_ax(ax1, '1. Distance Sensors', 'Distance')

if 'laser_front_mm' in raw.columns:
    laser_cm = raw['laser_front_mm'] / 10.0
    ax1.plot(t, laser_cm, color=CYAN, linewidth=1, alpha=0.9, label='Laser (cm)')
if 'sonar_front_cm' in raw.columns:
    sonar = raw['sonar_front_cm']
    sonar_valid = sonar[sonar > 0]
    if len(sonar_valid) > 0:
        ax1.plot(t[sonar > 0], sonar_valid, color=ORANGE, linewidth=1, alpha=0.7,
                 label='Sonar (cm)')

# Danger zone shading
ax1.axhspan(0, 15, alpha=0.15, color=RED, label='STOP zone (<15cm)')
ax1.axhspan(15, 25, alpha=0.08, color=ORANGE, label='CRAWL zone (<25cm)')
ax1.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax1.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 2. RPM SYNC — wheel speeds vs target
# ──────────────────────────────────────────────────────────────────
ax2 = fig.add_subplot(gs[1, :])
style_ax(ax2, '2. Wheel RPMs vs Target', 'RPM')

if 'target_rpm_left' in raw.columns:
    ax2.plot(t, raw['target_rpm_left'], color='white', linewidth=1.5,
             linestyle='--', alpha=0.6, label='Target L')
if 'target_rpm_right' in raw.columns:
    ax2.plot(t, raw['target_rpm_right'], color='white', linewidth=1.5,
             linestyle=':', alpha=0.6, label='Target R')
ax2.plot(t, raw['rpm_rear_left'], color=BLUE, linewidth=1.2, label='RL (metal)')
ax2.plot(t, raw['rpm_rear_right'], color=ORANGE, linewidth=1.2, label='RR (plastic)')
ax2.plot(t, raw['rpm_front_right'], color=GREEN, linewidth=1, alpha=0.7,
         label='FR (metal)')
ax2.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax2.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 3. RPM ERROR — per-wheel tracking error with convergence band
# ──────────────────────────────────────────────────────────────────
ax3 = fig.add_subplot(gs[2, :])
style_ax(ax3, '3. RPM Tracking Error (Target - Actual)', 'RPM Error')

ax3.axhline(0, color='white', linewidth=1, linestyle='--', alpha=0.4)
ax3.fill_between(t, -10, 10, alpha=0.08, color=GREEN, label='±10 RPM band')

if 'target_rpm_left' in raw.columns:
    err_rl = raw['target_rpm_left'] - raw['rpm_rear_left']
    ax3.plot(t, err_rl, color=BLUE, linewidth=1, alpha=0.85, label='RL Error')
if 'target_rpm_right' in raw.columns:
    err_rr = raw['target_rpm_right'] - raw['rpm_rear_right']
    ax3.plot(t, err_rr, color=ORANGE, linewidth=1, alpha=0.85, label='RR Error')
    err_fr = raw['target_rpm_right'] - raw['rpm_front_right']
    ax3.plot(t, err_fr, color=GREEN, linewidth=1, alpha=0.7, label='FR Error')
ax3.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax3.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 4. APPLIED PWM — per-wheel duty cycles (PID output)
# ──────────────────────────────────────────────────────────────────
ax4 = fig.add_subplot(gs[3, :])
style_ax(ax4, '4. Applied PWM per Wheel (PID Output)', 'PWM %')

ax4.plot(t, raw['applied_pwm_fl'], color=PURPLE, linewidth=1, alpha=0.7,
         linestyle=':', label='FL (mirror)')
ax4.plot(t, raw['applied_pwm_fr'], color=GREEN, linewidth=1, alpha=0.85, label='FR')
ax4.plot(t, raw['applied_pwm_rl'], color=BLUE, linewidth=1.2, label='RL')
ax4.plot(t, raw['applied_pwm_rr'], color=ORANGE, linewidth=1.2, label='RR (plastic)')
ax4.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax4.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 5. COMMANDED vs APPLIED PWM — sync correction magnitude
# ──────────────────────────────────────────────────────────────────
ax5 = fig.add_subplot(gs[4, :])
style_ax(ax5, '5. Commanded vs Applied PWM (Sync Correction)', 'PWM %')

ax5.plot(t, raw['cmd_pwm_left'], color=BLUE, linewidth=1.5, linestyle='--',
         alpha=0.6, label='Cmd L')
ax5.plot(t, raw['cmd_pwm_right'], color=ORANGE, linewidth=1.5, linestyle='--',
         alpha=0.6, label='Cmd R')
ax5.plot(t, raw['applied_pwm_rl'], color=BLUE, linewidth=1, alpha=0.9,
         label='Applied RL')
ax5.plot(t, raw['applied_pwm_rr'], color=ORANGE, linewidth=1, alpha=0.9,
         label='Applied RR')
ax5.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax5.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 6. ACCELEROMETER — XYZ + magnitude
# ──────────────────────────────────────────────────────────────────
ax6 = fig.add_subplot(gs[5, 0])
style_ax(ax6, '6. Accelerometer (MPU6500)', 'Accel (g)')

ax6.plot(t, raw['accel_x'], color=RED, linewidth=0.8, alpha=0.8, label='X')
ax6.plot(t, raw['accel_y'], color=GREEN, linewidth=0.8, alpha=0.8, label='Y')
ax6.plot(t, raw['accel_z'], color=BLUE, linewidth=0.8, alpha=0.8, label='Z')
ax6.plot(t, raw['accel_mag'], color='white', linewidth=1, alpha=0.5, label='|Mag|')
ax6.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax6.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 7. GYROSCOPE — yaw rate (gyro_z)
# ──────────────────────────────────────────────────────────────────
ax7 = fig.add_subplot(gs[5, 1])
style_ax(ax7, '7. Gyroscope Z (Yaw Rate)', 'deg/s')

ax7.axhline(0, color='white', linewidth=0.5, alpha=0.3)
ax7.plot(t, raw['gyro_z'], color=CYAN, linewidth=0.8, label='Gyro Z')
if 'gyro_x' in raw.columns:
    ax7.plot(t, raw['gyro_x'], color=RED, linewidth=0.5, alpha=0.4, label='Gyro X')
    ax7.plot(t, raw['gyro_y'], color=GREEN, linewidth=0.5, alpha=0.4, label='Gyro Y')
ax7.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax7.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 8. BATTERY VOLTAGE & CURRENT
# ──────────────────────────────────────────────────────────────────
ax8 = fig.add_subplot(gs[6, 0])
style_ax(ax8, '8. Battery Voltage', 'Volts')

bv_plot = raw['battery_v']
bv_valid_mask = bv_plot > 0
if bv_valid_mask.any():
    ax8.plot(t[bv_valid_mask], bv_plot[bv_valid_mask], color=GREEN, linewidth=1.2,
             label='Battery V')
    ax8.axhline(9.0, color=RED, linewidth=0.8, linestyle='--', alpha=0.5,
                label='Low voltage (9V)')
ax8.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax8.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

ax8b = fig.add_subplot(gs[6, 1])
style_ax(ax8b, '8b. Current Draw', 'Amps')

ca_plot = raw['current_a']
ca_valid_mask = ca_plot >= 0
if ca_valid_mask.any():
    ax8b.plot(t[ca_valid_mask], ca_plot[ca_valid_mask], color=ORANGE, linewidth=1,
              label='Current A')
ax8b.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax8b.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
            labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 9. CONTROL INPUTS — throttle + steering + gear markers
# ──────────────────────────────────────────────────────────────────
ax9 = fig.add_subplot(gs[7, :])
style_ax(ax9, '9. Control Inputs', 'Value')

ax9.plot(t, raw['throttle_input'], color=GREEN, linewidth=1, label='Throttle %')
ax9.plot(t, raw['steering_input'], color=CYAN, linewidth=1, alpha=0.8, label='Steering')

# Gear change markers
if 'gear' in raw.columns:
    gear_changes = raw['gear'] != raw['gear'].shift(1)
    for idx in raw.index[gear_changes]:
        if idx == 0:
            continue
        g = raw.loc[idx, 'gear']
        ax9.axvline(raw.loc[idx, 't'], color=PURPLE, linewidth=0.8, alpha=0.5, linestyle=':')
        ax9.text(raw.loc[idx, 't'], ax9.get_ylim()[1] * 0.9, g,
                 color=PURPLE, fontsize=7, ha='center', va='top')

ax9.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax9.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 10. DRIFT INDICATOR — RR-RL RPM difference
# ──────────────────────────────────────────────────────────────────
ax10 = fig.add_subplot(gs[8, :])
style_ax(ax10, '10. Drift Indicator (RR - RL RPM) — Positive = Right Faster', 'RPM Diff')

ax10.axhline(0, color='white', linewidth=1, linestyle='--', alpha=0.4)
ax10.fill_between(t, -10, 10, alpha=0.08, color=GREEN, label='±10 RPM OK')

drift = raw['rpm_drift']
# Color: green when within band, orange/red when drifting
colors = np.where(drift.abs() <= 10, GREEN,
                  np.where(drift.abs() <= 30, ORANGE, RED))
ax10.scatter(t, drift, c=colors, s=1, alpha=0.6)

# Rolling average
if len(drift) > 20:
    drift_smooth = drift.rolling(window=25, center=True, min_periods=1).mean()
    ax10.plot(t, drift_smooth, color='white', linewidth=1.5, alpha=0.7,
              label='Rolling avg (0.5s)')

ax10.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax10.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 11. TEMPERATURE
# ──────────────────────────────────────────────────────────────────
ax11 = fig.add_subplot(gs[9, 0])
style_ax(ax11, '11. IMU Temperature', '°C')

if 'temp_c' in raw.columns:
    temp = raw['temp_c'].dropna()
    if len(temp) > 0:
        ax11.plot(t[:len(temp)], temp, color=RED, linewidth=1, label='MPU6500 Temp')
ax11.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax11.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
            labelcolor=TXT)

# ──────────────────────────────────────────────────────────────────
# 12. SYNC STATUS TIMELINE
# ──────────────────────────────────────────────────────────────────
ax12 = fig.add_subplot(gs[9, 1])
style_ax(ax12, '12. Wheel Sync Status', '')

if 'sync_status' in raw.columns:
    status_map = {'IDLE': 0, 'ACTIVE': 1, 'FALLBACK': 2, 'OFF': -1}
    status_colors = {'IDLE': TXT, 'ACTIVE': GREEN, 'FALLBACK': ORANGE, 'OFF': RED}
    status_num = raw['sync_status'].map(status_map).fillna(-1)
    # Color regions
    prev_status = None
    start_t = 0
    for i, row in raw.iterrows():
        st = row.get('sync_status', 'OFF')
        if st != prev_status:
            if prev_status is not None:
                clr = status_colors.get(prev_status, TXT)
                ax12.axvspan(start_t, row['t'], alpha=0.3, color=clr)
            prev_status = st
            start_t = row['t']
    # Final span
    if prev_status is not None:
        clr = status_colors.get(prev_status, TXT)
        ax12.axvspan(start_t, raw['t'].iloc[-1], alpha=0.3, color=clr)
    # Legend patches
    from matplotlib.patches import Patch
    patches = [Patch(facecolor=c, alpha=0.5, label=s) for s, c in status_colors.items()]
    ax12.legend(handles=patches, loc='upper right', fontsize=6,
                facecolor=DARK, edgecolor=GRID, labelcolor=TXT)

ax12.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax12.set_yticks([])

# ── Save ───────────────────────────────────────────────────────────
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(LOG_DIR, f"drive_plot_{ts}.png")
fig.savefig(out_path, dpi=150, facecolor=DARK)
plt.close(fig)
print(f"\n✅ Plot saved → {out_path}")
