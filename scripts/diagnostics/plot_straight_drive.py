#!/usr/bin/env python3
"""
STRAIGHT-DRIVE LOG VISUALISER
==============================
Reads a straight-drive CSV from rover_logs/ and produces a multi-panel
dashboard: heading tracking, PID components, steering correction, gyro,
RPMs, accelerometer, and loop timing.

Usage:
    python3 plot_straight_drive.py                       # auto-picks latest log
    python3 plot_straight_drive.py rover_logs/straight_drive_*.csv
    python3 plot_straight_drive.py --all                 # overlay all runs
"""

import sys
import os
import glob

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')                  # headless — saves PNG, no display needed
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import FuncFormatter, MultipleLocator
from pathlib import Path

# ── Resolve log directory ──────────────────────────────────────────
_DIAG_DIR    = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.normpath(os.path.join(_DIAG_DIR, '..', '..'))
LOG_DIR      = os.path.join(_PROJECT_ROOT, 'rover_logs')
OUTPUT_DIR   = LOG_DIR                 # PNGs saved alongside CSVs


def find_logs(args):
    if '--all' in args:
        return sorted(glob.glob(os.path.join(LOG_DIR, 'straight_drive_*.csv')))
    for a in args[1:]:
        if os.path.isfile(a):
            return [a]
        cand = os.path.join(LOG_DIR, a)
        if os.path.isfile(cand):
            return [cand]
    # default: latest
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'straight_drive_*.csv')))
    return files[-1:] if files else []


paths = find_logs(sys.argv)
if not paths:
    print("No straight-drive logs found.")
    print(f"  Expected in : {LOG_DIR}")
    print( "  Pattern     : straight_drive_YYYYMMDD_HHMMSS.csv")
    print( "  Run test    : python3 test_drive.py  (arm with SPACE, then drive)")
    sys.exit(1)

print(f"Loading {len(paths)} log(s)…")

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

# Coerce all numeric columns
num_cols = [
    'elapsed_s', 'armed',
    'target_heading', 'current_heading', 'heading_error',
    'gyro_z', 'correction',
    'pid_p', 'pid_i_contrib', 'pid_d_contrib', 'integral',
    'mag_x', 'mag_y', 'mag_z',
    'accel_x', 'accel_y', 'accel_z',
    'rpm_rear_right', 'rpm_rear_left', 'rpm_front_right',
    'speed_pct', 'loop_hz',
]
for c in num_cols:
    if c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors='coerce')

# Derived
raw['accel_mag'] = np.sqrt(raw['accel_x']**2 + raw['accel_y']**2 + raw['accel_z']**2)
moving = raw[raw.get('armed', pd.Series(0, index=raw.index)).fillna(0) > 0]

duration  = raw['t'].iloc[-1]
total_rows = len(raw)

# ── CONSOLE SUMMARY ────────────────────────────────────────────────
print(f"\n{'='*64}")
print("  STRAIGHT-DRIVE LOG SUMMARY")
print(f"{'='*64}")
print(f"  Source      : {Path(paths[-1]).stem}")
print(f"  Duration    : {duration:.1f} s   |   {total_rows} rows")
print(f"  Armed rows  : {len(moving)}")

if len(moving) > 0 and 'heading_error' in moving.columns:
    err = moving['heading_error'].dropna()
    if len(err) > 0:
        print(f"\n  HEADING CONTROL")
        print(f"    Error mean   : {err.mean():+.2f}°")
        print(f"    Error std    : {err.std():.2f}°")
        print(f"    Error max abs: {err.abs().max():.2f}°")
        within_5 = (err.abs() <= 5.0).sum()
        print(f"    Within ±5°   : {100*within_5/max(len(err),1):.0f}%")
        within_2 = (err.abs() <= 2.0).sum()
        print(f"    Within ±2°   : {100*within_2/max(len(err),1):.0f}%")

if len(moving) > 0 and 'correction' in moving.columns:
    corr = moving['correction'].dropna()
    if len(corr) > 0:
        print(f"\n  STEERING CORRECTION")
        print(f"    Mean abs     : {corr.abs().mean():.2f}°")
        print(f"    Peak abs     : {corr.abs().max():.2f}°")

if 'rpm_rear_right' in raw.columns and 'rpm_rear_left' in raw.columns and len(moving) > 0:
    rr = moving['rpm_rear_right'].dropna()
    rl = moving['rpm_rear_left'].dropna()
    if len(rr) > 0 and len(rl) > 0:
        drift = rr.values[:min(len(rr), len(rl))] - rl.values[:min(len(rr), len(rl))]
        print(f"\n  RPM BALANCE (RR - RL)")
        print(f"    Drift mean   : {drift.mean():+.1f}")
        print(f"    Drift std    : {drift.std():.1f}")

if 'loop_hz' in raw.columns:
    hz = raw['loop_hz'].dropna()
    hz = hz[hz > 0]
    if len(hz) > 0:
        print(f"\n  LOOP TIMING")
        print(f"    Hz mean      : {hz.mean():.1f}")
        print(f"    Hz min       : {hz.min():.1f}")
        print(f"    Hz p5        : {hz.quantile(0.05):.1f}")

print(f"{'='*64}\n")

# ── DARK THEME ─────────────────────────────────────────────────────
DARK   = '#0d1117'
GRID   = '#21262d'
TXT    = '#c9d1d9'
BLUE   = '#58a6ff'
GREEN  = '#3fb950'
ORANGE = '#d29922'
RED    = '#f85149'
PURPLE = '#bc8cff'
CYAN   = '#39d2c0'
YELLOW = '#e3b341'

# ── Compass helpers ────────────────────────────────────────────────
_COMPASS_POINTS = [
    (0,   'N'),  (22.5,  'NNE'), (45,   'NE'),  (67.5,  'ENE'),
    (90,  'E'),  (112.5, 'ESE'), (135,  'SE'),   (157.5, 'SSE'),
    (180, 'S'),  (202.5, 'SSW'), (225,  'SW'),   (247.5, 'WSW'),
    (270, 'W'),  (292.5, 'WNW'), (315,  'NW'),   (337.5, 'NNW'),
    (360, 'N'),
]
_COMPASS_MAP = {d: lbl for d, lbl in _COMPASS_POINTS}


def _compass_label(deg: float) -> str:
    """Return the compass direction name for any degree value."""
    rounded = round((deg % 360) / 22.5) * 22.5 % 360
    return _COMPASS_MAP.get(rounded, f'{deg:.1f}°')


def compass_fmt(val, pos):
    """Formatter for heading y-axis: '135°\nSE'."""
    return f'{val:.0f}°\n{_compass_label(val)}'


def add_compass_refs(ax, hdg_min: float, hdg_max: float, t_end: float):
    """Draw dashed reference lines for compass points visible in range."""
    margin = 15.0
    for deg, lbl in _COMPASS_POINTS:
        if hdg_min - margin <= deg <= hdg_max + margin:
            ax.axhline(deg, color=GRID, linewidth=0.8, linestyle=':',
                       alpha=0.7, zorder=1)
            ax.text(t_end * 1.002, deg, f' {lbl}', color=TXT,
                    fontsize=7, va='center', clip_on=False)


def style_ax(ax, title, ylabel=''):
    ax.set_facecolor(DARK)
    ax.set_title(title, color=BLUE, fontsize=10, fontweight='bold', pad=6)
    ax.set_ylabel(ylabel, color=TXT, fontsize=8)
    ax.tick_params(colors=TXT, labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(True, color=GRID, alpha=0.4, linewidth=0.5)


def arm_shading(ax, t_series, armed_series):
    """Shade background green when armed."""
    if armed_series is None:
        return
    armed_bool = armed_series.fillna(0).astype(bool)
    in_arm = False
    t_start = None
    for i, (ti, a) in enumerate(zip(t_series, armed_bool)):
        if a and not in_arm:
            t_start = ti
            in_arm  = True
        elif not a and in_arm:
            ax.axvspan(t_start, ti, alpha=0.06, color=GREEN, zorder=0)
            in_arm = False
    if in_arm and t_start is not None:
        ax.axvspan(t_start, t_series.iloc[-1], alpha=0.06, color=GREEN, zorder=0)


# ── FIGURE LAYOUT ─────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 28), facecolor=DARK)
title_stem = Path(paths[-1]).stem
fig.suptitle(
    f'STRAIGHT-DRIVE TEST  —  {title_stem}   ({duration:.1f}s, {total_rows} rows)',
    fontsize=14, color=BLUE, fontweight='bold', y=0.999,
)
gs = GridSpec(8, 2, figure=fig,
              hspace=0.45, wspace=0.28,
              left=0.07, right=0.97, top=0.975, bottom=0.025)

t      = raw['t']
armed  = raw.get('armed', None)

# ══════════════════════════════════════════════════════════════════
# 1. HEADING — target vs current  (full width)
# ══════════════════════════════════════════════════════════════════
ax1 = fig.add_subplot(gs[0, :])
style_ax(ax1, '1. Compass Heading  —  Target vs Actual', 'Bearing')
arm_shading(ax1, t, armed)

ax1.plot(t, raw['target_heading'],  color='white',  linewidth=1.5,
         linestyle='--', alpha=0.7, label='Target heading')
ax1.plot(t, raw['current_heading'], color=BLUE,     linewidth=1.2,
         label='Current heading')

# Compass y-axis
all_hdg = pd.concat([raw['target_heading'], raw['current_heading']]).dropna()
hdg_min, hdg_max = all_hdg.min(), all_hdg.max()
ax1.set_ylim(hdg_min - 10, hdg_max + 10)
ax1.yaxis.set_major_locator(MultipleLocator(22.5))
ax1.yaxis.set_major_formatter(FuncFormatter(compass_fmt))
add_compass_refs(ax1, hdg_min, hdg_max, t.iloc[-1])

ax1.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax1.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 2. HEADING ERROR  (full width)
# ══════════════════════════════════════════════════════════════════
ax2 = fig.add_subplot(gs[1, :])
style_ax(ax2, '2. Heading Error  (negative = drifting right, positive = drifting left)', 'Error (°)')
arm_shading(ax2, t, armed)

ax2.axhline(0,  color='white',  linewidth=0.8, linestyle='--', alpha=0.5)
ax2.fill_between(t, -2,  2,  alpha=0.12, color=GREEN,  label='±2° band')
ax2.fill_between(t, -5,  5,  alpha=0.07, color=ORANGE, label='±5° band')
ax2.plot(t, raw['heading_error'], color=RED, linewidth=1.1, label='Error')

ax2.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax2.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 3. STEERING CORRECTION  (full width)
# ══════════════════════════════════════════════════════════════════
ax3 = fig.add_subplot(gs[2, :])
style_ax(ax3, '3. Steering Correction  (PID Output)', 'Correction (°)')
arm_shading(ax3, t, armed)

ax3.axhline(0, color='white', linewidth=0.8, linestyle='--', alpha=0.4)
ax3.fill_between(t,  0, raw['correction'], where=raw['correction'] >= 0,
                 alpha=0.25, color=ORANGE, label='Right correction')
ax3.fill_between(t,  0, raw['correction'], where=raw['correction'] <  0,
                 alpha=0.25, color=CYAN,   label='Left correction')
ax3.plot(t, raw['correction'], color=YELLOW, linewidth=1.1, label='Correction')

ax3.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax3.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 4. PID COMPONENT BREAKDOWN  (left half)
# ══════════════════════════════════════════════════════════════════
ax4 = fig.add_subplot(gs[3, 0])
style_ax(ax4, '4. PID Components', 'Contribution (°)')
arm_shading(ax4, t, armed)

ax4.axhline(0, color='white', linewidth=0.5, alpha=0.3)
ax4.plot(t, raw['pid_p'],         color=BLUE,   linewidth=1.0, label=f'P (Kp={0.80})')
ax4.plot(t, raw['pid_i_contrib'], color=GREEN,  linewidth=1.0, label=f'I (Ki={0.10})')
ax4.plot(t, raw['pid_d_contrib'], color=ORANGE, linewidth=1.0, label=f'D (Kd={0.10})')
ax4.plot(t, raw['correction'],    color='white', linewidth=1.0,
         linestyle=':', alpha=0.6, label='Total output')

ax4.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax4.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 5. INTEGRAL ACCUMULATOR  (right half)
# ══════════════════════════════════════════════════════════════════
ax5 = fig.add_subplot(gs[3, 1])
style_ax(ax5, '5. PID Integral Accumulator', 'Integral')
arm_shading(ax5, t, armed)

ax5.axhline(0,   color='white', linewidth=0.5, alpha=0.3)
ax5.fill_between(t, 0, raw['integral'],
                 where=raw['integral'] >= 0, alpha=0.2, color=GREEN)
ax5.fill_between(t, 0, raw['integral'],
                 where=raw['integral'] <  0, alpha=0.2, color=RED)
ax5.plot(t, raw['integral'], color=PURPLE, linewidth=1.0, label='Integral')

ax5.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax5.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 6. GYRO Z — yaw rate  (full width)
# ══════════════════════════════════════════════════════════════════
ax6 = fig.add_subplot(gs[4, :])
style_ax(ax6, '6. Gyroscope Z  (Yaw Rate)', 'deg/s')
arm_shading(ax6, t, armed)

ax6.axhline(0, color='white', linewidth=0.5, linestyle='--', alpha=0.3)
ax6.fill_between(t, 0, raw['gyro_z'],
                 where=raw['gyro_z'] >= 0, alpha=0.15, color=CYAN)
ax6.fill_between(t, 0, raw['gyro_z'],
                 where=raw['gyro_z'] <  0, alpha=0.15, color=PURPLE)
ax6.plot(t, raw['gyro_z'], color=CYAN, linewidth=1.0, label='Gyro Z')

ax6.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax6.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 7. WHEEL RPMs  (full width)
# ══════════════════════════════════════════════════════════════════
ax7 = fig.add_subplot(gs[5, :])
style_ax(ax7, '7. Wheel RPMs', 'RPM')
arm_shading(ax7, t, armed)

ax7.plot(t, raw['rpm_rear_right'],  color=ORANGE, linewidth=1.2, label='RR (rear-right)')
ax7.plot(t, raw['rpm_rear_left'],   color=BLUE,   linewidth=1.2, label='RL (rear-left)')
ax7.plot(t, raw['rpm_front_right'], color=GREEN,  linewidth=1.0,
         alpha=0.8, label='FR (front-right)')

ax7.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax7.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 8. ACCELEROMETER  (left half)
# ══════════════════════════════════════════════════════════════════
ax8 = fig.add_subplot(gs[6, 0])
style_ax(ax8, '8. Accelerometer', 'g')
arm_shading(ax8, t, armed)

ax8.plot(t, raw['accel_x'],   color=RED,    linewidth=0.8, alpha=0.85, label='X')
ax8.plot(t, raw['accel_y'],   color=GREEN,  linewidth=0.8, alpha=0.85, label='Y')
ax8.plot(t, raw['accel_z'],   color=BLUE,   linewidth=0.8, alpha=0.85, label='Z')
ax8.plot(t, raw['accel_mag'], color='white', linewidth=1.0, alpha=0.5,  label='|Mag|')

ax8.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax8.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 9. LOOP FREQUENCY  (right half)
# ══════════════════════════════════════════════════════════════════
ax9 = fig.add_subplot(gs[6, 1])
style_ax(ax9, '9. Control Loop Frequency', 'Hz')
arm_shading(ax9, t, armed)

hz_series = raw['loop_hz'].clip(0, 200)
ax9.axhline(20, color='white', linewidth=0.8, linestyle='--', alpha=0.4, label='Target 20 Hz')
ax9.plot(t, hz_series, color=CYAN, linewidth=0.8, label='Actual Hz')

ax9.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax9.legend(loc='upper right', fontsize=6, facecolor=DARK, edgecolor=GRID,
           labelcolor=TXT)

# ══════════════════════════════════════════════════════════════════
# 10. MAGNETOMETER RAW  (full width)
# ══════════════════════════════════════════════════════════════════
ax10 = fig.add_subplot(gs[7, :])
style_ax(ax10, '10. Magnetometer Raw (Gauss)', 'Gauss')
arm_shading(ax10, t, armed)

ax10.plot(t, raw['mag_x'], color=RED,    linewidth=0.8, alpha=0.85, label='Mag X')
ax10.plot(t, raw['mag_y'], color=GREEN,  linewidth=0.8, alpha=0.85, label='Mag Y')
ax10.plot(t, raw['mag_z'], color=BLUE,   linewidth=0.8, alpha=0.85, label='Mag Z')

ax10.set_xlabel('Time (s)', color=TXT, fontsize=8)
ax10.legend(loc='upper right', fontsize=7, facecolor=DARK, edgecolor=GRID,
            labelcolor=TXT)

# ── Save ───────────────────────────────────────────────────────────
stem     = Path(paths[-1]).stem
out_path = os.path.join(OUTPUT_DIR, f'{stem}.png')
plt.savefig(out_path, dpi=120, facecolor=DARK, bbox_inches='tight')
print(f"📊 Plot saved → {out_path}")
