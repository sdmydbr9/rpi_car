#!/usr/bin/env python3
"""
ROVER TELEMETRY VISUALISER
===========================
Reads a telemetry CSV from rover_logs/ and produces a multi-panel performance
dashboard plus a printed summary.

Usage:
    python3 rover_plots.py                      # auto-picks latest log
    python3 rover_plots.py rover_logs/telem_*.csv  # specific file
    python3 rover_plots.py --all                # overlay all logs
"""

import sys, os, glob
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless — saves PNG, no display needed
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path

# ── Resolve input file(s) ──────────────────────────────────────────
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, 'rover_logs')

def find_logs(args):
    if '--all' in args:
        return sorted(glob.glob(os.path.join(LOG_DIR, 'telem_*.csv')))
    for a in args[1:]:
        if os.path.isfile(a):
            return [a]
        cand = os.path.join(LOG_DIR, a)
        if os.path.isfile(cand):
            return [cand]
    # default: latest
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'telem_*.csv')))
    return files[-1:] if files else []

paths = find_logs(sys.argv)
if not paths:
    print("No telemetry logs found in rover_logs/")
    sys.exit(1)

print(f"Loading {len(paths)} log(s)...")

# ── Load & merge ───────────────────────────────────────────────────
frames = []
for p in paths:
    df = pd.read_csv(p)
    df['_src'] = Path(p).stem
    frames.append(df)
raw = pd.concat(frames, ignore_index=True)

# Time axis: seconds relative to first timestamp
raw['timestamp'] = pd.to_numeric(raw['timestamp'], errors='coerce')
raw = raw.dropna(subset=['timestamp']).sort_values('timestamp').reset_index(drop=True)
t0 = raw['timestamp'].iloc[0]
raw['t'] = raw['timestamp'] - t0

# Coerce numerics
num_cols = [
    'laser_front_cm', 'sonar_rear_cm', 'accel_x', 'accel_y', 'accel_z',
    'accel_mag', 'gyro_z', 'rpm', 'battery_v', 'current_a',
    'target_bbox_area', 'vision_steer_pull', 'raw_throttle', 'raw_steering',
    'smooth_throttle', 'smooth_steering', 'motor_L', 'motor_R',
    'sweep_esc_angle', 'sweep_esc_dist', 'touch_conf',
]
for c in num_cols:
    if c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors='coerce')

# Boolean columns
for c in ['ir_left', 'ir_right', 'target_found', 'sweep_active']:
    if c in raw.columns:
        raw[c] = raw[c].map({'True': True, 'False': False, '1': True, '0': False,
                             True: True, False: False, 1: True, 0: False})

duration = raw['t'].iloc[-1]
total_rows = len(raw)

# Split sensor rows (have battery_v) vs autopilot rows (have avoid_state)
has_motor = raw['motor_L'].notna()
has_sensor = raw['battery_v'].notna() & raw['laser_front_cm'].notna()
ap = raw[has_motor].copy()
sn = raw[has_sensor].copy()

print(f"  Duration: {duration:.1f}s  |  {total_rows} rows  |  "
      f"Autopilot: {len(ap)}  Sensor: {len(sn)}")

# ── Derived metrics ────────────────────────────────────────────────
if 'laser_front_cm' in ap.columns:
    ap['near_obstacle'] = ap['laser_front_cm'].fillna(999) < 40

if 'motor_L' in ap.columns and 'motor_R' in ap.columns:
    ap['motor_diff'] = ap['motor_L'] - ap['motor_R']

if 'vision_mode' in ap.columns:
    ap['is_tracking'] = ap['vision_mode'].isin(['TRACKING', 'TOUCH_TRACKING'])

# ── PERFORMANCE SUMMARY ───────────────────────────────────────────
print("\n" + "=" * 60)
print("  PERFORMANCE SUMMARY")
print("=" * 60)

# Driving time
driving = ap[ap['smooth_throttle'].fillna(0).abs() > 1]
drive_time = len(driving) * 0.05  # ~50ms per cycle
print(f"  Active driving time: {drive_time:.1f}s / {duration:.1f}s "
      f"({100*drive_time/max(duration,0.1):.0f}%)")

# Obstacle encounters
if 'avoid_state' in ap.columns:
    # Count transitions into SWEEP_WAIT as distinct events
    state_seq = ap['avoid_state'].fillna('CLEAR')
    sweep_starts = ((state_seq == 'SWEEP_WAIT') & (state_seq.shift(1) != 'SWEEP_WAIT')).sum()
    escape_starts = ((state_seq == 'ESCAPING') & (state_seq.shift(1) != 'ESCAPING')).sum()
    print(f"  Obstacle events (sweep triggers): {sweep_starts}")
    print(f"  Escape maneuvers completed: {escape_starts}")

    # Time in each state
    for st in ['CLEAR', 'SWEEP_WAIT', 'ESCAPING']:
        cnt = (state_seq == st).sum()
        pct = 100 * cnt / max(len(state_seq), 1)
        print(f"    {st:12s}: {cnt:5d} cycles ({pct:.1f}%)")

    if 'laser_front_cm' in ap.columns:
        close = ap['laser_front_cm'].fillna(999)
        min_dist = close[close < 900].min() if (close < 900).any() else 999
        print(f"  Closest front approach: {min_dist:.1f} cm")

if 'sonar_rear_cm' in sn.columns:
    rear = sn['sonar_rear_cm'].fillna(999)
    min_rear = rear[rear < 900].min() if (rear < 900).any() else 999
    print(f"  Closest rear approach: {min_rear:.1f} cm")

# Tracking stats
if 'vision_mode' in ap.columns:
    track_rows = ap['is_tracking'].sum() if 'is_tracking' in ap.columns else 0
    found_rows = ap['target_found'].fillna(False).sum()
    track_pct = 100 * found_rows / max(track_rows, 1)
    print(f"  Tracking frames: {track_rows}  |  Target found: {found_rows} ({track_pct:.0f}%)")

    # Lost target events
    if 'is_tracking' in ap.columns:
        tracking_mask = ap['is_tracking'].fillna(False)
        found_mask = ap['target_found'].fillna(False)
        lost_events = ((~found_mask) & tracking_mask & found_mask.shift(1).fillna(False)).sum()
        print(f"  Target lost events: {lost_events}")

if 'touch_conf' in ap.columns:
    tc = ap['touch_conf'].dropna()
    if len(tc) > 0:
        print(f"  Touch confidence: mean={tc.mean():.3f}  min={tc.min():.3f}  "
              f"max={tc.max():.3f}  std={tc.std():.3f}")

# Battery
if 'battery_v' in sn.columns:
    bv = sn['battery_v'].dropna()
    if len(bv) > 0:
        print(f"  Battery: start={bv.iloc[0]:.2f}V  end={bv.iloc[-1]:.2f}V  "
              f"drop={bv.iloc[0]-bv.iloc[-1]:.3f}V")

# Current draw
if 'current_a' in sn.columns:
    ca = sn['current_a'].dropna()
    if len(ca) > 0:
        print(f"  Current: mean={ca.mean():.2f}A  peak={ca.max():.2f}A")

# IMU
if 'accel_mag' in sn.columns:
    am = sn['accel_mag'].dropna()
    if len(am) > 0:
        print(f"  Accel magnitude: mean={am.mean():.3f}g  max={am.max():.3f}g  "
              f"std={am.std():.3f}g")

# Motor stats
if 'motor_L' in ap.columns:
    mL = ap['motor_L'].dropna()
    mR = ap['motor_R'].dropna()
    if len(mL) > 0:
        print(f"  Motor L: mean={mL.mean():.1f}  max={mL.max():.1f}")
        print(f"  Motor R: mean={mR.mean():.1f}  max={mR.max():.1f}")
        if 'motor_diff' in ap.columns:
            md = ap['motor_diff'].dropna()
            print(f"  Motor diff (L-R): mean={md.mean():.1f}  std={md.std():.1f}  "
                  f"(bias={'LEFT' if md.mean()>2 else 'RIGHT' if md.mean()<-2 else 'NONE'})")

# Loop rate estimate from timestamp gaps
if len(ap) > 1:
    dts = ap['t'].diff().dropna()
    dts = dts[dts > 0]
    if len(dts) > 0:
        hz = 1.0 / dts.mean()
        print(f"  Autopilot loop rate: {hz:.1f} Hz  (dt mean={dts.mean()*1000:.1f}ms  "
              f"p95={dts.quantile(0.95)*1000:.1f}ms)")

print("=" * 60)

# ── PLOTS ──────────────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 24), facecolor='#0d1117')
fig.suptitle(
    f'ROVER TELEMETRY — {Path(paths[-1]).stem}  ({duration:.1f}s, {total_rows} rows)',
    fontsize=14, color='#58a6ff', fontweight='bold', y=0.998
)
gs = GridSpec(8, 2, figure=fig, hspace=0.38, wspace=0.25,
              left=0.06, right=0.97, top=0.977, bottom=0.025)

DARK = '#0d1117'
GRID = '#21262d'
TXT = '#c9d1d9'

def style_ax(ax, title, ylabel=''):
    ax.set_facecolor(DARK)
    ax.set_title(title, color='#58a6ff', fontsize=11, fontweight='bold', pad=6)
    ax.set_ylabel(ylabel, color=TXT, fontsize=9)
    ax.tick_params(colors=TXT, labelsize=8)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(True, color=GRID, alpha=0.4, linewidth=0.5)

def shade_avoid(ax, data):
    """Background shading for obstacle avoidance states."""
    if 'avoid_state' not in data.columns or len(data) == 0:
        return
    state_colors = {'SWEEP_WAIT': '#ffcc0030', 'ESCAPING': '#ff334430'}
    prev = None; start = None
    for _, row in data.iterrows():
        st = row.get('avoid_state', 'CLEAR')
        if st != prev:
            if prev in state_colors and start is not None:
                ax.axvspan(start, row['t'], color=state_colors[prev], zorder=0)
            start = row['t']
            prev = st
    if prev in state_colors and start is not None:
        ax.axvspan(start, data['t'].iloc[-1], color=state_colors[prev], zorder=0)

# ── 1. DISTANCE SENSORS (full width) ─────────────────────────────
ax1 = fig.add_subplot(gs[0, :])
style_ax(ax1, 'DISTANCE SENSORS', 'cm')
if 'laser_front_cm' in sn.columns:
    front = sn['laser_front_cm'].clip(upper=300)
    ax1.plot(sn['t'], front, color='#39d353', linewidth=1, label='Front Laser', alpha=0.9)
if 'sonar_rear_cm' in sn.columns:
    rear = sn['sonar_rear_cm'].clip(upper=300)
    ax1.plot(sn['t'], rear, color='#f78166', linewidth=1, label='Rear Sonar', alpha=0.7)
ax1.axhline(12, color='#ff3344', linestyle='--', alpha=0.5, linewidth=0.8, label='Stop 12cm')
ax1.axhline(25, color='#ff8800', linestyle='--', alpha=0.4, linewidth=0.8, label='Creep 25cm')
ax1.axhline(50, color='#ffcc00', linestyle='--', alpha=0.4, linewidth=0.8, label='Slow 50cm')
ax1.axhline(100, color='#39d353', linestyle='--', alpha=0.3, linewidth=0.8, label='Aware 100cm')
shade_avoid(ax1, ap)
ax1.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax1.set_xlim(0, duration)

# ── 2. MOTOR OUTPUT ──────────────────────────────────────────────
ax2 = fig.add_subplot(gs[1, 0])
style_ax(ax2, 'MOTOR OUTPUT', '%')
if len(ap) > 0:
    ax2.plot(ap['t'], ap['motor_L'], color='#58a6ff', linewidth=0.9, label='Left', alpha=0.8)
    ax2.plot(ap['t'], ap['motor_R'], color='#f0883e', linewidth=0.9, label='Right', alpha=0.8)
    shade_avoid(ax2, ap)
    ax2.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax2.set_xlim(0, duration)

# ── 3. STEERING ──────────────────────────────────────────────────
ax3 = fig.add_subplot(gs[1, 1])
style_ax(ax3, 'STEERING', 'value')
if len(ap) > 0:
    ax3.plot(ap['t'], ap['raw_steering'], color='#8b949e', linewidth=0.7, label='Raw', alpha=0.5)
    ax3.plot(ap['t'], ap['smooth_steering'], color='#d2a8ff', linewidth=1, label='Smooth', alpha=0.9)
    ax3.axhline(0, color=TXT, linewidth=0.3, alpha=0.4)
    shade_avoid(ax3, ap)
    ax3.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax3.set_xlim(0, duration)

# ── 4. THROTTLE ──────────────────────────────────────────────────
ax4 = fig.add_subplot(gs[2, 0])
style_ax(ax4, 'THROTTLE', '%')
if len(ap) > 0:
    ax4.fill_between(ap['t'], 0, ap['smooth_throttle'], color='#39d353', alpha=0.3)
    ax4.plot(ap['t'], ap['smooth_throttle'], color='#39d353', linewidth=1, label='Smooth')
    ax4.plot(ap['t'], ap['raw_throttle'], color='#8b949e', linewidth=0.6, label='Raw', alpha=0.5)
    shade_avoid(ax4, ap)
    ax4.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax4.set_xlim(0, duration)

# ── 5. TRACKING CONFIDENCE ──────────────────────────────────────
ax5 = fig.add_subplot(gs[2, 1])
style_ax(ax5, 'TRACKING CONFIDENCE', '')
if 'touch_conf' in ap.columns and ap['touch_conf'].notna().any():
    tc = ap.dropna(subset=['touch_conf'])
    ax5.fill_between(tc['t'], 0, tc['touch_conf'], color='#58a6ff', alpha=0.2)
    ax5.plot(tc['t'], tc['touch_conf'], color='#58a6ff', linewidth=1, label='Touch Conf')
    ax5.axhline(0.35, color='#ff3344', linestyle='--', alpha=0.5, linewidth=0.8, label='Min Threshold')
    ax5.axhline(0.6, color='#39d353', linestyle='--', alpha=0.3, linewidth=0.8, label='Good Lock')
if 'target_found' in ap.columns:
    found = ap[ap['target_found'] == True]
    if len(found) > 0:
        ax5.scatter(found['t'], [0.02] * len(found), color='#39d353', s=2, alpha=0.5, label='Found')
    if 'is_tracking' in ap.columns:
        lost_tracking = ap[(ap['target_found'] == False) & (ap['is_tracking'] == True)]
        if len(lost_tracking) > 0:
            ax5.scatter(lost_tracking['t'], [0.02] * len(lost_tracking),
                        color='#ff3344', s=2, alpha=0.5, label='Lost')
ax5.set_ylim(-0.05, 1.05)
ax5.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax5.set_xlim(0, duration)

# ── 6. ACCEL MAGNITUDE ──────────────────────────────────────────
ax6 = fig.add_subplot(gs[3, 0])
style_ax(ax6, 'ACCELEROMETER (magnitude)', 'g')
if 'accel_mag' in sn.columns:
    am = sn['accel_mag'].dropna()
    ax6.plot(sn.loc[am.index, 't'], am, color='#f0883e', linewidth=0.8)
    ax6.axhline(1.0, color='#39d353', linestyle='--', alpha=0.3, linewidth=0.8, label='1g level')
    ax6.axhline(1.8, color='#ff3344', linestyle='--', alpha=0.5, linewidth=0.8, label='Tilt threshold')
    ax6.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax6.set_xlim(0, duration)

# ── 7. GYRO Z ───────────────────────────────────────────────────
ax7 = fig.add_subplot(gs[3, 1])
style_ax(ax7, 'GYROSCOPE Z (yaw rate)', 'deg/s')
if 'gyro_z' in sn.columns:
    gz = sn['gyro_z'].dropna()
    ax7.plot(sn.loc[gz.index, 't'], gz, color='#d2a8ff', linewidth=0.8)
    ax7.axhline(0, color=TXT, linewidth=0.3, alpha=0.4)
ax7.set_xlim(0, duration)

# ── 8. BATTERY + CURRENT ────────────────────────────────────────
ax8 = fig.add_subplot(gs[4, 0])
style_ax(ax8, 'BATTERY & CURRENT', 'V')
if 'battery_v' in sn.columns:
    ax8.plot(sn['t'], sn['battery_v'], color='#39d353', linewidth=1, label='Voltage')
    ax8.legend(loc='upper left', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax8_r = ax8.twinx()
if 'current_a' in sn.columns:
    ax8_r.plot(sn['t'], sn['current_a'], color='#f78166', linewidth=0.8, alpha=0.7, label='Current')
    ax8_r.set_ylabel('A', color='#f78166', fontsize=9)
    ax8_r.tick_params(colors='#f78166', labelsize=8)
    ax8_r.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor='#f78166').get_frame().set_facecolor(DARK)
ax8.set_xlim(0, duration)

# ── 9. BBOX AREA ────────────────────────────────────────────────
ax9 = fig.add_subplot(gs[4, 1])
style_ax(ax9, 'TARGET BBOX AREA', 'px^2')
if 'target_bbox_area' in ap.columns:
    ba = ap['target_bbox_area'].dropna()
    if len(ba) > 0:
        ax9.fill_between(ap.loc[ba.index, 't'], 0, ba, color='#58a6ff', alpha=0.15)
        ax9.plot(ap.loc[ba.index, 't'], ba, color='#58a6ff', linewidth=0.8)
        ax9.axhline(400000, color='#ffcc00', linestyle='--', alpha=0.5, label='Stop distance')
        ax9.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax9.set_xlim(0, duration)

# ── 10. MOTOR DIFFERENTIAL ──────────────────────────────────────
ax10 = fig.add_subplot(gs[5, 0])
style_ax(ax10, 'MOTOR DIFFERENTIAL (L-R = turn)', '')
if 'motor_diff' in ap.columns:
    md = ap['motor_diff'].dropna()
    if len(md) > 0:
        ax10.fill_between(ap.loc[md.index, 't'], 0, md,
                          where=md >= 0, color='#58a6ff', alpha=0.3)
        ax10.fill_between(ap.loc[md.index, 't'], 0, md,
                          where=md < 0, color='#f0883e', alpha=0.3)
        ax10.plot(ap.loc[md.index, 't'], md, color='#d2a8ff', linewidth=0.7)
        ax10.axhline(0, color=TXT, linewidth=0.3, alpha=0.4)
        shade_avoid(ax10, ap)
ax10.set_xlim(0, duration)

# ── 11. VISION STEER PULL ───────────────────────────────────────
ax11 = fig.add_subplot(gs[5, 1])
style_ax(ax11, 'VISION STEER PULL', '')
if 'vision_steer_pull' in ap.columns:
    sp = ap['vision_steer_pull'].dropna()
    if len(sp) > 0:
        ax11.plot(ap.loc[sp.index, 't'], sp, color='#d2a8ff', linewidth=0.8)
        ax11.axhline(0, color=TXT, linewidth=0.3, alpha=0.4)
ax11.set_xlim(0, duration)

# ── 12. IR SENSORS ──────────────────────────────────────────────
ax12 = fig.add_subplot(gs[6, 0])
style_ax(ax12, 'IR PROXIMITY', '')
if 'ir_left' in ap.columns:
    apl = ap['ir_left'].fillna(False).astype(float)
    apr = ap['ir_right'].fillna(False).astype(float)
    ax12.fill_between(ap['t'], 0, apl * 0.5, color='#ff3344', alpha=0.4, label='Left', step='post')
    ax12.fill_between(ap['t'], 0.5, 0.5 + apr * 0.5, color='#f0883e', alpha=0.4, label='Right', step='post')
    ax12.set_yticks([0.25, 0.75])
    ax12.set_yticklabels(['LEFT', 'RIGHT'], fontsize=8)
    ax12.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax12.set_xlim(0, duration)

# ── 13. IMU X/Y/Z COMPONENTS ───────────────────────────────────
ax13 = fig.add_subplot(gs[6, 1])
style_ax(ax13, 'ACCEL COMPONENTS', 'g')
for col, color, label in [('accel_x', '#ff3344', 'X'), ('accel_y', '#39d353', 'Y'), ('accel_z', '#58a6ff', 'Z')]:
    if col in sn.columns:
        vals = sn[col].dropna()
        if len(vals) > 0:
            ax13.plot(sn.loc[vals.index, 't'], vals, color=color, linewidth=0.7, label=label, alpha=0.8)
ax13.axhline(0, color=TXT, linewidth=0.3, alpha=0.4)
ax13.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax13.set_xlim(0, duration)

# ── 14. AVOID STATE TIMELINE (full width) ───────────────────────
ax14 = fig.add_subplot(gs[7, :])
style_ax(ax14, 'AVOID STATE TIMELINE', '')
ax14.set_xlabel('Time (s)', color=TXT, fontsize=9)
state_map = {'CLEAR': 0, 'SWEEP_WAIT': 1, 'ESCAPING': 2}
state_colors_bar = {'CLEAR': '#39d35340', 'SWEEP_WAIT': '#ffcc0070', 'ESCAPING': '#ff334470'}
if 'avoid_state' in ap.columns:
    states = ap['avoid_state'].fillna('CLEAR')
    y = states.map(state_map).fillna(0)
    for state_name, state_y in state_map.items():
        mask = (y == state_y)
        if mask.any():
            ax14.fill_between(ap['t'], state_y - 0.4, state_y + 0.4,
                              where=mask, color=state_colors_bar.get(state_name, '#333'),
                              step='post', label=state_name)
    ax14.set_yticks([0, 1, 2])
    ax14.set_yticklabels(['CLEAR', 'SWEEP', 'ESCAPE'], fontsize=8)
    ax14.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor=TXT).get_frame().set_facecolor(DARK)
ax14.set_xlim(0, duration)
ax14.set_ylim(-0.6, 2.6)

# ── SAVE ──────────────────────────────────────────────────────────
out_name = Path(paths[-1]).stem.replace('telem_', 'plot_') + '.png'
out_path = os.path.join(LOG_DIR, out_name)
fig.savefig(out_path, dpi=150, facecolor=DARK)
plt.close(fig)
print(f"\nPlot saved: {out_path}")
print(f"  ({os.path.getsize(out_path) / 1024:.0f} KB)")
