#!/usr/bin/env python3
"""
DEBUG LOG VISUALISER
====================
Reads a debug CSV produced by  python3 main.py --debug=true
and plots a full-page dashboard covering every sensor channel.

Usage
-----
  python3 plot_debug_log.py                         # latest debug_*.csv
  python3 plot_debug_log.py rover_logs/debug_X.csv  # specific file
  python3 plot_debug_log.py --all                   # overlay all debug logs
  python3 plot_debug_log.py --show                  # display window instead of saving

Output
------
  PNG saved alongside the CSV as  debug_YYYYMMDD_HHMMSS_plot.png
  (or shown in a window with --show)
"""

import sys
import os
import glob
import textwrap
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection

# ── headless by default; --show switches to interactive ───────────
SHOW = "--show" in sys.argv
if not SHOW:
    matplotlib.use("Agg")

# ── project paths ─────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
LOG_DIR = os.path.join(PROJECT_ROOT, "rover_logs")


# ══════════════════════════════════════════════════════════════════
# FILE DISCOVERY
# ══════════════════════════════════════════════════════════════════

def find_logs(argv):
    if "--all" in argv:
        return sorted(glob.glob(os.path.join(LOG_DIR, "debug_*.csv")))
    for a in argv[1:]:
        if a.startswith("--"):
            continue
        if os.path.isfile(a):
            return [a]
        cand = os.path.join(LOG_DIR, a)
        if os.path.isfile(cand):
            return [cand]
    # default: latest debug log
    files = sorted(glob.glob(os.path.join(LOG_DIR, "debug_*.csv")))
    return files[-1:] if files else []


paths = find_logs(sys.argv)
if not paths:
    print("❌  No debug logs found.")
    print(f"   Looking in: {LOG_DIR}/debug_*.csv")
    print("   Run the car with:  python3 main.py --debug=true")
    sys.exit(1)

print(f"📂  Loading {len(paths)} log(s)…")


# ══════════════════════════════════════════════════════════════════
# LOAD & CLEAN
# ══════════════════════════════════════════════════════════════════

frames = []
for p in paths:
    try:
        df = pd.read_csv(p, parse_dates=["timestamp"])
        df["_src"] = Path(p).stem
        frames.append(df)
        print(f"   ✅  {Path(p).name}  ({len(df)} rows)")
    except Exception as e:
        print(f"   ⚠️   {Path(p).name}: {e}")

if not frames:
    print("❌  All files failed to load.")
    sys.exit(1)

raw = pd.concat(frames, ignore_index=True).sort_values("timestamp").reset_index(drop=True)

# Numeric time axis in seconds from start
raw["t"] = (raw["timestamp"] - raw["timestamp"].iloc[0]).dt.total_seconds()

# Coerce every numeric-ish column
_NUM_COLS = [
    "current_pwm", "steer_angle", "user_steer_angle",
    "rpm_rear_right", "rpm_rear_left", "rpm_front_right",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z", "temp_c",
    "mag_x", "mag_y", "mag_z",
    "compass_heading", "compass_target_heading", "pid_correction",
    "laser_distance_cm", "laser_raw_mm",
    "battery_voltage", "current_amps",
    "power_limiter_max_duty", "power_limiter_l298n_drop",
    "duty_rl", "duty_rr",
]
for c in _NUM_COLS:
    if c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors="coerce")

# Boolean columns (stored as 0/1 ints)
for c in ("gas_pressed", "brake_pressed", "is_braking",
          "autonomous_mode", "hunter_mode", "emergency_brake_active"):
    if c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors="coerce").fillna(0).astype(int)

duration = raw["t"].iloc[-1]
n_rows   = len(raw)
sample_hz = n_rows / max(duration, 0.001)

# ── Derived ──────────────────────────────────────────────────────
raw["accel_mag"]  = np.sqrt(raw["accel_x"]**2 + raw["accel_y"]**2 + raw["accel_z"]**2)
raw["rpm_avg"]    = (raw["rpm_rear_right"] + raw["rpm_rear_left"]) / 2.0
raw["rpm_drift"]  = raw["rpm_rear_right"] - raw["rpm_rear_left"]
raw["duty_avg_l"] = (raw["duty_fl"] + raw["duty_rl"]) / 2.0
raw["duty_avg_r"] = (raw["duty_fr"] + raw["duty_rr"]) / 2.0
raw["duty_diff"]  = raw["duty_avg_r"] - raw["duty_avg_l"]

# ── Segment masks ────────────────────────────────────────────────
moving   = raw["current_pwm"] > 1.0
gas_on   = raw["gas_pressed"]  == 1
braking  = raw["is_braking"]   == 1
fwd_gear = raw["gear"].isin(["1", "2", "3", "S"])
rev_gear = raw["gear"] == "R"
# Detect unexpected reverse while gas + forward gear
bad_rev  = gas_on & fwd_gear & rev_gear   # should always be zero after fix


# ══════════════════════════════════════════════════════════════════
# CONSOLE SUMMARY
# ══════════════════════════════════════════════════════════════════

print()
print("=" * 68)
print("  DEBUG LOG SUMMARY")
print("=" * 68)
print(f"  Duration : {duration:.1f} s  |  {n_rows} rows  |  ~{sample_hz:.0f} Hz")
print(f"  Moving   : {moving.sum()} rows  ({100*moving.mean():.0f}%)")

gear_dist = raw["gear"].value_counts().to_dict()
print(f"  Gears    : {gear_dist}")

obs_dist = raw["obstacle_state"].value_counts().to_dict()
print(f"  Obstacle : {obs_dist}")

# Check for unexpected reversals
bad_count = bad_rev.sum()
if bad_count:
    print(f"  ⚠️  UNEXPECTED REVERSAL: {bad_count} rows with gas+fwdGear but gear='R'")
else:
    print(f"  ✅  No unexpected reversals detected")

# RPM sync
rpm_m = raw[moving]
if len(rpm_m) > 10:
    drift = rpm_m["rpm_drift"].dropna()
    print(f"  RPM drift (RR-RL): mean={drift.mean():.1f}  std={drift.std():.1f}  "
          f"max_abs={drift.abs().max():.1f}")

# Laser events
if "laser_distance_cm" in raw.columns:
    ls = raw["laser_distance_cm"].dropna()
    ls_valid = ls[ls > 0]
    if len(ls_valid):
        print(f"  Laser cm : min={ls_valid.min():.0f}  mean={ls_valid.mean():.0f}  "
              f"max={ls_valid.max():.0f}")
        close_count = (ls_valid < 15).sum()
        if close_count:
            print(f"  ⚠️  Laser < 15 cm detected: {close_count} times")

# Battery
bv = raw["battery_voltage"].dropna()
bv = bv[bv > 0]
if len(bv):
    print(f"  Battery  : start={bv.iloc[0]:.2f}V  end={bv.iloc[-1]:.2f}V  "
          f"drop={bv.iloc[0]-bv.iloc[-1]:.3f}V")

# Current
ca = raw["current_amps"].dropna()
ca = ca[ca >= 0]
if len(ca):
    print(f"  Current  : mean={ca.mean():.2f}A  peak={ca.max():.2f}A")

# Compass coverage
if "compass_heading" in raw.columns:
    ch = raw["compass_heading"].dropna()
    if len(ch):
        print(f"  Compass  : min={ch.min():.1f}°  max={ch.max():.1f}°")

# PID correction
if "pid_correction" in raw.columns:
    pc = raw["pid_correction"].dropna()
    active_pc = pc[pc.abs() > 0.1]
    if len(active_pc):
        print(f"  PID corr : mean={active_pc.mean():.3f}  "
              f"max_abs={active_pc.abs().max():.3f}  "
              f"active={100*len(active_pc)/max(len(pc),1):.0f}%")

print("=" * 68)

# ══════════════════════════════════════════════════════════════════
# THEME
# ══════════════════════════════════════════════════════════════════

DARK   = "#0d1117"
GRID   = "#21262d"
TXT    = "#c9d1d9"
BLUE   = "#58a6ff"
GREEN  = "#3fb950"
ORANGE = "#d29922"
RED    = "#f85149"
PURPLE = "#bc8cff"
CYAN   = "#39d2c0"
YELLOW = "#e3b341"
PINK   = "#ff7b72"
LITE   = "#8b949e"


def _style(ax, title, ylabel=""):
    ax.set_facecolor(DARK)
    ax.grid(color=GRID, linewidth=0.6)
    ax.tick_params(colors=TXT, labelsize=7)
    ax.xaxis.label.set_color(TXT)
    ax.yaxis.label.set_color(TXT)
    for spine in ax.spines.values():
        spine.set_edgecolor(GRID)
    ax.set_title(title, color=TXT, fontsize=8, fontweight="bold", pad=3)
    if ylabel:
        ax.set_ylabel(ylabel, color=TXT, fontsize=7)


def _shade_gas(ax, alpha=0.08):
    """Light fill where gas is pressed."""
    segs = _spans(raw["t"], gas_on)
    for (x0, x1) in segs:
        ax.axvspan(x0, x1, color=GREEN, alpha=alpha, linewidth=0)


def _shade_brake(ax, alpha=0.12):
    segs = _spans(raw["t"], braking)
    for (x0, x1) in segs:
        ax.axvspan(x0, x1, color=RED, alpha=alpha, linewidth=0)


def _spans(t_series, mask_series):
    """Return list of (t_start, t_end) for contiguous True runs in mask."""
    spans = []
    in_span = False
    t0 = None
    for t, m in zip(t_series, mask_series):
        if m and not in_span:
            t0 = t
            in_span = True
        elif not m and in_span:
            spans.append((t0, t))
            in_span = False
    if in_span:
        spans.append((t0, t))
    return spans


def _gear_colours(raw):
    """Map gear string to an RGBA array for scatter colouring."""
    gmap = {"1": CYAN, "2": BLUE, "3": ORANGE, "S": RED,
            "R": PURPLE, "N": LITE, "?": GRID}
    return [gmap.get(str(g), GRID) for g in raw["gear"].fillna("?")]


# ══════════════════════════════════════════════════════════════════
# FIGURE LAYOUT  (10 rows × 2 columns)
# ══════════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(22, 36), facecolor=DARK)
fig.suptitle(
    f"Debug Log — {Path(paths[-1]).stem}   [{duration:.1f} s  |  "
    f"{n_rows} rows  |  ~{sample_hz:.0f} Hz]",
    color=TXT, fontsize=11, fontweight="bold", y=0.995,
)
gs = GridSpec(
    10, 2,
    figure=fig,
    hspace=0.55, wspace=0.3,
    left=0.06, right=0.97, top=0.985, bottom=0.03,
)

t = raw["t"]


# ─────────────────────────────────────────────────────────────────
# Row 0 — Drive overview (spans full width)
# ─────────────────────────────────────────────────────────────────
ax0 = fig.add_subplot(gs[0, :])
_style(ax0, "Drive Overview — Throttle PWM, Gear & Braking")
ax0.plot(t, raw["current_pwm"], color=GREEN, linewidth=0.9, label="PWM %")
# Gear as coloured background bands
gear_colours = _gear_colours(raw)
ax0.scatter(t, raw["current_pwm"], c=gear_colours, s=1.5, zorder=3, label="Gear colour")
_shade_brake(ax0, alpha=0.18)
# Highlight unexpected reverse (should be empty after fix)
if bad_rev.any():
    ax0.scatter(t[bad_rev], raw.loc[bad_rev, "current_pwm"],
                color=RED, s=20, zorder=5, marker="x", label="⚠️ bad reverse")
ax0.set_ylim(-5, 105)
ax0.set_ylabel("Duty %", color=TXT, fontsize=7)
# Legend
patches = [
    mpatches.Patch(color=CYAN,   label="Gear 1"),
    mpatches.Patch(color=BLUE,   label="Gear 2"),
    mpatches.Patch(color=ORANGE, label="Gear 3"),
    mpatches.Patch(color=RED,    label="Gear S"),
    mpatches.Patch(color=PURPLE, label="Gear R"),
    mpatches.Patch(color=LITE,   label="Gear N"),
    mpatches.Patch(color=RED, alpha=0.3, label="Braking"),
]
ax0.legend(handles=patches, loc="upper right", fontsize=6,
           facecolor=GRID, labelcolor=TXT, framealpha=0.8)


# ─────────────────────────────────────────────────────────────────
# Row 1 — Steering
# ─────────────────────────────────────────────────────────────────
ax1a = fig.add_subplot(gs[1, 0])
_style(ax1a, "Steering — User Angle")
ax1a.plot(t, raw["user_steer_angle"], color=BLUE, linewidth=0.8, label="user")
ax1a.plot(t, raw["steer_angle"],      color=CYAN, linewidth=0.8, alpha=0.7, label="applied")
ax1a.axhline(0, color=GRID, linewidth=0.5)
ax1a.set_ylim(-95, 95)
ax1a.set_ylabel("°", color=TXT, fontsize=7)
ax1a.legend(fontsize=6, facecolor=GRID, labelcolor=TXT)

ax1b = fig.add_subplot(gs[1, 1])
_style(ax1b, "Per-Wheel Duty Cycles")
ax1b.plot(t, raw["duty_fl"], color=BLUE,   linewidth=0.7, alpha=0.8, label="FL")
ax1b.plot(t, raw["duty_fr"], color=GREEN,  linewidth=0.7, alpha=0.8, label="FR")
ax1b.plot(t, raw["duty_rl"], color=ORANGE, linewidth=0.7, alpha=0.8, label="RL")
ax1b.plot(t, raw["duty_rr"], color=RED,    linewidth=0.7, alpha=0.8, label="RR")
ax1b.set_ylabel("Duty %", color=TXT, fontsize=7)
ax1b.legend(fontsize=6, facecolor=GRID, labelcolor=TXT, ncol=2)


# ─────────────────────────────────────────────────────────────────
# Row 2 — Encoders / RPM
# ─────────────────────────────────────────────────────────────────
ax2a = fig.add_subplot(gs[2, 0])
_style(ax2a, "Wheel RPM — Rear Left & Right")
ax2a.plot(t, raw["rpm_rear_right"], color=CYAN,   linewidth=0.8, label="RR")
ax2a.plot(t, raw["rpm_rear_left"],  color=ORANGE, linewidth=0.8, label="RL")
ax2a.plot(t, raw["rpm_front_right"],color=PURPLE, linewidth=0.8, alpha=0.6, label="FR")
ax2a.set_ylabel("RPM", color=TXT, fontsize=7)
ax2a.legend(fontsize=6, facecolor=GRID, labelcolor=TXT)
_shade_gas(ax2a)

ax2b = fig.add_subplot(gs[2, 1])
_style(ax2b, "RPM Drift (RR − RL)")
ax2b.fill_between(t, raw["rpm_drift"], 0,
                  where=raw["rpm_drift"] >= 0, color=CYAN,   alpha=0.5, linewidth=0)
ax2b.fill_between(t, raw["rpm_drift"], 0,
                  where=raw["rpm_drift"] <  0, color=ORANGE, alpha=0.5, linewidth=0)
ax2b.axhline(0, color=TXT, linewidth=0.5)
ax2b.set_ylabel("RPM diff", color=TXT, fontsize=7)


# ─────────────────────────────────────────────────────────────────
# Row 3 — MPU6500 Accelerometer
# ─────────────────────────────────────────────────────────────────
ax3a = fig.add_subplot(gs[3, 0])
_style(ax3a, "Accelerometer (g)")
ax3a.plot(t, raw["accel_x"], color=RED,    linewidth=0.7, label="X")
ax3a.plot(t, raw["accel_y"], color=GREEN,  linewidth=0.7, label="Y")
ax3a.plot(t, raw["accel_z"], color=BLUE,   linewidth=0.7, label="Z")
ax3a.plot(t, raw["accel_mag"], color=YELLOW, linewidth=0.9, alpha=0.7, label="|a|")
ax3a.set_ylabel("g", color=TXT, fontsize=7)
ax3a.legend(fontsize=6, facecolor=GRID, labelcolor=TXT, ncol=2)

ax3b = fig.add_subplot(gs[3, 1])
_style(ax3b, "Gyroscope (°/s)")
ax3b.plot(t, raw["gyro_x"], color=RED,   linewidth=0.7, label="X")
ax3b.plot(t, raw["gyro_y"], color=GREEN, linewidth=0.7, label="Y")
ax3b.plot(t, raw["gyro_z"], color=BLUE,  linewidth=0.8, label="Z (yaw)")
ax3b.axhline(0, color=GRID, linewidth=0.5)
ax3b.set_ylabel("°/s", color=TXT, fontsize=7)
ax3b.legend(fontsize=6, facecolor=GRID, labelcolor=TXT)


# ─────────────────────────────────────────────────────────────────
# Row 4 — Magnetometer + Temperature
# ─────────────────────────────────────────────────────────────────
ax4a = fig.add_subplot(gs[4, 0])
_style(ax4a, "Magnetometer (Gauss)")
ax4a.plot(t, raw["mag_x"], color=RED,   linewidth=0.7, label="X")
ax4a.plot(t, raw["mag_y"], color=GREEN, linewidth=0.7, label="Y")
ax4a.plot(t, raw["mag_z"], color=BLUE,  linewidth=0.7, label="Z")
ax4a.set_ylabel("Gauss", color=TXT, fontsize=7)
ax4a.legend(fontsize=6, facecolor=GRID, labelcolor=TXT)

ax4b = fig.add_subplot(gs[4, 1])
_style(ax4b, "IMU Temperature (°C)")
ax4b.plot(t, raw["temp_c"], color=ORANGE, linewidth=0.9)
ax4b.set_ylabel("°C", color=TXT, fontsize=7)


# ─────────────────────────────────────────────────────────────────
# Row 5 — Compass heading & PID
# ─────────────────────────────────────────────────────────────────
ax5a = fig.add_subplot(gs[5, 0])
_style(ax5a, "Compass Heading (°)")
ax5a.plot(t, raw["compass_heading"],        color=CYAN,   linewidth=0.9, label="current")
ax5a.plot(t, raw["compass_target_heading"], color=ORANGE, linewidth=0.8,
          alpha=0.7, linestyle="--", label="target")
ax5a.set_ylim(0, 365)
ax5a.set_ylabel("°", color=TXT, fontsize=7)
ax5a.legend(fontsize=6, facecolor=GRID, labelcolor=TXT)
_shade_gas(ax5a)

ax5b = fig.add_subplot(gs[5, 1])
_style(ax5b, "Compass PID Correction (%)")
ax5b.fill_between(t, raw["pid_correction"], 0,
                  where=raw["pid_correction"] >= 0, color=BLUE,   alpha=0.5, linewidth=0)
ax5b.fill_between(t, raw["pid_correction"], 0,
                  where=raw["pid_correction"] <  0, color=ORANGE, alpha=0.5, linewidth=0)
ax5b.axhline(0, color=TXT, linewidth=0.5)
ax5b.set_ylabel("% correction", color=TXT, fontsize=7)


# ─────────────────────────────────────────────────────────────────
# Row 6 — Laser sensor
# ─────────────────────────────────────────────────────────────────
ax6 = fig.add_subplot(gs[6, :])
_style(ax6, "Laser Distance (cm)  — thresholds: 15 cm stop / 25 crawl / 40 slow / 60 caution")
laser_cm = raw["laser_distance_cm"].copy()
laser_cm[laser_cm <= 0] = np.nan  # mask invalid readings
ax6.plot(t, laser_cm, color=GREEN, linewidth=0.9, label="laser cm")
ax6.axhline(15, color=RED,    linewidth=0.8, linestyle="--", alpha=0.7, label="stop (15)")
ax6.axhline(25, color=ORANGE, linewidth=0.8, linestyle="--", alpha=0.7, label="crawl (25)")
ax6.axhline(40, color=YELLOW, linewidth=0.8, linestyle="--", alpha=0.7, label="slow (40)")
ax6.axhline(60, color=CYAN,   linewidth=0.8, linestyle="--", alpha=0.7, label="caution (60)")
ax6.set_ylabel("cm", color=TXT, fontsize=7)
ax6.legend(fontsize=6, facecolor=GRID, labelcolor=TXT, ncol=3)


# ─────────────────────────────────────────────────────────────────
# Row 7 — Battery & Current
# ─────────────────────────────────────────────────────────────────
ax7a = fig.add_subplot(gs[7, 0])
_style(ax7a, "Battery Voltage (V)")
bv = raw["battery_voltage"].copy()
bv[bv <= 0] = np.nan
ax7a.plot(t, bv, color=YELLOW, linewidth=0.9)
ax7a.set_ylabel("V", color=TXT, fontsize=7)

ax7b = fig.add_subplot(gs[7, 1])
_style(ax7b, "Motor Current (A)  +  Power Limiter Max Duty (%)")
ca = raw["current_amps"].copy()
ca[ca < 0] = np.nan
ax7b.plot(t, ca, color=ORANGE, linewidth=0.9, label="current A")
ax7b2 = ax7b.twinx()
ax7b2.set_facecolor(DARK)
ax7b2.tick_params(colors=TXT, labelsize=7)
ax7b2.plot(t, raw["power_limiter_max_duty"], color=PURPLE,
           linewidth=0.8, alpha=0.8, linestyle=":", label="max duty %")
ax7b2.set_ylabel("Max duty %", color=TXT, fontsize=7)
ax7b.set_ylabel("A", color=TXT, fontsize=7)
lines1, labels1 = ax7b.get_legend_handles_labels()
lines2, labels2 = ax7b2.get_legend_handles_labels()
ax7b.legend(lines1 + lines2, labels1 + labels2,
            fontsize=6, facecolor=GRID, labelcolor=TXT)


# ─────────────────────────────────────────────────────────────────
# Row 8 — Mode flags (binary event lanes)
# ─────────────────────────────────────────────────────────────────
ax8 = fig.add_subplot(gs[8, :])
_style(ax8, "Mode Flags (event lanes)")
flag_cols = [
    ("gas_pressed",           GREEN,  0.85, "Gas"),
    ("brake_pressed",         RED,    0.65, "Brake"),
    ("is_braking",            ORANGE, 0.45, "H-Bridge Brake"),
    ("autonomous_mode",       BLUE,   0.25, "Autonomous"),
    ("emergency_brake_active",RED,    0.05, "E-Brake"),
]
y_ticks, y_labels = [], []
for col, colour, y_base, label in flag_cols:
    if col not in raw.columns:
        continue
    vals = raw[col].fillna(0)
    ax8.fill_between(t, y_base, y_base + 0.18,
                     where=vals == 1,
                     color=colour, alpha=0.75, linewidth=0, step="post")
    y_ticks.append(y_base + 0.09)
    y_labels.append(label)
ax8.set_yticks(y_ticks)
ax8.set_yticklabels(y_labels, fontsize=7)
ax8.set_ylim(0, 1.05)

# Gear state as text markers along the top
gear_changes = raw["gear"].ne(raw["gear"].shift())
for idx in raw.index[gear_changes]:
    g = raw.loc[idx, "gear"]
    ax8.text(raw.loc[idx, "t"], 0.97, str(g),
             color=TXT, fontsize=6, va="top", ha="left", alpha=0.8)


# ─────────────────────────────────────────────────────────────────
# Row 9 — Magnetometer XY scatter (heading rose)  +  Accel scatter
# ─────────────────────────────────────────────────────────────────
ax9a = fig.add_subplot(gs[9, 0], aspect="equal")
ax9a.set_facecolor(DARK)
ax9a.grid(color=GRID, linewidth=0.6)
ax9a.tick_params(colors=TXT, labelsize=7)
for sp in ax9a.spines.values():
    sp.set_edgecolor(GRID)
ax9a.set_title("Magnetometer XY — calibration / hard-iron check",
               color=TXT, fontsize=8, fontweight="bold", pad=3)
sc = ax9a.scatter(raw["mag_x"], raw["mag_y"],
                  c=t, cmap="plasma", s=1.5, alpha=0.6)
ax9a.set_xlabel("mag_x (G)", color=TXT, fontsize=7)
ax9a.set_ylabel("mag_y (G)", color=TXT, fontsize=7)
cb = fig.colorbar(sc, ax=ax9a, pad=0.02, fraction=0.05)
cb.ax.tick_params(colors=TXT, labelsize=6)
cb.set_label("time (s)", color=TXT, fontsize=6)

ax9b = fig.add_subplot(gs[9, 1], aspect="equal")
ax9b.set_facecolor(DARK)
ax9b.grid(color=GRID, linewidth=0.6)
ax9b.tick_params(colors=TXT, labelsize=7)
for sp in ax9b.spines.values():
    sp.set_edgecolor(GRID)
ax9b.set_title("Accel XY scatter (lateral vs. longitudinal)",
               color=TXT, fontsize=8, fontweight="bold", pad=3)
ax9b.scatter(raw["accel_x"], raw["accel_y"],
             c=t, cmap="viridis", s=1.5, alpha=0.6)
ax9b.set_xlabel("accel_x (g)", color=TXT, fontsize=7)
ax9b.set_ylabel("accel_y (g)", color=TXT, fontsize=7)


# ─────────────────────────────────────────────────────────────────
# Shared X-label
# ─────────────────────────────────────────────────────────────────
for ax in fig.axes:
    if hasattr(ax, "_get_lines"):          # is a real axis
        ax.set_xlabel("Time (s)", color=TXT, fontsize=7)


# ══════════════════════════════════════════════════════════════════
# SAVE OR SHOW
# ══════════════════════════════════════════════════════════════════

if SHOW:
    plt.show()
else:
    out_path = str(paths[-1]).replace(".csv", "_plot.png")
    fig.savefig(out_path, dpi=130, bbox_inches="tight", facecolor=DARK)
    print(f"\n✅  Plot saved → {out_path}")
    plt.close(fig)
