#!/usr/bin/env python3
"""
plot_pantilt_log.py — Plot pan-tilt tracker telemetry CSV logs.

Usage:
    python3 scripts/plot_pantilt_log.py                         # latest log
    python3 scripts/plot_pantilt_log.py rover_logs/pantilt_track_20260308_120000.csv
"""

import argparse
import glob
import os
import sys

import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless-safe; switch to "TkAgg" if you want interactive
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, "rover_logs")


def find_latest_log():
    pattern = os.path.join(LOG_DIR, "pantilt_track_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print("No pantilt_track_*.csv logs found in", LOG_DIR)
        sys.exit(1)
    return files[-1]


def load(path):
    df = pd.read_csv(path)
    df["elapsed_s"] = pd.to_numeric(df["elapsed_s"], errors="coerce")
    for col in ["fps", "track_cx", "track_cy", "err_x", "err_y",
                 "int_x", "int_y", "pan_deg", "tilt_deg", "dp", "dt",
                 "bbox_x", "bbox_y", "bbox_w", "bbox_h", "tracker_lost"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def plot(df, csv_path):
    t = df["elapsed_s"]
    basename = os.path.basename(csv_path)

    fig = plt.figure(figsize=(16, 14), constrained_layout=True)
    fig.suptitle(f"Pan-Tilt Tracker Log — {basename}", fontsize=13, y=1.0)
    gs = GridSpec(5, 2, figure=fig)

    # 1. FPS
    ax = fig.add_subplot(gs[0, :])
    ax.plot(t, df["fps"], color="tab:blue", lw=0.8)
    ax.set_ylabel("FPS")
    ax.set_title("Frame Rate")
    ax.axhline(30, ls="--", color="gray", lw=0.5) ; ax.axhline(60, ls="--", color="green", lw=0.5)
    ax.set_xlim(t.iloc[0], t.iloc[-1])

    # 2. Pan / Tilt angles
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t, df["pan_deg"], label="Pan", color="tab:orange", lw=0.8)
    ax.axhline(90, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Degrees")
    ax.set_title("Pan Angle")
    ax.set_ylim(55, 125)
    ax.legend(loc="upper right", fontsize=8)

    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t, df["tilt_deg"], label="Tilt", color="tab:red", lw=0.8)
    ax.axhline(90, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Degrees")
    ax.set_title("Tilt Angle")
    ax.set_ylim(55, 125)
    ax.legend(loc="upper right", fontsize=8)

    # 3. Tracking error (smoothed)
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, df["err_x"], color="tab:cyan", lw=0.7, label="err_x")
    ax.axhline(0, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Error")
    ax.set_title("Smoothed Error X (pan)")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[2, 1])
    ax.plot(t, df["err_y"], color="tab:pink", lw=0.7, label="err_y")
    ax.axhline(0, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Error")
    ax.set_title("Smoothed Error Y (tilt)")
    ax.legend(fontsize=8)

    # 4. PID output (dp, dt)
    ax = fig.add_subplot(gs[3, 0])
    ax.plot(t, df["dp"], color="tab:purple", lw=0.7, label="dp (pan)")
    ax.axhline(0, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Step (deg)")
    ax.set_title("PID Output — Pan")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[3, 1])
    ax.plot(t, df["dt"], color="tab:brown", lw=0.7, label="dt (tilt)")
    ax.axhline(0, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Step (deg)")
    ax.set_title("PID Output — Tilt")
    ax.legend(fontsize=8)

    # 5. Target position (normalised) + mode
    ax = fig.add_subplot(gs[4, 0])
    ax.plot(t, df["track_cx"], color="tab:green", lw=0.7, label="cx")
    ax.plot(t, df["track_cy"], color="tab:olive", lw=0.7, label="cy")
    ax.axhline(0.5, ls=":", color="gray", lw=0.5)
    ax.set_ylabel("Normalised Pos")
    ax.set_xlabel("Elapsed (s)")
    ax.set_title("Target Centre (normalised)")
    ax.legend(fontsize=8)

    # Mode bar
    ax2 = fig.add_subplot(gs[4, 1])
    tracking_mask = df["mode"] == "TRACKING"
    ax2.fill_between(t, 0, tracking_mask.astype(int), step="post",
                     alpha=0.4, color="lime", label="TRACKING")
    ax2.set_ylabel("Active")
    ax2.set_xlabel("Elapsed (s)")
    ax2.set_title("Tracking Active")
    ax2.set_ylim(-0.05, 1.1)
    if "tracker_lost" in df.columns:
        ax2t = ax2.twinx()
        ax2t.plot(t, df["tracker_lost"], color="red", lw=0.6, alpha=0.6, label="lost frames")
        ax2t.set_ylabel("Lost frames", color="red")
    ax2.legend(fontsize=8)

    out_path = csv_path.replace(".csv", ".png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot → {out_path}")
    plt.close(fig)
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Plot pan-tilt tracker logs")
    parser.add_argument("csv", nargs="?", default=None,
                        help="Path to CSV log (default: latest)")
    args = parser.parse_args()

    csv_path = args.csv or find_latest_log()
    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    print(f"Loading {csv_path} …")
    df = load(csv_path)
    print(f"  {len(df)} rows, {df['elapsed_s'].iloc[-1]:.1f}s duration")
    plot(df, csv_path)


if __name__ == "__main__":
    main()
