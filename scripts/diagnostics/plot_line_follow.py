#!/usr/bin/env python3
"""
plot_line_follow.py — Visualise line-follower CSV telemetry logs.

Reads the CSV files produced by follow_line.py and generates a
multi-panel plot showing steering breakdown, sensor readings,
and state timeline.

Usage:
    python3 scripts/diagnostics/plot_line_follow.py                  # latest log
    python3 scripts/diagnostics/plot_line_follow.py <path_to_csv>    # specific file
    python3 scripts/diagnostics/plot_line_follow.py --save           # save PNG instead of showing
"""

import sys, os, glob, argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")  # headless-safe; will switch to TkAgg if --show
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
except ImportError:
    print("ERROR: matplotlib is required.  Install with:  pip install matplotlib")
    sys.exit(1)


# ── locate project paths ────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
LOG_DIR      = os.path.join(PROJECT_ROOT, "rover_logs")


def find_latest_csv():
    """Return the most recent line_follow_*.csv in rover_logs/."""
    pattern = os.path.join(LOG_DIR, "line_follow_*.csv")
    files = sorted(glob.glob(pattern), key=os.path.getmtime)
    if not files:
        print(f"No line_follow_*.csv files found in {LOG_DIR}")
        sys.exit(1)
    return files[-1]


def load_csv(path):
    """Load CSV into a dict of numpy arrays."""
    import csv
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print(f"CSV is empty: {path}")
        sys.exit(1)

    data = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            v = r[key]
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(v)
        if isinstance(vals[0], float):
            data[key] = np.array(vals)
        else:
            data[key] = vals
    return data


STATE_COLORS = {
    "SEARCHING":     "#f59e0b",
    "LINE_DETECTED": "#22c55e",
    "FOLLOWING":     "#3b82f6",
    "LINE_LOST":     "#ef4444",
    "STOPPED":       "#6b7280",
}


def plot(data, csv_path, save_path=None):
    """Generate a 4-panel figure from the telemetry data."""
    t = data["time"]
    duration = t[-1] - t[0] if len(t) > 1 else 1.0

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True,
                              gridspec_kw={"hspace": 0.15})
    fig.suptitle(f"Line Follower Telemetry — {os.path.basename(csv_path)}",
                 fontsize=13, fontweight="bold")

    # ── Panel 1: Steering breakdown ──────────────────
    ax = axes[0]
    ax.plot(t, data["vis_steer"],   label="Vision",  color="#22c55e", linewidth=1.2)
    ax.plot(t, data["gyro_corr"],   label="Gyro corr", color="#f97316", linewidth=0.9, alpha=0.8)
    ax.plot(t, data["enc_corr"],    label="Enc corr",  color="#a855f7", linewidth=0.9, alpha=0.8)
    ax.plot(t, data["total_steer"], label="Total",    color="#ffffff", linewidth=1.5, alpha=0.9)
    ax.axhline(0, color="#555", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Steering (°)")
    ax.legend(loc="upper right", fontsize=8, ncol=4)
    ax.set_title("Steering Components", fontsize=10)

    # ── Panel 2: Vision error + line detected ────────
    ax = axes[1]
    found = data["found"]
    error = data["error"]
    # Shade regions where line is lost
    lost_mask = found < 0.5
    ax.fill_between(t, -200, 200, where=lost_mask,
                    color="#ef4444", alpha=0.15, label="No line")
    ax.plot(t, error, color="#3b82f6", linewidth=1, label="Error (px)")
    ax.axhline(0, color="#555", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Centroid Error (px)")
    ax.set_ylim(-CAMERA_W // 2, CAMERA_W // 2)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Vision: Line Centroid Error", fontsize=10)

    # ── Panel 3: Sensors (gyro + RPM) ────────────────
    ax = axes[2]
    ax2 = ax.twinx()
    ln1 = ax.plot(t, data["gyro_z"], color="#f97316", linewidth=1, label="Gyro Z (°/s)")
    ax.axhline(0, color="#555", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Gyro Z (°/s)", color="#f97316")
    ax.tick_params(axis="y", labelcolor="#f97316")

    ln2 = ax2.plot(t, data["rpm_l"], color="#22d3ee", linewidth=0.9, label="RPM Left")
    ln3 = ax2.plot(t, data["rpm_r"], color="#e879f9", linewidth=0.9, label="RPM Right")
    ax2.set_ylabel("Wheel RPM", color="#e879f9")
    ax2.tick_params(axis="y", labelcolor="#e879f9")

    lns = ln1 + ln2 + ln3
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, loc="upper right", fontsize=8, ncol=3)
    ax.set_title("Sensors: Gyro + Encoders", fontsize=10)

    # ── Panel 4: State timeline  ─────────────────────
    ax = axes[3]
    states = data["state"] if "state" in data else ["FOLLOWING"] * len(t)
    # Build colour-coded bar segments
    prev_s = states[0]
    seg_start = t[0]
    for i in range(1, len(states)):
        if states[i] != prev_s or i == len(states) - 1:
            col = STATE_COLORS.get(prev_s, "#6b7280")
            ax.axvspan(seg_start, t[i], color=col, alpha=0.5)
            seg_start = t[i]
            prev_s = states[i]
    # Final segment
    col = STATE_COLORS.get(prev_s, "#6b7280")
    ax.axvspan(seg_start, t[-1], color=col, alpha=0.5)

    ax.set_yticks([])
    ax.set_xlabel("Time (s)")
    ax.set_title("State Timeline", fontsize=10)
    # Legend
    patches = [Patch(color=c, label=s, alpha=0.6) for s, c in STATE_COLORS.items()]
    ax.legend(handles=patches, loc="upper right", fontsize=7, ncol=5)

    # ── Styling ──
    for a in axes:
        a.set_facecolor("#1a1a2e")
        a.grid(True, alpha=0.2)
        a.tick_params(colors="#ccc")
        for spine in a.spines.values():
            spine.set_color("#444")
    fig.patch.set_facecolor("#0f0f1a")
    fig.subplots_adjust(left=0.07, right=0.93, top=0.93, bottom=0.06)

    # ── Summary stats ──
    if "total_steer" in data and len(data["total_steer"]) > 10:
        s = data["total_steer"]
        follow_mask = np.array([st == "FOLLOWING" for st in states])
        if follow_mask.any():
            fs = s[follow_mask]
            fig.text(0.5, 0.01,
                     f"Following: {follow_mask.sum()} samples | "
                     f"Steer mean={fs.mean():+.1f}° std={fs.std():.1f}° | "
                     f"Duration ≈ {duration:.1f}s",
                     ha="center", fontsize=9, color="#9ca3af")

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"📊 Saved: {save_path}")
    else:
        print("📊 Displaying plot...")
        plt.show()


# ── Half-frame width for error axis limits ──
CAMERA_W = 640


def main():
    parser = argparse.ArgumentParser(description="Visualise line-follower telemetry")
    parser.add_argument("csv", nargs="?", default=None, help="Path to CSV log")
    parser.add_argument("--save", action="store_true",
                        help="Save PNG next to CSV instead of displaying")
    parser.add_argument("--show", action="store_true",
                        help="Force interactive display (requires X11/display)")
    args = parser.parse_args()

    csv_path = args.csv or find_latest_csv()
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)
    print(f"Loading: {csv_path}")

    if args.show:
        matplotlib.use("TkAgg")

    data = load_csv(csv_path)

    save_path = None
    if args.save or not args.show:
        save_path = csv_path.replace(".csv", ".png")

    plot(data, csv_path, save_path=save_path)


if __name__ == "__main__":
    main()
