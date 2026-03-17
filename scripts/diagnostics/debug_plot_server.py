#!/usr/bin/env python3
"""
INTERACTIVE DEBUG TELEMETRY SERVER
====================================
Launched automatically when main.py is started with --debug=true.
Runs on port 5001 and provides a Plotly/Dash dashboard for exploring
rover telemetry logs with time-range filtering and live-reload.

Supports two CSV formats:
  - debug_YYYYMMDD_HHMMSS.csv  (records written by main.py --debug=true)
  - telem_YYYYMMDD_HHMMSS.csv  (legacy autopilot telemetry format)

Standalone usage:
    python3 debug_plot_server.py
    python3 debug_plot_server.py --port=5002
"""

import os
import sys
import glob
import threading
from pathlib import Path

import pandas as pd
import numpy as np

import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ─── Paths ────────────────────────────────────────────────────────────────────
_THIS_DIR    = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
LOG_DIR      = os.path.join(PROJECT_ROOT, "rover_logs")

DEBUG_PORT = 5001

# Path to the log file currently being written by main.py (set by start_debug_plot_server)
_active_log_path: str | None = None

# ─── Theme (matches rover_plots.py) ──────────────────────────────────────────
DARK   = "#0d1117"
CARD   = "#161b22"
GRID   = "#21262d"
TXT    = "#c9d1d9"
BLUE   = "#58a6ff"
GREEN  = "#39d353"
ORANGE = "#f0883e"
RED    = "#ff3344"
AMBER  = "#ffcc00"
PURPLE = "#d2a8ff"
GRAY   = "#8b949e"
CORAL  = "#f78166"
TEAL   = "#39c5cf"

# rgba helpers — plotly rejects 8-digit hex alpha notation
_RGBA = {
    GREEN:  "rgba(57,211,83,{a})",
    BLUE:   "rgba(88,166,255,{a})",
    ORANGE: "rgba(240,136,62,{a})",
    RED:    "rgba(255,51,68,{a})",
    AMBER:  "rgba(255,204,0,{a})",
    PURPLE: "rgba(210,168,255,{a})",
    GRAY:   "rgba(139,148,158,{a})",
    CORAL:  "rgba(247,129,102,{a})",
    TEAL:   "rgba(57,197,207,{a})",
}
def _fa(color: str, alpha: float) -> str:
    """Return an rgba() fill color string."""
    return _RGBA.get(color, f"rgba(128,128,128,{alpha})").format(a=alpha)

_PLOTLY_LAYOUT_BASE = dict(
    paper_bgcolor=DARK,
    plot_bgcolor=DARK,
    font=dict(color=TXT, size=11),
    xaxis=dict(
        gridcolor=GRID, gridwidth=0.5, zeroline=False,
        tickcolor=GRAY, linecolor=GRID,
    ),
    yaxis=dict(
        gridcolor=GRID, gridwidth=0.5, zeroline=False,
        tickcolor=GRAY, linecolor=GRID,
    ),
    margin=dict(l=48, r=16, t=40, b=36),
    hovermode="x unified",
    hoverlabel=dict(bgcolor=CARD, font_color=TXT, bordercolor=GRID),
)


# ─── CSV detection & loading ──────────────────────────────────────────────────

def _discover_logs():
    """Return sorted list of (display_name, abs_path) for all known logs."""
    debug_files = sorted(glob.glob(os.path.join(LOG_DIR, "debug_*.csv")), reverse=True)
    telem_files = sorted(glob.glob(os.path.join(LOG_DIR, "telem_*.csv")), reverse=True)
    entries = []
    for p in debug_files:
        entries.append((f"[debug] {Path(p).stem}", p))
    for p in telem_files:
        entries.append((f"[telem] {Path(p).stem}", p))
    return entries


def _is_debug_csv(df: pd.DataFrame) -> bool:
    return "duty_fl" in df.columns or "current_pwm" in df.columns


def _load_debug_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # ISO timestamps → seconds-from-start
    df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp").reset_index(drop=True)
    t0 = df["timestamp"].iloc[0]
    df["t"] = (df["timestamp"] - t0).dt.total_seconds()

    num_cols = [
        "current_pwm", "steer_angle", "user_steer_angle",
        "rpm_rear_right", "rpm_rear_left", "rpm_front_right",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z", "temp_c",
        "mag_x", "mag_y", "mag_z",
        "compass_heading", "compass_target_heading",
        "pid_correction", "heading_error_deg", "steer_heading_delta_deg",
        "laser_distance_cm", "laser_raw_mm",
        "battery_voltage", "current_amps",
        "power_limiter_max_duty", "power_limiter_l298n_drop",
        "duty_fl", "duty_fr", "duty_rl", "duty_rr",
    ]
    for c in num_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    bool_cols = ["gas_pressed", "brake_pressed", "is_braking",
                 "course_correction_active", "autonomous_mode",
                 "hunter_mode", "emergency_brake_active"]
    for c in bool_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce").fillna(0).astype(bool)

    # Derived
    for c in ["accel_x", "accel_y", "accel_z"]:
        if c not in df.columns:
            df[c] = np.nan
    df["accel_mag"] = np.sqrt(df["accel_x"]**2 + df["accel_y"]**2 + df["accel_z"]**2)

    # Average motor duty as throttle proxy
    duty_cols = [c for c in ["duty_fl", "duty_fr", "duty_rl", "duty_rr"] if c in df.columns]
    if duty_cols:
        df["avg_duty"] = df[duty_cols].abs().mean(axis=1)
        df["motor_diff"] = (
            (df.get("duty_fl", 0).fillna(0) + df.get("duty_rl", 0).fillna(0)) / 2
            - (df.get("duty_fr", 0).fillna(0) + df.get("duty_rr", 0).fillna(0)) / 2
        )

    return df


def _load_telem_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp").reset_index(drop=True)
    t0 = df["timestamp"].iloc[0]
    df["t"] = df["timestamp"] - t0

    num_cols = [
        "laser_front_cm", "sonar_rear_cm",
        "accel_x", "accel_y", "accel_z", "accel_mag", "gyro_z",
        "rpm", "battery_v", "current_a",
        "target_bbox_area", "vision_steer_pull",
        "raw_throttle", "raw_steering", "smooth_throttle", "smooth_steering",
        "motor_L", "motor_R", "sweep_esc_angle", "sweep_esc_dist", "touch_conf",
    ]
    for c in num_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    for c in ["target_found", "sweep_active"]:
        if c in df.columns:
            df[c] = df[c].map({
                "True": True, "False": False,
                "1": True, "0": False,
                True: True, False: False,
                1: True, 0: False,
            })

    if "motor_L" in df.columns and "motor_R" in df.columns:
        df["motor_diff"] = df["motor_L"] - df["motor_R"]

    return df


def load_log(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, nrows=3)
    if _is_debug_csv(df):
        return _load_debug_csv(path)
    return _load_telem_csv(path)


# ─── Plot builders ────────────────────────────────────────────────────────────

def _ax_style(fig, row, col, title, yaxis_title=""):
    fig.update_xaxes(
        gridcolor=GRID, gridwidth=0.5, zeroline=False,
        tickcolor=GRAY, linecolor=GRID,
        row=row, col=col,
    )
    fig.update_yaxes(
        title_text=yaxis_title, title_font=dict(color=GRAY, size=9),
        gridcolor=GRID, gridwidth=0.5, zeroline=False,
        tickcolor=GRAY, linecolor=GRID,
        row=row, col=col,
    )
    fig.update_annotations(
        selector=dict(text=title),
        font=dict(color=BLUE, size=12),
    )


def _hline(fig, y, color, dash="dash", row=1, col=1, name=""):
    fig.add_hline(
        y=y, line_color=color, line_dash=dash,
        line_width=0.8, opacity=0.55,
        annotation_text=name, annotation_font_color=color,
        annotation_font_size=9,
        row=row, col=col,
    )


def build_debug_figure(df: pd.DataFrame, t_range=None) -> go.Figure:
    """Build the 8-row × 2-col interactive figure for debug CSV data."""
    if t_range:
        df = df[(df["t"] >= t_range[0]) & (df["t"] <= t_range[1])].copy()

    if df.empty:
        fig = go.Figure()
        fig.update_layout(**_PLOTLY_LAYOUT_BASE, title="No data in selected range")
        return fig

    duration = df["t"].iloc[-1] - df["t"].iloc[0]

    specs = [
        [{"colspan": 2}, None],   # 1 – distance sensor  (full width)
        [{"type": "xy"},  {"type": "xy"}],  # 2 – motor duty / steering
        [{"type": "xy"},  {"type": "xy"}],  # 3 – throttle/pwm / RPM
        [{"type": "xy"},  {"type": "xy"}],  # 4 – accel mag / gyro
        [{"type": "xy"},  {"type": "xy"}],  # 5 – battery+current / compass
        [{"type": "xy"},  {"type": "xy"}],  # 6 – motor diff / PID correction
        [{"type": "xy"},  {"type": "xy"}],  # 7 – accel XYZ / temperature
        [{"colspan": 2}, None],   # 8 – obstacle state timeline (full width)
    ]
    subplot_titles = [
        "DISTANCE SENSOR", "",
        "MOTOR DUTY CYCLES (%)", "STEERING (°)",
        "THROTTLE / PWM (%)", "RPM",
        "ACCELEROMETER (magnitude)", "GYROSCOPE (deg/s)",
        "BATTERY (V) & CURRENT (A)", "COMPASS & HEADING ERROR",
        "MOTOR DIFFERENTIAL (L-R)", "PID CORRECTION",
        "ACCEL COMPONENTS (g)", "TEMPERATURE (°C)",
        "OBSTACLE STATE TIMELINE", "",
    ]

    fig = make_subplots(
        rows=8, cols=2,
        specs=specs,
        subplot_titles=subplot_titles,
        vertical_spacing=0.04,
        horizontal_spacing=0.06,
        row_heights=[0.10, 0.10, 0.10, 0.10, 0.12, 0.10, 0.10, 0.08],
    )

    t = df["t"]

    # ── Row 1: Distance Sensor ────────────────────────────────────
    if "laser_distance_cm" in df.columns:
        dist = df["laser_distance_cm"].clip(upper=400)
        fig.add_trace(go.Scatter(
            x=t, y=dist, mode="lines",
            name="Laser Front", line=dict(color=GREEN, width=1.2),
            fill="tozeroy", fillcolor=_fa(GREEN, 0.09),
        ), row=1, col=1)
    for y_val, color, label in [
        (12,  RED,    "Stop 12cm"),
        (25,  ORANGE, "Creep 25cm"),
        (50,  AMBER,  "Slow 50cm"),
        (100, GREEN,  "Aware 100cm"),
    ]:
        fig.add_hline(y=y_val, line_color=color, line_dash="dash",
                      line_width=0.8, opacity=0.5,
                      annotation_text=label, annotation_font_color=color,
                      annotation_font_size=8, row=1, col=1)

    # ── Row 2L: Motor Duty Cycles ────────────────────────────────
    duty_map = [
        ("duty_fl", BLUE,   "FL"),
        ("duty_fr", ORANGE, "FR"),
        ("duty_rl", GREEN,  "RL"),
        ("duty_rr", PURPLE, "RR"),
    ]
    for col_name, color, label in duty_map:
        if col_name in df.columns:
            fig.add_trace(go.Scatter(
                x=t, y=df[col_name], mode="lines",
                name=label, line=dict(color=color, width=1),
                opacity=0.85,
            ), row=2, col=1)

    # ── Row 2R: Steering ─────────────────────────────────────────
    if "steer_angle" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["steer_angle"], mode="lines",
            name="Steer", line=dict(color=PURPLE, width=1.2),
        ), row=2, col=2)
    if "user_steer_angle" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["user_steer_angle"], mode="lines",
            name="User Steer", line=dict(color=GRAY, width=0.8),
            opacity=0.6,
        ), row=2, col=2)
    fig.add_hline(y=0, line_color=TXT, line_width=0.4, opacity=0.4, row=2, col=2)

    # ── Row 3L: Throttle / PWM ───────────────────────────────────
    if "current_pwm" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["current_pwm"], mode="lines",
            name="PWM %", line=dict(color=GREEN, width=1.2),
            fill="tozeroy", fillcolor=_fa(GREEN, 0.13),
        ), row=3, col=1)
    if "avg_duty" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["avg_duty"], mode="lines",
            name="Avg Duty", line=dict(color=TEAL, width=0.9),
            opacity=0.7,
        ), row=3, col=1)
    # Shade gas pressed regions
    if "gas_pressed" in df.columns:
        gas_on = df[df["gas_pressed"]]
        if not gas_on.empty:
            fig.add_trace(go.Scatter(
                x=pd.concat([gas_on["t"], gas_on["t"].iloc[::-1]]),
                y=[100] * len(gas_on) + [0] * len(gas_on),
                fill="toself", fillcolor=_fa(GREEN, 0.07),
                line=dict(width=0), name="Gas", showlegend=True,
            ), row=3, col=1)
    if "brake_pressed" in df.columns:
        brk = df[df["brake_pressed"]]
        if not brk.empty:
            fig.add_trace(go.Scatter(
                x=pd.concat([brk["t"], brk["t"].iloc[::-1]]),
                y=[100] * len(brk) + [0] * len(brk),
                fill="toself", fillcolor=_fa(RED, 0.09),
                line=dict(width=0), name="Brake", showlegend=True,
            ), row=3, col=1)

    # ── Row 3R: RPM ──────────────────────────────────────────────
    rpm_map = [
        ("rpm_rear_right", GREEN,  "RPM RR"),
        ("rpm_rear_left",  BLUE,   "RPM RL"),
        ("rpm_front_right",ORANGE, "RPM FR"),
    ]
    for col_name, color, label in rpm_map:
        if col_name in df.columns:
            fig.add_trace(go.Scatter(
                x=t, y=df[col_name], mode="lines",
                name=label, line=dict(color=color, width=1),
                opacity=0.85,
            ), row=3, col=2)

    # ── Row 4L: Accel Magnitude ──────────────────────────────────
    if "accel_mag" in df.columns:
        am = df["accel_mag"].replace(0, np.nan)
        fig.add_trace(go.Scatter(
            x=t, y=am, mode="lines",
            name="|Accel|", line=dict(color=ORANGE, width=1),
        ), row=4, col=1)
    fig.add_hline(y=1.0, line_color=GREEN,  line_dash="dash",
                  line_width=0.8, opacity=0.35, row=4, col=1)
    fig.add_hline(y=1.8, line_color=RED,    line_dash="dash",
                  line_width=0.8, opacity=0.5,  row=4, col=1)

    # ── Row 4R: Gyroscope ────────────────────────────────────────
    gyro_map = [
        ("gyro_z", PURPLE, "Gyro Z"),
        ("gyro_x", CORAL,  "Gyro X"),
        ("gyro_y", TEAL,   "Gyro Y"),
    ]
    for col_name, color, label in gyro_map:
        if col_name in df.columns:
            vals = df[col_name].replace(0, np.nan)
            fig.add_trace(go.Scatter(
                x=t, y=vals, mode="lines",
                name=label, line=dict(color=color, width=1),
                opacity=0.8,
            ), row=4, col=2)
    fig.add_hline(y=0, line_color=TXT, line_width=0.3, opacity=0.4, row=4, col=2)

    # ── Row 5L: Battery + Current ────────────────────────────────
    if "battery_voltage" in df.columns:
        bv = df["battery_voltage"].replace(-1, np.nan)
        fig.add_trace(go.Scatter(
            x=t, y=bv, mode="lines",
            name="Battery V", line=dict(color=GREEN, width=1.2),
            yaxis="y9",
        ), row=5, col=1)
    if "current_amps" in df.columns:
        ca = df["current_amps"].replace(-1, np.nan)
        fig.add_trace(go.Scatter(
            x=t, y=ca, mode="lines",
            name="Current A", line=dict(color=CORAL, width=1),
            opacity=0.8, yaxis="y10",
        ), row=5, col=1)

    # ── Row 5R: Compass ──────────────────────────────────────────
    if "compass_heading" in df.columns:
        ch = df["compass_heading"]
        fig.add_trace(go.Scatter(
            x=t, y=ch, mode="lines",
            name="Heading", line=dict(color=BLUE, width=1.2),
        ), row=5, col=2)
    if "compass_target_heading" in df.columns:
        ct = df["compass_target_heading"]
        fig.add_trace(go.Scatter(
            x=t, y=ct, mode="lines",
            name="Target°", line=dict(color=AMBER, width=1, dash="dot"),
        ), row=5, col=2)
    if "heading_error_deg" in df.columns:
        he = df["heading_error_deg"]
        fig.add_trace(go.Scatter(
            x=t, y=he, mode="lines",
            name="Error°", line=dict(color=RED, width=0.9),
            opacity=0.7,
        ), row=5, col=2)

    # ── Row 6L: Motor Differential ───────────────────────────────
    if "motor_diff" in df.columns:
        md = df["motor_diff"].dropna()
        fig.add_trace(go.Scatter(
            x=t.loc[md.index], y=md, mode="lines",
            name="Diff (L-R)", line=dict(color=PURPLE, width=1),
            fill="tozeroy", fillcolor=_fa(PURPLE, 0.09),
        ), row=6, col=1)
    fig.add_hline(y=0, line_color=TXT, line_width=0.3, opacity=0.4, row=6, col=1)

    # ── Row 6R: PID Correction ───────────────────────────────────
    if "pid_correction" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["pid_correction"], mode="lines",
            name="PID", line=dict(color=TEAL, width=1.2),
            fill="tozeroy", fillcolor=_fa(TEAL, 0.09),
        ), row=6, col=2)
    if "steer_heading_delta_deg" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["steer_heading_delta_deg"], mode="lines",
            name="Steer Δ°", line=dict(color=AMBER, width=0.9),
            opacity=0.7,
        ), row=6, col=2)
    fig.add_hline(y=0, line_color=TXT, line_width=0.3, opacity=0.4, row=6, col=2)

    # ── Row 7L: Accel Components ─────────────────────────────────
    accel_map = [
        ("accel_x", RED,   "X"),
        ("accel_y", GREEN, "Y"),
        ("accel_z", BLUE,  "Z"),
    ]
    for col_name, color, label in accel_map:
        if col_name in df.columns:
            vals = df[col_name]
            fig.add_trace(go.Scatter(
                x=t, y=vals, mode="lines",
                name=f"Accel {label}", line=dict(color=color, width=0.9),
                opacity=0.8,
            ), row=7, col=1)
    fig.add_hline(y=0, line_color=TXT, line_width=0.3, opacity=0.4, row=7, col=1)

    # ── Row 7R: Temperature ──────────────────────────────────────
    if "temp_c" in df.columns:
        fig.add_trace(go.Scatter(
            x=t, y=df["temp_c"], mode="lines",
            name="Temp °C", line=dict(color=AMBER, width=1.2),
            fill="tozeroy", fillcolor=_fa(AMBER, 0.09),
        ), row=7, col=2)
    fig.add_hline(y=85, line_color=RED, line_dash="dash",
                  line_width=0.8, opacity=0.5, row=7, col=2)

    # ── Row 8: Obstacle State Timeline ───────────────────────────
    state_colors = {"IDLE": GREEN, "BRAKING": AMBER, "AVOIDING": RED,
                    "REVERSING": ORANGE, "SWEEPING": PURPLE}
    if "obstacle_state" in df.columns:
        states = df["obstacle_state"].fillna("IDLE")
        unique_states = states.unique()
        state_map = {s: i for i, s in enumerate(["IDLE", "BRAKING", "AVOIDING",
                                                  "REVERSING", "SWEEPING"])}
        y_vals = states.map(lambda s: state_map.get(s, 0))
        for state_name, state_y in state_map.items():
            mask = (y_vals == state_y)
            if mask.any():
                color = state_colors.get(state_name, GRAY)
                fig.add_trace(go.Scatter(
                    x=t[mask], y=[state_y] * mask.sum(),
                    mode="markers",
                    marker=dict(color=color, size=4, symbol="square"),
                    name=state_name,
                ), row=8, col=1)
        fig.update_yaxes(
            tickmode="array",
            tickvals=list(state_map.values()),
            ticktext=list(state_map.keys()),
            row=8, col=1,
        )
    elif "autonomous_state" in df.columns:
        # For autonomous state
        fig.add_trace(go.Scatter(
            x=t, y=df["autonomous_state"],
            mode="markers",
            marker=dict(color=BLUE, size=3, symbol="square"),
            name="Auto State",
        ), row=8, col=1)

    # ── Global layout ─────────────────────────────────────────────
    fig.update_layout(
        **_PLOTLY_LAYOUT_BASE,
        height=1800,
        showlegend=True,
        legend=dict(
            orientation="v", x=1.01, y=1,
            bgcolor="rgba(13,17,23,0.8)", bordercolor=GRID, borderwidth=1,
            font=dict(size=10, color=TXT),
            tracegroupgap=0,
        ),
        uirevision="keep-zoom",
    )

    # Style all subplot x-axes with x-label only on bottom rows
    for r in range(1, 9):
        for c in range(1, 3):
            fig.update_xaxes(
                gridcolor=GRID, gridwidth=0.5, zeroline=False,
                tickcolor=GRAY, linecolor=GRID,
                title_text="Time (s)" if r == 8 else "",
                row=r, col=c,
            )
            fig.update_yaxes(
                gridcolor=GRID, gridwidth=0.5, zeroline=False,
                tickcolor=GRAY, linecolor=GRID,
                row=r, col=c,
            )

    # Annotation color for subplot titles
    for ann in fig.layout.annotations:
        ann.font.color = BLUE
        ann.font.size = 11

    return fig


def build_telem_figure(df: pd.DataFrame, t_range=None) -> go.Figure:
    """Build interactive figure for legacy telem CSV data."""
    if t_range:
        df = df[(df["t"] >= t_range[0]) & (df["t"] <= t_range[1])].copy()

    if df.empty:
        fig = go.Figure()
        fig.update_layout(**_PLOTLY_LAYOUT_BASE, title="No data in selected range")
        return fig

    has_motor  = df["motor_L"].notna() if "motor_L" in df.columns else pd.Series([False] * len(df))
    has_sensor = df["battery_v"].notna() if "battery_v" in df.columns else pd.Series([False] * len(df))
    ap = df[has_motor].copy()
    sn = df[has_sensor].copy()
    duration = df["t"].iloc[-1]

    specs = [
        [{"colspan": 2}, None],
        [{"type": "xy"}, {"type": "xy"}],
        [{"type": "xy"}, {"type": "xy"}],
        [{"type": "xy"}, {"type": "xy"}],
        [{"type": "xy"}, {"type": "xy"}],
        [{"type": "xy"}, {"type": "xy"}],
        [{"type": "xy"}, {"type": "xy"}],
        [{"colspan": 2}, None],
    ]
    subplot_titles = [
        "DISTANCE SENSORS", "",
        "MOTOR OUTPUT (%)", "STEERING",
        "THROTTLE (%)", "TRACKING CONFIDENCE",
        "ACCELEROMETER (magnitude)", "GYROSCOPE Z (yaw rate)",
        "BATTERY (V) & CURRENT (A)", "TARGET BBOX AREA",
        "MOTOR DIFFERENTIAL (L-R)", "VISION STEER PULL",
        "ACCEL COMPONENTS (g)", "— unused —",
        "AVOID STATE TIMELINE", "",
    ]

    fig = make_subplots(
        rows=8, cols=2, specs=specs,
        subplot_titles=subplot_titles,
        vertical_spacing=0.04, horizontal_spacing=0.06,
        row_heights=[0.10, 0.10, 0.10, 0.10, 0.12, 0.10, 0.10, 0.08],
    )
    t_ap = ap["t"] if len(ap) else pd.Series(dtype=float)
    t_sn = sn["t"] if len(sn) else pd.Series(dtype=float)

    # Row 1 – Distance sensors
    if "laser_front_cm" in sn.columns:
        fig.add_trace(go.Scatter(
            x=t_sn, y=sn["laser_front_cm"].clip(upper=300),
            mode="lines", name="Front Laser",
            line=dict(color=GREEN, width=1.2),
        ), row=1, col=1)
    if "sonar_front_cm" in sn.columns:
        fig.add_trace(go.Scatter(
            x=t_sn, y=sn["sonar_front_cm"].clip(upper=300),
            mode="lines", name="Sonar",
            line=dict(color=CORAL, width=1), opacity=0.7,
        ), row=1, col=1)
    for y_val, color, label in [(12, RED, "Stop"), (25, ORANGE, "Creep"),
                                  (50, AMBER, "Slow"), (100, GREEN, "Aware")]:
        fig.add_hline(y=y_val, line_color=color, line_dash="dash",
                      line_width=0.8, opacity=0.5, row=1, col=1)

    # Row 2 – Motor output + Steering
    if "motor_L" in ap.columns:
        fig.add_trace(go.Scatter(x=t_ap, y=ap["motor_L"],
                                 mode="lines", name="Motor L",
                                 line=dict(color=BLUE, width=1)), row=2, col=1)
        fig.add_trace(go.Scatter(x=t_ap, y=ap["motor_R"],
                                 mode="lines", name="Motor R",
                                 line=dict(color=ORANGE, width=1)), row=2, col=1)
    if "smooth_steering" in ap.columns:
        fig.add_trace(go.Scatter(x=t_ap, y=ap["smooth_steering"],
                                 mode="lines", name="Smooth",
                                 line=dict(color=PURPLE, width=1.2)), row=2, col=2)
    if "raw_steering" in ap.columns:
        fig.add_trace(go.Scatter(x=t_ap, y=ap["raw_steering"],
                                 mode="lines", name="Raw",
                                 line=dict(color=GRAY, width=0.8), opacity=0.5), row=2, col=2)

    # Row 3 – Throttle + Tracking
    if "smooth_throttle" in ap.columns:
        fig.add_trace(go.Scatter(x=t_ap, y=ap["smooth_throttle"],
                                 mode="lines", name="Smooth Thr",
                                 line=dict(color=GREEN, width=1.2),
                                 fill="tozeroy", fillcolor=_fa(GREEN, 0.13)), row=3, col=1)
    if "raw_throttle" in ap.columns:
        fig.add_trace(go.Scatter(x=t_ap, y=ap["raw_throttle"],
                                 mode="lines", name="Raw Thr",
                                 line=dict(color=GRAY, width=0.8), opacity=0.5), row=3, col=1)
    if "touch_conf" in ap.columns:
        tc = ap.dropna(subset=["touch_conf"])
        if len(tc) > 0:
            fig.add_trace(go.Scatter(x=tc["t"], y=tc["touch_conf"],
                                     mode="lines", name="Touch Conf",
                                     line=dict(color=BLUE, width=1),
                                     fill="tozeroy", fillcolor=_fa(BLUE, 0.09)), row=3, col=2)
        fig.add_hline(y=0.35, line_color=RED, line_dash="dash",
                      line_width=0.8, opacity=0.5, row=3, col=2)

    # Row 4 – Accel mag + Gyro Z
    if "accel_mag" in sn.columns:
        am = sn["accel_mag"].dropna()
        fig.add_trace(go.Scatter(x=t_sn.loc[am.index], y=am,
                                 mode="lines", name="|Accel|",
                                 line=dict(color=ORANGE, width=1)), row=4, col=1)
        fig.add_hline(y=1.0, line_color=GREEN, line_dash="dash", opacity=0.35, row=4, col=1)
        fig.add_hline(y=1.8, line_color=RED,   line_dash="dash", opacity=0.5, row=4, col=1)
    if "gyro_z" in sn.columns:
        gz = sn["gyro_z"].dropna()
        fig.add_trace(go.Scatter(x=t_sn.loc[gz.index], y=gz,
                                 mode="lines", name="Gyro Z",
                                 line=dict(color=PURPLE, width=1)), row=4, col=2)

    # Row 5 – Battery/Current + BBox
    if "battery_v" in sn.columns:
        fig.add_trace(go.Scatter(x=t_sn, y=sn["battery_v"],
                                 mode="lines", name="Battery V",
                                 line=dict(color=GREEN, width=1.2)), row=5, col=1)
    if "current_a" in sn.columns:
        fig.add_trace(go.Scatter(x=t_sn, y=sn["current_a"],
                                 mode="lines", name="Current A",
                                 line=dict(color=CORAL, width=1), opacity=0.8), row=5, col=1)
    if "target_bbox_area" in ap.columns:
        ba = ap["target_bbox_area"].dropna()
        if len(ba) > 0:
            fig.add_trace(go.Scatter(x=t_ap.loc[ba.index], y=ba,
                                     mode="lines", name="BBox Area",
                                     line=dict(color=BLUE, width=1),
                                     fill="tozeroy", fillcolor=_fa(BLUE, 0.08)), row=5, col=2)
            fig.add_hline(y=400000, line_color=AMBER, line_dash="dash",
                          opacity=0.5, row=5, col=2)

    # Row 6 – Motor diff + Vision steer
    if "motor_diff" in ap.columns:
        md = ap["motor_diff"].dropna()
        fig.add_trace(go.Scatter(x=t_ap.loc[md.index], y=md,
                                 mode="lines", name="Diff (L-R)",
                                 line=dict(color=PURPLE, width=1),
                                 fill="tozeroy", fillcolor=_fa(PURPLE, 0.09)), row=6, col=1)
    if "vision_steer_pull" in ap.columns:
        sp = ap["vision_steer_pull"].dropna()
        fig.add_trace(go.Scatter(x=t_ap.loc[sp.index], y=sp,
                                 mode="lines", name="Vision Pull",
                                 line=dict(color=PURPLE, width=1)), row=6, col=2)

    # Row 7 – Accel components
    for col_name, color, label in [("accel_x", RED, "X"), ("accel_y", GREEN, "Y"),
                                    ("accel_z", BLUE, "Z")]:
        if col_name in sn.columns:
            vals = sn[col_name].dropna()
            fig.add_trace(go.Scatter(x=t_sn.loc[vals.index], y=vals,
                                     mode="lines", name=f"Accel {label}",
                                     line=dict(color=color, width=0.9), opacity=0.8), row=7, col=1)

    # Row 8 – State timeline
    state_map = {"CLEAR": 0, "SWEEP_WAIT": 1, "ESCAPING": 2}
    state_color = {"CLEAR": GREEN, "SWEEP_WAIT": AMBER, "ESCAPING": RED}
    if "avoid_state" in ap.columns:
        states = ap["avoid_state"].fillna("CLEAR")
        for sn_name, sy in state_map.items():
            mask = states == sn_name
            if mask.any():
                fig.add_trace(go.Scatter(
                    x=t_ap[mask], y=[sy] * mask.sum(),
                    mode="markers",
                    marker=dict(color=state_color.get(sn_name, GRAY), size=4, symbol="square"),
                    name=sn_name,
                ), row=8, col=1)
        fig.update_yaxes(tickmode="array",
                         tickvals=[0, 1, 2],
                         ticktext=["CLEAR", "SWEEP", "ESCAPE"],
                         row=8, col=1)

    # Global layout
    fig.update_layout(
        **_PLOTLY_LAYOUT_BASE,
        height=1800,
        showlegend=True,
        legend=dict(
            orientation="v", x=1.01, y=1,
            bgcolor="rgba(13,17,23,0.8)", bordercolor=GRID,
            borderwidth=1, font=dict(size=10, color=TXT),
        ),
        uirevision="keep-zoom",
    )
    for r in range(1, 9):
        for c in range(1, 3):
            fig.update_xaxes(gridcolor=GRID, gridwidth=0.5, zeroline=False,
                             tickcolor=GRAY, linecolor=GRID,
                             title_text="Time (s)" if r == 8 else "",
                             row=r, col=c)
            fig.update_yaxes(gridcolor=GRID, gridwidth=0.5, zeroline=False,
                             tickcolor=GRAY, linecolor=GRID, row=r, col=c)
    for ann in fig.layout.annotations:
        ann.font.color = BLUE
        ann.font.size = 11

    return fig


# ─── Summary helpers ──────────────────────────────────────────────────────────

def _build_summary(df: pd.DataFrame, is_debug: bool) -> str:
    if df.empty:
        return "No data."
    lines = []
    duration = df["t"].iloc[-1] - df["t"].iloc[0]
    lines.append(f"Duration:  {duration:.1f}s  |  Rows: {len(df)}")

    if is_debug:
        if "current_pwm" in df.columns:
            driving = df[df["current_pwm"].abs() > 1]
            lines.append(f"Active drive time: {len(driving) * 0.02:.1f}s "
                         f"({100*len(driving)/max(len(df),1):.0f}%)")
        if "battery_voltage" in df.columns:
            bv = df["battery_voltage"].replace(-1, np.nan).dropna()
            if len(bv) > 0:
                lines.append(f"Battery:   {bv.iloc[0]:.2f}V → {bv.iloc[-1]:.2f}V "
                             f"(Δ {bv.iloc[0]-bv.iloc[-1]:.3f}V)")
        if "current_amps" in df.columns:
            ca = df["current_amps"].replace(-1, np.nan).dropna()
            if len(ca) > 0:
                lines.append(f"Current:   mean {ca.mean():.2f}A  peak {ca.max():.2f}A")
        if "laser_distance_cm" in df.columns:
            ld = df["laser_distance_cm"]
            close = ld[ld < 300].dropna()
            if len(close) > 0:
                lines.append(f"Laser:     min {close.min():.1f}cm  "
                             f"mean {close.mean():.1f}cm")
        if "accel_mag" in df.columns:
            am = df["accel_mag"].replace(0, np.nan).dropna()
            if len(am) > 0:
                lines.append(f"Accel |g|: mean {am.mean():.3f}  max {am.max():.3f}")
        if "obstacle_state" in df.columns:
            st = df["obstacle_state"].value_counts().to_dict()
            lines.append("States:    " + "  ".join(f"{k}={v}" for k, v in st.items()))
    else:
        if "smooth_throttle" in df.columns:
            driving = df[df["smooth_throttle"].abs() > 1]
            lines.append(f"Active drive time: {len(driving) * 0.05:.1f}s")
        if "battery_v" in df.columns:
            bv = df["battery_v"].dropna()
            if len(bv) > 0:
                lines.append(f"Battery:   {bv.iloc[0]:.2f}V → {bv.iloc[-1]:.2f}V")
        if "avoid_state" in df.columns:
            st = df["avoid_state"].value_counts().to_dict()
            lines.append("Avoid:     " + "  ".join(f"{k}={v}" for k, v in st.items()))

    return "\n".join(lines)


# ─── Dash app ─────────────────────────────────────────────────────────────────

def create_dash_app() -> dash.Dash:
    app = dash.Dash(
        __name__,
        title="Rover Debug Plots",
        update_title=None,
        assets_folder=os.path.join(_THIS_DIR, "dash_assets"),
    )

    # ── shared inline styles ───────────────────────────────────────
    _ctrl = dict(
        backgroundColor=CARD, color=TXT,
        border=f"1px solid {GRID}", borderRadius="6px",
        padding="4px 8px", fontSize="13px",
    )
    _lbl = dict(color=GRAY, fontSize="12px", marginBottom="4px")
    _btn_live_on = dict(
        cursor="pointer", width="100%", padding="8px",
        backgroundColor=GREEN, color=DARK,
        border="none", borderRadius="6px",
        fontWeight="bold", fontSize="13px",
    )
    _btn_live_off = dict(
        cursor="pointer", width="100%", padding="8px",
        backgroundColor=CARD, color=GREEN,
        border=f"1px solid {GREEN}", borderRadius="6px",
        fontWeight="bold", fontSize="13px",
    )
    _btn_reload = dict(
        cursor="pointer", width="100%", padding="8px",
        backgroundColor="#1f6feb", color="white",
        border="none", borderRadius="6px",
        fontWeight="bold", fontSize="13px",
    )

    app.layout = html.Div(
        style=dict(backgroundColor=DARK, minHeight="100vh", padding="16px 24px",
                   fontFamily="'Segoe UI','Helvetica Neue',Arial,sans-serif"),
        children=[

            # ── Header ────────────────────────────────────────────
            html.Div(
                style=dict(display="flex", alignItems="center",
                           justifyContent="space-between",
                           borderBottom=f"1px solid {GRID}",
                           paddingBottom="12px", marginBottom="16px"),
                children=[
                    html.Div([
                        html.H2("🐛 Rover Debug Telemetry",
                                style=dict(color=BLUE, margin=0, fontSize="22px")),
                        html.Span("Live log explorer — runs alongside python3 main.py --debug=true",
                                  style=dict(color=GRAY, fontSize="12px")),
                    ]),
                    # Live badge + row counter
                    html.Div(
                        style=dict(display="flex", alignItems="center", gap="12px"),
                        children=[
                            html.Span(id="row-counter",
                                      style=dict(color=GRAY, fontSize="12px")),
                            html.Span("● LIVE", id="live-badge",
                                      style=dict(color=GREEN, fontSize="13px",
                                                 fontWeight="bold",
                                                 padding="4px 12px",
                                                 border=f"1px solid {GREEN}",
                                                 borderRadius="12px")),
                        ],
                    ),
                ],
            ),

            # ── Controls row 1: file + refresh ───────────────────
            html.Div(
                style=dict(display="grid",
                           gridTemplateColumns="3fr 1fr 1fr 1fr",
                           gap="14px", marginBottom="10px",
                           alignItems="end"),
                children=[
                    html.Div([
                        html.Label("Log file", style=_lbl),
                        dcc.Dropdown(
                            id="log-selector",
                            placeholder="Select a log file…",
                            style={**_ctrl, "backgroundColor": CARD, "color": TXT},
                            optionHeight=28,
                        ),
                    ]),
                    html.Div([
                        html.Label("Auto-refresh", style=_lbl),
                        dcc.Dropdown(
                            id="refresh-interval",
                            options=[
                                {"label": "Off",  "value": 0},
                                {"label": "1 s",  "value": 1000},
                                {"label": "2 s",  "value": 2000},
                                {"label": "5 s",  "value": 5000},
                                {"label": "10 s", "value": 10000},
                            ],
                            value=2000,
                            clearable=False,
                            style={**_ctrl, "backgroundColor": CARD},
                        ),
                    ]),
                    html.Div([
                        html.Label("Keep zoom on refresh", style=_lbl),
                        dcc.Dropdown(
                            id="zoom-lock",
                            options=[
                                {"label": "Yes (keep zoom)", "value": "keep"},
                                {"label": "No (reset)",      "value": "reset"},
                            ],
                            value="keep",
                            clearable=False,
                            style={**_ctrl, "backgroundColor": CARD},
                        ),
                    ]),
                    html.Div([
                        html.Label("\u00a0", style=_lbl),
                        html.Button("⟳ Reload", id="reload-btn", n_clicks=0,
                                    style=_btn_reload),
                    ]),
                ],
            ),

            # ── Controls row 2: Follow Live + Tail window ─────────
            html.Div(
                style=dict(display="grid",
                           gridTemplateColumns="1fr 1fr 4fr",
                           gap="14px", marginBottom="16px",
                           alignItems="end"),
                children=[
                    html.Div([
                        html.Label("Live mode", style=_lbl),
                        html.Button("⚡ Follow Live  ON", id="follow-live-btn",
                                    n_clicks=0, style=_btn_live_on),
                    ]),
                    html.Div([
                        html.Label("Tail window", style=_lbl),
                        dcc.Dropdown(
                            id="tail-window",
                            options=[
                                {"label": "Last 15 s",  "value": 15},
                                {"label": "Last 30 s",  "value": 30},
                                {"label": "Last 60 s",  "value": 60},
                                {"label": "Last 120 s", "value": 120},
                                {"label": "All data",   "value": 0},
                            ],
                            value=60,
                            clearable=False,
                            style={**_ctrl, "backgroundColor": CARD},
                        ),
                    ]),
                    html.Div(
                        html.Span(
                            "Follow Live: auto-selects the newest log, extends the slider "
                            "as rows stream in, and tracks the tail. "
                            "Turn off to freely explore historical logs.",
                            style=dict(color=GRAY, fontSize="11px",
                                       lineHeight="1.6"),
                        ),
                        style=dict(paddingBottom="6px"),
                    ),
                ],
            ),

            # ── Time-range slider ─────────────────────────────────
            html.Div(
                style=dict(backgroundColor=CARD, borderRadius="8px",
                           padding="14px 24px", marginBottom="14px",
                           border=f"1px solid {GRID}"),
                children=[
                    html.Div(
                        style=dict(display="flex", justifyContent="space-between",
                                   marginBottom="8px"),
                        children=[
                            html.Span("Time filter (seconds)",
                                      style=dict(color=GRAY, fontSize="12px")),
                            html.Span(id="time-range-label",
                                      style=dict(color=TXT, fontSize="12px")),
                        ],
                    ),
                    dcc.RangeSlider(
                        id="time-slider",
                        min=0, max=100, step=0.5,
                        value=[0, 100],
                        marks=None,
                        tooltip={"placement": "bottom", "always_visible": True},
                        updatemode="mouseup",
                    ),
                ],
            ),

            # ── Summary stats ─────────────────────────────────────
            html.Pre(
                id="summary-text",
                style=dict(
                    color=TXT, backgroundColor=CARD,
                    border=f"1px solid {GRID}", borderRadius="8px",
                    padding="12px 16px", fontSize="12px",
                    fontFamily="'Cascadia Code','Fira Code',monospace",
                    marginBottom="16px", whiteSpace="pre-wrap",
                ),
            ),

            # ── Main plot ─────────────────────────────────────────
            dcc.Loading(
                id="loading-graph", type="circle", color=BLUE,
                children=dcc.Graph(
                    id="main-graph",
                    config={
                        "displayModeBar": True,
                        "modeBarButtonsToRemove": ["select2d", "lasso2d"],
                        "scrollZoom": True,
                        "responsive": True,
                    },
                    style={"height": "1820px"},
                ),
            ),

            # ── Hidden stores ─────────────────────────────────────
            dcc.Store(id="log-data-store"),          # file meta dict
            dcc.Store(id="loaded-path-store"),       # path currently loaded
            dcc.Store(id="follow-live-store", data=True),  # bool: follow live on?
            dcc.Interval(id="refresh-timer", interval=2000, disabled=False),
        ],
    )

    # ── Callbacks ─────────────────────────────────────────────────────

    # ── Toggle Follow Live button ──────────────────────────────────
    @app.callback(
        Output("follow-live-store", "data"),
        Output("follow-live-btn", "children"),
        Output("follow-live-btn", "style"),
        Input("follow-live-btn", "n_clicks"),
        State("follow-live-store", "data"),
        prevent_initial_call=True,
    )
    def toggle_follow_live(n_clicks, currently_on):
        new_state = not currently_on
        if new_state:
            return new_state, "⚡ Follow Live  ON", _btn_live_on
        return new_state, "⚡ Follow Live  OFF", _btn_live_off

    # ── Populate log-file selector ─────────────────────────────────
    @app.callback(
        Output("log-selector", "options"),
        Output("log-selector", "value"),
        Input("refresh-timer", "n_intervals"),
        Input("reload-btn", "n_clicks"),
        State("log-selector", "value"),
        State("follow-live-store", "data"),
    )
    def update_log_list(_n_intervals, _n_clicks, current_value, follow_live):
        logs = _discover_logs()
        options = [{"label": label, "value": path} for label, path in logs]

        # In follow-live mode, always point to the newest log
        if follow_live and options:
            # Prefer the active log path set by start_debug_plot_server()
            if _active_log_path and os.path.isfile(_active_log_path):
                return options, _active_log_path
            return options, options[0]["value"]

        # On first load with no selection, pick the newest
        if not current_value and options:
            return options, options[0]["value"]

        return options, current_value

    # ── Refresh interval control ───────────────────────────────────
    @app.callback(
        Output("refresh-timer", "interval"),
        Output("refresh-timer", "disabled"),
        Input("refresh-interval", "value"),
    )
    def set_refresh_interval(value):
        if not value:
            return 2000, True
        return value, False

    # ── Load / update file metadata + slider ──────────────────────
    @app.callback(
        Output("time-slider", "min"),
        Output("time-slider", "max"),
        Output("time-slider", "value"),
        Output("time-slider", "marks"),
        Output("log-data-store", "data"),
        Output("loaded-path-store", "data"),
        Input("log-selector", "value"),
        Input("reload-btn", "n_clicks"),
        Input("refresh-timer", "n_intervals"),
        State("time-slider", "value"),
        State("loaded-path-store", "data"),
        State("log-data-store", "data"),
        State("follow-live-store", "data"),
        State("tail-window", "value"),
    )
    def load_file(path, _btn, _interval, current_range, loaded_path, current_meta,
                  follow_live, tail_window):
        if not path or not os.path.isfile(path):
            return 0, 100, [0, 100], {}, None, None

        try:
            df = load_log(path)
        except Exception:
            return 0, 100, [0, 100], {}, None, path

        if df.empty:
            return 0, 100, [0, 100], {}, None, path

        t_min = float(df["t"].iloc[0])
        t_max = float(df["t"].iloc[-1])

        if loaded_path != path:
            # Brand-new file selected — decide initial window
            if follow_live and tail_window:
                new_range = [max(t_min, t_max - tail_window), t_max]
            else:
                new_range = [t_min, t_max]
        else:
            # Same file: possibly grown since last tick
            prev_t_max = (current_meta or {}).get("t_max", t_min)
            file_grew = t_max > prev_t_max + 0.01

            if follow_live:
                # Slide the window to track the tail
                if tail_window:
                    new_range = [max(t_min, t_max - tail_window), t_max]
                else:
                    new_range = [t_min, t_max]
            elif file_grew and current_range:
                # Manual mode: extend the right edge of the range as new data arrives
                # but don't reset the user's left edge
                new_range = [current_range[0], t_max]
            else:
                new_range = current_range if current_range else [t_min, t_max]

        # Slider marks every ~10 ticks
        span = max(t_max - t_min, 1.0)
        n_marks = min(10, int(span) + 1)
        step_m = span / max(n_marks, 1)
        marks = {
            round(t_min + i * step_m, 1): {
                "label": f"{t_min + i * step_m:.0f}s",
                "style": {"color": GRAY, "fontSize": "10px"},
            }
            for i in range(n_marks + 1)
            if t_min + i * step_m <= t_max + 0.1
        }

        meta = {
            "path": path,
            "is_debug": _is_debug_csv(df),
            "t_min": t_min,
            "t_max": t_max,
            "rows": len(df),
        }
        return t_min, t_max, new_range, marks, meta, path

    # ── Time-range label ──────────────────────────────────────────
    @app.callback(
        Output("time-range-label", "children"),
        Input("time-slider", "value"),
        State("log-data-store", "data"),
    )
    def update_time_label(slider_value, meta):
        if not slider_value:
            return ""
        a, b = slider_value
        total = (meta or {}).get("t_max", 100) - (meta or {}).get("t_min", 0)
        selected = b - a
        pct = 100 * selected / max(total, 0.001)
        return f"{a:.1f}s – {b:.1f}s  ({selected:.1f}s selected, {pct:.0f}% of log)"

    # ── Render graph + badge ──────────────────────────────────────
    @app.callback(
        Output("main-graph", "figure"),
        Output("summary-text", "children"),
        Output("live-badge", "children"),
        Output("live-badge", "style"),
        Output("row-counter", "children"),
        Input("time-slider", "value"),
        Input("log-selector", "value"),
        Input("reload-btn", "n_clicks"),
        Input("refresh-timer", "n_intervals"),
        State("zoom-lock", "value"),
        State("log-data-store", "data"),
        State("follow-live-store", "data"),
    )
    def refresh_graph(time_range, path, _btn, _interval,
                      zoom_lock, prev_meta, follow_live):
        import time as _time

        _badge_live = dict(color=GREEN, fontSize="13px", fontWeight="bold",
                           padding="4px 12px", border=f"1px solid {GREEN}",
                           borderRadius="12px")
        _badge_idle = dict(color=GRAY, fontSize="13px", fontWeight="bold",
                           padding="4px 12px", border=f"1px solid {GRAY}",
                           borderRadius="12px")

        def _empty(title):
            fig = go.Figure()
            fig.update_layout(**_PLOTLY_LAYOUT_BASE, title=title)
            return fig, title, "○ IDLE", _badge_idle, ""

        if not path or not os.path.isfile(path):
            return *_empty("Select a log file above"),

        try:
            df = load_log(path)
        except Exception as e:
            return *_empty(f"Error loading log: {e}"),

        t_range = time_range if time_range else None
        is_debug = _is_debug_csv(df)

        fig = (build_debug_figure(df, t_range) if is_debug
               else build_telem_figure(df, t_range))

        # uirevision controls whether Plotly re-applies zoom
        if zoom_lock == "keep":
            fig.update_layout(uirevision="keep-zoom")
        else:
            fig.update_layout(uirevision=str(_time.time()))

        # Summary over the filtered window
        view_df = (df[(df["t"] >= t_range[0]) & (df["t"] <= t_range[1])]
                   if t_range else df)
        summary = _build_summary(view_df, is_debug)

        # Badge: green when file modified in last 5s
        mtime = os.path.getmtime(path)
        is_live = (_time.time() - mtime) < 5

        # New-rows counter vs previous render
        prev_rows = (prev_meta or {}).get("rows", 0)
        total_rows = len(df)
        new_rows = max(0, total_rows - prev_rows)

        if is_live:
            badge_txt = "● LIVE"
            badge_style = _badge_live
        else:
            badge_txt = "○ IDLE"
            badge_style = _badge_idle

        mode_tag = "⚡" if follow_live else ""
        row_info = (f"{mode_tag} {total_rows:,} rows"
                    + (f"  +{new_rows} new" if new_rows > 0 else ""))

        return fig, summary, badge_txt, badge_style, row_info

    return app


# ─── Public entry-point ───────────────────────────────────────────────────────

def start_debug_plot_server(port: int = DEBUG_PORT,
                            log_path: str | None = None,
                            open_browser: bool = False):
    """
    Start the Dash debug plot server in a background daemon thread.
    Call this from main.py immediately after _init_debug_logger().

    Parameters
    ----------
    port      : TCP port (default 5001, so it doesn't clash with main Flask on 5000)
    log_path  : Absolute path to the CSV currently being written.
                When set, Follow-Live mode auto-selects this file.
    open_browser : Open http://localhost:<port> in the default browser after startup.
    """
    global _active_log_path
    if log_path:
        _active_log_path = log_path

    app = create_dash_app()

    def _run():
        import logging as _log
        _log.getLogger("werkzeug").setLevel(_log.WARNING)
        print(f"🐛 [DebugPlots] Dashboard → http://0.0.0.0:{port}"
              + (f"  (tracking {os.path.basename(log_path)})" if log_path else ""))
        app.run(
            host="0.0.0.0",
            port=port,
            debug=False,
            use_reloader=False,
            dev_tools_silence_routes_logging=True,
        )

    t = threading.Thread(target=_run, daemon=True, name="Thread-DebugPlots")
    t.start()

    if open_browser:
        import webbrowser, time
        time.sleep(2.0)
        webbrowser.open(f"http://localhost:{port}")

    return t


# ─── Standalone ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    port = DEBUG_PORT
    for arg in sys.argv[1:]:
        if arg.startswith("--port="):
            port = int(arg.split("=", 1)[1])

    app = create_dash_app()
    print(f"🐛 [DebugPlots] Standalone → http://localhost:{port}")
    app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False)

