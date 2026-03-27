#!/usr/bin/env python3
"""
debug_plot_server.py
====================
Lightweight JSON API server for the RPi Car debug ring buffer.

The React Debug page (served by the main Flask app at /debug) polls
  GET http://<pi>:5001/api/snapshot
every 500 ms to display live telemetry charts.

Started automatically by scripts/main.py when launched with --debug=true.
Port 5001 is used to avoid conflicting with the main Flask server (5000).

Public API (called by main.py)
-------------------------------
  set_fields(fields)   — register column names once, before any rows arrive
  push_row(row)        — thread-safe append of one physics-loop snapshot
  start_debug_plot_server(port=5001)  — start background Flask thread
"""

from __future__ import annotations

import threading
import time
import collections
from typing import List, Tuple, Any

# ═══════════════════════════════════════════════════════════════════
# 1.  SHARED RING BUFFER  (written by physics loop, read by API)
# ═══════════════════════════════════════════════════════════════════

_MAXROWS = 36_000          # 12 min @ 50 Hz
_buf: collections.deque   = collections.deque(maxlen=_MAXROWS)
_fields: List[str]        = []
_buf_lock                  = threading.Lock()
_session_start: float     = time.time()

# Rate tracker (rows / second, last 2 s window)
_rate_window: collections.deque = collections.deque(maxlen=200)
_seq_counter: int   = 0    # total rows ever pushed (monotonic, never resets)
_rate_hz: float     = 0.0  # cached ingest rate – updated every 10 pushes


def set_fields(fields: List[str]) -> None:
    """Called once by main.py before any rows are pushed."""
    global _fields
    with _buf_lock:
        _fields = list(fields)


def push_row(row: List[Any]) -> None:
    """Thread-safe append of one physics-loop snapshot row."""
    global _seq_counter, _rate_hz
    with _buf_lock:
        _buf.append(row)
        _seq_counter += 1
        now = time.monotonic()
        _rate_window.append(now)
        # Recompute rate every 10 pushes instead of on every HTTP request
        if _seq_counter % 10 == 0:
            _rate_hz = sum(1 for ts in _rate_window if ts >= now - 2.0) / 2.0


def get_snapshot() -> Tuple[List[str], List[List[Any]]]:
    """Return a copy of (fields, rows) safe to use outside the lock."""
    with _buf_lock:
        return list(_fields), list(_buf)


# ═══════════════════════════════════════════════════════════════════
# 2.  FLASK JSON API
# ═══════════════════════════════════════════════════════════════════

import os as _os
import gzip as _gzip
import json as _json
from flask import Flask, request, redirect, send_from_directory

_api = Flask(__name__)

# Path to the built React app (same dist/ folder served by main.py on port 5000)
_DIST_DIR = _os.path.join(
    _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))),
    "dist",
)


@_api.after_request
def _add_cors(response):
    response.headers["Access-Control-Allow-Origin"]  = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


@_api.route("/")
def root():
    """Redirect bare root to the debug UI page."""
    return redirect("/debug")


@_api.route("/debug")
@_api.route("/debug/")
def debug_ui():
    """Serve the React SPA for the /debug route."""
    return send_from_directory(_DIST_DIR, "index.html")


@_api.route("/assets/<path:filename>")
def static_assets(filename):
    """Serve React build assets (JS/CSS chunks)."""
    return send_from_directory(_os.path.join(_DIST_DIR, "assets"), filename)


@_api.route("/api/snapshot", methods=["GET", "OPTIONS"])
def api_snapshot():
    """
    Query params
    ------------
    since_seq=<int>   Return only rows pushed after this sequence number.
                      Omit (or -1) for a full snapshot (first load).
    max_rows=<int>    Hard cap on rows returned (default: 3600, max: 3600)
    """
    if request.method == "OPTIONS":
        return _api.make_response(("", 204))

    since_seq = request.args.get("since_seq", None, type=int)
    max_rows  = max(1, min(request.args.get("max_rows", 3600, type=int), 3600))

    # Atomic snapshot of buffer + sequence counter
    with _buf_lock:
        fields       = list(_fields)
        all_rows     = list(_buf)
        seq_tail     = _seq_counter
        rate_hz_snap = _rate_hz

    # Incremental slice: only rows newer than the client's last known seq
    if since_seq is not None and since_seq >= 0:
        new_count = seq_tail - since_seq
        if new_count <= 0:
            rows = []
        elif new_count < len(all_rows):
            rows = all_rows[-new_count:]
        else:
            rows = all_rows
    else:
        rows = all_rows

    # Hard cap
    if len(rows) > max_rows:
        rows = rows[-max_rows:]

    payload = {
        "fields":            fields,
        "rows":              rows,
        "row_count":         len(rows),
        "session_elapsed_s": round(time.time() - _session_start, 1),
        "rate_hz":           round(rate_hz_snap, 1),
        "seq_tail":          seq_tail,
    }

    body = _json.dumps(payload, separators=(',', ':')).encode()

    # Gzip compress when the browser supports it (always true for modern browsers)
    if "gzip" in request.headers.get("Accept-Encoding", ""):
        body = _gzip.compress(body, compresslevel=1)   # level-1: fast, ~70 % smaller
        resp = _api.make_response(body)
        resp.headers["Content-Type"]     = "application/json"
        resp.headers["Content-Encoding"] = "gzip"
        resp.headers["Vary"]             = "Accept-Encoding"
        return resp

    resp = _api.make_response(body)
    resp.headers["Content-Type"] = "application/json"
    return resp


# ═══════════════════════════════════════════════════════════════════
# 3.  START FUNCTION  (called from main.py)
# ═══════════════════════════════════════════════════════════════════

def start_debug_plot_server(port: int = 5001) -> None:
    """Launch the JSON API server in a background daemon thread."""
    def _run():
        import logging as _logging
        _logging.getLogger("werkzeug").setLevel(_logging.WARNING)
        _api.run(host="0.0.0.0", port=port, debug=False,
                 use_reloader=False, threaded=True)

    t = threading.Thread(target=_run, daemon=True, name="Thread-DebugAPI")
    t.start()
