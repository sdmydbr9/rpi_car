"""
steering_calibration.py - Shared Ackermann steering calibration helpers.

Keeps steering pulse-width validation and angle mapping in one place so the
Pi runtime, diagnostics tools, and Pico bridge all agree on the same limits.
"""

from __future__ import annotations

import re
from typing import Mapping


STEERING_PW_MIN = 500
STEERING_PW_MAX = 2500

DEFAULT_STEERING_CALIBRATION = {
    "left_pw": 940,
    "center_pw": 1440,
    "right_pw": 2150,
}

VALID_STEERING_PRESETS = ("left", "center", "right", "off")


class SteeringCalibrationError(ValueError):
    """Raised when a steering calibration payload is invalid."""


def default_steering_calibration() -> dict[str, int]:
    """Return a copy of the default steering pulse widths."""
    return dict(DEFAULT_STEERING_CALIBRATION)


def _coerce_int_field(name: str, value) -> int:
    if isinstance(value, bool) or value is None:
        raise SteeringCalibrationError(f"{name} must be an integer pulse width")
    if isinstance(value, int):
        return value
    if isinstance(value, str) and re.fullmatch(r"-?\d+", value.strip()):
        return int(value.strip())
    raise SteeringCalibrationError(f"{name} must be an integer pulse width")


def normalize_steering_calibration(data: Mapping[str, object]) -> dict[str, int]:
    """Validate and normalize steering pulse widths."""
    if not isinstance(data, Mapping):
        raise SteeringCalibrationError("steering calibration must be a mapping")

    missing = [key for key in DEFAULT_STEERING_CALIBRATION if key not in data]
    if missing:
        raise SteeringCalibrationError(
            f"missing steering calibration fields: {', '.join(missing)}"
        )

    normalized = {
        "left_pw": _coerce_int_field("left_pw", data.get("left_pw")),
        "center_pw": _coerce_int_field("center_pw", data.get("center_pw")),
        "right_pw": _coerce_int_field("right_pw", data.get("right_pw")),
    }

    for key, value in normalized.items():
        if value < STEERING_PW_MIN or value > STEERING_PW_MAX:
            raise SteeringCalibrationError(
                f"{key} must be between {STEERING_PW_MIN} and {STEERING_PW_MAX} us"
            )

    if not (normalized["left_pw"] < normalized["center_pw"] < normalized["right_pw"]):
        raise SteeringCalibrationError(
            "steering calibration must satisfy left_pw < center_pw < right_pw"
        )

    return normalized


def clamp_steering_pw(pw_us, calibration: Mapping[str, object]) -> int:
    """Clamp a pulse width to the configured left/right limits."""
    normalized = normalize_steering_calibration(calibration)
    try:
        pulse = int(pw_us)
    except (TypeError, ValueError):
        pulse = normalized["center_pw"]
    return max(normalized["left_pw"], min(normalized["right_pw"], pulse))


def steering_angle_to_pw(angle, calibration: Mapping[str, object]) -> int:
    """Map steering angle (-50..+50) onto the configured pulse widths."""
    normalized = normalize_steering_calibration(calibration)
    try:
        angle_value = float(angle)
    except (TypeError, ValueError):
        angle_value = 0.0
    angle_value = max(-50.0, min(50.0, angle_value))

    if angle_value < 0:
        pulse = normalized["center_pw"] + (
            (angle_value / 50.0) * (normalized["center_pw"] - normalized["left_pw"])
        )
    else:
        pulse = normalized["center_pw"] + (
            (angle_value / 50.0) * (normalized["right_pw"] - normalized["center_pw"])
        )
    return clamp_steering_pw(int(pulse), normalized)


def steering_preset_to_pw(preset: str, calibration: Mapping[str, object]) -> int:
    """Resolve a preset name into an exact pulse width."""
    normalized = normalize_steering_calibration(calibration)
    preset_name = str(preset or "").strip().lower()
    if preset_name not in VALID_STEERING_PRESETS:
        raise SteeringCalibrationError(
            f"preset must be one of: {', '.join(VALID_STEERING_PRESETS)}"
        )
    if preset_name == "left":
        return normalized["left_pw"]
    if preset_name == "right":
        return normalized["right_pw"]
    return normalized["center_pw"]
