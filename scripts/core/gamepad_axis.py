"""Helpers for normalizing centred gamepad axes.

These utilities prefer the real evdev axis metadata when a physical
controller is connected, and fall back to the common 0..255 / center 128
range used by many Bluetooth pads.
"""

from __future__ import annotations


def build_default_axis_profiles(center=128.0, minimum=0.0, maximum=255.0, deadzone=15.0):
    """Return default profiles for the throttle and steering axes we use."""
    base = {
        "center": float(center),
        "min": float(minimum),
        "max": float(maximum),
        "deadzone": float(deadzone),
    }
    return {
        "ABS_Y": dict(base),
        "ABS_Z": dict(base),
    }


def load_inputs_axis_profiles(default_profiles):
    """Load evdev absolute-axis ranges for the first connected gamepad.

    Returns `(profiles, source_path)` where `profiles` preserves the keys
    from `default_profiles`, and `source_path` is the `/dev/input/eventX`
    device used when metadata was found.
    """
    profiles = {
        axis_code: {
            "center": float(profile.get("center", 128.0)),
            "min": float(profile.get("min", 0.0)),
            "max": float(profile.get("max", 255.0)),
            "deadzone": float(profile.get("deadzone", 15.0)),
        }
        for axis_code, profile in (default_profiles or {}).items()
    }

    try:
        from inputs import devices
        from evdev import InputDevice, ecodes
    except Exception:
        return profiles, None

    for pad in getattr(devices, "gamepads", []) or []:
        path_getter = getattr(pad, "get_char_device_path", None)
        device_path = path_getter() if callable(path_getter) else None
        if not device_path:
            continue

        try:
            dev = InputDevice(device_path)
            for axis_code, fallback in profiles.items():
                axis_id = getattr(ecodes, axis_code, None)
                if axis_id is None:
                    continue
                absinfo = dev.absinfo(axis_id)
                if absinfo is None:
                    continue

                minimum = float(absinfo.min)
                maximum = float(absinfo.max)
                if maximum <= minimum:
                    continue

                flat = float(getattr(absinfo, "flat", 0) or 0)
                profiles[axis_code] = {
                    "center": (minimum + maximum) / 2.0,
                    "min": minimum,
                    "max": maximum,
                    "deadzone": max(float(fallback.get("deadzone", 15.0)), flat),
                }
            try:
                dev.close()
            except Exception:
                pass
            return profiles, device_path
        except Exception:
            continue

    return profiles, None


def normalize_centered_axis(raw_value, profile, output_scale=100.0, invert=False):
    """Map a centred axis to `-output_scale .. +output_scale`.

    Positive/negative halves use separate spans so left/right steering keeps
    its full range even when the device is slightly asymmetric.
    """
    output_scale = float(output_scale)
    raw_value = float(raw_value)
    center = float(profile.get("center", 128.0))
    minimum = float(profile.get("min", 0.0))
    maximum = float(profile.get("max", 255.0))
    deadzone = max(0.0, float(profile.get("deadzone", 0.0)))

    if abs(raw_value - center) <= deadzone:
        normalized = 0.0
    elif raw_value >= center:
        span = max(1.0, maximum - center)
        normalized = ((raw_value - center) / span) * output_scale
    else:
        span = max(1.0, center - minimum)
        normalized = ((raw_value - center) / span) * output_scale

    if invert:
        normalized = -normalized

    return max(-output_scale, min(output_scale, normalized))


def describe_axis_profile(axis_code, profile):
    """Return a compact text description for logs/debug output."""
    return (
        f"{axis_code}: "
        f"min={int(round(float(profile.get('min', 0.0))))} "
        f"center={float(profile.get('center', 0.0)):.1f} "
        f"max={int(round(float(profile.get('max', 0.0))))} "
        f"deadzone={int(round(float(profile.get('deadzone', 0.0))))}"
    )
