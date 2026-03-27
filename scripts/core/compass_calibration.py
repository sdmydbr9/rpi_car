"""
compass_calibration.py — Shared 3D magnetometer calibration helpers.

This module provides the common math and JSON I/O used by the diagnostics
tools.  It supports the new 3D ellipsoid-fit calibration while remaining
backward-safe with the older XY offset/scale schema.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)

DEFAULT_CALIBRATION_PATH = os.path.join(SCRIPTS_DIR, "diagnostics", "compass_cal.json")
DEFAULT_CAPTURE_DIR = os.path.join(PROJECT_ROOT, "rover_logs")

CALIBRATION_VERSION = 2
CALIBRATION_METHOD = "3d_least_squares_ellipsoid"

MIN_REQUIRED_SAMPLES = 1000
MIN_REQUIRED_FACES = 6
MIN_REQUIRED_OCTANTS = 6
MAX_FIELD_NORM_CV = 0.10

FACE_THRESHOLD_G = 0.72
OCTANT_COMPONENT_THRESHOLD_G = 0.20
MIN_ACCEL_NORM_G = 0.55
MAX_ACCEL_NORM_G = 1.45
MIN_HORIZONTAL_FIELD_SQ = 1e-9


class CalibrationError(RuntimeError):
    """Raised when magnetometer calibration data is invalid or cannot be fit."""


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def wrap_heading_deg(angle_deg: float) -> float:
    """Wrap a compass bearing to the range [0, 360)."""
    return (float(angle_deg) % 360.0 + 360.0) % 360.0


def _require_matrix3(value: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise CalibrationError(f"Expected a 3x3 matrix, got shape {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise CalibrationError("Calibration matrix contains non-finite values")
    return matrix


def _require_vector3(value: Sequence[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,):
        raise CalibrationError(f"Expected a 3-vector, got shape {vector.shape}")
    if not np.isfinite(vector).all():
        raise CalibrationError("Calibration vector contains non-finite values")
    return vector


def _coerce_samples_xyz(samples_xyz: Iterable[Sequence[float]]) -> np.ndarray:
    samples = np.asarray(list(samples_xyz), dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise CalibrationError(f"Expected samples shaped (N, 3), got {samples.shape}")
    if samples.shape[0] < 9:
        raise CalibrationError("At least 9 XYZ samples are required for an ellipsoid fit")
    if not np.isfinite(samples).all():
        raise CalibrationError("Sample set contains non-finite values")
    return samples


def default_calibration_path() -> str:
    return DEFAULT_CALIBRATION_PATH


def default_capture_dir() -> str:
    return DEFAULT_CAPTURE_DIR


def identity_calibration(heading_offset_deg: float = 90.0) -> dict[str, Any]:
    """Return a neutral calibration that preserves the legacy heading convention."""
    calibration = {
        "version": 1,
        "method": "legacy_identity",
        "hard_iron_bias": [0.0, 0.0, 0.0],
        "soft_iron_matrix": np.eye(3, dtype=np.float64).tolist(),
        "field_strength": 1.0,
        "heading_offset_deg": float(heading_offset_deg),
        "sample_count": 0,
        "capture_csv": None,
        "quality": {},
        "offset_x": 0.0,
        "offset_y": 0.0,
        "scale_x": 1.0,
        "scale_y": 1.0,
    }
    return calibration


@dataclass
class CoverageTracker:
    """Track whether the tumble dance covered enough 3D orientations."""

    face_threshold: float = FACE_THRESHOLD_G
    octant_threshold: float = OCTANT_COMPONENT_THRESHOLD_G
    min_norm_g: float = MIN_ACCEL_NORM_G
    max_norm_g: float = MAX_ACCEL_NORM_G
    samples_seen: int = 0
    accepted_samples: int = 0
    face_counts: dict[str, int] = field(default_factory=dict)
    octant_counts: dict[str, int] = field(default_factory=dict)

    def update(self, accel_xyz: Sequence[float]) -> bool:
        self.samples_seen += 1
        vec = np.asarray(accel_xyz, dtype=np.float64)
        if vec.shape != (3,) or not np.isfinite(vec).all():
            return False

        norm = float(np.linalg.norm(vec))
        if not (self.min_norm_g <= norm <= self.max_norm_g):
            return False

        self.accepted_samples += 1
        unit = vec / norm

        axis = int(np.argmax(np.abs(unit)))
        axis_label = ("X", "Y", "Z")[axis]
        sign_label = "+" if unit[axis] >= 0.0 else "-"
        if abs(unit[axis]) >= self.face_threshold:
            face = f"{sign_label}{axis_label}"
            self.face_counts[face] = self.face_counts.get(face, 0) + 1

        if np.all(np.abs(unit) >= self.octant_threshold):
            octant = "".join("+" if component >= 0.0 else "-" for component in unit)
            self.octant_counts[octant] = self.octant_counts.get(octant, 0) + 1

        return True

    @property
    def face_count(self) -> int:
        return len(self.face_counts)

    @property
    def octant_count(self) -> int:
        return len(self.octant_counts)

    @property
    def coverage_ok(self) -> bool:
        return (
            self.face_count >= MIN_REQUIRED_FACES
            and self.octant_count >= MIN_REQUIRED_OCTANTS
        )

    def summary(self) -> dict[str, Any]:
        return {
            "samples_seen": int(self.samples_seen),
            "accepted_samples": int(self.accepted_samples),
            "faces_seen": sorted(self.face_counts),
            "face_count": int(self.face_count),
            "octants_seen": sorted(self.octant_counts),
            "octant_count": int(self.octant_count),
            "coverage_ok": bool(self.coverage_ok),
            "thresholds": {
                "min_faces": MIN_REQUIRED_FACES,
                "min_octants": MIN_REQUIRED_OCTANTS,
                "face_threshold_g": self.face_threshold,
                "octant_component_threshold_g": self.octant_threshold,
                "min_accel_norm_g": self.min_norm_g,
                "max_accel_norm_g": self.max_norm_g,
            },
        }


def summarize_coverage(accel_samples_xyz: Iterable[Sequence[float]]) -> dict[str, Any]:
    tracker = CoverageTracker()
    for sample in accel_samples_xyz:
        tracker.update(sample)
    return tracker.summary()


def legacy_xy_scales_from_soft_matrix(soft_iron_matrix: Sequence[Sequence[float]]) -> tuple[float, float]:
    """
    Derive compatibility-only XY gains from the 3D correction matrix.

    These gains intentionally ignore cross-axis terms and renormalise the
    diagonal so old XY-only code paths still behave sensibly.
    """
    soft = _require_matrix3(soft_iron_matrix)
    x_gain = abs(float(soft[0, 0]))
    y_gain = abs(float(soft[1, 1]))
    avg_xy = (x_gain + y_gain) / 2.0
    if avg_xy <= 1e-12:
        return 1.0, 1.0
    return x_gain / avg_xy, y_gain / avg_xy


def normalize_calibration_dict(data: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize either v2 or legacy XY calibration JSON to a common dict."""
    version = _as_int(data.get("version", 1), 1)

    if version >= CALIBRATION_VERSION:
        bias = _require_vector3(data.get("hard_iron_bias", [0.0, 0.0, 0.0]))
        soft = _require_matrix3(data.get("soft_iron_matrix", np.eye(3)))
        heading_offset_deg = _as_float(data.get("heading_offset_deg", 0.0), 0.0)
        field_strength = _as_float(data.get("field_strength", 1.0), 1.0)
        if not math.isfinite(field_strength) or field_strength <= 0.0:
            raise CalibrationError("field_strength must be positive")
        scale_x, scale_y = legacy_xy_scales_from_soft_matrix(soft)
        quality = data.get("quality", {})
        capture_csv = data.get("capture_csv")
        method = str(data.get("method", CALIBRATION_METHOD))
    else:
        offset_x = _as_float(data.get("offset_x", 0.0), 0.0)
        offset_y = _as_float(data.get("offset_y", 0.0), 0.0)
        scale_x = _as_float(data.get("scale_x", 1.0), 1.0)
        scale_y = _as_float(data.get("scale_y", 1.0), 1.0)
        bias = np.array([offset_x, offset_y, 0.0], dtype=np.float64)
        soft = np.array(
            [
                [scale_x, 0.0, 0.0],
                [0.0, scale_y, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        heading_offset_deg = _as_float(data.get("heading_offset_deg", 90.0), 90.0)
        field_strength = _as_float(data.get("field_strength", 1.0), 1.0)
        quality = data.get("quality", {})
        capture_csv = data.get("capture_csv")
        method = str(data.get("method", "legacy_xy_offsets"))

    normalized = {
        "version": int(version),
        "method": method,
        "hard_iron_bias": bias.astype(float).tolist(),
        "soft_iron_matrix": soft.astype(float).tolist(),
        "field_strength": float(field_strength),
        "heading_offset_deg": float(heading_offset_deg),
        "sample_count": _as_int(data.get("sample_count", 0), 0),
        "capture_csv": capture_csv,
        "quality": quality if isinstance(quality, Mapping) else {},
        "offset_x": float(bias[0]),
        "offset_y": float(bias[1]),
        "scale_x": float(scale_x),
        "scale_y": float(scale_y),
    }
    return normalized


def serialize_calibration(calibration: Mapping[str, Any]) -> dict[str, Any]:
    """Convert a normalized calibration dict into the on-disk JSON schema."""
    normalized = normalize_calibration_dict(calibration)
    legacy_xy = {
        "offset_x": normalized["offset_x"],
        "offset_y": normalized["offset_y"],
        "scale_x": normalized["scale_x"],
        "scale_y": normalized["scale_y"],
        "note": "Compatibility-only XY approximation derived from the full 3D calibration.",
    }
    return {
        "version": int(normalized["version"]),
        "method": normalized["method"],
        "hard_iron_bias": [round(v, 12) for v in normalized["hard_iron_bias"]],
        "soft_iron_matrix": [
            [round(v, 12) for v in row]
            for row in normalized["soft_iron_matrix"]
        ],
        "field_strength": round(float(normalized["field_strength"]), 12),
        "heading_offset_deg": round(float(normalized["heading_offset_deg"]), 12),
        "sample_count": int(normalized["sample_count"]),
        "capture_csv": normalized["capture_csv"],
        "quality": normalized["quality"],
        "legacy_2d": legacy_xy,
        "offset_x": round(float(normalized["offset_x"]), 12),
        "offset_y": round(float(normalized["offset_y"]), 12),
        "scale_x": round(float(normalized["scale_x"]), 12),
        "scale_y": round(float(normalized["scale_y"]), 12),
    }


def save_calibration(calibration: Mapping[str, Any], path: str = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    payload = serialize_calibration(calibration)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=False)
        handle.write("\n")
    return payload


def load_calibration(path: str = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    return normalize_calibration_dict(data)


def apply_calibration(mag_xyz: Sequence[float], calibration: Mapping[str, Any]) -> np.ndarray:
    """Apply hard- and soft-iron correction to a raw XYZ magnetometer vector."""
    normalized = normalize_calibration_dict(calibration)
    bias = _require_vector3(normalized["hard_iron_bias"])
    soft = _require_matrix3(normalized["soft_iron_matrix"])
    raw = _require_vector3(mag_xyz)
    return soft @ (raw - bias)


def base_heading_from_corrected(corrected_xyz: Sequence[float]) -> float | None:
    """
    Return the clockwise heading from the corrected XY vector before offset.

    This keeps the historic "clockwise-positive" convention.  The stored
    heading_offset_deg then rotates that base heading onto the rover's chosen
    world reference bearing.
    """
    corrected = _require_vector3(corrected_xyz)
    horizontal_sq = float(corrected[0] * corrected[0] + corrected[1] * corrected[1])
    if horizontal_sq <= MIN_HORIZONTAL_FIELD_SQ:
        return None
    return wrap_heading_deg(-math.degrees(math.atan2(corrected[1], corrected[0])))


def compute_heading_degrees(mag_xyz: Sequence[float], calibration: Mapping[str, Any]) -> float | None:
    corrected = apply_calibration(mag_xyz, calibration)
    base_heading = base_heading_from_corrected(corrected)
    if base_heading is None:
        return None
    heading_offset_deg = _as_float(calibration.get("heading_offset_deg", 0.0), 0.0)
    return wrap_heading_deg(base_heading + heading_offset_deg)


def estimate_heading_offset(
    calibration: Mapping[str, Any],
    mag_samples_xyz: Iterable[Sequence[float]],
    reference_bearing_deg: float = 0.0,
) -> float:
    """
    Estimate the rover's heading offset from corrected live samples.

    The rover should be level and pointing at a known bearing during this
    measurement.  The returned offset is stored in the calibration JSON and is
    applied on top of the base clockwise heading.
    """
    corrected_vectors = np.asarray(
        [apply_calibration(sample, calibration) for sample in mag_samples_xyz],
        dtype=np.float64,
    )
    if corrected_vectors.size == 0:
        raise CalibrationError("No magnetometer samples were available for heading alignment")

    mean_corrected = np.mean(corrected_vectors, axis=0)
    base_heading = base_heading_from_corrected(mean_corrected)
    if base_heading is None:
        raise CalibrationError("Aligned samples did not contain a usable horizontal field vector")
    return wrap_heading_deg(reference_bearing_deg - base_heading)


def fit_ellipsoid_calibration(
    mag_samples_xyz: Iterable[Sequence[float]],
    *,
    accel_samples_xyz: Iterable[Sequence[float]] | None = None,
    capture_csv: str | None = None,
    heading_offset_deg: float = 0.0,
) -> dict[str, Any]:
    """
    Fit a 3D least-squares ellipsoid and derive a soft-iron correction matrix.

    The correction matrix is scaled by the geometric mean of the fitted raw
    ellipsoid radii.  That makes the corrected field norm settle around the
    stored field_strength while keeping the matrix determinant normalized.
    """
    mags = _coerce_samples_xyz(mag_samples_xyz)
    x, y, z = mags[:, 0], mags[:, 1], mags[:, 2]
    design = np.column_stack(
        [
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
        ]
    )

    params, _, _, _ = np.linalg.lstsq(design, np.ones(mags.shape[0], dtype=np.float64), rcond=None)

    A = np.array(
        [
            [params[0], params[3], params[4]],
            [params[3], params[1], params[5]],
            [params[4], params[5], params[2]],
        ],
        dtype=np.float64,
    )
    g = np.array(params[6:9], dtype=np.float64)

    try:
        center = -np.linalg.solve(A, g)
    except np.linalg.LinAlgError as exc:
        raise CalibrationError("Ellipsoid fit produced a singular center solve") from exc

    beta = 1.0 - float(center @ A @ center + 2.0 * g @ center)
    if not math.isfinite(beta) or beta <= 0.0:
        raise CalibrationError(f"Invalid ellipsoid translation scale beta={beta!r}")

    normalized_shape = A / beta
    eigenvalues, eigenvectors = np.linalg.eigh(normalized_shape)
    if np.any(~np.isfinite(eigenvalues)) or np.any(eigenvalues <= 0.0):
        raise CalibrationError(
            "Ellipsoid fit is not positive definite; the tumble data likely lacked 3D coverage"
        )

    soft_iron_unit = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T
    semi_axis_lengths = 1.0 / np.sqrt(eigenvalues)
    field_strength = float(np.cbrt(np.prod(semi_axis_lengths)))
    soft_iron_matrix = soft_iron_unit * field_strength
    scale_x, scale_y = legacy_xy_scales_from_soft_matrix(soft_iron_matrix)

    calibration = {
        "version": CALIBRATION_VERSION,
        "method": CALIBRATION_METHOD,
        "hard_iron_bias": center.astype(float).tolist(),
        "soft_iron_matrix": soft_iron_matrix.astype(float).tolist(),
        "field_strength": field_strength,
        "heading_offset_deg": float(heading_offset_deg),
        "sample_count": int(mags.shape[0]),
        "capture_csv": capture_csv,
        "quality": {
            "design_matrix_condition": float(np.linalg.cond(design)),
            "beta": float(beta),
            "semi_axis_lengths": semi_axis_lengths.astype(float).tolist(),
            "normalized_shape_eigenvalues": eigenvalues.astype(float).tolist(),
        },
        "offset_x": float(center[0]),
        "offset_y": float(center[1]),
        "scale_x": float(scale_x),
        "scale_y": float(scale_y),
    }

    validation = validate_calibration(
        mags,
        calibration,
        accel_samples_xyz=accel_samples_xyz,
        require_sample_count=False,
    )
    calibration["quality"].update(validation)
    return normalize_calibration_dict(calibration)


def validate_calibration(
    mag_samples_xyz: Iterable[Sequence[float]],
    calibration: Mapping[str, Any],
    *,
    accel_samples_xyz: Iterable[Sequence[float]] | None = None,
    require_sample_count: bool = True,
) -> dict[str, Any]:
    """Validate a calibration against raw sample data and optional accel coverage."""
    mags = _coerce_samples_xyz(mag_samples_xyz)
    normalized = normalize_calibration_dict(calibration)

    corrected = np.asarray([apply_calibration(sample, normalized) for sample in mags], dtype=np.float64)
    corrected_norms = np.linalg.norm(corrected, axis=1)
    field_norm_mean = float(np.mean(corrected_norms))
    field_norm_std = float(np.std(corrected_norms))
    field_norm_cv = float(field_norm_std / field_norm_mean) if field_norm_mean > 1e-12 else float("inf")

    soft = _require_matrix3(normalized["soft_iron_matrix"])
    field_strength = float(normalized["field_strength"])
    unit_soft = soft / max(field_strength, 1e-12)
    soft_eigenvalues = np.linalg.eigvalsh(unit_soft)
    positive_definite = bool(np.all(soft_eigenvalues > 0.0))
    semi_axis_lengths = (1.0 / soft_eigenvalues).astype(float).tolist()

    if accel_samples_xyz is None:
        coverage = normalized.get("quality", {}).get("coverage", {})
        if not isinstance(coverage, Mapping):
            coverage = {}
    else:
        coverage = summarize_coverage(accel_samples_xyz)

    errors: list[str] = []
    if require_sample_count and mags.shape[0] < MIN_REQUIRED_SAMPLES:
        errors.append(
            f"Only {mags.shape[0]} samples available; at least {MIN_REQUIRED_SAMPLES} are recommended"
        )
    if not positive_definite:
        errors.append("Soft-iron correction matrix is not positive definite")
    if field_norm_cv > MAX_FIELD_NORM_CV:
        errors.append(
            f"Corrected field norm CV {field_norm_cv:.3f} exceeds max {MAX_FIELD_NORM_CV:.3f}"
        )
    if coverage:
        face_count = _as_int(coverage.get("face_count", 0), 0)
        octant_count = _as_int(coverage.get("octant_count", 0), 0)
        if face_count < MIN_REQUIRED_FACES or octant_count < MIN_REQUIRED_OCTANTS:
            errors.append(
                "Coverage insufficient: need all 6 major faces and at least 6 octants"
            )

    return {
        "valid": not errors,
        "errors": errors,
        "sample_count": int(mags.shape[0]),
        "field_norm_mean": field_norm_mean,
        "field_norm_std": field_norm_std,
        "field_norm_cv": field_norm_cv,
        "positive_definite": positive_definite,
        "soft_iron_eigenvalues": soft_eigenvalues.astype(float).tolist(),
        "semi_axis_lengths": semi_axis_lengths,
        "coverage": dict(coverage) if isinstance(coverage, Mapping) else {},
        "thresholds": {
            "min_samples": MIN_REQUIRED_SAMPLES,
            "min_faces": MIN_REQUIRED_FACES,
            "min_octants": MIN_REQUIRED_OCTANTS,
            "max_field_norm_cv": MAX_FIELD_NORM_CV,
        },
    }


def load_capture_csv(path: str) -> dict[str, Any]:
    """Load a capture CSV written by the calibration utility."""
    import csv

    mag_samples: list[list[float]] = []
    accel_samples: list[list[float]] = []
    rows: list[dict[str, Any]] = []

    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
            mag_samples.append(
                [
                    _as_float(row.get("mag_x", 0.0), 0.0),
                    _as_float(row.get("mag_y", 0.0), 0.0),
                    _as_float(row.get("mag_z", 0.0), 0.0),
                ]
            )
            accel_samples.append(
                [
                    _as_float(row.get("accel_x", 0.0), 0.0),
                    _as_float(row.get("accel_y", 0.0), 0.0),
                    _as_float(row.get("accel_z", 0.0), 0.0),
                ]
            )

    return {
        "rows": rows,
        "mag_samples": np.asarray(mag_samples, dtype=np.float64),
        "accel_samples": np.asarray(accel_samples, dtype=np.float64),
    }

