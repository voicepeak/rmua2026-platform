"""Track identity checks for training checkpoints.

A policy is only valid on the exact track it was trained on. New checkpoints
store the map, track mode, gate count and geometry fingerprint. Loading code
must reject a mismatch instead of silently falling back to the legacy proxy.
"""

from __future__ import annotations

TRACK_METADATA_VERSION = 1
OBSERVATION_SOURCE = "sim_truth_state_only"

_EXACT_KEYS = (
    "map_name",
    "track_mode",
    "segment_id",
    "scoring_gate_count",
    "audited_stage1",
)
_FLOAT_KEYS = ("detection_gate_width_m", "detection_gate_height_m")
_REQUIRED_KEYS = (
    "metadata_version",
    *_EXACT_KEYS,
    *_FLOAT_KEYS,
    "total_length_m",
)


def track_metadata(env) -> dict:
    """Describe the environment track for checkpoint compatibility checks."""
    config = env.config
    track = env.track
    return {
        "metadata_version": TRACK_METADATA_VERSION,
        "map_name": str(config.map_name),
        "track_mode": str(config.track_mode),
        "segment_id": int(config.segment_id),
        "total_length_m": round(float(track.total_length_m), 3),
        "scoring_gate_count": int(len(track.scoring_gates)),
        "audited_stage1": bool(track.audited_stage1),
        "detection_gate_width_m": float(config.detection_gate_width_m),
        "detection_gate_height_m": float(config.detection_gate_height_m),
        "observation_source": OBSERVATION_SOURCE,
    }


def missing_track_metadata_error(checkpoint: str = "checkpoint") -> ValueError:
    return ValueError(
        f"{checkpoint} has no track metadata (legacy checkpoint). "
        "Refusing to guess its track; old proxy checkpoints are invalid for "
        "stage1_audited. Pass --allow-legacy-proxy only for legacy proxy "
        "evaluation."
    )


def validate_track_metadata(
    saved: dict | None,
    current: dict,
    length_tolerance_m: float = 0.5,
    checkpoint: str = "checkpoint",
) -> None:
    """Raise ValueError when a checkpoint does not match the built track."""
    if not saved:
        raise missing_track_metadata_error(checkpoint)
    missing = [key for key in _REQUIRED_KEYS if key not in saved]
    if missing:
        raise ValueError(
            f"{checkpoint} track metadata is incomplete ({missing}); refusing "
            "to load it onto a possibly different track."
        )
    mismatches = []
    for key in _EXACT_KEYS:
        if saved.get(key) != current.get(key):
            mismatches.append(f"{key}: checkpoint={saved.get(key)!r} current={current.get(key)!r}")
    for key in _FLOAT_KEYS:
        if abs(float(saved.get(key)) - float(current.get(key))) > 1e-9:
            mismatches.append(f"{key}: checkpoint={saved.get(key)!r} current={current.get(key)!r}")
    length_delta = abs(float(saved["total_length_m"]) - float(current["total_length_m"]))
    if length_delta > length_tolerance_m:
        mismatches.append(
            "total_length_m: "
            f"checkpoint={saved['total_length_m']!r} current={current['total_length_m']!r} "
            f"(tolerance {length_tolerance_m} m)"
        )
    if mismatches:
        raise ValueError(
            f"{checkpoint} track identity does not match the environment:\n  "
            + "\n  ".join(mismatches)
        )
