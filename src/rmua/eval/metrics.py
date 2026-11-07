"""Evaluation metrics aligned with the official ranking."""

from __future__ import annotations

from collections import Counter

import numpy as np


def summarize(info: dict) -> dict:
    success = np.asarray(info.get("success", []), dtype=bool)
    gates = np.asarray(info.get("gates_passed", []), dtype=np.float64)
    times = np.asarray(info.get("segment_time", []), dtype=np.float64)
    reasons = list(info.get("failure_reason", []))
    return {
        "episodes": int(success.size),
        "success_rate": float(success.mean()) if success.size else 0.0,
        "mean_gates": float(gates.mean()) if gates.size else 0.0,
        "mean_time_s": float(times.mean()) if times.size else 0.0,
        "failure_counts": dict(Counter(reasons)),
    }


def summarize_episodes(
    info: dict,
    required_gates: int | None = None,
    terminated=None,
    truncated=None,
    track_length_m: float | None = None,
) -> dict:
    """Per-episode summary including failure reasons and completion speed.

    ``mean_time_s`` and ``mean_speed_mps`` cover successful completions.
    ``mean_time_all_s`` and ``mean_progress_speed_mps`` cover every episode so
    failures cannot be hidden by only reporting successful samples.
    """
    success = np.asarray(info.get("success", []), dtype=bool)
    gates = np.asarray(info.get("gates_passed", []), dtype=np.float64)
    times = np.asarray(info.get("segment_time", []), dtype=np.float64)
    progress = np.asarray(
        info.get("progress_m", np.zeros_like(times)), dtype=np.float64
    )
    reasons = list(info.get("failure_reason", [""] * success.size))
    n = int(success.size)
    term = (
        np.zeros(n, dtype=bool)
        if terminated is None
        else np.asarray(terminated, dtype=bool)
    )
    trunc = (
        np.zeros(n, dtype=bool)
        if truncated is None
        else np.asarray(truncated, dtype=bool)
    )
    labels = []
    for index in range(n):
        reason = str(reasons[index]) if index < len(reasons) else ""
        if success[index]:
            labels.append("success")
        elif term[index] and reason:
            labels.append(reason)
        elif trunc[index]:
            labels.append("truncated")
        elif reason:
            labels.append(reason)
        else:
            labels.append("running")
    success_times = times[success]
    time_mask = times > 0
    progress_speeds = progress[time_mask] / times[time_mask]
    completion_speeds = (
        np.asarray(track_length_m, dtype=np.float64) / success_times
        if track_length_m is not None and success_times.size
        else np.array([])
    )
    return {
        "episodes": n,
        "success_rate": float(success.mean()) if n else 0.0,
        "full_gate_rate": (
            float(np.mean(gates >= required_gates))
            if required_gates is not None and n
            else 0.0
        ),
        "gates_required": int(required_gates) if required_gates is not None else 0,
        "mean_gates": float(gates.mean()) if n else 0.0,
        "mean_time_s": float(success_times.mean()) if success_times.size else 0.0,
        "mean_time_all_s": float(times.mean()) if n else 0.0,
        "mean_speed_mps": float(completion_speeds.mean()) if completion_speeds.size else 0.0,
        "mean_progress_speed_mps": (
            float(progress_speeds.mean()) if progress_speeds.size else 0.0
        ),
        "failure_counts": dict(Counter(labels)),
    }


def compare_to_official(ours: dict, official: dict) -> dict:
    keys = ("success_rate", "mean_gates", "mean_time_s")
    return {key: float(ours.get(key, 0.0)) - float(official.get(key, 0.0)) for key in keys}
