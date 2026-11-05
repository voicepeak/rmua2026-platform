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


def compare_to_official(ours: dict, official: dict) -> dict:
    keys = ("success_rate", "mean_gates", "mean_time_s")
    return {key: float(ours.get(key, 0.0)) - float(official.get(key, 0.0)) for key in keys}
