"""Compare timestamped trajectories without extrapolation.

Input JSON files contain arrays of samples with t, position, velocity,
quaternion_wxyz, omega. Both sides must already use the same frame and units.
An explicit time offset shifts the candidate timestamps; it is never fitted to
the evaluation trajectory automatically. Quaternions are interpolated by SLERP.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

FIELDS = {"position": 3, "velocity": 3, "quaternion_wxyz": 4, "omega": 3}


def validate(samples):
    if len(samples) < 2:
        raise ValueError("at least two samples are required")
    t = np.array([s["t"] for s in samples], dtype=float)
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError("timestamps must be finite and strictly increasing")
    values = {}
    for key, width in FIELDS.items():
        value = np.asarray([s[key] for s in samples], dtype=float)
        if value.shape != (len(t), width) or not np.all(np.isfinite(value)):
            raise ValueError(f"invalid {key}")
        values[key] = value
    q = values["quaternion_wxyz"]
    norm = np.linalg.norm(q, axis=1)
    if np.any(norm < 1e-12):
        raise ValueError("zero quaternion")
    values["quaternion_wxyz"] = q / norm[:, None]
    return t, values


def rotations(wxyz):
    return Rotation.from_quat(wxyz[:, [1, 2, 3, 0]])


def compare(reference, candidate, time_offset=0.0, max_gap=None):
    if not np.isfinite(time_offset):
        raise ValueError("time offset must be finite")
    if max_gap is not None and (not np.isfinite(max_gap) or max_gap <= 0):
        raise ValueError("max_gap must be positive and finite")
    rt, rv = validate(reference)
    ct, cv = validate(candidate)
    ct = ct + time_offset
    mask = (rt >= ct[0]) & (rt <= ct[-1])
    # Reject interpolations across large gaps, but preserve exact samples.
    if max_gap is not None:
        right = np.clip(np.searchsorted(ct, rt, side="left"), 0, len(ct) - 1)
        left = np.maximum(right - 1, 0)
        exact = np.isclose(rt, ct[right], rtol=0, atol=1e-9)
        mask &= exact | ((ct[right] - ct[left]) <= max_gap)
    times = rt[mask]
    if len(times) < 2:
        raise ValueError("fewer than two overlapping samples after gap filtering")
    errors = {}
    for key in ("position", "velocity", "omega"):
        pred = np.column_stack([np.interp(times, ct, cv[key][:, i]) for i in range(3)])
        errors[key] = np.linalg.norm(pred - rv[key][mask], axis=1)
    interp = Slerp(ct, rotations(cv["quaternion_wxyz"]))(times)
    ref = rotations(rv["quaternion_wxyz"][mask])
    errors["attitude_deg"] = np.rad2deg((ref.inv() * interp).magnitude())
    metrics = {}
    for key, err in errors.items():
        metrics[key] = {"rmse": float(np.sqrt(np.mean(err ** 2))),
                        "p95": float(np.percentile(err, 95)), "max": float(np.max(err))}
    return {"n_reference": len(rt), "n_matched": len(times),
            "start": float(times[0]), "end": float(times[-1]),
            "candidate_time_offset_s": float(time_offset), "max_gap_s": max_gap,
            "metrics": metrics,
            "samples": [{"t": float(t), **{k: float(v[i]) for k, v in errors.items()}}
                        for i, t in enumerate(times)]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("reference", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--time-offset", type=float, default=0.0)
    p.add_argument("--max-gap", type=float, required=True)
    a = p.parse_args()
    result = compare(json.loads(a.reference.read_text(encoding="utf-8")),
                     json.loads(a.candidate.read_text(encoding="utf-8")),
                     a.time_offset, a.max_gap)
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
