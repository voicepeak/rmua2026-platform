"""Summarize passive ROS capture without interpreting unknown wire payloads."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(path, mode="passive"):
    rows = [json.loads(s) for s in path.read_text(encoding="utf-8").splitlines() if s.strip()]
    groups = collections.defaultdict(list)
    for row in rows:
        groups[row["topic"]].append(row)
    summary = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
               "mode": mode,
               "limitations": ["ground-start observation; contact and free-flight effects may mix",
                               "debug truth is diagnostic only",
                               "no command or actuator latency calibration"], "topics": {}}
    for name, items in sorted(groups.items()):
        entry = {"count": len(items)}
        if "stamp_ns" in items[0]:
            stamps = np.array([r["stamp_ns"] for r in items], dtype=np.int64)
            delta = np.diff(stamps) * 1e-9
            entry.update({"duplicate_stamps": int(np.sum(delta == 0)),
                          "backwards_stamps": int(np.sum(delta < 0)),
                          "span_s": float((stamps[-1] - stamps[0]) * 1e-9),
                          "interval_median_s": float(np.median(delta)) if len(delta) else None,
                          "interval_max_s": float(np.max(delta)) if len(delta) else None})
        if name == "pose_gt":
            pos = np.array([r["position"] for r in items], dtype=float)
            entry["position_min_m"] = pos.min(axis=0).tolist()
            entry["position_max_m"] = pos.max(axis=0).tolist()
            entry["displacement_from_first_max_m"] = float(np.linalg.norm(pos - pos[0], axis=1).max())
        if name == "imu":
            accel = np.array([r["acceleration_body"] for r in items], dtype=float)
            omega = np.array([r["omega_body"] for r in items], dtype=float)
            entry["acceleration_mean_m_s2"] = accel.mean(axis=0).tolist()
            entry["omega_mean_rad_s"] = omega.mean(axis=0).tolist()
        if "type" in items[0]:
            entry["wire_types"] = sorted({(r.get("type") or "") + ":" + (r.get("md5sum") or "") for r in items})
        summary["topics"][name] = entry
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--mode", choices=("passive", "collective"), default="passive")
    a = p.parse_args()
    a.output.write_text(json.dumps(analyze(a.capture, a.mode), indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
