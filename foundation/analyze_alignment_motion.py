"""Characterize stationary bias and movement residuals in paired frame data."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(path, speed_threshold=.2, residual_threshold_cm=5.):
    data = json.loads(path.read_text(encoding='utf-8'))
    pairs = data['samples']
    if len(pairs) < 3 or speed_threshold <= 0 or residual_threshold_cm <= 0:
        raise ValueError('insufficient pairs or invalid thresholds')
    pose = np.array([p['pose_ned_m'] for p in pairs])
    root = np.array([p['actor_root_cm'] for p in pairs])
    t = np.array([p['gdb_monotonic_s'] + (p['clock_gap_s'] if 'clock_gap_s' in p else p['arrival_gap_s']) for p in pairs])
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError('non-increasing paired pose times')
    speed = np.linalg.norm(np.diff(pose, axis=0), axis=1)/dt
    origins = np.array([p['candidate_origin_cm'] for p in pairs])
    residual = np.linalg.norm(origins-np.median(origins, axis=0), axis=1)
    moving = speed > speed_threshold
    high = residual[1:] > residual_threshold_cm
    _, counts = np.unique(root, axis=0, return_counts=True)
    return {'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'pairs': len(pairs),
            'unique_actor_root_values': len(counts), 'most_common_actor_root_count': int(counts.max()),
            'unique_ros_pose_values': int(len(np.unique(pose, axis=0))),
            'speed_threshold_m_s': speed_threshold, 'residual_threshold_cm': residual_threshold_cm,
            'moving_intervals': int(moving.sum()), 'high_residual_pairs': int((residual > residual_threshold_cm).sum()),
            'moving_and_high_residual': int((moving & high).sum()),
            'limitations': ['speed is finite difference of asynchronous paired ROS pose samples',
                            'exact repeated values are not proof that a physical body was stationary',
                            'correlation does not identify delay, root-update semantics, or cause',
                            'debugger and injected collision fixture affect the experiment']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.input)
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','limitations')}, indent=2))


if __name__ == '__main__':
    main()
