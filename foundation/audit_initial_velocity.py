"""Compare frozen-window causal and two-sided pose velocity estimates."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.audit_calibration_wind import RUNS


def fit(times, positions, at, *, causal):
    dt = times - at
    mask = ((dt >= -0.12) & (dt <= (0. if causal else 0.12)))
    t, p = dt[mask], positions[mask]
    if len(t) < 5 or t[0] >= 0 or (not causal and t[-1] <= 0):
        raise ValueError('insufficient pose samples')
    design = np.column_stack((np.ones(len(t)), t, t*t))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError('singular pose fit')
    coefficients = np.linalg.lstsq(design, p, rcond=None)[0]
    return coefficients[1], len(t)


def analyze(directory):
    paths = [directory/'commands.jsonl', directory/'capture.jsonl']
    raw = [p.read_bytes() for p in paths]
    commands = [json.loads(line) for line in raw[0].splitlines() if line]
    first = next(x['stamp_ns'] for x in commands if x.get('kind') == 'command')
    rows = [x for x in (json.loads(line) for line in raw[1].splitlines() if line) if x.get('topic') == 'pose_gt']
    stamps = np.array([x['stamp_ns'] for x in rows], dtype=np.int64)
    position = np.array([x['position'] for x in rows], dtype=float)
    if (first <= 0 or len(stamps) < 5 or np.any(stamps <= 0) or np.any(np.diff(stamps) <= 0)
            or position.shape != (len(stamps), 3) or not np.isfinite(position).all()):
        raise ValueError('invalid pose stream')
    times = (stamps - first).astype(float)*1e-9
    index = int(np.searchsorted(times, .3, side='left'))
    if index == len(times):
        raise ValueError('missing initial pose')
    at = times[index]
    centered, centered_n = fit(times, position, at, causal=False)
    causal, causal_n = fit(times, position, at, causal=True)
    difference = causal - centered
    return {'directory': str(directory).replace('\\', '/'), 'pose_start_after_command_s': float(at),
            'centered_sample_count': centered_n, 'causal_sample_count': causal_n,
            'centered_velocity_m_s': centered.tolist(), 'causal_velocity_m_s': causal.tolist(),
            'difference_m_s': difference.tolist(), 'difference_norm_m_s': float(np.linalg.norm(difference)),
            'constant_velocity_half_second_scale_m': float(np.linalg.norm(difference)*.5),
            'source_sha256': {p.name: hashlib.sha256(b).hexdigest() for p, b in zip(paths, raw)}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('--datasets', type=Path, default=Path('foundation/datasets'))
    a = p.parse_args()
    result = {'status': 'pose-estimator sensitivity, not measured velocity or model validation',
              'runs': [analyze(a.datasets/name) for name in RUNS],
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'limitations': ['neither estimate is an independently measured velocity',
                              'source time, actuation timing, wind and contact remain unverified',
                              'difference times duration is not trajectory prediction error']}
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    for run in result['runs']:
        print(run['directory'], run['difference_norm_m_s'], run['constant_velocity_half_second_scale_m'])


if __name__ == '__main__':
    main()
