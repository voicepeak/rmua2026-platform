"""Measure official pose source-time vs host-arrival real-time factor.

This is a same-machine real-time observation, not a physics-tick throughput
measurement or a 64/256 environment-command comparison.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path('foundation/datasets')
OUTPUT = Path('foundation/reports/v2_official_realtime_factor.json')
RUNS = ('v2_official_passive_smoke_20261109',
        'v2_official_heldout_mix_small_seed260005_20261109')


def analyze(run):
    directory = ROOT / run
    rows = [json.loads(line) for line in (directory / 'capture.jsonl').read_text(encoding='utf-8').splitlines() if line]
    pose = [row for row in rows if row.get('topic') == 'pose_gt']
    if len(pose) < 2:
        raise ValueError('missing pose samples')
    host = np.array([row['arrival_monotonic_ns'] for row in pose], float)
    source = np.array([row['stamp_ns'] for row in pose], float)
    host_duration = (host[-1] - host[0]) * 1e-9
    source_duration = (source[-1] - source[0]) * 1e-9
    if host_duration <= 0 or source_duration <= 0:
        raise ValueError('invalid durations')
    return {
        'run': run,
        'pose_samples': len(pose),
        'host_duration_s': host_duration,
        'source_duration_s': source_duration,
        'source_to_host_ratio': source_duration / host_duration,
        'capture_sha256': hashlib.sha256((directory / 'capture.jsonl').read_bytes()).hexdigest(),
        'run_sha256': hashlib.sha256((directory / 'run.json').read_bytes()).hexdigest(),
    }


def main():
    runs = [analyze(run) for run in RUNS]
    result = {
        'status': 'official pose source-vs-arrival real-time factor; not environment throughput',
        'runs': runs,
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'source Header is not an independently decoded physics tick',
            'arrival time includes transport and scheduling',
            'official runs are real-time; this cannot be converted to 64/256 env-command throughput',
            'no same-operation official batch benchmark was obtained',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for run in runs:
        print(run['run'], run['source_to_host_ratio'])


if __name__ == '__main__':
    main()
