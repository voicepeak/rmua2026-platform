"""Summarize source/arrival clock ratios without assuming a physical clock mapping."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize(directory, start=.30, end=.80):
    paths = [directory / name for name in ('run.json', 'commands.jsonl', 'capture.jsonl')]
    raw = [p.read_bytes() for p in paths]
    run = json.loads(raw[0])
    if run['exit_status'] != 0:
        raise ValueError(f'{directory}: failed run')
    commands = [json.loads(line) for line in raw[1].splitlines() if line]
    origin = next(row['monotonic_ns'] for row in commands if row.get('kind') == 'start')
    poses = [row for row in (json.loads(line) for line in raw[2].splitlines() if line)
             if row.get('topic') == 'pose_gt' and row.get('stamp_ns')]
    host = np.array([(row['arrival_monotonic_ns'] - origin) * 1e-9 for row in poses])
    if not poses:
        raise ValueError(f'{directory}: no stamped poses')
    epoch = poses[0]['stamp_ns']
    source = np.array([(row['stamp_ns'] - epoch) * 1e-9 for row in poses])
    if np.any(np.diff(host) <= 0) or np.any(np.diff(source) <= 0):
        raise ValueError(f'{directory}: nonmonotonic timestamps')
    selected = (host >= start) & (host <= end)
    h, s = host[selected], source[selected]
    if len(h) < 5 or np.max(np.diff(h)) > .1:
        raise ValueError(f'{directory}: insufficient coverage')
    return {'directory': str(directory), 'samples': len(h), 'host_interval_s': [float(h[0]), float(h[-1])],
            'elapsed_source_over_host': float((s[-1] - s[0]) / (h[-1] - h[0])),
            'host_gap_s_max': float(np.max(np.diff(h))),
            'source_sha256': {p.name: hashlib.sha256(data).hexdigest() for p, data in zip(paths, raw)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('runs', nargs='+', type=Path)
    args = parser.parse_args()
    result = {'status': 'descriptive comparison, not a controlled measurement of debugger effects',
              'window_host_s': [.30, .80], 'runs': [summarize(p) for p in args.runs],
              'limitations': ['different seeds and run conditions can affect elapsed ratios',
                              'arrival timestamps include variable queue and scheduling delay',
                              'header clock semantics and true physics time remain unverified'],
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for row in result['runs']:
        print(row['directory'], row['samples'], row['elapsed_source_over_host'])


if __name__ == '__main__':
    main()
