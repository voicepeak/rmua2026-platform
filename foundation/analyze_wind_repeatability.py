"""Compare repeated injected-wind trajectories on the command-header clock.

This characterizes repeatability only; it cannot identify wind force without a
controlled no-wind counterfactual and matching applied motor commands.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(run):
    command = run / 'pwm_commands.jsonl'
    pose = run / 'pose_gt_posthoc.csv'
    wind = run / 'before_reset_wind.csv'
    commands = [json.loads(line) for line in command.read_text(encoding='utf-8').splitlines() if line]
    origin = int(commands[0]['header_ns'])

    def read(path, fields):
        with path.open(encoding='utf-8', newline='') as f:
            rows = list(csv.DictReader(f))
        t = np.array([(int(row['field.header.stamp']) - origin) * 1e-9 for row in rows])
        values = np.array([[float(row[field]) for field in fields] for row in rows])
        if len(t) < 2 or np.any(np.diff(t) < 0) or not np.isfinite(values).all():
            raise ValueError(f'invalid or nonmonotonic stream: {path}')
        # Keep the final observation when a publisher repeats a timestamp.
        keep = np.r_[np.diff(t) > 0, True]
        t, values = t[keep], values[keep]
        if len(t) < 2:
            raise ValueError(f'insufficient distinct timestamps: {path}')
        return t, values

    pt, pv = read(pose, [f'field.pose.position.{axis}' for axis in 'xyz'])
    wt, wv = read(wind, [f'field.vector.{axis}' for axis in 'xyz'])
    ct = np.array([(int(c['header_ns']) - origin) * 1e-9 for c in commands])
    cv = np.array([c['command'] for c in commands], dtype=float)
    if len(ct) < 2 or np.any(np.diff(ct) <= 0) or cv.shape != (len(ct), 4):
        raise ValueError(f'invalid commands: {command}')
    return {'pose': (pt, pv), 'wind': (wt, wv), 'command': (ct, cv),
            'sha256': {p.name: digest(p) for p in (command, pose, wind)}}


def compare(a, b, duration=4.8, interval=.05):
    if duration <= 0 or interval <= 0:
        raise ValueError('duration and interval must be positive')
    lo = max(0., *(r[s][0][0] for r in (a, b) for s in ('pose', 'wind', 'command')))
    hi = min(duration, *(r[s][0][-1] for r in (a, b) for s in ('pose', 'wind', 'command')))
    t = np.arange(lo, hi + 1e-9, interval)
    if len(t) < 2:
        raise ValueError('streams have no useful shared time interval')

    def sample(run, name):
        x, y = run[name]
        if name == 'command':
            return y[np.searchsorted(x, t, side='right') - 1]
        return np.column_stack([np.interp(t, x, y[:, i]) for i in range(y.shape[1])])

    result = {'interval_s': [float(t[0]), float(t[-1])], 'sample_count': len(t)}
    for name, unit in [('pose', 'm'), ('wind', 'm_s'), ('command', 'pwm')]:
        delta = np.linalg.norm(sample(a, name) - sample(b, name), axis=1)
        result[name + '_difference_' + unit] = {
            'median': float(np.median(delta)), 'p95': float(np.percentile(delta, 95)),
            'max': float(delta.max())}
    result['source_sha256'] = [a['sha256'], b['sha256']]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    runs = {s: [load(args.experiment / f'seed{s}_repeat{i}') for i in (1, 2)]
            for s in (123, 456, 2026)}
    result = {'method': '50 ms grid, relative to first command header; duplicate stream timestamps keep last; linear pose/wind interpolation; zero-order-hold command; no extrapolation',
              'limitations': ['diagnostic injected stage and wind, not natural progression',
                              'command publication is not verified applied motor input',
                              'header alignment does not establish physical synchronization',
                              'repeat variability is not a causal estimate of wind force',
                              'trajectory may include ground contact and other environmental effects'],
              'within_seed': {str(s): compare(*r) for s, r in runs.items()},
              'across_seed_repeat1': {f'{a}_{b}': compare(runs[a][0], runs[b][0])
                                      for a, b in ((123, 456), (123, 2026), (456, 2026))}}
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: {pair: {m: v for m, v in data.items() if m.endswith(('_m', '_m_s', '_pwm'))}
                          for pair, data in result[k].items()} for k in ('within_seed', 'across_seed_repeat1')}, indent=2))


if __name__ == '__main__':
    main()
