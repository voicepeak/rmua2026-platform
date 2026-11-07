"""Characterize small collective PWM steps; not an actuator identification."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.analyze_collective import analyze, read
from foundation.fit_vertical_step import curve


def analyze_steps(directory):
    phase = analyze(directory)
    commands = [r for r in read(directory / 'commands.jsonl') if r.get('kind') == 'command']
    edges = []
    for row in commands:
        if not edges or row['pwm'] != edges[-1]['pwm']:
            edges.append(row)
    imu = [r for r in read(directory / 'capture.jsonl') if r.get('topic') == 'imu']
    times = np.array([r['arrival_monotonic_ns'] for r in imu], dtype=np.int64)
    z = np.array([r['acceleration_body'][2] for r in imu], dtype=float)
    if len(times) < 20 or np.any(np.diff(times) <= 0) or not np.isfinite(z).all():
        raise ValueError('invalid IMU stream')
    results = []
    for i in range(2, len(edges) - 1):
        previous, edge, next_edge = edges[i-1:i+2]
        if len(set(edge['pwm'])) != 1 or len(set(previous['pwm'])) != 1:
            raise ValueError('non-collective input')
        t = (times - edge['monotonic_ns']) * 1e-9
        before = (t >= -.08) & (t < -.02)
        after = (t >= .12) & (times < next_edge['monotonic_ns'] - 10_000_000)
        fit = (t >= -.08) & (t <= .14)
        if min(before.sum(), after.sum(), fit.sum()) < 4:
            raise ValueError('insufficient samples for a step')
        best = None
        for delay in np.arange(0, .0601, .002):
            for tau in np.arange(.002, .0801, .002):
                design = np.column_stack((np.ones(fit.sum()), curve(t[fit], delay, tau)))
                coeff = np.linalg.lstsq(design, z[fit], rcond=None)[0]
                mse = float(np.mean((z[fit] - design @ coeff) ** 2))
                if best is None or mse < best['mse']:
                    best = {'delay_s': float(delay), 'tau_s': float(tau),
                            'change_m_s2': float(coeff[1]), 'mse': mse}
        best['tau_at_boundary'] = best['tau_s'] <= .0020001 or best['tau_s'] >= .0799999
        best['delay_at_boundary'] = best['delay_s'] <= 1e-9 or best['delay_s'] >= .0599999
        results.append({'from_pwm': previous['pwm'][0], 'to_pwm': edge['pwm'][0],
                        'pre_samples': int(before.sum()), 'post_samples': int(after.sum()),
                        'pre_median_m_s2': float(np.median(z[before])),
                        'post_median_m_s2': float(np.median(z[after])),
                        'effective_first_order_fit': best})
    return {'status': phase['status'], 'steps': results,
            'source_sha256': {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
                              for name in ('commands.jsonl', 'capture.jsonl')}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', type=Path, nargs='+')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    result = {'runs': {str(path): analyze_steps(path) for path in args.runs},
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'limitations': ['post-publish to subscriber-arrival timing combines transport, scheduler, bridge and sensor response',
                              'first-order fit is exploratory, highly correlated, and has no confidence interval',
                              'body-z specific force is not a direct motor sensor; orientation, contact and prior state are not controlled',
                              'two seeds use the same short waveform and are not a held-out excitation']}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for name, run in result['runs'].items():
        print(name, [(x['from_pwm'], x['to_pwm'], round(x['post_median_m_s2'] - x['pre_median_m_s2'], 3),
                      x['effective_first_order_fit']['tau_s']) for x in run['steps']])


if __name__ == '__main__':
    main()
