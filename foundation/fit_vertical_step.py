"""Fit an exploratory IMU step response, preserving latency ambiguities.

Uses command publish-side and IMU arrival-side host monotonic timestamps. The
calibration run determines delay, time constant, baseline and step amplitude;
the other runs are evaluated with those exact four values, without refitting.
It is an effective sensor/command response, not an isolated motor time constant.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def load(directory):
    commands_path, capture_path = directory/'commands.jsonl', directory/'capture.jsonl'
    commands = [x for x in read(commands_path) if x.get('kind') == 'command']
    edges = []
    for x in commands:
        if not edges or x['pwm'] != edges[-1]['pwm']:
            edges.append(x)
    matching = [x for i, x in enumerate(edges) if i > 0 and
                np.allclose(edges[i-1]['pwm'], [.2]*4) and np.allclose(x['pwm'], [.16]*4)]
    if len(matching) != 1:
        raise ValueError('expected one .20 to .16 edge')
    edge = matching[0]['monotonic_ns']
    rows = read(capture_path)
    imu = [x for x in rows if x.get('topic') == 'imu' and -.18 <= (x['arrival_monotonic_ns']-edge)*1e-9 <= .30]
    pose = [x for x in rows if x.get('topic') == 'pose_gt' and -.18 <= (x['arrival_monotonic_ns']-edge)*1e-9 <= .30]
    t = np.array([(x['arrival_monotonic_ns']-edge)*1e-9 for x in imu])
    y = np.array([x['acceleration_body'][2] for x in imu], float)
    if len(t) < 20 or np.any(np.diff(t) <= 0) or not np.isfinite(y).all() or not pose:
        raise ValueError('insufficient or invalid IMU/pose observations')
    z = [x['position'][2] for x in pose]
    return {'t': t, 'y': y, 'z_range': [min(z), max(z)], 'edge_ns': edge,
            'hashes': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (commands_path, capture_path)}}


def curve(t, delay, tau):
    return -np.expm1(-np.maximum(t-delay, 0)/tau)


def fit(run):
    t, y = run['t'], run['y']
    best = None
    for delay in np.arange(0, .1201, .002):
        for tau in np.arange(.002, .1501, .002):
            x = np.column_stack((np.ones(len(t)), curve(t, delay, tau)))
            coeff = np.linalg.lstsq(x, y, rcond=None)[0]
            residual = y - x@coeff
            mse = float(np.mean(residual**2))
            if best is None or mse < best['mse']:
                best = {'delay_s': float(delay), 'tau_s': float(tau),
                        'baseline_m_s2': float(coeff[0]), 'change_m_s2': float(coeff[1]), 'mse': mse}
    return best


def evaluate(run, params):
    prediction = params['baseline_m_s2'] + params['change_m_s2']*curve(run['t'], params['delay_s'], params['tau_s'])
    error = run['y']-prediction
    return {'samples': len(error), 'imu_z_rmse_m_s2': float(np.sqrt(np.mean(error**2))),
            'imu_z_max_abs_error_m_s2': float(np.max(np.abs(error))),
            'pose_z_range_m': run['z_range'], 'source_sha256': run['hashes']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fit', type=Path, required=True)
    p.add_argument('--holdout', type=Path, action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    training = load(a.fit)
    params = fit(training)
    params['tau_at_search_boundary'] = bool(params['tau_s'] <= .002000001 or params['tau_s'] >= .149999999)
    params['delay_at_search_boundary'] = bool(params['delay_s'] <= 1e-12 or params['delay_s'] >= .119999999)
    result = {'model': 'post-publish to IMU-arrival effective first-order step',
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'fit_run': str(a.fit), 'fit_parameters': params, 'fit_metrics': evaluate(training, params),
              'holdout': {str(path): evaluate(load(path), params) for path in a.holdout},
              'limitations': ['not isolated motor response: includes publish, scheduling, bridge, actuator and IMU effects',
                              'fitted delay and time constant are correlated; boundary estimates are unresolved and have no confidence interval',
                              'one decrease at one thrust level and short window; geometry/contact unverified',
                              'holdout runs use different command rates and are not independent waveform designs',
                              'no physics-step or control-timeout inference']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'parameters': params, 'holdout_rmse': {k: v['imu_z_rmse_m_s2'] for k, v in result['holdout'].items()}}, indent=2))


if __name__ == '__main__':
    main()
