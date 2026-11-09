"""Consolidate observed four-motor differential pulse signs.

The sign comparison uses the candidate FRD body frame and the published motor
order. It supports channel/sign consistency in two processes; it does not
decode runtime motor identity or verify the full operating envelope.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.quad import Params, Quad


ROOT = Path('foundation/datasets')
OUTPUT = Path('foundation/reports/v2_motor_direction_audit.json')
RUNS = ('d03_differential_retry_20261107', 'd03_differential_seed456_20261107',
        'v2_official_differential_seed260007_20261109')


def candidate_signs():
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    signs = []
    for motor in range(4):
        signal = np.zeros(4)
        signal[motor] = .2
        quad = Quad(params)
        quad.reset(rotor_signal=signal)
        omega = quad.step(signal, .001)['omega']
        signs.append([int(np.sign(value)) for value in omega])
    return signs


def observed_signs(phase):
    slopes = np.asarray(phase['angular_rate_slope_rad_s2'], float)
    if slopes.shape != (3,) or not np.isfinite(slopes).all():
        raise ValueError('invalid angular-rate slope')
    return [int(np.sign(value)) if abs(value) >= .1 else 0 for value in slopes]


def analyze(run):
    path = ROOT / run / 'phase_analysis.json'
    data = json.loads(path.read_bytes())
    phases = data['phases']
    if len(phases) != 7:
        raise ValueError('expected seven phases')
    observed = [observed_signs(phase) for phase in phases[1:5]]
    return {
        'run': run,
        'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'pulse_pwm': [phase['pwm'] for phase in phases[1:5]],
        'observed_signs': observed,
        'samples': [phase['angular_slope_samples'] for phase in phases[1:5]],
        'limitations': data.get('limitations', []),
    }


def main():
    expected = candidate_signs()
    runs = [analyze(run) for run in RUNS]
    for run in runs:
        run['matches_candidate_xy'] = [obs[:2] == exp[:2] for obs, exp in zip(run['observed_signs'], expected)]
        run['z_sign_resolved'] = [obs[2] != 0 for obs in run['observed_signs']]
    result = {
        'status': 'two-process differential sign consistency; not full motor identity or envelope validation',
        'published_motor_order': ['front_right', 'rear_left', 'front_left', 'rear_right'],
        'candidate_frd_signs': expected,
        'runs': runs,
        'all_motor_xy_signs_match_in_each_run': all(all(run['matches_candidate_xy']) for run in runs),
        'any_z_sign_resolved': any(any(run['z_sign_resolved']) for run in runs),
        'cross_run_signs_identical': runs[0]['observed_signs'] == runs[1]['observed_signs'],
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'the candidate FRD frame and published motor order are assumptions, not independently decoded runtime labels',
            'only roll/pitch (x/y) signs are resolved; yaw (z) responses are below the 0.1 rad/s^2 reporting threshold',
            'pulse slopes include prior motion, gravity coupling and unknown actuation latency',
            'ground contact and later flight are not classified per phase',
            'only two processes of one differential waveform; not the full operating envelope',
            'sign consistency does not establish PWM range, clipping, hold or timeout',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('candidate signs', expected)
    for run in runs:
        print(run['run'], run['observed_signs'], run['matches_candidate_xy'], run['z_sign_resolved'])


if __name__ == '__main__':
    main()
