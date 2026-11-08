"""Compare fixed round-1 candidate and baseline on different repeat processes."""
import hashlib
import json
from pathlib import Path

from foundation.replay_closed_loop import run


def main():
    directories = [Path('foundation/datasets/v2_roll_pitch_seed314159_20261108/replay_input'),
                   Path('foundation/datasets/v2_yaw_seed27182_20261108/replay_input')]
    reports = {}
    for name, tau in [('baseline', .005), ('candidate', .002)]:
        reports[name] = [run(d, .30, .50, step_s=.002, motor_tau_s=tau, command_delay_s=0.,
                             experiment='V2 round 1 repeat check; not held-out acceptance') for d in directories]
    baseline = [r['metrics']['omega']['rmse'] for r in reports['baseline']]
    candidate = [r['metrics']['omega']['rmse'] for r in reports['candidate']]
    ratios = [a / b for a, b in zip(candidate, baseline)]
    improvement = 1 - sum(candidate) / sum(baseline)
    result = {'status': 'same waveform, different process; not held-out acceptance',
              'protocol': 'v2_calibration_round1_protocol.md', 'directories': [str(d) for d in directories],
              'baseline_tau_s': .005, 'candidate_tau_s': .002, 'delay_s': 0.,
              'baseline_metrics': [r['metrics'] for r in reports['baseline']],
              'candidate_metrics': [r['metrics'] for r in reports['candidate']],
              'omega_rmse_ratios': ratios, 'mean_omega_rmse_improvement_fraction': improvement,
              'meets_engineering_screen_only': improvement >= .10 and all(x <= 1.20 for x in ratios),
              'source_sha256': {str(d / f): hashlib.sha256((d / f).read_bytes()).hexdigest() for d in directories
                                for f in ('evaluation_truth.jsonl', 'evaluation_imu.jsonl', 'evaluation_pwm_commands.jsonl')},
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path(__file__), Path('foundation/quad.py'), Path('foundation/replay_closed_loop.py'), Path('foundation/compare.py'))},
              'limitations': ['same waveform types seen during fitting', 'initial velocity estimated using future pose',
                              'actuation time, motor initial state, contact and wind unverified',
                              'engineering screen is not an official or task-derived threshold']}
    Path('foundation/reports/v2_calibration_round1_repeat.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('baseline omega', baseline, 'candidate omega', candidate, 'ratios', ratios,
          'improvement', improvement, 'screen', result['meets_engineering_screen_only'])


if __name__ == '__main__':
    main()
