"""Second and final predeclared G1 training grid; never uses reserved runs."""
import hashlib
import itertools
import json
from pathlib import Path

from foundation.replay_closed_loop import run


TRAINING = [Path('foundation/datasets/d03_roll_pitch_seed2026_20261107/replay_input'),
            Path('foundation/datasets/d04_yaw_seed2026_20261107/replay_input'),
            Path('foundation/datasets/d03_roll_pitch_small_seed456_20261107/replay_input'),
            Path('foundation/datasets/d04_yaw_small_seed456_20261107/replay_input')]
TAUS = (.0005, .001, .002, .005)
DELAYS = (0., .002, .005, .010)
STEP_S = .00025


def evaluate(tau, delay):
    metrics = [run(p, .30, .50, step_s=STEP_S, motor_tau_s=tau, command_delay_s=delay,
                   experiment='V2 round 2 fit only; not acceptance')['metrics'] for p in TRAINING]
    return {'tau_s': tau, 'delay_s': delay, 'runs': metrics}


def main():
    rows = [evaluate(tau, delay) for tau, delay in itertools.product(TAUS, DELAYS)]
    baseline = next(x for x in rows if x['tau_s'] == .002 and x['delay_s'] == 0.)
    base_values = [m['omega']['rmse'] for m in baseline['runs']]
    if any(x <= 0 for x in base_values):
        raise ValueError('baseline angular-rate RMSE must be positive')
    for row in rows:
        row['omega_rmse_ratios_to_baseline'] = [m['omega']['rmse'] / b for m, b in zip(row['runs'], base_values)]
        row['normalized_objective'] = sum(row['omega_rmse_ratios_to_baseline']) / len(base_values)
    best = min(rows, key=lambda x: x['normalized_objective'])
    improvement = 1 - best['normalized_objective']
    result = {'status': 'last allowed training-grid round; not physical identification or acceptance',
              'protocol': 'v2_calibration_round2_protocol.md', 'training_directories': [str(p) for p in TRAINING],
              'step_s': STEP_S, 'baseline': baseline, 'best': best, 'all_candidates': rows,
              'training_objective_improvement_fraction': improvement,
              'meets_training_screen_only': improvement >= .1 and all(x <= 1.2 for x in best['omega_rmse_ratios_to_baseline']),
              'best_on_grid_boundary': best['tau_s'] in (min(TAUS), max(TAUS)) or best['delay_s'] in (min(DELAYS), max(DELAYS)),
              'source_sha256': {str(p / f): hashlib.sha256((p / f).read_bytes()).hexdigest() for p in TRAINING
                                for f in ('evaluation_truth.jsonl', 'evaluation_imu.jsonl', 'evaluation_pwm_commands.jsonl')},
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path(__file__), Path('foundation/quad.py'), Path('foundation/replay_closed_loop.py'), Path('foundation/compare.py'))},
              'limitations': ['posthoc initial velocity, assumed motor state and unverified ROS-to-actuation timing',
                              'sub-millisecond candidates are numerical sensitivity, not identifiable motor measurements',
                              'wind and collision remain unexcluded', 'training screen is not task acceptance']}
    Path('foundation/reports/v2_calibration_round2.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('objective', best['normalized_objective'], 'improvement', improvement, 'tau', best['tau_s'],
          'delay', best['delay_s'], 'ratios', best['omega_rmse_ratios_to_baseline'],
          'screen', result['meets_training_screen_only'], 'boundary', result['best_on_grid_boundary'])


if __name__ == '__main__':
    main()
