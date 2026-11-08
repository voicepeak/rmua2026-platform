"""Predeclared training-only G1 delay/tau sensitivity grid; no holdout use."""
import hashlib
import itertools
import json
from pathlib import Path

from foundation.replay_closed_loop import run


TRAINING = [Path('foundation/datasets/d03_roll_pitch_seed2026_20261107/replay_input'),
            Path('foundation/datasets/d04_yaw_seed2026_20261107/replay_input')]
TAUS = (.002, .005, .010, .020)
DELAYS = (0., .010, .020, .040)


def main():
    rows = []
    for tau, delay in itertools.product(TAUS, DELAYS):
        reports = [run(path, .30, .50, step_s=.002,
                       experiment='V2 round 1 fit data only; not acceptance',
                       command_delay_s=delay, motor_tau_s=tau) for path in TRAINING]
        metrics = [report['metrics'] for report in reports]
        rows.append({'tau_s': tau, 'delay_s': delay, 'runs': metrics,
                     'mean_omega_rmse_rad_s': sum(m['omega']['rmse'] for m in metrics) / len(metrics)})
    baseline = next(x for x in rows if x['tau_s'] == .005 and x['delay_s'] == 0.)
    best = min(rows, key=lambda x: x['mean_omega_rmse_rad_s'])
    improvement = 1 - best['mean_omega_rmse_rad_s'] / baseline['mean_omega_rmse_rad_s']
    per_run_ratio = [a['omega']['rmse'] / b['omega']['rmse'] for a, b in zip(best['runs'], baseline['runs'])]
    result = {'status': 'training-only parameter sensitivity; not physical identification or acceptance',
              'protocol': 'v2_calibration_round1_protocol.md', 'training_directories': [str(p) for p in TRAINING],
              'baseline': baseline, 'best': best, 'all_candidates': rows,
              'training_objective_improvement_fraction': improvement,
              'training_omega_rmse_ratios_to_baseline': per_run_ratio,
              'meets_training_screen_only': improvement >= .10 and all(x <= 1.20 for x in per_run_ratio),
              'best_on_grid_boundary': best['tau_s'] in (min(TAUS), max(TAUS)) or best['delay_s'] in (min(DELAYS), max(DELAYS)),
              'source_sha256': {str(p / name): hashlib.sha256((p / name).read_bytes()).hexdigest()
                                for p in TRAINING for name in ('evaluation_truth.jsonl', 'evaluation_imu.jsonl', 'evaluation_pwm_commands.jsonl')},
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path(__file__), Path('foundation/quad.py'), Path('foundation/replay_closed_loop.py'), Path('foundation/compare.py'))},
              'limitations': ['posthoc initial velocity and assumed motor state', 'ROS stamps are not verified physical application times',
                              'wind and contact not excluded', 'grid and objective do not separate correlated physical parameters',
                              'screen thresholds are engineering choices, not task acceptance thresholds']}
    Path('foundation/reports/v2_calibration_round1.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('baseline', baseline['mean_omega_rmse_rad_s'], 'best', best['mean_omega_rmse_rad_s'],
          'tau', best['tau_s'], 'delay', best['delay_s'], 'improvement', improvement,
          'screen', result['meets_training_screen_only'], 'boundary', result['best_on_grid_boundary'])


if __name__ == '__main__':
    main()
