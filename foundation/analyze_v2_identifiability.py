"""Summarize the already-collected V2 calibration grids without refitting.

This report records the near-optimal set, boundary locations and metric ranges
of the two allowed model-calibration rounds. It does not read the sealed
holdout, run a model, or establish a physical parameter confidence interval.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path('foundation/reports')
SOURCES = ('v2_calibration_round1.json', 'v2_calibration_round2.json')
OUTPUT = ROOT / 'v2_identifiability.json'


def objective(row):
    return row.get('normalized_objective', row.get('mean_omega_rmse_rad_s'))


def summarize(source):
    data = json.loads((ROOT / source).read_text(encoding='utf-8'))
    rows = data['all_candidates']
    best = min(rows, key=objective)
    base = data['baseline']
    best_value = objective(best)
    base_value = objective(base)
    near = [row for row in rows if objective(row) <= best_value * 1.05 + 1e-15]
    taus = sorted({row['tau_s'] for row in rows})
    delays = sorted({row['delay_s'] for row in rows})

    def ranges(metric):
        values = [run[metric]['rmse'] for run in best['runs']]
        baseline_values = [run[metric]['rmse'] for run in base['runs']]
        return {'best_min': min(values), 'best_max': max(values),
                'baseline_min': min(baseline_values), 'baseline_max': max(baseline_values)}

    def delay_penalty(tau):
        ordered = sorted((row for row in rows if row['tau_s'] == tau), key=lambda row: row['delay_s'])
        return [{'delay_s': row['delay_s'], 'objective': objective(row)}
                for row in ordered]

    def tau_penalty(delay):
        ordered = sorted((row for row in rows if row['delay_s'] == delay), key=lambda row: row['tau_s'])
        return [{'tau_s': row['tau_s'], 'objective': objective(row)} for row in ordered]

    return {
        'source': source,
        'source_sha256': hashlib.sha256((ROOT / source).read_bytes()).hexdigest(),
        'model_family': 'first-order motor filter plus one nonnegative uniform command delay',
        'grid': {'tau_s': taus, 'delay_s': delays},
        'best': {'tau_s': best['tau_s'], 'delay_s': best['delay_s'],
                 'objective': best_value, 'boundary': bool(data.get('best_on_grid_boundary'))},
        'baseline': {'tau_s': base['tau_s'], 'delay_s': base['delay_s'],
                     'objective': base_value},
        'improvement_fraction': 1 - best_value / base_value,
        'near_optimal_within_5_percent': {
            'count': len(near),
            'tau_s': sorted({row['tau_s'] for row in near}),
            'delay_s': sorted({row['delay_s'] for row in near}),
            'rows': [{'tau_s': row['tau_s'], 'delay_s': row['delay_s'], 'objective': objective(row)}
                     for row in near],
        },
        'best_per_run_omega_ratio': best.get('omega_rmse_ratios_to_baseline'),
        'metric_ranges_at_best_and_baseline': {
            'position_rmse_m': ranges('position'),
            'attitude_rmse_deg': ranges('attitude_deg'),
            'omega_rmse_rad_s': ranges('omega'),
        },
        'delay_penalty_by_tau': {str(tau): delay_penalty(tau) for tau in taus},
        'tau_penalty_at_delay_zero': tau_penalty(0.0),
    }


def main():
    result = {
        'status': 'posthoc identifiability summary of existing calibration grids; not a new fit',
        'rounds': [summarize(source) for source in SOURCES],
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'no new data or model replay; the source grids already used the allowed two rounds',
            'the sealed v2_holdout_axis records were not read',
            'near-optimal grid rows are not statistical confidence intervals',
            'the ROS command header is not the physical command application time',
            'wind, contact and independently measured initial velocity remain unexcluded',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for item in result['rounds']:
        print(item['source'], 'best', item['best'], 'near', item['near_optimal_within_5_percent'])


if __name__ == '__main__':
    main()
