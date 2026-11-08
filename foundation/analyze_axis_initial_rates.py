"""Describe IMU initial and in-window angular-rate differences for G1 repeats."""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from foundation.analyze_axis_repeatability import GROUPS
from foundation.analyze_p02_repeatability import read_run


def load_imu(directory):
    commands = [json.loads(s) for s in (directory / 'commands.jsonl').read_text().splitlines() if s]
    epoch = next(x['stamp_ns'] for x in commands if x.get('kind') == 'command')
    rows = [x for x in (json.loads(s) for s in (directory / 'capture.jsonl').read_text().splitlines() if s)
            if x.get('topic') == 'imu' and x.get('stamp_ns')]
    t = np.array([(x['stamp_ns'] - epoch) * 1e-9 for x in rows])
    omega = np.array([x['omega_body'] for x in rows])
    grid = np.linspace(.30, .80, 51)
    if len(t) < 2 or np.any(np.diff(t) <= 0) or t[0] > grid[0] or t[-1] < grid[-1]:
        raise ValueError(f'{directory}: insufficient monotonic IMU coverage')
    used = (t >= .25) & (t <= .85)
    if np.max(np.diff(t[used])) > .05:
        raise ValueError(f'{directory}: IMU gap exceeds 50 ms')
    return np.column_stack([np.interp(grid, t, omega[:, j]) for j in range(3)]), float(np.max(np.diff(t[used])))


def main():
    out = {}
    for mode, names in GROUPS.items():
        dirs = [Path('foundation/datasets') / name for name in names]
        pose = [read_run(p, mode) for p in dirs]
        imu = [load_imu(p) for p in dirs]
        runs = [{'name': name, 'omega_initial_rad_s': x[0][0].tolist(),
                 'imu_max_gap_s': x[1]} for name, x in zip(names, imu)]
        pairs = []
        for i, j in itertools.combinations(range(3), 2):
            delta = imu[i][0] - imu[j][0]
            angle = (pose[i][2].inv() * pose[j][2]).magnitude()
            pairs.append({'runs': [i, j], 'initial_omega_difference_rad_s': delta[0].tolist(),
                          'initial_omega_difference_norm_rad_s': float(np.linalg.norm(delta[0])),
                          'constant_initial_rate_scale_deg_over_0_5_s': float(np.degrees(np.linalg.norm(delta[0]) * .5)),
                          'omega_difference_rmse_rad_s': float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))),
                          'relative_attitude_difference_end_deg': float(np.degrees(angle[-1])),
                          'relative_attitude_difference_max_deg': float(np.degrees(angle.max()))})
        out[mode] = {'runs': runs, 'pairs': pairs}
    result = {'status': 'exploratory diagnostic; correlations do not establish cause',
              'window_source_s': [.30, .80], 'grid_step_s': .01, 'groups': out,
              'limitations': ['IMU bias, sensor delay, clock alignment and contact remain unverified',
                              'rate scale is a small-angle magnitude estimate, not an orientation correction or error decomposition',
                              'source files and command phase timing are recorded in v2_axis_repeatability.json'],
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    Path('foundation/reports/v2_axis_initial_rates.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for mode, group in out.items():
        print(mode, group['pairs'])


if __name__ == '__main__':
    main()
