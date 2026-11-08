"""Compare predeclared no-debug axis repeats using the P02 time alignment."""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from foundation.analyze_p02_repeatability import read_run


GROUPS = {
    'roll_pitch': ['d03_roll_pitch_seed2026_20261107', 'v2_roll_pitch_seed314159_20261108',
                   'v2_roll_pitch_seed27182_20261108'],
    'yaw': ['d04_yaw_seed2026_20261107', 'clock_pair_yaw_seed314159_20261108',
            'v2_yaw_seed27182_20261108'],
}


def main():
    results = {}
    for mode, names in GROUPS.items():
        data = [read_run(Path('foundation/datasets') / name, mode) for name in names]
        if not all(x[0]['input_sequence'] == data[0][0]['input_sequence'] for x in data):
            raise ValueError(f'{mode}: PWM phase sequences differ')
        pairs = []
        for (i, a), (j, b) in itertools.combinations(enumerate(data), 2):
            pos = np.linalg.norm(a[1] - b[1], axis=1)
            angle = (a[2].inv() * b[2]).magnitude()
            pairs.append({'runs': [i, j], 'relative_position_max_m': float(pos.max()),
                          'relative_position_rmse_m': float(np.sqrt(np.mean(pos ** 2))),
                          'relative_attitude_max_deg': float(np.degrees(angle.max()))})
        results[mode] = {'runs': [x[0] for x in data], 'pairs': pairs}
    result = {'status': 'exploratory repeatability; not a held-out acceptance',
              'window_s_after_first_published_source_stamp': [.30, .80], 'grid_step_s': .01,
              'protocol': 'v2_axis_repeat_collection_protocol.md', 'groups': results,
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'shared_reader_sha256': hashlib.sha256(Path('foundation/analyze_p02_repeatability.py').read_bytes()).hexdigest()}
    Path('foundation/reports/v2_axis_repeatability.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for mode, group in results.items():
        print(mode, group['pairs'])


if __name__ == '__main__':
    main()
