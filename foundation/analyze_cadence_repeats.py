"""Report predeclared within-frequency repeatability for no-debug PWM runs."""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from foundation.analyze_p02_repeatability import read_run


def main():
    root = Path('foundation/datasets')
    result = {}
    expected_sequence = None
    for mode in ('cadence20', 'cadence50', 'cadence100'):
        paths = [root / f'd06_{mode}_seed123_20261107',
                 root / f'v2_{mode}_seed456_20261109', root / f'v2_{mode}_seed2026_20261109']
        data = [read_run(p, mode, window=(.60, 1.00)) for p in paths]
        for row, _, _ in data:
            if expected_sequence is None:
                expected_sequence = row['input_sequence']
            elif row['input_sequence'] != expected_sequence:
                raise ValueError(f'{row["run"]}: PWM phase sequence differs')
        pairs = []
        for (i, a), (j, b) in itertools.combinations(enumerate(data), 2):
            distances = np.linalg.norm(a[1] - b[1], axis=1)
            angles = (a[2].inv() * b[2]).magnitude()
            pairs.append({'runs': [i, j], 'relative_position_max_m': float(distances.max()),
                          'relative_position_rmse_m': float(np.sqrt(np.mean(distances ** 2))),
                          'relative_attitude_max_deg': float(np.degrees(angles.max()))})
        result[mode] = {'runs': [row for row, _, _ in data], 'pairs': pairs}
    output = {'status': 'exploratory repeatability; not independent held-out acceptance',
              'protocol': 'v2_cadence_repeat_collection_protocol.md',
              'window_source_s': [.60, 1.00], 'grid_step_s': .01, 'groups': result,
              'limitations': ['No collision-free guarantee or independently calibrated actuation clock.',
                              'Different processes may have different initial velocity, wind and timing.',
                              'Historical waveforms previously used for exploration; no acceptance threshold frozen.'],
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'shared_reader_sha256': hashlib.sha256(Path('foundation/analyze_p02_repeatability.py').read_bytes()).hexdigest()}
    Path('foundation/reports/v2_cadence_repeatability.json').write_text(json.dumps(output, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for mode, group in result.items():
        print(mode, group['pairs'])


if __name__ == '__main__':
    main()
