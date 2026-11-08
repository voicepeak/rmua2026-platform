"""Fixed-window, descriptive comparison of three historical P02 recordings."""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


RUNS = [
    Path('foundation/datasets/d02_small_steps_seed123_20261107'),
    Path('foundation/datasets/d02_small_steps_seed456_20261107'),
    Path('foundation/datasets/clock_pair_steps_retry_seed27182_20261108'),
]
OUTPUT = Path('foundation/reports/v2_p02_repeatability.json')


def read_run(directory, expected_mode='small_steps', window=(.30, .80)):
    paths = [directory / 'run.json', directory / 'commands.jsonl',
             directory / 'capture.jsonl', directory / 'pool/pool.json']
    contents = [p.read_bytes() for p in paths]
    run, pool = json.loads(contents[0]), json.loads(contents[3])
    if run.get('exit_status') != 0 or run.get('mode') != expected_mode:
        raise ValueError(f'{directory}: invalid run status or mode')
    if list(directory.glob('*.gdb')):
        raise ValueError(f'{directory}: debugger script present; investigate')
    events = [json.loads(line) for line in contents[1].splitlines() if line]
    commands = [x for x in events if x.get('kind') == 'command']
    if not commands or not any(x.get('kind') == 'end' and x.get('status') == 'completed' for x in events):
        raise ValueError(f'{directory}: missing complete command stream')
    sequence = []
    transition_ns = []
    for command in commands:
        if not sequence or command['pwm'] != sequence[-1]:
            sequence.append(command['pwm'])
            transition_ns.append(command['stamp_ns'])
    epoch = commands[0]['stamp_ns']
    command_t = np.array([(x['stamp_ns'] - epoch) * 1e-9 for x in commands])
    poses = [x for x in (json.loads(line) for line in contents[2].splitlines() if line)
             if x.get('topic') == 'pose_gt' and x.get('stamp_ns')]
    t = np.array([(x['stamp_ns'] - epoch) * 1e-9 for x in poses])
    if len(t) < 5 or np.any(np.diff(t) <= 0) or np.any(np.diff(command_t) <= 0):
        raise ValueError(f'{directory}: invalid timestamps')
    start, end = window
    if end <= start or not np.isclose((end - start) / .01, round((end - start) / .01)):
        raise ValueError('window must be a positive multiple of 10 ms')
    grid = np.linspace(start, end, round((end - start) / .01) + 1)
    if t[0] > grid[0] or t[-1] < grid[-1] or command_t[-1] < grid[-1]:
        raise ValueError(f'{directory}: window not covered')
    used = (t >= grid[0] - .05) & (t <= grid[-1] + .05)
    if np.max(np.diff(t[used])) > .05:
        raise ValueError(f'{directory}: pose gap exceeds 50 ms')
    position = np.asarray([x['position'] for x in poses])
    quaternion = np.asarray([x['quaternion_wxyz'] for x in poses])
    xyz = np.column_stack([np.interp(grid, t, position[:, i]) for i in range(3)])
    rotations = Slerp(t, Rotation.from_quat(quaternion[:, [1, 2, 3, 0]]))(grid)
    relative_rotation = rotations[0].inv() * rotations
    row = {'run': str(directory), 'seed_requested': run.get('seed_requested'),
           'settings_sha256': pool.get('settings_sha256'), 'input_sequence': sequence,
           'initial_position_m': xyz[0].tolist(),
           'initial_quaternion_wxyz': rotations[0].as_quat()[[3, 0, 1, 2]].tolist(),
           'phase_start_s': [(stamp - epoch) * 1e-9 for stamp in transition_ns],
           'relative_position_min_m': (xyz - xyz[0]).min(axis=0).tolist(),
           'relative_position_max_m': (xyz - xyz[0]).max(axis=0).tolist(),
           'relative_attitude_change_max_deg': float(np.degrees(relative_rotation.magnitude().max())),
           'max_pose_interval_s': float(np.max(np.diff(t[used]))),
           'median_command_interval_s': float(np.median(np.diff(command_t))),
           'source_sha256': {str(p.relative_to(directory)): hashlib.sha256(b).hexdigest()
                             for p, b in zip(paths, contents)}}
    return row, xyz - xyz[0], relative_rotation


def main():
    data = [read_run(p) for p in RUNS]
    if not all(row[0]['input_sequence'] == data[0][0]['input_sequence'] for row in data):
        raise ValueError('actual PWM phase sequences differ')
    pairs = []
    for (i, a), (j, b) in itertools.combinations(enumerate(data), 2):
        pos = np.linalg.norm(a[1] - b[1], axis=1)
        angle = (a[2].inv() * b[2]).magnitude()
        pairs.append({'runs': [i, j], 'relative_position_max_m': float(pos.max()),
                      'relative_position_rmse_m': float(np.sqrt(np.mean(pos ** 2))),
                      'relative_attitude_max_deg': float(np.degrees(angle.max()))})
    output = {'status': 'exploratory descriptive repeatability; not a held-out acceptance',
              'window_s_after_first_published_source_stamp': [.30, .80],
              'grid_step_s': .01, 'runs': [x[0] for x in data], 'pairs': pairs,
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'protocol': 'v2_p02_repeatability_protocol.md'}
    OUTPUT.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(pairs, indent=2))


if __name__ == '__main__':
    main()
