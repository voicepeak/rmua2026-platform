"""Audit observed wind in the frozen round-2 fitting windows; no model replay."""
import argparse
import base64
import hashlib
import json
import math
import struct
from pathlib import Path


RUNS = ('d03_roll_pitch_seed2026_20261107', 'd03_roll_pitch_small_seed456_20261107',
        'd04_yaw_seed2026_20261107', 'd04_yaw_small_seed456_20261107')


def decode(row):
    if row.get('type') != 'geometry_msgs/Vector3Stamped' or row.get('md5sum') != '7b324c7325e683bf02a9b14b01090ec7':
        raise ValueError('unexpected wind type or MD5')
    wire = base64.b64decode(row['wire_base64'], validate=True)
    if len(wire) < 40:
        raise ValueError('short wind message')
    _, sec, nsec, length = struct.unpack_from('<IIII', wire)
    if nsec >= 1_000_000_000 or len(wire) != 16 + length + 24:
        raise ValueError('invalid wind message length or timestamp')
    vector = struct.unpack_from('<3d', wire, 16 + length)
    if not all(math.isfinite(x) for x in vector):
        raise ValueError('nonfinite wind vector')
    return sec * 1_000_000_000 + nsec, vector


def analyze(directory):
    paths = [directory/'commands.jsonl', directory/'capture.jsonl']
    raw = [p.read_bytes() for p in paths]
    commands = [json.loads(x) for x in raw[0].splitlines() if x]
    first = next(x for x in commands if x.get('kind') == 'command')
    start = first['stamp_ns']
    if start <= 0:
        raise ValueError('invalid first command stamp')
    data = [decode(row) for row in (json.loads(x) for x in raw[1].splitlines() if x)
            if row.get('topic') == 'wind']
    invalid_stamps = sum(stamp <= 0 for stamp, _ in data)
    selected = [(stamp, vector) for stamp, vector in data
                if 300_000_000 <= stamp - start <= 800_000_000]
    deltas = [(b[0] - a[0]) * 1e-9 for a, b in zip(selected, selected[1:])]
    norms = [math.sqrt(sum(x*x for x in vector)) for _, vector in selected]
    return {'directory': str(directory).replace('\\', '/'), 'window_s': [.3, .8],
            'first_command_header_ns': start, 'total_wind_samples': len(data),
            'invalid_or_zero_stamp_count': invalid_stamps, 'window_sample_count': len(selected),
            'first_sample_relative_s': (selected[0][0]-start)*1e-9 if selected else None,
            'last_sample_relative_s': (selected[-1][0]-start)*1e-9 if selected else None,
            'nonincreasing_stamp_pairs': sum(x <= 0 for x in deltas),
            'max_adjacent_gap_s': max(deltas) if deltas else None,
            'max_observed_vector_norm': max(norms) if norms else None,
            'max_abs_component': max((abs(x) for _, v in selected for x in v), default=None),
            'source_sha256': {p.name: hashlib.sha256(b).hexdigest() for p, b in zip(paths, raw)}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('--datasets', type=Path, default=Path('foundation/datasets'))
    a = p.parse_args()
    result = {'status': 'observed ROS debug wind only; not proof of physical zero wind',
              'runs': [analyze(a.datasets/name) for name in RUNS],
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'limitations': ['sampled debug messages do not establish continuous physical wind',
                              'command header and wind header are not verified physical timing',
                              'absence of observed wind does not establish absence of contact']}
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    for run in result['runs']:
        print(run['directory'], run['window_sample_count'], run['max_observed_vector_norm'])


if __name__ == '__main__':
    main()
