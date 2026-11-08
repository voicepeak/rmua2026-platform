"""Audit sent PWM and ROS debug echoes without claiming physical latency."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np


def decode(row):
    if row.get('type') != 'airsim_ros/RotorPWM' or row.get('md5sum') != '4967432b608c06d70b17be20124fa258':
        raise ValueError('unexpected ROS message type or MD5')
    data = base64.b64decode(row['wire_base64'], validate=True)
    if len(data) < 48:
        raise ValueError('truncated ROS message')
    seq, secs, nsecs, size = struct.unpack_from('<IIII', data)
    offset = 16 + size
    if nsecs >= 1_000_000_000 or len(data) != offset + 32:
        raise ValueError('invalid header or PWM message length')
    frame = data[16:offset].decode('utf-8')
    pwm = struct.unpack_from('<4d', data, offset)
    if not all(math.isfinite(x) for x in pwm):
        raise ValueError('nonfinite debug PWM')
    return {'seq': seq, 'stamp_ns': secs * 1_000_000_000 + nsecs,
            'frame_id': frame, 'pwm': list(pwm),
            'arrival_monotonic_ns': row['arrival_monotonic_ns']}


def audit(commands, observations, tolerance=1e-4):
    if not commands or not observations:
        raise ValueError('missing commands or PWM observations')
    if tolerance <= 0 or not math.isfinite(tolerance):
        raise ValueError('invalid tolerance')
    command_times = [x['monotonic_ns'] for x in commands]
    arrival_times = [x['arrival_monotonic_ns'] for x in observations]
    if any(b <= a for a, b in zip(command_times, command_times[1:])) or any(b <= a for a, b in zip(arrival_times, arrival_times[1:])):
        raise ValueError('nonmonotonic host timestamps')
    edges = []
    for c in commands:
        if not edges or c['pwm'] != edges[-1]['pwm']:
            edges.append(c)
    out = []
    for i, edge in enumerate(edges):
        start = edge['monotonic_ns']
        end = edges[i + 1]['monotonic_ns'] if i + 1 < len(edges) else command_times[-1] + 1
        prior = next((x for x in reversed(observations) if x['arrival_monotonic_ns'] < start), None)
        same = lambda x: bool(np.all(np.abs(np.array(x['pwm']) - edge['pwm']) <= tolerance))
        interval = [x for x in observations if start <= x['arrival_monotonic_ns'] < end]
        first = next((x for x in interval if same(x)), None)
        prior_match = same(prior) if prior else None
        out.append({'edge_index': i, 'command_pwm': edge['pwm'], 'command_source_stamp_ns': edge['stamp_ns'],
                    'command_recorded_monotonic_ns': start, 'interval_sample_count': len(interval),
                    'prior_debug_pwm': prior['pwm'] if prior else None,
                    'prior_already_matches': prior_match,
                    'first_match_arrival_minus_record_ms': (first['arrival_monotonic_ns'] - start) / 1e6 if first else None,
                    'first_match_source_minus_command_header_ms': (first['stamp_ns'] - edge['stamp_ns']) / 1e6 if first else None,
                    'first_match_debug_stamp_ns': first['stamp_ns'] if first else None,
                    'interpretation': 'no match before next edge' if first is None else
                        'pre-edge value unavailable or already matching; edge correspondence ambiguous' if prior_match is not False else
                        'matching debug observation after nonmatching prior sample; physical application still unverified'})
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    paths = [args.directory / 'run.json', args.directory / 'commands.jsonl', args.directory / 'capture.jsonl']
    raw = [x.read_bytes() for x in paths]
    run = json.loads(raw[0])
    if run.get('exit_status') != 0:
        raise ValueError('remote run not confirmed successful')
    commands = [x for x in (json.loads(s) for s in raw[1].splitlines() if s) if x.get('kind') == 'command']
    observations = [decode(x) for x in (json.loads(s) for s in raw[2].splitlines() if s) if x.get('topic') == 'rotor_pwm']
    result = {'status': 'debug message audit; not motor or physics latency', 'mode': run.get('mode'),
              'edges': audit(commands, observations), 'debug_samples': len(observations),
              'source_sha256': {x.name: hashlib.sha256(b).hexdigest() for x, b in zip(paths, raw)},
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'limitations': ['command record is after publish; debug arrival includes ROS transport and scheduling',
                              'debug stamp is not independently verified as command application or simulation clock',
                              'matching values can recur; matching does not prove command causality',
                              'physical response onset and latency remain unidentified']}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('edges', len(result['edges']), 'matched', sum(x['first_match_debug_stamp_ns'] is not None for x in result['edges']))


if __name__ == '__main__':
    main()
