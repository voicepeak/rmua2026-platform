"""Describe ROS capture clock metadata and GPS availability, not actuation delay."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.pwm_echo_audit import decode


def summarize(rows, topic, start_ns, window=(.30, .80)):
    selected = [x for x in rows if x.get('topic') == topic]
    prepared = []
    for row in selected:
        stamp = decode(row)['stamp_ns'] if topic == 'rotor_pwm' else row.get('stamp_ns', 0)
        prepared.append((row['arrival_monotonic_ns'], row['arrival_wall_ns'], stamp))
    prepared.sort(key=lambda x: x[0])
    nonzero = [x for x in prepared if x[2] > 0]
    source_nonincreasing = sum(b[2] <= a[2] for a, b in zip(nonzero, nonzero[1:]))
    lo, hi = window
    within = [x for x in nonzero if lo <= (x[0] - start_ns) * 1e-9 <= hi]
    ratio = None
    if len(within) >= 2 and within[-1][0] > within[0][0]:
        ratio = (within[-1][2] - within[0][2]) / (within[-1][0] - within[0][0])
    lag_ms = [(x[1] - x[2]) * 1e-6 for x in nonzero]
    return {'sample_count': len(prepared), 'nonzero_stamp_count': len(nonzero),
            'nonincreasing_source_stamp_pairs': source_nonincreasing,
            'source_stamp_first_ns': nonzero[0][2] if nonzero else None,
            'source_stamp_last_ns': nonzero[-1][2] if nonzero else None,
            'source_over_host_elapsed_ratio_window': ratio,
            'arrival_wall_minus_source_ms_median': float(np.median(lag_ms)) if lag_ms else None,
            'samples_in_window': len(within)}


def run(directory):
    paths = [directory / name for name in ('run.json', 'capture.jsonl', 'commands.jsonl', 'pool/pool.json')]
    raw = [p.read_bytes() for p in paths]
    manifest, pool = json.loads(raw[0]), json.loads(raw[3])
    if manifest.get('exit_status') != 0:
        raise ValueError('run did not complete successfully')
    rows = [json.loads(x) for x in raw[1].splitlines() if x]
    events = [json.loads(x) for x in raw[2].splitlines() if x]
    meta = [x for x in rows if x.get('topic') == 'collector_meta']
    starts = [x for x in events if x.get('kind') == 'start']
    if len(meta) != 1 or len(starts) != 1:
        raise ValueError('missing collector metadata or command start')
    meta = meta[0]
    topic_names = {x[0] for x in meta['published_topics_at_start']}
    summary = {topic: summarize(rows, topic, starts[0]['monotonic_ns'])
               for topic in ('pose_gt', 'imu', 'gps', 'rotor_pwm', 'clock')}
    return {'status': 'clock and message diagnostic; not actuation timing',
            'use_sim_time_at_collector_start': meta['use_sim_time'],
            'collector_ros_now_ns': meta['ros_now_ns'], 'collector_wall_ns': meta['arrival_wall_ns'],
            'clock_published_at_start': '/clock' in topic_names,
            'gps_published_at_start': '/airsim_node/drone_1/gps' in topic_names,
            'published_topics_at_start': meta['published_topics_at_start'],
            'window_host_s_from_player_start': [.30, .80], 'topics': summary,
            'settings_sha256': pool.get('settings_sha256'),
            'source_sha256': {str(p.relative_to(directory)).replace('\\', '/'): hashlib.sha256(b).hexdigest() for p, b in zip(paths, raw)},
            'limitations': ['ROS parameter and topic presence are point-in-time observations',
                            'header and wall-time differences include uncalibrated clock and transport offsets',
                            'ROS time is not verified physical simulation time or actuator application time',
                            'truth and debug topics are for offline research, not policy observations']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = run(args.directory)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('use_sim_time', result['use_sim_time_at_collector_start'],
          'clock published', result['clock_published_at_start'],
          'counts', {k: v['sample_count'] for k, v in result['topics'].items()})


if __name__ == '__main__':
    main()
