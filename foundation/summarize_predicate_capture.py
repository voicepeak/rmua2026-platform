"""Pair observed predicate changes with nearest GPS callback arrivals."""
import argparse
import bisect
import hashlib
import json
from pathlib import Path


def summarize(log, gps):
    times = [x['arrival_monotonic_s'] for x in gps]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError('GPS arrivals are not monotonic')
    events = []
    for line in log.splitlines():
        if 'RMUA_PREDICATE_CHANGE ' not in line:
            continue
        event = json.loads(line.split('RMUA_PREDICATE_CHANGE ', 1)[1])
        idx = bisect.bisect_left(times, event['monotonic_s'])
        choices = [j for j in (idx-1, idx) if 0 <= j < len(times)]
        if choices:
            j = min(choices, key=lambda k: abs(times[k]-event['monotonic_s']))
            event['nearest_gps'] = {'arrival_monotonic_delta_s': times[j]-event['monotonic_s'],
                                    'gps_stamp_ros_s': gps[j].get('gps_stamp_ros_s'),
                                    'position': gps[j].get('position'), 'accepted': gps[j].get('accepted', False)}
        else:
            event['nearest_gps'] = None
        events.append(event)
    return events


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    log_path, gps_path = a.run/'gdb.log', a.run/'pose_samples.jsonl'
    log, gps_bytes = log_path.read_bytes(), gps_path.read_bytes()
    gps = [json.loads(line) for line in gps_bytes.splitlines() if line.strip()]
    events = summarize(log.decode('utf-8', 'replace'), gps)
    result = {'events': events, 'source_sha256': {'log': hashlib.sha256(log).hexdigest(),
                                                'gps': hashlib.sha256(gps_bytes).hexdigest()},
              'limitations': ['nearest callback arrival is not synchronized physical pose',
                              'only predicate changes logged; debugger perturbs timing']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('predicate observations', len(events), 'GPS records', len(gps))


if __name__ == '__main__':
    main()
