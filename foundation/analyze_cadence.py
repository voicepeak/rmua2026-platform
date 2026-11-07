"""Compare independently launched same-seed PWM cadence pilots.

Uses subscriber arrival monotonic time relative to first command, rather than
claiming physical clock synchronization. Pose is translated to its interpolated
value at first command. No orientation alignment, latency fitting or statistical
equivalence is claimed; a single trial per frequency cannot establish parity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def load(directory):
    command_path, capture_path = directory/'commands.jsonl', directory/'capture.jsonl'
    commands = [json.loads(s) for s in command_path.read_text().splitlines() if s.strip()]
    commands = [x for x in commands if x.get('kind') == 'command']
    pose = [json.loads(s) for s in capture_path.read_text().splitlines() if s.strip()]
    pose = [x for x in pose if x.get('topic') == 'pose_gt']
    if len(commands) < 3 or len(pose) < 3:
        raise ValueError('insufficient samples')
    t0 = commands[0]['monotonic_ns']
    times = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in pose])
    positions = np.array([x['position'] for x in pose], float)
    if np.any(np.diff(times) <= 0) or times[0] > 0 or times[-1] < 1.5:
        raise ValueError('pose timestamps do not cover the requested interval')
    initial = np.array([np.interp(0, times, positions[:, i]) for i in range(3)])
    ctime = np.array([x['monotonic_ns'] for x in commands], dtype=np.int64)
    intervals = np.diff(ctime)*1e-9
    if np.any(intervals <= 0):
        raise ValueError('nonmonotonic commands')
    edges = []
    for x in commands:
        if not edges or x['pwm'] != edges[-1]['pwm']:
            edges.append({'pwm': x['pwm'], 'offset_s': (x['monotonic_ns']-t0)*1e-9})
    return {'time': times, 'pos': positions-initial, 'edges': edges,
            'intervals': intervals, 'count': len(commands),
            'hashes': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (command_path, capture_path)}}


def compare(dirs):
    runs = {name: load(path) for name, path in dirs.items()}
    grid = np.arange(.05, 1.5, .01)
    def interpolate(run):
        return np.column_stack([np.interp(grid, run['time'], run['pos'][:, i]) for i in range(3)])
    reference = interpolate(runs['50'])
    result = {}
    for name, run in runs.items():
        error = np.linalg.norm(interpolate(run)-reference, axis=1)
        result[name] = {'commands': run['count'], 'interval_ms_p50': float(np.median(run['intervals'])*1000),
                        'interval_ms_p95': float(np.percentile(run['intervals'], 95)*1000),
                        'edges': run['edges'], 'position_difference_vs_50hz_rmse_m': float(np.sqrt(np.mean(error**2))),
                        'position_difference_vs_50hz_max_m': float(error.max()), 'source_sha256': run['hashes']}
    return {'window_s': [float(grid[0]), float(grid[-1])], 'reference': '50 Hz run', 'runs': result,
            'limitations': ['one independent process per target frequency; seed alone does not ensure identical state',
                            'pose arrival and post-publish times include transport and scheduling',
                            'different actual edge times are not corrected',
                            'no inference of motor or physics timestep',
                            'ground contact and unknown actuation time limit causal interpretation']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hz20', type=Path, required=True)
    p.add_argument('--hz50', type=Path, required=True)
    p.add_argument('--hz100', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = compare({'20': a.hz20, '50': a.hz50, '100': a.hz100})
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    for name, row in result['runs'].items():
        print(name, 'median ms', round(row['interval_ms_p50'], 2), 'max difference m', round(row['position_difference_vs_50hz_max_m'], 4))


if __name__ == '__main__':
    main()
