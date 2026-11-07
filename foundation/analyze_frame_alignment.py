"""Posthoc match GDB actor-root world positions to ROS pose via IMU stamps.

This estimates a *candidate* axis-aligned translation; the actor root may differ
from the AirSim reported pose point. The ROS pose header and IMU clock are
matched first, then the debugger monotonic time is nearest-neighbor matched.
Debugger stops, asynchronous callbacks, world-origin shifting and injection
can invalidate the fit. This is not independent runtime coordinate acceptance.
"""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import re
import numpy as np

PATTERN = re.compile(r'DRONE_WORLD mono=([\d.]+) actor=(0x[0-9a-f]+) component=(0x[0-9a-f]+) xyz_cm=([^\s]+) quat=([^\s]+)')


def nearest(values, target, limit):
    i = bisect.bisect_left(values, target)
    choices = [j for j in (i-1, i) if 0 <= j < len(values)]
    if not choices:
        return None
    j = min(choices, key=lambda x: abs(values[x]-target))
    return j if abs(values[j]-target) <= limit else None


def analyze(directory, stamp_gap_ns=1_000_000, monotonic_gap_s=.03):
    paths = {x: directory/x for x in ('gdb.log', 'sensor_clock.jsonl', 'pose_gt_posthoc.csv')}
    raw = paths['gdb.log'].read_bytes()
    gdb = [{'mono_s': float(m[1]), 'xyz_cm': [float(v) for v in m[4].split(',')], 'component': m[3]}
           for m in PATTERN.finditer(raw.decode('utf-8', 'replace'))]
    clocks = [json.loads(s) for s in paths['sensor_clock.jsonl'].read_text().splitlines() if s.strip()]
    clocks = sorted([x for x in clocks if x.get('topic') == 'imu'], key=lambda x: x['header_stamp_ns'])
    stamps = [x['header_stamp_ns'] for x in clocks]
    with paths['pose_gt_posthoc.csv'].open(newline='') as f:
        poses = list(csv.DictReader(f))
    records = []
    for x in poses:
        index = nearest(stamps, int(x['field.header.stamp']), stamp_gap_ns)
        if index is None:
            continue
        c = clocks[index]
        records.append({'mono_s': c['arrival_monotonic_ns']*1e-9,
                        'stamp_gap_ns': c['header_stamp_ns']-int(x['field.header.stamp']),
                        'xyz_ned_m': [float(x['field.pose.position.'+k]) for k in 'xyz']})
    records.sort(key=lambda x:x['mono_s'])
    times = [x['mono_s'] for x in records]
    pairs = []
    for sample in gdb:
        i = nearest(times, sample['mono_s'], monotonic_gap_s)
        if i is None:
            continue
        pose = records[i]
        ned_ue_axes = np.asarray(pose['xyz_ned_m'])*[100,100,-100]
        translation = np.asarray(sample['xyz_cm']) - ned_ue_axes
        pairs.append({'gdb_monotonic_s': sample['mono_s'], 'clock_gap_s': pose['mono_s']-sample['mono_s'],
                      'stamp_gap_ns': pose['stamp_gap_ns'], 'actor_root_cm': sample['xyz_cm'],
                      'pose_ned_m': pose['xyz_ned_m'], 'candidate_origin_cm': translation.tolist()})
    if not pairs:
        raise ValueError('no synchronized samples; refusing origin estimate')
    origins = np.array([x['candidate_origin_cm'] for x in pairs])
    median = np.median(origins, axis=0)
    return {'samples': pairs, 'count': len(pairs), 'median_candidate_origin_cm': median.tolist(),
            'residual_cm_p50_p95_max': np.percentile(np.linalg.norm(origins-median,axis=1), [50,95,100]).tolist(),
            'source_sha256': {n: hashlib.sha256(p.read_bytes()).hexdigest() for n,p in paths.items()},
            'limits': {'max_imu_pose_stamp_gap_ns': stamp_gap_ns, 'max_debugger_arrival_gap_s': monotonic_gap_s},
            'limitations': ['actor root and AirSim pose may be different physical points',
                            'IMU header matching and arrival times do not establish exact measurement simultaneity',
                            'GDB pauses and experiment interventions affect timing',
                            'one candidate coordinate mapping is not independently validated']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.directory)
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('pairs', result['count'], 'origin cm', result['median_candidate_origin_cm'],
          'residual p50/p95/max cm', result['residual_cm_p50_p95_max'])


if __name__ == '__main__':
    main()
