"""Compare read-only debugger root samples with direct ROS pose arrivals."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.analyze_frame_alignment import PATTERN, nearest


def analyze(directory, max_gap_s=.03):
    logs = list((directory/'pool').glob('*.log'))
    if len(logs) != 1:
        raise ValueError('expected one debugger log')
    capture = directory/'capture.jsonl'
    commands = directory/'commands.jsonl'
    rawlog, rawcap, rawcommands = logs[0].read_bytes(), capture.read_bytes(), commands.read_bytes()
    gdb = [{'mono': float(m[1]), 'actor': m[2], 'component': m[3],
            'xyz': np.array([float(v) for v in m[4].split(',')]),
            'quat_xyzw': np.array([float(v) for v in m[5].split(',')])}
           for m in PATTERN.finditer(rawlog.decode('utf-8', 'replace'))]
    pose = [json.loads(s) for s in rawcap.decode().splitlines() if s.strip()]
    pose = sorted([x for x in pose if x.get('topic') == 'pose_gt'], key=lambda x: x['arrival_monotonic_ns'])
    times = [x['arrival_monotonic_ns']*1e-9 for x in pose]
    pairs = []
    for item in gdb:
        index = nearest(times, item['mono'], max_gap_s)
        if index is None:
            continue
        p = pose[index]
        origin = item['xyz']-np.asarray(p['position'])*[100, 100, -100]
        pair = {'gdb_monotonic_s': item['mono'], 'arrival_gap_s': times[index]-item['mono'],
                      'actor': item['actor'], 'component': item['component'],
                      'actor_root_cm': item['xyz'].tolist(), 'pose_ned_m': p['position'],
                      'candidate_origin_cm': origin.tolist()}
        if 'quaternion_wxyz' in p:
            # World UE z-up and body UE z-up both become z-down: D R D.
            q = item['quat_xyzw'] * [-1, -1, 1, 1]
            pose_q = np.asarray(p['quaternion_wxyz'], float)[[1, 2, 3, 0]]
            if np.linalg.norm(q) < 1e-8 or np.linalg.norm(pose_q) < 1e-8:
                raise ValueError('invalid quaternion')
            pair['attitude_difference_deg'] = float(np.rad2deg((Rotation.from_quat(q).inv() * Rotation.from_quat(pose_q)).magnitude()))
        pairs.append(pair)
    if len(pairs) < 3:
        raise ValueError('insufficient paired samples')
    origins = np.array([x['candidate_origin_cm'] for x in pairs])
    med = np.median(origins, axis=0)
    residual = np.linalg.norm(origins-med, axis=1)
    events = [json.loads(s) for s in rawcommands.decode().splitlines() if s.strip()]
    end_events = [x for x in events if x.get('kind') == 'end']
    if len(end_events) != 1:
        raise ValueError('expected one player end event')
    end_s = end_events[0]['monotonic_ns']*1e-9
    before = origins[np.array([x['gdb_monotonic_s'] <= end_s for x in pairs])]
    if len(before) < 3:
        raise ValueError('insufficient samples before player exit')
    before_med = np.median(before, axis=0)
    angle = [x['attitude_difference_deg'] for x in pairs if x['gdb_monotonic_s'] <= end_s and 'attitude_difference_deg' in x]
    return {'samples': pairs, 'count': len(pairs), 'actor_count': len({x['actor'] for x in pairs}),
            'median_candidate_origin_cm': med.tolist(),
            'residual_cm_p50_p95_max': np.percentile(residual, [50, 95, 100]).tolist(),
            'player_end_monotonic_s': end_s, 'before_player_end_count': len(before),
            'before_player_end_origin_cm': before_med.tolist(),
            'before_player_end_residual_cm_p50_p95_max': np.percentile(np.linalg.norm(before-before_med, axis=1), [50,95,100]).tolist(),
            'before_player_end_attitude_deg_p50_p95_max': np.percentile(angle, [50,95,100]).tolist() if angle else None,
            'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (logs[0], capture, commands)},
            'limitations': ['debugger breakpoint is read-only but all-stop changes timing',
                            'actor type and pose reference-point equivalence are not established',
                            'ROS subscriber arrival is not measurement timestamp',
                            'before-player-end split is operational, not proof of absence of contact',
                            'ground-start and possible contacts are not independently ruled out',
                            'single run and no independent coordinate calibration']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.directory)
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('pairs', result['count'], 'actors', result['actor_count'], 'origin cm', result['median_candidate_origin_cm'],
          'residual cm p50/p95/max', result['residual_cm_p50_p95_max'])


if __name__ == '__main__':
    main()
