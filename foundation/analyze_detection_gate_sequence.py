"""Analyze a diagnostic waypoint sequence through several candidate detection gates."""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np

from foundation.detection_gates import DetectionGateCandidates


GATE_MANIFEST = Path('foundation/maps/OL1/detection_gate_candidates.json')
CONTACT = re.compile(r'MAP_CONTACT mono=([0-9.]+)')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--group', default='road1')
    parser.add_argument('--indices', default='32,31,30,29')
    parser.add_argument('--axis', type=int, default=0)
    parser.add_argument('--half-extent-cm', type=float, default=200.)
    args = parser.parse_args()
    indices = [int(value) for value in args.indices.split(',') if value]
    commands = [json.loads(line) for line in (args.run/'commands.jsonl').read_text(encoding='utf-8').splitlines() if line]
    start = next(item for item in commands if item.get('kind') == 'start')
    finish = next(item for item in commands if item.get('kind') == 'finish')
    poses = [json.loads(line) for line in (args.run/'capture.jsonl').read_text(encoding='utf-8').splitlines() if line]
    poses = [row for row in poses if row.get('topic') == 'pose_gt']
    gates = DetectionGateCandidates.load(GATE_MANIFEST, axis=args.axis,
                                         half_extents_cm=(args.half_extent_cm,)*3)
    records = {record['index']: record for name, group in gates.groups if name == args.group
               for record in group}
    results = []
    for index in indices:
        gate = records[index]
        target = np.array([gate['position_ue_cm'][0], gate['position_ue_cm'][1],
                           -gate['position_ue_cm'][2]]) / 100
        closest_distance = min(float(np.linalg.norm(np.asarray(row['position']) - target))
                               for row in poses)
        crossing = None
        for begin, end in zip(poses, poses[1:]):
            hits = gates.crossings_ned(begin['position'], end['position'], origin_ue_cm=[0, 0, 0])
            match = [hit for hit in hits if hit['group'] == args.group and hit['index'] == index]
            if match:
                crossing = match[0]
                break
        results.append({'index': index, 'gate_actor': gate['actor'], 'gate_root': gate['root'],
                        'gate_center_ned_m': target.tolist(),
                        'closest_approach_m': closest_distance, 'crossing': crossing})
    log_paths = list((args.run/'pool').glob('*.log'))
    if len(log_paths) != 1:
        raise ValueError('expected one simulator log')
    log_text = log_paths[0].read_text(encoding='utf-8', errors='replace')
    contacts = [(float(value), line) for line, value in
                ((line, match.group(1)) for line in log_text.splitlines()
                 for match in [CONTACT.search(line)] if match)]
    start_mono = start['monotonic_ns'] * 1e-9
    contacts_after_start = [row for row in contacts if row[0] >= start_mono]
    result = {
        'status': 'diagnostic candidate detection-gate sequence clearance; trigger and official scoring unverified',
        'group': args.group,
        'indices': indices,
        'player_status': finish.get('status'),
        'pose_samples': len(poses),
        'start_pose': poses[0]['position'],
        'end_pose': poses[-1]['position'],
        'gates': results,
        'contact_count': len(contacts),
        'contacts_after_command_start': len(contacts_after_start),
        'contacts': [{'monotonic_s': time, 'line': line[:400]} for time, line in contacts],
        'source_sha256': {
            'run.json': digest(args.run/'run.json'),
            'commands.jsonl': digest(args.run/'commands.jsonl'),
            'capture.jsonl': digest(args.run/'capture.jsonl'),
            'simulator_log': digest(log_paths[0]),
        },
        'gate_manifest_sha256': gates.manifest_sha256,
        'limitations': [
            'velocity waypoint control uses debug pose truth and is diagnostic only',
            'GDB all-stop contact breakpoint changes timing and may change collision response',
            '2 m gate half extents are an explicit probe assumption, not an official trigger volume',
            'crossing a candidate center plane does not prove official gate trigger, aperture or scoring',
            'only early road1 gates were probed',
        ],
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('player', finish.get('status'), 'contacts after start', len(contacts_after_start))
    for gate in results:
        print(gate['index'], 'closest', gate['closest_approach_m'], 'crossing', bool(gate['crossing']))


if __name__ == '__main__':
    main()
