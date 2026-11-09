"""Analyze a diagnostic waypoint run through a candidate detection-gate center."""
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


def analyze(run, *, group, index, axis, half_extents_cm):
    run = Path(run)
    commands = [json.loads(line) for line in (run/'commands.jsonl').read_text(encoding='utf-8').splitlines() if line]
    start = next(item for item in commands if item.get('kind') == 'start')
    finish = next(item for item in commands if item.get('kind') == 'finish')
    poses = [json.loads(line) for line in (run/'capture.jsonl').read_text(encoding='utf-8').splitlines() if line]
    poses = [row for row in poses if row.get('topic') == 'pose_gt']
    gates = DetectionGateCandidates.load(GATE_MANIFEST, axis=axis, half_extents_cm=half_extents_cm)
    gate = next(record for name, records in gates.groups if name == group
                for record in records if record['index'] == index)
    target_ned = np.array([gate['position_ue_cm'][0], gate['position_ue_cm'][1],
                           -gate['position_ue_cm'][2]]) / 100
    closest = min(poses, key=lambda row: np.linalg.norm(np.asarray(row['position']) - target_ned))
    closest_distance = float(np.linalg.norm(np.asarray(closest['position']) - target_ned))
    crossing = None
    for begin, end in zip(poses, poses[1:]):
        hits = gates.crossings_ned(begin['position'], end['position'], origin_ue_cm=[0, 0, 0])
        match = [hit for hit in hits if hit['group'] == group and hit['index'] == index]
        if match:
            crossing = match[0]
            break
    log_paths = list((run/'pool').glob('*.log'))
    if len(log_paths) != 1:
        raise ValueError('expected one simulator log')
    log_text = log_paths[0].read_text(encoding='utf-8', errors='replace')
    contacts = [(float(value), line) for line, value in
                ((line, m.group(1)) for line in log_text.splitlines() for m in [CONTACT.search(line)] if m)]
    start_mono = start['monotonic_ns'] * 1e-9
    contacts_after_start = [row for row in contacts if row[0] >= start_mono]
    return {
        'status': 'diagnostic candidate detection-gate clearance; trigger and official scoring unverified',
        'group': group,
        'index': index,
        'gate_actor': gate['actor'],
        'gate_root': gate['root'],
        'gate_center_ned_m': target_ned.tolist(),
        'player_status': finish.get('status'),
        'pose_samples': len(poses),
        'closest_approach_m': closest_distance,
        'closest_pose': closest['position'],
        'crossing': crossing,
        'contact_count': len(contacts),
        'contacts_after_command_start': len(contacts_after_start),
        'contacts': [{'monotonic_s': time, 'line': line[:400]} for time, line in contacts],
        'source_sha256': {
            'run.json': digest(run/'run.json'),
            'commands.jsonl': digest(run/'commands.jsonl'),
            'capture.jsonl': digest(run/'capture.jsonl'),
            'simulator_log': digest(log_paths[0]),
        },
        'gate_manifest_sha256': gates.manifest_sha256,
        'limitations': [
            'velocity waypoint control uses debug pose truth and is diagnostic only',
            'GDB all-stop contact breakpoint changes timing and may change collision response',
            'gate half extents 2 m are an explicit probe assumption, not an official trigger volume',
            'a no-contact center crossing does not prove official gate trigger, aperture or scoring',
            'only one candidate gate was probed',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--group', default='road1')
    parser.add_argument('--index', type=int, default=32)
    parser.add_argument('--axis', type=int, default=0)
    parser.add_argument('--half-extent-cm', type=float, default=200.)
    args = parser.parse_args()
    result = analyze(args.run, group=args.group, index=args.index, axis=args.axis,
                     half_extents_cm=(args.half_extent_cm,)*3)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('closest', result['closest_approach_m'], 'crossing', bool(result['crossing']),
          'contacts after start', result['contacts_after_command_start'])


if __name__ == '__main__':
    main()
