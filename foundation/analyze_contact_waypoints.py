#!/usr/bin/env python3
"""Summarize official contact events from a contact_waypoints run.

Groups repeated breakpoint hits by runtime source pointer and compares the
candidate hit point with the candidate map.  GDB all-stop and the diagnostic
pilot remain limitations; this is not a complete G2 pass.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

from foundation.analyze_contact_surface import analyze as analyze_contact_events
from foundation.analyze_contact_transform import EVENT
from foundation.map_scene import CandidateMap

DRONE = re.compile(r'\bdrone=(\S+)')


def summarize(run_dir, scene, plan):
    meta_raw = (run_dir / 'run.json').read_bytes()
    meta = json.loads(meta_raw)
    logs = list((run_dir / 'pool').glob('*.log'))
    if len(logs) != 1:
        raise ValueError('expected one simulator log')
    log_raw = logs[0].read_bytes()
    log_text = log_raw.decode('utf-8', 'replace')
    events = analyze_contact_events(scene, log_text)
    targets = {}
    for name, target in plan.get('targets', {}).items():
        if isinstance(target, dict):
            targets[(target.get('candidate_actor'), target.get('candidate_component'))] = name
    groups = {}
    for event in events:
        key = (event['candidate_actor'], event['candidate_component'],
               event['runtime_component_ptr'])
        group = groups.setdefault(key, {
            'candidate_actor': event['candidate_actor'],
            'candidate_component': event['candidate_component'],
            'candidate_mesh': event['candidate_mesh'],
            'runtime_component_ptr': event['runtime_component_ptr'],
            'event_count': 0,
            'first_monotonic_s': event['monotonic_s'],
            'last_monotonic_s': event['monotonic_s'],
            'min_surface_distance_cm': event['candidate_surface_distance_cm'],
            'max_surface_distance_cm': event['candidate_surface_distance_cm'],
            'max_source_transform_error_cm': event['source_transform_error_cm'],
            'min_source_transform_error_cm': event['source_transform_error_cm'],
            'max_rotation_error_deg': event['source_rotation_error_deg'],
            'second_nearest_transform_distance_cm': event['second_nearest_transform_distance_cm'],
            'sample_hit_point_cm': event['candidate_hit_point_cm'],
            'sample_hit_sha256': event['hit_bytes_sha256'],
            'drone_xyz_cm': None,
        })
        group['event_count'] += 1
        group['last_monotonic_s'] = max(group['last_monotonic_s'], event['monotonic_s'])
        group['min_surface_distance_cm'] = min(group['min_surface_distance_cm'],
                                               event['candidate_surface_distance_cm'])
        group['max_surface_distance_cm'] = max(group['max_surface_distance_cm'],
                                               event['candidate_surface_distance_cm'])
        group['max_source_transform_error_cm'] = max(group['max_source_transform_error_cm'],
                                                     event['source_transform_error_cm'])
        group['min_source_transform_error_cm'] = min(group['min_source_transform_error_cm'],
                                                     event['source_transform_error_cm'])
        group['max_rotation_error_deg'] = max(group['max_rotation_error_deg'],
                                              event['source_rotation_error_deg'])
    drone_positions = {}
    for match in EVENT.finditer(log_text):
        pointer = match[6]
        position = [float(x) for x in match[4].split(',')]
        drone_positions.setdefault(pointer, []).append(position)
    for group in groups.values():
        positions = drone_positions.get(group['runtime_component_ptr'])
        if positions:
            lows = [min(p[i] for p in positions) for i in range(3)]
            highs = [max(p[i] for p in positions) for i in range(3)]
            group['drone_xyz_cm'] = {'min': lows, 'max': highs, 'count': len(positions)}
    for group in groups.values():
        group['matched_target'] = targets.get((group['candidate_actor'],
                                               group['candidate_component']))
    summary = sorted(groups.values(), key=lambda item: -item['event_count'])
    return {
        'status': 'official contact sources grouped and compared to candidate geometry; not a complete G2 pass',
        'run': str(run_dir),
        'seed': meta.get('seed'),
        'binary_sha256': meta.get('binary_sha256'),
        'settings_sha256': meta.get('settings_sha256'),
        'player_exit_status': meta.get('player_exit_status'),
        'event_count': len(events),
        'source_group_count': len(summary),
        'groups': summary,
        'source_sha256': {
            'run': hashlib.sha256(meta_raw).hexdigest(),
            'log': hashlib.sha256(log_raw).hexdigest(),
            'waypoints': meta.get('waypoints_sha256'),
            'map': scene.source_sha256,
        },
        'limitations': [
            'GDB all-stop changes timing and may change collision response',
            'velocity pilot uses debug ground truth and is diagnostic only',
            'repeated callbacks during sustained contact are grouped, not independent contacts',
            'candidate hit point at byte offset 0x18 is not independently typed',
            'candidate maps and transforms remain candidate geometry',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--assets', type=Path, default=Path('foundation/asset_analysis'))
    args = parser.parse_args()
    meta = json.loads((args.run / 'run.json').read_text(encoding='utf-8'))
    plan = json.loads(Path(meta['waypoints']).read_text(encoding='utf-8'))
    scene = CandidateMap.load(args.assets, include_unverified_simple=True,
                              include_unverified_sprites=True)
    result = summarize(args.run, scene, plan)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    for group in result['groups']:
        print(group['event_count'], group['candidate_actor'], group['candidate_component'],
              'target=', group['matched_target'],
              'surface_cm=%.3g..%.3g' % (group['min_surface_distance_cm'],
                                          group['max_surface_distance_cm']))


if __name__ == '__main__':
    main()
