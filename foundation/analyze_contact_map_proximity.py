"""Compare event-time component origins to candidate-map surface proximity.

The recorded origin is not the hit point, source identity is not decoded, and
GDB changes timing. This cannot establish official collision alignment.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from foundation.map_scene import CandidateMap

EVENT = re.compile(r'HIT_SOURCE mono=([\d.]+) drone=(0x[0-9a-f]+) '
                   r'drone_component=(0x[0-9a-f]+) drone_world_cm=([^\s]+) '
                   r'source_actor=(0x[0-9a-f]+) source_component=(0x[0-9a-f]+)')


def analyze(scene, log, maximum_cm=100., resolution_cm=.25):
    raw = log.read_bytes()
    records = []
    cache = {}
    for match in EVENT.finditer(raw.decode('utf-8', errors='replace')):
        point = tuple(float(x) for x in match[4].split(','))
        if len(point) != 3 or not np.isfinite(point).all():
            raise ValueError('invalid event origin')
        if point not in cache:
            top = scene.sweep_sphere_ue(point, point, maximum_cm)
            if not top['hits']:
                cache[point] = {'distance_upper_bound_cm': None, 'hits': []}
            else:
                lo, hi = 0., maximum_cm
                while hi-lo > resolution_cm:
                    mid = (lo+hi)/2
                    if scene.sweep_sphere_ue(point, point, mid)['hits']:
                        hi = mid
                    else:
                        lo = mid
                near = scene.sweep_sphere_ue(point, point, hi)
                cache[point] = {'distance_lower_bound_cm': lo, 'distance_upper_bound_cm': hi,
                                'hits': near['hits']}
        records.append({'monotonic_s': float(match[1]), 'drone': match[2],
                        'drone_component': match[3], 'origin_ue_cm': list(point),
                        'source_actor_ptr': match[5], 'source_component_ptr': match[6],
                        'candidate_surface': cache[point]})
    if not records:
        raise ValueError('no hit-source events')
    return {'source_sha256': hashlib.sha256(raw).hexdigest(), 'count': len(records),
            'unique_origin_count': len(cache), 'maximum_cm': maximum_cm,
            'resolution_cm': resolution_cm, 'events': records,
            'limitations': ['drone component world origin is not the contact point or collision shape',
                            'source component pointer is not mapped to candidate asset identity',
                            'dynamic/fixture contact may cause the event; not necessarily a map surface',
                            'injected stage and forced vehicle path plus all-stop debugger affect behavior',
                            'candidate mesh modes, placement and channel are unverified',
                            'surface distance is not signed and does not check containment']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--assets', type=Path, default=Path('foundation/asset_analysis'))
    args = parser.parse_args()
    scene = CandidateMap.load(args.assets, include_unverified_simple=True)
    result = analyze(scene, args.log)
    result['source_map_sha256'] = scene.source_sha256
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('events:', result['count'], 'unique origins:', result['unique_origin_count'])
    print('distance upper bounds cm:', list(dict.fromkeys(e['candidate_surface']['distance_upper_bound_cm'] for e in result['events'])))


if __name__ == '__main__':
    main()
