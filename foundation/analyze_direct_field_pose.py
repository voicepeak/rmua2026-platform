"""Compare synchronous candidate UE component poses to candidate region boxes."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.region_volumes import CandidateBoxVolumes
from foundation.frames import ned_m_to_ue_cm


def analyze(events, groups, origin=None, margin_cm=0.):
    if not np.isfinite(margin_cm) or margin_cm < 0:
        raise ValueError('invalid candidate margin')
    rows, counts = [], Counter()
    for event in events:
        row = {'monotonic_s': event['monotonic_s'], 'observed_predicate': event['predicate'],
               'is_transition': event.get('is_transition'),
               'drone_actor': event.get('drone_actor'), 'drone_component': event.get('drone_component'),
               'drone_world_cm': event.get('drone_world_cm'), 'available': False,
               'passing_objects': {name: len(value['passing']) for name, value in event['lists'].items()}}
        if event.get('drone_world_cm') is None:
            row['reason'] = event.get('drone_read_error', 'component pointer unavailable')
        else:
            point = np.asarray(event['drone_world_cm'], float)
            if point.shape != (3,) or not np.isfinite(point).all():
                raise ValueError('invalid component location')
            nearest = {}
            for name, volumes in groups.items():
                candidates = volumes.query_ue(point)
                nearest[name] = min(candidates, key=lambda x: x['signed_distance_cm'])
            inside = any(x['contains'] for x in nearest.values())
            margin_inside = any(x['signed_distance_cm'] <= -margin_cm for x in nearest.values())
            row.update(available=True, candidate_inside=inside, candidate_margin_inside=margin_inside, region_minima=nearest)
            gps = event.get('nearest_gps')
            if origin is not None and gps and gps.get('accepted') and gps.get('position') is not None:
                gps_ue = ned_m_to_ue_cm(gps['position'], origin)
                row['gps_arrival_delta_s'] = gps['arrival_monotonic_delta_s']
                row['gps_vs_component_distance_cm'] = float(np.linalg.norm(point-gps_ue))
            counts[(int(event['predicate']), int(inside))] += 1
        rows.append(row)
    return rows, {f'observed_{a}_inside_{b}': n for (a, b), n in sorted(counts.items())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('assets', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--origin-ue-cm', type=float, nargs=3)
    p.add_argument('--margin-cm', type=float, default=0.)
    a = p.parse_args()
    summary_path, manifest_path = a.run/'predicate_summary.json', a.run/'manifest.json'
    summary, manifest = summary_path.read_bytes(), manifest_path.read_bytes()
    if json.loads(manifest).get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary')
    sources = {'road': a.assets/'OL1_road_box_volumes_candidate.json',
               'port': a.assets/'OL1_port_volumes_candidate.json',
               'factory': a.assets/'OL1_factory_volumes_candidate.json'}
    groups = {name: CandidateBoxVolumes.load(path) for name, path in sources.items()}
    groups['road'] = CandidateBoxVolumes([x for x in groups['road'].placements if x['component'] == 'ChargeArea'])
    rows, counts = analyze(json.loads(summary)['events'], groups, a.origin_ue_cm, a.margin_cm)
    margin_counts = Counter((int(row['observed_predicate']), int(row['candidate_margin_inside']))
                            for row in rows if row['available'])
    result = {'status': 'same-breakpoint component location comparison; official overlap unverified',
              'counts': counts, 'candidate_margin_cm': a.margin_cm,
              'margin_counts': {f'observed_{a}_inside_{b}': n for (a, b), n in sorted(margin_counts.items())},
              'gps_origin_ue_cm': a.origin_ue_cm, 'events': rows,
              'source_sha256': {'summary': hashlib.sha256(summary).hexdigest(),
                                'manifest': hashlib.sha256(manifest).hexdigest(),
                                **{name: hashlib.sha256((a.run/name).read_bytes()).hexdigest() for name in
                                   ('stage_trace_predicate.gdb', 'predicate_pose_probe.gdb', 'charge_state_probe.py')},
                                **{p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources.values()}},
              'limitations': ['component pointer identity inferred from separate probe, not independently decoded',
                              'component transform may lag physics position or differ from overlap primitive',
                              'candidate union omits other predicate branches',
                              'only predicate changes sampled; debugger alters scheduling',
                              'candidate boxes and channel settings not independently validated']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('events', len(rows), 'pose available', sum(r['available'] for r in rows), 'counts', counts)


if __name__ == '__main__':
    main()
