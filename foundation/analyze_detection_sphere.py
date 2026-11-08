"""Exploratory blueprint sphere/road overlap at field predicate snapshots."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.region_volumes import CandidateBoxVolumes
from foundation.road_scene import value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('assets', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--max-gap-s', type=float, required=True)
    a = p.parse_args()
    source_paths = [a.run/'predicate_summary.json', a.run/'pose_samples.jsonl',
                    a.assets/'blueprints/BP_FlyingPawn.json', a.assets/'OL1_road_box_volumes_candidate.json',
                    a.assets/'OL1_port_volumes_candidate.json', a.assets/'OL1_factory_volumes_candidate.json']
    raw = [p.read_bytes() for p in source_paths]
    events = json.loads(raw[0])['events']
    gps = [json.loads(x) for x in raw[1].splitlines() if x.strip()]
    gps = [x for x in gps if x.get('accepted') and x.get('orientation_xyzw') is not None]
    times = np.asarray([x['arrival_monotonic_s'] for x in gps], float)
    if len(times) < 1 or np.any(np.diff(times) <= 0) or not np.isfinite(a.max_gap_s) or a.max_gap_s <= 0:
        raise ValueError('invalid GPS orientation series or gap')
    bp = json.loads(raw[2]); entries = [e for e in bp['Exports'] if e.get('ObjectName') == 'JianCeQiu_GEN_VARIABLE']
    if len(entries) != 1:
        raise ValueError('missing sphere component')
    fields = {p['Name']: p.get('Value') for p in entries[0]['Data']}
    radius = float(fields['SphereRadius'])
    offset = value(fields, 'RelativeLocation', 'XYZ', [0, 0, 0])
    if radius <= 0 or not np.isfinite(radius):
        raise ValueError('invalid sphere radius')
    groups = [CandidateBoxVolumes.load(p) for p in source_paths[3:]]
    groups[0] = CandidateBoxVolumes([x for x in groups[0].placements if x['component'] == 'ChargeArea'])
    reflect = np.diag([1., 1., -1.]); rows=[]; counts=Counter()
    for event in events:
        row = {'monotonic_s': event['monotonic_s'], 'observed_predicate': event['predicate'], 'eligible': False}
        t = event['monotonic_s']; i = int(np.argmin(np.abs(times-t))); gap = float(times[i]-t)
        row['orientation_arrival_gap_s'] = gap
        if event.get('drone_world_cm') is None or abs(gap) > a.max_gap_s:
            row['reason'] = 'missing component position or orientation gap too large'
        else:
            ned_rotation = Rotation.from_quat(gps[i]['orientation_xyzw']).as_matrix()
            ue_rotation = reflect@ned_rotation@reflect
            center = np.asarray(event['drone_world_cm'], float)+ue_rotation@offset
            nearest = [min(g.query_ue(center), key=lambda x:x['signed_distance_cm']) for g in groups]
            distance = min(x['signed_distance_cm'] for x in nearest)
            prediction = int(distance <= radius)
            row.update(eligible=True, sphere_center_ue_cm=center.tolist(), signed_distance_cm=distance,
                       predicted_overlap=prediction, nearest_by_group=nearest)
            counts[(event['predicate'], prediction)] += 1
        rows.append(row)
    output = {'status':'exploratory same-run sphere hypothesis, not independent validation',
              'sphere_radius_cm':radius, 'sphere_offset_cm':offset.tolist(), 'max_gap_s':a.max_gap_s,
              'counts':{f'observed_{x}_predicted_{y}':n for (x,y),n in sorted(counts.items())}, 'events':rows,
              'source_sha256':{str(p):hashlib.sha256(b).hexdigest() for p,b in zip(source_paths,raw)},
              'limitations':['sphere identity and runtime use for this predicate not proven',
                             'cached component is assumed to be sphere parent and scale is assumed identity',
                             'ROS orientation paired by nearest arrival; UE/NED body reflection assumed',
                             'sampled only at predicate changes with debugger timing perturbation',
                             'hypothesis selected after inspecting these recordings; not held-out']}
    a.output.write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print('events',len(rows),'eligible',sum(r['eligible'] for r in rows),'counts',output['counts'])


if __name__=='__main__':
    main()
