"""Compare candidate runtime sphere centers to region volumes at predicate samples."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.region_volumes import CandidateBoxVolumes


def analyze(events, groups, radius_cm):
    if not np.isfinite(radius_cm) or radius_cm <= 0:
        raise ValueError('invalid sphere radius')
    rows, counts = [], Counter()
    for event in events:
        row = {'monotonic_s': event['monotonic_s'], 'observed_predicate': event['predicate'],
               'is_transition': event.get('is_transition'), 'available': False,
               'sphere_component':event.get('sphere_component')}
        if event.get('sphere_world_cm') is None or event.get('drone_world_cm') is None:
            row['reason'] = event.get('sphere_read_error', 'sphere or root position missing')
        else:
            center=np.asarray(event['sphere_world_cm'], float)
            root=np.asarray(event.get('drone_world_cm'), float)
            if center.shape != (3,) or root.shape != (3,) or not np.isfinite(np.r_[center, root]).all():
                raise ValueError('invalid runtime center')
            nearest = {name:min(group.query_ue(center), key=lambda x:x['signed_distance_cm'])
                       for name,group in groups.items()}
            distance = min(r['signed_distance_cm'] for r in nearest.values())
            predicted = int(distance <= radius_cm)
            row.update(available=True, sphere_center_cm=center.tolist(), root_position_cm=root.tolist(),
                       root_to_sphere_cm=float(np.linalg.norm(center-root)), signed_distance_cm=distance,
                       predicted_overlap=predicted, nearest_by_group=nearest)
            counts[(event.get('is_transition'),int(event['predicate']),predicted)] += 1
        rows.append(row)
    return rows, [{'transition':k[0], 'observed':k[1], 'predicted':k[2], 'count':n}
                  for k,n in sorted(counts.items(),key=lambda x:(str(x[0][0]),x[0][1],x[0][2]))]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('assets',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--radius-cm',type=float,required=True);a=p.parse_args()
    summary, manifest = (a.run/'predicate_summary.json').read_bytes(), (a.run/'manifest.json').read_bytes()
    if json.loads(manifest).get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary')
    paths={'road':a.assets/'OL1_road_box_volumes_candidate.json', 'port':a.assets/'OL1_port_volumes_candidate.json',
           'factory':a.assets/'OL1_factory_volumes_candidate.json'}
    groups={k:CandidateBoxVolumes.load(v) for k,v in paths.items()}
    groups['road']=CandidateBoxVolumes([x for x in groups['road'].placements if x['component']=='ChargeArea'])
    rows, counts = analyze(json.loads(summary)['events'], groups, a.radius_cm)
    result={'status':'candidate direct sphere overlap, not F01 acceptance', 'radius_cm':a.radius_cm,
            'counts':counts,'events':rows, 'source_sha256':{'summary':hashlib.sha256(summary).hexdigest(),
            'manifest':hashlib.sha256(manifest).hexdigest(), **{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths.values()},
            **{n:hashlib.sha256((a.run/n).read_bytes()).hexdigest() for n in ('stage_trace_predicate.gdb','predicate_pose_probe.gdb','charge_state_probe.py')}},
            'limitations':['actor offset and 2 cm radius attributed from Blueprint/startup correspondence, not independently typed',
                           'component may not be the actual overlap participant', 'sampling correlated and all-stop GDB perturbs timing',
                           'other field predicate branches and natural map coverage remain incomplete']}
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('events',len(rows),'available',sum(r['available'] for r in rows),'counts',counts)


if __name__=='__main__':main()
