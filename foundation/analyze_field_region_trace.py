"""Compare natural predicate transitions with candidate road boxes, not a fitted field model."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.region_volumes import CandidateBoxVolumes


def analyze(events, volumes, origin, max_arrival_gap_s, extra_volumes=None):
    if not np.isfinite(max_arrival_gap_s) or max_arrival_gap_s <= 0:
        raise ValueError('invalid arrival gap limit')
    rows = []
    for event in events:
        gps = event.get('nearest_gps')
        if not gps or not gps.get('accepted') or gps.get('position') is None:
            continue
        gap = float(gps['arrival_monotonic_delta_s'])
        candidates = [x for x in volumes.query_ned(gps['position'], origin_ue_cm=origin)
                      if x['component'] == 'ChargeArea']
        if not candidates or not np.isfinite(gap):
            raise ValueError('missing road volume or GPS timestamp')
        candidate = min(candidates, key=lambda x: x['signed_distance_cm'])
        passing = {name: len(value['passing']) for name, value in event['lists'].items()}
        flag = int(event['predicate'])
        if flag not in (0, 1):
            raise ValueError('invalid observed predicate')
        extras = {}
        for name, box_set in (extra_volumes or {}).items():
            matches = box_set.query_ned(gps['position'], origin_ue_cm=origin)
            extras[name] = min(matches, key=lambda x: x['signed_distance_cm'])
        union_inside = candidate['contains'] or any(x['contains'] for x in extras.values())
        rows.append({'monotonic_s': event['monotonic_s'], 'observed_predicate': flag,
                     'gps_position_ned_m': gps['position'], 'gps_arrival_delta_s': gap,
                     'within_arrival_gap': abs(gap) <= max_arrival_gap_s,
                     'candidate_road_inside': candidate['contains'],
                     'candidate_road_signed_distance_cm': candidate['signed_distance_cm'],
                     'candidate_actor': candidate['actor'], 'other_regions': extras,
                     'candidate_union_inside': bool(union_inside), 'passing_objects': passing})
    if not rows:
        raise ValueError('no usable predicate snapshots')
    selected = [r for r in rows if r['within_arrival_gap']]
    confusion = Counter((r['observed_predicate'], int(r['candidate_road_inside'])) for r in selected)
    return rows, {f'observed_{k[0]}_road_inside_{k[1]}': v for k, v in sorted(confusion.items())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('summary', type=Path)
    p.add_argument('volumes', type=Path)
    p.add_argument('manifest', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--origin-ue-cm', nargs=3, type=float, required=True)
    p.add_argument('--max-arrival-gap-s', type=float, required=True)
    p.add_argument('--port-volumes', type=Path)
    p.add_argument('--factory-volumes', type=Path)
    a = p.parse_args()
    summary, manifest = a.summary.read_bytes(), a.manifest.read_bytes()
    if json.loads(manifest).get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary')
    volumes = CandidateBoxVolumes.load(a.volumes)
    extras = {name: CandidateBoxVolumes.load(path) for name, path in
              (('port', a.port_volumes), ('factory', a.factory_volumes)) if path is not None}
    rows, counts = analyze(json.loads(summary)['events'], volumes, a.origin_ue_cm, a.max_arrival_gap_s, extras)
    selected = [row for row in rows if row['within_arrival_gap']]
    union_counts = Counter((r['observed_predicate'], int(r['candidate_union_inside'])) for r in selected)
    result = {'status': 'exploratory comparison; road union is not the official predicate',
              'origin_ue_cm': a.origin_ue_cm, 'max_arrival_gap_s': a.max_arrival_gap_s,
              'counts': counts, 'union_counts': {f'observed_{k[0]}_union_inside_{k[1]}': v
                                                for k, v in sorted(union_counts.items())}, 'events': rows,
              'source_sha256': {'summary': hashlib.sha256(summary).hexdigest(),
                                'manifest': hashlib.sha256(manifest).hexdigest(),
                                'volumes': volumes.source_sha256,
                                'other_volumes': {name: value.source_sha256 for name, value in extras.items()}},
              'limitations': ['GPS arrival is not physical event time and candidate origin is unverified',
                              'point model excludes finite drone collision shape and overlap latency',
                              'union may omit other predicate branches, including weak reference and side routes',
                              'single raised-route run, all-stop GDB and only predicate changes sampled',
                              'matches do not establish natural field boundary or F01']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('events', len(rows), 'within gap', sum(r['within_arrival_gap'] for r in rows), 'counts', counts)


if __name__ == '__main__':
    main()
