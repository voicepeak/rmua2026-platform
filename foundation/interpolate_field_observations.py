"""Posthoc GPS-arrival interpolation at predicate transitions; exploratory."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.region_volumes import CandidateRegionUnion


def analyze(events, gps, field, max_gap_s):
    usable = [x for x in gps if x.get('accepted') and x.get('position') is not None]
    times = np.asarray([x['arrival_monotonic_s'] for x in usable], float)
    positions = np.asarray([x['position'] for x in usable], float).reshape(-1, 3)
    if not np.isfinite(max_gap_s) or max_gap_s <= 0 or np.any(np.diff(times) <= 0):
        raise ValueError('invalid interpolation time base')
    if not np.isfinite(times).all() or not np.isfinite(positions).all():
        raise ValueError('nonfinite GPS')
    rows, counts = [], Counter()
    for event in events:
        t = float(event['monotonic_s'])
        index = int(np.searchsorted(times, t))
        row = {'monotonic_s': t, 'observed_predicate': int(event['predicate']), 'eligible': False}
        if index == 0 or index == len(times):
            row['reason'] = 'no accepted bracketing samples'
        else:
            gap = float(times[index]-times[index-1])
            row['bracketing_gap_s'] = gap
            if gap > max_gap_s:
                row['reason'] = 'accepted GPS gap exceeds limit'
            else:
                weight = (t-times[index-1])/gap
                position = positions[index-1]*(1-weight)+positions[index]*weight
                result = field.evaluate(position)
                row.update(eligible=True, position_ned_m=position.tolist(),
                           candidate_flag=result['flag'], matching_regions=result['matches'])
                counts[(row['observed_predicate'], row['candidate_flag'])] += 1
        rows.append(row)
    return rows, {f'observed_{k[0]}_candidate_{k[1]}': v for k, v in sorted(counts.items())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('assets', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--origin-ue-cm', type=float, nargs=3, required=True)
    p.add_argument('--max-gap-s', type=float, required=True)
    a = p.parse_args()
    events_path, gps_path = a.run/'predicate_summary.json', a.run/'pose_samples.jsonl'
    raw_events, raw_gps = events_path.read_bytes(), gps_path.read_bytes()
    sources = [a.assets/'OL1_road_box_volumes_candidate.json', a.assets/'OL1_port_volumes_candidate.json',
               a.assets/'OL1_factory_volumes_candidate.json']
    field = CandidateRegionUnion.from_candidates(*sources, origin_ue_cm=a.origin_ue_cm)
    rows, counts = analyze(json.loads(raw_events)['events'],
                           [json.loads(line) for line in raw_gps.splitlines() if line.strip()], field, a.max_gap_s)
    result = {'status': 'posthoc interpolation sensitivity, not acceptance', 'counts': counts, 'events': rows,
              'max_gap_s': a.max_gap_s, 'origin_ue_cm': a.origin_ue_cm,
              'source_sha256': {'events': hashlib.sha256(raw_events).hexdigest(),
                                'gps': hashlib.sha256(raw_gps).hexdigest(),
                                **{p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}},
              'limitations': ['arrival-time interpolation chosen after observing nearest-sample mismatches',
                              'GPS arrival times are not physical source time; interpolation is approximate',
                              'events without closely spaced accepted bracketing GPS remain unclassified',
                              'finite drone shape, overlap semantics and other field branches unverified']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('events', len(rows), 'eligible', sum(r['eligible'] for r in rows), 'counts', counts)


if __name__ == '__main__':
    main()
