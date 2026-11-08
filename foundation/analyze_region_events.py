"""Compare unsynchronized segment events to candidate road-box point containment."""
import argparse
from collections import defaultdict
import hashlib
import json
import re
from pathlib import Path

import numpy as np

from foundation.region_volumes import CandidateBoxVolumes


def analyze(events, volumes, origin, segment_actors=None):
    rows, counts = [], defaultdict(lambda: {'events': 0, 'inside': defaultdict(int)})
    for event in events:
        gps = event.get('nearest_gps')
        if not gps or not gps.get('accepted'):
            continue
        results = volumes.query_ned(gps['position'], origin_ue_cm=origin)
        kinds = sorted(set(r['component'] for r in results))
        nearest = {name: min((r for r in results if r['component'] == name),
                             key=lambda r: r['signed_distance_cm']) for name in kinds}
        key = f"enter={event['enter_flag']},event={event['event_flag']}"
        counts[key]['events'] += 1
        for name in kinds:
            counts[key]['inside'][name] += int(nearest[name]['contains'])
        row = {'event_kind': key, 'segment_index': event['segment_index'],
                     'spline_pointer': event['spline'], 'gps_position_ned_m': gps['position'],
                     'gps_arrival_delta_s': gps['arrival_delta_s'], 'minimum_signed_distance_by_component': nearest}
        if segment_actors is not None:
            actor = segment_actors[event['segment_index']]
            matches = [r for r in results if r['actor'] == actor]
            matched = {r['component']: r for r in matches}
            if len(matches) != len(kinds) or set(matched) != set(kinds):
                raise ValueError('missing or duplicated segment volume')
            row['hypothesized_segment_actor'] = actor
            row['hypothesized_segment_volumes'] = matched
        rows.append(row)
    if not rows:
        raise ValueError('no accepted GPS/event pairs')
    return rows, {key: {'events': value['events'], 'inside': dict(value['inside'])}
                  for key, value in counts.items()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('summary', type=Path)
    p.add_argument('volumes', type=Path)
    p.add_argument('manifest', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--origin-ue-cm', type=float, nargs=3, required=True)
    p.add_argument('--inventory', type=Path, help='independent runtime spline inventory for unique FName/root matching')
    p.add_argument('--hypothesize-reverse-index', action='store_true',
                   help='exploratory mapping: descending event index to ascending actor numeric suffix')
    a = p.parse_args()
    summary, manifest = a.summary.read_bytes(), a.manifest.read_bytes()
    meta = json.loads(manifest)
    if meta.get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary hash')
    volumes = CandidateBoxVolumes.load(a.volumes)
    source_events = json.loads(summary)['events']
    attribution = None
    inventory_hash = None
    if a.inventory is not None:
        inventory_raw = a.inventory.read_bytes()
        inventory_hash = hashlib.sha256(inventory_raw).hexdigest()
        entries = json.loads(inventory_raw)['inventory']['array30']
        fname_pairs = {(e['fname_index'], e['fname_number']) for e in source_events}
        if len(fname_pairs) != 1:
            raise ValueError('trace contains more than one spline FName')
        fname = next(iter(fname_pairs))
        matches = [x for x in entries if tuple(x['object_fname']) == fname]
        if len(matches) != 1 or len(matches[0]['within_0_1cm']) != 1:
            raise ValueError('runtime FName/root attribution ambiguous')
        spline_name = matches[0]['within_0_1cm'][0]['name']
        selected = [x for x in volumes.placements if any(
            owner['class'] == 'BP_Spline_C' and owner['actor'] == spline_name
            for owner in x.get('attachment_owners', []))]
        volumes = CandidateBoxVolumes(selected)
        attribution = {'runtime_fname': list(fname), 'candidate_spline': spline_name,
                       'selected_boxes': len(selected), 'evidence': 'separate run FName and unique root-position match'}
    segment_actors = None
    if a.hypothesize_reverse_index:
        if attribution is None:
            raise ValueError('spline inventory required for segment hypothesis')
        actors = {x['actor'] for x in volumes.placements}
        numbered = []
        for actor in actors:
            match = re.search(r'(\d+)(?:_Arrow(?:In|Out))?$', actor)
            if match is None:
                raise ValueError('actor has no numeric suffix')
            numbered.append((int(match.group(1)), actor))
        numbered.sort()
        indices = sorted({e['segment_index'] for e in source_events})
        if (len({n for n, _ in numbered}) != len(numbered)
                or any(i < 0 or i >= len(numbered) for i in indices)
                or any(b[0] != a[0]+1 for a, b in zip(numbered, numbered[1:]))):
            raise ValueError('segment and numeric actor sequences are not one-to-one consecutive')
        segment_actors = {index: numbered[len(numbered)-1-index][1] for index in indices}
    rows, counts = analyze(source_events, volumes, a.origin_ue_cm, segment_actors)
    segment_summary = None
    if segment_actors is not None:
        segment_summary = {}
        for key in counts:
            group = [row for row in rows if row['event_kind'] == key]
            by_component = {}
            for component in group[0]['hypothesized_segment_volumes']:
                distance = np.array([row['hypothesized_segment_volumes'][component]['signed_distance_cm'] / 100
                                     for row in group])
                by_component[component] = {'inside': int(np.sum(distance <= 0)),
                                           'signed_distance_median_m': float(np.median(distance)),
                                           'absolute_distance_p95_m': float(np.percentile(np.abs(distance), 95)),
                                           'absolute_distance_max_m': float(np.max(np.abs(distance)))}
            segment_summary[key] = by_component
    result = {'status': 'exploratory point containment, not official field predicate',
              'origin_ue_cm': a.origin_ue_cm, 'spline_attribution': attribution,
              'event_count': len(rows), 'counts': counts,
              'hypothesized_reverse_index_summary': segment_summary, 'events': rows,
              'source_sha256': {'summary': hashlib.sha256(summary).hexdigest(),
                                'manifest': hashlib.sha256(manifest).hexdigest(),
                                'volume_candidates': hashlib.sha256(a.volumes.read_bytes()).hexdigest(),
                                'inventory': inventory_hash},
              'limitations': ['GPS arrival is nearest event, not synchronized physical position',
                              'axis-aligned UE/NED origin is candidate from startup only',
                              ('cross-process FName/root match does not prove identity; individual segment not matched'
                               if attribution and segment_actors is None else
                               'actor numeric suffix/index reversal is an unverified hypothesis selected after observing this trace'
                               if segment_actors is not None else
                               'search considers any road box, not the event spline or segment'),
                              'point containment ignores drone collision geometry and UE overlap rules',
                              'serialized box transforms and runtime profiles are unverified',
                              'association does not establish natural field membership']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('paired events', len(rows), 'counts', counts)


if __name__ == '__main__':
    main()
