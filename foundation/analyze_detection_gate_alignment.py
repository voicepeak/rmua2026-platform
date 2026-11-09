"""Compare candidate detection-gate centers with the candidate stage-1 route."""
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.detection_gates import DetectionGateCandidates


GATES = Path('foundation/maps/OL1/detection_gate_candidates.json')
ROUTE = Path('foundation/datasets/route_candidate_lift6_20261108/route.json')
OUTPUT = Path('foundation/reports/v2_detection_gate_alignment.json')


def analyze(gates, route):
    points = np.asarray(route, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('invalid candidate route')
    rows = []
    for group, records in gates.groups:
        for gate in records:
            target = gate['position_ue_cm'] / 100
            distances = np.linalg.norm(points-target, axis=1)
            nearest = int(np.argmin(distances))
            rows.append({'group': group, 'index': gate['index'], 'actor': gate['actor'],
                         'target_m': target.tolist(), 'nearest_route_index': nearest,
                         'nearest_route_point_m': points[nearest].tolist(),
                         'distance_m': float(distances[nearest])})
    return rows


def main():
    gates = DetectionGateCandidates.load(GATES, axis=0, half_extents_cm=(200, 200, 200))
    route = json.loads(ROUTE.read_text(encoding='utf-8'))['routes']['static_spline1']
    rows = analyze(gates, route)
    distances = np.array([row['distance_m'] for row in rows])

    def stats(values):
        values = np.asarray(values)
        return {'count': int(len(values)), 'min': float(values.min()),
                'median': float(np.median(values)), 'max': float(values.max()),
                'within_1_m': int((values <= 1).sum()),
                'within_2_m': int((values <= 2).sum()),
                'within_5_m': int((values <= 5).sum())}

    result = {
        'status': 'candidate detection-gate centers vs candidate stage-1 route; not official alignment',
        'gate_manifest_sha256': gates.manifest_sha256,
        'route_sha256': hashlib.sha256(ROUTE.read_bytes()).hexdigest(),
        'gate_count': len(rows),
        'groups': {group: stats([row['distance_m'] for row in rows if row['group'] == group])
                   for group in ('road1', 'road3')},
        'all_gates': stats(distances),
        'gates': rows,
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'the route is a candidate static_spline1 connector, not an official flown trajectory',
            'the same route is compared to both road1 and road3 candidate gates; group-specific route coverage is not established',
            'runtime gate roots are not decoded trigger planes or apertures',
            'closest-point distance is not a gate crossing or a free-space guarantee',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(result['groups'], indent=1))


if __name__ == '__main__':
    main()
