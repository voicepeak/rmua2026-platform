"""Analytic synthetic line/vehicle sweep checks over speed and timestep grids."""
import hashlib
import json
from pathlib import Path

from foundation.obstacles import Line, Vehicle


def benchmark():
    rows = []
    radius = .2
    line = Line((0, 0, -2), (0, 0, 2), .03)
    for speed in (10., 50., 150.):
        for dt in (.001, .01, .1):
            length = speed * dt
            for offset_factor in (-1.5, -.9, 0., .9, 1.5):
                for kind in ('line', 'vehicle'):
                    threshold = .23 if kind == 'line' else 1.0
                    y = offset_factor * threshold
                    if kind == 'line':
                        hit = line.hit_swept((-length/2, y, 0), (length/2, y, 0), radius)
                    else:
                        vehicle = Vehicle((0, 0, 0), (1, .8, 1), (speed/2, 0, 0), 0)
                        hit = vehicle.hit_swept((-length/2, y, 0), (length/2+speed*dt/2, y, 0), dt, radius)
                    expected = abs(y) <= threshold
                    rows.append({'kind': kind, 'relative_speed_m_s': speed, 'dt_s': dt,
                                 'lateral_offset_m': y, 'expected_hit': expected, 'observed_hit': bool(hit)})
    return rows


def main():
    rows = benchmark()
    misses = [x for x in rows if x['expected_hit'] and not x['observed_hit']]
    false_hits = [x for x in rows if not x['expected_hit'] and x['observed_hit']]
    result = {'status': 'synthetic axis-aligned collision check; not official parity',
              'case_count': len(rows), 'miss_count': len(misses), 'false_hit_count': len(false_hits),
              'cases': rows, 'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'model_sha256': hashlib.sha256(Path('foundation/obstacles.py').read_bytes()).hexdigest(),
              'limitations': ['analytic line crossings and constant-velocity axis-aligned vehicles only',
                              'vehicle expansion is conservative near edges and corners',
                              'turning, real-world mesh, collision response and official collision channels are not covered']}
    Path('foundation/reports/v2_collision_sweep_benchmark.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print('cases', len(rows), 'misses', len(misses), 'false hits', len(false_hits))
    if misses or false_hits:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
