"""Build OL1 per-stage route manifests from the static route table.

Sources:
  data/tables/routes.csv  (map, route_index, spline, control points, interval)
  data/audits/ol1_seed123/road3_components.json (30 runtime road3 centres)

The waypoint geometry is a diagnostic reconstruction: control points are
densified with a Catmull-Rom spline and route joints use the same tangent
extension / Bezier connector idea as the remote splines_to_yaml tool.  It is
not an official trajectory and it is not accepted as official stage geometry.
"""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path('.')
ROUTES = ROOT / 'data' / 'tables' / 'routes.csv'
ROAD3 = ROOT / 'data' / 'audits' / 'ol1_seed123' / 'road3_components.json'
OUT = ROOT / 'foundation' / 'maps' / 'OL1'

STAGES = {
    1: [(1, 1), (3, -1)],
    2: [(3, 1), (2, -1)],
    3: [(2, 1), (4, -1)],
    4: [(4, 1), (5, -1)],
    5: [(5, 1), (6, -1)],
    6: [(6, 1), (7, -1)],
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def control_points():
    rows = [r for r in csv.DictReader(ROUTES.open(encoding='utf-8')) if r['map'] == 'RMUA2024OL1']
    groups = {}
    for row in rows:
        groups.setdefault(int(row['route_index']), []).append(row)
    result = {}
    for index, rows in groups.items():
        rows.sort(key=lambda r: int(r['point']))
        result[index] = [(float(r['world_x']) / 100.0,
                          float(r['world_y']) / 100.0,
                          float(r['world_z']) / 100.0) for r in rows]
    return result


def catmull_rom(points, step=4.0):
    if len(points) < 2:
        raise ValueError('need at least two control points')
    padded = [points[0]] + list(points) + [points[-1]]
    out = []
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        count = max(2, int(math.ceil(math.dist(p1, p2) / step)))
        for j in range(count):
            t = j / count
            out.append(tuple(
                0.5 * (2 * p1[k] + (-p0[k] + p2[k]) * t
                       + (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t * t
                       + (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t * t * t)
                for k in range(3)))
    out.append(points[-1])
    return out


def unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    return (dx / length, dy / length)


def connect(incoming, outgoing, clearance=25.0, spacing=3.0):
    a, b = incoming[-1], outgoing[0]
    gap = math.hypot(b[0] - a[0], b[1] - a[1])
    if gap < 2 * spacing:
        return list(incoming) + list(outgoing)
    d0 = unit(incoming[-2], a)
    d1 = unit(b, outgoing[1])
    extension = min(clearance, gap * 0.2)
    c = (a[0] + extension * d0[0], a[1] + extension * d0[1])
    d = (b[0] - extension * d1[0], b[1] - extension * d1[1])
    handle = math.hypot(d[0] - c[0], d[1] - c[1]) * 0.4
    p1 = (c[0] + handle * d0[0], c[1] + handle * d0[1])
    p2 = (d[0] - handle * d1[0], d[1] - handle * d1[1])
    xy = [a[:2]]
    n = max(2, int(math.ceil(extension / spacing)))
    xy += [(a[0] + extension * i / n * d0[0], a[1] + extension * i / n * d0[1])
           for i in range(1, n + 1)]
    n = max(8, int(math.ceil(gap / spacing)))
    for i in range(1, n + 1):
        t = i / n
        u = 1.0 - t
        xy.append(tuple(u ** 3 * c[j] + 3 * u * u * t * p1[j] + 3 * u * t * t * p2[j] + t ** 3 * d[j]
                        for j in (0, 1)))
    n = max(2, int(math.ceil(extension / spacing)))
    xy += [(d[0] + extension * i / n * d1[0], d[1] + extension * i / n * d1[1])
           for i in range(1, n + 1)]
    arc = [0.0]
    for before, after in zip(xy, xy[1:]):
        arc.append(arc[-1] + math.hypot(after[0] - before[0], after[1] - before[1]))
    bridge = [(x, y, a[2] + (b[2] - a[2]) * (3 * (s / arc[-1]) ** 2 - 2 * (s / arc[-1]) ** 3))
              for (x, y), s in zip(xy[1:-1], arc[1:-1])]
    return list(incoming) + bridge + list(outgoing)


def road3_centres():
    gates = json.loads(ROAD3.read_text(encoding='utf-8'))['gates']
    gates.sort(key=lambda g: g['index'])
    return [(g['position_cm'][0] / 100.0, g['position_cm'][1] / 100.0,
             g['position_cm'][2] / 100.0) for g in gates]


def main():
    controls = control_points()
    road3 = road3_centres()
    manifests = {}
    for stage, segments in STAGES.items():
        paths = []
        for index, direction in segments:
            if index == 3:
                # Road3 runtime centres give the correct gate-height path.
                points = list(reversed(road3)) if direction == 1 else road3
                source = 'data/audits/ol1_seed123/road3_components.json'
            else:
                points = controls[index]
                if direction == -1:
                    points = list(reversed(points))
                points = catmull_rom(points)
                source = 'data/tables/routes.csv'
            paths.append((index, direction, points, source))
        route = paths[0][2]
        for _, _, points, _ in paths[1:]:
            route = connect(route, points)
        if len(route) < 2:
            raise ValueError('empty route')
        length = sum(math.dist(route[i], route[i + 1]) for i in range(len(route) - 1))
        waypoints = []
        for i, point in enumerate(route):
            waypoints.append({
                'name': 'stage%d_%03d' % (stage, i),
                'position': [point[0], point[1], -point[2]],
                'speed': 3.0 if i == 0 else 8.0,
                'arrive_radius_m': 0.5 if i == 0 else 1.5,
                'hold_s': 0.0,
            })
        manifest = {
            'status': 'diagnostic route reconstruction; official stage geometry unverified',
            'stage': stage,
            'route_segments': [
                {'route_index': index, 'direction': 'forward' if direction == 1 else 'reverse',
                 'source': source, 'points': len(points)} for index, direction, _, source in paths],
            'length_m': length,
            'expected_detection_components_ceil_length_over_21m': int(math.ceil(length / 21.0)),
            'known_detection_components': (
                {'route3': 30} if any(index == 3 for index, _ in segments) else {}),
            'waypoint_count': len(waypoints),
            'waypoints_sha256': hashlib.sha256(
                json.dumps(waypoints, separators=(',', ':')).encode('utf-8')).hexdigest(),
            'source_sha256': {'routes.csv': digest(ROUTES), 'road3_components.json': digest(ROAD3)},
            'limitations': [
                'route control points are sparse static spline points, densified here with Catmull-Rom',
                'hub connectors are reconstructed, not official',
                'expected component count uses ceil(length / 21 m) and is a hypothesis',
                'no score or contact evidence is implied by this manifest',
            ],
        }
        (OUT / ('stage%d_route_manifest.json' % stage)).write_text(
            json.dumps(manifest, indent=1) + '\n', encoding='utf-8')
        plan = {
            'name': 'OL1_stage%d_route_20261110' % stage,
            'purpose': 'diagnostic stage route reconstruction for runtime component/score audit',
            'frame': 'NED metres, z down; routes.csv world points are UE metres z-up',
            'safety': {'max_speed': 8.0, 'max_duration_s': 900.0, 'max_xy_m': 1500.0,
                       'min_z_m': -190.0, 'max_z_m': 1.0, 'stale_pose_s': 0.3,
                       'position_gain': 0.8, 'log_period_s': 0.5},
            'waypoints': waypoints,
        }
        (OUT / ('stage%d_waypoints.json' % stage)).write_text(
            json.dumps(plan, indent=1) + '\n', encoding='utf-8')
        manifests[stage] = {k: manifest[k] for k in
                            ('length_m', 'expected_detection_components_ceil_length_over_21m',
                             'known_detection_components', 'waypoint_count', 'waypoints_sha256')}
    print(json.dumps(manifests, indent=1))


if __name__ == '__main__':
    main()
