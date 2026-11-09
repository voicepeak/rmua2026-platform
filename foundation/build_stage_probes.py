"""Generate stage-N return-route probes and teleport GDB scripts.

Stage N (2..6) starts at the previous route's endpoint, returns to the hub, then
departs on the next route.  These probes inject the stage at the native
stage-init call and teleport to the previous route endpoint, then fly that
return route to the hub so its detection components and score can be observed.
Diagnostic only.
"""
import csv
import json
import math
from pathlib import Path

from foundation.build_stage_manifests import catmull_rom

ROOT = Path('.')
TEMPLATE = ROOT / 'foundation' / 'probes' / 'stage2_teleport.gdb'
OUT_CONFIG = ROOT / 'foundation' / 'config'
OUT_PROBES = ROOT / 'foundation' / 'probes'

STAGES = {
    3: {'inject': (5, 8, 3), 'route': 2, 'name': 'route5 return'},
    4: {'inject': (8, 12, 4), 'route': 4, 'name': 'route8 return'},
    5: {'inject': (12, 10, 5), 'route': 5, 'name': 'route12 return'},
    6: {'inject': (10, 7, 6), 'route': 6, 'name': 'route10 return'},
}


def control_points():
    rows = [r for r in csv.DictReader((ROOT / 'data' / 'tables' / 'routes.csv').open(encoding='utf-8'))
            if r['map'] == 'RMUA2024OL1']
    groups = {}
    for row in rows:
        groups.setdefault(int(row['route_index']), []).append(row)
    result = {}
    for index, rows in groups.items():
        rows.sort(key=lambda r: int(r['point']))
        result[index] = [(float(r['world_x']) / 100, float(r['world_y']) / 100,
                          float(r['world_z']) / 100) for r in rows]
    return result


def main():
    controls = control_points()
    for stage, spec in STAGES.items():
        route = catmull_rom(controls[spec['route']], 5.0)
        start = controls[spec['route']][0]
        waypoints = []
        for i, point in enumerate(route):
            waypoints.append({
                'name': 'stage%d_%03d' % (stage, i),
                'position': [point[0], point[1], -point[2]],
                'speed': 2.0 if i == 0 else 7.0,
                'arrive_radius_m': 0.8 if i == 0 else 1.5,
                'hold_s': 0.0,
            })
        plan = {
            'name': 'OL1_stage%d_return_probe_20261110' % stage,
            'purpose': 'Stage-%d injection + teleport to %s endpoint; fly the return route to the hub and record detection components/score. Diagnostic only.' % (stage, spec['name']),
            'frame': 'NED metres, z down; routes.csv world points UE metres z-up',
            'safety': {'max_speed': 7.5, 'max_duration_s': 300.0, 'max_xy_m': 1500.0,
                       'min_z_m': -190.0, 'max_z_m': 1.0, 'stale_pose_s': 0.3,
                       'position_gain': 0.8, 'log_period_s': 0.5},
            'waypoints': waypoints,
        }
        (OUT_CONFIG / ('g2_stage%d_return_probe_20261110.json' % stage)).write_text(
            json.dumps(plan, indent=1) + '\n', encoding='utf-8')
        template = TEMPLATE.read_text(encoding='utf-8')
        template = template.replace('TARGET_CM = (54570.0, 51960.0, 3288.0)',
                                    'TARGET_CM = (%.3f, %.3f, %.3f)' % (
                                        start[0] * 100, start[1] * 100, start[2] * 100))
        template = template.replace('new=3,5,2', 'new=%d,%d,%d' % spec['inject'])
        template = template.replace('gdb.execute(\'set $esi=3\')', 'gdb.execute(\'set $esi=%d\')' % spec['inject'][0])
        template = template.replace('gdb.execute(\'set $edx=5\')', 'gdb.execute(\'set $edx=%d\')' % spec['inject'][1])
        template = template.replace('gdb.execute(\'set $ecx=2\')', 'gdb.execute(\'set $ecx=%d\')' % spec['inject'][2])
        (OUT_PROBES / ('stage%d_return.gdb' % stage)).write_text(template, encoding='utf-8')
        print('stage', stage, spec['name'], 'start', start, 'waypoints', len(waypoints))


if __name__ == '__main__':
    main()
