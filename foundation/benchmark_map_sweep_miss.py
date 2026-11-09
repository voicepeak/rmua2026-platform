"""High-speed swept-sphere checks against actual OL1 candidate triangles.

Each case starts outside a candidate triangle, crosses it, and is compared with
a direct continuous sphere sweep. The simulator is then stepped at several
maximum substep sizes. This is a candidate-map self-consistency benchmark, not
official collision miss rate or contact validation.
"""
import hashlib
import json
import platform
import sys
from pathlib import Path

import numpy as np

from foundation.frames import ue_cm_to_ned_m
from foundation.map_scene import CandidateMap
from foundation.quad import Params
from foundation.sim import PhysicsSim


ROOT = Path('foundation')
ASSETS = ROOT / 'asset_analysis'
OUTPUT = ROOT / 'reports' / 'v2_map_sweep_miss_benchmark.json'
SPEEDS_M_S = (10., 50., 100., 150.)
SUBSTEPS_S = (.001, .005, .01, .02, .05, .1)
RADIUS_M = .05


def _cases(scene, count_per_group=16):
    groups = {
        'road': {'Segment', 'Segment_Half'},
        'wall': {name for name in scene.meshes
                 if name not in ('Segment', 'Segment_Half')
                 and not name.startswith(('unverified_rack', 'unverified_road_cube', 'unverified_sprite'))},
    }
    cases = []
    for group, meshes in groups.items():
        found = 0
        for index, obj in enumerate(scene.placements):
            if obj['mesh'] not in meshes:
                continue
            rotation, translation, scale = scene.transforms[index]
            vertices, indices = scene.meshes[obj['mesh']]
            for triangle_index in indices:
                tri = rotation.apply(vertices[triangle_index] * scale) + translation
                normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                norm = np.linalg.norm(normal)
                if norm < 1e-9:
                    continue
                normal /= norm
                if group == 'road' and normal[2] < .9:
                    continue
                center = tri.mean(axis=0)
                start_ue = center + normal * 100
                end_ue = center - normal * 100
                if scene.sweep_sphere_ue(start_ue, start_ue, RADIUS_M * 100)['hits']:
                    continue
                probe = scene.sweep_sphere_ue(start_ue, end_ue, RADIUS_M * 100)
                if not probe['hits']:
                    continue
                cases.append({
                    'group': group,
                    'actor': obj['actor'],
                    'component': obj.get('component'),
                    'mesh': obj['mesh'],
                    'start_ue_cm': start_ue.tolist(),
                    'end_ue_cm': end_ue.tolist(),
                    'start_ned_m': ue_cm_to_ned_m(start_ue, [0, 0, 0]).tolist(),
                    'end_ned_m': ue_cm_to_ned_m(end_ue, [0, 0, 0]).tolist(),
                })
                found += 1
                break
            if found == count_per_group:
                break
        if found != count_per_group:
            raise ValueError(f'only found {found} {group} sweep cases')
    return cases


def main():
    scene = CandidateMap.load(
        ASSETS, include_unverified_simple=True, include_unverified_sprites=True,
        include_unverified_racks=True, include_unverified_road_cubes=True,
        include_unverified_building_meshes=True, include_unverified_rack_meshes=True)
    cases = _cases(scene)
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    results = {}
    for speed in SPEEDS_M_S:
        duration = 2.0 / speed
        for substep in SUBSTEPS_S:
            hits, misses = 0, []
            for case in cases:
                start = np.asarray(case['start_ned_m'])
                end = np.asarray(case['end_ned_m'])
                sim = PhysicsSim(params, candidate_map=scene, map_origin_ue_cm=[0, 0, 0],
                                 drone_radius_m=RADIUS_M, max_substep_s=substep)
                sim.reset({'position': start, 'velocity': (end - start) / duration})
                _, info = sim.step([0] * 4, duration)
                if info['collision'] is not None and info['collision']['kind'] == 'static_candidate':
                    hits += 1
                else:
                    misses.append({'actor': case['actor'], 'component': case['component'],
                                   'group': case['group']})
            results.setdefault(f'{speed:g}', {})[str(substep)] = {
                'speed_m_s': speed, 'duration_s': duration, 'substep_s': substep,
                'cases': len(cases), 'hits': hits, 'misses': len(misses),
                'miss_rate': len(misses) / len(cases), 'miss_examples': misses[:5],
            }
    paths = [Path(__file__), *(Path('foundation') / name for name in
             ('sim.py', 'quad.py', 'map_scene.py', 'swept_mesh.py', 'frames.py'))]
    result = {
        'status': 'candidate-map high-speed swept-sphere self-check; not official miss rate',
        'candidate_instance_count': len(scene.placements),
        'candidate_map_source_sha256': scene.source_sha256,
        'case_count': len(cases),
        'groups': {group: sum(case['group'] == group for case in cases)
                   for group in ('road', 'wall')},
        'speeds_m_s': SPEEDS_M_S,
        'substeps_s': SUBSTEPS_S,
        'radius_m': RADIUS_M,
        'results': results,
        'code_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        'platform': platform.platform(),
        'processor': platform.processor(),
        'python_version': sys.version.split()[0],
        'numpy_version': np.__version__,
        'limitations': [
            'direct sweep and simulator share the same candidate triangles and swept test',
            'straight normal crossings only; no grazing, turning or moving obstacles',
            'candidate transforms, channels and contact response remain unverified',
            'a zero miss count is not a measured official miss probability or a frozen acceptance threshold',
            'the terminal state is the end of the substep containing the hit, not exact contact time',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('cases', len(cases), 'runs', len(SPEEDS_M_S) * len(SUBSTEPS_S))
    total_misses = sum(x['misses'] for group in results.values() for x in group.values())
    print('total misses', total_misses)


if __name__ == '__main__':
    main()
