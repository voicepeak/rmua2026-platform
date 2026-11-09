"""Audit ground-adjacent vehicle map rejection with single vs sub-box spheres.

The probe vehicle extents are explicit candidates, not an official vehicle
dimension. Top-facing road triangles are sampled only to create a controlled
candidate-surface workload; the audit is not official road-contact evidence.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.frames import ue_cm_to_ned_m
from foundation.map_scene import CandidateMap
from foundation.obstacles import CandidateMapObstacleFilter, Vehicle


ROOT = Path('foundation')
ASSETS = ROOT / 'asset_analysis'
OUTPUT = ROOT / 'reports' / 'v2_vehicle_map_clearance_audit.json'
VEHICLE_HALF_EXTENTS_M = (.8, .4, .3)
GAPS_M = (0., .1, .3, .5)
SUBDIVISIONS = (1, 2, 4, 10)


def top_surface_points(scene, count):
    points = []
    for index, obj in enumerate(scene.placements):
        if obj['mesh'] not in ('Segment', 'Segment_Half'):
            continue
        rotation, translation, scale = scene.transforms[index]
        vertices, indices = scene.meshes[obj['mesh']]
        for tri_index in indices:
            tri = rotation.apply(vertices[tri_index] * scale) + translation
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal /= norm
            if normal[2] <= .9:
                continue
            points.append({'actor': obj['actor'], 'component': obj.get('component'),
                           'surface_ue_cm': tri.mean(axis=0).tolist(),
                           'normal_ue': normal.tolist()})
            break
        if len(points) == count:
            break
    if len(points) != count:
        raise ValueError(f'only found {len(points)} top-facing road points')
    return points


def main():
    scene = CandidateMap.load(
        ASSETS, include_unverified_simple=True, include_unverified_sprites=True,
        include_unverified_racks=True, include_unverified_road_cubes=True,
        include_unverified_building_meshes=True, include_unverified_rack_meshes=True,
        assume_closed_meshes_solid=True)
    points = top_surface_points(scene, 64)
    filters = {sub: CandidateMapObstacleFilter(
        scene, origin_ue_cm=[0, 0, 0], horizon_s=1., vehicle_subdivisions=sub)
        for sub in SUBDIVISIONS}
    results = {}
    for sub, check in filters.items():
        by_gap = {}
        for gap in GAPS_M:
            accepted = []
            for row in points:
                center_ue = (np.asarray(row['surface_ue_cm'])
                             + np.asarray(row['normal_ue'])
                             * ((VEHICLE_HALF_EXTENTS_M[2] + gap) * 100.))
                center = ue_cm_to_ned_m(center_ue, [0, 0, 0])
                vehicle = Vehicle(tuple(center), VEHICLE_HALF_EXTENTS_M, (0, 0, 0), 0.)
                if check(vehicle):
                    accepted.append({'actor': row['actor'], 'component': row['component']})
            by_gap[str(gap)] = {'accepted': len(accepted), 'sample_count': len(points),
                                'accepted_instances': accepted}
        results[str(sub)] = by_gap
    result = {
        'status': 'candidate map clearance audit; not official vehicle or road contact',
        'vehicle_half_extents_m': VEHICLE_HALF_EXTENTS_M,
        'vehicle_geometry_source': 'explicit probe dimensions, not an official vehicle measurement',
        'sample_count': len(points),
        'sample_source': 'first top-facing triangle of distinct OL1 road candidates',
        'gaps_m': GAPS_M,
        'subdivisions': SUBDIVISIONS,
        'results': results,
        'candidate_map_source_sha256': scene.source_sha256,
        'candidate_map_instance_count': len(scene.placements),
        'limitations': [
            'the probe vehicle dimensions are caller-chosen, not an official vehicle asset',
            'sampled road triangles may overlap other candidate geometry or map objects',
            'increasing subdivisions reduces sphere-cover conservatism but not all of it',
            'exact road contact, slope following and official clearance remain unverified',
            'accepted routes are not proof of official passability',
        ],
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for sub in SUBDIVISIONS:
        print(sub, {gap: results[str(sub)][str(gap)]['accepted'] for gap in GAPS_M})


if __name__ == '__main__':
    main()
