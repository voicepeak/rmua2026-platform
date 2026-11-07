"""Approximate static sanity check of candidate road vertices vs spline knots.

Spline coordinates are treated as local to the exported spline transform.
Nearest-vertex distances are upper bounds on mesh-surface distance, and neither
independent official observations nor physical collision acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from foundation.road_scene import RoadScene


def generate(placements, meshes, map_data):
    vertices = []
    for item in placements:
        v = meshes[item['mesh']][0]
        vertices.append(Rotation.from_quat(item['quaternion_xyzw']).apply(v * item['scale']) + item['translation_ue_cm'])
    tree = cKDTree(np.concatenate(vertices))
    samples = []
    for route in map_data['config']['RouteConfigs']:
        spline = route.get('spline')
        if not spline:
            continue
        pos, rot = spline['location'], spline['rotation']
        origin = np.array([pos[k] for k in 'XYZ'], dtype=float)
        q = Rotation.from_euler('ZYX', [float(rot['Yaw']), -float(rot['Pitch']), -float(rot['Roll'])], degrees=True)
        for i, point in enumerate(spline.get('points', [])):
            local = np.array([point['pos'][k] for k in 'XYZ'], dtype=float)
            world = origin + q.apply(local)
            distance, _ = tree.query(world)
            samples.append({'route_index': route['RouteIndex'], 'point_index': i,
                            'world_ue_cm': world.tolist(), 'nearest_road_vertex_m': float(distance / 100)})
    values = np.array([x['nearest_road_vertex_m'] for x in samples])
    scene = RoadScene(placements, meshes)
    for sample in samples:
        point = np.asarray(sample['world_ue_cm'])
        hits = scene.raycast(point + [0, 0, 1000], point + [0, 0, -1000])
        sample['vertical_20m_segment_hit_count'] = len(hits)
    return {'samples': samples, 'count': len(samples),
            'vertical_20m_segments_with_hit': sum(x['vertical_20m_segment_hit_count'] > 0 for x in samples),
            'distance_m': {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
                           'max': float(values.max())},
            'limitations': ['spline local-to-world convention and candidate road placement assumed',
                            'nearest vertex is not nearest triangle or collision contact',
                            'spline knots may legitimately sit away from road surface',
                            'vertical line is zero-radius and cannot establish free space or collision outcome',
                            'not independent runtime validation']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('analysis', type=Path)
    p.add_argument('map_file', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    placements = json.loads((a.analysis/'OL1_road_placements_candidate.json').read_text())['instances']
    meshes = {}
    for name in ('Segment', 'Segment_Half'):
        with np.load(a.analysis/(name+'_collision.npz')) as mesh:
            meshes[name] = (mesh['vertices'], mesh['triangles'])
    data = json.loads(a.map_file.read_text(encoding='utf-8'))['RMUA2024OL1']
    result = generate(placements, meshes, data)
    result['map_sha256'] = hashlib.sha256(a.map_file.read_bytes()).hexdigest()
    result['placements_sha256'] = hashlib.sha256((a.analysis/'OL1_road_placements_candidate.json').read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'distance_m': result['distance_m'], 'vertical_hits': result['vertical_20m_segments_with_hit'],
                      'knots': result['count']}))


if __name__ == '__main__':
    main()
