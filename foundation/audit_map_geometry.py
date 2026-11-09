"""Probe representative gate, road and building candidate surfaces.

All results are self-consistency checks against the same extracted geometry.
They are not official aperture, clearance or contact validation.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.gate_geometry import GateMeshVolumes
from foundation.map_scene import CandidateMap


ROOT = Path('foundation')
ASSETS = ROOT / 'asset_analysis'
OUTPUT = ROOT / 'reports' / 'v2_map_geometry_probe.json'


def _surface_sample(scene, meshes, count, *, min_normal_z=None):
    rows = []
    for index, obj in enumerate(scene.placements):
        if obj['mesh'] not in meshes:
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
            if min_normal_z is not None and normal[2] < min_normal_z:
                continue
            point = tri.mean(axis=0)
            hits = scene.raycast_ue(point + normal * 100, point - normal * 100)['hits']
            matched = any(hit['actor'] == obj['actor'] and hit['component'] == obj.get('component')
                          for hit in hits)
            rows.append({'actor': obj['actor'], 'component': obj.get('component'),
                         'mesh': obj['mesh'], 'matched': matched,
                         'hit_count': len(hits), 'first_hits': hits[:3]})
            break
        if len(rows) == count:
            break
    return rows


def probe(scene, gates, road_count=64, wall_count=64):
    gate_rows = []
    for name, translation, rotation, scale in gates.gates:
        midpoint = (gates.low + gates.high) / 2
        local_start, local_end = midpoint.copy(), midpoint.copy()
        local_start[gates.axis] -= 1
        local_end[gates.axis] += 1
        start = translation + rotation.apply(local_start * scale)
        end = translation + rotation.apply(local_end * scale)
        hits = gates.crossings_ue(start, end)
        gate_rows.append({'gate': name, 'matched': any(hit['gate'] == name for hit in hits),
                          'hit_count': len(hits)})
    road_meshes = {'Segment', 'Segment_Half'}
    wall_meshes = {name for name in scene.meshes
                   if name not in road_meshes
                   and not name.startswith(('unverified_rack', 'unverified_road_cube',
                                            'unverified_sprite'))}
    road = _surface_sample(scene, road_meshes, road_count, min_normal_z=.9)
    walls = _surface_sample(scene, wall_meshes, wall_count)
    complex_meshes = {'Building_Factory_Factory', 'Building_Port',
                      'Building_Junction_2_Building_Junction_2'}
    complex_simple = _surface_sample(scene, complex_meshes, 25)
    return {
        'gate_center_plane_crossings': gate_rows,
        'gate_matched': sum(x['matched'] for x in gate_rows),
        'road_surface_self_hits': road,
        'road_matched': sum(x['matched'] for x in road),
        'wall_surface_self_hits': walls,
        'wall_matched': sum(x['matched'] for x in walls),
        'complex_as_simple_surfaces': complex_simple,
        'complex_as_simple_matched': sum(x['matched'] for x in complex_simple),
    }


def main():
    scene = CandidateMap.load(
        ASSETS, include_unverified_simple=True, include_unverified_sprites=True,
        include_unverified_racks=True, include_unverified_road_cubes=True,
        include_unverified_building_meshes=True, include_unverified_rack_meshes=True)
    gates = GateMeshVolumes.load(ASSETS, crossing_axis=1)
    result = probe(scene, gates)
    official = {}
    for name in ('contact_surface_seed123.json', 'contact_surface_seed456.json'):
        path = ROOT / 'reports' / name
        data = json.loads(path.read_text(encoding='utf-8'))
        official[name] = {
            'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'event_count': data['event_count'],
            'max_candidate_surface_distance_cm': data['max_candidate_surface_distance_cm'],
            'actors': sorted({event['candidate_actor'] for event in data['events']}),
            'components': sorted({event['candidate_component'] for event in data['events']}),
        }
    output = {
        'status': 'candidate geometry self-consistency probe; not official clearance or contact',
        'candidate_map_instance_count': len(scene.placements),
        'candidate_map_source_sha256': scene.source_sha256,
        'gate_count': len(gates.gates),
        'gate_axis': gates.axis,
        'probe': result,
        'official_contact_evidence': official,
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'gate events use a candidate center plane and explicit local axis, not an official aperture or trigger',
            'road and wall rays are self-hits against the same extracted triangles',
            'official contact evidence covers one startup sprite component and uses all-stop GDB',
            'factory entrance and representative wall clearance are not covered by official contacts',
            'candidate transforms, collision channels and omitted categories remain unverified',
        ],
    }
    OUTPUT.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('gates', result['gate_matched'], '/', len(gates.gates),
          'road', result['road_matched'], '/', len(result['road_surface_self_hits']),
          'wall', result['wall_matched'], '/', len(result['wall_surface_self_hits']))


if __name__ == '__main__':
    main()
