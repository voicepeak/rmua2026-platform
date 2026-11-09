"""Audit opt-in cooked-mesh fallbacks across all serialized maps.

The query proves only that each added component is reachable through the
candidate map's own transform and triangle data. It is not official collision
coverage, a collision channel check or an official contact comparison.
"""
import hashlib
import json
from pathlib import Path

from foundation.map_scene import CandidateMap


ROOT = Path('foundation')
ASSETS = ROOT / 'asset_analysis'
OUTPUT = ROOT / 'reports' / 'v2_map_cooked_mesh_audit.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_scene(number):
    if number == 1:
        return CandidateMap.load(
            ASSETS, include_unverified_simple=True, include_unverified_sprites=True,
            include_unverified_racks=True, include_unverified_road_cubes=True,
            include_unverified_building_meshes=True, include_unverified_rack_meshes=True)
    assets = ASSETS
    return CandidateMap.load_serialized_candidates(
        ROOT / 'maps' / f'OL{number}', assets / 'mesh_collision',
        simple_shapes_file=assets / 'OL1_simple_convex_candidates.json',
        road_cube_shapes_file=assets / 'OL1_simple_convex_candidates.json',
        rack_shapes_file=assets / 'rack_simple_candidates.json',
        sprite_shapes_file=assets / 'sprite_collision_candidates.json',
        building_mesh_fallback=True, rack_mesh_fallback=True,
        rack_mesh_directory=assets / 'rack_collision')


def check_group(scene, prefix):
    rows = []
    for index, obj in enumerate(scene.placements):
        if not obj['mesh'].startswith(prefix):
            continue
        rotation, translation, scale = scene.transforms[index]
        point = rotation.apply(scene.meshes[obj['mesh']][0][0] * scale) + translation
        hits = scene.sweep_sphere_ue(point, point, .01)['hits']
        rows.append({
            'actor': obj['actor'],
            'component': obj['component'],
            'mesh': obj['mesh'],
            'query_hit': any(hit['actor'] == obj['actor'] and hit['component'] == obj['component']
                             for hit in hits),
        })
    return rows


def main():
    maps = {}
    for number in range(1, 13):
        scene = load_scene(number)
        buildings = check_group(scene, 'unverified_building_mesh:')
        racks = check_group(scene, 'unverified_rack_mesh:')
        fallback_names = sorted(name for name in scene.meshes
                                if name.startswith(('unverified_building_mesh:', 'unverified_rack_mesh:')))
        fallback_sources = {name: scene.source_sha256[name] for name in sorted(scene.source_sha256)
                            if any(mesh.split(':', 1)[1] in name for mesh in fallback_names)}
        records = {**{x['mesh']: x for x in buildings}, **{x['mesh']: x for x in racks}}
        maps[f'OL{number}'] = {
            'instance_count_with_cooked_fallbacks': len(scene.placements),
            'included_building_mesh': len(buildings),
            'included_rack_mesh': len(racks),
            'excluded_building_components': scene.excluded_count,
            'excluded_rack_components': getattr(scene, 'excluded_rack_components', None),
            'building_query_hits': sum(x['query_hit'] for x in buildings),
            'rack_query_hits': sum(x['query_hit'] for x in racks),
            'all_fallback_queries_hit': all(x['query_hit'] for x in buildings + racks),
            'fallback_meshes': fallback_names,
            'fallback_source_sha256': fallback_sources,
            'source_sha256_digest': hashlib.sha256(
                json.dumps(scene.source_sha256, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
        }
        print(f"OL{number}: instances={len(scene.placements)} buildings={len(buildings)} "
              f"racks={len(racks)} hits={sum(x['query_hit'] for x in buildings + racks)}")

    result = {
        'status': 'candidate cooked-mesh query audit; not official collision coverage',
        'map_count': len(maps),
        'aggregate': {
            'building_mesh_instances': sum(x['included_building_mesh'] for x in maps.values()),
            'rack_mesh_instances': sum(x['included_rack_mesh'] for x in maps.values()),
            'all_fallback_queries_hit': all(x['all_fallback_queries_hit'] for x in maps.values()),
            'all_building_components_included': all(x['excluded_building_components'] == 0
                                                    for x in maps.values()),
            'all_rack_components_included': all(x['excluded_rack_components'] == 0
                                                for x in maps.values()),
        },
        'maps': maps,
        'analysis_sha256': digest(Path(__file__)),
        'limitations': [
            'only decoded cooked triangle prefixes are used; material and mesh-tree data are not interpreted',
            'collision trace flags are omitted in the source metadata and remain unknown',
            'component transforms, construction scripts and runtime collision channels are unverified',
            'query hits are self-consistency checks against the same extracted geometry, not official contacts',
            'a miss still does not prove free space',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('aggregate', json.dumps(result['aggregate'], ensure_ascii=False))


if __name__ == '__main__':
    main()
