import json
from pathlib import Path

import pytest
import numpy as np

from foundation.map_scene import CandidateMap
from foundation.gate_geometry import GateMeshVolumes


ROOT = Path(__file__).resolve().parents[1]


def test_serialized_other_maps_are_loadable_and_separately_sourced():
    sources = set()
    roads = []
    for number in range(2, 13):
        directory = ROOT / 'maps' / f'OL{number}'
        road = json.loads((directory / 'road_candidates.json').read_bytes())
        building = json.loads((directory / 'building_candidates.json').read_bytes())
        assert road['source_file'] == f'RMUA2024OL{number}.json'
        assert len(road['instances']) == 483
        assert len(building['placements']) == 8682
        assert not road['missing_blueprint_actors'] and not building['unresolved']
        sources.add(road['source_sha256']['map_json'])
        roads.append(road['instances'])
        scene = CandidateMap.load_serialized_candidates(directory, ROOT/'asset_analysis'/'mesh_collision')
        assert len(scene.placements) == 508 and scene.excluded_count == 8657
        gate = GateMeshVolumes.load_serialized_candidates(directory, ROOT/'asset_analysis', crossing_axis=0)
        assert len(gate.gates) == 12
        name, center, rotation, scale = gate.gates[0]
        midpoint = (gate.low + gate.high) / 2
        start_local, end_local = midpoint.copy(), midpoint.copy()
        start_local[0] -= 1
        end_local[0] += 1
        start = center + rotation.apply(start_local * scale)
        end = center + rotation.apply(end_local * scale)
        assert any(hit['gate'] == name and np.isclose(hit['fraction'], .5)
                   for hit in gate.crossings_ue(start, end))
    assert len(sources) == 11
    assert roads[0] != roads[1]


def test_rejects_cross_map_candidate_pairs(tmp_path):
    (tmp_path/'road_candidates.json').write_bytes((ROOT/'maps'/'OL2'/'road_candidates.json').read_bytes())
    (tmp_path/'building_candidates.json').write_bytes((ROOT/'maps'/'OL3'/'building_candidates.json').read_bytes())
    with pytest.raises(ValueError, match='mismatched or unresolved'):
        CandidateMap.load_serialized_candidates(tmp_path, ROOT/'asset_analysis'/'mesh_collision')


def test_rejects_gate_from_another_map(tmp_path):
    (tmp_path/'road_candidates.json').write_bytes((ROOT/'maps'/'OL2'/'road_candidates.json').read_bytes())
    (tmp_path/'gate_candidates.json').write_bytes((ROOT/'maps'/'OL3'/'gate_candidates.json').read_bytes())
    with pytest.raises(ValueError, match='mismatched or unresolved'):
        GateMeshVolumes.load_serialized_candidates(tmp_path, ROOT/'asset_analysis', crossing_axis=0)


def test_other_map_simple_candidates_are_explicit_and_queryable():
    assets = ROOT/'asset_analysis'
    simple_file = assets/'OL1_simple_convex_candidates.json'
    scene = CandidateMap.load_serialized_candidates(ROOT/'maps'/'OL2', assets/'mesh_collision',
                                                     simple_shapes_file=simple_file)
    assert len(scene.placements) == 9153 and scene.excluded_count == 12
    assert str(simple_file) in scene.source_sha256
    index = next(i for i, p in enumerate(scene.placements) if p['mesh'].startswith('unverified_simple:'))
    obj = scene.placements[index]
    rotation, translation, scale = scene.transforms[index]
    vertex = scene.meshes[obj['mesh']][0][0]
    point = rotation.apply(vertex * scale) + translation
    hits = scene.sweep_sphere_ue(point, point, .01)['hits']
    assert any(x['actor'] == obj['actor'] and x['component'] == obj['component'] for x in hits)


def test_other_map_auxiliary_groups_and_cross_map_rejection(tmp_path):
    assets = ROOT/'asset_analysis'
    source = ROOT/'maps'/'OL2'
    scene = CandidateMap.load_serialized_candidates(
        source, assets/'mesh_collision', simple_shapes_file=assets/'OL1_simple_convex_candidates.json',
        road_cube_shapes_file=assets/'OL1_simple_convex_candidates.json',
        rack_shapes_file=assets/'rack_simple_candidates.json')
    assert len(scene.placements) == 11997 and scene.excluded_count == 12
    for prefix in ('unverified_road_cube:', 'unverified_rack:'):
        index = next(i for i, p in enumerate(scene.placements) if p['mesh'].startswith(prefix))
        obj = scene.placements[index]
        rotation, translation, scale = scene.transforms[index]
        vertex = scene.meshes[obj['mesh']][0][0]
        point = rotation.apply(vertex * scale) + translation
        hits = scene.sweep_sphere_ue(point, point, .01)['hits']
        assert any(x['actor'] == obj['actor'] and x['component'] == obj['component'] for x in hits)
    for name in ('road_candidates.json', 'building_candidates.json'):
        (tmp_path/name).write_bytes((source/name).read_bytes())
    (tmp_path/'road_cube_candidates.json').write_bytes((ROOT/'maps'/'OL3'/'road_cube_candidates.json').read_bytes())
    with pytest.raises(ValueError, match='mismatched or unresolved auxiliary'):
        CandidateMap.load_serialized_candidates(tmp_path, assets/'mesh_collision',
                                                road_cube_shapes_file=assets/'OL1_simple_convex_candidates.json')


def test_other_map_cooked_mesh_fallback_is_explicit_and_queryable():
    assets = ROOT/'asset_analysis'
    source = ROOT/'maps'/'OL2'
    kwargs = dict(
        simple_shapes_file=assets/'OL1_simple_convex_candidates.json',
        road_cube_shapes_file=assets/'OL1_simple_convex_candidates.json',
        rack_shapes_file=assets/'rack_simple_candidates.json',
        sprite_shapes_file=assets/'sprite_collision_candidates.json')
    default = CandidateMap.load_serialized_candidates(source, assets/'mesh_collision', **kwargs)
    assert len(default.placements) == 12105 and default.excluded_count == 12
    scene = CandidateMap.load_serialized_candidates(
        source, assets/'mesh_collision', building_mesh_fallback=True,
        rack_mesh_fallback=True, rack_mesh_directory=assets/'rack_collision', **kwargs)
    assert len(scene.placements) == 12597 and scene.excluded_count == 0
    assert scene.included_building_mesh == 12
    assert scene.included_rack_mesh == 480 and scene.excluded_rack_components == 0
    assert len(scene.placements) == len(default.placements) + 492
    for prefix in ('unverified_building_mesh:', 'unverified_rack_mesh:'):
        index = next(i for i, p in enumerate(scene.placements) if p['mesh'].startswith(prefix))
        obj = scene.placements[index]
        rotation, translation, scale = scene.transforms[index]
        point = rotation.apply(scene.meshes[obj['mesh']][0][0] * scale) + translation
        assert any(hit['actor'] == obj['actor'] and hit['component'] == obj['component']
                   for hit in scene.sweep_sphere_ue(point, point, .01)['hits'])
    assert any('AirSimROS_SM_plane.npz' in name for name in scene.source_sha256)
    assert any('Rack_Paper.npz' in name for name in scene.source_sha256)
    assert any('Rack_Pointer.npz' in name for name in scene.source_sha256)


def test_cooked_mesh_fallback_requires_its_opt_in_prerequisites():
    assets = ROOT/'asset_analysis'
    source = ROOT/'maps'/'OL2'
    with pytest.raises(ValueError, match='building mesh fallback'):
        CandidateMap.load_serialized_candidates(source, assets/'mesh_collision',
                                                building_mesh_fallback=True)
    with pytest.raises(ValueError, match='rack mesh fallback'):
        CandidateMap.load_serialized_candidates(
            source, assets/'mesh_collision',
            rack_shapes_file=assets/'rack_simple_candidates.json',
            rack_mesh_fallback=True)


def test_sprite_group_is_opt_in_and_rejects_cross_map_sources(tmp_path):
    assets, source = ROOT/'asset_analysis', ROOT/'maps'/'OL2'
    sprite_shapes = assets/'sprite_collision_candidates.json'
    scene = CandidateMap.load_serialized_candidates(source, assets/'mesh_collision', sprite_shapes_file=sprite_shapes)
    sprites = [obj for obj in scene.placements if obj['mesh'].startswith('unverified_sprite:')]
    assert len(sprites) == 108 and len(scene.placements) == 616
    assert str(sprite_shapes) in scene.source_sha256
    index = next(i for i, obj in enumerate(scene.placements) if obj['mesh'].startswith('unverified_sprite:'))
    obj = scene.placements[index]
    rotation, translation, scale = scene.transforms[index]
    point = rotation.apply(scene.meshes[obj['mesh']][0][0] * scale) + translation
    assert any(hit['actor'] == obj['actor'] and hit['component'] == obj['component']
               for hit in scene.sweep_sphere_ue(point, point, .01)['hits'])
    for name in ('road_candidates.json', 'building_candidates.json'):
        (tmp_path/name).write_bytes((source/name).read_bytes())
    (tmp_path/'sprite_candidates.json').write_bytes((ROOT/'maps'/'OL3'/'sprite_candidates.json').read_bytes())
    with pytest.raises(ValueError, match='mismatched or unresolved auxiliary'):
        CandidateMap.load_serialized_candidates(tmp_path, assets/'mesh_collision', sprite_shapes_file=sprite_shapes)


def test_sprite_instance_source_override_is_not_silently_ignored():
    from foundation.sprite_inventory import build
    asset = {'Imports': [{'ObjectName': 'BP_Building_1_C'}, {'ObjectName': 'PaperSpriteComponent'}],
             'Exports': [{'ClassIndex': -1, 'ObjectName': 'building'},
                         {'ClassIndex': -2, 'ObjectName': 'logo', 'OuterIndex': 1,
                          'Data': [{'Name': 'SourceSprite', 'Value': -3}]}]}
    result = build(asset, {'BP_Building_1': {'Exports': []}})
    assert not result['placements']
    assert 'instance SourceSprite override' in result['unresolved'][0]['reason']
