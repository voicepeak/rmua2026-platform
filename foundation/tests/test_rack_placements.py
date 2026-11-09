import pytest

from foundation.rack_placements import build


def vec(name, x):
    return {'Name': name, 'Value': [{'Value': {'X': x, 'Y': 0, 'Z': 0}}]}


def test_nested_child_actor_transforms_and_mesh_override():
    imports = [{'ObjectName': 'BP_Rack_C'}, {'ObjectName': 'BP_Building_C'},
               {'ObjectName': 'StaticMeshComponent'}, {'ObjectName': 'SceneComponent'},
               {'ObjectName': 'ChildActorComponent'}]
    exports = [
        {'ObjectName': 'building', 'ClassIndex': -2},
        {'ObjectName': 'building_root', 'ClassIndex': -4, 'OuterIndex': 1, 'Data': [vec('RelativeLocation', 100)]},
        {'ObjectName': 'rack_child', 'ClassIndex': -5, 'OuterIndex': 1, 'Data': [{'Name': 'AttachParent', 'Value': 2}]},
        {'ObjectName': 'rack', 'ClassIndex': -1},
        {'ObjectName': 'rack_root', 'ClassIndex': -4, 'OuterIndex': 4, 'Data': [{'Name': 'AttachParent', 'Value': 3}]},
        {'ObjectName': 'part', 'ClassIndex': -3, 'OuterIndex': 4, 'Data': [{'Name': 'AttachParent', 'Value': 5}]},
    ]
    blueprints = {
        'BP_Building': {'Exports': [{'ObjectName': 'building_root_GEN_VARIABLE'},
                                    {'ObjectName': 'rack_child_GEN_VARIABLE', 'Data': [vec('RelativeLocation', 10)]}]},
        'BP_Rack': {'Imports': [{'ObjectName': 'mesh', 'OuterIndex': -2},
                                {'ObjectName': '/Game/mesh', 'ClassName': 'Package'}],
                    'Exports': [{'ObjectName': 'rack_root_GEN_VARIABLE', 'Data': [vec('RelativeLocation', 2)]},
                                {'ObjectName': 'part_GEN_VARIABLE', 'Data': [
                                    {'Name': 'StaticMesh', 'Value': -1}, vec('RelativeLocation', 1)]}]},
    }
    asset = {'Imports': imports, 'Exports': exports}
    result = build(asset, blueprints)
    assert not result['unresolved']
    assert result['placements'][0]['translation_ue_cm'] == pytest.approx([113, 0, 0])
    assert result['placements'][0]['mesh_package'] == '/Game/mesh'
    exports[-1]['Data'].append({'Name': 'StaticMesh', 'Value': -1})
    result = build(asset, blueprints)
    assert not result['placements']
    assert 'override' in result['unresolved'][0]['reason']


def test_rack_candidates_are_opt_in_and_queryable():
    from pathlib import Path
    from foundation.map_scene import CandidateMap
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    default = CandidateMap.load(assets)
    scene = CandidateMap.load(assets, include_unverified_racks=True)
    racks = [x for x in scene.placements if x['mesh'].startswith('unverified_rack:')]
    assert len(scene.placements) == len(default.placements) + 960
    assert len(racks) == 960
    obj = racks[0]
    single = CandidateMap([obj], {obj['mesh']: scene.meshes[obj['mesh']]})
    center = (single.low[0] + single.high[0]) / 2
    start, end = center.copy(), center.copy()
    start[0], end[0] = single.low[0, 0] - 100, single.high[0, 0] + 100
    assert single.raycast_ue(start, end)['hits']


def test_road_cube_candidates_are_opt_in_and_queryable():
    from pathlib import Path
    from foundation.map_scene import CandidateMap
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    scene = CandidateMap.load(assets, include_unverified_road_cubes=True)
    cubes = [x for x in scene.placements if x['mesh'].startswith('unverified_road_cube:')]
    assert len(cubes) == 1884
    obj = cubes[0]
    single = CandidateMap([obj], {obj['mesh']: scene.meshes[obj['mesh']]})
    center = (single.low[0] + single.high[0]) / 2
    start, end = center.copy(), center.copy()
    start[0], end[0] = single.low[0, 0] - 100, single.high[0, 0] + 100
    assert single.raycast_ue(start, end)['hits']


def test_rack_paper_pointer_and_plane_cooked_meshes_are_opt_in():
    from pathlib import Path
    from foundation.map_scene import CandidateMap
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    with pytest.raises(ValueError, match='building mesh fallback'):
        CandidateMap.load(assets, include_unverified_building_meshes=True)
    with pytest.raises(ValueError, match='rack mesh fallback'):
        CandidateMap.load(assets, include_unverified_rack_meshes=True)
    scene = CandidateMap.load(
        assets, include_unverified_simple=True, include_unverified_racks=True,
        include_unverified_building_meshes=True, include_unverified_rack_meshes=True)
    buildings = [x for x in scene.placements if x['mesh'].startswith('unverified_building_mesh:')]
    racks = [x for x in scene.placements if x['mesh'].startswith('unverified_rack_mesh:')]
    assert len(buildings) == 12 and len(racks) == 480
    assert scene.excluded_count == 0 and scene.excluded_rack_components == 0
    for group in (buildings, racks):
        obj = group[0]
        single = CandidateMap([obj], {obj['mesh']: scene.meshes[obj['mesh']]})
        center = (single.low[0] + single.high[0]) / 2
        start, end = center.copy(), center.copy()
        start[0], end[0] = single.low[0, 0] - 100, single.high[0, 0] + 100
        assert single.raycast_ue(start, end)['hits']
