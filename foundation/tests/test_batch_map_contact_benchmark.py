from foundation.benchmark_batch_map_contact import exercise
from foundation.benchmark_batch_dense_map import exercise as dense_exercise
from foundation.map_scene import CandidateMap


def test_contact_benchmark_exercises_real_candidate_map():
    scene = CandidateMap.load('foundation/asset_analysis')
    result = exercise(scene, 2, 91)
    assert result['terminal_count'] == 2
    assert result['preflight_candidate_count'] > 0 and result['preflight_hits']
    assert max(result['max_state_errors'].values()) < 1e-12


def test_dense_map_benchmark_uses_distinct_full_candidate_surfaces():
    scene = CandidateMap.load(
        'foundation/asset_analysis', include_unverified_simple=True,
        include_unverified_sprites=True, include_unverified_racks=True,
        include_unverified_road_cubes=True, include_unverified_building_meshes=True,
        include_unverified_rack_meshes=True)
    result = dense_exercise(scene, 4, 91)
    assert len(scene.placements) == 12609
    assert result['terminal_count'] == 4
    assert result['distinct_surface_components'] == 4
    assert result['median_probe_candidate_count'] >= 1
    assert max(result['max_state_errors'].values()) < 1e-12
