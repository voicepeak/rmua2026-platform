from foundation.benchmark_batch_map_contact import exercise
from foundation.map_scene import CandidateMap


def test_contact_benchmark_exercises_real_candidate_map():
    scene = CandidateMap.load('foundation/asset_analysis')
    result = exercise(scene, 2, 91)
    assert result['terminal_count'] == 2
    assert result['preflight_candidate_count'] > 0 and result['preflight_hits']
    assert max(result['max_state_errors'].values()) < 1e-12
