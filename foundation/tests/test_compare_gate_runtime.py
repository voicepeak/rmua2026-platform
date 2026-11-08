import pytest

from foundation.compare_gate_runtime import compare


def test_runtime_gate_matching_and_duplicate_rejection():
    candidate = {'unresolved': [], 'placements': [
        {'actor': 'one', 'translation_ue_cm': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]},
        {'actor': 'two', 'translation_ue_cm': [100, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}]}
    lines = ['RMUA_GATE actor=0x2 fname_index=2 fname_number=0 root=0x3 xyz_cm=100,0,0 quat=0,0,0,-1 scale=1,1,1',
             'RMUA_GATE actor=0x1 fname_index=1 fname_number=0 root=0x4 xyz_cm=0,0,0 quat=0,0,0,1 scale=1,1,1']
    result = compare(candidate, lines)
    assert [x['runtime_actor'] for x in result] == ['0x1', '0x2']
    assert max(x['position_error_cm'] for x in result) == 0
    assert max(x['rotation_error_deg'] for x in result) == pytest.approx(0)
    with pytest.raises(ValueError, match='identity mismatch'):
        compare(candidate, [lines[0], lines[0]])
