import numpy as np
import pytest
from foundation.map_scene import CandidateMap


def test_partial_map_ray_and_explicit_frame():
    vertices = np.array([[0, 0, 0], [100, 0, 0], [0, 100, 0]], float)
    obj = {'actor': 'a', 'mesh': 'triangle', 'translation_ue_cm': [1000, 2000, 300],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'triangle': (vertices, np.array([[0, 1, 2]]))}, excluded_count=12)
    result = scene.raycast_ned([.2, .2, -1], [.2, .2, 1], origin_ue_cm=[1000, 2000, 300])
    assert result['hits'][0]['point_ned_m'] == pytest.approx([.2, .2, 0])
    assert result['excluded_building_components'] == 12
    assert 'partial' in result['coverage']
    assert not scene.raycast_ue([9000, 9000, 0], [9000, 9000, 100])['hits']
    with pytest.raises(ValueError):
        scene.raycast_ue([0, 0, 0], [0, 0, 0])
