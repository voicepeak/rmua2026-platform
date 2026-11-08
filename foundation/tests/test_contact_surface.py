import struct

import numpy as np
import pytest

from foundation.analyze_contact_surface import analyze
from foundation.map_scene import CandidateMap


def test_contact_source_matching_and_surface_distance():
    placements = [
        {'actor': 'port', 'component': 'surface', 'mesh': 'plane',
         'translation_ue_cm': [10, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]},
        {'actor': 'far', 'component': 'surface', 'mesh': 'plane',
         'translation_ue_cm': [100, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]},
    ]
    mesh = (np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0]]), np.array([[0, 1, 2]]))
    scene = CandidateMap(placements, {'plane': mesh})
    payload = bytearray(136)
    struct.pack_into('<3f', payload, 0x18, 11, 1, .5)
    line = ('MAP_CONTACT mono=1.0 drone=0x1 drone_component=0x2 drone_xyz=0,0,0 '
            'source_actor=0x3 source_component=0x4 source_xyz=10,0,0 source_quat=0,0,0,1 '
            f'hit_bytes={payload.hex()}')
    result = analyze(scene, line)
    assert result[0]['candidate_actor'] == 'port'
    assert result[0]['candidate_surface_distance_cm'] == pytest.approx(.5)
    assert result[0]['second_nearest_transform_distance_cm'] == pytest.approx(90)
    with pytest.raises(ValueError, match='readable'):
        analyze(scene, line.replace(payload.hex(), 'null'))
