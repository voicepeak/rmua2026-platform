import json
import numpy as np
import pytest
from foundation.map_scene import CandidateMap
from foundation.lidar_map_probe import probe


def test_synthetic_lidar_ray_hits_known_plane(tmp_path):
    vertices = np.array([[0, 0, -100], [100, 0, -100], [0, 100, -100]], float)
    obj = {'actor': 'plane', 'mesh': 'm', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'m': (vertices, np.array([[0, 1, 2]]))})
    path = tmp_path/'scan.npz'
    meta = {'position': [0, 0, 0], 'orientation_xyzw': [0, 0, 0, 1],
            'gps_stamp_ros': 1., 'imu_stamp_ros': 1., 'cloud_stamp_ros': 1.}
    np.savez(path, xyz=np.array([[.2, .2, 1.05]]), metadata=json.dumps(meta))
    result = probe(scene, path, origin_ue_cm=[0, 0, 0], count=1, seed=1)
    assert result['partial_map_hits'] == 1
    assert result['rays'][0]['signed_error_m'] == pytest.approx(0, abs=1e-8)
