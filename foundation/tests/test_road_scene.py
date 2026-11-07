import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from foundation.road_scene import relative, compose, segment_intersections, RoadScene


def field(values):
    return [{'Value': values}]


def test_ue_rotator_axis_signs():
    _, r, _ = relative({'RelativeRotation': field({'Pitch': 90, 'Yaw': 0, 'Roll': 0})})
    np.testing.assert_allclose(r.apply([1, 0, 0]), [0, 0, 1], atol=1e-12)
    _, r, _ = relative({'RelativeRotation': field({'Pitch': 0, 'Yaw': 90, 'Roll': 0})})
    np.testing.assert_allclose(r.apply([1, 0, 0]), [0, 1, 0], atol=1e-12)


def test_parent_transform_and_scaled_ray():
    parent = (np.array([10., 0, 0]), Rotation.from_euler('z', 90, degrees=True), np.array([2., 2, 2]))
    child = (np.array([1., 0, 0]), Rotation.identity(), np.ones(3))
    t, r, s = compose(parent, child)
    np.testing.assert_allclose(t, [10, 2, 0], atol=1e-12)
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    tri = np.array([[0, 1, 2]])
    assert segment_intersections([.2, .2, -1], [.2, .2, 1], vertices, tri) == pytest.approx([.5])
    obj = {'actor': 'test', 'mesh': 'mesh', 'translation_ue_cm': t,
           'quaternion_xyzw': r.as_quat(), 'scale': s}
    hits = RoadScene([obj], {'mesh': (vertices, tri)}).raycast([9.6, 2.4, -2], [9.6, 2.4, 2])
    assert hits[0]['fraction'] == pytest.approx(.5)
    assert not RoadScene([obj], {'mesh': (vertices, tri)}).raycast([100, 100, -2], [100, 100, 2])
    with pytest.raises(ValueError, match='zero-length'):
        RoadScene([], {}).raycast([0, 0, 0], [0, 0, 0])
