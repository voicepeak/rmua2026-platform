import numpy as np
import pytest
from foundation.frames import (ue_cm_to_ned_m, ned_m_to_ue_cm,
                               ue_body_to_ned_frd_quaternion_wxyz,
                               ned_frd_to_ue_body_quaternion_wxyz)


def test_position_origin_and_axis_signs():
    origin = [1000, -2000, 300]
    positions = [[1000, -2000, 300], [1100, -1800, 600]]
    expected = [[0, 0, 0], [1, 2, -3]]
    np.testing.assert_allclose(ue_cm_to_ned_m(positions, origin), expected)
    np.testing.assert_allclose(ned_m_to_ue_cm(expected, origin), positions)


def test_invalid_origin_is_rejected():
    with pytest.raises(ValueError):
        ue_cm_to_ned_m([1, 2, 3], [0, 0])


def test_candidate_attitude_double_reflection_and_roundtrip():
    from scipy.spatial.transform import Rotation
    rotations = Rotation.from_euler('xyz', [[.2, -.4, .7], [-.5, .1, -1.2], [0, 0, 0]])
    ue_q = rotations.as_quat()[:, [3, 0, 1, 2]]
    ned_q = ue_body_to_ned_frd_quaternion_wxyz(ue_q)
    ned_rot = Rotation.from_quat(ned_q[:, [1, 2, 3, 0]])
    reflection = np.diag([1, 1, -1])
    np.testing.assert_allclose(ned_rot.as_matrix(), reflection @ rotations.as_matrix() @ reflection, atol=1e-14)
    np.testing.assert_allclose(ned_frd_to_ue_body_quaternion_wxyz(ned_q), ue_q, atol=1e-14)
    # Basis vectors expose roll/pitch sign flips, not just a circular roundtrip.
    quarter_roll = [2**-.5, 2**-.5, 0, 0]
    np.testing.assert_allclose(ue_body_to_ned_frd_quaternion_wxyz(quarter_roll),
                               [2**-.5, -2**-.5, 0, 0])


def test_nonfinite_and_zero_frame_inputs_are_rejected():
    with pytest.raises(ValueError):
        ue_cm_to_ned_m([float('nan'), 0, 0], [0, 0, 0])
    with pytest.raises(ValueError):
        ue_body_to_ned_frd_quaternion_wxyz([0, 0, 0, 0])
    with pytest.raises(ValueError):
        ned_frd_to_ue_body_quaternion_wxyz([1, 0, float('inf'), 0])
