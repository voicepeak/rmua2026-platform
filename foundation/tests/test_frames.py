import numpy as np
import pytest
from foundation.frames import ue_cm_to_ned_m, ned_m_to_ue_cm


def test_position_origin_and_axis_signs():
    origin = [1000, -2000, 300]
    positions = [[1000, -2000, 300], [1100, -1800, 600]]
    expected = [[0, 0, 0], [1, 2, -3]]
    np.testing.assert_allclose(ue_cm_to_ned_m(positions, origin), expected)
    np.testing.assert_allclose(ned_m_to_ue_cm(expected, origin), positions)


def test_invalid_origin_is_rejected():
    with pytest.raises(ValueError):
        ue_cm_to_ned_m([1, 2, 3], [0, 0])
