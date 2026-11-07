import numpy as np
import pytest
from foundation.effects import ForceReplay


def test_force_replay_interpolates_and_respects_region():
    force = ForceReplay([0, 2], [[0, 0, 0], [2, 4, 6]], [-1, -1, -1], [1, 1, 1])
    np.testing.assert_allclose(force.sample(1, [0, 0, 0]), [1, 2, 3])
    np.testing.assert_allclose(force.sample(1, [2, 0, 0]), [0, 0, 0])
    with pytest.raises(ValueError, match='extrapolate'):
        force.sample(3, [0, 0, 0])


def test_force_replay_rejects_bad_bounds():
    with pytest.raises(ValueError, match='bounds'):
        ForceReplay([0, 1], [[0]*3, [1]*3], [0]*3, [0]*3)
