import numpy as np
import pytest

from rmua.core.frames import (
    ue_cm_to_m,
    ue_to_ned,
    ned_to_ue,
    wrap_pi,
    yaw_deg_to_forward,
    yaw_deg_to_right,
)


def test_units():
    assert ue_cm_to_m(100.0) == 1.0
    assert np.allclose(ue_cm_to_m(np.array([0.0, 150.0, -50.0])), [0.0, 1.5, -0.5])


def test_ned_roundtrip():
    ue = np.array([1.0, 2.0, 3.0])
    ned = ue_to_ned(ue)
    assert np.allclose(ned, [1.0, 2.0, -3.0])
    assert np.allclose(ned_to_ue(ned), ue)


def test_yaw_frame():
    forward = yaw_deg_to_forward(-83.16549682617188)
    right = yaw_deg_to_right(-83.16549682617188)
    assert np.isclose(np.linalg.norm(forward), 1.0)
    assert np.isclose(np.linalg.norm(right), 1.0)
    assert np.isclose(np.dot(forward, right), 0.0)
    assert forward[2] == 0.0


def test_wrap_pi():
    assert np.isclose(abs(wrap_pi(3.0 * np.pi)), np.pi)
    assert np.isclose(abs(wrap_pi(-3.0 * np.pi)), np.pi)
    with pytest.raises(ValueError):
        ue_to_ned(np.array([1.0, 2.0]))
