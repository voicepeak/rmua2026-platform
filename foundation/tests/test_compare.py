import numpy as np
import pytest
from foundation.compare import compare


def sample(t, position=None, quaternion=None):
    return {"t": t, "position": position or [t, 0, 0], "velocity": [1, 0, 0],
            "quaternion_wxyz": quaternion or [1, 0, 0, 0], "omega": [0, 0, 0]}


def test_interpolation_and_quaternion_sign():
    reference = [sample(t) for t in (0, 0.5, 1)]
    candidate = [sample(0), sample(1, quaternion=[-1, 0, 0, 0])]
    report = compare(reference, candidate, max_gap=1.1)
    assert report["n_matched"] == 3
    assert report["metrics"]["position"]["max"] == pytest.approx(0)
    assert report["metrics"]["attitude_deg"]["max"] == pytest.approx(0)


def test_known_quaternion_error_and_time_offset():
    ref = [sample(0), sample(1)]
    quat = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    cand = [sample(2, [0, 0, 0], quat), sample(3, [1, 0, 0], quat)]
    report = compare(ref, cand, time_offset=-2, max_gap=2)
    assert report["metrics"]["attitude_deg"]["rmse"] == pytest.approx(90)
    assert report["metrics"]["position"]["max"] == pytest.approx(0)


def test_reject_bad_timestamps_and_large_gaps():
    with pytest.raises(ValueError, match="strictly increasing"):
        compare([sample(0), sample(0)], [sample(0), sample(1)])
    with pytest.raises(ValueError, match="fewer than two"):
        compare([sample(.2), sample(.8)], [sample(0), sample(1)], max_gap=.1)
