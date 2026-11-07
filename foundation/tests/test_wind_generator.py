import math
import pytest
from foundation.wind_generator import WindGenerator
from vendor.rmua_re.wind import Keyframe


def test_replay_is_bounded_and_interpolates():
    constants = {'radians_per_degree': math.pi / 180, 'zero': 0}
    frames = [Keyframe(0, 1, 0, 1), Keyframe(2, 3, 0, 0), Keyframe(4, 3, 0, 0)]
    wind = WindGenerator(frames, constants)
    assert wind.sample(1) == pytest.approx((2, 0, 0))
    assert wind.sample(2.5) == pytest.approx((3, 0, 0))
    assert len(wind.keyframe_bytes()) == 48
    for t in (-1, 4, float('nan')):
        with pytest.raises(ValueError):
            wind.sample(t)


def test_reject_invalid_frames():
    with pytest.raises(ValueError):
        WindGenerator([Keyframe(0, 1, 0, 0), Keyframe(0, 1, 0, 0)], {})
    with pytest.raises(ValueError):
        WindGenerator([Keyframe(0, float('nan'), 0, 0), Keyframe(1, 1, 0, 0)], {})
