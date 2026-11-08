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


def test_float32_keyframe_rounding_cannot_reuse_stale_wind_index():
    constants = {'radians_per_degree': math.pi / 180, 'zero': 0}
    wind = WindGenerator([Keyframe(0, 1, 0, 0), Keyframe(1, 2, 0, 0), Keyframe(2, 3, 0, 0)], constants)
    assert wind.sample(1.5) == pytest.approx((2, 0, 0))
    # This float64 is below 2 but rounds to 2 in the reference's float32 clock.
    with pytest.raises(ValueError, match='outside'):
        wind.sample(math.nextafter(2., 0.))
    # Adjacent times that collapse in float32 cannot form valid intervals.
    with pytest.raises(ValueError, match='ordered in float32'):
        WindGenerator([Keyframe(1., 1, 0, 0), Keyframe(1. + 1e-9, 1, 0, 0)], constants)


def test_interleaved_seed_generation_and_nonmonotonic_sampling_are_independent():
    from vendor.rmua_re.wind import WindParameters, generate_keyframes, keyframes_bytes
    constants = {'hold_min_s': 3., 'rand_denominator': 16777216., 'base_hold_s': 5.,
                 'transition_s': 2., 'speed_cap_delta': 1., 'speed_spread': 1.,
                 'speed_bias': -.5, 'angle_span': 360., 'zero': 0., 'radians_per_degree': math.pi/180}
    parameters = WindParameters(30., 50., 4., 1.)
    first = generate_keyframes(parameters, seed=123, constants=constants)
    other = generate_keyframes(parameters, seed=456, constants=constants)
    again = generate_keyframes(parameters, seed=123, constants=constants)
    assert keyframes_bytes(first) == keyframes_bytes(again)
    assert keyframes_bytes(first) != keyframes_bytes(other)
    a, b = WindGenerator(first, constants), WindGenerator(again, constants)
    times = [first[0].time, first[-2].time, first[1].time]
    for t in times:
        expected = WindGenerator(again, constants).sample(t)
        assert a.sample(t) == pytest.approx(expected)
        b.sample(first[-2].time)
        assert b.sample(t) == pytest.approx(expected)
