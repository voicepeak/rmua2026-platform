import pytest
from foundation.obstacles import Line, Vehicle, generate, segment_distance_squared


def test_fast_line_crossing_and_near_miss():
    line = Line((0, 0, -3.5), (0, 0, 3.5), .026)
    assert line.hit_swept((-10, 0, 0), (10, 0, 0), .2)
    assert not line.hit_swept((-10, 1, 0), (10, 1, 0), .2)
    assert segment_distance_squared((0, 0, 0), (0, 0, 0), (1, 0, 0), (1, 0, 0)) == pytest.approx(1)


def test_relative_vehicle_motion_and_parallel_miss():
    vehicle = Vehicle((0, 0, 0), (1, 1, 1), (10, 0, 0), 0)
    assert vehicle.hit_swept((5, 0, 0), (5, 0, 0), 1, .1)
    assert not vehicle.hit_swept((5, 5, 0), (5, 5, 0), 1, .1)


def test_sweeps_agree_when_step_is_split():
    line = Line((0, 0, -2), (0, 0, 2), .026)
    whole = line.hit_swept((-20, .1, 0), (20, .1, 0), .2)
    split = (line.hit_swept((-20, .1, 0), (0, .1, 0), .2) or
             line.hit_swept((0, .1, 0), (20, .1, 0), .2))
    assert whole and split
    v1 = Vehicle((-10, 0, 0), (1, 1, 1), (20, 0, 0), 0)
    v2 = Vehicle((0, 0, 0), (1, 1, 1), (20, 0, 0), 0)
    assert v1.hit_swept((0, 0, 0), (0, 0, 0), 1, .2)
    assert (v1.hit_swept((0, 0, 0), (0, 0, 0), .5, .2) or
            v2.hit_swept((0, 0, 0), (0, 0, 0), .5, .2))


def test_reproducible_generation_and_launch_clearance():
    args = (42, [-30]*3, [30]*3, [0]*3, 5, 3, 2, .026, 7, [2, 1, 1], [1, 6])
    assert generate(*args) == generate(*args)
    lines, vehicles = generate(*args)
    assert all(not line.hit_swept([0]*3, [0]*3, 5) for line in lines)
    assert all(not v.hit_swept([0]*3, [0]*3, 0, 5) for v in vehicles)
