import numpy as np
import pytest

from foundation.obstacles import Line
from foundation.effects import ForceReplay
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_command_hold_uses_same_physics_steps_and_hides_truth():
    single = RmuaPhysics(params(), physics_dt=.0025)
    grouped = RmuaPhysics(params(), physics_dt=.0025)
    initial = {'velocity': [1, 0, 0]}
    observation = single.reset(initial)
    grouped.reset(initial)
    assert observation == {'time_s': 0., 'observable_state': None}
    for _ in range(20):
        single.step([.18]*4, .0025)
    _, events = grouped.step([.18]*4, .05)
    assert events['completed_physics_steps'] == 20
    assert events['map_status'] == 'unimplemented'
    assert events['wind_status'] == 'unidentified'
    assert events['force_degradation_status'] == 'unidentified'
    for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
        np.testing.assert_allclose(single.truth_state()[key], grouped.truth_state()[key], atol=1e-14)
    truth = grouped.truth_state()
    truth['position'][0] = 1000
    assert grouped.truth_state()['position'][0] != 1000


def test_collision_and_timeout_stop_during_command_and_reset():
    line = Line((0, 0, -2), (0, 0, 2), .03)
    sim = RmuaPhysics(params(), physics_dt=.01, lines=[line], drone_radius_m=.1)
    sim.reset({'position': [-1, 0, 0], 'velocity': [50, 0, 0]})
    observation, events = sim.step([0]*4, .05)
    assert events['contact']['kind'] == 'line' and events['terminal']
    assert observation['time_s'] < .05
    with pytest.raises(RuntimeError):
        sim.step([0]*4, .01)
    timeout = RmuaPhysics(params(), physics_dt=.01, timeout_s=.03)
    timeout.reset()
    observation, events = timeout.step([0]*4, .1)
    assert events['timeout'] and events['terminal']
    assert observation['time_s'] == pytest.approx(.03)
    assert timeout.reset()['time_s'] == 0


def test_invalid_command_or_nonintegral_duration_does_not_advance():
    sim = RmuaPhysics(params(), physics_dt=.01)
    sim.reset()
    for pwm, dt in (([.1]*3, .01), ([float('nan')]*4, .01), ([.1]*4, .015)):
        with pytest.raises(ValueError):
            sim.step(pwm, dt)
        assert sim.truth_state()['t'] == 0


def test_timeout_must_include_at_least_one_physics_step():
    with pytest.raises(ValueError, match='at least one'):
        RmuaPhysics(params(), physics_dt=.01, timeout_s=1e-12)
    sim = RmuaPhysics(params(), physics_dt=.01, timeout_s=.01)
    sim.reset()
    _, events = sim.step([0] * 4, .01)
    assert events['timeout'] and events['completed_physics_steps'] == 1


def test_candidate_flight_bounds_stop_at_first_outside_step_and_validate_reset():
    with pytest.raises(ValueError, match='flight bounds'):
        RmuaPhysics(params(), flight_bounds_ned_m=[[0, 0, 0], [1, 0, 1]])
    sim = RmuaPhysics(params(), physics_dt=.01,
                      flight_bounds_ned_m=[[-1, -1, -1], [1, 1, 1]])
    with pytest.raises(ValueError, match='initial position'):
        sim.reset({'position': [2, 0, 0]})
    with pytest.raises(RuntimeError):
        sim.step([0]*4, .01)
    sim.reset({'position': [0, 0, 0], 'velocity': [60, 0, 0]})
    obs, event = sim.step([0]*4, .05)
    assert event['out_of_bounds_candidate'] and event['terminal']
    assert not event['timeout'] and obs['time_s'] == pytest.approx(.02)
    with pytest.raises(RuntimeError):
        sim.step([0]*4, .01)
    sim.reset()
    assert not sim.step([0]*4, .01)[1]['out_of_bounds_candidate']


def test_uncovered_force_schedule_is_rejected_before_any_step():
    replay = ForceReplay([0, .03], [[0, 0, 0], [0, 0, 0]])
    sim = RmuaPhysics(params(), physics_dt=.01, force_replay=replay)
    sim.reset()
    with pytest.raises(ValueError, match='replay interval'):
        sim.step([.1]*4, .04)
    assert sim.truth_state()['t'] == 0
    sim.step([.1]*4, .03)
    assert sim.truth_state()['t'] == pytest.approx(.03)
