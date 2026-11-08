import numpy as np
import pytest
from foundation.sim import PhysicsSim
from foundation.quad import Params
from foundation.obstacles import Line, Vehicle, VehiclePath
from foundation.effects import ForceReplay, RecordedEffects
from foundation.out_time import OutTimeCounter


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_repeatable_hover_and_unknowns():
    p = params()
    hover = p.mass*p.gravity/(4*p.max_thrust)
    sim = PhysicsSim(p)
    initial = {'rotor_signal': [hover]*4}
    sim.reset(initial)
    first, info = sim.step([hover]*4, .1)
    sim.reset(initial)
    second, _ = sim.step([hover]*4, .1)
    np.testing.assert_allclose(first['position'], second['position'])
    assert info['static_map_collision'] == 'unimplemented'
    assert info['physics_status'] == 'uncalibrated_candidate'


def test_swept_collision_and_reset():
    line = Line((0, 0, -2), (0, 0, 2), .03)
    sim = PhysicsSim(params(), lines=[line], max_substep_s=.05)
    sim.reset({'position': [-1, 0, 0], 'velocity': [50, 0, 0]})
    _, info = sim.step([0]*4, .05)
    assert info['collision'] == {'kind': 'line', 'index': 0}
    with pytest.raises(RuntimeError):
        sim.step([0]*4, .05)
    sim.reset()
    assert not sim.terminal


def test_moving_vehicle_and_replay_bounds():
    vehicle = Vehicle((-1, 0, 0), (.1, .1, .1), (20, 0, 0), 0)
    force = ForceReplay([0, .5], [[0, 0, 0], [0, 0, 0]])
    sim = PhysicsSim(params(), vehicles=[vehicle], force_replay=force, max_substep_s=.02)
    sim.reset()
    _, info = sim.step([0]*4, .1)
    assert info['collision']['kind'] == 'vehicle'
    sim.reset()
    with pytest.raises(ValueError, match='replay interval'):
        sim.step([0]*4, .75)
    assert sim.quad.time == 0


def test_path_vehicle_collision_during_step():
    vehicle = VehiclePath((0, .05, .10), ((-2, 0, 0), (-1, 0, 0), (1, 0, 0)),
                          (.1, .1, .1), (0, 0))
    sim = PhysicsSim(params(), vehicles=[vehicle], max_substep_s=.05)
    sim.reset()
    _, info = sim.step([0]*4, .1)
    assert info['collision'] == {'kind': 'vehicle', 'index': 0}


def test_recorded_effects_split_transitions_and_counter_without_terminal():
    effects = RecordedEffects([0, .03, .06, .1], None, [1, 0, 1, 1])
    sim = PhysicsSim(params(), recorded_effects=effects,
                     out_time_counter=OutTimeCounter(.02), max_substep_s=.1)
    manual = PhysicsSim(params(), max_substep_s=.1)
    sim.reset(); manual.reset()
    actual, info = sim.step([.2]*4, .1)
    for duration, scale in ((.03, 1), (.03, .25), (.04, 1)):
        expected, _ = manual.step([.2]*4, duration, rotor_target_scale=scale)
    np.testing.assert_allclose(actual['position'], expected['position'], atol=1e-12)
    np.testing.assert_allclose(actual['rotor_signal'], expected['rotor_signal'], atol=1e-12)
    assert info['out_time_s'] == pytest.approx(.03)
    assert info['out_time_exceeded'] and not info['terminal']
    sim.reset()
    assert sim.out_time_counter.elapsed_s == 0 and not sim.out_time_exceeded
    _, suppressed = sim.step([.2]*4, .1, timeout_suppressed=True)
    assert suppressed['out_time_s'] == pytest.approx(.03) and not suppressed['out_time_exceeded']
    sim.reset()
    with pytest.raises(ValueError, match='cannot be combined'):
        sim.step([.2]*4, .01, rotor_target_scale=1)
    with pytest.raises(ValueError, match='extrapolate'):
        sim.step([.2]*4, .2)
    assert sim.quad.time == 0


def test_recorded_effects_wind_interpolation_and_field_hold():
    effects = RecordedEffects([0, 1, 2], [[0, 0, 0], [2, 4, 0], [0, 0, 0]], [1, 0, 1])
    wind, flag = effects.sample(.5)
    np.testing.assert_allclose(wind, [1, 2, 0])
    assert flag == 1 and effects.sample(1)[1] == 0
    assert effects.transitions(.5, 1.5) == [1]
    with pytest.raises(ValueError, match='extrapolate'):
        effects.sample(2.01)


def test_recorded_field_can_combine_with_explicit_wind():
    class ObserveDrag:
        def __init__(self):
            self.winds = []
        def wrench(self, rotation, velocity, omega, wind):
            self.winds.append(wind.copy())
            return np.zeros(3), np.zeros(3)
    aero = ObserveDrag()
    effects = RecordedEffects([0, .1], None, [0, 0])
    sim = PhysicsSim(params(), recorded_effects=effects, aero_model=aero)
    sim.reset()
    sim.step([.2]*4, .01, wind_world_m_s=[1, 2, 3])
    assert aero.winds and all(np.array_equal(wind, [1, 2, 3]) for wind in aero.winds)
