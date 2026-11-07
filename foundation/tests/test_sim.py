import numpy as np
import pytest
from foundation.sim import PhysicsSim
from foundation.quad import Params
from foundation.obstacles import Line, Vehicle
from foundation.effects import ForceReplay


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
