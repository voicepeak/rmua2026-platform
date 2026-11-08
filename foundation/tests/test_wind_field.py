import math

import numpy as np
import pytest

from foundation.aero import DragFaces
from foundation.effects import RecordedEffects
from foundation.quad import Params
from foundation.runtime import RmuaPhysics
from foundation.sim import PhysicsSim
from foundation.wind_field import CandidateWindField
from foundation.wind_generator import WindGenerator
from vendor.rmua_re.wind import Keyframe


def generator():
    return WindGenerator([Keyframe(0, 1, 0, 1), Keyframe(1, 3, 0, 0), Keyframe(2, 3, 0, 0)],
                         {'radians_per_degree': math.pi / 180, 'zero': 0})


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def drag():
    normals = np.vstack([np.eye(3), -np.eye(3)])
    return DragFaces(np.zeros((6, 3)), normals, [.08]*6, 1.225, 0.)


def field():
    transform = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    return CandidateWindField(generator(), source_to_ned=transform,
                              bounds_min_ned_m=[-1]*3, bounds_max_ned_m=[1]*3,
                              source_time_offset_s=.2)


def test_explicit_wind_frame_region_and_time():
    candidate = field()
    assert candidate.sample(.3, [0, 0, 0]) == pytest.approx([0, 2, 0])
    assert candidate.sample(.3, [2, 0, 0]) == pytest.approx([0, 0, 0])
    with pytest.raises(ValueError, match='extrapolate'):
        candidate.sample(1.8, [0, 0, 0])
    with pytest.raises(ValueError, match='transform'):
        CandidateWindField(generator(), source_to_ned=np.zeros((3, 3)),
                           bounds_min_ned_m=[-1]*3, bounds_max_ned_m=[1]*3)


def test_generated_wind_is_applied_at_each_physics_step_and_bounds_are_preflighted():
    auto = RmuaPhysics(params(), physics_dt=.01, wind_field=field(), aero_model=drag())
    manual = PhysicsSim(params(), max_substep_s=.01, aero_model=drag())
    initial = {'rotor_signal': [.18]*4}
    auto.reset(initial); manual.reset(initial)
    _, events = auto.step([.18]*4, .04)
    for i in range(4):
        manual.step([.18]*4, .01, wind_world_m_s=field().sample(i*.01, [0, 0, 0]))
    for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
        np.testing.assert_allclose(auto.truth_state()[key], manual.quad.state()[key], atol=1e-12)
    assert events['wind_field_status'] == 'unverified_spatial_wind_candidate'
    before = auto.truth_state()['t']
    with pytest.raises(ValueError, match='extrapolate'):
        auto.step([.18]*4, 1.76)
    assert auto.truth_state()['t'] == before


def test_wind_sources_are_explicit_and_field_flags_may_coexist():
    with pytest.raises(ValueError, match='aerodynamic'):
        PhysicsSim(params(), wind_field=field())
    recorded_wind = RecordedEffects([0, 1], [[0, 0, 0]]*2, [1, 1])
    with pytest.raises(ValueError, match='choose candidate wind'):
        PhysicsSim(params(), wind_field=field(), aero_model=drag(), recorded_effects=recorded_wind)
    flags = RecordedEffects([0, 1], None, [0, 0])
    sim = PhysicsSim(params(), wind_field=field(), aero_model=drag(), recorded_effects=flags,
                     field_rotor_target_scale=.25)
    sim.step([.18]*4, .01)
    with pytest.raises(ValueError, match='cannot be combined'):
        sim.step([.18]*4, .01, wind_world_m_s=[1, 0, 0])


def test_generated_wind_batch_uses_reference_fallback():
    from foundation.batch import BatchRmuaPhysics
    batch = BatchRmuaPhysics([RmuaPhysics(params(), physics_dt=.01, wind_field=field(), aero_model=drag())
                              for _ in range(2)])
    batch.reset()
    _, events = batch.batch_step([[.18]*4, [.18]*4], .02)
    assert batch.last_backend == 'reference'
    assert all(e['wind_field_status'] == 'unverified_spatial_wind_candidate' for e in events)
