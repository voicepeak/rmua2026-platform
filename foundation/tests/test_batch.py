import numpy as np
import pytest

from foundation.batch import BatchRmuaPhysics
from foundation.obstacles import Line
from foundation.effects import RecordedEffects
from foundation.out_time import OutTimeCounter
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_batch_matches_separate_single_instances_and_reset():
    batch = BatchRmuaPhysics([RmuaPhysics(params(), physics_dt=.01) for _ in range(3)])
    singles = [RmuaPhysics(params(), physics_dt=.01) for _ in range(3)]
    initial = [{'position': [i, 0, 0], 'rotor_signal': [.18]*4} for i in range(3)]
    batch.reset(initial)
    for env, state in zip(singles, initial):
        env.reset(state)
    rng = np.random.default_rng(2026)
    for _ in range(10):
        actions = rng.uniform(.15, .22, (3, 4))
        observations, events = batch.batch_step(actions, .02)
        assert batch.last_backend == 'vectorized_free_flight'
        for index, env in enumerate(singles):
            obs, event = env.step(actions[index], .02)
            assert observations[index] == obs and events[index] == event
            for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
                np.testing.assert_allclose(batch.truth_state(index)[key], env.truth_state()[key], atol=1e-14)
    before = batch.truth_state(1)['position'].copy()
    batch.reset([None, None, None], active=[True, False, False])
    assert batch.truth_state(0)['t'] == 0
    np.testing.assert_array_equal(batch.truth_state(1)['position'], before)


def test_batch_terminal_mask_and_invalid_action_preflight():
    line = Line((0, 0, -2), (0, 0, 2), .03)
    first = RmuaPhysics(params(), physics_dt=.01, lines=[line], drone_radius_m=.1)
    second = RmuaPhysics(params(), physics_dt=.01)
    batch = BatchRmuaPhysics([first, second])
    batch.reset([{'position': [-1, 0, 0], 'velocity': [50, 0, 0]}, None])
    with pytest.raises(ValueError):
        batch.batch_step([[.1]*4, [.1, .1, .1, float('nan')]], .02)
    assert first.truth_state()['t'] == second.truth_state()['t'] == 0
    _, events = batch.batch_step([[0]*4, [0]*4], .02)
    assert batch.last_backend == 'reference'
    assert events[0]['terminal'] and not events[1]['terminal']
    with pytest.raises(RuntimeError):
        batch.batch_step([[0]*4, [0]*4], .01)
    previous = first.truth_state()['t']
    _, events = batch.batch_step([[0]*4, [0]*4], .01, active=[False, True])
    assert events[0] is None and first.truth_state()['t'] == previous
    assert second.truth_state()['t'] == pytest.approx(.03)


def test_rejects_shared_mutable_state():
    env = RmuaPhysics(params())
    with pytest.raises(ValueError, match='distinct'):
        BatchRmuaPhysics([env, env])
    # Mutable counter sharing would otherwise mix force-zone histories.
    counter = OutTimeCounter(.5)
    a, b = RmuaPhysics(params()), RmuaPhysics(params())
    a.sim.out_time_counter = b.sim.out_time_counter = counter
    with pytest.raises(ValueError, match='counters'):
        BatchRmuaPhysics([a, b])


def test_vectorized_matches_reference_with_heterogeneous_state_and_parameters():
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(91)
    parameters = [Params(.7 + .1*i, (.003+i*.0001, .006, .01), .16+.01*i,
                         11.+i, .14+i*.01, .005+.001*i,
                         linear_drag=.03*i, angular_drag=.001*i) for i in range(4)]
    fast = BatchRmuaPhysics([RmuaPhysics(p, physics_dt=.0025) for p in parameters])
    reference = BatchRmuaPhysics([RmuaPhysics(p, physics_dt=.0025) for p in parameters], backend='reference')
    initial = []
    for _ in parameters:
        q = Rotation.from_rotvec(rng.uniform(-.3, .3, 3)).as_quat()[[3, 0, 1, 2]]
        initial.append({'position': rng.uniform(-1, 1, 3), 'velocity': rng.uniform(-1, 1, 3),
                        'quaternion_wxyz': q, 'omega': rng.uniform(-1, 1, 3),
                        'rotor_signal': rng.uniform(.1, .3, 4)})
    fast.reset(initial); reference.reset(initial)
    for index in range(40):
        pwm = rng.uniform(.1, .3, (4, 4))
        mask = [True, index % 2 == 0, True, True]
        obs_a, events_a = fast.batch_step(pwm, .01, active=mask)
        obs_b, events_b = reference.batch_step(pwm, .01, active=mask)
        assert fast.last_backend == 'vectorized_free_flight'
        assert events_a == events_b
        for i in range(4):
            assert obs_a[i]['time_s'] == pytest.approx(obs_b[i]['time_s'], abs=1e-13)
            for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
                np.testing.assert_allclose(fast.truth_state(i)[key], reference.truth_state(i)[key], atol=1e-12, rtol=1e-12)


def test_recorded_effects_use_reference_with_independent_counters():
    effects = RecordedEffects([0., .1], None, [0, 0])
    envs = [RmuaPhysics(params(), physics_dt=.01, recorded_effects=effects, field_rotor_target_scale=.25,
                        out_time_counter=OutTimeCounter(.5)) for _ in range(2)]
    batch = BatchRmuaPhysics(envs)
    batch.reset()
    _, events = batch.batch_step([[.2]*4, [.2]*4], .02)
    assert batch.last_backend == 'reference'
    assert [e['force_zone_elapsed_s'] for e in events] == pytest.approx([.02, .02])


def test_flight_bounds_use_reference_and_terminate_only_exiting_environment():
    bounds = [[-1, -1, -1], [1, 1, 1]]
    envs = [RmuaPhysics(params(), physics_dt=.01, flight_bounds_ned_m=bounds) for _ in range(2)]
    batch = BatchRmuaPhysics(envs)
    batch.reset([{'velocity': [150, 0, 0]}, None])
    _, events = batch.batch_step([[0]*4, [0]*4], .01)
    assert batch.last_backend == 'reference'
    assert events[0]['out_of_bounds_candidate'] and events[0]['terminal']
    assert not events[1]['out_of_bounds_candidate'] and not events[1]['terminal']
