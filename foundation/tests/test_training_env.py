import numpy as np
import pytest

from foundation.quad import Params
from foundation.training_env import RmuaPhysicsVecEnv


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_observations_actions_and_seed_isolation():
    first = RmuaPhysicsVecEnv(params(), 3, seed=7, physics_dt=.01, command_dt=.02)
    second = RmuaPhysicsVecEnv(params(), 3, seed=7, physics_dt=.01, command_dt=.02)
    third = RmuaPhysicsVecEnv(params(), 3, seed=8, physics_dt=.01, command_dt=.02)
    obs_a, obs_b, obs_c = first.reset(), second.reset(), third.reset()
    assert obs_a.shape == (3, 22)
    np.testing.assert_allclose(obs_a, obs_b)
    assert not np.allclose(obs_a[0][:3], obs_c[0][:3])
    action = np.full((3, 4), .18)
    step_a = first.step(action)
    step_b = second.step(action)
    np.testing.assert_allclose(step_a[0], step_b[0])
    np.testing.assert_allclose(step_a[1], step_b[1])
    assert step_a[0].shape == (3, 22)
    truth = first.truth_state(0)
    assert np.linalg.norm(truth['quaternion_wxyz']) == pytest.approx(1.0, abs=1e-12)
    assert truth['t'] == pytest.approx(.02)


def test_pwm_validation_and_terminal_masking():
    env = RmuaPhysicsVecEnv(params(), 2, physics_dt=.01, command_dt=.02)
    env.reset()
    with pytest.raises(ValueError):
        env.step([[.1, .1, .1, float('nan')], [.1] * 4])
    with pytest.raises(ValueError):
        env.step([[.1, .1, .1, 1.1], [.1] * 4])
    before = env.truth_state(0)['t']
    obs, reward, done, truncated, reached, infos = env.step(np.full((2, 4), .18))
    assert before == 0 and env.truth_state(0)['t'] == pytest.approx(.02)
    assert len(infos) == 2 and infos[0]['approximate_sandbox']
    assert env.observable_state() is None


def test_gate_progress_reward_and_index_advance():
    gates = np.array([[0.0, 0.0, -2.0], [0.0, 0.0, -3.0]])
    env = RmuaPhysicsVecEnv(params(), 1, gates=gates, gate_radius_m=.5,
                            physics_dt=.01, command_dt=.02)
    env.reset()
    _, reward, _, _, reached, info = env.step(np.full((1, 4), .18))
    assert reached[0] and info[0]['gate_reached']
    assert info[0]['gate_index'] == 1
    assert reward[0] >= 1.0


def test_timeout_truncation_and_reset_indices():
    env = RmuaPhysicsVecEnv(params(), 2, physics_dt=.01, command_dt=.02, timeout_s=.04)
    env.reset()
    done = truncated = None
    for _ in range(2):
        _, _, done, truncated, _, _ = env.step(np.full((2, 4), .18))
    assert np.all(done | truncated)
    before = env.truth_state(0)['t']
    env.reset(indices=[0])
    assert env.truth_state(0)['t'] == 0
    assert env.truth_state(1)['t'] == before
