import numpy as np

from rmua.envs.sim_fast import OBS_DIM, SimFastConfig, SimFastVecEnv


def _waypoint_action(env):
    targets = [env.track.start_gate.center, env.map_data.centroid, env.track.end_gate.center]
    if not hasattr(env, "_wp"):
        env._wp = 0
    target = targets[env._wp]
    delta = target - env.pos[0]
    if np.linalg.norm(delta) < 3.0 and env._wp < 2:
        env._wp += 1
        target = targets[env._wp]
        delta = target - env.pos[0]
    yaw = env.yaw[0]
    c, s = np.cos(yaw), np.sin(yaw)
    vx = delta[0] * c + delta[1] * s
    vy = -delta[0] * s + delta[1] * c
    horizontal = np.array([vx, vy])
    horizontal = horizontal / max(np.linalg.norm(horizontal), 1e-9) * 8.0
    desired_yaw = np.arctan2(delta[1], delta[0])
    yaw_rate = np.clip((desired_yaw - yaw + np.pi) % (2 * np.pi) - np.pi, -1.5, 1.5)
    return np.array([[horizontal[0], horizontal[1], np.clip(delta[2] * 1.5, -3.0, 3.0), yaw_rate]])


def test_reset_step_shapes():
    env = SimFastVecEnv(SimFastConfig(num_envs=8, segment_id=1, dynamics="kinematic"))
    obs = env.reset()
    assert obs.shape == (8, OBS_DIM)
    obs, reward, terminated, truncated, info = env.step(np.zeros((8, 4)))
    assert obs.shape == (8, OBS_DIM)
    assert reward.shape == (8,)
    assert terminated.shape == (8,)
    assert "gates_passed" in info


def test_same_seed_same_reset():
    cfg = SimFastConfig(num_envs=2, segment_id=1, dynamics="kinematic", seed=123)
    env_a = SimFastVecEnv(cfg)
    env_b = SimFastVecEnv(cfg)
    assert np.allclose(env_a.reset(), env_b.reset())
    assert np.allclose(env_a.pos, env_b.pos)


def test_different_seed_changes_obstacles():
    env_a = SimFastVecEnv(SimFastConfig(num_envs=1, segment_id=4, dynamics="kinematic", seed=1))
    env_b = SimFastVecEnv(SimFastConfig(num_envs=1, segment_id=4, dynamics="kinematic", seed=2))
    env_a.reset()
    env_b.reset()
    assert not np.allclose(env_a.lines.start, env_b.lines.start)


def test_start_timeout():
    env = SimFastVecEnv(SimFastConfig(num_envs=1, segment_id=1, dynamics="kinematic"))
    env.reset()
    done = False
    for _ in range(3005):
        _, _, terminated, truncated, info = env.step(np.zeros((1, 4)))
        if terminated[0] or truncated[0]:
            done = True
            break
    assert done
    assert info["failure_reason"][0] == "timeout"


def test_kinematic_segment_success():
    env = SimFastVecEnv(
        SimFastConfig(num_envs=1, segment_id=1, dynamics="kinematic", action_mode="vel")
    )
    env.reset()
    done = False
    for _ in range(2500):
        _, _, terminated, truncated, info = env.step(_waypoint_action(env))
        if terminated[0] or truncated[0]:
            done = True
            break
    assert done
    assert bool(info["success"][0])
    assert info["gates_passed"][0] == 2


def test_quad_hover_smoke():
    env = SimFastVecEnv(
        SimFastConfig(num_envs=2, segment_id=1, dynamics="quad", action_mode="pwm")
    )
    env.reset()
    pwm = np.full((2, 4), env.quad.params.hover_pwm)
    start_z = env.pos[:, 2].copy()
    for _ in range(100):
        _, _, terminated, _, _ = env.step(pwm)
    assert not np.any(terminated)
    assert np.allclose(env.pos[:, 2], start_z, atol=0.05)
