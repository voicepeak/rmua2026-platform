import numpy as np

from rmua.envs.sim_fast import OBS_DIM, SimFastConfig, SimFastVecEnv


def _waypoint_action(env):
    distance, _, _, _, _ = env.track.project(env.pos)
    ahead = np.minimum(distance[0] + 8.0, env.track.total_length_m)
    target = env.track.point_at_distance(ahead)
    tangent_now = env.track.tangent_at_distance(distance[0])
    tangent_ahead = env.track.tangent_at_distance(
        min(distance[0] + 25.0, env.track.total_length_m)
    )
    turn_angle = np.arccos(np.clip(np.dot(tangent_now, tangent_ahead), -1.0, 1.0))
    turn_factor = max(0.55, min(1.0, 1.0 - turn_angle / 1.6))
    delta = target - env.pos[0]
    length = max(np.linalg.norm(delta), 1e-6)
    speed = min(20.0, length * 3.0) * turn_factor
    velocity = delta / length * speed
    return np.array([[velocity[0], velocity[1], velocity[2], 0.0]])


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
        SimFastConfig(
            num_envs=1,
            segment_id=1,
            dynamics="kinematic",
            action_mode="vel",
            action_frame="world",
            line_max=0,
            vehicle_max=0,
        )
    )
    env.reset()
    done = False
    for _ in range(20000):
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
