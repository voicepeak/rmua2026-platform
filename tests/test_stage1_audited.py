import numpy as np
import pytest

from rmua.envs.sim_fast import (
    PHASE_DONE,
    PHASE_FLYING,
    PHASE_WAIT_START,
    SimFastConfig,
    SimFastVecEnv,
)


def make_env(n=1):
    return SimFastVecEnv(SimFastConfig(num_envs=n, track_mode='stage1_audited',
        action_frame='world', line_max=0, vehicle_max=0))


def test_stage1_route_and_captured_gate_order():
    env = make_env()
    gates = env.track.scoring_gates
    assert len(gates) == 63
    assert gates[0].gate.name == 'road1_components.json:32'
    assert gates[32].gate.name == 'road1_components.json:0'
    assert gates[33].gate.name == 'road3_components.json:0'
    assert gates[-1].gate.name == 'road3_components.json:29'
    assert np.allclose(env.track.start_position, [0, 0, 0])
    assert np.allclose(env.track.end_position, [545.55995057, 519.85475477, 30.21285629])
    assert 1300 < env.track.total_length_m < 1700
    assert np.max([np.min(np.linalg.norm(env.track.path-g.center, axis=1)) for g in gates]) < 1.0


def test_ordered_gate_during_start_and_no_out_of_order_score():
    env = make_env(2)
    first, second = env.track.scoring_gates[:2]
    for i, gate in enumerate((first, second)):
        env.pos[i] = gate.center - gate.normal * 0.04
        env.vel[i] = gate.normal * 10
    env.phase[:] = PHASE_WAIT_START
    action = np.zeros((2, 4))
    action[0, :3] = first.normal * 10
    action[1, :3] = second.normal * 10
    env.step(action)
    assert env.gates_passed.tolist() == [1, 0]
    assert env.gate_index.tolist() == [1, 0]


def test_end_requires_all_gates():
    env = make_env(2)
    end = env.track.end_gate
    env.phase[:] = PHASE_FLYING
    env.gate_index[:] = [62, 63]
    env.gates_passed[:] = [62, 63]
    env.pos[:] = end.center - end.normal * (end.trigger_depth_m / 2 + 0.04)
    env.vel[:] = end.normal * 10
    action = np.zeros((2, 4)); action[:, :3] = end.normal * 10
    _, _, _, _, info = env.step(action)
    assert info['success'].tolist() == [False, True]


def test_terminal_state_is_immutable_after_later_collision():
    env = make_env(2)
    env.phase[:] = PHASE_DONE
    env.failure_reason[:] = "success"
    env.gate_index[:] = 63
    env.gates_passed[:] = 63
    env.pos[0] = env.track.path[-1] + np.array([0.0, 0.0, -10.0])
    env.pos[1] = env.track.path[-1]
    _, _, terminated, _, info = env.step(np.zeros((2, 4)))
    assert info["success"].tolist() == [True, True]
    assert info["failure_reason"].tolist() == ["success", "success"]
    assert terminated.tolist() == [False, False]


def test_audited_mode_rejects_unverified_map_and_segment():
    with pytest.raises(ValueError):
        SimFastVecEnv(SimFastConfig(track_mode='stage1_audited', segment_id=2))
    with pytest.raises(ValueError):
        SimFastVecEnv(SimFastConfig(track_mode='stage1_audited', map_name='RMUA2024OL2'))


def test_obstacles_are_opt_in_and_partial_reset_works():
    env = SimFastVecEnv(SimFastConfig(num_envs=2, track_mode='stage1_audited'))
    assert env.lines.max_count == env.vehicles.max_count == 0
    env.reset_indices([0])
    with_obstacles = SimFastVecEnv(SimFastConfig(num_envs=2, track_mode='stage1_audited',
        stage1_enable_obstacles=True, line_max=2, vehicle_max=1))
    with_obstacles.reset_indices([1])
    assert with_obstacles.lines.max_count == 2
    assert with_obstacles.vehicles.max_count == 1
