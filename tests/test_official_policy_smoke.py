import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location('official_policy_smoke',
    Path(__file__).resolve().parents[1] / 'scripts' / 'official_policy_smoke.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_world_up_to_body_command():
    assert np.allclose(bridge.world_up_to_body_command([1, 0, 2], 0), [1, 0, 2])
    assert np.allclose(bridge.world_up_to_body_command([1, 0, 2], np.pi/2), [0, -1, 2])


def test_numpy_actor_matches_torch():
    import torch
    torch.manual_seed(1)
    model = torch.nn.Sequential(torch.nn.Linear(33, 8), torch.nn.Tanh(),
        torch.nn.Linear(8, 8), torch.nn.Tanh(), torch.nn.Linear(8, 4))
    weights = {'actor.'+k: v.detach().numpy() for k, v in model.state_dict().items()}
    weights['obs_scale'] = np.linspace(1, 2, 33, dtype=np.float32)
    obs = np.linspace(-1, 1, 33, dtype=np.float32)
    expected = model(torch.tensor(obs / weights['obs_scale'])).detach().numpy()
    assert np.allclose(bridge.actor_forward(obs, weights), expected, atol=1e-6)


def test_state_monitor_fails_closed_and_departure_is_bounded(tmp_path):
    path = tmp_path / 'state.json'
    assert bridge.read_official_state(path, 10.) is None
    path.write_text('{"monotonic": 9.5, "state": "State1", "score": 1}')
    assert bridge.read_official_state(path, 10.)['state'] == 'State1'
    assert bridge.read_official_state(path, 12.) is None
    assert bridge.read_official_state(path, 9.) is None
    assert np.allclose(bridge.departure_command([0, 0, 0], 10), [0, 0, .8])
    assert np.allclose(bridge.departure_command([0, 0, 1.8], 10), [2, 0, 0])
    assert bridge.departure_command([0, 0, 5.], 10)[2] == -.6


def test_guarded_horizontal_limits_speed_and_acceleration():
    a = bridge.limited_planar_velocity([20., 0.], [2., 0.], .1)
    assert np.allclose(a, [2.1, 0.])
    b = bridge.limited_planar_velocity([20., 0.], [4., 0.], .1)
    assert np.allclose(b, [4., 0.])
    c = bridge.limited_planar_velocity([0., 20.], [0., 0.], .1)
    assert np.allclose(c, [0., .1])


def test_guarded_vertical_limits_climb_and_altitude():
    assert bridge.guarded_vertical_velocity(5., 1.8) == .4
    assert bridge.guarded_vertical_velocity(5., 3.0) == 0.
    assert bridge.guarded_vertical_velocity(-5., 1.4) == 0.
    assert bridge.guarded_vertical_velocity(5., 3.5) < 0.
    assert np.isclose(bridge.road_vertical_velocity(5., 4., 4.), .64)
    assert bridge.road_vertical_velocity(5., 4.8, 4.) <= 1e-12
    assert bridge.road_vertical_velocity(-5., 3.2, 4.) >= -1e-12
