import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from foundation.aero import DragFaces
from foundation.quad import Params
from foundation.sim import PhysicsSim


def test_relative_velocity_rotation_and_torque():
    model = DragFaces([[0, 1, 0]], [[1, 0, 0]], [.5], 2., .1)
    force, torque = model.wrench(Rotation.identity(), [2, 0, 0], [0, 0, 0], [0, 0, 0])
    np.testing.assert_allclose(force, [-4, 0, 0])
    np.testing.assert_allclose(torque, [0, 0, 4])
    force2, torque2 = model.wrench(Rotation.from_euler('z', 90, degrees=True), [0, 5, 0], [0, 0, 0], [0, 3, 0])
    np.testing.assert_allclose(force2, [0, -4, 0], atol=1e-12)
    np.testing.assert_allclose(torque2, torque, atol=1e-12)
    zero, _ = model.wrench(Rotation.identity(), [1, 0, 0], [0, 0, 0], [1, 0, 0])
    np.testing.assert_allclose(zero, 0)


def test_spin_and_explicit_simulator_wind():
    model = DragFaces([[0, 1, 0]], [[1, 0, 0]], [1.], 1., 0.)
    force, _ = model.wrench(Rotation.identity(), [0, 0, 0], [0, 0, -2], [0, 0, 0])
    np.testing.assert_allclose(force, [-4, 0, 0])
    params = Params(.9, (.0047, .0069, .0104), .18, 12., .16, .01)
    sim = PhysicsSim(params, aero_model=model)
    sim.reset()
    state, info = sim.step([0]*4, .01, wind_world_m_s=[-2, 0, 0])
    assert state['velocity'][0] < 0
    assert info['wind_force'] == 'uncalibrated_drag_face_candidate'
    with pytest.raises(ValueError, match='requires'):
        PhysicsSim(params).step([0]*4, .01, wind_world_m_s=[1, 0, 0])


def test_observed_face_snapshot_requires_initialized_state():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1] / 'reports'
    model = DragFaces.from_observed_snapshot(root/'drag_faces_seed456.json', air_density=1.2, min_velocity=.1)
    assert len(model.positions) == 6
    with pytest.raises(ValueError, match='initialized'):
        DragFaces.from_observed_snapshot(root/'drag_faces_seed123.json', air_density=1.2, min_velocity=.1)
