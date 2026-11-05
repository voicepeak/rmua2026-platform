import numpy as np

from rmua.core.constants import GRAVITY
from rmua.core.control import VelocityController
from rmua.core.quad import QuadBatch


def test_hover_holds_altitude():
    quad = QuadBatch(4)
    quad.reset(pos=np.tile([0.0, 0.0, 10.0], (4, 1)))
    pwm = np.full((4, 4), quad.hover_pwm())
    for _ in range(100):
        quad.step_substeps(pwm, dt=0.01, substeps=4)
    assert np.allclose(quad.pos[:, 2], 10.0, atol=0.02)
    assert np.allclose(quad.vel[:, 2], 0.0, atol=0.05)


def test_thrust_increases_with_pwm():
    quad = QuadBatch(2)
    quad.reset(pos=np.tile([0.0, 0.0, 10.0], (2, 1)))
    low = np.full((2, 4), 0.5)
    high = np.full((2, 4), 0.8)
    quad.step(low, dt=1 / 400)
    low_rpm = quad.rpm.copy()
    quad.reset(pos=np.tile([0.0, 0.0, 10.0], (2, 1)))
    quad.step(high, dt=1 / 400)
    assert np.all(quad.rpm > low_rpm)


def test_velocity_controller_tracks_forward_and_altitude():
    quad = QuadBatch(1)
    controller = VelocityController()
    quad.reset(pos=np.array([[0.0, 0.0, 10.0]]))
    v_des = np.array([[3.0, 0.0, 0.0]])
    for _ in range(300):
        pwm = controller.pwm(quad, v_des, np.zeros(1), 0.01)
        quad.step_substeps(pwm, dt=0.01, substeps=4)
    assert quad.vel[0, 0] > 2.0
    assert abs(quad.pos[0, 2] - 10.0) < 0.3


def test_gravity_is_used():
    assert GRAVITY > 9.7
