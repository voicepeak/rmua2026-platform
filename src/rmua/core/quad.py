"""Vectorized 6-DOF quad plant for sim-fast.

This is a deliberately small plant: rigid body, four rotors, first-order motor
lag, linear drag and a wind force term. It is not a claim of AirSim/PhysX
equivalence. Use it for low-level policy prototyping and system-id scaffolding;
the official simulator remains the calibration source.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .constants import GRAVITY, QuadParams
from . import quat as quat_utils


def _as_batch(value, n, width):
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim == 1:
        arr = np.broadcast_to(arr, (n, width)).copy()
    if arr.shape != (n, width):
        raise ValueError(f"expected ({n}, {width}), got {arr.shape}")
    return arr


@dataclass
class QuadState:
    pos: np.ndarray
    vel: np.ndarray
    quat: np.ndarray
    omega: np.ndarray
    rpm: np.ndarray


class QuadBatch:
    """A batch of identical quadrotors."""

    def __init__(self, num_envs: int, params: QuadParams | None = None):
        self.n = int(num_envs)
        self.params = params or QuadParams()
        self.pos = np.zeros((self.n, 3), dtype=np.float64)
        self.vel = np.zeros((self.n, 3), dtype=np.float64)
        self.quat = quat_utils.identity((self.n,))
        self.omega = np.zeros((self.n, 3), dtype=np.float64)
        self.rpm = np.full((self.n, 4), self.params.max_rpm * self.params.hover_pwm)

        d = self.params.arm_length_m / np.sqrt(2.0)
        # Official order: 0 right_front, 1 left_rear, 2 left_front, 3 right_rear.
        self.motor_pos = np.array(
            [
                [d, d, 0.0],
                [-d, -d, 0.0],
                [d, -d, 0.0],
                [-d, d, 0.0],
            ],
            dtype=np.float64,
        )
        self.motor_spin = np.array([1.0, 1.0, -1.0, -1.0], dtype=np.float64)
        self.inertia = np.array(
            [self.params.ixx, self.params.iyy, self.params.izz], dtype=np.float64
        )

    # ------------------------------------------------------------------
    def reset(self, pos=None, vel=None, quat=None, omega=None, rpm=None):
        if pos is None:
            pos = np.zeros((self.n, 3))
        if vel is None:
            vel = np.zeros((self.n, 3))
        if quat is None:
            quat = quat_utils.identity((self.n,))
        if omega is None:
            omega = np.zeros((self.n, 3))
        if rpm is None:
            rpm = np.full((self.n, 4), self.params.max_rpm * self.params.hover_pwm)
        self.pos = _as_batch(pos, self.n, 3)
        self.vel = _as_batch(vel, self.n, 3)
        self.quat = quat_utils.normalize(_as_batch(quat, self.n, 4))
        self.omega = _as_batch(omega, self.n, 3)
        self.rpm = np.clip(_as_batch(rpm, self.n, 4), 0.0, self.params.max_rpm)
        return self.state()

    def state(self) -> QuadState:
        return QuadState(
            self.pos.copy(),
            self.vel.copy(),
            self.quat.copy(),
            self.omega.copy(),
            self.rpm.copy(),
        )

    def body_velocity(self):
        return quat_utils.rotate_inverse(self.quat, self.vel)

    def body_z(self):
        return quat_utils.body_z(self.quat)

    def hover_pwm(self):
        return self.params.hover_pwm

    # ------------------------------------------------------------------
    def step(self, pwm, wind=None, dt: float = 1.0 / 400.0):
        """Advance the plant by dt with a constant PWM command."""
        pwm = np.clip(np.asarray(pwm, dtype=np.float64), 0.0, 1.0)
        if pwm.shape != (self.n, 4):
            raise ValueError(f"pwm must be ({self.n}, 4), got {pwm.shape}")
        if wind is None:
            wind = np.zeros((self.n, 3))
        wind = _as_batch(wind, self.n, 3)

        p = self.params
        target_rpm = pwm * p.max_rpm
        alpha = min(1.0, dt / max(p.motor_tau_s, 1e-6))
        self.rpm += (target_rpm - self.rpm) * alpha
        self.rpm = np.clip(self.rpm, 0.0, p.max_rpm)

        # Thrust per motor, normalized so full PWM == max thrust.
        thrust = p.max_thrust_per_motor_n * (self.rpm / p.max_rpm) ** 2

        # Body force and torque.
        body_force = np.zeros((self.n, 3), dtype=np.float64)
        body_force[:, 2] = np.sum(thrust, axis=1)
        torque = np.zeros((self.n, 3), dtype=np.float64)
        for index in range(4):
            r = self.motor_pos[index]
            t = thrust[:, index]
            torque[:, 0] += r[1] * t
            torque[:, 1] += -r[0] * t
            torque[:, 2] += -self.motor_spin[index] * p.rotor_torque_ratio_m * t

        world_force = quat_utils.rotate(self.quat, body_force)
        world_force[:, 2] -= p.mass_kg * GRAVITY
        world_force += p.wind_force_coeff * (wind - self.vel)
        world_force -= p.linear_drag * self.vel

        acc = world_force / p.mass_kg
        self.vel += acc * dt
        self.pos += self.vel * dt

        # Euler's rigid-body equation in the body frame.
        omega = self.omega
        gyro = np.cross(omega, self.inertia * omega)
        omega_dot = (torque - gyro) / self.inertia
        omega_dot -= p.angular_damping * omega
        self.omega += omega_dot * dt
        self.quat = quat_utils.integrate(self.quat, self.omega, dt)

        return self.state()

    def step_substeps(self, pwm, wind=None, dt: float = 1.0 / 100.0, substeps: int = 4):
        for _ in range(int(substeps)):
            self.step(pwm, wind=wind, dt=dt / substeps)
        return self.state()
