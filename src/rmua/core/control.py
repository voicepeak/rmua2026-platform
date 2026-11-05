"""Simple velocity-to-PWM controller for the sim-fast quad.

This is a low-level tracking controller, not a learned policy. It exists so the
task-level env can accept the official 4-value body-velocity command while the
plant remains a 6-DOF rigid body. Replace or identify it on the server.
"""

from __future__ import annotations

import numpy as np

from .constants import GRAVITY, QuadParams
from . import quat as quat_utils


class VelocityController:
    def __init__(
        self,
        params: QuadParams | None = None,
        kp_velocity: float = 2.5,
        kp_attitude: float = 0.8,
        kd_attitude: float = 0.08,
        max_horizontal_accel: float = 8.0,
        max_vertical_accel: float = 4.0,
    ):
        self.params = params or QuadParams()
        self.kp_velocity = float(kp_velocity)
        self.kp_attitude = float(kp_attitude)
        self.kd_attitude = float(kd_attitude)
        self.max_horizontal_accel = float(max_horizontal_accel)
        self.max_vertical_accel = float(max_vertical_accel)

    def pwm(self, quad, v_des_body, yaw_rate_des, dt):
        p = self.params
        n = quad.n
        v_des = np.asarray(v_des_body, dtype=np.float64)
        if v_des.shape != (n, 3):
            raise ValueError(f"v_des_body must be ({n}, 3), got {v_des.shape}")
        yaw_rate = np.asarray(yaw_rate_des, dtype=np.float64).reshape(n)

        # Use the yaw frame for horizontal commands and world Z for altitude.
        # This keeps the simple tracker stable; the official vel_body_cmd is
        # body-frame and is handled by the deployment adapter.
        yaw, _, _ = quat_utils.to_euler(quad.quat)
        c = np.cos(yaw)
        s = np.sin(yaw)
        v_des_world = np.empty((n, 3), dtype=np.float64)
        v_des_world[:, 0] = v_des[:, 0] * c - v_des[:, 1] * s
        v_des_world[:, 1] = v_des[:, 0] * s + v_des[:, 1] * c
        v_des_world[:, 2] = v_des[:, 2]
        a_world = self.kp_velocity * (v_des_world - quad.vel)
        horizontal = a_world[:, :2]
        h_norm = np.linalg.norm(horizontal, axis=1, keepdims=True)
        h_scale = np.minimum(1.0, self.max_horizontal_accel / np.maximum(h_norm, 1e-9))
        a_world[:, :2] = horizontal * h_scale
        a_world[:, 2] = np.clip(a_world[:, 2], -self.max_vertical_accel, self.max_vertical_accel)

        # The thrust magnitude is the norm of the desired force, so a tilted
        # vehicle keeps its altitude while accelerating horizontally.
        force_world = p.mass_kg * (a_world + np.array([0.0, 0.0, GRAVITY]))
        force_norm = np.linalg.norm(force_world, axis=1, keepdims=True)
        zb_world = force_world / np.maximum(force_norm, 1e-6)

        yaw_des = yaw + yaw_rate * dt
        xf = np.stack((np.cos(yaw_des), np.sin(yaw_des), np.zeros(n)), axis=1)
        dot = np.sum(xf * zb_world, axis=1, keepdims=True)
        xb = xf - dot * zb_world
        xb_norm = np.linalg.norm(xb, axis=1, keepdims=True)
        # If the desired z is nearly horizontal, keep the current heading.
        fallback = xb_norm[:, 0] < 1e-6
        if np.any(fallback):
            xb[fallback] = quat_utils.rotate(
                quad.quat[fallback], np.array([1.0, 0.0, 0.0])
            )
            xb_norm = np.linalg.norm(xb, axis=1, keepdims=True)
        xb = xb / np.maximum(xb_norm, 1e-9)
        yb = np.cross(zb_world, xb)
        yb = yb / np.maximum(np.linalg.norm(yb, axis=1, keepdims=True), 1e-9)
        matrix = np.stack((xb, yb, zb_world), axis=2)
        q_des = quat_utils.from_matrix(matrix)

        q_err = quat_utils.multiply(quat_utils.conjugate(quad.quat), q_des)
        q_err = np.where(q_err[:, 0:1] < 0.0, -q_err, q_err)
        err_vec = 2.0 * q_err[:, 1:4]
        tau = self.kp_attitude * err_vec - self.kd_attitude * quad.omega

        d = p.arm_length_m / np.sqrt(2.0)
        base = np.sqrt(np.clip(force_norm[:, 0] / (4.0 * p.max_thrust_per_motor_n), 0.0, 1.0))
        base = np.maximum(base, 1e-3)
        denom_roll = 8.0 * d * p.max_thrust_per_motor_n * base
        roll_cmd = np.clip(tau[:, 0] / denom_roll, -0.45, 0.45)
        pitch_cmd = np.clip(tau[:, 1] / denom_roll, -0.45, 0.45)
        yaw_denom = 8.0 * max(p.rotor_torque_ratio_m, 1e-6) * p.max_thrust_per_motor_n * base
        yaw_cmd = np.clip(-tau[:, 2] / yaw_denom, -0.35, 0.35)

        pwm = np.empty((n, 4), dtype=np.float64)
        # 0 right_front, 1 left_rear, 2 left_front, 3 right_rear
        pwm[:, 0] = base - pitch_cmd + roll_cmd - yaw_cmd
        pwm[:, 1] = base + pitch_cmd - roll_cmd - yaw_cmd
        pwm[:, 2] = base - pitch_cmd - roll_cmd + yaw_cmd
        pwm[:, 3] = base + pitch_cmd + roll_cmd + yaw_cmd
        return np.clip(pwm, 0.0, 1.0)
