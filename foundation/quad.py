"""Experimental NED/FRD PWM rigid-body candidate, not an official replay.

The linear filtered-command thrust relationship follows upstream AirSim's
RotorActuator, whose exact relationship to the shipped binary is unverified.
Parameters, particularly motor lag and coefficient units, must be calibrated.
No collision, aerodynamic torque or wind force is implied by this model.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Params:
    mass: float
    inertia: tuple
    arm_radius: float
    max_thrust: float
    max_reaction_torque: float
    motor_tau: float
    gravity: float = 9.80665
    linear_drag: float = 0.0
    angular_drag: float = 0.0


class Quad:
    def __init__(self, params: Params):
        self.p = params
        inertia = np.asarray(params.inertia, dtype=float)
        if (inertia.shape != (3,) or not np.all(np.isfinite(inertia)) or np.any(inertia <= 0)
            or any(not np.isfinite(x) or x <= 0 for x in
                   (params.mass, params.arm_radius, params.max_thrust, params.motor_tau, params.gravity))
            or any(not np.isfinite(x) or x < 0 for x in
                   (params.max_reaction_torque, params.linear_drag, params.angular_drag))):
            raise ValueError("invalid physical parameters")
        self.inertia = inertia
        d = params.arm_radius / np.sqrt(2)
        self.arms = np.array([[d, d, 0], [-d, -d, 0], [d, -d, 0], [-d, d, 0]])
        # Candidate signs supported by a historical pulse summary, not verified
        # against a fresh run. FRD torque is expressed about body Z down.
        self.yaw_sign = np.array([1., 1., -1., -1.])
        self.reset()

    def reset(self, position=(0, 0, 0), velocity=(0, 0, 0),
              quaternion_wxyz=(1, 0, 0, 0), omega=(0, 0, 0), rotor_signal=(0, 0, 0, 0)):
        def vector(value, n):
            x = np.asarray(value, dtype=float)
            if x.shape != (n,) or not np.all(np.isfinite(x)):
                raise ValueError("invalid state")
            return x.copy()
        self.position = vector(position, 3)
        self.velocity = vector(velocity, 3)
        self.omega = vector(omega, 3)
        q = vector(quaternion_wxyz, 4)
        if np.linalg.norm(q) < 1e-12:
            raise ValueError("zero quaternion")
        self.rotation = Rotation.from_quat(q[[1, 2, 3, 0]])
        self.signal = vector(rotor_signal, 4)
        if np.any((self.signal < 0) | (self.signal > 1)):
            raise ValueError("rotor signal outside candidate range")
        self.time = 0.
        return self.state()

    def state(self):
        q = self.rotation.as_quat()
        return {"t": self.time, "position": self.position.copy(), "velocity": self.velocity.copy(),
                "quaternion_wxyz": q[[3, 0, 1, 2]], "omega": self.omega.copy(),
                "rotor_signal": self.signal.copy()}

    def step(self, pwm, dt, external_force_world=(0, 0, 0), external_torque_body=(0, 0, 0),
             rotor_target_scale=1.0):
        pwm = np.asarray(pwm, dtype=float)
        ef, et = np.asarray(external_force_world, dtype=float), np.asarray(external_torque_body, dtype=float)
        if (pwm.shape != (4,) or not np.all(np.isfinite(pwm)) or np.any((pwm < 0) | (pwm > 1))
            or not np.isfinite(dt) or dt <= 0 or ef.shape != (3,) or et.shape != (3,)
            or not np.all(np.isfinite(ef)) or not np.all(np.isfinite(et))
            or not np.isfinite(rotor_target_scale) or not 0 <= rotor_target_scale <= 1):
            raise ValueError("invalid step input")
        # The observed official rotor update computes output from the current
        # filter state and then updates that state for the next rotor update.
        # The correspondence between our dt and its clock remains unverified.
        thrust = self.signal * self.p.max_thrust
        forces = np.column_stack((np.zeros(4), np.zeros(4), -thrust))
        torque = np.cross(self.arms, forces).sum(axis=0)
        torque[2] += np.dot(self.signal, self.yaw_sign) * self.p.max_reaction_torque
        force = self.rotation.apply(forces.sum(axis=0)) + ef
        force += np.array([0, 0, self.p.mass * self.p.gravity]) - self.p.linear_drag * self.velocity
        self.velocity += force / self.p.mass * dt
        self.position += self.velocity * dt
        domega = (torque + et - np.cross(self.omega, self.inertia * self.omega)
                  - self.p.angular_drag * self.omega) / self.inertia
        self.omega += domega * dt
        self.rotation = self.rotation * Rotation.from_rotvec(self.omega * dt)
        self.signal += (pwm * rotor_target_scale - self.signal) * (-np.expm1(-dt / self.p.motor_tau))
        self.time += dt
        return self.state()
