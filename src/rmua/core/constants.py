"""Physical constants and sim-fast defaults."""

from __future__ import annotations

from dataclasses import dataclass

GRAVITY = 9.80665


@dataclass(frozen=True)
class QuadParams:
    mass_kg: float = 0.9
    arm_length_m: float = 0.18
    ixx: float = 0.0046890742
    iyy: float = 0.0069312
    izz: float = 0.010421166
    motor_lift_coeff: float = 0.000367717
    motor_torque_coeff: float = 4.888486266072161e-06
    max_rpm: float = 11079.03

    # Sim-fast calibrated values. Official coefficients are kept above for
    # traceability; these are the values the state-only plant uses until
    # system identification against the official simulator is complete.
    max_thrust_per_motor_n: float = 8.829 / 0.25 / 4.0
    rotor_torque_ratio_m: float = 4.888486266072161e-06 / 0.000367717
    motor_tau_s: float = 0.035
    linear_drag: float = 0.08
    angular_damping: float = 0.02
    wind_force_coeff: float = 0.12

    @property
    def gravity_force_n(self):
        return self.mass_kg * GRAVITY

    @property
    def hover_thrust_per_motor_n(self):
        return self.gravity_force_n / 4.0

    @property
    def hover_pwm(self):
        return (self.hover_thrust_per_motor_n / self.max_thrust_per_motor_n) ** 0.5


DEFAULT_CONTROL_HZ = 100.0
DEFAULT_PHYSICS_HZ = 400.0
DEFAULT_SIM_DT = 1.0 / DEFAULT_CONTROL_HZ
DEFAULT_PHYSICS_DT = 1.0 / DEFAULT_PHYSICS_HZ
