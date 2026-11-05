"""Action contract for the official VEL/PWM interfaces."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ActionSpec:
    mode: str = "vel"  # vel | pwm
    dim: int = 4
    accel_limit_mps2: float = 8.0
    pwm_order: tuple[str, ...] = (
        "right_front",
        "left_rear",
        "left_front",
        "right_rear",
    )

    def clip(self, action):
        arr = np.asarray(action, dtype=np.float64)
        if arr.shape[-1] != self.dim:
            raise ValueError(f"expected last dim {self.dim}, got {arr.shape}")
        if self.mode == "vel":
            arr = arr.copy()
            arr[..., :3] = np.clip(arr[..., :3], -12.0, 12.0)
            arr[..., 3] = np.clip(arr[..., 3], -2.5, 2.5)
        else:
            arr = np.clip(arr, 0.0, 1.0)
        return arr

    def to_vel_command(self, action):
        """Official vel_body_cmd fields excluding acceleration/emergency stop."""
        if self.mode != "vel":
            raise ValueError("action mode is not vel")
        return self.clip(action)

    def to_pwm_command(self, action):
        if self.mode != "pwm":
            raise ValueError("action mode is not pwm")
        return self.clip(action)


DEFAULT_VEL_ACTION = ActionSpec(mode="vel", dim=4)
DEFAULT_PWM_ACTION = ActionSpec(mode="pwm", dim=4)
