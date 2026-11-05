"""Deployment safety layer.

The safety layer is rule-based, not learned. It can veto or override a policy
action before the ROS command is published. Node 1 only defines the contract;
the ROS adapter is wired in P4.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class FirstSignalLock:
    """Official rule: the first received control signal type locks the interface."""

    def __init__(self):
        self.mode: str | None = None

    def accept(self, mode: str) -> bool:
        if self.mode is None:
            self.mode = mode
            return True
        return self.mode == mode

    def reset(self):
        self.mode = None


@dataclass
class SafetyDecision:
    allow: bool
    action: np.ndarray
    reason: str = ""


class SafetyLayer:
    def __init__(self, force_field_limit_s=2.0, max_tilt_rad=1.0):
        self.force_field_limit_s = float(force_field_limit_s)
        self.max_tilt_rad = float(max_tilt_rad)

    def filter(self, action, info: dict | None = None, mode: str = "vel"):
        action = np.asarray(action, dtype=np.float64).copy()
        info = info or {}
        if np.any(~np.isfinite(action)):
            return SafetyDecision(False, np.zeros_like(action), "non_finite_action")
        if info.get("force_time", 0.0) > self.force_field_limit_s:
            return SafetyDecision(False, np.zeros_like(action), "force_field")
        if info.get("collision", False):
            return SafetyDecision(False, np.zeros_like(action), "collision")
        if mode == "vel":
            action[..., :3] = np.clip(action[..., :3], -12.0, 12.0)
            action[..., 3] = np.clip(action[..., 3], -2.5, 2.5)
        else:
            action = np.clip(action, 0.0, 1.0)
        return SafetyDecision(True, action, "")
