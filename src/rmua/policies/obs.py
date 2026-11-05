"""Observation contract shared by training and deployment."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..envs.sim_fast import OBS_DIM, OBS_NAMES


@dataclass(frozen=True)
class ObsSpec:
    names: tuple[str, ...] = OBS_NAMES
    dim: int = OBS_DIM
    dtype: str = "float32"

    def validate(self, obs) -> np.ndarray:
        arr = np.asarray(obs, dtype=np.float32)
        if arr.shape[-1] != self.dim:
            raise ValueError(f"expected last dim {self.dim}, got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError("observation contains NaN/Inf")
        return arr


DEFAULT_OBS_SPEC = ObsSpec()


def actor_observation(obs_truth, gps_noise_m=0.0, seed=None):
    """Placeholder for the deployable actor observation.

    Node 1 uses the state-only observation directly. The deployment adapter must
    replace position/velocity with EKF estimates and add sensor noise/dropout;
    never ship `pose_gt` as an actor input.
    """
    arr = np.asarray(obs_truth, dtype=np.float32).copy()
    if gps_noise_m > 0.0:
        rng = np.random.default_rng(seed) if seed is not None else np.random.default_rng()
        arr[..., :3] += rng.normal(0.0, gps_noise_m, size=arr[..., :3].shape).astype(np.float32)
    return arr
