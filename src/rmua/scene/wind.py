"""Simple seed-driven wind for sim-fast.

Not the byte-exact official generator. The official generator is available in
vendor/rmua_re/wind.py and should be calibrated on the server. This model keeps
the key curriculum properties: only outdoor stages have wind, direction/speed
vary with time, and the seed changes the realisation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

STAGE_WIND = {
    4: {"base": 1.5, "amplitude": 0.5, "period": 18.0},
    5: {"base": 2.0, "amplitude": 1.0, "period": 20.0},
    6: {"base": 4.0, "amplitude": 2.0, "period": 24.0},
}


@dataclass
class WindField:
    enabled: bool
    speed: np.ndarray  # (N,)
    direction: np.ndarray  # (N,)
    amplitude: np.ndarray  # (N,)
    period: np.ndarray  # (N,)
    phase: np.ndarray  # (N,)

    @classmethod
    def create(cls, rngs: list[np.random.Generator], stage: int, enabled: bool):
        n = len(rngs)
        if not enabled:
            return cls(
                False,
                np.zeros(n),
                np.zeros(n),
                np.zeros(n),
                np.ones(n),
                np.zeros(n),
            )
        params = STAGE_WIND.get(stage, STAGE_WIND[4])
        speed = np.array([rng.uniform(0.6, 1.4) * params["base"] for rng in rngs])
        direction = np.array([rng.uniform(-np.pi, np.pi) for rng in rngs])
        amplitude = np.array([rng.uniform(0.3, 1.0) * params["amplitude"] for rng in rngs])
        period = np.array([rng.uniform(0.7, 1.3) * params["period"] for rng in rngs])
        phase = np.array([rng.uniform(0.0, 2.0 * np.pi) for rng in rngs])
        return cls(enabled, speed, direction, amplitude, period, phase)

    def sample(self, time_s, indoor_mask=None):
        n = self.speed.shape[0]
        if not self.enabled:
            return np.zeros((n, 3), dtype=np.float64)
        t = np.asarray(time_s, dtype=np.float64)
        if t.ndim == 0:
            t = np.full(n, float(t))
        speed = self.speed + self.amplitude * np.sin(2.0 * np.pi * t / self.period + self.phase)
        angle = self.direction + 0.15 * np.sin(2.0 * np.pi * t / (self.period * 1.7) + self.phase)
        wind = np.stack((speed * np.cos(angle), speed * np.sin(angle), np.zeros_like(speed)), axis=1)
        if indoor_mask is not None:
            wind = np.where(np.asarray(indoor_mask)[:, None], 0.0, wind)
        return wind
