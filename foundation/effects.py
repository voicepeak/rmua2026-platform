"""Explicit measured-force replay; not an identified official wind model."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ForceReplay:
    times_s: np.ndarray
    forces_n: np.ndarray
    bounds_min: np.ndarray | None = None
    bounds_max: np.ndarray | None = None
    enabled: bool = True

    def __post_init__(self):
        t, f = np.asarray(self.times_s, float), np.asarray(self.forces_n, float)
        if (t.ndim != 1 or len(t) < 2 or f.shape != (len(t), 3) or not np.isfinite(t).all()
                or not np.isfinite(f).all() or np.any(np.diff(t) <= 0)):
            raise ValueError('force replay requires finite ordered samples')
        lo, hi = self.bounds_min, self.bounds_max
        if (lo is None) != (hi is None):
            raise ValueError('both spatial bounds must be supplied')
        if lo is not None:
            lo, hi = np.asarray(lo, float), np.asarray(hi, float)
            if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
                raise ValueError('invalid spatial bounds')
            object.__setattr__(self, 'bounds_min', lo.copy())
            object.__setattr__(self, 'bounds_max', hi.copy())
        object.__setattr__(self, 'times_s', t.copy())
        object.__setattr__(self, 'forces_n', f.copy())

    def sample(self, t, position):
        position = np.asarray(position, float)
        if position.shape != (3,) or not np.isfinite(position).all() or not np.isfinite(t):
            raise ValueError('invalid time or position')
        if not self.enabled:
            return np.zeros(3)
        if not self.times_s[0] <= t <= self.times_s[-1]:
            raise ValueError('force replay cannot extrapolate')
        if self.bounds_min is not None and (np.any(position < self.bounds_min) or np.any(position > self.bounds_max)):
            return np.zeros(3)
        return np.array([np.interp(t, self.times_s, self.forces_n[:, i]) for i in range(3)])
