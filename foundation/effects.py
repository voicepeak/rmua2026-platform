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


@dataclass(frozen=True)
class RecordedEffects:
    """Explicit time-aligned wind and externally observed field flags.

    Wind samples are linearly interpolated; flags are held until the next
    sample. The source clock, interpolation and spatial membership remain the
    caller's assumptions. No natural zone logic is inferred.
    """
    times_s: np.ndarray
    wind_ned_m_s: np.ndarray | None
    flags: np.ndarray
    enabled: bool = True

    def __post_init__(self):
        t, flags = np.asarray(self.times_s, float), np.asarray(self.flags)
        w = None if self.wind_ned_m_s is None else np.asarray(self.wind_ned_m_s, float)
        if (t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0) or not np.isfinite(t).all()
                or (w is not None and (w.shape != (len(t), 3) or not np.isfinite(w).all()))
                or flags.shape != (len(t),) or not np.isin(flags, [0, 1]).all()
                or not isinstance(self.enabled, bool)):
            raise ValueError('invalid recorded effects')
        object.__setattr__(self, 'times_s', t.copy())
        object.__setattr__(self, 'wind_ned_m_s', None if w is None else w.copy())
        object.__setattr__(self, 'flags', flags.astype(int, copy=True))

    def check_window(self, start, end):
        if not np.isfinite([start, end]).all() or start > end:
            raise ValueError('invalid effects window')
        if self.enabled and (start < self.times_s[0] or end > self.times_s[-1]):
            raise ValueError('effects replay cannot extrapolate')

    def sample(self, t):
        if not np.isfinite(t):
            raise ValueError('invalid effect time')
        if not self.enabled:
            return np.zeros(3), 1
        self.check_window(t, t)
        index = min(np.searchsorted(self.times_s, t, side='right')-1, len(self.times_s)-1)
        wind = (np.zeros(3) if self.wind_ned_m_s is None else
                np.array([np.interp(t, self.times_s, self.wind_ned_m_s[:, i]) for i in range(3)]))
        return wind, int(self.flags[index])

    def transitions(self, start, end):
        if not self.enabled:
            return []
        self.check_window(start, end)
        return self.times_s[(self.times_s > start) & (self.times_s < end)].tolist()
