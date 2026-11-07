"""Candidate for one observed official out-time tick branch.

The caller supplies the raw stored flag and simulation delta. This does not
derive zone membership, act on thrust, or define when official stages reset.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class OutTimeCounter:
    threshold_s: float = 2.0
    elapsed_s: float = 0.0

    def __post_init__(self):
        limit = np.finfo(np.float32).max
        if (not math.isfinite(self.threshold_s) or not 0 <= self.threshold_s <= limit or
                not math.isfinite(self.elapsed_s) or not 0 <= self.elapsed_s <= limit):
            raise ValueError('invalid out-time parameters')
        self.threshold_s = float(np.float32(self.threshold_s))
        self.elapsed_s = float(np.float32(self.elapsed_s))
        if not math.isfinite(self.threshold_s) or not math.isfinite(self.elapsed_s):
            raise ValueError('out-time parameters exceed float32 range')

    def tick(self, stored_flag, dt_s, *, timeout_suppressed=False):
        if stored_flag not in (0, 1) or not math.isfinite(dt_s) or not 0 <= dt_s <= np.finfo(np.float32).max:
            raise ValueError('invalid stored flag or elapsed time')
        if not isinstance(timeout_suppressed, bool):
            raise ValueError('suppression must be boolean')
        if stored_flag == 0:
            if self.elapsed_s + dt_s > np.finfo(np.float32).max:
                raise ValueError('out-time overflow')
            self.elapsed_s = float(np.float32(np.float32(self.elapsed_s) + np.float32(dt_s)))
            return self.elapsed_s > self.threshold_s and not timeout_suppressed
        return False
