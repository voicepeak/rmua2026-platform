"""Explicit candidate wind-region adapter; not an official natural-zone model."""
import math

import numpy as np

from foundation.wind_generator import WindGenerator


class CandidateWindField:
    def __init__(self, generator, *, source_to_ned, bounds_min_ned_m, bounds_max_ned_m,
                 source_time_offset_s=0.):
        if not isinstance(generator, WindGenerator):
            raise ValueError('expected a WindGenerator')
        transform = np.asarray(source_to_ned, float)
        low, high = np.asarray(bounds_min_ned_m, float), np.asarray(bounds_max_ned_m, float)
        if (transform.shape != (3, 3) or not np.isfinite(transform).all()
                or not np.allclose(transform.T @ transform, np.eye(3), atol=1e-9, rtol=0)
                or low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all()
                or np.any(low >= high) or not math.isfinite(source_time_offset_s)):
            raise ValueError('invalid wind transform, region or clock offset')
        self.generator = generator
        self.transform = transform.copy()
        self.low, self.high = low.copy(), high.copy()
        self.offset = float(source_time_offset_s)

    def check_window(self, start, end):
        if not np.isfinite([start, end]).all() or start > end:
            raise ValueError('invalid wind time window')
        first, last = self.generator.frames[0].time, self.generator.frames[-1].time
        # The vendored sampler converts time to float32 before locating frames.
        # Reject an end which can round to the exclusive final keyframe.
        if start + self.offset < first or np.float32(end + self.offset) >= np.float32(last):
            raise ValueError('wind field cannot extrapolate its keyframes')

    def sample(self, time_s, position_ned_m):
        point = np.asarray(position_ned_m, float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('invalid wind query position')
        self.check_window(time_s, time_s)
        if np.any(point < self.low) or np.any(point > self.high):
            return np.zeros(3)
        return self.transform @ np.asarray(self.generator.sample(time_s + self.offset), float)
