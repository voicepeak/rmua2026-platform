"""Static force field outside the road/hub corridor."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .track import SegmentTrack


@dataclass
class ForceField:
    inside: np.ndarray
    factor: np.ndarray
    outside: np.ndarray

    @classmethod
    def evaluate(cls, track: SegmentTrack, positions: np.ndarray, thrust_scale: float = 0.35):
        inside = track.in_corridor(positions)
        factor = np.where(inside, 1.0, thrust_scale)
        return cls(inside=inside, factor=factor, outside=~inside)
