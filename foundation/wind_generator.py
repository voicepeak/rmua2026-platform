"""Fresh-seed wind keyframe replay for calibration, not a wind force model.

The official natural random-stream consumption, spatial zones, coordinate
alignment and aerodynamic coupling have not been validated. This wrapper
rejects out-of-domain times instead of using the reference sampler's retained
last index outside its keyframe interval.
"""
import math
from pathlib import Path
from vendor.rmua_re.wind import (ElfFloatReader, Keyframe, WindSeries,
                                 generate_stage, keyframes_bytes)


class WindGenerator:
    def __init__(self, frames, constants, origin='unknown'):
        self.frames = list(frames)
        if len(self.frames) < 2 or any(not all(math.isfinite(x) for x in (f.time, f.speed, f.direction))
                                       or f.speed < 0 or f.transition not in (0, 1) for f in self.frames):
            raise ValueError('invalid wind frames')
        if any(a.time >= b.time for a, b in zip(self.frames, self.frames[1:])):
            raise ValueError('wind frame times must strictly increase')
        self.series = WindSeries(self.frames, constants)
        self.origin = origin

    @classmethod
    def fresh(cls, stage, seed, elf):
        if stage not in (4, 5, 6) or not 0 <= seed <= 0x7fffffff:
            raise ValueError('invalid stage or seed')
        reader = ElfFloatReader(Path(elf), verify_sha256=True)
        constants = reader.constants()
        return cls(generate_stage(stage, seed, constants=constants), constants,
                   origin=f'fresh stage {stage}, seed {seed}, no prior rand consumers')

    @classmethod
    def recorded(cls, raw, elf):
        if len(raw) % 16:
            raise ValueError('wind keyframe bytes must be a multiple of 16')
        reader = ElfFloatReader(Path(elf), verify_sha256=True)
        frames = [Keyframe.unpack(raw[i:i+16]) for i in range(0, len(raw), 16)]
        return cls(frames, reader.constants(), origin='recorded keyframe bytes')

    def keyframe_bytes(self):
        return keyframes_bytes(self.frames)

    def sample(self, t):
        if not math.isfinite(t) or not self.frames[0].time <= t < self.frames[-1].time:
            raise ValueError('time outside captured keyframe interval')
        return self.series.sample(t)
