"""Offline port of the wind generator (0x2fb23b0) and sampler (0x2fb2050/0x2fb2130).

Keyframe synthesis is a byte-exact float32 translation of the decompiled
routine.  All numeric constants are read from the local 12.0.0.5 ELF at the
virtual addresses shown in `CONSTANT_ADDRESSES`; stage parameters come from the
0x2fbac30 call sites and the default GameConfig time limits documented in
docs/03_seed_config_game_logic.md.

Scope: a fresh stage initialization immediately after srand(seed), with no
earlier libc rand() consumer.  The generator and sampler are separate: keyframe
bytes are exact; sampler trigonometry uses the host libm rounded to float32 and
is not claimed to be byte-identical to glibc sincosf.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
from dataclasses import dataclass
from pathlib import Path

from .glibc_rand import GlibcRand
from .ue_rng import f32

WINDOWS_ELF = Path(
    r"D:\RMUA2026-re\sim\simulator_12.0.0.5"
    r"\Build\LinuxNoEditor\RMUA\Binaries\Linux\RMUA-Linux-Shipping"
)
WSL_ELF = Path(
    "/mnt/d/RMUA2026-re/sim/simulator_12.0.0.5"
    "/Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping"
)


def default_elf_path() -> Path:
    override = os.environ.get("RMUA_ELF")
    if override:
        return Path(override)
    if WINDOWS_ELF.exists():
        return WINDOWS_ELF
    return WSL_ELF


DEFAULT_ELF = default_elf_path()
EXPECTED_ELF_SHA256 = "5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800"

CONSTANT_ADDRESSES = {
    "hold_min_s": 0x6C1440,
    "rand_denominator": 0x6C163C,
    "base_hold_s": 0x6C0D20,
    "transition_s": 0x6C14D4,
    "speed_cap_delta": 0x6C0FDC,
    "speed_spread": 0x6C1A40,
    "speed_bias": 0x6C0C84,
    "angle_span": 0x6C0E50,
    "zero": 0x6C0FA8,
    "radians_per_degree": 0x6C0F10,
    "stage4_target": 0x6C0924,
    "stage4_maximum": 0x6C14D4,
    "stage4_minimum": 0x6C0FBC,
    "stage5_target": 0x6C17BC,
    "stage5_maximum": 0x6C0DA4,
    "stage5_minimum": 0x6C14D4,
    "stage6_target": 0x6C1860,
    "stage6_maximum": 0x6C1228,
    "stage6_minimum": 0x6C0DA4,
}

GAMECONFIG_DURATION_DEFAULTS = {4: 200.0, 5: 220.0, 6: 200.0}
STAGE_STATE_CALLS = {4: (8, 12, 4), 5: (12, 10, 5), 6: (10, 7, 6)}


class ElfFloatReader:
    """Read little-endian ELF64 PT_LOAD contents by virtual address."""

    def __init__(self, path: Path = DEFAULT_ELF, verify_sha256: bool = True):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        if self.data[:6] != b"\x7fELF\x02\x01":
            raise ValueError(f"{self.path} is not a little-endian ELF64 file")
        self.sha256 = hashlib.sha256(self.data).hexdigest()
        if verify_sha256 and self.sha256 != EXPECTED_ELF_SHA256:
            raise ValueError(
                f"unexpected ELF SHA-256 {self.sha256}; expected {EXPECTED_ELF_SHA256}"
            )
        phoff = struct.unpack_from("<Q", self.data, 32)[0]
        entsize, count = struct.unpack_from("<HH", self.data, 54)
        self._segments = []
        for index in range(count):
            typ, _, off, va, _, filesz, _, _ = struct.unpack_from(
                "<IIQQQQQQ", self.data, phoff + index * entsize
            )
            if typ == 1:
                self._segments.append((va, va + filesz, off))

    def read(self, vaddr: int, size: int) -> bytes:
        matches = [
            off + vaddr - start
            for start, end, off in self._segments
            if start <= vaddr and vaddr + size <= end
        ]
        if len(matches) != 1:
            raise ValueError(f"unmapped or ambiguous virtual address {vaddr:#x}")
        offset = matches[0]
        return self.data[offset : offset + size]

    def float32(self, vaddr: int) -> float:
        return struct.unpack("<f", self.read(vaddr, 4))[0]

    def constants(self) -> dict[str, float]:
        return {name: self.float32(address) for name, address in CONSTANT_ADDRESSES.items()}


def load_constants(path: Path = DEFAULT_ELF, verify_sha256: bool = True) -> dict[str, float]:
    return ElfFloatReader(path, verify_sha256=verify_sha256).constants()


@dataclass(frozen=True)
class WindParameters:
    duration_s: float
    target_integral: float
    maximum_speed: float
    minimum_speed: float

    @property
    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.duration_s,
            self.target_integral,
            self.maximum_speed,
            self.minimum_speed,
        )


def stage_parameters(
    stage: int, constants: dict[str, float] | None = None
) -> WindParameters:
    if constants is None:
        constants = load_constants()
    if stage == 4:
        target = constants["stage4_target"]
        maximum = constants["stage4_maximum"]
        minimum = constants["stage4_minimum"]
    elif stage == 5:
        target = constants["stage5_target"]
        maximum = constants["stage5_maximum"]
        minimum = constants["stage5_minimum"]
    elif stage == 6:
        target = constants["stage6_target"]
        maximum = constants["stage6_maximum"]
        minimum = constants["stage6_minimum"]
    else:
        raise ValueError("stage must be 4, 5, or 6")
    return WindParameters(
        f32(GAMECONFIG_DURATION_DEFAULTS[stage]),
        f32(target),
        f32(maximum),
        f32(minimum),
    )


@dataclass(frozen=True)
class Keyframe:
    time: float
    speed: float
    direction: float
    transition: int

    def pack(self) -> bytes:
        return struct.pack(
            "<fffI", self.time, self.speed, self.direction, self.transition
        )

    @classmethod
    def unpack(cls, raw: bytes) -> "Keyframe":
        time, speed, direction, transition = struct.unpack("<fffI", raw)
        return cls(time, speed, direction, transition)

    def as_dict(self, index: int | None = None) -> dict:
        payload = {
            "time_s": self.time,
            "speed": self.speed,
            "direction_deg": self.direction,
            "transition": self.transition,
            "raw_hex": self.pack().hex(),
        }
        if index is not None:
            payload = {"index": index, **payload}
        return payload


def _rand_unit(rng, denominator: float) -> float:
    return f32(f32(rng.rand() & 0xFFFFFF) / denominator)


def generate_keyframes(
    parameters: WindParameters,
    *,
    seed: int | None = None,
    rng=None,
    constants: dict[str, float] | None = None,
) -> list[Keyframe]:
    """Translate 0x2fb23b0 into float32 operations."""
    if constants is None:
        constants = load_constants()
    if rng is None:
        if seed is None:
            raise ValueError("seed or rng is required")
        rng = GlibcRand(seed)

    hold_min = f32(constants["hold_min_s"])
    denominator = f32(constants["rand_denominator"])
    base_hold = f32(constants["base_hold_s"])
    transition = f32(constants["transition_s"])
    cap_delta = f32(constants["speed_cap_delta"])
    spread = f32(constants["speed_spread"])
    bias = f32(constants["speed_bias"])
    angle_span = f32(constants["angle_span"])
    zero = f32(constants["zero"])

    target = f32(parameters.target_integral)
    maximum = f32(parameters.maximum_speed)
    minimum = f32(parameters.minimum_speed)
    remaining = f32(parameters.duration_s)

    durations: list[float] = []
    while hold_min <= remaining:
        unit = _rand_unit(rng, denominator)
        length = f32(f32(unit + unit) + base_hold)
        durations.append(length)
        remaining = f32(remaining - f32(length + transition))

    count = len(durations)
    if count < 2:
        return []

    unit = _rand_unit(rng, denominator)
    selected = int(f32(f32(float(count)) * unit))
    if count <= selected:
        selected = count - 1

    denominator_base = f32(
        f32(f32(parameters.duration_s) - durations[selected])
        - f32(f32(count - 1) + f32(count - 1))
    )
    numerator = f32(target - f32(durations[selected] * maximum))
    base_speed = f32(numerator / denominator_base)
    speed_cap = f32(cap_delta + maximum)

    speeds: list[float] = []
    directions: list[float] = []
    for index in range(count):
        if index == selected:
            speed = maximum
        else:
            raw = f32(f32(f32(_rand_unit(rng, denominator) * spread) + bias) + base_speed)
            speed = speed_cap if speed_cap <= raw else raw
            if raw < minimum:
                speed = minimum
        speeds.append(speed)
        directions.append(f32(_rand_unit(rng, denominator) * angle_span))

    frames: list[Keyframe] = []
    current = f32(zero)
    for index in range(count):
        frames.append(Keyframe(current, speeds[index], directions[index], 0))
        hold_end = f32(current + durations[index])
        if index < count - 1:
            frames.append(Keyframe(hold_end, speeds[index], directions[index], 1))
            current = f32(hold_end + transition)
        else:
            current = hold_end
    return frames


def generate_stage(
    stage: int,
    seed: int,
    *,
    constants: dict[str, float] | None = None,
    rng=None,
) -> list[Keyframe]:
    if constants is None:
        constants = load_constants()
    return generate_keyframes(
        stage_parameters(stage, constants),
        seed=seed,
        rng=rng,
        constants=constants,
    )


def keyframes_bytes(frames: list[Keyframe]) -> bytes:
    return b"".join(frame.pack() for frame in frames)


class WindSeries:
    """Stateful sampler corresponding to 0x2fb2050 and 0x2fb2130."""

    def __init__(self, frames: list[Keyframe], constants: dict[str, float] | None = None):
        self.frames = frames
        self.index = 0
        if constants is None:
            constants = load_constants()
        self._radians_per_degree = f32(constants["radians_per_degree"])
        self._zero = f32(constants["zero"])

    def locate(self, time_s: float) -> int:
        """0x2fb2050: first i with frame[i].time <= t < frame[i+1].time."""
        t = f32(time_s)
        for index in range(len(self.frames) - 1):
            if f32(self.frames[index].time) <= t < f32(self.frames[index + 1].time):
                self.index = index
                return index
        return self.index

    def sample(self, time_s: float) -> tuple[float, float, float]:
        """0x2fb2130: (speed*cos(angle), speed*sin(angle), 0)."""
        if len(self.frames) < 2:
            return (self._zero, self._zero, self._zero)
        index = self.locate(time_s)
        if index < 0 or index + 1 >= len(self.frames):
            raise IndexError("wind sampler index outside the keyframe table")
        current = self.frames[index]
        following = self.frames[index + 1]
        if current.transition == 0:
            speed = current.speed
            angle = current.direction
        else:
            t = f32(time_s)
            denominator = f32(following.time - current.time)
            ratio = f32(f32(t - current.time) / denominator)
            limit = f32(1.0)
            if limit <= ratio:
                ratio = limit
            if ratio < 0.0:
                ratio = 0.0
            speed = f32(f32(f32(following.speed - current.speed) * ratio) + current.speed)
            angle = f32(
                f32(f32(following.direction - current.direction) * ratio)
                + current.direction
            )
        radians = f32(angle * self._radians_per_degree)
        x = f32(speed * math.cos(radians))
        y = f32(speed * math.sin(radians))
        return (x, y, self._zero)


def model_report(
    stages: tuple[int, ...] = (4, 5, 6), seeds: tuple[int, ...] = (123, 456, 2026)
) -> dict:
    constants = load_constants()
    report = {
        "elf_sha256": EXPECTED_ELF_SHA256,
        "stages": {},
    }
    for stage in stages:
        parameters = stage_parameters(stage, constants)
        stage_report = {"parameters": list(parameters.as_tuple), "seeds": {}}
        for seed in seeds:
            frames = generate_keyframes(parameters, seed=seed, constants=constants)
            raw = keyframes_bytes(frames)
            stage_report["seeds"][str(seed)] = {
                "items": [frame.as_dict(index) for index, frame in enumerate(frames)],
                "count": len(frames),
                "sha256": hashlib.sha256(raw).hexdigest(),
            }
        report["stages"][str(stage)] = stage_report
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=int, choices=(4, 5, 6))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--json", action="store_true", help="emit JSON")
    parser.add_argument(
        "--all",
        action="store_true",
        help="emit the stage 4/5/6 x seed 123/456/2026 model report",
    )
    args = parser.parse_args(argv)

    if args.all:
        print(json.dumps(model_report(), indent=2))
        return 0

    if args.stage is None or args.seed is None:
        parser.error("--stage and --seed are required unless --all is used")

    constants = load_constants(args.elf)
    frames = generate_stage(args.stage, args.seed, constants=constants)
    if args.json:
        print(
            json.dumps(
                {
                    "stage": args.stage,
                    "seed": args.seed,
                    "parameters": list(stage_parameters(args.stage, constants).as_tuple),
                    "items": [frame.as_dict(index) for index, frame in enumerate(frames)],
                    "sha256": hashlib.sha256(keyframes_bytes(frames)).hexdigest(),
                },
                indent=2,
            )
        )
    else:
        for index, frame in enumerate(frames):
            print(
                f"{index:3d} t={frame.time:.9g} "
                f"speed={frame.speed:.9g} dir={frame.direction:.9g} "
                f"transition={frame.transition}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
