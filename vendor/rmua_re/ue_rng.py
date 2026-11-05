"""Exact UE4 FRandomStream step and float conversion used by this binary.

Evidence: 0x2fc77b0, 0x2fc4980, 0x2fc8900, 0x2fc93c0, 0x2fa3dd0 in
artifacts/rea/*.c and the assembly excerpts in the dossier.  The singleton at
0x2fcc220 stores the current state at +0x1a0:

    state = (state * 0x0bb38435 + 0x3619636b) mod 2**32

The float draw takes bits 31..9, forces the exponent to 1.0f and subtracts 1.0f.
No value is clamped or divided by a modulus; the conversion is bit-for-bit.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

LCG_MULTIPLIER = 0x0BB38435
LCG_INCREMENT = 0x3619636B
UINT32_MASK = 0xFFFFFFFF
EXPONENT_ONE = 0x3F800000

LANE_SPEED_SCALE = 11.0
LINE_OFFSET_SPAN_CM = 7000.0
LINE_OFFSET_BASE_CM = 1500.0


def f32(value: float) -> float:
    """Round a Python float to IEEE-754 binary32."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def bits_to_f32(bits: int) -> float:
    return struct.unpack("<f", struct.pack("<I", bits & UINT32_MASK))[0]


def fraction_from_state(state: int) -> float:
    """`((float)(state >> 9 | 0x3f800000)) - 1.0f`, rounded as float32."""
    bits = ((state & UINT32_MASK) >> 9) | EXPONENT_ONE
    return f32(bits_to_f32(bits) - 1.0)


def step_state(state: int) -> int:
    return ((state & UINT32_MASK) * LCG_MULTIPLIER + LCG_INCREMENT) & UINT32_MASK


@dataclass(frozen=True)
class FRandomStream:
    """Stateful FRandomStream facade.

    `draw_fraction()` performs the LCG step first and returns the fraction of
    the updated state, which is the order used by the line and vehicle code.
    """

    initial_seed: int
    state: int | None = None

    def __post_init__(self) -> None:
        seed = self.initial_seed & UINT32_MASK
        object.__setattr__(self, "initial_seed", seed)
        if self.state is None:
            object.__setattr__(self, "state", seed)
        else:
            object.__setattr__(self, "state", self.state & UINT32_MASK)

    def reset(self) -> int:
        object.__setattr__(self, "state", self.initial_seed)
        return self.state

    def step(self) -> int:
        state = step_state(self.state)
        object.__setattr__(self, "state", state)
        return state

    def fraction(self) -> float:
        return fraction_from_state(self.state)

    def draw_fraction(self) -> float:
        self.step()
        return self.fraction()

    def vehicle_bucket(self) -> int:
        """0x2fc8900/0x2fc8bb0: (int)(fraction * 11.0f), truncating."""
        return int(f32(self.fraction() * f32(LANE_SPEED_SCALE)))

    def line_offset_cm(self, reverse: bool = False) -> float:
        """0x2fc77b0: fraction * 7000.0f + 1500.0f, sign-flipped in reverse."""
        offset = f32(f32(self.fraction() * f32(LINE_OFFSET_SPAN_CM)) + f32(LINE_OFFSET_BASE_CM))
        return f32(-offset) if reverse else offset


def vehicle_bucket_from_state(state: int) -> int:
    return int(f32(fraction_from_state(state) * f32(LANE_SPEED_SCALE)))


def line_offset_from_state(state: int, reverse: bool = False) -> float:
    offset = f32(
        f32(fraction_from_state(state) * f32(LINE_OFFSET_SPAN_CM))
        + f32(LINE_OFFSET_BASE_CM)
    )
    return f32(-offset) if reverse else offset
