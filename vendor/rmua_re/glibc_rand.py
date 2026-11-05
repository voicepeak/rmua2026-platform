"""glibc `srand()`/`rand()` sequence used by the wind generator.

The simulator calls libc `srand(seed)` at 0x2fb2040 and libc `rand()` inside
0x2fb23b0.  On the captured host this is glibc's TYPE_3 additive-feedback
generator with degree 31 and separation 3.

Pure-Python fallback (documented algorithm):

* initialise r[0] = seed (0 becomes 1), then for i = 1..30:
  r[i] = (16807 * r[i-1]) mod 2147483647, computed with the 127773/2836
  Schrage split used by glibc;
* discard 10 * 31 outputs;
* each call returns ((r[f] += r[r]) mod 2**32) >> 1, then advances f and r
  modulo 31.

This is not the ANSI C `rand()` example and not `random.Random`.  When running
under Linux/glibc, `make_glibc_rand()` prefers ctypes and exposes the host
sequence directly.  The fallback is byte-identical for every glibc version
whose random.c still uses TYPE_3 (verified here against glibc 2.39 and against
captured glibc 2.35 wind keyframes).
"""

from __future__ import annotations

import ctypes
import platform
import sys
from dataclasses import dataclass, field

DEGREE = 31
SEPARATION = 3
SEED_MODULUS = 2147483647
UINT32_MASK = 0xFFFFFFFF


class GlibcRand:
    """Pure-Python reproduction of glibc TYPE_3 `random()`."""

    def __init__(self, seed: int = 1):
        self.srand(seed)

    def srand(self, seed: int) -> None:
        seed = int(seed) & UINT32_MASK
        if seed == 0:
            seed = 1
        state = [0] * DEGREE
        state[0] = seed
        word = seed
        for index in range(1, DEGREE):
            hi = word // 127773
            lo = word % 127773
            word = 16807 * lo - 2836 * hi
            if word < 0:
                word += SEED_MODULUS
            state[index] = word
        self._state = state
        self._fptr = SEPARATION
        self._rptr = 0
        for _ in range(10 * DEGREE):
            self.rand()

    def rand(self) -> int:
        state = self._state
        value = (state[self._fptr] + state[self._rptr]) & UINT32_MASK
        state[self._fptr] = value
        result = value >> 1
        self._fptr = (self._fptr + 1) % DEGREE
        self._rptr = (self._rptr + 1) % DEGREE
        return result


@dataclass
class CtypesGlibcRand:
    """ctypes wrapper around the running host's glibc `srand`/`rand`."""

    seed: int = 1
    _lib: object = field(init=False, repr=False)
    _last_seed: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        lib = ctypes.CDLL(None)
        lib.rand.restype = ctypes.c_int
        lib.srand.argtypes = [ctypes.c_uint]
        self._lib = lib
        self.srand(self.seed)

    def srand(self, seed: int) -> None:
        self.seed = int(seed) & UINT32_MASK
        self._last_seed = self.seed
        self._lib.srand(ctypes.c_uint(self.seed))

    def rand(self) -> int:
        return int(self._lib.rand())


def host_is_glibc() -> bool:
    return sys.platform.startswith("linux") and platform.libc_ver()[0].lower() == "glibc"


def make_glibc_rand(seed: int = 1, backend: str = "auto"):
    """Return a ctypes glibc stream on glibc, otherwise the pure fallback."""
    if backend == "pure":
        return GlibcRand(seed)
    if backend == "ctypes":
        return CtypesGlibcRand(seed)
    if backend != "auto":
        raise ValueError("backend must be auto, pure, or ctypes")
    if host_is_glibc():
        return CtypesGlibcRand(seed)
    return GlibcRand(seed)
