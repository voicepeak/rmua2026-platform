# Vendored reference models

These files are copied from `voicepeak/RMUA2026-re/local_models` for reference:

- `wind.py`: byte-exact official wind keyframe generator (needs the verified ELF).
- `lines.py`, `vehicles.py`, `spline.py`, `ue_rng.py`, `glibc_rand.py`, `engine.py`.

They are **read-only reference code**, not imported by the default node 1 path.
The default `sim-fast` uses the simplified models in `src/rmua/scene/`. When the
server has the verified simulator binary, replace the simplified models with
these calibrated implementations and re-run the evidence tests.

Do not edit these files in place; add an adapter under `src/rmua/scene/` and keep
the original as the provenance copy.
