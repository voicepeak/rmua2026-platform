# Local build

## Requirements

- Python 3.10+ (tested 3.11.9)
- NumPy, PyYAML
- pytest for tests
- Optional: Hugging Face CLI for backup

## Install

    cd D:\RMUA2026-Platform
    python -m pip install -e ".[dev]"

## Verify node 0

    python -m pytest tests/test_rules.py tests/test_frames.py tests/test_map_data.py -q
    python scripts/validate_data.py

## Verify node 1

    python -m pytest tests/test_quad.py tests/test_gate.py tests/test_env_smoke.py -q
    python scripts/benchmark.py --envs 64,256,1024 --steps 200

Expected on the current local machine (20 logical cores, Python reference):

- 64 envs: ~66k env-steps/s, ~10.3x realtime/env
- 256 envs: ~195k env-steps/s, ~7.6x realtime/env
- 1024 envs: ~285k env-steps/s, ~2.8x realtime/env

These numbers are a baseline, not a server promise. The server must re-run the
same benchmark and freeze the acceptance threshold. The Python reference does
not reach the 1M env-steps/s target for 10k envs; the server backend must be
C++/JAX/MJX and/or multi-process.

## Windows proxy note

Git currently has a global proxy at `127.0.0.1:7892`; the active Clash port on
this machine is `7890`. Use per-command overrides or set the local repo config:

    git -c http.proxy=http://127.0.0.1:7890 -c https.proxy=http://127.0.0.1:7890 clone ...
    $env:HTTPS_PROXY="http://127.0.0.1:7890"

## Reproduce a segment

    python -c "from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv; e=SimFastVecEnv(SimFastConfig(num_envs=1, segment_id=1)); o=e.reset(); print(o.shape)"
