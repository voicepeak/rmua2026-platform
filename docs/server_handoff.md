# Server handoff checklist

The local node 0/1 build is complete. Rent the smallest useful server first and
run this checklist before installing CUDA/PyTorch or scaling out.

## 1. Clone and smoke test

    git clone https://github.com/voicepeak/rmua2026-platform.git
    cd rmua2026-platform
    bash scripts/server_smoke.sh

Save `benchmark_local.json`, the pytest summary and the hardware lines.

## 2. Acceptance gates before training

- `python scripts/validate_data.py` passes.
- `python -m pytest -q` passes.
- Benchmark output exists; no thresholds are frozen until this run.
- CPU single-core performance and RAM are recorded.
- If a GPU is present, record `nvidia-smi` and driver/CUDA versions.

## 3. Do not waste server time on

- Downloading the 2 GB simulator zip before node 2 calibration is scheduled.
- Running more than 4 official simulator instances.
- Relying on native `/airsim_node/reset`; use a new process/container per episode.
- Installing large DL frameworks before the state-only throughput baseline is
  recorded.

## 4. Next server task: node 2

1. Port the inner loop to C++/JAX/MJX and re-run the same benchmark.
2. System-identify PWM step, hover and braking against the official simulator.
3. Replace simplified wind/line/vehicle models with `vendor/rmua_re` calibrated
   implementations.
4. Add latency/noise/GPS-dropout domain randomization.
5. Freeze throughput and task acceptance, then start PPO/APPO.

## 5. What to send back

- `benchmark_local.json`
- pytest summary and any failure logs
- `nproc`, RAM, GPU/driver
- exact commit hash (`git rev-parse HEAD`)
- any change to `configs/rules_2026.yaml` (must update G0 tests)
