#!/usr/bin/env bash
# First command to run on a freshly rented Linux server.
# It installs the package, validates data, runs the node 0/1 tests and records
# the throughput baseline. Save the output and send it back before scaling.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "== host =="
uname -a || true
nproc || true
free -h || true
nvidia-smi || true

if [ "${SKIP_VENV:-0}" != "1" ]; then
  python3 -m venv .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

python -m pip install -U pip
python -m pip install -e ".[dev]"

echo "== data =="
python scripts/validate_data.py

echo "== tests =="
python -m pytest -q

echo "== benchmark =="
python scripts/benchmark.py --envs 64,256,1024 --steps 200 --json | tee benchmark_local.json

echo "== done =="
echo "Send benchmark_local.json + pytest output + hardware lines to the local owner."
