"""Train 3-5 and 5-8 route-spline policies with BC+DAgger (no PPO)."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    for segment in (2, 3):
        save_dir = ROOT / "runs" / f"route_seg{segment}"
        log_path = ROOT / "runs" / f"route_seg{segment}.log"
        command = [
            sys.executable,
            "-u",
            str(ROOT / "scripts" / "train_bc_ppo.py"),
            "--segment",
            str(segment),
            "--track-mode",
            "proxy",
            "--skip-ppo",
            "--demo-steps",
            "50000",
            "--dagger-rounds",
            "6",
            "--dagger-steps",
            "30000",
            "--bc-epochs",
            "12",
            "--dagger-bc-epochs",
            "6",
            "--save-dir",
            str(save_dir),
        ]
        print(f"[route] segment {segment} starting", flush=True)
        started = time.perf_counter()
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        print(
            f"[route] segment {segment} exit={result.returncode} wall={time.perf_counter()-started:.1f}s",
            flush=True,
        )
        if result.returncode != 0:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
