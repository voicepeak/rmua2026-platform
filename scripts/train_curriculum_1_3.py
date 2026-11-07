"""Sequential 1-3, 3-5, 5-8 curriculum: BC + DAgger + PPO.

Each segment is independent and saves under runs/curriculum/segN. The script is
safe to run in the background; it logs to runs/curriculum/segN.log.

    python scripts/train_curriculum_1_3.py
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    out_root = ROOT / "runs" / "curriculum"
    out_root.mkdir(parents=True, exist_ok=True)
    for segment in (1, 2, 3):
        save_dir = out_root / f"seg{segment}"
        log_path = out_root / f"seg{segment}.log"
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
            "--ppo-steps",
            "50000000",
            "--envs",
            "256",
            "--max-wall-hours",
            "1.0",
            "--target-success",
            "0.95",
            "--save-dir",
            str(save_dir),
        ]
        print(f"[curriculum] segment {segment} starting", flush=True)
        started = time.perf_counter()
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        elapsed = time.perf_counter() - started
        print(
            f"[curriculum] segment {segment} exit={process.returncode} wall={elapsed:.1f}s",
            flush=True,
        )
        if process.returncode != 0:
            print(f"[curriculum] stopped at segment {segment}", flush=True)
            return process.returncode
    print("[curriculum] all segments finished", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
