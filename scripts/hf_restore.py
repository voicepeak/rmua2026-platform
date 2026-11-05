"""Restore an artifact folder from Hugging Face."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from huggingface_hub import snapshot_download


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--repo-type", choices=("dataset", "model"), default="dataset")
    parser.add_argument("--revision", default=None)
    parser.add_argument("--local-dir", type=Path, required=True)
    args = parser.parse_args(argv)

    path = snapshot_download(
        repo_id=args.repo_id,
        repo_type=args.repo_type,
        revision=args.revision,
        local_dir=str(args.local_dir),
        token=os.environ.get("HF_TOKEN"),
    )
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
