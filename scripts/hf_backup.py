"""Upload small data or model artifacts to a Hugging Face repo.

Examples:
    python scripts/hf_backup.py --repo-id voicepeak/rmua2026-platform-data --repo-type dataset --folder data --path-in-repo data
    python scripts/hf_backup.py --repo-id voicepeak/rmua2026-platform-policies --repo-type model --folder models --path-in-repo models

Large simulator dumps, raw UAsset JSON and full sensor logs are not committed to
Git; put them in the HF dataset repo and record the revision/hash here.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from huggingface_hub import HfApi


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--repo-type", choices=("dataset", "model"), default="dataset")
    parser.add_argument("--folder", type=Path, required=True)
    parser.add_argument("--path-in-repo", required=True)
    parser.add_argument("--private", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    if not args.folder.exists():
        raise SystemExit(f"folder not found: {args.folder}")
    api = HfApi(token=os.environ.get("HF_TOKEN"))
    if args.dry_run:
        print(f"dry-run: create {args.repo_type} repo {args.repo_id}")
        print(f"dry-run: upload {args.folder} -> {args.path_in_repo}")
        return 0

    url = api.create_repo(
        repo_id=args.repo_id,
        repo_type=args.repo_type,
        private=args.private,
        exist_ok=True,
    )
    print(f"repo: {url}")
    commit = api.upload_folder(
        repo_id=args.repo_id,
        repo_type=args.repo_type,
        folder_path=str(args.folder),
        path_in_repo=args.path_in_repo,
        commit_message="backup: RMUA2026 platform artifacts",
    )
    print(f"commit: {commit.commit_url}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
