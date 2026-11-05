# Hugging Face backup

GitHub stores code, configs, docs and the small data bundle. Hugging Face stores
large data and policy weights.

## Repositories

- Dataset: `Vociepeak/rmua2026-platform-data` (private)
- Model: `Vociepeak/rmua2026-platform-policies` (private)

## Current backups

- Dataset data: https://huggingface.co/datasets/Vociepeak/rmua2026-platform-data/commit/31de685792f63ef69000751c57c9785a7f500651
- Dataset card: https://huggingface.co/datasets/Vociepeak/rmua2026-platform-data/commit/766392e9a5b233cab27e2c969af690da6d732ef9
- Model card: https://huggingface.co/Vociepeak/rmua2026-platform-policies/commit/d7a51424ca132774c7c5fade7d40eeba1e7c8274

The HF username is `Vociepeak` (capital V). GitHub remains `voicepeak`.

## Upload

    $env:HF_TOKEN="hf_..."   # or use `hf auth login`
    $env:HTTPS_PROXY="http://127.0.0.1:7890"
    $env:HTTP_PROXY="http://127.0.0.1:7890"

    python scripts/hf_backup.py `
      --repo-id Vociepeak/rmua2026-platform-data `
      --repo-type dataset `
      --folder data `
      --path-in-repo data

    python scripts/hf_backup.py `
      --repo-id Vociepeak/rmua2026-platform-policies `
      --repo-type model `
      --folder models `
      --path-in-repo models

## Restore

    python scripts/hf_restore.py `
      --repo-id Vociepeak/rmua2026-platform-data `
      --repo-type dataset `
      --local-dir D:\RMUA2026-Platform\data

## Rules

- Never commit `HF_TOKEN` or the cached token.
- Record the HF revision/commit hash in `data/manifest.json` or the run manifest.
- Keep the official simulator zip and raw UAsset JSON out of Git.
- Do not upload the official simulator binary or assets unless the license allows
  it; prefer a download script and a hash.

