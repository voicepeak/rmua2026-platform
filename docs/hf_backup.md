# Hugging Face backup

GitHub stores code, configs, docs and the small data bundle. Hugging Face stores
large data and policy weights.

## Repositories

- Dataset: `voicepeak/rmua2026-platform-data` (private)
- Model: `voicepeak/rmua2026-platform-policies` (private)

## Upload

    $env:HF_TOKEN="hf_..."   # or use `hf auth login`
    $env:HTTPS_PROXY="http://127.0.0.1:7890"
    $env:HTTP_PROXY="http://127.0.0.1:7890"

    python scripts/hf_backup.py `
      --repo-id voicepeak/rmua2026-platform-data `
      --repo-type dataset `
      --folder data `
      --path-in-repo data

    python scripts/hf_backup.py `
      --repo-id voicepeak/rmua2026-platform-policies `
      --repo-type model `
      --folder models `
      --path-in-repo models

## Restore

    python scripts/hf_restore.py `
      --repo-id voicepeak/rmua2026-platform-data `
      --repo-type dataset `
      --local-dir D:\RMUA2026-Platform\data

## Rules

- Never commit `HF_TOKEN` or the cached token.
- Record the HF revision/commit hash in `data/manifest.json` or the run manifest.
- Keep the official simulator zip and raw UAsset JSON out of Git.
- Do not upload the official simulator binary or assets unless the license allows
  it; prefer a download script and a hash.
