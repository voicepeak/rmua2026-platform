# Models

Policy weights are not stored in Git. Use the Hugging Face model repo:

    voicepeak/rmua2026-platform-policies

Naming convention:

    policies/<run_id>/<stage>/<policy>.onnx
    policies/<run_id>/<stage>/normalization.json
    policies/<run_id>/manifest.json

Every model manifest must record:

- git commit
- training config hash
- seed range
- sim-fast throughput baseline
- evaluation seed list
- official calibration revision (if any)

Upload:

    python scripts/hf_backup.py --repo-id voicepeak/rmua2026-platform-policies --repo-type model --folder models --path-in-repo models
