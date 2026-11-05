# Data bundle

This folder contains the small, versionable part of the RMUA2026 reverse-engineering data:

- `maps/maps_summary_all.json`: 12 maps, route configs, gates, factories, ports, splines.
- `maps/obstacle_collision_meshes.json`: extracted convex collision meshes for thick/thin lines and vehicles.
- `tables/*.csv`: flat tables used by the sim-fast loader.
- `manifest.json`: size and SHA-256 for every bundled file.

## Provenance

Source: `voicepeak/RMUA2026-re` (simulator 12.0.0.5 static analysis). The data is
kept here for the training platform; it is not a new official release. Verify the
simulator hash and the source docs before using it for competition decisions.

## Large artifacts

The simulator zip, raw UAssetAPI JSON, extracted pak assets and full sensor logs
are **not** stored in Git. Use the Hugging Face dataset repo:

    voicepeak/rmua2026-platform-data

Upload/restore:

    python scripts/hf_backup.py --repo-id voicepeak/rmua2026-platform-data --repo-type dataset --folder data --path-in-repo data
    python scripts/hf_restore.py --repo-id voicepeak/rmua2026-platform-data --repo-type dataset --local-dir D:\RMUA2026-Platform\data

## Known limitations

- Gate aperture is approximated as 10 m x 5 m; docs/20 marks it unmeasured.
- Factory interior device layout is not in the static export.
- Building/road mesh collision is approximated by a 10 m wide / 5 m high corridor.
- Segment-to-route-spline/gate mapping has `confidence: approx` in
  `configs/rules_2026.yaml`; runtime calibration is required.
