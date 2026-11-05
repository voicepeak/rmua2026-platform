"""Validate bundled map data, table files and rule contracts."""

from __future__ import annotations

import csv
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rmua.rules.model import load_rules  # noqa: E402
from rmua.scene.map_data import available_maps, load_map_data  # noqa: E402
from rmua.scene.track import build_track  # noqa: E402

EXPECTED_TABLES = {
    "gates.csv": 144,
    "factories.csv": 144,
    "ports.csv": 144,
    "check_ports.csv": 12,
    "routes.csv": None,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    rules = load_rules()
    maps = available_maps()
    assert len(maps) == 12, maps
    for name in maps:
        data = load_map_data(name)
        assert len(data.gates) == 12, (name, len(data.gates))
        assert len(data.factories) == 12, (name, len(data.factories))
        assert len(data.ports) == 12, (name, len(data.ports))
        assert len(data.route_configs) == 12, (name, len(data.route_configs))
        for spec in rules.segments:
            track = build_track(data, spec)
            assert track.total_length_m > 10.0, (name, spec.name, track.total_length_m)

    tables_dir = ROOT / "data" / "tables"
    for filename, expected in EXPECTED_TABLES.items():
        path = tables_dir / filename
        assert path.exists(), path
        with path.open(newline="", encoding="utf-8") as handle:
            rows = sum(1 for _ in csv.DictReader(handle))
        if expected is not None:
            assert rows == expected, (filename, rows, expected)

    files = [ROOT / "data" / "maps" / "maps_summary_all.json"]
    files += sorted(tables_dir.glob("*.csv"))
    manifest = {
        "git_commit": None,
        "files": [
            {
                "path": str(path.relative_to(ROOT)).replace("\\", "/"),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in files
        ],
    }
    manifest_path = ROOT / "data" / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"maps": len(maps), "manifest": str(manifest_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
