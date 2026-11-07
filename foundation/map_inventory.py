"""Inventory exported UE actor transforms, explicitly excluding collision claims."""
import argparse
import hashlib
import json
from pathlib import Path


def build(source: Path):
    raw = source.read_bytes()
    maps = json.loads(raw)
    result = {"source_sha256": hashlib.sha256(raw).hexdigest(),
              "source_units": "UE centimetres, degrees", "collision_geometry": "not included",
              "origin_and_alignment": "unverified", "maps": {}}
    for map_name, value in sorted(maps.items()):
        actors = {}
        for category in ("gates", "factories", "ports"):
            entries = value.get(category, {})
            actors[category] = {name: {key: (actor or {}).get(key) for key in
                                      ("root", "location", "rotation", "scale")}
                                for name, actor in sorted(entries.items())}
        result["maps"][map_name] = {"actors": actors,
            "counts": {key: len(items) for key, items in actors.items()},
            "missing_locations": [f"{category}/{name}" for category, items in actors.items()
                                  for name, obj in items.items() if obj["location"] is None]}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    args.output.write_text(json.dumps(build(args.source), indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")


if __name__ == "__main__":
    main()
