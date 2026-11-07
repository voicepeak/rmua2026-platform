import json
from foundation.map_inventory import build


def test_inventory_preserves_missing_geometry_and_source(tmp_path):
    source = tmp_path / "map.json"
    source.write_text(json.dumps({"OL1": {"gates": {"g": {"location": None}, "h": None},
                                                 "factories": {}, "ports": {}}}))
    result = build(source)
    assert result["maps"]["OL1"]["missing_locations"] == ["gates/g", "gates/h"]
    assert result["collision_geometry"] == "not included"
    assert len(result["source_sha256"]) == 64
