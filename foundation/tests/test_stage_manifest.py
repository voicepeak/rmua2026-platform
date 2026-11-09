import json
from pathlib import Path


def test_stage1_manifest_matches_official_route_length():
    manifest = json.loads(Path('foundation/maps/OL1/stage1_route_manifest.json').read_text(encoding='utf-8'))
    assert manifest['stage'] == 1
    assert abs(manifest['length_m'] - 1479.47) < 2.0
    assert manifest['known_detection_components'] == {'route3': 30}
    assert manifest['waypoint_count'] > 100


def test_all_six_stage_manifests_exist():
    for stage in range(1, 7):
        manifest = json.loads(Path('foundation/maps/OL1/stage%d_route_manifest.json' % stage).read_text(encoding='utf-8'))
        assert manifest['stage'] == stage
        assert manifest['waypoint_count'] > 100
        assert len(manifest['source_sha256']) == 2
        plan = json.loads(Path('foundation/maps/OL1/stage%d_waypoints.json' % stage).read_text(encoding='utf-8'))
        assert len(plan['waypoints']) == manifest['waypoint_count']
