from pathlib import Path

from foundation.audit_map_geometry import probe
from foundation.gate_geometry import GateMeshVolumes
from foundation.map_scene import CandidateMap


def test_representative_gate_road_and_wall_candidate_probe():
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    scene = CandidateMap.load(assets)
    gates = GateMeshVolumes.load(assets, crossing_axis=1)
    result = probe(scene, gates, road_count=4, wall_count=4)
    assert result['gate_matched'] == len(gates.gates) == 12
    assert result['road_matched'] == len(result['road_surface_self_hits']) == 4
    assert result['wall_matched'] == len(result['wall_surface_self_hits']) == 4
    assert result['complex_as_simple_matched'] == len(result['complex_as_simple_surfaces']) == 25
