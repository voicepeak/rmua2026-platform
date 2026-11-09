import json
from pathlib import Path

import numpy as np
import pytest

from foundation.detection_gates import DetectionGateCandidates


ROOT = Path(__file__).resolve().parents[1]


def test_ol1_detection_gate_manifest_is_loadable_and_traceable():
    path = ROOT / 'maps' / 'OL1' / 'detection_gate_candidates.json'
    gates = DetectionGateCandidates.load(path, axis=0, half_extents_cm=(200, 200, 200))
    assert [group for group, _ in gates.groups] == ['road1', 'road3']
    assert [len(records) for _, records in gates.groups] == [33, 30]
    assert gates.manifest_sha256
    assert set(gates.source_sha256) == {'road1_components.json', 'road3_components.json'}
    assert all(gate['rotation'].as_quat().shape == (4,) for _, records in gates.groups for gate in records)


def test_detection_gate_center_plane_and_transverse_bounds():
    gate = {'index': 0, 'actor': 'a', 'root': 'r', 'fname': [1, 2],
            'position_cm': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    gates = DetectionGateCandidates({'road1': [gate]}, source_sha256={'x': 'y'},
                                    axis=0, half_extents_cm=(100, 100, 100))
    hit = gates.crossings_ue([-10, 0, 0], [10, 0, 0])
    assert len(hit) == 1 and hit[0]['fraction'] == pytest.approx(.5)
    assert gates.crossings_ue([-10, 200, 0], [10, 200, 0]) == []
    assert gates.crossings_ue([10, 0, 0], [-10, 0, 0])[0]['direction'] == -1


def test_detection_gate_rejects_invalid_manifest_values(tmp_path):
    path = tmp_path / 'gates.json'
    path.write_text(json.dumps({
        'source_sha256': {'x': 'y'},
        'groups': {'road1': {'gates': [{
            'index': 0, 'position_cm': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 0],
            'scale': [1, 1, 1]}]}}
    }))
    with pytest.raises(ValueError, match='invalid detection gate'):
        DetectionGateCandidates.load(path, axis=0, half_extents_cm=(1, 1, 1))
