import json
from pathlib import Path
import struct

import numpy as np
import pytest

from foundation.analyze_line_instances import parse
from foundation.obstacle_asset_instances import load, line_from_submission


def sample_lines():
    raw = struct.pack('<12f', 0, 0, 0, 1, 100, 200, 300, 0, 1, 1, 1.2, 0).hex()
    return [f'LINE_INSTANCE component=0x12 selector=0 transform_hex={raw} floats=ignored',
            'LINE_INSTANCE_RESULT index=3']


def test_instance_pair_decode_and_fail_closed():
    rows = parse(sample_lines())
    assert rows[0]['asset'] == 'thinline' and rows[0]['return_index'] == 3
    assert rows[0]['scale_candidate'] == pytest.approx([1, 1, 1.2])
    with pytest.raises(ValueError, match='missing'):
        parse(sample_lines()[:1])
    with pytest.raises(ValueError, match='unmatched'):
        parse(sample_lines()[1:])


def test_submission_frame_must_be_explicit():
    assets = load(Path(__file__).resolve().parents[1] / 'asset_analysis' / 'obstacle_geometry_candidates.json')
    row = parse(sample_lines())[0]
    line = line_from_submission(assets, row, frame='world', origin_ue_cm=[0, 0, 0])
    assert np.mean([line.start, line.end], axis=0) == pytest.approx([1, 2, -3], abs=1e-4)
    parent = {'translation_ue_cm': [100, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1], 'scale': [2, 2, 2]}
    line = line_from_submission(assets, row, frame='component_local', component_transform=parent,
                                origin_ue_cm=[0, 0, 0])
    assert np.mean([line.start, line.end], axis=0) == pytest.approx([3, 4, -6], abs=1e-4)
    parent['scale'] = [2, 1, 2]
    with pytest.raises(ValueError, match='uniform'):
        line_from_submission(assets, row, frame='component_local', component_transform=parent,
                             origin_ue_cm=[0, 0, 0])


def test_empirical_line_sampling_is_seeded_and_requires_feasibility():
    from foundation.line_distribution import sample_lines as generate_lines
    from scipy.spatial.transform import Rotation
    assets = load(Path(__file__).resolve().parents[1] / 'asset_analysis' / 'obstacle_geometry_candidates.json')
    record = parse(sample_lines())[0]
    record['quaternion_xyzw'] = Rotation.from_euler('y', 45, degrees=True).as_quat().tolist()
    args = (42, [record], assets, 3, [-20]*3, [20]*3, [0]*3, 2)
    lines = generate_lines(*args, frame_assumption='world_assumed', feasible=lambda line: True)
    assert lines == generate_lines(*args, frame_assumption='world_assumed', feasible=lambda line: True)
    for row in lines:
        direction = np.asarray(row['line'].end)-row['line'].start
        assert abs(direction[0]/direction[2]) == pytest.approx(1)
        assert np.linalg.norm(direction) == pytest.approx(8.4029, abs=.001)
        assert row['line'].radius_m == pytest.approx(.027947816)
        assert not row['line'].hit_swept([0]*3, [0]*3, 2)
    with pytest.raises(ValueError, match='world_assumed'):
        generate_lines(*args, frame_assumption='unknown', feasible=lambda line: True)
    with pytest.raises(ValueError, match='feasibility'):
        generate_lines(*args, frame_assumption='world_assumed', feasible=None)
