import json
import pytest
from foundation.analyze_frame_alignment import analyze, nearest


def test_clock_match_and_origin_estimate(tmp_path):
    (tmp_path/'gdb.log').write_text('DRONE_WORLD mono=100.000 actor=0x1 component=0x2 xyz_cm=100,200,300 quat=0,0,0,1\n')
    (tmp_path/'sensor_clock.jsonl').write_text(json.dumps({'topic': 'imu', 'header_stamp_ns': 1234567890,
                                                          'arrival_monotonic_ns': 100_005_000_000})+'\n')
    (tmp_path/'pose_gt_posthoc.csv').write_text('field.header.stamp,field.pose.position.x,field.pose.position.y,field.pose.position.z\n1234567890,1,2,-3\n')
    result = analyze(tmp_path)
    assert result['count'] == 1
    assert result['median_candidate_origin_cm'] == pytest.approx([0, 0, 0])
    assert result['samples'][0]['clock_gap_s'] == pytest.approx(.005)
    with pytest.raises(ValueError, match='no synchronized'):
        analyze(tmp_path, monotonic_gap_s=.001)
    assert nearest([1, 3], 2, 1) in (0, 1)
