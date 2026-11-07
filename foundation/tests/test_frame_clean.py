import json
import pytest
from foundation.analyze_frame_clean import analyze


def test_direct_pose_pairing_and_explicit_end_split(tmp_path):
    (tmp_path/'pool').mkdir()
    lines, poses = [], []
    for i in range(4):
        t = 100 + .1*i
        lines.append(f'DRONE_WORLD mono={t:.3f} actor=0x1 component=0x2 xyz_cm=0,0,{-100*i} quat=0,0,0,1\n')
        poses.append({'topic': 'pose_gt', 'arrival_monotonic_ns': round(t*1e9), 'position': [0, 0, i],
                      'quaternion_wxyz': [1, 0, 0, 0]})
    (tmp_path/'pool'/'sim.log').write_text(''.join(lines))
    (tmp_path/'capture.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in poses))
    (tmp_path/'commands.jsonl').write_text(json.dumps({'kind': 'end', 'monotonic_ns': 100_250_000_000})+'\n')
    report = analyze(tmp_path)
    assert report['count'] == 4
    assert report['before_player_end_count'] == 3
    assert report['before_player_end_residual_cm_p50_p95_max'][2] == pytest.approx(0)
    assert report['before_player_end_attitude_deg_p50_p95_max'][2] == pytest.approx(0)
