import json
from foundation.analyze_alignment_motion import analyze


def test_repeated_roots_do_not_hide_movement_residual(tmp_path):
    samples = []
    for i, position in enumerate([0., 0., 1., 2.]):
        samples.append({'gdb_monotonic_s': float(i), 'clock_gap_s': 0.,
                        'actor_root_cm': [0., 0., 0.], 'pose_ned_m': [position, 0., 0.],
                        'candidate_origin_cm': [-100*position, 0., 0.]})
    path = tmp_path/'samples.json'
    path.write_text(json.dumps({'samples': samples}))
    report = analyze(path)
    assert report['unique_actor_root_values'] == 1
    assert report['unique_ros_pose_values'] == 3
    assert report['moving_intervals'] == 2
    assert report['moving_and_high_residual'] >= 1
