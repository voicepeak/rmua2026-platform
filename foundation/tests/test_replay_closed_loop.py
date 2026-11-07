import json
import numpy as np
import pytest
from foundation.replay_closed_loop import run, deduplicate_stamp


def write_rows(path, rows):
    path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")


def test_synthetic_hover_replay_and_bounds(tmp_path):
    mass, gravity, max_rpm, coefficient = .9, 9.80665, 11079.03, .000367717
    pwm = mass * gravity / (4 * coefficient * (max_rpm / 60) ** 2)
    times = np.arange(0, 3.01, .01)
    truth = [{"stamp": float(t), "position": [0, 0, 0], "orientation": [1, 0, 0, 0]} for t in times]
    imu = [{"stamp": float(t), "omega_body": [0, 0, 0]} for t in times]
    commands = [{"data": {"header": {"stamp": {"secs": int(t), "nsecs": round((t % 1)*1e9)}},
                           **{f"rotorPWM{i}": pwm for i in range(4)}}} for t in times]
    write_rows(tmp_path / "evaluation_truth.jsonl", truth)
    write_rows(tmp_path / "evaluation_imu.jsonl", imu)
    write_rows(tmp_path / "evaluation_pwm_commands.jsonl", commands)
    result = run(tmp_path, .5, 1.)
    assert result["metrics"]["position"]["max"] < 1e-10
    assert result["metrics"]["attitude_deg"]["max"] < 1e-10
    delayed = run(tmp_path, .5, 1., command_delay_s=.02, motor_tau_s=.01)
    assert delayed['command_delay_s'] == .02
    assert delayed['metrics']['position']['max'] < 1e-10
    with pytest.raises(ValueError, match='candidate delay'):
        run(tmp_path, .5, 1., command_delay_s=-.1)
    with pytest.raises(ValueError, match="exceeds recorded"):
        run(tmp_path, .5, 5.)


def test_deduplication_is_explicit_and_rejects_reverse_time():
    rows, count = deduplicate_stamp([{"stamp": 1, "v": 1}, {"stamp": 1, "v": 2}, {"stamp": 2}])
    assert count == 1 and rows[0]["v"] == 2
    with pytest.raises(ValueError, match="backwards"):
        deduplicate_stamp([{"stamp": 2}, {"stamp": 1}])
