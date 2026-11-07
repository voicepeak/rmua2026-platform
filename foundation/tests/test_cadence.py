import json
import pytest
from foundation.analyze_cadence import compare


def make_run(path, position_offset, delta):
    path.mkdir()
    base = 10_000_000_000
    commands = [{'kind': 'command', 'monotonic_ns': base+i*delta, 'pwm': [.2]*4} for i in range(4)]
    poses = [{'topic': 'pose_gt', 'arrival_monotonic_ns': base+i*10_000_000,
              'position': [position_offset+i*.01, 0, 0]} for i in range(-10, 161)]
    for name, values in [('commands.jsonl', commands), ('capture.jsonl', poses)]:
        (path/name).write_text(''.join(json.dumps(v)+'\n' for v in values))


def test_comparison_normalizes_position_but_preserves_real_command_rate(tmp_path):
    paths = {k: tmp_path/k for k in ('20', '50', '100')}
    for k, p in paths.items():
        make_run(p, int(k), int(1e9/int(k)))
    result = compare(paths)
    assert result['runs']['20']['interval_ms_p50'] == pytest.approx(50)
    assert result['runs']['100']['interval_ms_p50'] == pytest.approx(10)
    assert result['runs']['20']['position_difference_vs_50hz_max_m'] == pytest.approx(0, abs=1e-12)
