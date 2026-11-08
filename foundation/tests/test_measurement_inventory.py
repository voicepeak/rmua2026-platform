import hashlib
import json

from foundation.audit_v2_measurements import EXPECTED_BINARY, EXPECTED_SETTINGS, inspect


def test_inventory_requires_complete_command_phases_and_observations(tmp_path):
    files = ['collect_ros.py', 'collective_remote.sh', 'pwm_collective.py']
    hashes = {}
    for name in files:
        path = tmp_path / name
        path.write_bytes(name.encode())
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (tmp_path / 'run.json').write_text('{}')
    (tmp_path / 'pool').mkdir()
    (tmp_path / 'pool/pool.json').write_text(json.dumps({'binary_sha256': EXPECTED_BINARY,
                                                          'settings_sha256': EXPECTED_SETTINGS,
                                                          'instances': [{'seed': 1}]}))
    events = [{'kind': 'start', 'phases': [[.1, [.2]*4], [.1, [.1]*4]]},
              {'kind': 'command', 'monotonic_ns': 100, 'pwm': [.2]*4},
              {'kind': 'command', 'monotonic_ns': 200, 'pwm': [.1]*4},
              {'kind': 'end', 'status': 'completed'}]
    commands = tmp_path / 'commands.jsonl'
    commands.write_text(''.join(json.dumps(x) + '\n' for x in events))
    capture = tmp_path / 'capture.jsonl'
    capture.write_text(''.join(json.dumps({'topic': x}) + '\n' for x in ('pose_gt', 'imu', 'rotor_pwm')))
    run = {'mode': 'small_steps', 'exit_status': 0, 'source_sha256': hashes}
    assert inspect(tmp_path, run)['eligible']
    events[0]['phases'] = [[.1, .2], [.1, .1]]
    commands.write_text(''.join(json.dumps(x) + '\n' for x in events))
    assert inspect(tmp_path, run)['eligible']
    events.pop(2)
    commands.write_text(''.join(json.dumps(x) + '\n' for x in events))
    assert 'actual command phase sequence does not match declared waveform' in inspect(tmp_path, run)['reasons']
    capture.write_text(json.dumps({'topic': 'imu'}) + '\n')
    assert 'missing pose, IMU, or PWM echo observations' in inspect(tmp_path, run)['reasons']
