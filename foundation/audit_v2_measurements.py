"""Inventory completed non-GDB-configured PWM training captures for V2 G1.

This audit deliberately excludes reserved holdout runs and does not fit a model.
"""
import hashlib
import json
from pathlib import Path
import statistics


ROOT = Path('foundation/datasets')
MODES = {'collective', 'small_steps', 'roll_pitch', 'roll_pitch_small',
         'yaw', 'yaw_small', 'cadence20', 'cadence50', 'cadence100'}
EXPECTED_BINARY = '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800'
EXPECTED_SETTINGS = '0d72886f92b091cbc5ba2ae58d25b4b4bc589045d4e725f483603efdc26c6c17'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect(directory, run):
    reasons = []
    required = [directory / 'commands.jsonl', directory / 'capture.jsonl', directory / 'pool/pool.json']
    if any(not p.is_file() or p.stat().st_size == 0 for p in required):
        return {'directory': str(directory), 'mode': run.get('mode'), 'eligible': False,
                'reasons': ['missing or empty command, capture, or pool record']}
    pool = json.loads(required[2].read_bytes())
    records = [json.loads(s) for s in required[0].read_bytes().splitlines() if s]
    starts = [x for x in records if x.get('kind') == 'start']
    ends = [x for x in records if x.get('kind') == 'end']
    commands = [x for x in records if x.get('kind') == 'command']
    if run.get('exit_status') != 0:
        reasons.append('run exit status is not zero')
    if len(starts) != 1 or len(ends) != 1 or ends[0].get('status') != 'completed' or len(commands) < 2:
        reasons.append('player start/end/command stream incomplete')
    if pool.get('binary_sha256') != EXPECTED_BINARY:
        reasons.append('binary hash does not match baseline')
    if pool.get('settings_sha256') != EXPECTED_SETTINGS:
        reasons.append('settings hash does not match comparison group')
    if list(directory.glob('*.gdb')):
        reasons.append('debugger script found; investigate before inclusion')
    source_checks = {}
    for name in ('collect_ros.py', 'collective_remote.sh', 'pwm_collective.py'):
        expected = run.get('source_sha256', {}).get(name)
        path = directory / name
        source_checks[name] = bool(expected and path.is_file() and digest(path) == expected)
        if not source_checks[name]:
            reasons.append(f'{name} is missing or does not match recorded hash')
    phase_list = starts[0].get('phases') if len(starts) == 1 else None
    if not phase_list:
        reasons.append('declared phases missing')
    actual_sequence = []
    for command in commands:
        if not actual_sequence or actual_sequence[-1] != command.get('pwm'):
            actual_sequence.append(command.get('pwm'))
    declared_sequence = []
    normalized_phases = []
    for phase in phase_list or []:
        value = phase[1]
        # Older collective players stored one scalar for all four motors.
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            value = [value] * 4
        normalized_phases.append([phase[0], value])
        if not declared_sequence or declared_sequence[-1] != value:
            declared_sequence.append(value)
    phase_hash = (hashlib.sha256(json.dumps(normalized_phases, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
                  if normalized_phases else None)
    phase_sequence_matches = bool(declared_sequence) and actual_sequence == declared_sequence
    if not phase_sequence_matches:
        reasons.append('actual command phase sequence does not match declared waveform')
    capture = [json.loads(s) for s in required[1].read_bytes().splitlines() if s]
    topics = {topic: sum(x.get('topic') == topic for x in capture)
              for topic in ('pose_gt', 'imu', 'rotor_pwm', 'gps', 'clock')}
    if any(topics[t] == 0 for t in ('pose_gt', 'imu', 'rotor_pwm')):
        reasons.append('missing pose, IMU, or PWM echo observations')
    times = [x['monotonic_ns'] for x in commands]
    if any(b <= a for a, b in zip(times, times[1:])):
        reasons.append('command host time is nonmonotonic')
    intervals = [(b-a)*1e-9 for a, b in zip(times, times[1:])]
    instances = pool.get('instances', [])
    return {'directory': str(directory), 'mode': run.get('mode'), 'eligible': not reasons,
            'reasons': reasons, 'seed_requested': run.get('seed_requested'),
            'seed_recorded_by_pool': instances[0].get('seed') if len(instances) == 1 else None,
            'requested_rate_hz': starts[0].get('requested_rate_hz') if len(starts) == 1 else None,
            'declared_phases': phase_list, 'declared_phase_sha256': phase_hash,
            'actual_pwm_phase_vectors': actual_sequence, 'actual_phase_sequence_matches_declared': phase_sequence_matches,
            'topic_sample_counts': topics,
            'command_count': len(commands), 'median_host_interval_s': statistics.median(intervals) if intervals else None,
            'source_script_hash_matches': source_checks,
            'binary_sha256': pool.get('binary_sha256'), 'settings_sha256': pool.get('settings_sha256'),
            'input_sha256': {str(p.relative_to(directory)).replace('\\', '/'): digest(p) for p in [directory/'run.json', *required]}}


def main():
    groups = {x: [] for x in sorted(MODES)}
    for path in sorted(ROOT.glob('*/run.json')):
        run = json.loads(path.read_bytes())
        mode = run.get('mode')
        if mode in MODES:
            groups[mode].append(inspect(path.parent, run))
    summary = {}
    for mode, runs in groups.items():
        eligible = [x for x in runs if x['eligible']]
        phase_counts = {}
        for row in eligible:
            key = row['declared_phase_sha256']
            phase_counts[key] = phase_counts.get(key, 0) + 1
        summary[mode] = {'records': runs, 'eligible_count': len(eligible),
                         'largest_exact_declared_phase_group': max(phase_counts.values(), default=0),
                         'eligible_phase_group_counts': phase_counts}
    result = {'status': 'metadata eligibility audit; not proof of free flight, independence or G1 acceptance',
              'groups': summary, 'expected_binary_sha256': EXPECTED_BINARY,
              'expected_settings_sha256': EXPECTED_SETTINGS,
              'analysis_sha256': digest(Path(__file__)),
              'limitations': ['mode and saved script indicate no runtime debugger configured, not an independent process trace',
                              'declared phases and command timestamps do not establish identical applied motor input',
                              'seed and completed process do not prove matched initial states or no wind/contact',
                              'source and timestamp coverage need separate analysis',
                              'reserved v2_holdout_axis is explicitly excluded from this inventory']}
    Path('foundation/reports/v2_g1_capture_inventory.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for mode, group in summary.items():
        print(mode, 'eligible', group['eligible_count'], 'same declared phases', group['largest_exact_declared_phase_group'])


if __name__ == '__main__':
    main()
