"""Parse the official score/SEG trace from a candidate detection-gate run."""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np

from foundation.detection_gates import DetectionGateCandidates


GATE_MANIFEST = Path('foundation/maps/OL1/detection_gate_candidates.json')
SEG = re.compile(
    r'SEG actor=(?P<actor>\S+) name=(?P<name>\d+,\d+) idx=(?P<index>-?\d+) '
    r'enter=(?P<enter>-?\d+) event=(?P<event>-?\d+) components=(?P<components>\d+) '
    r'score=(?P<score>-?\d+)')
STATE = re.compile(
    r'STATE old=(?P<old>-?\d+) new=(?P<new>-?\d+) flag=(?P<flag>-?\d+) '
    r'score=(?P<score>-?\d+)')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--group', default='road1')
    args = parser.parse_args()
    run = args.run
    metadata = json.loads((run/'run.json').read_text(encoding='utf-8'))
    commands_path = run/'commands.jsonl'
    finish_status = None
    if commands_path.is_file():
        commands = [json.loads(line) for line in commands_path.read_text(encoding='utf-8').splitlines() if line]
        finish = next((item for item in commands if item.get('kind') == 'finish'), None)
        finish_status = finish.get('status') if finish else None
    logs = list((run/'pool').glob('*.log'))
    if len(logs) != 1:
        raise ValueError('expected one simulator log')
    log_text = logs[0].read_text(encoding='utf-8', errors='replace')
    seg_events = []
    for line in log_text.splitlines():
        match = SEG.search(line)
        if match:
            row = {key: (int(value) if key not in ('actor', 'name') else value)
                   for key, value in match.groupdict().items()}
            seg_events.append(row)
    state_events = []
    for line in log_text.splitlines():
        match = STATE.search(line)
        if match:
            state_events.append({key: int(value) for key, value in match.groupdict().items()})
    if not seg_events:
        raise ValueError('no SEG events in simulator log')
    score_values = [row['score'] for row in seg_events]
    increments = []
    previous = 0
    for row in seg_events:
        if row['score'] != previous:
            increments.append({'from': previous, 'to': row['score'], **row})
            previous = row['score']
    gates = DetectionGateCandidates.load(GATE_MANIFEST, axis=0, half_extents_cm=(200, 200, 200))
    candidate_indices = {record['index'] for name, records in gates.groups if name == args.group for record in records}
    observed_indices = sorted({row['index'] for row in seg_events})
    result = {
        'status': 'official score/SEG trace during diagnostic candidate-detection-gate path; not complete G2',
        'player_status': finish_status or ('completed' if metadata.get('player_exit_status') == 0 else metadata.get('player_exit_status')),
        'player_exit_status': metadata.get('player_exit_status'),
        'commands_file_present': commands_path.is_file(),
        'score_start': score_values[0] if score_values else None,
        'score_end': score_values[-1] if score_values else None,
        'score_increment_count': len(increments),
        'score_increments': increments,
        'seg_events': seg_events,
        'state_events': state_events,
        'observed_gate_indices': observed_indices,
        'observed_indices_in_candidate_manifest': [index for index in observed_indices if index in candidate_indices],
        'source_sha256': {
            'run.json': digest(run/'run.json'),
            **({'commands.jsonl': digest(commands_path)} if commands_path.is_file() else {}),
            'capture.jsonl': digest(run/'capture.jsonl'),
            'simulator_log': digest(logs[0]),
        },
        'gate_manifest_sha256': gates.manifest_sha256,
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'GDB all-stop score probe changes timing; score events are not a policy or race result',
            'score increments are not one-to-one with candidate gate indices',
            'the candidate gate index mapping comes from the runtime score audit, not decoded names',
            'one seed and one diagnostic path; other gates and road3 are not covered',
            'score semantics, stage transitions and factory logic remain unverified',
        ],
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('score', result['score_start'], '->', result['score_end'],
          'increments', result['score_increment_count'],
          'observed', observed_indices)


if __name__ == '__main__':
    main()
