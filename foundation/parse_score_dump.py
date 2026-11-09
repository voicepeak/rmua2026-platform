"""Parse a score_dump GDB log into a runtime detection-component manifest.

The log is produced by foundation/probes/score_dump.gdb or map_score_dump*.gdb.
Component roots are read from the native routing object's +0x240 pointer array
and +0x248 count.  The manifest is a runtime candidate layer, not an official
trigger aperture or scoring equivalence.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

DUMP_BEGIN = re.compile(r'DUMP_BEGIN actor=(0x[0-9a-f]+) count=(\d+) arr=(0x[0-9a-f]+)')
DUMP_ITEM = re.compile(
    r'DUMP_ITEM actor=(0x[0-9a-f]+) idx=(\d+) component=(0x[0-9a-f]+) '
    r'xyz_cm=([-\d.e+]+),([-\d.e+]+),([-\d.e+]+) '
    r'quat=([-\d.e+]+),([-\d.e+]+),([-\d.e+]+),([-\d.e+]+) '
    r'scale=([-\d.e+]+),([-\d.e+]+),([-\d.e+]+)')
SEG = re.compile(r'SEG actor=(0x[0-9a-f]+)(?: name=[^ ]+)?(?: idx=|.*? idx=)(\d+) enter=(\d+) event=(\d+)(?: components=\d+)? score=(\d+)')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse(log_path):
    groups = {}
    events = []
    for line in Path(log_path).read_text(encoding='utf-8', errors='replace').splitlines():
        match = DUMP_BEGIN.search(line)
        if match:
            actor = match.group(1)
            groups.setdefault(actor, {'count': int(match.group(2)), 'array': match.group(3), 'components': []})
            continue
        match = DUMP_ITEM.search(line)
        if match:
            actor = match.group(1)
            group = groups.setdefault(actor, {'count': None, 'array': None, 'components': []})
            group['components'].append({
                'index': int(match.group(2)),
                'component': match.group(3),
                'position_cm': [float(match.group(i)) for i in (4, 5, 6)],
                'quaternion_xyzw': [float(match.group(i)) for i in (7, 8, 9, 10)],
                'scale': [float(match.group(i)) for i in (11, 12, 13)],
            })
            continue
        match = SEG.search(line)
        if match:
            events.append({
                'actor': match.group(1),
                'index': int(match.group(2)),
                'enter': int(match.group(3)),
                'event': int(match.group(4)),
                'score': int(match.group(5)),
            })
    for group in groups.values():
        group['components'].sort(key=lambda item: item['index'])
    increments = []
    previous = 0
    for event in events:
        if event['score'] > previous:
            increments.append({'from': previous, 'to': event['score'], **event})
            previous = event['score']
    return {
        'status': 'runtime detection-component roots; trigger aperture and official scoring unverified',
        'source_log': str(log_path).replace('\\', '/'),
        'source_sha256': digest(log_path),
        'actors': groups,
        'actor_count': len(groups),
        'declared_component_count': sum(int(group['count'] or 0) for group in groups.values()),
        'component_count': sum(len(group['components']) for group in groups.values()),
        'score_start': 0,
        'score_end': increments[-1]['to'] if increments else 0,
        'score_increment_count': len(increments),
        'score_increments': increments,
        'seg_events': events,
        'observed_indices': sorted({event['index'] for event in events}),
        'limitations': [
            'GDB all-stop changes timing; this is a diagnostic trace, not a race result',
            'component roots do not establish trigger plane, aperture or extent',
            'the 12 named *_Gate actors are solid route gates, not these components',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--map', default=None)
    args = parser.parse_args()
    result = parse(args.log)
    if args.map:
        result['map'] = args.map
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1) + '\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in
                      ('actor_count', 'declared_component_count', 'component_count', 'score_end',
                       'score_increment_count', 'observed_indices')}, indent=1))


if __name__ == '__main__':
    main()
