"""Record observed injected-predicate event and rotor-scale write ordering."""
import argparse
import hashlib
import json
import re
from pathlib import Path

PATTERN = re.compile(r'(?P<kind>TOGGLE|FIELD_EVENT_CONSUMER|SCALE_WRITE|TARGET_WRITE) mono=(?P<time>[\d.]+) (?P<body>[^\r\n]+)')


def analyze(log):
    raw = log.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    if any(error in text for error in ('FIELD_EVENT_CONSUMER_ERROR', 'TARGET_WATCH_ERROR', 'TOGGLE_ERROR')):
        raise ValueError('probe error')
    rows = []
    for m in PATTERN.finditer(text):
        fields = dict(re.findall(r'(\w+)=([^\s]+)', m['body']))
        rows.append({'kind': m['kind'], 'monotonic_s': float(m['time']), 'fields': fields})
    toggles = [r for r in rows if r['kind'] == 'TOGGLE' and r['fields'].get('elapsed') != 'None']
    if [int(r['fields']['forced']) for r in toggles] != [0, 1]:
        raise ValueError('missing injected 1→0→1 transitions')
    matches = []
    for i, toggle in enumerate(toggles):
        flag = int(toggle['fields']['forced'])
        stop = toggles[i+1]['monotonic_s'] if i+1 < len(toggles) else float('inf')
        candidates = [r for r in rows if toggle['monotonic_s'] <= r['monotonic_s'] < stop]
        events = [r for r in candidates if r['kind'] == 'FIELD_EVENT_CONSUMER' and r['fields'].get('type') == '30' and r['fields'].get('value') == str(flag)]
        writes = [r for r in candidates if r['kind'] == 'SCALE_WRITE' and r['fields'].get('flag') == str(flag)]
        target = [r for r in candidates if r['kind'] == 'TARGET_WRITE' and r['fields'].get('flag') == str(flag)]
        if not events or not writes or not target or not events[0]['monotonic_s'] <= writes[0]['monotonic_s'] <= target[0]['monotonic_s']:
            raise ValueError('expected matching event and writes were not observed in order')
        matches.append({'flag': flag, 'predicate_override': toggle, 'event_consumer': events[0],
                        'scale_write': writes[0], 'target_write': target[0]})
    return {'log_sha256': hashlib.sha256(raw).hexdigest(), 'matches': matches,
            'limitations': ['controlled predicate injection and timeout suppression, not natural boundary traversal',
                            'all-stop and hardware watchpoints perturb scheduling and measured latency',
                            'one rotor watched for writes and one seed in this trace',
                            'same-run timing and static control flow support the chain, not all runtime cases']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    result = analyze(args.log)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('observed matched flag transitions:', [x['flag'] for x in result['matches']])


if __name__ == '__main__':
    main()
