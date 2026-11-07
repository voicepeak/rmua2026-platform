"""Check qualitative counter behavior in a historical predicate trace."""
import argparse
import hashlib
import json
from pathlib import Path


def analyze(path):
    raw = path.read_bytes()
    data = json.loads(raw)
    events = [e for e in data['events'] if e['state'] == 1]
    if len(events) < 3:
        raise ValueError('insufficient stage-one predicate transitions')
    windows = []
    for a, b in zip(events, events[1:]):
        dt = b['monotonic_s'] - a['monotonic_s']
        change = b['out_timer'] - a['out_timer']
        if dt <= 0 or a['predicate'] not in (0, 1) or b['predicate'] == a['predicate']:
            raise ValueError('invalid trace transitions')
        windows.append({'predicate_between_transitions': a['predicate'], 'host_monotonic_s': dt,
                        'timer_change_s': change, 'timer_start_s': a['out_timer'],
                        'timer_end_s': b['out_timer']})
    return {'source_sha256': hashlib.sha256(raw).hexdigest(), 'transition_windows': windows,
            'zero_predicate_timer_increases': all(x['timer_change_s'] > 0 for x in windows if x['predicate_between_transitions'] == 0),
            'one_predicate_timer_held': all(abs(x['timer_change_s']) < 1e-6 for x in windows if x['predicate_between_transitions'] == 1),
            'limitations': ['predicate transition observations are not full tick-by-tick input',
                            'stored flag changes after predicate read; per-tick timing is unobserved',
                            'host monotonic time and game time are not calibrated',
                            'only one stage-one raised-connector run; all-stop debugger changes timing',
                            'no inference of spatial boundary, stage reset or thrust degradation']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('summary', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    result = analyze(args.summary)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('windows', len(result['transition_windows']), 'increases', result['zero_predicate_timer_increases'],
          'holds', result['one_predicate_timer_held'])


if __name__ == '__main__':
    main()
