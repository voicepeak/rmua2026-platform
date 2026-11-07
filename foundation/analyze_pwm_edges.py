"""Measure host-arrival command-to-debug PWM changes, not motor response."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from foundation.analyze_collective import read, pwm_value


def analyze(directory):
    command_file, capture_file = directory/'commands.jsonl', directory/'capture.jsonl'
    commands = [x for x in read(command_file) if x.get('kind') == 'command']
    output = [x for x in read(capture_file) if x.get('topic') == 'rotor_pwm']
    if not commands or not output:
        raise ValueError('missing command or debug PWM data')
    edges = []
    for row in commands:
        if not edges or row['pwm'] != edges[-1]['pwm']:
            edges.append(row)
    decoded = [(x['arrival_monotonic_ns'], np.array(pwm_value(x))) for x in output]
    rows = []
    for i, edge in enumerate(edges):
        start = edge['monotonic_ns']
        stop = edges[i+1]['monotonic_ns'] if i+1 < len(edges) else start + 500_000_000
        samples = [(t, v) for t, v in decoded if start <= t < stop]
        prior = [(t, v) for t, v in decoded if t < start]
        target = np.array(edge['pwm'])
        matches = [(t, v) for t, v in samples if np.all(np.abs(v-target) < 1e-4)]
        first = matches[0][0] if matches else None
        # Inference is bounded by asynchronous observation; no pre-edge
        # agreement is assumed and no physical response delay is inferred.
        rows.append({'command': target.tolist(), 'sample_count': len(samples),
                     'first_match_delay_ms': (first-start)/1e6 if first else None,
                     'prior_sample_age_ms': (start-prior[-1][0])/1e6 if prior else None,
                     'prior_value': prior[-1][1].tolist() if prior else None,
                     'matching_samples': len(matches)})
    return {'edges': rows, 'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                             for p in (command_file, capture_file)},
            'limitations': ['command time is recorded after publish, debug time is subscriber arrival',
                            'both use host monotonic clock but include scheduling and transport',
                            'debug PWM publication is not verified motor speed or physical actuation',
                            'short phases can contain too few samples to estimate delay']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.directory)
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('edges:', len(result['edges']), 'matched:', sum(e['first_match_delay_ms'] is not None for e in result['edges']))


if __name__ == '__main__':
    main()
