"""Summarize controlled field-flag rotor samples, without zone claims."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np

ROTOR = re.compile(r'FIELD_ROTOR mono=([\d.]+) rotor=(0x[0-9a-f]+) filtered=([^ ]+) '
                   r'(?:target=([^ ]+) )?input=([^ ]+) thrust=([^ ]+) output=([^\s]+)')
INJECT = re.compile(r'FIELD_INJECT mono=([\d.]+) state=(0x[0-9a-f]+) natural=(\d+) forced=(\d+) config=(0x[0-9a-f]+) count=(\d+)')


def analyze(directory):
    log_path, = (directory/'pool').glob('*.log')
    commands_path = directory/'commands.jsonl'
    log = log_path.read_bytes()
    text = log.decode('utf-8', errors='replace')
    events = [json.loads(s) for s in commands_path.read_text().splitlines() if s]
    edges = []
    for row in events:
        if row.get('kind') == 'command' and (not edges or row['pwm'] != edges[-1]['pwm']):
            edges.append(row)
    if len(edges) < 4:
        raise ValueError('missing waveform edges')
    samples = []
    for m in ROTOR.finditer(text):
        samples.append((float(m[1]), [float(m[3]), None if m[4] is None else float(m[4]),
                                      float(m[5]), float(m[6]), float(m[7])]))
    injections = list(INJECT.finditer(text))
    if not samples or not injections or 'FIELD_INJECT_ERROR' in text or 'FIELD_ROTOR_ERROR' in text:
        raise ValueError('missing samples or probe error')
    stages = []
    for i in (1, 2):
        start, end = edges[i]['monotonic_ns']*1e-9+.12, edges[i+1]['monotonic_ns']*1e-9
        rows = [x[1] for x in samples if start <= x[0] < end]
        if len(rows) < 3:
            raise ValueError('insufficient settled rotor samples')
        columns = list(zip(*rows))
        stages.append({'command': edges[i]['pwm'], 'samples': len(rows),
                       'median': {key: (None if values[0] is None else float(np.median(values)))
                                  for key, values in zip(('filtered', 'target', 'input', 'thrust', 'output'), columns)}})
    return {'flag_values': sorted({int(m[4]) for m in injections}),
            'natural_values_observed': sorted({int(m[3]) for m in injections}),
            'injection_log_count': len(injections), 'stages': stages,
            'source_sha256': {str(p.name): hashlib.sha256(p.read_bytes()).hexdigest() for p in (log_path, commands_path)}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', nargs='+', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = {'runs': {str(path): analyze(path) for path in a.runs},
              'limitations': ['predicate result is forced every tick and overtime disabled in both conditions',
                              'GDB all-stop changes timing; no natural region boundary or transition delay measured',
                              'one rotor sampled; other rotors and varied commands require validation',
                              'time windows use host monotonic time; debug input is not proven physical actuator input',
                              'ground contact confounds IMU and trajectory']}
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    for name, row in result['runs'].items():
        print(name, 'flag', row['flag_values'], 'stages', [x['median'] for x in row['stages']])


if __name__ == '__main__':
    main()
