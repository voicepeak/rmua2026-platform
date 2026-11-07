"""Summarize per-rotor injected-flag transitions; not a natural boundary test."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np

PATTERN = re.compile(r'TOGGLE_ROTOR mono=([\d.]+) rotor=(0x[0-9a-f]+) flag=(\d+) '
                     r'filtered=([^ ]+) target=([^ ]+) input=([^ ]+) thrust=([^\s]+)')
TOGGLE = re.compile(r'TOGGLE mono=([\d.]+) start=([^ ]+) elapsed=([^ ]+) natural=(\d+) forced=(\d+) config=(0x[0-9a-f]+)')


def analyze(directory):
    log_path, = (directory/'pool').glob('*.log')
    cmd_path = directory/'commands.jsonl'
    text = log_path.read_text(encoding='utf-8', errors='replace')
    if 'TOGGLE_ERROR' in text or 'TOGGLE_ROTOR_ERROR' in text:
        raise ValueError('debugger probe error')
    first = json.loads(cmd_path.read_text(encoding='utf-8').splitlines()[0])
    if first.get('kind') != 'start':
        raise ValueError('missing start marker')
    start = first['monotonic_ns']*1e-9
    rows = [(float(m[1])-start, m[2], int(m[3]), *[float(m[i]) for i in range(4, 8)])
            for m in PATTERN.finditer(text)]
    transitions = [{'elapsed_s': float(m[1])-start, 'natural': int(m[4]), 'forced': int(m[5])}
                   for m in TOGGLE.finditer(text) if m[2] != 'None']
    if [x['forced'] for x in transitions] != [0, 1]:
        raise ValueError('expected two injected transitions')
    ids = sorted({x[1] for x in rows})
    if len(ids) != 4:
        raise ValueError('expected four observed rotors')
    windows = [(.25, .45, 1), (.70, 1.05, 0), (1.35, 1.60, 1)]
    rotors = {}
    for rotor in ids:
        phases = []
        for low, high, flag in windows:
            values = np.array([x[3:] for x in rows if x[1] == rotor and low <= x[0] < high and x[2] == flag])
            if len(values) < 2 or not np.isfinite(values).all():
                raise ValueError('insufficient valid rotor samples')
            phases.append({'window_s': [low, high], 'flag': flag, 'samples': len(values),
                           'medians': dict(zip(('filtered', 'target', 'input', 'thrust'),
                                               map(float, np.median(values, axis=0))))})
        rotors[rotor] = phases
    return {'transitions': transitions, 'rotors': rotors,
            'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (log_path, cmd_path)},
            'limitations': ['predicate forced on a host-clock schedule; overtime suppression enabled',
                            'ground-start constant command; no natural boundary or free-flight response',
                            'all-stop GDB and 60 ms rotor sampling do not establish physical delay',
                            'pointer ordering does not identify motor numbering',
                            'one seed and one input amplitude']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.run)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('rotors', len(result['rotors']), 'transitions', result['transitions'])
    print('first rotor medians', next(iter(result['rotors'].values())))


if __name__ == '__main__':
    main()
