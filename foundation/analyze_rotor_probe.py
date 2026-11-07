"""Summarize read-only native rotor observations; debugger affects timing."""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np

PATTERN = re.compile(r'ROTOR mono=([\d.]+) ptr=(0x[0-9a-f]+).*?fields_b8=([^\s]+)')


def analyze(path):
    raw = path.read_bytes()
    rows = []
    for match in PATTERN.finditer(raw.decode('utf-8', errors='replace')):
        values = np.array([float(s) for s in match[3].split(',')])
        if len(values) != 30:
            raise ValueError('unexpected rotor field count')
        # +0xe8, +0xec, +0x118, +0x128, +0x12c, relative to rotor.
        rows.append((match[2], *values[[12, 13, 24, 28, 29]]))
    if not rows:
        raise ValueError('no native rotor observations')
    by_rotor = {}
    for rotor in sorted({r[0] for r in rows}):
        values = np.array([r[1:] for r in rows if r[0] == rotor])
        valid = np.isfinite(values).all(axis=1) & (values[:, 3] > .01)
        ratios = values[valid, 2] / values[valid, 3]
        if not len(ratios):
            raise ValueError('no positive filtered output')
        by_rotor[rotor] = {'samples': len(values), 'e8_range': [float(values[:, 0].min()), float(values[:, 0].max())],
                           'last_filtered_range': [float(values[:, 3].min()), float(values[:, 3].max())],
                           'thrust_over_last_filtered_median': float(np.median(ratios)),
                           'thrust_over_last_filtered_max_deviation': float(np.max(np.abs(ratios-np.median(ratios))))}
    return {'source_sha256': hashlib.sha256(raw).hexdigest(), 'rotors': by_rotor,
            'limitations': ['GDB all-stop breakpoints alter simulator timing',
                            'addresses and offsets are specific to this executable build',
                            'observation is before each update; 0x128 and 0x118 are prior outputs',
                            'one short run does not validate the whole physics or controller bridge']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.log)
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result['rotors'], indent=2))


if __name__ == '__main__':
    main()
