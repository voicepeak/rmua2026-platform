"""Compare fresh-seed wind generator output with historical debugger captures."""
import argparse
import hashlib
import json
import re
from pathlib import Path
from foundation.wind_generator import WindGenerator


def validate(root, elf):
    cases = []
    for stage in (4, 5, 6):
        for seed in (123, 456, 2026):
            generator = WindGenerator.fresh(stage, seed, elf)
            expected = generator.keyframe_bytes()
            for repeat in (1, 2):
                path = root / f'stage_inject_{stage}_20261104' / f'seed{seed}_repeat{repeat}' / 'gdb.log'
                raw = path.read_bytes()
                matches = re.findall(rb'WIND_ITEM\s+\d+ hex=([0-9a-f]{32})', raw)
                observed = b''.join(bytes.fromhex(x.decode()) for x in matches)
                cases.append({'stage': stage, 'seed': seed, 'repeat': repeat,
                              'source_sha256': hashlib.sha256(raw).hexdigest(),
                              'observed_items': len(matches), 'generated_items': len(generator.frames),
                              'exact': bool(matches) and observed == expected})
    return {'scope': 'injected stage fresh-seed keyframes; no prior libc rand consumers',
            'limitations': ['not natural stage random-stream validation', 'no wind force or zone validation',
                            'debugger injections alter execution and may affect timing'], 'cases': cases}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('experiments', type=Path)
    p.add_argument('elf', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    data = validate(a.experiments, a.elf)
    a.output.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    print('exact:', sum(c['exact'] for c in data['cases']), '/', len(data['cases']))
    if not all(c['exact'] for c in data['cases']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
