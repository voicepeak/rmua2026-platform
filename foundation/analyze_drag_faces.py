"""Decode read-only candidate physics-body drag vertices from GDB snapshots."""
import argparse
import hashlib
import json
import math
import re
import struct
from pathlib import Path


def analyze(directory):
    log_path, = (directory/'pool').glob('*.log')
    raw = log_path.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    match = re.search(r'DRAG_ARRAY mono=([\d.]+) body=(0x[0-9a-f]+) begin=(0x[0-9a-f]+) count=(\d+) stride=(\d+)', text)
    if not match or int(match[4]) != 6 or int(match[5]) != 128 or 'DRAG_READ_ERROR' in text:
        raise ValueError('missing or unexpected drag array')
    rows = list(re.finditer(r'DRAG_FACE index=(\d+) hex=([0-9a-f]{256})', text))
    if [int(x[1]) for x in rows] != list(range(6)):
        raise ValueError('incomplete drag array')
    faces = []
    for row in rows:
        data = bytes.fromhex(row[2])
        vec = lambda off: list(struct.unpack_from('<3f', data, off))
        position, current_position = vec(0x30), vec(0x3c)
        normal, current_normal = vec(0x48), vec(0x54)
        factor = struct.unpack_from('<f', data, 0x78)[0]
        if not all(math.isfinite(x) for x in position+normal+[factor]) or factor < 0:
            raise ValueError('invalid candidate initial drag values')
        initialized = (all(math.isfinite(x) for x in current_position+current_normal) and
                       all(abs(a-b) < 1e-5 for a,b in zip(position,current_position)) and
                       all(abs(a-b) < 1e-5 for a,b in zip(normal,current_normal)))
        faces.append({'index': int(row[1]), 'initial_position_m': position, 'initial_normal': normal,
                      'candidate_drag_factor': factor, 'current_matches_initial': initialized})
    pool_path = directory/'pool'/'pool.json'
    pool = json.loads(pool_path.read_text())
    return {'capture_monotonic_s': float(match[1]), 'body_ptr': match[2], 'faces': faces,
            'binary_sha256': pool['binary_sha256'], 'settings_sha256': pool['settings_sha256'],
            'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (log_path, pool_path)},
            'limitations': ['offset interpretation based on rotor reset disassembly and upstream PhysicsBodyVertex layout',
                            'initial values are not by themselves proof of applied wind force',
                            'density, threshold, runtime integration and wind zone remain independently unverified',
                            'one snapshot in a debugger-instrumented process']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.run)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('faces',len(result['faces']),'initialized',sum(f['current_matches_initial'] for f in result['faces']))
    print([(x['initial_position_m'],x['initial_normal'],x['candidate_drag_factor']) for x in result['faces']])


if __name__ == '__main__':
    main()
