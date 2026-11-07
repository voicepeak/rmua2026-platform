"""Check the native per-face drag scalar against captured loop operands."""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path
import numpy as np

SAMPLE = re.compile(r'AERO_FORCE mono=([\d.]+) face=(0x[0-9a-f]+) position=([^ ]+) normal=([^ ]+) '
                    r'factor=([^ ]+) density=([^ ]+) component=([^ ]+) scalar=([^\s]+)')


def analyze(directory, binary):
    log_path, = (directory/'pool').glob('*.log')
    raw = log_path.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    if 'AERO_FORCE_ERROR' in text:
        raise ValueError('probe error')
    binary_hash = hashlib.file_digest(binary.open('rb'), 'sha256').hexdigest()
    if binary_hash != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected simulator binary')
    # This ELF's verified first load segment maps file offset zero to 0x200000.
    with binary.open('rb') as f:
        f.seek(0x6c0e44-0x200000)
        cutoff = struct.unpack('<f', f.read(4))[0]
    rows = []
    for m in SAMPLE.finditer(text):
        factor, density, speed, actual = map(float, (m[5], m[6], m[7], m[8]))
        pos = [float(x) for x in m[3].split(',')]
        normal = [float(x) for x in m[4].split(',')]
        if (len(pos) != 3 or len(normal) != 3 or not np.isfinite(pos+normal+[factor, density, speed, actual]).all()
                or speed <= cutoff or factor < 0 or density <= 0):
            raise ValueError('invalid observed drag sample')
        expected = np.float32(-np.float32(factor) * np.float32(density))
        expected = np.float32(expected * np.float32(speed))
        expected = np.float32(expected * np.float32(speed))
        rows.append({'monotonic_s': float(m[1]), 'face_ptr': m[2], 'position': pos, 'normal': normal,
                     'factor': factor, 'density_operand': density, 'normal_speed': speed,
                     'native_scalar': actual, 'expected_scalar': float(expected),
                     'absolute_error': abs(float(expected)-actual)})
    if not rows:
        raise ValueError('no drag samples')
    return {'binary_sha256': binary_hash, 'log_sha256': hashlib.sha256(raw).hexdigest(),
            'cutoff_m_s': cutoff, 'samples': rows,
            'max_scalar_abs_error': max(x['absolute_error'] for x in rows),
            'density_operand_range': [min(x['density_operand'] for x in rows), max(x['density_operand'] for x in rows)],
            'limitations': ['loop operands and scalar output observed, not complete net wind wrench',
                            'wind input and velocity frame were not independently captured in this probe',
                            'single process with all-stop debugger, startup and ground contact possible',
                            'density operand source and natural wind zones require separate verification']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('binary', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.run, a.binary)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('samples',len(result['samples']),'cutoff',result['cutoff_m_s'],
          'density range',result['density_operand_range'],'max abs error',result['max_scalar_abs_error'])


if __name__ == '__main__':
    main()
