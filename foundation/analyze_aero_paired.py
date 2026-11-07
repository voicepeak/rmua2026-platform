"""Compare native drag-face operands to the explicit relative-wind model."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

PATTERN = re.compile(r'AERO_PAIRED mono=([\d.]+) face=(0x[0-9a-f]+) position=([^ ]+) normal=([^ ]+) '
                     r'factor=([^ ]+) density=([^ ]+) component=([^ ]+) scalar=([^ ]+) '
                     r'velocity=([^ ]+) wind=([^ ]+) omega=([^ ]+) quat=([^\s]+)')


def analyze(directory, expected_wind):
    log_path, = (directory/'pool').glob('*.log')
    raw = log_path.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    if any(err in text for err in ('WIND_INJECT_ERROR', 'AERO_PAIRED_ERROR')) or 'WIND_INJECT ' not in text:
        raise ValueError('missing injection or probe error')
    rows = []
    for m in PATTERN.finditer(text):
        vector = lambda i: np.array([float(x) for x in m[i].split(',')])
        p,n,v,wind,omega,q = (vector(i) for i in (3,4,9,10,11,12))
        fac,density,component,scalar = map(float, (m[5],m[6],m[7],m[8]))
        if (any(x.shape != (3,) for x in (p,n,v,wind,omega)) or q.shape != (4,)
                or not np.isfinite(np.r_[p,n,v,wind,omega,q,fac,density,component,scalar]).all()
                or not np.allclose(wind,[expected_wind,0,0],atol=1e-6)):
            raise ValueError('unexpected or invalid probe values')
        body_v = Rotation.from_quat(q).inv().apply(v-wind)
        predicted_component = float(n @ (body_v + np.cross(omega,p)))
        predicted_scalar = -fac*density*predicted_component**2
        rows.append({'monotonic_s': float(m[1]), 'face': m[2], 'position': p.tolist(), 'normal': n.tolist(),
                     'factor': fac, 'density': density, 'velocity_world': v.tolist(), 'wind_world': wind.tolist(),
                     'omega_body': omega.tolist(), 'quaternion_xyzw': q.tolist(),
                     'component_native': component, 'component_predicted': predicted_component,
                     'scalar_native': scalar, 'scalar_predicted': predicted_scalar})
    if not rows:
        raise ValueError('no paired native samples')
    return {'samples': rows, 'expected_wind_x_m_s': expected_wind,
            'max_component_abs_error': max(abs(x['component_native']-x['component_predicted']) for x in rows),
            'max_scalar_abs_error': max(abs(x['scalar_native']-x['scalar_predicted']) for x in rows),
            'source_sha256': hashlib.sha256(raw).hexdigest(),
            'limitations': ['wind memory overwritten at internal physics entry, not natural-stage wind',
                            'one face sampled at a time; no simultaneous complete net wrench',
                            'quaternion and input argument layout are inferred from disassembly and tested by this comparison',
                            'GDB all-stop changes timing; startup/ground contact and field effects may affect motion']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--wind-x', type=float, required=True)
    a = p.parse_args()
    result = analyze(a.run,a.wind_x)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('samples',len(result['samples']),'component max error',result['max_component_abs_error'],
          'scalar max error',result['max_scalar_abs_error'])


if __name__ == '__main__':
    main()
