"""Compare native pre-rotation drag wrench against the six-face candidate."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.aero import DragFaces

PATTERN = re.compile(r'AERO_NET mono=([\d.]+) density=([^ ]+) force_body=([^ ]+) torque_body=([^ ]+) '
                     r'velocity=([^ ]+) wind=([^ ]+) omega=([^ ]+) quat=([^\s]+)')


def analyze(run, snapshot):
    log_path, = (run/'pool').glob('*.log')
    raw = log_path.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    if any(x in text for x in ('AERO_NET_ERROR', 'WIND_INJECT_ERROR')):
        raise ValueError('probe error')
    model = DragFaces.from_observed_snapshot(snapshot, air_density=1.0, min_velocity=0.10000000149011612)
    rows = []
    for m in PATTERN.finditer(text):
        vec = lambda i: np.asarray([float(v) for v in m[i].split(',')])
        density = float(m[2])
        observed_force, observed_torque, velocity, wind, omega, quat = (vec(i) for i in (3,4,5,6,7,8))
        if (any(v.shape != (3,) for v in (observed_force, observed_torque, velocity, wind, omega))
                or quat.shape != (4,) or not np.isfinite(np.r_[observed_force, observed_torque, velocity, wind, omega, quat, density]).all()
                or density <= 0 or not np.allclose(wind, [3,0,0], atol=1e-6)):
            raise ValueError('invalid probe values')
        model.air_density = density
        rotation = Rotation.from_quat(quat)
        world_force, torque = model.wrench(rotation, velocity, omega, wind)
        force = rotation.inv().apply(world_force)
        rows.append({'monotonic_s': float(m[1]), 'density': density, 'velocity_world': velocity.tolist(),
                     'wind_world': wind.tolist(), 'omega_body': omega.tolist(), 'quaternion_xyzw': quat.tolist(),
                     'native_force_body': observed_force.tolist(), 'candidate_force_body': force.tolist(),
                     'native_torque_body': observed_torque.tolist(), 'candidate_torque_body': torque.tolist(),
                     'force_error_norm': float(np.linalg.norm(force-observed_force)),
                     'torque_error_norm': float(np.linalg.norm(torque-observed_torque))})
    if not rows:
        raise ValueError('no native net wrench samples')
    return {'samples': rows, 'max_force_error': max(r['force_error_norm'] for r in rows),
            'max_torque_error': max(r['torque_error_norm'] for r in rows),
            'source_sha256': {'log': hashlib.sha256(raw).hexdigest(), 'faces': model.snapshot_sha256},
            'limitations': ['debugger injects wind at physics entry; natural wind path not tested',
                            'native wrench sampled before world-force rotation',
                            'candidate parameters from a separate seed snapshot; density sampled from same update',
                            'debugger affects timing; startup and contact are not excluded',
                            'matching instantaneous wrench does not validate trajectory integration']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('snapshot', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = analyze(a.run, a.snapshot)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('samples', len(result['samples']), 'max force', result['max_force_error'],
          'max torque', result['max_torque_error'])


if __name__ == '__main__':
    main()
