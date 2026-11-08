"""Match serialized gate candidates to startup runtime root observations."""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


PATTERN = re.compile(
    r'^RMUA_GATE actor=(\S+) fname_index=(\d+) fname_number=(\d+) root=(\S+) '
    r'xyz_cm=([^ ]+) quat=([^ ]+) scale=([^\s]+)$')


def vectors(text, size):
    value = np.asarray([float(x) for x in text.split(',')], float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError('invalid observed gate vector')
    return value


def compare(candidate, lines):
    objects = candidate['placements']
    if candidate['unresolved'] or not objects:
        raise ValueError('missing or unresolved candidates')
    observations = []
    for line in lines:
        if not line.startswith('RMUA_GATE '):
            continue
        match = PATTERN.fullmatch(line.strip())
        if not match:
            raise ValueError('unsupported gate observation')
        actor, index, number, root, position, quat, scale = match.groups()
        observations.append({'actor': actor, 'root': root, 'fname': [int(index), int(number)],
                             'position': vectors(position, 3), 'quat': vectors(quat, 4),
                             'scale': vectors(scale, 3)})
    if len(observations) != len(objects) or len({x['actor'] for x in observations}) != len(observations):
        raise ValueError('gate observation count or actor identity mismatch')
    positions = np.array([x['translation_ue_cm'] for x in objects], float)
    observed = np.array([x['position'] for x in observations])
    if not np.isfinite(positions).all():
        raise ValueError('invalid candidate position')
    distances = np.linalg.norm(positions[:, None, :] - observed[None, :, :], axis=2)
    ii, jj = linear_sum_assignment(distances)
    rows = []
    for i, j in zip(ii, jj):
        obj, obs = objects[i], observations[j]
        angle = np.rad2deg((Rotation.from_quat(obj['quaternion_xyzw']).inv()
                            * Rotation.from_quat(obs['quat'])).magnitude())
        scale_error = np.max(np.abs(np.array(obj['scale']) - obs['scale']))
        nearest = np.sort(distances[i])
        rows.append({'candidate_actor': obj['actor'], 'runtime_actor': obs['actor'],
                     'runtime_root': obs['root'], 'runtime_fname': obs['fname'],
                     'position_error_cm': float(distances[i, j]),
                     'second_nearest_distance_cm': float(nearest[1]) if len(nearest) > 1 else None,
                     'rotation_error_deg': float(angle), 'max_scale_error': float(scale_error)})
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('candidates', type=Path)
    p.add_argument('log', type=Path)
    p.add_argument('manifest', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    data = [p.read_bytes() for p in (a.candidates, a.log, a.manifest)]
    manifest = json.loads(data[2])
    if manifest.get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected official binary')
    run = None
    if 'runs' in manifest:
        match = re.fullmatch(r'seed(\d+)_repeat(\d+)', a.log.parent.name)
        if not match:
            raise ValueError('run path does not identify seed and repeat')
        seed, repeat = map(int, match.groups())
        matching = [r for r in manifest['runs'] if r.get('seed') == seed and r.get('repeat') == repeat]
        if len(matching) != 1 or matching[0].get('status') != 'observed':
            raise ValueError('manifest does not identify an observed run')
        run = {'seed': seed, 'repeat': repeat, 'status': matching[0]['status']}
    else:
        run = {'seed': manifest.get('seed'), 'status': manifest.get('status')}
    rows = compare(json.loads(data[0]), data[1].decode('utf-8', 'replace').splitlines())
    result = {'status': 'runtime root transform correspondence, not collision or trigger acceptance',
              'source_sha256': {p.name: hashlib.sha256(raw).hexdigest()
                                for p, raw in zip((a.candidates, a.log, a.manifest), data)},
              'binary_sha256_reported_by_manifest': manifest['binary_sha256'],
              'run': run, 'count': len(rows),
              'max_position_error_cm': max(x['position_error_cm'] for x in rows),
              'max_rotation_error_deg': max(x['rotation_error_deg'] for x in rows),
              'max_scale_error': max(x['max_scale_error'] for x in rows), 'matches': rows,
              'limitations': ['one historical process and startup samples, all-stop GDB',
                              'identity assigned by minimum-distance matching; runtime FNames not decoded',
                              'root transform comparison does not prove collision geometry or scoring trigger',
                              'printed values have limited decimal precision']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('gates', len(rows), 'max cm', result['max_position_error_cm'],
          'max degrees', result['max_rotation_error_deg'], 'max scale', result['max_scale_error'])


if __name__ == '__main__':
    main()
