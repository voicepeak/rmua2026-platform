"""Sample training lines from instrumented candidate orientation/scale records.

The recorded transform is only interpreted as world-space when the caller
explicitly selects that assumption. This is not official spawn replay.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.obstacles import generate


def sample_lines(seed, records, assets, count, bounds_min, bounds_max, launch, exclusion_m,
                 *, frame_assumption, feasible):
    if frame_assumption != 'world_assumed':
        raise ValueError('explicit world_assumed interpretation required')
    if not isinstance(count, int) or count < 0 or not callable(feasible):
        raise ValueError('invalid count or missing feasibility predicate')
    candidates = [r for r in records if r.get('return_index', -1) >= 0
                  and r.get('asset') in ('thinline', 'thickline')]
    if not candidates and count:
        raise ValueError('no candidate submissions')
    rng = np.random.default_rng(seed)
    output = []
    for _ in range(count):
        row = candidates[int(rng.integers(len(candidates)))]
        scale = np.asarray(row['scale_candidate'], float)
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
            raise ValueError('invalid candidate scale')
        capsule = assets[row['asset']]['conservative_capsule']
        local = np.asarray(capsule['end_cm'], float)-np.asarray(capsule['start_cm'], float)
        axis_ue = Rotation.from_quat(row['quaternion_xyzw']).apply(local*scale)
        axis_ned = axis_ue*np.array([1, 1, -1])
        length_m = float(np.linalg.norm(axis_ned)/100)
        radius_m = float(capsule['radius_m']*max(scale[0], scale[1]))
        draw_seed = int(rng.integers(0, np.iinfo(np.int64).max))
        lines, _ = generate(draw_seed, bounds_min, bounds_max, launch, exclusion_m, 1, 0,
                            radius_m, length_m, [1, 1, 1], [0, 0],
                            line_directions=[axis_ned], feasible=feasible)
        output.append({'line': lines[0], 'asset': row['asset'],
                       'source_component': row['component'], 'source_index': row['return_index'],
                       'orientation_provenance': 'instrumented submission, world frame assumed'})
    return output


def load_submissions(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    return value['instances']
