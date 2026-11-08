"""Compare two static road placement calculations; not runtime validation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.rack_placements import build


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map', type=Path)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    raw = args.map.read_bytes()
    blueprint_files = sorted((args.directory/'blueprints').glob('*.json'))
    blueprints = {f.stem: json.loads(f.read_bytes()) for f in blueprint_files}
    derived = build(json.loads(raw), blueprints, 'BP_Segment_Rot_C', 'Segment')
    old_path = args.directory/'OL1_road_placements_candidate.json'
    old_raw = old_path.read_bytes()
    old_rows = json.loads(old_raw)['instances']
    old = {x['actor']: x for x in old_rows}
    if len(old) != len(old_rows):
        raise ValueError('duplicate actor names in original placements')
    rows = []
    for candidate in derived['placements']:
        original = old[candidate['actor']]
        distance = np.linalg.norm(np.asarray(candidate['translation_ue_cm'])-original['translation_ue_cm'])
        angle = (Rotation.from_quat(candidate['quaternion_xyzw']).inv()
                 * Rotation.from_quat(original['quaternion_xyzw'])).magnitude()
        rows.append({'actor': candidate['actor'], 'position_difference_cm': float(distance),
                     'rotation_difference_rad': float(angle)})
    result = {'status': 'static calculation consistency only; runtime unverified',
              'source_sha256': {'map': hashlib.sha256(raw).hexdigest(),
                                'original_placements': hashlib.sha256(old_raw).hexdigest(),
                                'blueprints': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                               for f in blueprint_files}},
              'unresolved': derived['unresolved'], 'compared': len(rows),
              'max_position_difference_cm': max((r['position_difference_cm'] for r in rows), default=None),
              'max_rotation_difference_rad': max((r['rotation_difference_rad'] for r in rows), default=None),
              'rows': rows}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('compared', len(rows), 'unresolved', len(derived['unresolved']),
          'max position cm', result['max_position_difference_cm'],
          'max rotation rad', result['max_rotation_difference_rad'])


if __name__ == '__main__':
    main()
