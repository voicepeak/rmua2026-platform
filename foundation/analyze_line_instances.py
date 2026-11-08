"""Decode candidate FTransform fields from instrumented line submissions.

Interpretation of the 48-byte layout is a candidate, not independently
established world transforms or physical collision instances.
"""
import argparse
from collections import Counter
import hashlib
import json
import re
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


PATTERN = re.compile(r'^LINE_INSTANCE component=(0x[0-9a-f]+) selector=(\d+) '
                     r'transform_hex=([0-9a-f]{96})\s+floats=.*$')
RESULT = re.compile(r'^LINE_INSTANCE_RESULT index=(-?\d+)$')


def parse(lines):
    rows, pending = [], None
    for line in lines:
        if line.startswith('LINE_INSTANCE component='):
            match = PATTERN.fullmatch(line.strip())
            if not match or pending is not None:
                raise ValueError('malformed or nested instance submission')
            component, selector, raw = match.groups()
            if selector not in ('0', '1'):
                raise ValueError('unexpected component selector')
            values = np.array(struct.unpack('<12f', bytes.fromhex(raw)))
            q, xyz, scale = values[:4], values[4:7], values[8:11]
            if (not np.isfinite(values).all() or abs(np.linalg.norm(q)-1) > .001
                    or np.any(scale <= 0) or abs(values[7]) > 1e-6 or abs(values[11]) > 1e-6):
                raise ValueError('invalid candidate transform layout')
            up = Rotation.from_quat(q).apply([0, 0, 1])
            tilt = float(np.rad2deg(np.arccos(np.clip(abs(up[2]), 0, 1))))
            pending = {'component': component, 'selector': int(selector),
                       'asset': 'thickline' if selector == '1' else 'thinline',
                       'raw_hex': raw, 'quaternion_xyzw': q.tolist(),
                       'translation_candidate_cm': xyz.tolist(), 'scale_candidate': scale.tolist(),
                       'axis_tilt_from_vertical_deg': tilt}
        elif line.startswith('LINE_INSTANCE_RESULT '):
            match = RESULT.fullmatch(line.strip())
            if not match or pending is None:
                raise ValueError('unmatched instance return')
            pending['return_index'] = int(match.group(1))
            rows.append(pending)
            pending = None
    if pending is not None or not rows:
        raise ValueError('missing instance returns')
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('manifest', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    log, manifest_raw = a.log.read_bytes(), a.manifest.read_bytes()
    manifest = json.loads(manifest_raw)
    if manifest.get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary hash')
    match = re.fullmatch(r'seed(\d+)_repeat(\d+)', a.log.parent.name)
    if not match:
        raise ValueError('cannot determine run identity')
    seed, repeat = map(int, match.groups())
    run = [r for r in manifest['runs'] if r['seed'] == seed and r['repeat'] == repeat]
    if len(run) != 1 or run[0].get('status') != 'observed' or not run[0].get('injection_observed'):
        raise ValueError('manifest does not establish injected run')
    rows = parse(log.decode('utf-8', 'replace').splitlines())
    good = [x for x in rows if x['return_index'] >= 0]
    summary = {'count': len(rows), 'nonnegative_returns': len(good),
               'asset_counts': dict(Counter(x['asset'] for x in good)),
               'scale_counts': dict(Counter(str(x['scale_candidate']) for x in good)),
               'axis_tilt_deg_min_max': [min(x['axis_tilt_from_vertical_deg'] for x in good),
                                          max(x['axis_tilt_from_vertical_deg'] for x in good)] if good else None}
    result = {'status': 'candidate submitted instance transforms; not verified collision or world coordinates',
              'source_sha256': {'log': hashlib.sha256(log).hexdigest(),
                                'manifest': hashlib.sha256(manifest_raw).hexdigest()},
              'stage_injected': manifest.get('injected_stage'), 'seed': seed, 'repeat': repeat,
              'summary': summary, 'instances': rows,
              'limitations': ['instrumented stage injection rather than natural progression',
                              '48-byte FTransform interpretation is candidate, component/world relation unverified',
                              'nonnegative instance index does not establish collision or runtime contact',
                              'quaternion and scale axis meanings not independently calibrated']}
    a.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('stage', result['stage_injected'], 'seed', seed, 'summary', summary)


if __name__ == '__main__':
    main()
