"""Match candidate runtime owned-component offsets to Blueprint defaults."""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np

from foundation.road_scene import value


PATTERN = re.compile(r'DRONE_OWNED_PTR actor=(0x[0-9a-f]+) actor_offset=(0x[0-9a-f]+) '
                     r'component=(0x[0-9a-f]+) fname=([^ ]+) class=(0x[0-9a-f]+) '
                     r'world_xyz=([^ ]+) world_quat=([^\s]+)')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('blueprint', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    logs = list((a.run/'pool').glob('*.log'))
    if len(logs) != 1:
        raise ValueError('expected one simulator log')
    log, raw_bp, raw_run = logs[0].read_bytes(), a.blueprint.read_bytes(), (a.run/'run.json').read_bytes()
    run = json.loads(raw_run)
    if run['exit_status'] != 0:
        raise ValueError('incomplete run')
    rows = {}
    for m in PATTERN.finditer(log.decode('utf-8', 'replace')):
        position = np.asarray([float(x) for x in m[6].split(',')])
        quat = np.asarray([float(x) for x in m[7].split(',')])
        rows[int(m[2],16)] = {'actor': m[1], 'component': m[3], 'fname': m[4], 'class': m[5],
                              'position_cm': position.tolist(), 'quaternion_xyzw': quat.tolist()}
    bp = json.loads(raw_bp)
    definitions = {}
    for name in ('JianCeQiu', 'Outermost'):
        entry = next(e for e in bp['Exports'] if e.get('ObjectName') == name+'_GEN_VARIABLE')
        fields = {p['Name']:p.get('Value') for p in entry['Data']}
        definitions[name] = value(fields, 'RelativeLocation', 'XYZ', [0,0,0])
    root = np.asarray(rows[0x130]['position_cm'])
    q = np.asarray(rows[0x130]['quaternion_xyzw'])
    if not np.allclose(q[:3], 0, atol=1e-4) or not np.isclose(abs(q[3]), 1, atol=1e-4):
        raise ValueError('startup root orientation is not near identity')
    results = {}
    for name, off in (('JianCeQiu_candidate', 0x348), ('Outermost_candidate', 0x340)):
        position = np.asarray(rows[off]['position_cm'])
        expected = definitions[name.removesuffix('_candidate')]
        delta = position-root
        results[name] = {'actor_offset_hex':hex(off), **rows[off], 'relative_delta_cm': delta.tolist(),
                         'blueprint_default_offset_cm':expected.tolist(),
                         'difference_cm':float(np.linalg.norm(delta-expected))}
    output = {'status':'startup transform correspondence, runtime names and use not verified',
              'seed':run['seed_requested'], 'root':rows[0x130], 'candidates':results,
              'source_sha256':{'log':hashlib.sha256(log).hexdigest(), 'run':hashlib.sha256(raw_run).hexdigest(),
                               'blueprint':hashlib.sha256(raw_bp).hexdigest()},
              'limitations':['one startup pose with near-identity orientation', 'object fields and names not independently decoded',
                             'actor offsets and component identity may vary by build or instance',
                             'does not establish sphere overlap event participation']}
    a.output.write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print('seed',output['seed'], 'candidate errors cm', {k:v['difference_cm'] for k,v in results.items()})


if __name__=='__main__':
    main()
