"""Compare ROS wind messages with native physics-entry wind in a forced stage."""
import argparse
import base64
import hashlib
import json
import re
import struct
from pathlib import Path
import numpy as np

PATTERN = re.compile(r'PHYSICS_WIND mono=([\d.]+) vector=([^\s]+)')


def analyze(directory, max_gap_s=.06):
    log_path, = (directory/'pool').glob('*.log')
    capture_path = directory/'capture.jsonl'
    raw = log_path.read_bytes()
    text = raw.decode('utf-8', errors='replace')
    if not all(x in text for x in ('STAGE4_FORCED', 'STAGE4_WIND_READY', 'STAGE4_CLOCK_SET')):
        raise ValueError('stage injection markers missing')
    if 'PHYSICS_WIND_ERROR' in text or 'STAGE4_INJECT_ERROR' in text:
        raise ValueError('probe error')
    native = [(float(m[1]), np.array([float(x) for x in m[2].split(',')])) for m in PATTERN.finditer(text)]
    ros = []
    for line in capture_path.read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        if row.get('topic') != 'wind': continue
        if row.get('type') != 'geometry_msgs/Vector3Stamped' or row.get('md5sum') != '7b324c7325e683bf02a9b14b01090ec7':
            raise ValueError('unexpected wind message type')
        b = base64.b64decode(row['wire_base64'], validate=True)
        if len(b) < 16: raise ValueError('short header')
        length = struct.unpack_from('<I',b,12)[0]
        if len(b) != 16+length+24: raise ValueError('unexpected wire length')
        ros.append((row['arrival_monotonic_ns']*1e-9, np.array(struct.unpack_from('<3d',b,16+length))))
    if not native or not ros: raise ValueError('missing input samples')
    rt = np.array([x[0] for x in ros]); pairs=[]
    if np.any(np.diff(rt) <= 0): raise ValueError('nonmonotonic ROS arrivals')
    for nt, nv in native:
        if nv.shape != (3,) or not np.isfinite(nv).all(): raise ValueError('invalid native vector')
        i = int(np.argmin(abs(rt-nt))); gap=rt[i]-nt
        if abs(gap) <= max_gap_s:
            rv=ros[i][1]
            if not np.isfinite(rv).all(): raise ValueError('invalid ROS vector')
            pairs.append({'native_time_s':nt,'arrival_gap_s':gap,'native':nv.tolist(),'ros':rv.tolist(),
                          'difference_norm':float(np.linalg.norm(nv-rv))})
    if not pairs: raise ValueError('no matching arrivals')
    errors=np.array([x['difference_norm'] for x in pairs])
    return {'native_samples':len(native),'ros_samples':len(ros),'paired':len(pairs),
            'max_gap_s':max_gap_s,'nonzero_native_pairs':int(sum(np.linalg.norm(x['native']) > .01 for x in pairs)),
            'difference_p50_p95_max':np.percentile(errors,[50,95,100]).tolist(), 'pairs':pairs,
            'source_sha256': {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (log_path,capture_path)},
            'limitations':['stage initialization and state forced; overtime suppression enabled',
                           'GDB all-stop changes timing; nearest subscriber arrival is not synchronized source time',
                           'initial-area coverage and fixed waveform only; not natural progression or boundary',
                           'equal observed vectors do not identify every intermediary or guarantee other stages']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=analyze(a.run)
    result['analysis_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('pairs',result['paired'],'nonzero',result['nonzero_native_pairs'],'errors',result['difference_p50_p95_max'])


if __name__ == '__main__': main()
