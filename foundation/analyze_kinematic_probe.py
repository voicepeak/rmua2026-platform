"""Compare read-only physics-loop velocity with pose-derived velocity estimates."""
import argparse
from collections import Counter
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from foundation.replay_closed_loop import position_velocity


INPUT=re.compile(r'KIN_INPUT mono=([\d.]+) velocity=([^ ]+) wind=([^ ]+) omega=([^ ]+) quaternion=([^\s]+)')
ROTOR=re.compile(r'KIN_ROTOR mono=([\d.]+) rotor=(0x[0-9a-f]+) filtered=([^ ]+) target=([^ ]+) input=([^ ]+) thrust=([^\s]+)')


def vec(raw, n):
    x=np.asarray([float(v) for v in raw.split(',')],float)
    if x.shape!=(n,) or not np.isfinite(x).all():raise ValueError('invalid native vector')
    return x


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--start',type=float,default=.3);p.add_argument('--end',type=float,default=.8);a=p.parse_args()
    logs=list((a.run/'pool').glob('*.log'))
    if len(logs)!=1:raise ValueError('expected one log')
    paths=[logs[0],a.run/'capture.jsonl',a.run/'commands.jsonl',a.run/'run.json'];raw=[p.read_bytes() for p in paths]
    meta=json.loads(raw[3]);
    if meta['exit_status']!=0:raise ValueError('run failed')
    commands=[json.loads(x) for x in raw[2].splitlines() if x];marker=next(x for x in commands if x.get('kind')=='start')
    t0=marker['monotonic_ns']*1e-9
    capture=[json.loads(x) for x in raw[1].splitlines() if x];poses=[x for x in capture if x.get('topic')=='pose_gt']
    times=np.array([x['arrival_monotonic_ns']*1e-9-t0 for x in poses]);positions=np.array([x['position'] for x in poses])
    if len(times)<5 or np.any(np.diff(times)<=0):raise ValueError('invalid pose stream')
    log=raw[0].decode('utf-8','replace'); rotor=[]
    for m in ROTOR.finditer(log):
        row={'t':float(m[1])-t0,'rotor':m[2],'filtered':float(m[3]),'target':float(m[4]),'input':float(m[5]),'thrust':float(m[6])}
        if not np.isfinite([row[k] for k in ('filtered','target','input','thrust')]).all():raise ValueError('invalid rotor')
        rotor.append(row)
    events=[]
    for m in INPUT.finditer(log):
        t=float(m[1])-t0
        if not a.start<=t<=a.end:continue
        native=vec(m[2],3);wind=vec(m[3],3);omega=vec(m[4],3);quat=vec(m[5],4)
        try:_,velocity=position_velocity(times,positions,t)
        except ValueError:continue
        nearest=[]
        for ptr in sorted(set(r['rotor'] for r in rotor)):
            candidates=[r for r in rotor if r['rotor']==ptr]
            if candidates:
                sample=min(candidates,key=lambda r:abs(r['t']-t));nearest.append({**sample,'arrival_delta_s':sample['t']-t})
        events.append({'t':t,'native_velocity_candidate':native.tolist(),'pose_derived_velocity':velocity.tolist(),
                       'velocity_difference':float(np.linalg.norm(native-velocity)), 'wind':wind.tolist(),
                       'omega':omega.tolist(),'quaternion':quat.tolist(),'nearest_rotors':nearest})
    if not events:raise ValueError('no paired samples in requested window')
    diffs=np.array([e['velocity_difference'] for e in events]);counts=Counter(r['rotor'] for r in rotor)
    result={'status':'exploratory cross-clock comparison; not calibrated velocity','window_s':[a.start,a.end],
            'paired_samples':len(events),'velocity_difference_m_s':{'median':float(np.median(diffs)),
            'p95':float(np.percentile(diffs,95)),'max':float(diffs.max())},'rotor_sample_counts':dict(counts),
            'events':events,'source_sha256':{p.name:hashlib.sha256(b).hexdigest() for p,b in zip(paths,raw)},
            'limitations':['physics-loop input velocity interpretation and frame are candidate',
                           'host monotonic breakpoint time and ROS arrival are not synchronized physical time',
                           'pose-derived velocity is a two-sided posthoc polynomial fit',
                           'rotor snapshots are asynchronous; pointer-to-motor index not established',
                           'all-stop GDB perturbs time and collision conditions are not independently ruled out']}
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('pairs',len(events),'velocity difference',result['velocity_difference_m_s'],'rotors',len(counts))


if __name__=='__main__':main()
