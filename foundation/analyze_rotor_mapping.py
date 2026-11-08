"""Compare a fixed address-order rotor hypothesis with PWM command samples."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from foundation.analyze_kinematic_probe import ROTOR


def compare(log, commands, *, start_s, end_s, delays_s):
    marker=next(x for x in commands if x.get('kind')=='start')
    t0=marker['monotonic_ns']*1e-9
    sent=[x for x in commands if x.get('kind')=='command']
    t=np.array([x['monotonic_ns']*1e-9-t0 for x in sent]);values=np.array([x['pwm'] for x in sent],float)
    if len(t)<2 or values.shape!=(len(t),4) or np.any(np.diff(t)<=0) or not np.isfinite(values).all():
        raise ValueError('invalid command stream')
    rows=[]
    for m in ROTOR.finditer(log):
        stamp=float(m[1])-t0; raw=float(m[5])
        if start_s<=stamp<=end_s:
            if not np.isfinite(raw):raise ValueError('invalid rotor input')
            rows.append((stamp,int(m[2],16),raw))
    pointers=sorted(set(row[1] for row in rows))
    if len(pointers)!=4:raise ValueError('expected four rotor pointers in window')
    series=[[(t,v) for t,p,v in rows if p==ptr] for ptr in pointers]
    if min(map(len,series))<3:raise ValueError('insufficient rotor observations')
    results=[]
    for delay in delays_s:
        cost=np.empty((4,4))
        for i,samples in enumerate(series):
            stamps=np.array([x[0] for x in samples]);observed=np.array([x[1] for x in samples])
            indices=np.searchsorted(t,stamps-delay,side='right')-1
            if np.any(indices<0):raise ValueError('command coverage insufficient')
            for channel in range(4):cost[i,channel]=np.mean((observed-values[indices,channel])**2)
        permutations=[]
        for order in itertools.permutations(range(4)):
            error=float(np.sqrt(sum(cost[i,c] for i,c in enumerate(order))/4))
            permutations.append((error,order))
        fixed=next(x for x in permutations if x[1]==(0,1,2,3))
        alternatives=sorted(x for x in permutations if x[1]!=(0,1,2,3))
        results.append({'assumed_delay_s':float(delay),'fixed_mapping_rmse':fixed[0],
                        'best_alternative_rmse':alternatives[0][0],
                        'best_alternative_mapping':list(alternatives[0][1])})
    return {'rotor_pointers_ascending':[hex(x) for x in pointers], 'counts':[len(s) for s in series],
            'window_s':[start_s,end_s], 'delay_results':results}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    logs=list((a.run/'pool').glob('*.log'))
    if len(logs)!=1:raise ValueError('expected one simulator log')
    paths=[logs[0],a.run/'commands.jsonl',a.run/'run.json'];raw=[p.read_bytes() for p in paths]
    if json.loads(raw[2])['exit_status']!=0:raise ValueError('run did not complete')
    result=compare(raw[0].decode('utf-8','replace'),[json.loads(s) for s in raw[1].splitlines() if s],
                   start_s=.30,end_s=.85,delays_s=[.002,.003,.004,.005])
    result.update(status='exploratory raw-input mapping, not actuator-latency acceptance',
                  source_sha256={p.name:hashlib.sha256(b).hexdigest() for p,b in zip(paths,raw)},
                  limitations=['address ordering is a build/run hypothesis', 'arrival times differ from physical command and sensor time',
                               'same waveform used to choose mapping and repeat', 'all-stop debugger affects execution'])
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('counts',result['counts'],'delay_results',result['delay_results'])


if __name__=='__main__':main()
