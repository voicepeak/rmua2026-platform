"""Exploratory same-run replay using candidate native velocity and rotor snapshots."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.analyze_kinematic_probe import INPUT, ROTOR, vec
from foundation.compare import compare
from foundation.quad import Params, Quad
from foundation.replay_closed_loop import position_velocity


def run(directory, start_s=.30, end_s=.80, max_age_s=.05, clock='arrival'):
    if clock not in ('arrival','source_mapped'):raise ValueError('unknown clock')
    logs=list((directory/'pool').glob('*.log'))
    if len(logs)!=1:raise ValueError('expected one simulator log')
    paths=[logs[0],directory/'capture.jsonl',directory/'commands.jsonl',directory/'run.json']
    raw=[p.read_bytes() for p in paths]
    if json.loads(raw[3])['exit_status']!=0:raise ValueError('run failed')
    commands=[json.loads(x) for x in raw[2].splitlines() if x]
    t0=next(x['monotonic_ns'] for x in commands if x.get('kind')=='start')*1e-9
    sent=[x for x in commands if x.get('kind')=='command']
    ct=np.array([x['monotonic_ns']*1e-9-t0 for x in sent]);pwm=np.array([x['pwm'] for x in sent])
    capture=[json.loads(x) for x in raw[1].splitlines() if x]
    poses=[x for x in capture if x.get('topic')=='pose_gt'];imu=[x for x in capture if x.get('topic')=='imu']
    pt=np.array([x['arrival_monotonic_ns']*1e-9-t0 for x in poses]); xyz=np.array([x['position'] for x in poses])
    quat=np.array([x['quaternion_wxyz'] for x in poses]);it=np.array([x['arrival_monotonic_ns']*1e-9-t0 for x in imu]);omega=np.array([x['omega_body'] for x in imu])
    if any(len(t)<5 or np.any(np.diff(t)<=0) for t in (ct,pt,it)):raise ValueError('invalid timestamp stream')
    if pt[0]>start_s-.12 or pt[-1]<end_s+.12 or it[0]>start_s or it[-1]<end_s or ct[0]>start_s or ct[-1]<end_s:
        raise ValueError('insufficient temporal coverage')
    chosen=np.flatnonzero((pt>=start_s)&(pt<=end_s))
    if len(chosen)<5 or np.max(np.diff(pt[chosen]))>.05:raise ValueError('insufficient pose samples')
    start_host=float(pt[chosen[0]])
    log=raw[0].decode('utf-8','replace')
    native=[(float(m[1])-t0,vec(m[2],3)) for m in INPUT.finditer(log)]
    before=[r for r in native if r[0]<=start_host]
    if not before:raise ValueError('no prior physics snapshot')
    velocity_time,native_velocity=max(before,key=lambda x:x[0])
    if start_host-velocity_time>max_age_s:raise ValueError('physics snapshot too old')
    rotor=[]
    for m in ROTOR.finditer(log):rotor.append((float(m[1])-t0,int(m[2],16),float(m[3])))
    pointers=sorted({r[1] for r in rotor})
    if len(pointers)!=4:raise ValueError('expected four rotor objects')
    motor_state=[];ages=[]
    for pointer in pointers:
        prior=[x for x in rotor if x[1]==pointer and x[0]<=start_host]
        if not prior:raise ValueError('missing rotor snapshot')
        stamp,_,signal=max(prior,key=lambda x:x[0]);ages.append(start_host-stamp);motor_state.append(signal)
    if max(ages)>max_age_s or not np.isfinite(motor_state).all() or np.any((np.array(motor_state)<0)|(np.array(motor_state)>1)):
        raise ValueError('stale or invalid rotor state')
    if clock=='source_mapped':
        if not all(x.get('stamp_ns') for x in poses+imu):raise ValueError('missing source stamps')
        epoch=poses[0]['stamp_ns'];source=np.array([(x['stamp_ns']-epoch)*1e-9 for x in poses])
        imu_source=np.array([(x['stamp_ns']-epoch)*1e-9 for x in imu])
        if np.any(np.diff(source)<=0) or np.any(np.diff(imu_source)<=0):raise ValueError('nonmonotonic source stamps')
        covered=(ct>=pt[0])&(ct<=pt[-1]);ct=np.interp(ct[covered],pt,source);pwm=pwm[covered]
        pt,it=source,imu_source
        if np.any(np.diff(ct)<=0) or len(ct)<2:raise ValueError('mapped command timestamps invalid')
    start=float(pt[chosen[0]]); pos,derived_velocity=position_velocity(pt,xyz,start)
    w=np.array([np.interp(start,it,omega[:,j]) for j in range(3)]);q=quat[chosen[0]]
    ci=np.searchsorted(ct,start,side='right')-1
    if ci<0 or ct[-1]<pt[chosen[-1]] or it[0]>start or it[-1]<pt[chosen[-1]]:
        raise ValueError('mapped stream does not cover replay window')
    reference=[]
    for i in chosen:
        _,v=position_velocity(pt,xyz,pt[i]);reference.append({'t':float(pt[i]-start),'position':xyz[i].tolist(),
            'velocity':v.tolist(),'quaternion_wxyz':quat[i].tolist(),
            'omega':[float(np.interp(pt[i],it,omega[:,j])) for j in range(3)]})
    params=Params(.9,(.0046890742,.0069312,.010421166),.18,.000367717*(11079.03/60)**2,
                  4.888486266072161e-06*(11079.03/60)**2,.005)
    outputs={}
    for velocity_name,v0 in (('derived_velocity',derived_velocity),('native_velocity',native_velocity)):
        for motor_name,m0 in (('command_motor',pwm[ci]),('sampled_motor',motor_state)):
            quad=Quad(params);quad.reset(pos,v0,q,w,m0);predicted=[]
            for ref in reference:
                while quad.time<ref['t']-1e-10:
                    now=start+quad.time;idx=np.searchsorted(ct,now+1e-10,side='right')-1
                    next_command=ct[idx+1]-now if idx+1<len(ct) else np.inf
                    dt=min(.002,ref['t']-quad.time,next_command)
                    if dt<=1e-12:raise ValueError('invalid command interval')
                    quad.step(pwm[idx],dt)
                state=quad.state();predicted.append({k:float(v) if k=='t' else v.tolist() for k,v in state.items() if k!='rotor_signal'})
            outputs[velocity_name+'_'+motor_name]=compare(reference,predicted,max_gap=.05)
    return {'status':'same-run exploratory initial-state sensitivity; not acceptance','clock':clock,
            'window_host_s':[start_host,float(poses[chosen[-1]]['arrival_monotonic_ns']*1e-9-t0)],
            'window_s':[start,float(pt[chosen[-1]])],
            'native_velocity_snapshot_age_s':start_host-velocity_time,'rotor_snapshot_age_s':ages,
            'native_velocity_candidate':native_velocity.tolist(),'derived_velocity':derived_velocity.tolist(),
            'assumed_pointer_order':[hex(x) for x in pointers],'sampled_rotor_state':motor_state,
            'models':outputs,'source_sha256':{p.name:hashlib.sha256(b).hexdigest() for p,b in zip(paths,raw)},
            'limitations':['same run used for initialization and evaluation; not held out',
                           'source_mapped mode interpolates command host time onto pose source timestamps; true actuation time is unverified',
                           'sample age is host-monotonic difference, not calibrated physical timing',
                           'pointer-to-channel address order and physics-input frame remain candidate',
                           'ROS pose, native velocity and rotor snapshots are asynchronous',
                           'candidate excludes collisions and aerodynamic forces; free flight not established']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path);p.add_argument('--clock',choices=('arrival','source_mapped'),default='arrival');a=p.parse_args()
    result=run(a.run,clock=a.clock);result['analysis_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('window',result['window_s'],'native age',result['native_velocity_snapshot_age_s'],
          'max position error', {k:v['metrics']['position']['max'] for k,v in result['models'].items()})


if __name__=='__main__':main()
