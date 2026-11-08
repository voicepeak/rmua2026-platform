"""Exploratory paired same-recording replay with measured vs zero wind."""
import argparse
import base64
import hashlib
import json
import struct
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from foundation.aero import DragFaces
from foundation.compare import compare
from foundation.quad import Params, Quad
from foundation.replay_closed_loop import position_velocity
from foundation.effects import RecordedEffects
from foundation.sim import PhysicsSim


def run(directory, snapshot, start_s=.8, end_s=1.35, engine='direct'):
    if engine not in ('direct', 'integrated'):
        raise ValueError('unknown replay engine')
    cmd_path, cap_path = directory/'commands.jsonl', directory/'capture.jsonl'
    events = [json.loads(x) for x in cmd_path.read_text().splitlines() if x]
    t0 = next(x['monotonic_ns'] for x in events if x.get('kind') == 'start')
    commands = [x for x in events if x.get('kind') == 'command']
    ct = np.array([(x['monotonic_ns']-t0)*1e-9 for x in commands]); pwm = np.array([x['pwm'] for x in commands])
    records = [json.loads(x) for x in cap_path.read_text().splitlines() if x]
    poses = [x for x in records if x.get('topic') == 'pose_gt']; imu = [x for x in records if x.get('topic') == 'imu']
    winds = [x for x in records if x.get('topic') == 'wind']
    pt = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in poses]); it = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in imu])
    wt = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in winds])
    def decode(row):
        if row['type'] != 'geometry_msgs/Vector3Stamped' or row['md5sum'] != '7b324c7325e683bf02a9b14b01090ec7': raise ValueError('unexpected wind type')
        b=base64.b64decode(row['wire_base64'],validate=True); n=struct.unpack_from('<I',b,12)[0]
        if len(b)!=16+n+24: raise ValueError('invalid wind length')
        return struct.unpack_from('<3d',b,16+n)
    wv=np.array([decode(x) for x in winds]); xyz=np.array([x['position'] for x in poses]); q=np.array([x['quaternion_wxyz'] for x in poses]); omega=np.array([x['omega_body'] for x in imu])
    if any(len(t)<4 or np.any(np.diff(t)<=0) for t in (ct,pt,it,wt)): raise ValueError('invalid time series')
    if not all(np.isfinite(x).all() for x in (pwm,xyz,q,omega,wv)): raise ValueError('nonfinite observations')
    if not (pt[0] <= start_s-.12 and pt[-1] >= end_s+.12 and it[0] <= start_s and it[-1] >= end_s and ct[0] <= start_s and ct[-1] >= end_s and wt[0] <= start_s and wt[-1] >= end_s): raise ValueError('missing coverage')
    selected=np.where((pt>=start_s)&(pt<=end_s))[0]
    if len(selected)<5 or np.max(np.diff(pt[selected]))>.05: raise ValueError('pose gap')
    for times in (ct,it,wt):
        window=times[(times>=start_s-.06)&(times<=end_s+.06)]
        if len(window)<2 or np.max(np.diff(window))>.1: raise ValueError('command or sensor gap')
    start=float(pt[selected[0]]); pos,vel=position_velocity(pt,xyz,start)
    q0=Slerp(pt,Rotation.from_quat(q[:,[1,2,3,0]]))([start]).as_quat()[0]; w0=np.array([np.interp(start,it,omega[:,i]) for i in range(3)])
    params=Params(.9,(.0046890742,.0069312,.010421166),.18,.000367717*(11079.03/60)**2,4.888486266072161e-06*(11079.03/60)**2,.005)
    drag=DragFaces.from_observed_snapshot(snapshot,air_density=1.2107,min_velocity=.1)
    refs=[]
    for i in selected:
        _,v=position_velocity(pt,xyz,pt[i]); refs.append({'t':float(pt[i]-start),'position':xyz[i].tolist(),'velocity':v.tolist(),'quaternion_wxyz':q[i].tolist(),'omega':[float(np.interp(pt[i],it,omega[:,j])) for j in range(3)]})
    outcomes={}
    for label in ('recorded_wind','zero_wind'):
        signal=pwm[np.searchsorted(ct,start,side='right')-1]
        if engine == 'integrated':
            samples = wv if label == 'recorded_wind' else np.zeros_like(wv)
            effects = RecordedEffects(wt-start, samples, np.ones(len(wt), dtype=int))
            sim = PhysicsSim(params, aero_model=drag, recorded_effects=effects, max_substep_s=.002)
            sim.reset({'position':pos, 'velocity':vel, 'quaternion_wxyz':q0[[3,0,1,2]],
                       'omega':w0, 'rotor_signal':signal})
            plant = sim.quad
        else:
            plant=Quad(params); plant.reset(pos,vel,q0[[3,0,1,2]],w0,signal)
        predictions=[]
        for ref in refs:
            while plant.time < ref['t']-1e-10:
                now=start+plant.time; ix=np.searchsorted(ct,now+1e-10,side='right')-1; nxt=ct[ix+1]-now if ix+1<len(ct) else np.inf
                dt=min(.002,ref['t']-plant.time,nxt)
                if dt<=1e-12: raise ValueError('invalid command timing')
                if engine == 'integrated':
                    sim.step(pwm[ix], dt)
                else:
                    wind=np.array([np.interp(now,wt,wv[:,j]) for j in range(3)]) if label=='recorded_wind' else np.zeros(3)
                    f,t=drag.wrench(plant.rotation,plant.velocity,plant.omega,wind); plant.step(pwm[ix],dt,external_force_world=f,external_torque_body=t)
            s=plant.state();predictions.append({k:(float(v) if k=='t' else v.tolist()) for k,v in s.items() if k!='rotor_signal'})
        result=compare(refs,predictions,max_gap=.05)
        delta=np.array([p['position'] for p in predictions])-np.array([r['position'] for r in refs]); result['horizontal_position_rmse_m']=float(np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1))))
        result['horizontal_position_max_m']=float(np.linalg.norm(delta[:,:2],axis=1).max()); outcomes[label]=result
    return {'window_s':[start,float(pt[selected[-1]])], 'engine':engine, 'models':outcomes,'candidate_air_density':1.2107,'candidate_cutoff':.1,
            'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (cmd_path,cap_path,snapshot)},
            'limitations':['same-run exploratory replay with posthoc position-derived initial velocity and assumed initial rotor state',
                           'ROS wind and pose use subscriber arrival times, not calibrated physical synchronization',
                           'constant approximate density and rotor parameters; wind uses linear interpolation',
                           'no static collision, force field, wind-zone logic or actual actuator delay',
                           'forced stage/state, overtime suppression, all-stop GDB and possible contact',
                           'same seed may not ensure identical state and this is not a held-out acceptance']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('snapshot',type=Path);p.add_argument('output',type=Path);p.add_argument('--engine',choices=('direct','integrated'),default='direct');a=p.parse_args()
    result=run(a.run,a.snapshot,engine=a.engine);result['analysis_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('window',result['window_s'],{k:(v['horizontal_position_rmse_m'],v['horizontal_position_max_m'],v['metrics']['position']['max']) for k,v in result['models'].items()})


if __name__=='__main__': main()
