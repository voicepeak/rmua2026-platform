"""Compare velocity differentiation in ROS source time versus host arrival time."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from foundation.analyze_kinematic_probe import INPUT, vec
from foundation.replay_closed_loop import position_velocity


def run(directory, start_s=.30, end_s=.80):
    logs=list((directory/'pool').glob('*.log'))
    if len(logs)!=1:raise ValueError('expected one log')
    paths=[logs[0],directory/'capture.jsonl',directory/'commands.jsonl',directory/'run.json']
    raw=[p.read_bytes() for p in paths]
    if json.loads(raw[3])['exit_status']!=0:raise ValueError('run failed')
    commands=[json.loads(x) for x in raw[2].splitlines() if x]
    marker=next(x for x in commands if x.get('kind')=='start');t0=marker['monotonic_ns']
    poses=[x for x in (json.loads(s) for s in raw[1].splitlines() if s) if x.get('topic')=='pose_gt' and x.get('stamp_ns')]
    if len(poses)<5:raise ValueError('insufficient poses')
    host=np.asarray([(x['arrival_monotonic_ns']-t0)*1e-9 for x in poses]);base=poses[0]['stamp_ns']
    source=np.asarray([(x['stamp_ns']-base)*1e-9 for x in poses]);pos=np.asarray([x['position'] for x in poses])
    if np.any(np.diff(host)<=0) or np.any(np.diff(source)<=0):raise ValueError('nonmonotonic timestamps')
    rows=[]
    for m in INPUT.finditer(raw[0].decode('utf-8','replace')):
        t=float(m[1])-t0*1e-9
        if not start_s<=t<=end_s:continue
        try:
            _,vhost=position_velocity(host,pos,t)
            mapped=float(np.interp(t,host,source));_,vsource=position_velocity(source,pos,mapped)
        except ValueError:continue
        native=vec(m[2],3)
        rows.append({'host_t_s':t,'mapped_source_t_s':mapped,'native':native.tolist(),
                     'arrival_derivative':vhost.tolist(),'source_derivative':vsource.tolist(),
                     'arrival_error_m_s':float(np.linalg.norm(native-vhost)),
                     'source_error_m_s':float(np.linalg.norm(native-vsource))})
    if not rows:raise ValueError('no usable comparisons')
    def stats(field):
        x=np.asarray([r[field] for r in rows]);return {'median':float(np.median(x)),
                                                        'p95':float(np.percentile(x,95)),'max':float(x.max())}
    selected=(host>=start_s)&(host<=end_s)
    if selected.sum()<2:raise ValueError('insufficient clock coverage')
    speed=(source[selected][-1]-source[selected][0])/(host[selected][-1]-host[selected][0])
    return {'status':'exploratory source-vs-arrival clock comparison','window_s':[start_s,end_s],
            'source_to_host_elapsed_ratio':float(speed),'sample_count':len(rows),
            'arrival_error_m_s':stats('arrival_error_m_s'),'source_error_m_s':stats('source_error_m_s'),
            'samples':rows,'source_sha256':{p.name:hashlib.sha256(b).hexdigest() for p,b in zip(paths,raw)},
            'limitations':['source stamp interpreted as simulation-related time; clock mapping not independently certified',
                           'native probe time paired with ROS arrival, not physical timestamp',
                           'two-sided polynomial velocity fit uses future pose samples',
                           'same recordings and interval used in prior investigation',
                           'GDB all-stop alters runtime cadence; no flight/contact guarantee']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    result=run(a.run);result['analysis_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('count',result['sample_count'],'source/host ratio',result['source_to_host_elapsed_ratio'],
          'arrival/source velocity errors',result['arrival_error_m_s']['median'],result['source_error_m_s']['median'])


if __name__=='__main__':main()
