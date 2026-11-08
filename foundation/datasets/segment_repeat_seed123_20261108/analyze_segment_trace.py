#!/usr/bin/env python3
"""Summarize logged route segment events and adjacent raw sparse-set counters."""
import argparse,bisect,collections,json,re
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
pattern=re.compile(r'RMUA_SEGMENT_EVENT spline=(\S+) fname_index=(\d+) fname_number=(\d+) segment_index=(-?\d+) enter_flag=(-?\d+) event_flag=(-?\d+) sparse_num=(-?\d+) free_num=(-?\d+)')
gps=[json.loads(l) for l in (a.run/'pose_samples.jsonl').open()];times=[x['arrival_monotonic_s'] for x in gps];events=[];clock=None
for line in (a.run/'gdb.log').open(errors='replace'):
 if 'RMUA_SEGMENT_CLOCK ' in line:
  m=re.search(r'wall_unix=([\d.]+) monotonic=([\d.]+)',line)
  clock={'wall_unix_s':float(m.group(1)),'monotonic_s':float(m.group(2))} if m else None
 m=pattern.search(line)
 if m:
  groups=m.groups();e={'spline':groups[0],'fname_index':int(groups[1]),'fname_number':int(groups[2]),'segment_index':int(groups[3]),'enter_flag':int(groups[4]),'event_flag':int(groups[5]),'sparse_num_before':int(groups[6]),'free_num_before':int(groups[7]),'clock':clock}
  e['active_before']=e['sparse_num_before']-e['free_num_before']
  if clock and times:
   idx=bisect.bisect_left(times,clock['monotonic_s']);valid=[j for j in (idx-1,idx) if 0<=j<len(times)];j=min(valid,key=lambda k:abs(times[k]-clock['monotonic_s']));e['nearest_gps']={'position':gps[j].get('position'),'arrival_delta_s':times[j]-clock['monotonic_s'],'accepted':gps[j]['accepted']}
  events.append(e);clock=None
counts=collections.Counter((x['enter_flag'],x['event_flag']) for x in events);same=[]
for x,y in zip(events,events[1:]):
 if x['spline']==y['spline']:
  same.append({'enter_flag':x['enter_flag'],'event_flag':x['event_flag'],'delta_to_next_before':y['active_before']-x['active_before']})
summary={'events':events,'counts':{f'enter={k[0]},event={k[1]}':v for k,v in counts.items()},'adjacent_same_spline_counter_deltas':dict(collections.Counter(f"enter={r['enter_flag']},event={r['event_flag']},delta={r['delta_to_next_before']}" for r in same)),'limits':['all-stop GDB can change scheduling','adjacent log entries need not prove no intervening mutation','nearest GPS callback arrival is not a synchronized source pose','same FName/position associations are not independently decoded']}
a.output.write_text(json.dumps(summary,indent=2)+'\n');print('events',len(events),'counts',summary['counts']);print('deltas',summary['adjacent_same_spline_counter_deltas'])
