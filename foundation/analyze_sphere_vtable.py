"""Compare observed component vptr to exported USphereComponent vtable."""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('symbol',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    logs=list((a.run/'pool').glob('*.log'))
    if len(logs)!=1:raise ValueError('expected one simulator log')
    log,symbol,manifest=logs[0].read_bytes(),a.symbol.read_bytes(),(a.run/'run.json').read_bytes()
    if json.loads(manifest)['exit_status']!=0:raise ValueError('run failed')
    sym=re.search(r'^([0-9a-f]+)\s+\w\s+vtable for USphereComponent$',symbol.decode(),re.MULTILINE)
    entries=re.findall(r'DRONE_OWNED_PTR .*actor_offset=0x348 .* vtable=(0x[0-9a-f]+)',log.decode('utf-8','replace'))
    if not sym or len(entries)!=1:raise ValueError('missing vtable symbol or component observation')
    base=int(sym[1],16);vptr=int(entries[0],16)
    raw_match=re.search(r'DRONE_CANDIDATE_BYTES component=0x[0-9a-f]+ bytes=([0-9a-f]+)',log.decode('utf-8','replace'))
    candidate_offsets=[]
    if raw_match:
        data=bytes.fromhex(raw_match[1]);candidate_offsets=[hex(i) for i in range(0,len(data)-3,4)
            if struct.unpack_from('<f',data,i)[0]==2.0]
    result={'status':'runtime vtable correspondence, not event attribution', 'symbol_vtable':hex(base),
            'object_vptr':hex(vptr),'vptr_offset_bytes':vptr-base,'matches_itanium_address_point':vptr==base+16,
            'float32_equal_2_offsets':candidate_offsets,
            'source_sha256':{'log':hashlib.sha256(log).hexdigest(),'symbol':hashlib.sha256(symbol).hexdigest(),
                             'run':hashlib.sha256(manifest).hexdigest()},
            'limitations':['2.0 float offsets are untyped and do not independently establish sphere radius',
                           'one instrumented startup process; UE reflection name not decoded',
                           'type correspondence does not prove field-event binding']}
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('vptr delta',vptr-base,'matches',result['matches_itanium_address_point'],'2.0 candidates',candidate_offsets)


if __name__=='__main__':main()
