"""GDB read-only transition trace for GameState predicate 0x2fb9830.

The observed raw fields are intentionally not assigned product semantics.
One all-stop breakpoint per invocation can change simulation scheduling.
"""
import gdb,json,struct,time
last=None

def read(addr,size):return gdb.selected_inferior().read_memory(addr,size).tobytes()
def u64(addr):return struct.unpack('<Q',read(addr,8))[0]
def u32(addr):return struct.unpack('<I',read(addr,4))[0]
def f32(addr):return struct.unpack('<f',read(addr,4))[0]
def lists(ptr,offset,count_offset,kind):
 count=u32(ptr+count_offset)
 if count>4096:return {'error':'invalid_count','count':count}
 data=u64(ptr+offset);out=[]
 for i in range(count):
  obj=u64(data+8*i)
  if not obj:continue
  if kind=='compare':v={'value_288':u32(obj+0x288),'value_2b4':u32(obj+0x2b4)};v['passed']=v['value_288']>v['value_2b4']
  else:v={'flag':read(obj+(0x220 if kind=='220' else 0x238),1)[0]};v['passed']=bool(v['flag'])
  if v['passed']:out.append(dict(index=i,object=hex(obj),**v))
 return {'count':count,'passing':out[:32]}
class Ret(gdb.FinishBreakpoint):
 def __init__(self,ptr):super().__init__(gdb.newest_frame(),internal=True);self.silent=True;self.ptr=ptr
 def stop(self):
  global last
  result=int(gdb.parse_and_eval('$rax'))&255
  if result!=last:
   try:
    p=self.ptr;row={'wall_unix_s':time.time(),'monotonic_s':time.monotonic(),'state':u32(p+0xd0),'predicate':result,'stored_flag_before_update':read(p+0x180,1)[0],'out_timer':f32(p+0x1ac),'field_14':f32(p+0x14),'field_f4':f32(p+0xf4),'lists':{}}
    try:
     drone=int(gdb.parse_and_eval('$foundation_drone_component'))
     actor=int(gdb.parse_and_eval('$foundation_drone_actor'))
     row['drone_component']=hex(drone);row['drone_actor']=hex(actor)
     if drone: row['drone_world_cm']=[f32(drone+0x1d0+i*4) for i in range(3)]
    except Exception as exc: row['drone_read_error']=repr(exc)
    for name,off,cnt,kind in [('array30',0x30,0x38,'compare'),('array70',0x70,0x78,'compare'),('array60',0x60,0x68,'220'),('array40',0x40,0x48,'238')]:row['lists'][name]=lists(p,off,cnt,kind)
    print('RMUA_PREDICATE_CHANGE '+json.dumps(row,sort_keys=True),flush=True)
   except Exception as exc:print('RMUA_PREDICATE_ERROR '+repr(exc),flush=True)
   last=result
  return False
class Entry(gdb.Breakpoint):
 def stop(self):
  try:Ret(int(gdb.parse_and_eval('$rdi')))
  except Exception as exc:print('RMUA_PREDICATE_ERROR '+repr(exc),flush=True)
  return False
Entry('*0x2fb9830')
