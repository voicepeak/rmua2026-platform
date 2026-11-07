set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, json, struct, time
start_time = None
current_flag = 1
class ForcePredicate(gdb.Breakpoint):
 last = None
 def stop(self):
  global start_time, current_flag
  now = time.monotonic()
  if start_time is None:
   try:
    with open('/tmp/foundation_commands.jsonl') as f:
     first = json.loads(f.readline())
    if first.get('kind') == 'start': start_time = first['monotonic_ns']*1e-9
   except (OSError, ValueError, KeyError): pass
  elapsed = None if start_time is None else now-start_time
  desired = 0 if elapsed is not None and .5 <= elapsed < 1.15 else 1
  try:
   mem=gdb.selected_inferior()
   config=struct.unpack('<Q',bytes(mem.read_memory(0x719f540,8)))[0]
   if config: mem.write_memory(config+0x2d4,b'\x01')
   natural=int(gdb.parse_and_eval('$rax')) & 255
   gdb.execute('set $rax = ($rax & ~255) | %d' % desired)
   current_flag = desired
   if desired != self.last:
    print('TOGGLE mono=%.9f start=%s elapsed=%s natural=%d forced=%d config=%#x' %
          (now,str(start_time),str(elapsed),natural,desired,config),flush=True)
    self.last=desired
  except gdb.MemoryError: print('TOGGLE_ERROR',flush=True)
  return False
ForcePredicate('*0x2fb9184')
class TargetWatch(gdb.Breakpoint):
 count=0
 def __init__(self,ptr):
  super().__init__('*(float*)%#x' % (ptr+0xf0),type=gdb.BP_WATCHPOINT,wp_class=gdb.WP_WRITE,internal=True)
  self.silent=True
 def stop(self):
  self.count+=1
  print('TARGET_WRITE mono=%.9f flag=%d number=%d' % (time.monotonic(),current_flag,self.count),flush=True)
  gdb.execute('bt 10')
  if self.count>=20: self.enabled=False
  return False
class RotorSample(gdb.Breakpoint):
 last={}
 watched=False
 def stop(self):
  ptr=int(gdb.parse_and_eval('$rdi')); now=time.monotonic()
  if not self.watched:
   try:
    TargetWatch(ptr); self.watched=True
    print('TARGET_WATCH_INSTALLED rotor=%#x' % ptr,flush=True)
   except Exception as exc: print('TARGET_WATCH_ERROR %s' % exc,flush=True); self.watched=True
  if now-self.last.get(ptr,0) < .06: return False
  try:
   mem=gdb.selected_inferior()
   get=lambda off: struct.unpack('<f',bytes(mem.read_memory(ptr+off,4)))[0]
   print('TOGGLE_ROTOR mono=%.9f rotor=%#x flag=%d filtered=%.9g target=%.9g input=%.9g thrust=%.9g' %
         (now,ptr,current_flag,get(0xec),get(0xf0),get(0xf4),get(0x118)),flush=True)
   self.last[ptr]=now
  except gdb.MemoryError: print('TOGGLE_ROTOR_ERROR',flush=True)
  return False
RotorSample('*0x269ab90')
end
run
