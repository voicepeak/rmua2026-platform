set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
wind_ready=False
class StageInit(gdb.Breakpoint):
 def stop(self):
  if int(gdb.parse_and_eval('$ecx')) != 1: return False
  gdb.execute('set $esi=8'); gdb.execute('set $edx=12'); gdb.execute('set $ecx=4')
  print('STAGE4_FORCED',flush=True); self.enabled=False; return False
class WindCreated(gdb.Breakpoint):
 def stop(self):
  global wind_ready
  wind_ready=True; print('STAGE4_WIND_READY',flush=True); self.enabled=False; return False
class ForceStage(gdb.Breakpoint):
 started=False
 def stop(self):
  if not wind_ready: return False
  try:
   mem=gdb.selected_inferior(); state=int(gdb.parse_and_eval('$rdi'))
   if not self.started:
    config=struct.unpack('<Q',bytes(mem.read_memory(0x719f540,8)))[0]
    now=time.monotonic_ns()
    mem.write_memory(state+0x158,struct.pack('<Q',now-5000000000))
    if config: mem.write_memory(config+0x2d4,b'\x01')
    self.started=True; print('STAGE4_CLOCK_SET mono_ns=%d offset_s=5' % now,flush=True)
   mem.write_memory(state+0xd0,struct.pack('<I',7))
  except gdb.MemoryError: print('STAGE4_INJECT_ERROR',flush=True)
  return False
class WindInput(gdb.Breakpoint):
 last=0
 def stop(self):
  now=time.monotonic()
  if now-self.last < .05: return False
  try:
   ptr=int(gdb.parse_and_eval('$r9'))
   values=struct.unpack('<3f',bytes(gdb.selected_inferior().read_memory(ptr,12)))
   print('PHYSICS_WIND mono=%.9f vector=%s' % (now,','.join('%.9g' % v for v in values)),flush=True)
   self.last=now
  except gdb.MemoryError: print('PHYSICS_WIND_ERROR',flush=True); self.last=now
  return False
StageInit('*0x2fbac30')
WindCreated('*0x2fbbf1f')
ForceStage('*0x2fb8f40')
WindInput('*0x2669530')
end
run
