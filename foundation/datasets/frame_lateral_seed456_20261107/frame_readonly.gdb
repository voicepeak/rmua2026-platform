set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class RootSample(gdb.Breakpoint):
 last = 0.0
 def stop(self):
  now = time.monotonic()
  if now - self.last < 0.05:
   return False
  actor = int(gdb.parse_and_eval('$r14'))
  component = int(gdb.parse_and_eval('$rbx'))
  if not actor or not component:
   return False
  try:
   mem = gdb.selected_inferior()
   xyz = struct.unpack('<fff', bytes(mem.read_memory(component+0x1d0, 12)))
   quat = struct.unpack('<ffff', bytes(mem.read_memory(component+0x1c0, 16)))
   print('DRONE_WORLD mono=%.9f actor=%#x component=%#x xyz_cm=%s quat=%s' %
         (now, actor, component, ','.join('%.9g' % x for x in xyz), ','.join('%.9g' % x for x in quat)), flush=True)
   self.last = now
  except gdb.MemoryError:
   print('ROOT_SAMPLE_UNREADABLE', flush=True)
  return False
RootSample('*0x26302df')
end
run
