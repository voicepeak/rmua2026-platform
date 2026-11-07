set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class ForcePredicate(gdb.Breakpoint):
 count = 0
 def stop(self):
  mem = gdb.selected_inferior()
  state = int(gdb.parse_and_eval('$r14'))
  old = int(gdb.parse_and_eval('$rax')) & 255
  desired = FORCED_FLAG
  try:
   config = struct.unpack('<Q',bytes(mem.read_memory(0x719f540,8)))[0]
   if config:
    mem.write_memory(config+0x2d4,b'\x01')
   gdb.execute('set $rax = ($rax & ~255) | %d' % desired)
   self.count += 1
   if self.count <= 5 or self.count % 100 == 0:
    print('FIELD_INJECT mono=%.9f state=%#x natural=%d forced=%d config=%#x count=%d' %
          (time.monotonic(),state,old,desired,config,self.count),flush=True)
  except gdb.MemoryError:
   print('FIELD_INJECT_ERROR unreadable config',flush=True)
  return False
ForcePredicate('*0x2fb9184')
end
run
