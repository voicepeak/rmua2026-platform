set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class DragForce(gdb.Breakpoint):
 last=0
 def stop(self):
  now=time.monotonic()
  if now-self.last < .04: return False
  try:
   mem=gdb.selected_inferior(); face=int(gdb.parse_and_eval('$rax')); rsp=int(gdb.parse_and_eval('$rsp'))
   get=lambda addr: struct.unpack('<f',bytes(mem.read_memory(addr,4)))[0]
   xyz=lambda addr: ','.join('%.9g' % x for x in struct.unpack('<3f',bytes(mem.read_memory(addr,12))))
   speed=float(gdb.parse_and_eval('$xmm5.v4_float[0]'))
   scalar=float(gdb.parse_and_eval('$xmm6.v4_float[0]'))
   print('AERO_FORCE mono=%.9f face=%#x position=%s normal=%s factor=%.9g density=%.9g component=%.9g scalar=%.9g' %
         (now,face,xyz(face+0x3c),xyz(face+0x54),get(face+0x78),get(rsp+0xc),speed,scalar),flush=True)
   self.last=now
  except gdb.MemoryError: print('AERO_FORCE_ERROR',flush=True)
  return False
DragForce('*0x2669755')
end
run
