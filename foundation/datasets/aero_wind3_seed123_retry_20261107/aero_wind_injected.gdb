set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
entry_inputs={}
class InjectWind(gdb.Breakpoint):
 count=0
 def stop(self):
  try:
   ptr=int(gdb.parse_and_eval('$r9'))
   if ptr:
    mem=gdb.selected_inferior()
    mem.write_memory(ptr,struct.pack('<3f',WIND_X,0,0))
    read=lambda reg,n: struct.unpack('<%df' % n,bytes(mem.read_memory(int(gdb.parse_and_eval('$'+reg)),n*4)))
    entry_inputs[gdb.selected_thread().global_num]=(read('rcx',3),read('r9',3),read('r8',3),read('rdx',4))
    self.count+=1
    if self.count<=3 or self.count%500==0:
     print('WIND_INJECT mono=%.9f ptr=%#x x=%g count=%d' % (time.monotonic(),ptr,WIND_X,self.count),flush=True)
  except gdb.MemoryError: print('WIND_INJECT_ERROR',flush=True)
  return False
InjectWind('*0x2669530')
class DragForce(gdb.Breakpoint):
 last=0
 def stop(self):
  now=time.monotonic()
  if now-self.last < .04: return False
  try:
   mem=gdb.selected_inferior(); face=int(gdb.parse_and_eval('$rax')); rsp=int(gdb.parse_and_eval('$rsp'))
   get=lambda addr: struct.unpack('<f',bytes(mem.read_memory(addr,4)))[0]
   vals=lambda addr,n: ','.join('%.9g' % x for x in struct.unpack('<%df' % n,bytes(mem.read_memory(addr,n*4))))
   inputs=entry_inputs.get(gdb.selected_thread().global_num)
   if inputs is None: return False
   render=lambda v: ','.join('%.9g' % x for x in v)
   speed=float(gdb.parse_and_eval('$xmm5.v4_float[0]')); scalar=float(gdb.parse_and_eval('$xmm6.v4_float[0]'))
   print('AERO_PAIRED mono=%.9f face=%#x position=%s normal=%s factor=%.9g density=%.9g component=%.9g scalar=%.9g velocity=%s wind=%s omega=%s quat=%s' %
         (now,face,vals(face+0x3c,3),vals(face+0x54,3),get(face+0x78),get(rsp+0xc),speed,scalar,
          *(render(v) for v in inputs)),flush=True)
   self.last=now
  except gdb.MemoryError: print('AERO_PAIRED_ERROR',flush=True); self.last=now
  return False
DragForce('*0x2669755')
end
run
