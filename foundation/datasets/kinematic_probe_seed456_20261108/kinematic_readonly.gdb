set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb,struct,time
class PhysicsInput(gdb.Breakpoint):
 last=0.
 def stop(self):
  now=time.monotonic()
  if now-self.last<.02:return False
  try:
   mem=gdb.selected_inferior()
   def read(reg,n):
    ptr=int(gdb.parse_and_eval('$'+reg))
    return ','.join('%.9g'%x for x in struct.unpack('<%df'%n,bytes(mem.read_memory(ptr,n*4))))
   print('KIN_INPUT mono=%.9f velocity=%s wind=%s omega=%s quaternion=%s' %
         (now,read('rcx',3),read('r9',3),read('r8',3),read('rdx',4)),flush=True)
   self.last=now
  except (gdb.MemoryError,ValueError):print('KIN_INPUT_UNREADABLE',flush=True)
  return False
class Rotor(gdb.Breakpoint):
 last={}
 def stop(self):
  now=time.monotonic();ptr=int(gdb.parse_and_eval('$rdi'))
  if now-self.last.get(ptr,0)<.02:return False
  try:
   mem=gdb.selected_inferior();get=lambda off:struct.unpack('<f',bytes(mem.read_memory(ptr+off,4)))[0]
   print('KIN_ROTOR mono=%.9f rotor=%#x filtered=%.9g target=%.9g input=%.9g thrust=%.9g' %
         (now,ptr,get(0xec),get(0xf0),get(0xf4),get(0x118)),flush=True)
   self.last[ptr]=now
  except gdb.MemoryError:print('KIN_ROTOR_UNREADABLE',flush=True)
  return False
PhysicsInput('*0x2669530')
Rotor('*0x269ab90')
end
run
