set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class Contact(gdb.Breakpoint):
 def stop(self):
  drone = int(gdb.parse_and_eval('$rdi'))
  drone_component = int(gdb.parse_and_eval('$rsi'))
  source_actor = int(gdb.parse_and_eval('$rdx'))
  source_component = int(gdb.parse_and_eval('$rcx'))
  mem = gdb.selected_inferior()
  def vec(ptr, offset, count):
   if not ptr: return 'null'
   try: return ','.join('%.9g' % v for v in struct.unpack('<%df' % count, bytes(mem.read_memory(ptr+offset, count*4))))
   except gdb.MemoryError: return 'unreadable'
  print('MAP_CONTACT mono=%.9f drone=%#x drone_component=%#x drone_xyz=%s source_actor=%#x source_component=%#x source_xyz=%s source_quat=%s' %
        (time.monotonic(), drone, drone_component, vec(drone_component,0x1d0,3),
         source_actor, source_component, vec(source_component,0x1d0,3), vec(source_component,0x1c0,4)), flush=True)
  return False
Contact('*0x2630780')
end
run
