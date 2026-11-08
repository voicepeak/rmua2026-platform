set pagination off
set print thread-events off
set $foundation_drone_component = 0
set $foundation_drone_actor = 0
python
import gdb
class DronePointer(gdb.Breakpoint):
 def stop(self):
  actor = int(gdb.parse_and_eval('$r14'))
  component = int(gdb.parse_and_eval('$rbx'))
  if actor and component:
   gdb.set_convenience_variable('foundation_drone_actor', gdb.Value(actor))
   gdb.set_convenience_variable('foundation_drone_component', gdb.Value(component))
  return False
DronePointer('*0x26302df')
end
