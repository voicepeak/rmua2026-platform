set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class Contact(gdb.Breakpoint):
 inventoried = False
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
  def u64(ptr, offset):
   if not ptr: return 0
   try: return struct.unpack('<Q',bytes(mem.read_memory(ptr+offset,8)))[0]
   except gdb.MemoryError: return 0
  def fname(ptr):
   if not ptr: return 'null'
   try: return '%d,%d' % struct.unpack('<II',bytes(mem.read_memory(ptr+0x18,8)))
   except gdb.MemoryError: return 'unreadable'
  root = u64(source_actor,0x130)
  if not self.inventoried and drone:
   self.inventoried = True
   try:
    actor_data = bytes(mem.read_memory(drone, 0x1000))
    seen = set()
    for off in range(0, len(actor_data), 8):
     ptr = struct.unpack_from('<Q', actor_data, off)[0]
     if ptr < 0x10000 or ptr % 8 or ptr in seen: continue
     seen.add(ptr)
     try:
      if u64(ptr, 0x20) != drone: continue
      print('DRONE_OWNED_PTR actor=%#x actor_offset=%#x component=%#x fname=%s class=%#x world_xyz=%s world_quat=%s' %
            (drone, off, ptr, fname(ptr), u64(ptr,0x10), vec(ptr,0x1d0,3), vec(ptr,0x1c0,4)), flush=True)
     except (gdb.MemoryError, ValueError): pass
   except gdb.MemoryError: print('DRONE_INVENTORY_UNREADABLE', flush=True)
  hit_ptr = int(gdb.parse_and_eval('$r9'))
  try: hit_bytes = bytes(mem.read_memory(hit_ptr, 136)).hex() if hit_ptr else 'null'
  except gdb.MemoryError: hit_bytes = 'unreadable'
  print('MAP_CONTACT mono=%.9f drone=%#x drone_component=%#x drone_xyz=%s source_actor=%#x source_component=%#x source_xyz=%s source_quat=%s source_root=%#x root_xyz=%s actor_fname=%s component_fname=%s source_class=%#x hit_ptr=%#x hit_bytes=%s' %
        (time.monotonic(), drone, drone_component, vec(drone_component,0x1d0,3),
         source_actor, source_component, vec(source_component,0x1d0,3), vec(source_component,0x1c0,4),
         root, vec(root,0x1d0,3), fname(source_actor), fname(source_component), u64(source_actor,0x10), hit_ptr, hit_bytes), flush=True)
  return False
Contact('*0x2630780')
end
run
