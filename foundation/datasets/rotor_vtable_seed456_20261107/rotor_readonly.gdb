set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
class RotorSample(gdb.Breakpoint):
 last = {}
 def stop(self):
  now = time.monotonic()
  rotor = int(gdb.parse_and_eval('$rdi'))
  if not rotor or now - self.last.get(rotor, 0) < 0.03:
   return False
  try:
   mem = gdb.selected_inferior()
   rotor_vt = struct.unpack('<Q', bytes(mem.read_memory(rotor, 8)))[0]
   rotor_slots = struct.unpack('<12Q', bytes(mem.read_memory(rotor_vt, 96)))
   raw = bytes(mem.read_memory(rotor + 0xb8, 0x78))
   vt = struct.unpack_from('<Q', raw, 0)[0]
   slots = struct.unpack('<8Q', bytes(mem.read_memory(vt, 64)))
   fields = struct.unpack('<30f', raw)
   print('ROTOR mono=%.9f ptr=%#x rotor_vtable=%#x rotor_slots=%s vtable=%#x slots=%s fields_b8=%s' %
         (now, rotor, rotor_vt, ','.join(hex(v) for v in rotor_slots), vt, ','.join(hex(v) for v in slots), ','.join('%.9g' % f for f in fields)), flush=True)
   self.last[rotor] = now
  except gdb.MemoryError:
   print('ROTOR_UNREADABLE', flush=True)
  return False
RotorSample('*0x269af00')
end
run
