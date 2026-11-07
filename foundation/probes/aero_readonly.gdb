set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, math, struct, time
class DragFaces(gdb.Breakpoint):
 def stop(self):
  try:
   mem=gdb.selected_inferior(); body=int(gdb.parse_and_eval('$rdi'))
   start,end=struct.unpack('<QQ',bytes(mem.read_memory(body+0x170,16)))
   size=end-start
   if size <= 0 or size%128 or size//128 > 64: return False
   faces=[bytes(mem.read_memory(start+i*128,128)) for i in range(size//128)]
   for raw in faces:
    initial=struct.unpack_from('<3f',raw,0x48)
    current=struct.unpack_from('<3f',raw,0x54)
    if not all(math.isfinite(v) for v in current) or any(abs(a-b)>1e-5 for a,b in zip(initial,current)):
     return False
   print('DRAG_ARRAY mono=%.9f body=%#x begin=%#x count=%d stride=128' % (time.monotonic(),body,start,size//128),flush=True)
   for i,raw in enumerate(faces):
    print('DRAG_FACE index=%d hex=%s' % (i,raw.hex()),flush=True)
   self.enabled=False
  except gdb.MemoryError: print('DRAG_READ_ERROR',flush=True)
  return False
DragFaces('*0x26987f0')
end
run
