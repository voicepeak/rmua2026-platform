set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time
seen = set()
class Dump(gdb.Breakpoint):
    def stop(self):
        actor = int(gdb.parse_and_eval('$rdi')) & ((1 << 64) - 1)
        index = int(gdb.parse_and_eval('$esi'))
        enter = int(gdb.parse_and_eval('$edx'))
        event = int(gdb.parse_and_eval('$ecx'))
        cfg = 0
        try:
            cfg = struct.unpack('<Q', bytes(gdb.selected_inferior().read_memory(0x719f540, 8)))[0]
        except gdb.MemoryError:
            pass
        score = 0
        if cfg:
            try:
                score = struct.unpack('<i', bytes(gdb.selected_inferior().read_memory(cfg + 0x2fc, 4)))[0]
            except gdb.MemoryError:
                pass
        mem = gdb.selected_inferior()
        if actor and actor not in seen:
            seen.add(actor)
            try:
                count = struct.unpack('<i', bytes(mem.read_memory(actor + 0x248, 4)))[0]
                arr = struct.unpack('<Q', bytes(mem.read_memory(actor + 0x240, 8)))[0]
                print('DUMP_BEGIN actor=%#x count=%d arr=%#x' % (actor, count, arr), flush=True)
                for i in range(max(0, min(count, 256))):
                    comp = struct.unpack('<Q', bytes(mem.read_memory(arr + i * 8, 8)))[0]
                    if not comp:
                        print('DUMP_ITEM actor=%#x idx=%d component=0x0' % (actor, i), flush=True)
                        continue
                    pos = struct.unpack('<3f', bytes(mem.read_memory(comp + 0x1d0, 12)))
                    quat = struct.unpack('<4f', bytes(mem.read_memory(comp + 0x1c0, 16)))
                    scale = struct.unpack('<3f', bytes(mem.read_memory(comp + 0x1e0, 12)))
                    print('DUMP_ITEM actor=%#x idx=%d component=%#x xyz_cm=%.6f,%.6f,%.6f quat=%.8f,%.8f,%.8f,%.8f scale=%.6f,%.6f,%.6f'
                          % (actor, i, comp, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3],
                             scale[0], scale[1], scale[2]), flush=True)
                print('DUMP_END actor=%#x count=%d' % (actor, count), flush=True)
            except gdb.MemoryError:
                print('DUMP_UNREADABLE actor=%#x' % actor, flush=True)
        print('SEG actor=%#x idx=%d enter=%d event=%d score=%d' % (actor, index, enter, event, score), flush=True)
        return False
Dump('*0x2fc8d80')
class State(gdb.Breakpoint):
    def stop(self):
        try:
            cfg = struct.unpack('<Q', bytes(gdb.selected_inferior().read_memory(0x719f540, 8)))[0]
        except gdb.MemoryError:
            cfg = 0
        score = 0
        if cfg:
            try:
                score = struct.unpack('<i', bytes(gdb.selected_inferior().read_memory(cfg + 0x2fc, 4)))[0]
            except gdb.MemoryError:
                pass
        print('STATE old=%d new=%d flag=%d score=%d' % (
            int(gdb.parse_and_eval('*(int*)($rdi+0xd0)')), int(gdb.parse_and_eval('$esi')),
            int(gdb.parse_and_eval('$edx')), score), flush=True)
        return False
State('*0x2fb9960')
end
run
