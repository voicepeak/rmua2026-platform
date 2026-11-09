set pagination off
set confirm off
set breakpoint pending on
set print thread-events off
set disable-randomization off
python
import gdb, os, struct, time

target = '/Game/Maps/RMUA2024OL/RMUA2024OL10'
old = '/Game/Maps/RMUA2024OL/RMUA2024OL1'.encode('utf-16-le') + b'\0\0'

class RedirectMap(gdb.Breakpoint):
    def stop(self):
        if not target:
            return False
        mem = gdb.selected_inferior()
        base = int(gdb.parse_and_eval('$r14'))
        try:
            ptr, n, cap = struct.unpack('<Qii', bytes(mem.read_memory(base + 0x28, 16)))
            if not ptr or n <= 0 or n > 256 or cap < n:
                return False
            raw = bytes(mem.read_memory(ptr, 2 * n))
            if raw == old:
                new = target.encode('utf-16-le') + b'\0\0'
                if len(new) != len(old):
                    print('RMUA_MAP_REDIRECT_LENGTH_MISMATCH old=%d new=%d' % (len(old), len(new)), flush=True)
                    return False
                mem.write_memory(ptr, new)
                print('RMUA_MAP_REDIRECT old=OL1 new=%s' % target, flush=True)
        except (gdb.MemoryError, ValueError):
            pass
        return False
RedirectMap('*0x593dd30')

seen = set()
class Dump(gdb.Breakpoint):
    def stop(self):
        actor = int(gdb.parse_and_eval('$rdi')) & ((1 << 64) - 1)
        index = int(gdb.parse_and_eval('$esi'))
        enter = int(gdb.parse_and_eval('$edx'))
        event = int(gdb.parse_and_eval('$ecx'))
        mem = gdb.selected_inferior()
        score = 0
        try:
            cfg = struct.unpack('<Q', bytes(mem.read_memory(0x719f540, 8)))[0]
            if cfg:
                score = struct.unpack('<i', bytes(mem.read_memory(cfg + 0x2fc, 4)))[0]
        except gdb.MemoryError:
            pass
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
        mem = gdb.selected_inferior()
        score = 0
        try:
            cfg = struct.unpack('<Q', bytes(mem.read_memory(0x719f540, 8)))[0]
            if cfg:
                score = struct.unpack('<i', bytes(mem.read_memory(cfg + 0x2fc, 4)))[0]
        except gdb.MemoryError:
            pass
        print('STATE old=%d new=%d flag=%d score=%d' % (
            int(gdb.parse_and_eval('*(int*)($rdi+0xd0)')), int(gdb.parse_and_eval('$esi')),
            int(gdb.parse_and_eval('$edx')), score), flush=True)
        return False
State('*0x2fb9960')
end
run
