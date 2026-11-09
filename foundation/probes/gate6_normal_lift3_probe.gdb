set pagination off
set confirm off
set print thread-events off
set disable-randomization off
python
import gdb, struct, time

# Diagnostic intervention: move the drone to the first road3 approach point as
# soon as the first contact callback exposes the drone component.  This is not
# an official reset and is reported as such.
TARGET_CM = (69917.0, 14846.0, 12737.0)
teleported = False

class Inject(gdb.Breakpoint):
    def stop(self):
        if int(gdb.parse_and_eval('$ecx')) != 1:
            return False
        print('INJECT_STAGE2 old=%d,%d,%d new=3,5,2' % (
            int(gdb.parse_and_eval('$esi')), int(gdb.parse_and_eval('$edx')),
            int(gdb.parse_and_eval('$ecx'))), flush=True)
        gdb.execute('set $esi=3')
        gdb.execute('set $edx=5')
        gdb.execute('set $ecx=2')
        self.enabled = False
        return False
Inject('*0x2fbac30')

class Teleport(gdb.Breakpoint):
    def stop(self):
        global teleported
        if teleported:
            return False
        component = int(gdb.parse_and_eval('$rsi')) & ((1 << 64) - 1)
        if not component:
            return False
        mem = gdb.selected_inferior()
        try:
            old = struct.unpack('<3f', bytes(mem.read_memory(component + 0x1d0, 12)))
            mem.write_memory(component + 0x1d0, struct.pack('<fff', *TARGET_CM))
            mem.write_memory(component + 0x1c0, struct.pack('<ffff', 0.0, 0.0, 0.0, 1.0))
            teleported = True
            print('TELEPORT component=%#x old_cm=%.6f,%.6f,%.6f new_cm=%.6f,%.6f,%.6f' %
                  (component, old[0], old[1], old[2], TARGET_CM[0], TARGET_CM[1], TARGET_CM[2]), flush=True)
        except gdb.MemoryError:
            print('TELEPORT_UNREADABLE component=%#x' % component, flush=True)
        return False
Teleport('*0x2630780')

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
