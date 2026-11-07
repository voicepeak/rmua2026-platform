"""Read-only OL1 12.0.0.5 state probe; requires Linux ptrace permissions.

Offsets are specific to the exact binary hash below. This is a diagnostic
monitor, not an official scoring API. Run in a bounded, isolated simulator.
"""
import argparse
import hashlib
import json
import os
import struct
import time

EXPECTED_SHA256 = '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pid', required=True, type=int)
    p.add_argument('--state', required=True)
    p.add_argument('--log', required=True)
    p.add_argument('--duration', type=float, default=90.)
    args = p.parse_args()
    with open('/proc/%d/exe' % args.pid, 'rb') as binary:
        digest = hashlib.sha256()
        for chunk in iter(lambda: binary.read(1024*1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != EXPECTED_SHA256:
        raise SystemExit('unrecognized simulator binary: refusing offset-based probe')
    fd = os.open('/proc/%d/mem' % args.pid, os.O_RDONLY)
    def read(addr, count):
        data = os.pread(fd, count, addr)
        if len(data) != count:
            raise OSError('short read')
        return data
    start = time.monotonic()
    with open(args.log, 'w', buffering=1) as log:
        while time.monotonic() - start < args.duration:
            ptr = struct.unpack('<Q', read(0x719f540, 8))[0]
            if ptr:
                score = struct.unpack('<i', read(ptr+0x2fc, 4))[0]
                text, n = struct.unpack('<Qi', read(ptr+0x308, 12))
                state = read(text, n*2).decode('utf-16le', errors='replace').rstrip('\0') if text and 0 < n < 100 else ''
                item = dict(monotonic=time.monotonic(), score=score, state=state)
                temp = args.state + '.tmp'
                with open(temp, 'w') as out:
                    json.dump(item, out)
                os.replace(temp, args.state)
                log.write(json.dumps(item)+'\n')
                if state == 'Finished':
                    break
            time.sleep(.1)
    os.close(fd)


if __name__ == '__main__':
    main()
