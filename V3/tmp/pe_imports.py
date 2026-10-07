import struct
import sys
from pathlib import Path

for arg in sys.argv[1:]:
    b = Path(arg).read_bytes()
    pe = struct.unpack_from('<I', b, 60)[0]
    n, size = struct.unpack_from('<H', b, pe+6)[0], struct.unpack_from('<H', b, pe+20)[0]
    opt = pe+24
    sections = []
    for i in range(n):
        at = opt+size+40*i
        vs, va, rawsize, ptr = struct.unpack_from('<IIII', b, at+8)
        sections.append((va, max(vs, rawsize), ptr))
    def offset(rva):
        for va, length, ptr in sections:
            if va <= rva < va+length:
                return ptr+rva-va
        return rva
    directory = opt+(112 if struct.unpack_from('<H', b, opt)[0] == 0x20b else 96)
    at = offset(struct.unpack_from('<I', b, directory+8)[0])
    print(arg)
    while any(b[at:at+20]):
        p = offset(struct.unpack_from('<I', b, at+12)[0])
        print(b[p:b.index(b'\0', p)].decode())
        at += 20
