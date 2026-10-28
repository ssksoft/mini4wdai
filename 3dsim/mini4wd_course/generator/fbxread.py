"""Minimal binary FBX reader (geometry + model transforms)."""
import struct, zlib, sys
import numpy as np

def _read_node(f, ver):
    if ver >= 7500:
        end, nprops, plen = struct.unpack('<QQQ', f.read(24))
    else:
        end, nprops, plen = struct.unpack('<III', f.read(12))
    nlen = f.read(1)[0]
    name = f.read(nlen).decode('latin1')
    if end == 0:
        return None
    props = []
    for _ in range(nprops):
        t = f.read(1).decode()
        if t == 'Y': props.append(struct.unpack('<h', f.read(2))[0])
        elif t == 'C': props.append(bool(f.read(1)[0]))
        elif t == 'I': props.append(struct.unpack('<i', f.read(4))[0])
        elif t == 'F': props.append(struct.unpack('<f', f.read(4))[0])
        elif t == 'D': props.append(struct.unpack('<d', f.read(8))[0])
        elif t == 'L': props.append(struct.unpack('<q', f.read(8))[0])
        elif t in 'fdlib':
            n, enc, clen = struct.unpack('<III', f.read(12))
            data = f.read(clen)
            if enc == 1: data = zlib.decompress(data)
            dt = {'f': '<f4', 'd': '<f8', 'l': '<i8', 'i': '<i4', 'b': 'u1'}[t]
            props.append(np.frombuffer(data, dtype=dt, count=n))
        elif t in 'SR':
            n = struct.unpack('<I', f.read(4))[0]
            d = f.read(n)
            props.append(d.decode('utf8', 'replace') if t == 'S' else d)
        else:
            raise ValueError('prop type ' + t)
    children = []
    sentinel = 25 if ver >= 7500 else 13
    while f.tell() < end:
        if end - f.tell() == sentinel:
            f.read(sentinel); break
        c = _read_node(f, ver)
        if c is None: break
        children.append(c)
    return {'name': name, 'props': props, 'children': children}

def load(path):
    with open(path, 'rb') as f:
        assert f.read(21) == b'Kaydara FBX Binary  \x00'
        f.read(2)
        ver = struct.unpack('<I', f.read(4))[0]
        nodes = []
        while True:
            n = _read_node(f, ver)
            if n is None: break
            nodes.append(n)
    return ver, nodes

def find(node_list, name):
    return [n for n in node_list if n['name'] == name]

def child(n, name):
    for c in n['children']:
        if c['name'] == name: return c
    return None

def dump(nodes, depth=0, maxd=3):
    for n in nodes:
        ps = []
        for p in n['props']:
            if isinstance(p, np.ndarray): ps.append(f'arr[{p.dtype},{len(p)}]')
            elif isinstance(p, bytes): ps.append(f'raw[{len(p)}]')
            else: ps.append(repr(p)[:60])
        print('  ' * depth + n['name'], ps)
        if depth < maxd: dump(n['children'], depth + 1, maxd)

if __name__ == '__main__':
    ver, nodes = load(sys.argv[1])
    print('version', ver)
    dump(nodes, maxd=int(sys.argv[2]) if len(sys.argv) > 2 else 3)
