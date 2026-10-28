"""Minimal binary FBX 7.4 writer for static meshes, UCX collision and SOCKET_ nulls.

Coordinates passed in are in FBX space: right-handed, Z-up, centimeters.
(Unreal's FBX importer mirrors Y on import, so UE(x, y, z) = FBX(x, -y, z).)
"""
import struct, zlib
import numpy as np

_FILE_ID = bytes.fromhex('28b32aebb624ccc2bfc8b02aa92bfcf1')
_TIME_ID = '1970-01-01 10:00:00:000'
_FOOT_ID = bytes.fromhex('fabcab09d0c8d466b176fb831cf7267e')
_FOOT_MAGIC = bytes.fromhex('f85a8c6adef5d97eece90ce3758f290b')
VERSION = 7400


class N:
    def __init__(self, name, *props):
        self.name = name; self.props = list(props); self.children = []

    def add(self, name, *props):
        c = N(name, *props); self.children.append(c); return c

    def p70(self):
        return self.add('Properties70')


def _enc_prop(p):
    if isinstance(p, bool):
        return b'C' + struct.pack('<B', p)
    if isinstance(p, (int, np.integer)) and not isinstance(p, bool):
        p = int(p)
        if -2**31 <= p < 2**31:
            return b'I' + struct.pack('<i', p)
        return b'L' + struct.pack('<q', p)
    if isinstance(p, float):
        return b'D' + struct.pack('<d', p)
    if isinstance(p, str):
        b = p.encode('utf8'); return b'S' + struct.pack('<I', len(b)) + b
    if isinstance(p, bytes):
        return b'R' + struct.pack('<I', len(p)) + p
    if isinstance(p, tuple) and p[0] == 'L':  # forced int64
        return b'L' + struct.pack('<q', int(p[1]))
    if isinstance(p, np.ndarray):
        if p.dtype.kind == 'f':
            code, arr = b'd', p.astype('<f8')
        elif p.dtype == np.int64:
            code, arr = b'l', p.astype('<i8')
        else:
            code, arr = b'i', p.astype('<i4')
        raw = arr.tobytes()
        comp = zlib.compress(raw)
        if len(comp) < len(raw):
            return code + struct.pack('<III', len(arr), 1, len(comp)) + comp
        return code + struct.pack('<III', len(arr), 0, len(raw)) + raw
    raise TypeError(type(p))


def _write_node(buf, n, start):
    props = b''.join(_enc_prop(p) for p in n.props)
    name = n.name.encode()
    head_len = 13 + len(name)
    body = bytearray()
    off = start + head_len + len(props)
    for c in n.children:
        cb = _write_node(None, c, off)
        body += cb; off += len(cb)
    if n.children or not n.props:
        body += b'\0' * 13
    end = start + head_len + len(props) + len(body)
    out = struct.pack('<III', end, len(n.props), len(props)) + bytes([len(name)]) + name + props + bytes(body)
    return out


def _p(p70, name, typ, label, flags, *vals):
    p70.add('P', name, typ, label, flags, *vals)


class Scene:
    def __init__(self):
        self.objects = []      # N nodes
        self.conns = []        # (type, child, parent, prop?)
        self._id = 1000000
        self.counts = {}

    def nid(self):
        self._id += 1; return self._id

    def _count(self, k):
        self.counts[k] = self.counts.get(k, 0) + 1

    def material(self, name, rgb=(0.5, 0.5, 0.5), opacity=1.0):
        mid = self.nid()
        m = N('Material', ('L', mid), name + '\x00\x01Material', '')
        m.add('Version', 102); m.add('ShadingModel', 'phong'); m.add('MultiLayer', 0)
        p = m.p70()
        _p(p, 'DiffuseColor', 'Color', '', 'A', *map(float, rgb))
        _p(p, 'Diffuse', 'Vector3D', 'Vector', '', *map(float, rgb))
        _p(p, 'TransparencyFactor', 'Number', '', 'A', float(1 - opacity))
        _p(p, 'Opacity', 'double', 'Number', '', float(opacity))
        self.objects.append(m); self._count('Material')
        return mid

    def texture(self, mat_id, jpg_bytes, fname):
        vid, tid = self.nid(), self.nid()
        v = N('Video', ('L', vid), 'Tex\x00\x01Video', 'Clip')
        v.add('Type', 'Clip')
        p = v.p70(); _p(p, 'Path', 'KString', 'XRefUrl', '', fname)
        v.add('UseMipMap', 0); v.add('Filename', fname); v.add('RelativeFilename', fname)
        v.add('Content', jpg_bytes)
        t = N('Texture', ('L', tid), 'Tex\x00\x01Texture', '')
        t.add('Type', 'TextureVideoClip'); t.add('Version', 202); t.add('TextureName', 'Tex\x00\x01Texture')
        p = t.p70(); _p(p, 'UseMaterial', 'bool', '', '', 1)
        t.add('Media', 'Tex\x00\x01Video'); t.add('FileName', fname); t.add('RelativeFilename', fname)
        t.add('ModelUVTranslation', 0.0, 0.0); t.add('ModelUVScaling', 1.0, 1.0)
        t.add('Texture_Alpha_Source', 'None'); t.add('Cropping', 0, 0, 0, 0)
        self.objects += [v, t]; self._count('Video'); self._count('Texture')
        self.conns.append(('OO', vid, tid, None))
        self.conns.append(('OP', tid, mat_id, 'DiffuseColor'))

    def model(self, name, kind, parent=0, t=(0, 0, 0), r=(0, 0, 0)):
        mid = self.nid()
        m = N('Model', ('L', mid), name + '\x00\x01Model', kind)
        m.add('Version', 232)
        p = m.p70()
        _p(p, 'RotationActive', 'bool', '', '', 1)
        _p(p, 'InheritType', 'enum', '', '', 1)
        _p(p, 'ScalingMax', 'Vector3D', 'Vector', '', 0.0, 0.0, 0.0)
        _p(p, 'DefaultAttributeIndex', 'int', 'Integer', '', 0)
        _p(p, 'Lcl Translation', 'Lcl Translation', '', 'A', *map(float, t))
        _p(p, 'Lcl Rotation', 'Lcl Rotation', '', 'A', *map(float, r))
        _p(p, 'Lcl Scaling', 'Lcl Scaling', '', 'A', 1.0, 1.0, 1.0)
        m.add('Shading', True); m.add('Culling', 'CullingOff')
        self.objects.append(m); self._count('Model')
        self.conns.append(('OO', mid, parent, None))
        return mid

    def mesh(self, name, V, polys, mat_ids=None, materials=(), uv=None, parent=0):
        """V: (n,3) float; polys: list of index lists (CCW seen from outside);
        mat_ids: per-polygon material index; uv: per polygon-vertex (m,2) or None (auto planar)."""
        V = np.asarray(V, float)
        mid = self.model(name, 'Mesh', parent)
        gid = self.nid()
        g = N('Geometry', ('L', gid), name + '\x00\x01Geometry', 'Mesh')
        pvi, normals, uvs = [], [], []
        for poly in polys:
            P = V[poly]
            nrm = np.zeros(3)
            for i in range(len(P)):  # Newell normal
                a, b = P[i], P[(i + 1) % len(P)]
                nrm += [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]
            ln = np.linalg.norm(nrm); nrm = nrm / ln if ln > 0 else np.array([0, 0, 1.0])
            ax = np.argmax(np.abs(nrm))
            for j, vi in enumerate(poly):
                pvi.append(vi if j < len(poly) - 1 else -vi - 1)
                normals.append(nrm)
                if uv is None:
                    q = V[vi]
                    a, b = [(1, 2), (0, 2), (0, 1)][ax]
                    uvs.append((q[a] / 20.0, q[b] / 20.0))
        g.add('Vertices', V.reshape(-1).astype(float))
        g.add('PolygonVertexIndex', np.array(pvi, np.int32))
        g.add('GeometryVersion', 124)
        ln_ = g.add('LayerElementNormal', 0)
        ln_.add('Version', 101); ln_.add('Name', '')
        ln_.add('MappingInformationType', 'ByPolygonVertex'); ln_.add('ReferenceInformationType', 'Direct')
        ln_.add('Normals', np.array(normals, float).reshape(-1))
        lu = g.add('LayerElementUV', 0)
        lu.add('Version', 101); lu.add('Name', 'UVMap')
        lu.add('MappingInformationType', 'ByPolygonVertex'); lu.add('ReferenceInformationType', 'IndexToDirect')
        if uv is None:
            UV = np.array(uvs, float)
        else:
            UV = np.asarray(uv, float)
        lu.add('UV', UV.reshape(-1)); lu.add('UVIndex', np.arange(len(UV), dtype=np.int32))
        lm = g.add('LayerElementMaterial', 0)
        lm.add('Version', 101); lm.add('Name', '')
        if mat_ids is None:
            lm.add('MappingInformationType', 'AllSame'); lm.add('ReferenceInformationType', 'IndexToDirect')
            lm.add('Materials', np.zeros(1, np.int32))
        else:
            lm.add('MappingInformationType', 'ByPolygon'); lm.add('ReferenceInformationType', 'IndexToDirect')
            lm.add('Materials', np.asarray(mat_ids, np.int32))
        lay = g.add('Layer', 0); lay.add('Version', 100)
        for t in ('LayerElementNormal', 'LayerElementMaterial', 'LayerElementUV'):
            le = lay.add('LayerElement'); le.add('Type', t); le.add('TypedIndex', 0)
        self.objects.append(g); self._count('Geometry')
        self.conns.append(('OO', gid, mid, None))
        for m in materials:
            self.conns.append(('OO', m, mid, None))
        return mid

    def null(self, name, parent, t, r):
        aid = self.nid()
        a = N('NodeAttribute', ('L', aid), name + '\x00\x01NodeAttribute', 'Null')
        a.add('TypeFlags', 'Null')
        p = a.p70(); _p(p, 'Look', 'enum', '', '', 0)
        self.objects.append(a); self._count('NodeAttribute')
        mid = self.model(name, 'Null', parent, t, r)
        self.conns.append(('OO', aid, mid, None))
        return mid

    def save(self, path):
        root = []
        h = N('FBXHeaderExtension')
        h.add('FBXHeaderVersion', 1003); h.add('FBXVersion', VERSION); h.add('EncryptionType', 0)
        ts = h.add('CreationTimeStamp')
        for k, v in [('Version', 1000), ('Year', 2026), ('Month', 1), ('Day', 1), ('Hour', 0), ('Minute', 0), ('Second', 0), ('Millisecond', 0)]:
            ts.add(k, v)
        h.add('Creator', 'mini4wd-course-builder')
        root.append(h)
        root.append(N('FileId', _FILE_ID)); root.append(N('CreationTime', _TIME_ID)); root.append(N('Creator', 'mini4wd-course-builder'))
        gs = N('GlobalSettings'); gs.add('Version', 1000); p = gs.p70()
        for k, v in [('UpAxis', 2), ('UpAxisSign', 1), ('FrontAxis', 1), ('FrontAxisSign', -1), ('CoordAxis', 0), ('CoordAxisSign', 1),
                     ('OriginalUpAxis', 2), ('OriginalUpAxisSign', 1)]:
            _p(p, k, 'int', 'Integer', '', v)
        _p(p, 'UnitScaleFactor', 'double', 'Number', '', 1.0)
        _p(p, 'OriginalUnitScaleFactor', 'double', 'Number', '', 1.0)
        _p(p, 'TimeMode', 'enum', '', '', 11)
        root.append(gs)
        docs = N('Documents'); docs.add('Count', 1)
        d = docs.add('Document', ('L', 999999), '', 'Scene'); dp = d.p70()
        _p(dp, 'SourceObject', 'object', '', ''); _p(dp, 'ActiveAnimStackName', 'KString', '', '', '')
        d.add('RootNode', ('L', 0))
        root.append(docs); root.append(N('References'))
        defs = N('Definitions'); defs.add('Version', 100)
        defs.add('Count', sum(self.counts.values()) + 1)
        ot = defs.add('ObjectType', 'GlobalSettings'); ot.add('Count', 1)
        for k, v in self.counts.items():
            ot = defs.add('ObjectType', k); ot.add('Count', v)
        root.append(defs)
        objs = N('Objects'); objs.children = self.objects; root.append(objs)
        con = N('Connections')
        for typ, c, pnt, prop in self.conns:
            if prop is None: con.add('C', typ, ('L', c), ('L', pnt))
            else: con.add('C', typ, ('L', c), ('L', pnt), prop)
        root.append(con)
        out = bytearray(b'Kaydara FBX Binary  \x00\x1a\x00' + struct.pack('<I', VERSION))
        for n in root:
            out += _write_node(None, n, len(out))
        out += b'\0' * 13
        out += _FOOT_ID
        pad = ((len(out) + 15) & ~15) - len(out)
        if pad == 0: pad = 16
        out += b'\0' * pad + struct.pack('<I', VERSION) + b'\0' * 120 + _FOOT_MAGIC
        with open(path, 'wb') as f:
            f.write(out)
