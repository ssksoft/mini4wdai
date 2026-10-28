"""Generate clean, drivable Mini 4WD course parts as FBX for Unreal Engine.

Geometry is built in a right-handed FBX frame F: X = travel direction, Y = LEFT, Z = up (mm),
written in centimeters. Unreal imports this as X forward, Y right, Z up (Y mirrored by the importer).

Every part has:   SM_<Part>            visual mesh (material slots M_Track / M_Cover)
                  UCX_SM_<Part>_NN     convex collision pieces (floor slabs, walls)
                  SOCKET_In / SOCKET_Out (+ SOCKET_BridgeMount on the lane change base)
"""
import json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fbxwrite import Scene

MM2CM = 0.1


class Builder:
    def __init__(self):
        self.V = []; self.polys = []; self.mats = []; self.hulls = []

    def _add_poly(self, pts, mat, inside_ref):
        """pts: list of 3D points; orient polygon so its normal points away from inside_ref."""
        P = np.array(pts)
        n = np.zeros(3)
        for i in range(len(P)):
            a, b = P[i], P[(i + 1) % len(P)]
            n += [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]
        if np.dot(n, P.mean(0) - inside_ref) < 0:
            P = P[::-1]
        base = len(self.V)
        self.V.extend(P.tolist())
        self.polys.append(list(range(base, base + len(P))))
        self.mats.append(mat)

    def loft(self, rings, mat=0, collide=True, coll_every=1, cap=True, visual=True):
        """rings: list of (4,3) arrays (rectangular cross sections along a path)."""
        rings = [np.asarray(r, float) for r in rings]
        if not visual:
            cap = False
        for a, b in (zip(rings[:-1], rings[1:]) if visual else []):
            c = (a.mean(0) + b.mean(0)) / 2
            for i in range(4):
                j = (i + 1) % 4
                self._add_poly([a[i], a[j], b[j], b[i]], mat, c)
        if cap:
            self._add_poly(list(rings[0]), mat, rings[1].mean(0))
            self._add_poly(list(rings[-1]), mat, rings[-2].mean(0))
        if collide:
            idx = list(range(0, len(rings), coll_every))
            if idx[-1] != len(rings) - 1: idx.append(len(rings) - 1)
            for i0, i1 in zip(idx[:-1], idx[1:]):
                self.hulls.append(np.vstack([rings[i0], rings[i1]]))


def rect(p, lat, y0, y1, z0, z1):
    """Cross-section rectangle at path point p with horizontal lateral axis lat (points to +Y/left)."""
    p = np.asarray(p, float); lat = np.asarray(lat, float)
    up = np.array([0, 0, 1.0])
    return np.array([p + lat * y0 + up * z0, p + lat * y1 + up * z0, p + lat * y1 + up * z1, p + lat * y0 + up * z1])


def box_polys(H):
    """8 hull verts (ring a: 0-3, ring b: 4-7) -> 6 quads, oriented outward."""
    c = H.mean(0)
    faces = [[0, 1, 2, 3], [4, 5, 6, 7]] + [[i, (i + 1) % 4, 4 + (i + 1) % 4, 4 + i] for i in range(4)]
    out = []
    for f in faces:
        P = H[f]
        n = np.cross(P[1] - P[0], P[2] - P[0])
        if np.linalg.norm(n) < 1e-9: n = np.cross(P[2] - P[0], P[3] - P[0])
        if np.dot(n, P.mean(0) - c) < 0: f = f[::-1]
        out.append(f)
    return out


# ------------------------------------------------------------------ part definitions
def wall(b, prm, stations, yc, z0, ztop, outward, every=1, mat=0):
    """Visual wall of real thickness; collision wall thickened by `collision_extra_thickness`
    on the outward side (0 = shared wall between two lanes -> not thickened)."""
    t = prm['wall_thickness']; ex = prm.get('collision_extra_thickness', 0.0)
    ring = lambda p, l, y0, y1: rect(p, l, y0, y1, z0(p) if callable(z0) else z0, ztop(p) if callable(ztop) else ztop)
    b.loft([ring(p, l, yc - t / 2, yc + t / 2) for p, l in stations], mat=mat, collide=False)
    y0, y1 = yc - t / 2, yc + t / 2
    if outward > 0: y1 += ex
    elif outward < 0: y0 -= ex
    b.loft([ring(p, l, y0, y1) for p, l in stations], collide=True, coll_every=every, visual=False)


def floor(b, prm, stations, y0, y1, ztop, every=1):
    """Visual floor slab 0..ztop; collision slab extends `collision_extra_thickness` below 0."""
    ex = prm.get('collision_extra_thickness', 0.0)
    zt = (lambda p: ztop(p)) if callable(ztop) else (lambda p: ztop)
    b.loft([rect(p, l, y0, y1, 0, zt(p)) for p, l in stations], collide=False)
    b.loft([rect(p, l, y0, y1, -ex, zt(p)) for p, l in stations], collide=True, coll_every=every, visual=False)

def lateral_layout(prm):
    pitch = prm['lane_width'] + prm['wall_thickness']
    t = prm['wall_thickness']
    return pitch, t


def straight(prm, length=None, coll=True):
    b = Builder()
    L = length or prm['straight_length']
    pitch, t = lateral_layout(prm)
    W = pitch + t / 2
    lat = [0, 1, 0]
    st = [(np.array([0.0, 0, 0]), lat), (np.array([L, 0, 0.0]), lat)]
    floor(b, prm, st, -W, W, prm['floor_top'])
    for yc in (-pitch, 0.0, pitch):
        wall(b, prm, st, yc, 0, prm['floor_top'] + prm['wall_height'], np.sign(yc))
    sockets = {'In': ([0, 0, 0], 0.0), 'Out': ([L, 0, 0], 0.0)}
    return b, sockets


def curve(prm, left=True):
    b = Builder()
    pitch, t = lateral_layout(prm)
    W = pitch + t / 2
    Rc = prm['curve_center_radius']; A = prm['curve_angle']
    sgn = 1.0 if left else -1.0          # F: +Y is left
    nvis = max(2, int(round(prm['visual_segments_per_45deg'] * A / 45.0)))
    ncol = max(1, int(round(prm['collision_segments_per_45deg'] * A / 45.0)))
    every = max(1, nvis // ncol)
    th = np.radians(np.linspace(0, A, nvis + 1))
    C = np.array([0, sgn * Rc, 0])
    # lateral axis must stay "left of travel": rotate (0,1,0) by heading
    stations = []
    for a in th:
        h = sgn * a                                  # heading (yaw, CCW+)
        p = C + np.array([math.sin(a) * Rc, -sgn * math.cos(a) * Rc, 0])
        lat = np.array([-math.sin(h), math.cos(h), 0])
        stations.append((p, lat))
    floor(b, prm, stations, -W, W, prm['floor_top'], every)
    for yc in (-pitch, 0.0, pitch):
        wall(b, prm, stations, yc, 0, prm['floor_top'] + prm['wall_height'], np.sign(yc), every)
    pe, _ = stations[-1]
    sockets = {'In': ([0, 0, 0], 0.0), 'Out': (pe.tolist(), sgn * A)}
    return b, sockets


def s_offset(x, x0, x1, total):
    """Cosine S-curve lateral offset (0 -> total) between x0 and x1, plus its slope."""
    u = np.clip((x - x0) / (x1 - x0), 0, 1)
    off = total * (1 - np.cos(np.pi * u)) / 2
    slope = total * np.pi / (x1 - x0) * np.sin(np.pi * u) / 2 if x0 <= x <= x1 else 0.0
    return off, slope


def lanechange_base(prm):
    """Ramp up in the RIGHT lane (F: -Y) over [0, split]; the LEFT lane continues on the ground and
    starts an S-curve towards the right lane (reaches the centre line at the far end).
    Two of these, the second one attached reversed, form the full crossing."""
    b = Builder()
    pitch, t = lateral_layout(prm)
    W = pitch + t / 2
    L = prm['lanechange_length']; split = prm['lanechange_split']
    ft, wh = prm['floor_top'], prm['wall_height']
    lat = [0, 1, 0]
    # --- ramp lane (right, F y<0)
    xs = [0.0, prm['ramp_start'], prm['ramp_end'], split]
    zs = [ft, ft, prm['ramp_top'], prm['ramp_top']]
    lat = np.array(lat, float)
    zmap = dict(zip(xs, zs))
    floor(b, prm, [(np.array([x, 0, 0.0]), lat) for x in xs], -W, 0.0, lambda p: zmap[p[0]])
    wall_top = lambda p: (ft + wh) + (prm['ramp_wall_top_at_end'] - ft - wh) * p[0] / split
    st2 = [(np.array([0.0, 0, 0]), lat), (np.array([split, 0, 0.0]), lat)]
    for yc in (-pitch, 0.0):
        wall(b, prm, st2, yc, 0, wall_top, np.sign(yc))
    # --- ground lane (left, F y>0), straight part
    floor(b, prm, st2, 0.0, W, ft)
    wall(b, prm, st2, pitch, 0, ft + wh, 1)
    # --- ground lane S-curve: centre goes from +pitch/2 towards -pitch/2 over [split, 2L - split]
    x0, x1 = split, 2 * L - split
    step = prm['s_curve_visual_step']
    xs = list(np.arange(split, L, step)) + [L]
    every = max(1, int(round(prm['s_curve_collision_step'] / step)))
    st = []
    for x in xs:
        off, slope = s_offset(x, x0, x1, pitch)
        yc = pitch / 2 - off
        tang = np.array([1.0, -slope, 0]); tang /= np.linalg.norm(tang)
        latv = np.array([-tang[1], tang[0], 0])
        st.append((np.array([x, yc, 0.0]), latv))
    half = prm['lane_width'] / 2 + t / 2
    floor(b, prm, st, -half - t / 2, half + t / 2, ft, every)
    for s in (-1, 1):
        wall(b, prm, st, s * half, 0, ft + wh, s, every)
    sockets = {'In': ([0, 0, 0], 0.0), 'Out': ([L, 0, 0], 0.0),
               'BridgeMount': ([split, -pitch / 2, prm['ramp_top']], 0.0)}
    return b, sockets


def bridge(prm):
    """Elevated S-shaped single lane: starts at the right lane (local y=0), ends one lane pitch to the left."""
    b = Builder()
    pitch, t = lateral_layout(prm)
    L = 2 * (prm['lanechange_length'] - prm['lanechange_split'])
    step = prm['s_curve_visual_step']
    every = max(1, int(round(prm['s_curve_collision_step'] / step)))
    xs = list(np.arange(0, L, step)) + [L]
    st = []
    for x in xs:
        off, slope = s_offset(x, 0, L, pitch)
        z = prm['bridge_hump'] * (1 - math.cos(2 * math.pi * x / L)) / 2
        tang = np.array([1.0, slope, 0]); tang /= np.linalg.norm(tang)
        st.append((np.array([x, off, z]), np.array([-tang[1], tang[0], 0])))
    half = prm['lane_width'] / 2 + t / 2
    dt, wh = prm['bridge_deck_thickness'], prm['wall_height']
    # rect() offsets by p (incl. its z), so deck/wall heights below are relative to the deck surface
    ex = prm.get('collision_extra_thickness', 0.0)
    b.loft([rect(p, l, -half - t / 2, half + t / 2, -dt, 0) for p, l in st], collide=False)
    b.loft([rect(p, l, -half - t / 2, half + t / 2, -dt - ex, 0) for p, l in st], coll_every=every, visual=False)
    for s in (-1, 1):
        wall(b, prm, st, s * half, -dt, wh, s, every)
    if prm.get('bridge_cover'):
        ct = prm['bridge_cover_thickness']
        b.loft([rect(p, l, -half - t / 2, half + t / 2, wh, wh + ct) for p, l in st], mat=1, collide=False)
        b.loft([rect(p, l, -half - t / 2 - ex, half + t / 2 + ex, wh, wh + ct + ex) for p, l in st], coll_every=every, visual=False)
    sockets = {'In': ([0, 0, 0], 0.0), 'Out': ([L, pitch, 0], 0.0)}
    return b, sockets


# ------------------------------------------------------------------ export
def to_ue(p_mm, yaw_f):
    """F (mm, Y-left) -> Unreal (cm, Y-right)."""
    return [round(p_mm[0] * MM2CM, 4), round(-p_mm[1] * MM2CM, 4), round(p_mm[2] * MM2CM, 4)], round(-yaw_f, 4)


def export(name, b, sockets, outdir, manifest, color):
    sc = Scene()
    m_track = sc.material('M_Track', color)
    mats = [m_track]
    if 1 in b.mats:
        mats.append(sc.material('M_Cover', (0.8, 0.9, 1.0), opacity=0.25))
    V = np.array(b.V) * MM2CM
    mesh = sc.mesh('SM_' + name, V, b.polys, b.mats if len(mats) > 1 else None, mats)
    for i, H in enumerate(b.hulls):
        Hc = H * MM2CM
        sc.mesh('UCX_SM_%s_%02d' % (name, i), Hc, box_polys(Hc), None, [m_track])
    for s, (p, yaw) in sockets.items():
        sc.null('SOCKET_' + s, mesh, np.array(p) * MM2CM, (0, 0, yaw))
    path = os.path.join(outdir, 'SM_%s.fbx' % name)
    sc.save(path)
    manifest[name] = {
        'file': os.path.basename(path),
        'sockets_ue': {s: {'location_cm': to_ue(p, y)[0], 'yaw_deg': to_ue(p, y)[1]} for s, (p, y) in sockets.items()},
        'triangles': int(sum(len(p) - 2 for p in b.polys)),
        'collision_hulls': len(b.hulls),
    }
    return path


PARTS = {
    'Straight':        (lambda p: straight(p), (0.03, 0.03, 0.03)),
    'Curve45_L':       (lambda p: curve(p, True), (0.45, 0.85, 0.15)),
    'Curve45_R':       (lambda p: curve(p, False), (0.10, 0.25, 0.85)),
    'LaneChange_Base': (lambda p: lanechange_base(p), (0.03, 0.03, 0.03)),
    'LaneChange_Bridge': (lambda p: bridge(p), (0.03, 0.03, 0.03)),
}

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    prm = json.load(open(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'track_params.json')))
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, '..', 'fbx')
    os.makedirs(outdir, exist_ok=True)
    manifest = {'units': 'Unreal: cm, X forward, Y right, Z up. yaw + = turn right', 'parts': {}}
    for name, (fn, col) in PARTS.items():
        b, sockets = fn(prm)
        export(name, b, sockets, outdir, manifest['parts'], col)
        print('%-18s tris %5d  hulls %3d' % (name, manifest['parts'][name]['triangles'], manifest['parts'][name]['collision_hulls']))
    json.dump(manifest, open(os.path.join(outdir, 'parts_manifest.json'), 'w'), indent=2, ensure_ascii=False)
