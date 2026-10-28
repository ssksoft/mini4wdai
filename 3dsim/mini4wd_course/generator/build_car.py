"""Tamiya 1/32 Mad Racer (MA chassis) - simplified drivable model for Unreal.

Official specs (tamiya.com): length 156, width 97, height 36 mm, wheelbase 80, tread 59.5 mm,
weight 125.9 g, ground clearance 2.2 mm, gear ratio 3.5:1, 6 low-friction rollers,
small-diameter low-profile tyres (~24 mm, assumed).

Frame F (right-handed): X forward, Y left, Z up, mm. Origin = ground point midway between axles.
Output: SM_M4WD_MadRacer.fbx with
  visual mesh (materials: Body, Accent, Canopy, Chassis, Tire, Wheel, Roller)
  UCX_ chassis box + USP_ spheres for the 4 tyres and 6 rollers (all frictionless in UE;
  tyre grip / motor are applied by the runtime script).
"""
import json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fbxwrite import Scene

CAR = {
    "length": 156.0, "width": 97.0, "height": 36.0,
    "wheelbase": 80.0, "tread": 59.5, "mass_kg": 0.1259, "ground_clearance": 2.2,
    "gear_ratio": 3.5, "tire_radius": 12.0, "tire_width": 8.5,
    "roller_radius": 6.5, "roller_height": 6.0,
    "rollers": [  # x, y(abs), z  (2 front, 2 rear lower, 2 rear upper)
        [71.5, 42.0, 9.0], [-71.5, 42.0, 9.0], [-66.0, 42.0, 22.0]],
    "chassis_box": [-68.0, 68.0, -35.0, 35.0, 3.0, 30.0],
    "motor": {"no_load_rpm": 14000.0, "note": "standard double-shaft motor, approx."},
}


class Mesh:
    def __init__(self):
        self.V, self.P, self.M = [], [], []

    def poly(self, pts, mat, inside):
        P = np.array(pts, float)
        n = np.zeros(3)
        for i in range(len(P)):
            a, b = P[i], P[(i + 1) % len(P)]
            n += [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]
        if np.dot(n, P.mean(0) - inside) < 0:
            P = P[::-1]
        base = len(self.V); self.V += P.tolist(); self.P.append(list(range(base, base + len(P)))); self.M.append(mat)

    def loft(self, rings, mat, cap=True, cap_mat=None):
        rings = [np.asarray(r, float) for r in rings]
        k = len(rings[0])
        for a, b in zip(rings[:-1], rings[1:]):
            c = (a.mean(0) + b.mean(0)) / 2
            for i in range(k):
                j = (i + 1) % k
                self.poly([a[i], a[j], b[j], b[i]], mat, c)
        if cap:
            cm = mat if cap_mat is None else cap_mat
            self.poly(list(rings[0]), cm, rings[1].mean(0))
            self.poly(list(rings[-1]), cm, rings[-2].mean(0))

    def box(self, x0, x1, y0, y1, z0, z1, mat):
        r = lambda x: [[x, y0, z0], [x, y1, z0], [x, y1, z1], [x, y0, z1]]
        self.loft([r(x0), r(x1)], mat)

    def cylinder(self, c, axis, r, h, mat, n=20, cap_mat=None):
        c = np.asarray(c, float); axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
        u = np.cross(axis, [0, 0, 1.0]) if abs(axis[2]) < 0.9 else np.cross(axis, [1.0, 0, 0])
        u /= np.linalg.norm(u); v = np.cross(axis, u)
        ring = lambda o: [o + r * (math.cos(t) * u + math.sin(t) * v) for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
        self.loft([ring(c - axis * h / 2), ring(c + axis * h / 2)], mat, cap_mat=cap_mat)


def body_section(x):
    """(half width, bottom z, shoulder z, top z, top half width) of the body shell along X (mm)."""
    # nose x=75 .. tail x=-74 ; low wedge rising to the canopy
    xs = [77, 64, 53, 35, 10, -15, -40, -54, -66, -76]
    hw = [30, 36, 38, 39, 40, 40, 40, 40, 39, 36]
    zb = [9, 9, 9, 9, 10, 10, 10, 10, 10, 11]
    zs = [12, 19, 25.5, 26, 25, 25, 25, 26, 25, 23]
    zt = [13, 21, 27, 28.5, 30, 30, 29, 28.5, 27, 25]
    tw = [22, 26, 28, 28, 28, 28, 28, 28, 28, 26]
    f = lambda arr: float(np.interp(-x, [-v for v in xs], arr))
    return f(hw), f(zb), f(zs), f(zt), f(tw)


def build(outdir):
    m = Mesh()
    BODY, ACC, CAN, CHS, TIRE, WHEEL, ROL = range(7)
    wb2, tr2 = CAR['wheelbase'] / 2, CAR['tread'] / 2
    R, TW = CAR['tire_radius'], CAR['tire_width']
    # chassis tub
    m.box(-66, 66, -33, 33, CAR['ground_clearance'], 11, CHS)
    # bumpers / roller stays
    m.box(62, 73, -42, 42, 5, 8, CHS)
    m.box(-73, -62, -42, 42, 5, 8, CHS)
    m.box(-70, -62, -42, 42, 18, 20, CHS)
    for yy in (-42, 42):
        m.box(-70, -62, yy - 2, yy + 2, 8, 18, CHS)
    # body shell (lofted)
    rings = []
    for x in np.linspace(77, -76, 24):
        hw, zb, zs, zt, tw = body_section(x)
        rings.append([[x, -hw, zb], [x, hw, zb], [x, hw, zs], [x, tw, zt], [x, -tw, zt], [x, -hw, zs]])
    m.loft(rings, BODY)
    # canopy
    can = []
    for x, h, w in [(18, 28, 18), (8, 34, 20), (-12, 36, 20), (-30, 31, 18), (-40, 27.5, 16)]:
        can.append([[x, -w, 28], [x, w, 28], [x, w * 0.8, h], [x, -w * 0.8, h]])
    m.loft(can, CAN)
    # red accents: side stripes and rear wing
    for s in (-1, 1):
        stripe = []
        for x in (55, 20, -20, -55):
            hw, zb, zs, zt, tw = body_section(x)
            y = s * (hw + 0.4)
            stripe.append([[x, y, zs - 4], [x, y + s * 0.6, zs - 4], [x, y + s * 0.6, zs - 1], [x, y, zs - 1]])
        m.loft(stripe, ACC)
    m.box(-76, -64, -36, 36, 29, 32, ACC)
    for yy in (-30, 30):
        m.box(-72, -66, yy - 1.5, yy + 1.5, 24, 29, ACC)
    # wheels: tyre + gold hub
    for xx in (-wb2, wb2):
        for s in (-1, 1):
            c = [xx, s * tr2, R]
            m.cylinder(c, [0, 1, 0], R, TW, TIRE, n=24)
            m.cylinder([xx, s * (tr2 + TW / 2 + 0.3), R], [0, 1, 0], R * 0.62, 0.8, WHEEL, n=24)
    # rollers (vertical axis)
    rr, rh = CAR['roller_radius'], CAR['roller_height']
    for x, y, z in CAR['rollers']:
        for s in (-1, 1):
            m.cylinder([x, s * y, z], [0, 0, 1], rr, rh, ROL, n=18)

    sc = Scene()
    mats = [sc.material('M_Car_Body', (0.75, 0.76, 0.78)), sc.material('M_Car_Accent', (0.75, 0.05, 0.04)),
            sc.material('M_Car_Canopy', (0.02, 0.02, 0.025)), sc.material('M_Car_Chassis', (0.35, 0.36, 0.38)),
            sc.material('M_Car_Tire', (0.02, 0.02, 0.02)), sc.material('M_Car_Wheel', (0.85, 0.65, 0.2)),
            sc.material('M_Car_Roller', (0.8, 0.8, 0.78))]
    flipY = np.array([1, 1, 1.0])
    name = 'SM_M4WD_MadRacer'
    V = np.array(m.V) * 0.1
    mid = sc.mesh(name, V, m.P, m.M, mats)
    # collision
    x0, x1, y0, y1, z0, z1 = CAR['chassis_box']
    cb = Mesh(); cb.box(x0, x1, y0, y1, z0, z1, 0)
    sc.mesh('UCX_%s_00' % name, np.array(cb.V) * 0.1, cb.P, None, [mats[0]])
    k = 0
    def sphere(c, r):
        nonlocal k
        sm = Mesh()
        # UV sphere vertices -> polygons (only verts matter for the importer's sphere fit)
        n1, n2 = 6, 12            # 12 around (contains 0/90/180/270 deg) + poles -> square bounding box
        rings = []
        for i in range(1, n1):
            ph = math.pi * i / n1
            rings.append([[c[0] + r * math.sin(ph) * math.cos(t), c[1] + r * math.sin(ph) * math.sin(t), c[2] + r * math.cos(ph)]
                          for t in np.linspace(0, 2 * math.pi, n2, endpoint=False)])
        sm.loft(rings, 0, cap=False)
        top, bot = np.array([c[0], c[1], c[2] + r]), np.array([c[0], c[1], c[2] - r])
        cen = np.array(c, float)
        for j in range(n2):
            k2 = (j + 1) % n2
            sm.poly([top, rings[0][j], rings[0][k2]], 0, cen)
            sm.poly([bot, rings[-1][j], rings[-1][k2]], 0, cen)
        sc.mesh('USP_%s_%02d' % (name, k), np.array(sm.V) * 0.1, sm.P, None, [mats[0]])
        k += 1
    for xx in (-wb2, wb2):
        for s in (-1, 1):
            sphere([xx, s * tr2, R], R)
    for x, y, z in CAR['rollers']:
        for s in (-1, 1):
            sphere([x, s * y, z], rr)
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, name + '.fbx')
    sc.save(path)
    spec = dict(CAR)
    spec['fbx'] = os.path.basename(path)
    spec['bounds_mm'] = {'x': [float(np.min(np.array(m.V)[:, 0])), float(np.max(np.array(m.V)[:, 0]))],
                         'y': [float(np.min(np.array(m.V)[:, 1])), float(np.max(np.array(m.V)[:, 1]))],
                         'z': [float(np.min(np.array(m.V)[:, 2])), float(np.max(np.array(m.V)[:, 2]))]}
    json.dump(spec, open(os.path.join(outdir, 'car_spec.json'), 'w'), indent=2)
    print('car tris', sum(len(p) - 2 for p in m.P), 'bounds', spec['bounds_mm'])
    return path, m


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'car'))
