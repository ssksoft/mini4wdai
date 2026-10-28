"""Offline check of a course layout: chains parts by their sockets (same math as the UE script),
reports loop-closure error, and renders a preview. Reads the generated FBX files directly."""
import json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fbxread import load, find, child

def read_fbx(path):
    ver, nodes = load(path)
    objs = find(nodes, 'Objects')[0]['children']
    con = find(nodes, 'Connections')[0]['children']
    geos = {o['props'][0]: o for o in objs if o['name'] == 'Geometry'}
    models = {o['props'][0]: o for o in objs if o['name'] == 'Model'}
    out = {}
    for c in con:
        _, a, b = c['props'][:3]
        if a in geos and b in models:
            g = geos[a]
            V = child(g, 'Vertices')['props'][0].reshape(-1, 3)
            pvi = child(g, 'PolygonVertexIndex')['props'][0]
            polys, cur = [], []
            for i in pvi:
                if i < 0: cur.append(-i - 1); polys.append(cur); cur = []
                else: cur.append(i)
            name = models[b]['props'][1].split('\x00')[0]
            out[name] = (V * [1, -1, 1], polys)   # -> UE coords (cm)
    return out

def compose(T, S):
    x, y, z, yaw = T; c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (x + c * S[0] - s * S[1], y + s * S[0] + c * S[1], z + S[2], yaw + S[3])

def inverse(S):
    c, s = math.cos(math.radians(-S[3])), math.sin(math.radians(-S[3]))
    return (-(c * S[0] - s * S[1]), -(s * S[0] + c * S[1]), -S[2], -S[3])

def sock(man, part, name):
    s = man['parts'][part]['sockets_ue'][name]
    return tuple(s['location_cm']) + (s['yaw_deg'],)

TOKENS = {'S': ('Straight', False), 'L': ('Curve45_L', False), 'R': ('Curve45_R', False),
          'LC_UP': ('LaneChange_Base', False), 'LC_DOWN': ('LaneChange_Base', True)}

def build(man, tokens):
    T = (0.0, 0.0, 0.0, 0.0); placed = []
    for tok in tokens:
        part, rev = TOKENS[tok]
        IN, OUT = sock(man, part, 'In'), sock(man, part, 'Out')
        if not rev:
            W = compose(T, inverse(IN)); T = compose(W, OUT)
        else:
            W = compose(compose(T, (0, 0, 0, 180.0)), inverse(OUT)); T = compose(compose(W, IN), (0, 0, 0, 180.0))
        placed.append((part, W))
        if tok == 'LC_UP':
            Wb = compose(compose(W, sock(man, part, 'BridgeMount')), inverse(sock(man, 'LaneChange_Bridge', 'In')))
            placed.append(('LaneChange_Bridge', Wb))
    return placed, T

def xform(V, W):
    c, s = math.cos(math.radians(W[3])), math.sin(math.radians(W[3]))
    return np.c_[W[0] + c * V[:, 0] - s * V[:, 1], W[1] + s * V[:, 0] + c * V[:, 1], W[2] + V[:, 2]]

if __name__ == '__main__':
    d = sys.argv[1]; tokens = sys.argv[2].split(','); out = sys.argv[3]
    man = json.load(open(os.path.join(d, 'parts_manifest.json')))
    placed, T = build(man, tokens)
    err = math.hypot(T[0], T[1]); yerr = (T[3] + 180) % 360 - 180
    print('end transform', [round(v, 3) for v in T], ' closure gap %.3f cm, yaw %.3f deg' % (err, yerr))
    meshes = {p: read_fbx(os.path.join(d, man['parts'][p]['file'])) for p in set(p for p, _ in placed)}
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    cols = {'Straight': '#222', 'Curve45_L': '#7c3', 'Curve45_R': '#36c', 'LaneChange_Base': '#444', 'LaneChange_Bridge': '#888'}
    fig = plt.figure(figsize=(18, 9))
    ax = fig.add_subplot(1, 2, 1)
    ax3 = fig.add_subplot(1, 2, 2, projection='3d')
    allp = []
    for part, W in placed:
        V, polys = meshes[part]['SM_' + part]
        Vw = xform(V, W); allp.append(Vw)
        P2 = [Vw[p][:, :2] * [1, -1] for p in polys]          # plot UE Y downward (top view, X right)
        ax.add_collection(PolyCollection(P2, facecolors=cols[part], edgecolors='none', alpha=0.85))
        ax3.add_collection3d(Poly3DCollection([Vw[p] * [1, -1, 1] for p in polys], facecolors=cols[part], edgecolors='none', alpha=0.9))
    A = np.vstack(allp)
    ax.set_xlim(A[:, 0].min() - 5, A[:, 0].max() + 5); ax.set_ylim(-A[:, 1].max() - 5, -A[:, 1].min() + 5)
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_title('top view (UE X right, UE +Y down)   gap %.2f cm' % err)
    ax3.set_xlim(A[:, 0].min(), A[:, 0].max()); ax3.set_ylim(-A[:, 1].max(), -A[:, 1].min()); ax3.set_zlim(0, (A[:, 0].max() - A[:, 0].min()) / 3)
    ax3.view_init(30, -60); ax3.set_box_aspect((np.ptp(A[:, 0]), np.ptp(A[:, 1]), (np.ptp(A[:, 0])) / 3))
    fig.tight_layout(); fig.savefig(out, dpi=80)
    print('extent cm: X %.1f  Y %.1f' % (np.ptp(A[:, 0]), np.ptp(A[:, 1])))
