"""Drivability test on the COLLISION hulls (UCX) of an assembled layout.
A virtual Mini 4WD (point probe) is guided by the side walls like the real car's rollers:
it ray-casts down for the floor and sideways for the walls, recentres in the lane and moves on.
Fails if it loses the floor, loses a wall, or hits something head-on."""
import json, math, os, sys
import numpy as np
from scipy.spatial import ConvexHull
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from layout_preview import read_fbx, build, xform, TOKENS

def load_hulls(d, man, placed):
    planes = []
    cache = {}
    for part, W in placed:
        if part not in cache:
            cache[part] = [v for k, v in read_fbx(os.path.join(d, man['parts'][part]['file'])).items() if k.startswith('UCX_')]
        for V, _ in cache[part]:
            Vw = xform(V, W)
            h = ConvexHull(Vw)
            planes.append(h.equations)        # n.x + d <= 0 inside
    m = max(len(p) for p in planes)
    N = np.zeros((len(planes), m, 3)); D = np.full((len(planes), m), -1e9)
    for i, p in enumerate(planes):
        N[i, :len(p)] = p[:, :3]; D[i, :len(p)] = p[:, 3]
    return N, D

def raycast(N, D, o, dirn, tmax):
    """nearest hit distance of ray o + t*dir against all convex hulls (from outside); inf if none."""
    nd = N @ dirn                       # (H,F)
    no = N @ o + D                      # (H,F)  signed dist of origin
    with np.errstate(divide='ignore', invalid='ignore'):
        t = -no / nd
    enter = np.where(nd < 0, t, -np.inf).max(1)
    exit_ = np.where(nd > 0, t, np.inf).min(1)
    par_out = ((np.abs(nd) < 1e-12) & (no > 0)).any(1)
    inside0 = (no <= 1e-9).all(1)
    ok = (enter <= exit_) & ~par_out & (exit_ >= 0) & ~inside0
    te = np.where(ok, np.maximum(enter, 0), np.inf)
    te[te > tmax] = np.inf
    return te.min()

def inside_any(N, D, p):
    return ((N @ p + D) <= -0.05).all(1).any()

def drive(N, D, start, heading, max_dist=2400.0, step=1.0, lane=11.5):
    p = np.array(start, float); h = heading; dist = 0.0; path = []
    while dist < max_dist:
        f = np.array([math.cos(h), math.sin(h), 0.0]); l = np.array([-f[1], f[0], 0.0])
        # floor under the probe
        dz = raycast(N, D, p + [0, 0, 3.0], np.array([0, 0, -1.0]), 30.0)
        if not np.isfinite(dz): return path, dist, 'fell (no floor) at %s' % np.round(p, 1)
        zf = p[2] + 3.0 - dz
        p[2] = zf + 1.5
        # walls left / right at roller height
        q = p + [0, 0, 1.0]
        dl = raycast(N, D, q, -l, 20.0); dr = raycast(N, D, q, l, 20.0)   # UE: +Y is right, l = +90deg yaw = right
        if not (np.isfinite(dl) and np.isfinite(dr)):
            return path, dist, 'lost wall at %s (dl=%s dr=%s)' % (np.round(p, 1), dl, dr)
        if dl + dr < lane - 1.0:
            return path, dist, 'lane too narrow (%.1f cm) at %s' % (dl + dr, np.round(p, 1))
        # look ahead and steer towards the lane centre there
        a = p + f * 4.0
        dza = raycast(N, D, a + [0, 0, 3.0], np.array([0, 0, -1.0]), 30.0)
        za = a[2] + 3.0 - dza if np.isfinite(dza) else p[2]
        qa = np.array([a[0], a[1], za + 2.5])
        dla = raycast(N, D, qa, -l, 20.0); dra = raycast(N, D, qa, l, 20.0)
        if np.isfinite(dla) and np.isfinite(dra):
            c = qa + l * (dra - dla) / 2
            h = math.atan2(c[1] - p[1], c[0] - p[0])
        f = np.array([math.cos(h), math.sin(h), 0.0])
        # head-on obstacle?
        hit = raycast(N, D, p + [0, 0, 0.5], f, step + 0.3)
        if np.isfinite(hit):
            # allow climbing a ramp: obstacle only if it is taller than a few mm
            hit2 = raycast(N, D, p + [0, 0, 1.5], f, step + 0.3)
            if np.isfinite(hit2): return path, dist, 'blocked at %s' % np.round(p, 1)
        p = p + f * step; dist += step
        path.append(p.copy())
    return path, dist, 'ok'

if __name__ == '__main__':
    d = sys.argv[1]; tokens = sys.argv[2].split(',')
    man = json.load(open(os.path.join(d, 'parts_manifest.json')))
    placed, _ = build(man, tokens)
    N, D = load_hulls(d, man, placed)
    print('hulls', len(N))
    lane_off = (man['parts']['Straight']['sockets_ue']['Out']['location_cm'][0] * 0 + 5.85)
    results = {}
    for name, y0 in (('right lane start', +5.85), ('left lane start', -5.85)):
        # one lap = come back to x=0 heading +X; run 2 laps
        path, dist, status = drive(N, D, [1.0, y0, 0.0], 0.0, max_dist=float(sys.argv[3]) if len(sys.argv) > 3 else 1200.0)
        P = np.array(path)
        # lap detection: crossings of x=1 plane going +X near start
        laps = [i for i in range(1, len(P)) if P[i - 1][0] < 1.0 <= P[i][0] and abs(P[i][1]) < 15]
        lanes = [round(P[i][1], 2) for i in laps]
        print('%-17s status=%s  dist=%.0f cm  max z=%.1f cm  lap crossings at y=%s (dist %s)' % (
            name, status, dist, P[:, 2].max(), lanes, [i for i in laps]))
        results[name] = P
    np.savez('/home/claude/work/drive_paths.npz', **{k.replace(' ', '_'): v for k, v in results.items()})
