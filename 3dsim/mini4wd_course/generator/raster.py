"""Tiny numpy z-buffer renderer (perspective, lambert) for previews."""
import numpy as np
from PIL import Image

def look_at(eye, target, up=(0, 0, 1)):
    f = np.asarray(target, float) - eye; f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, u, -f]), np.asarray(eye, float)

def render(tris, colors, eye, target, size=(1200, 800), fov=35, light=(0.4, -0.3, 0.85), bg=(245, 245, 242)):
    """tris: (n,3,3) world coords (right-handed, z up); colors: (n,3) 0..1."""
    W, H = size
    R, e = look_at(np.asarray(eye, float), target)
    cam = (tris - e) @ R.T                     # camera looks down -z
    f = 0.5 * H / np.tan(np.radians(fov) / 2)
    z = -cam[:, :, 2]
    keep = (z > 1e-3).all(1)
    cam, z, tris, colors = cam[keep], z[keep], tris[keep], colors[keep]
    sx = W / 2 + f * cam[:, :, 0] / z
    sy = H / 2 - f * cam[:, :, 1] / z
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    view = e - tris.mean(1)
    flip = (n * view).sum(1) < 0
    n[flip] *= -1                                  # two-sided lighting
    L = np.asarray(light, float); L /= np.linalg.norm(L)
    shade = 0.35 + 0.65 * np.clip(n @ L, 0, 1)
    rgb = np.clip(colors * shade[:, None], 0, 1)
    zbuf = np.full((H, W), np.inf); img = np.ones((H, W, 3)) * np.array(bg) / 255.0
    for t in range(len(sx)):
        x, y, zz = sx[t], sy[t], z[t]
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(W - 1, np.ceil(x.max())))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(H - 1, np.ceil(y.max())))
        if x0 > x1 or y0 > y1: continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        d = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(d) < 1e-9: continue
        a = ((y[1] - y[2]) * (X - x[2]) + (x[2] - x[1]) * (Y - y[2])) / d
        b = ((y[2] - y[0]) * (X - x[2]) + (x[0] - x[2]) * (Y - y[2])) / d
        c = 1 - a - b
        m = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
        if not m.any(): continue
        iz = a / zz[0] + b / zz[1] + c / zz[2]
        depth = 1 / iz
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        upd = m & (depth < sub)
        sub[upd] = depth[upd]
        img[y0:y1 + 1, x0:x1 + 1][upd] = rgb[t]
    return Image.fromarray((img * 255).astype(np.uint8))

def polys_to_tris(V, polys):
    out = []
    for p in polys:
        for k in range(1, len(p) - 1):
            out.append([V[p[0]], V[p[k]], V[p[k + 1]]])
    return np.array(out)
