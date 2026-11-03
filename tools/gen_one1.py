#!/usr/bin/env python3
"""Generates the Koenigsegg One:1 mesh (assets/one1.mesh).

The skin is one smooth surface, lofted through the same station tables as the structure in src/one1.cpp.
Parts are cut from it by where each small triangle lies, with shut lines left dark between them. The
edge of each wheel arch is snapped onto a true circle, so the arches are round however fine the grid.

Coordinates: x from the nose rearwards, y to the left, z up, metres. Ground at z = 0.
"""
import math
import struct
import sys

import numpy as np
from scipy.interpolate import PchipInterpolator as Pchip

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent", "sand"]
M = {n: i for i, n in enumerate(MATS)}
parts = []

X = np.array([0.00, 0.12, 0.30, 0.55, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00, 2.30, 2.60, 2.90, 3.20, 3.45, 3.70, 3.95, 4.20, 4.40, 4.50])
ZTOP = np.array([0.42, 0.50, 0.58, 0.64, 0.69, 0.72, 0.75, 0.84, 0.98, 1.10, 1.15, 1.15, 1.12, 1.05, 0.98, 0.92, 0.88, 0.86, 0.84, 0.72])
W = np.array([0.62, 0.78, 0.90, 0.98, 1.015, 1.03, 1.00, 0.96, 0.94, 0.94, 0.96, 0.99, 1.02, 1.03, 1.03, 1.03, 1.02, 0.98, 0.90, 0.72])
ZBELT = np.array([0.34, 0.40, 0.52, 0.68, 0.78, 0.81, 0.77, 0.69, 0.70, 0.74, 0.74, 0.76, 0.80, 0.84, 0.87, 0.88, 0.86, 0.84, 0.82, 0.70])
ZRAIL = np.array([0.40, 0.46, 0.56, 0.68, 0.78, 0.81, 0.79, 0.81, 0.93, 1.05, 1.10, 1.10, 1.06, 0.99, 0.93, 0.89, 0.86, 0.85, 0.83, 0.71])
WRAIL = np.array([0.30, 0.42, 0.55, 0.62, 0.66, 0.67, 0.67, 0.65, 0.60, 0.55, 0.52, 0.52, 0.55, 0.62, 0.72, 0.80, 0.82, 0.80, 0.72, 0.55])
FRONT, REAR, AXLE_Z = 0.98, 3.64, 0.34
ARCH_R = 0.385

ztop, wmax, zbelt, zrail, wrail = [Pchip(X, a) for a in (ZTOP, W, ZBELT, ZRAIL, WRAIL)]


def sill(x):
    return np.full_like(x, 0.19) + 0.02 * np.exp(-((x - 0.0) / 0.25) ** 2) + 0.10 * np.clip((x - 4.0) / 0.5, 0, 1)


# A cross-section goes round from the sill, up the side, over the shoulder and across the roof to the
# centre line. Row positions run t = 0..1; between rows it is a smooth curve.
ROW_T = np.array([0.0, 0.22, 0.46, 0.72, 1.0])


def section(x, t):
    """x and t arrays of equal shape -> y, z on the left half."""
    pts_y = np.stack([wmax(x) * 0.965, wmax(x), wmax(x) * 0.985, wrail(x), np.zeros_like(x)], 0)
    zs = sill(x)
    zb = zbelt(x)
    pts_z = np.stack([zs, 0.5 * (zs + zb), zb, zrail(x), ztop(x)], 0)
    # Catmull-Rom across the rows, in row space.
    u = np.interp(t, ROW_T, np.arange(len(ROW_T)))
    i = np.clip(u.astype(int), 0, len(ROW_T) - 2)
    f = u - i
    i0, i1, i2, i3 = np.clip(i - 1, 0, 4), i, i + 1, np.clip(i + 2, 0, 4)
    rows = np.arange(x.shape[0])

    def cr(p):
        a, b, c, d = [p[k, rows] for k in (i0, i1, i2, i3)]
        return 0.5 * ((2 * b) + (c - a) * f + (2 * a - 5 * b + 4 * c - d) * f ** 2 + (3 * b - a - 3 * c + d) * f ** 3)

    return cr(pts_y), cr(pts_z)


# ---- Skin grid. A nose and a tail rounding in plan view, on top of the tables.
NX, NT = 300, 90
gx = np.linspace(0.0, 4.5, NX)
gt = np.linspace(0.0, 1.0, NT)
XX, TT = np.meshgrid(gx, gt, indexing="ij")
YY, ZZ = section(XX.ravel(), TT.ravel())
YY = YY.reshape(XX.shape)
ZZ = ZZ.reshape(XX.shape)
XX = XX.copy()
cell = float(gx[1] - gx[0])


def inside_poly(px, py, poly):
    inside = np.zeros_like(px, bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        c = ((y0 > py) != (y1 > py)) & (px < (x1 - x0) * (py - y0) / (y1 - y0 + 1e-12) + x0)
        inside ^= c
    return inside


SIDE_WIN = [(1.90, 0.80), (2.06, 1.00), (2.32, 1.06), (2.66, 1.05), (2.84, 0.86), (2.76, 0.79)]
DOOR = [(1.86, 0.30), (2.94, 0.30), (2.98, 0.82), (2.84, 1.07), (2.32, 1.10), (2.04, 1.03), (1.84, 0.80)]
SCOOP = [(3.00, 0.44), (3.32, 0.48), (3.36, 0.74), (3.02, 0.70)]
LAMP = [(0.10, 0.40), (0.60, 0.60), (0.70, 0.69), (0.50, 0.66), (0.14, 0.50)]
LOWER_INTAKE = [(0.00, 0.14), (0.40, 0.14), (0.42, 0.30), (0.00, 0.33)]



def lower_line(x):
    return 0.31 + 0.03 * np.clip((x - 2.9) / 1.0, 0, 1)


def densify(points, closed=True, step=0.004):
    pts = []
    seq = points + [points[0]] if closed else points
    for (x0, z0), (x1, z1) in zip(seq, seq[1:]):
        n = max(int(math.hypot(x1 - x0, z1 - z0) / step), 1)
        for k in range(n):
            t = k / n
            pts.append((x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))
    return np.array(pts)


def poly_sdf(px, pz, poly):
    """Signed distance to a polygon in side view: negative inside."""
    p = np.stack([px, pz], -1)
    d = np.full(px.shape, 1e9)
    n = len(poly)
    for i in range(n):
        a = np.array(poly[i])
        b = np.array(poly[(i + 1) % n])
        ab = b - a
        t = np.clip(((p - a) * ab).sum(-1) / (ab * ab).sum(), 0, 1)
        q = p - (a + t[..., None] * ab)
        d = np.minimum(d, np.hypot(q[..., 0], q[..., 1]))
    return np.where(inside_poly(px, pz, poly), -d, d)


def rbox(x, y, x0, x1, y0, y1, r):
    """Signed distance to a rounded rectangle: negative inside."""
    cx_, cy_ = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    qx = np.abs(x - cx_) - (0.5 * (x1 - x0) - r)
    qy = np.abs(y - cy_) - (0.5 * (y1 - y0) - r)
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - r


# ---- Cut the skin exactly along each part's edge, by splitting the triangles there. A part made of whole
# grid squares can only have stairs for edges, however fine the grid.
class Skin:
    def __init__(self):
        self.P = PP_all.reshape(-1, 3).copy()
        self.N = NN_all.reshape(-1, 3).copy()
        self.T = TT.reshape(-1).copy()
        i = np.arange(NX - 1)
        j = np.arange(NT - 1)
        I, J = np.meshgrid(i, j, indexing="ij")
        a = (I * NT + J).ravel()
        b = ((I + 1) * NT + J).ravel()
        c = ((I + 1) * NT + J + 1).ravel()
        d = (I * NT + J + 1).ravel()
        self.F = np.concatenate([np.stack([a, b, c], -1), np.stack([a, c, d], -1)])
        self.mat = np.full(len(self.F), "pearl", dtype=object)
        self.grp = np.full(len(self.F), "body", dtype=object)

    def field(self, fn):
        return fn(self.P[:, 0], np.abs(self.P[:, 1]), self.P[:, 2], self.T)

    def cut(self, fn):
        """Splits every triangle crossing fn = 0. Afterwards each one lies wholly on one side."""
        f = self.field(fn)
        s = f[self.F] < 0
        cross = s.any(1) & ~s.all(1)
        if not cross.any():
            return
        P, N, T = list(self.P), list(self.N), list(self.T)
        seen = {}

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in seen:
                k = f[a] / (f[a] - f[b])
                nn = self.N[a] + (self.N[b] - self.N[a]) * k
                P.append(self.P[a] + (self.P[b] - self.P[a]) * k)
                N.append(nn / (np.linalg.norm(nn) + 1e-12))
                T.append(self.T[a] + (self.T[b] - self.T[a]) * k)
                seen[key] = len(P) - 1
            return seen[key]

        newF, newM, newG = [], [], []
        for idx in np.nonzero(cross)[0]:
            tri = self.F[idx]
            ins = s[idx]
            # Rotate so that vertex 0 is the one alone on its side.
            k = next(i for i in range(3) if ins[i] != ins[(i + 1) % 3] and ins[i] != ins[(i + 2) % 3])
            a, b, c = tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]
            ab, ca = mid(a, b), mid(c, a)
            newF += [[a, ab, ca], [ab, b, c], [ab, c, ca]]
            newM += [self.mat[idx]] * 3
            newG += [self.grp[idx]] * 3
        keep = ~cross
        self.F = np.concatenate([self.F[keep], np.array(newF)])
        self.mat = np.concatenate([self.mat[keep], np.array(newM, dtype=object)])
        self.grp = np.concatenate([self.grp[keep], np.array(newG, dtype=object)])
        self.P, self.N, self.T = np.array(P), np.array(N), np.array(T)

    def inside(self, fn):
        f = self.field(fn)
        # After a cut each triangle is on one side: its centre tells which, without rounding trouble.
        return f[self.F].mean(1) < 0

    def paint(self, fn, mat):
        self.cut(fn)
        self.mat[self.inside(fn)] = mat

    def remove(self, fn):
        self.cut(fn)
        keep = ~self.inside(fn)
        self.F, self.mat, self.grp = self.F[keep], self.mat[keep], self.grp[keep]


def both(*conds):
    """Inside all of several regions: the largest of their distances."""
    def fn(x, y, z, t):
        out = conds[0](x, y, z, t)
        for c in conds[1:]:
            out = np.maximum(out, c(x, y, z, t))
        return out
    return fn


side_only = lambda x, y, z, t: t - 0.70
top_only = lambda x, y, z, t: 0.70 - t


PP_all = np.stack([XX, YY, ZZ], -1)
_du = np.gradient(PP_all, axis=0)
_dv = np.gradient(PP_all, axis=1)
NN_all = np.cross(_dv, _du)
NN_all /= np.linalg.norm(NN_all, axis=-1, keepdims=True) + 1e-12
if NN_all[NX // 2, NT // 4, 1] < 0:
    NN_all = -NN_all

S = Skin()
S.paint(lambda x, y, z, t: z - lower_line(x), "carbon")
# Rear deck, then the glass and roof, all with rounded corners in plan view.
S.paint(both(top_only, lambda x, y, z, t: rbox(x, y, 3.28, 4.28, -0.1, 0.80, 0.10)), "carbon")
S.paint(both(top_only, lambda x, y, z, t: rbox(x, y, 2.28, 3.02, -0.1, 0.49, 0.08)), "carbon")
S.paint(both(top_only, lambda x, y, z, t: rbox(x, y, 3.02, 3.28, -0.1, 0.40, 0.06)), "glass")
S.paint(both(top_only, lambda x, y, z, t: rbox(x, y, 1.52, 2.28, -0.1, 0.62, 0.10)), "glass")
# Side windows, the door and the intake behind it.
S.paint(both(side_only, lambda x, y, z, t: poly_sdf(x, z, SIDE_WIN)), "glass")
door_fn = both(side_only, lambda x, y, z, t: poly_sdf(x, z, DOOR))
S.cut(door_fn)
S.grp[S.inside(door_fn)] = "door"
S.paint(both(side_only, lambda x, y, z, t: poly_sdf(x, z, SCOOP)), "dark")
# The mouth in the nose.
S.paint(lambda x, y, z, t: rbox(x, y, -0.2, 0.25, -0.1, 0.56, 0.06) if False else np.maximum(rbox(x, y, -0.2, 0.25, -0.1, 0.56, 0.06), z - 0.33), "dark")
# Round wheel arches: a true circle, right through the triangles.
for c0 in (FRONT, REAR):
    S.remove(both(lambda x, y, z, t, c0=c0: np.hypot(x - c0, z - AXLE_Z) - ARCH_R, lambda x, y, z, t: t - 0.5))


def add_skin(side_sign, suffix):
    F = S.F
    for key in sorted({(m, g) for m, g in zip(S.mat, S.grp)}):
        m, g = key
        sel = np.nonzero((S.mat == m) & (S.grp == g))[0]
        tris = F[sel]
        used, inv = np.unique(tris, return_inverse=True)
        v = S.P[used].copy()
        n = S.N[used].copy()
        v[:, 1] *= side_sign
        n[:, 1] *= side_sign
        f = inv.reshape(-1, 3)
        if side_sign < 0:
            f = f[:, [0, 2, 1]]
        group = ("door_l" if side_sign > 0 else "door_r") if g == "door" else "body"
        parts.append(("skin_%s_%s%s" % (m, g, suffix), group, M[m], v, n, f))


add_skin(1, "_l")
add_skin(-1, "_r")


# ---- Clean overlays laid on the surface: everything that needs a crisp edge. A part cut out of the grid
# can only follow the grid's squares; these have their own vertices, along their own outline.
def surface_at(x, t):
    x = np.atleast_1d(np.asarray(x, float))
    t = np.atleast_1d(np.asarray(t, float))
    y, z = section(x, t)
    h = 0.004
    y1, z1 = section(x, np.clip(t + h, 0, 1))
    y0, z0 = section(x, np.clip(t - h, 0, 1))
    yb, zb = section(x + h * 2, t)
    ya, za = section(x - h * 2, t)
    du_ = np.stack([np.full_like(x, 4 * h), yb - ya, zb - za], -1)
    dv_ = np.stack([np.zeros_like(x), y1 - y0, z1 - z0], -1)
    n = np.cross(dv_, du_)
    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-12
    n[n[:, 1] < 0] *= 1.0 if False else 1.0
    return np.stack([x, y, z], -1), n


def t_of_side(x, z):
    """The surface parameter where a side-view point lies on the side of the car."""
    x = np.asarray(x, float)
    z = np.asarray(z, float)
    lo, hi = np.zeros_like(x), np.full_like(x, 0.72)
    for _ in range(26):
        m = 0.5 * (lo + hi)
        _, zm = section(x, m)
        lo = np.where(zm < z, m, lo)
        hi = np.where(zm < z, hi, m)
    return 0.5 * (lo + hi)


OVER = {}


def overlay(mat, group, v, n, f):
    OVER.setdefault((mat, group), []).append((np.asarray(v, float), np.asarray(n, float), np.asarray(f, int)))


def outward(p, n):
    # Normals point away from the car's core.
    core = np.stack([p[:, 0], np.zeros(len(p)), np.full(len(p), 0.45)], -1)
    flip = ((p - core) * n).sum(-1) < 0
    n = n.copy()
    n[flip] *= -1
    return n


def ribbon(points, width, lift, mat, group="body", closed=False):
    """A thin strip along a line given in side view: shut lines, trim, the accent stripe."""
    if len(points) < 2:
        return
    pts = densify(list(map(tuple, points)), closed=closed, step=0.012)
    t = t_of_side(pts[:, 0], pts[:, 1])
    p, n = surface_at(pts[:, 0], t)
    n = outward(p, n)
    d = np.gradient(p, axis=0)
    side = np.cross(n, d)
    side /= np.linalg.norm(side, axis=-1, keepdims=True) + 1e-12
    v = np.concatenate([p + n * lift + side * width * 0.5, p + n * lift - side * width * 0.5])
    nn = np.concatenate([n, n])
    m = len(p)
    f = []
    for i in range(m - 1):
        f += [[i, i + 1, m + i + 1], [i, m + i + 1, m + i]]
    overlay(mat, group, v, nn, f)


def patch(cx0, t0, ax, at, skew, power, lift, mat, group="body", rings=8, seg=56):
    """A rounded shape on the surface: an ellipse-like outline around a centre in surface coordinates."""
    v, n, f = [], [], []
    pc, nc = surface_at(cx0, t0)
    nc = outward(pc, nc)
    v.append(pc[0] + nc[0] * lift)
    n.append(nc[0])
    for r in range(1, rings + 1):
        q = r / rings
        for j in range(seg):
            a = 2 * math.pi * j / seg
            ca, sa = math.cos(a), math.sin(a)
            e = 2.0 / power
            u = abs(ca) ** e * (1 if ca >= 0 else -1)
            w = abs(sa) ** e * (1 if sa >= 0 else -1)
            xx = cx0 + ax * q * u
            tt = t0 + at * q * w + skew * q * u
            p1, n1 = surface_at(xx, tt)
            n1 = outward(p1, n1)
            # A rim that dips back to the surface at the edge, so that it looks set in.
            v.append(p1[0] + n1[0] * (lift * (1.0 - q ** 8) + 0.0005))
            n.append(n1[0])
    for j in range(seg):
        f.append([0, 1 + j, 1 + (j + 1) % seg])
    for r in range(rings - 1):
        for j in range(seg):
            A, B = 1 + r * seg + j, 1 + r * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, C, D], [A, D, B]]
    overlay(mat, group, v, n, f)


# Shut lines of the door, and the frame of its window.
ribbon(DOOR, 0.007, 0.0015, "black", "door", closed=True)
ribbon(SIDE_WIN, 0.020, 0.0025, "black", "door", closed=True)
# The orange accent running from wing to tail, broken by the wheel arches and the intake.
for x0, x1 in ((0.46, 0.56), (1.40, 2.98), (4.08, 4.25)):
    xs = np.arange(x0, x1, 0.02)
    ribbon(np.stack([xs, zbelt(xs) - 0.10], -1), 0.012, 0.002, "accent")
# The edges where the carbon underbody meets the paint.
for x0, x1 in ((0.58, FRONT - ARCH_R - 0.02), (FRONT + ARCH_R + 0.02, REAR - ARCH_R - 0.02), (REAR + ARCH_R + 0.02, 4.42)):
    xs = np.arange(x0, x1, 0.02)
    ribbon(np.stack([xs, lower_line(xs)], -1), 0.006, 0.0015, "black")
# Intake frame, behind the door.
ribbon(SCOOP, 0.012, 0.002, "black", closed=True)
# Black lips around the wheel arches.
for c0 in (FRONT, REAR):
    a = np.radians(np.arange(8, 172, 3.0))
    ribbon(np.stack([c0 + (ARCH_R + 0.004) * np.cos(a), AXLE_Z + (ARCH_R + 0.004) * np.sin(a)], -1), 0.012, 0.002, "black")
# Headlights: long, narrow and swept back along the top of the wing.
patch(0.36, 0.60, 0.25, 0.060, 0.055, 3.4, 0.0015, "dark")
for _x, _t in ((0.26, 0.572), (0.42, 0.610)):
    patch(_x, _t, 0.045, 0.030, 0.0, 2.0, 0.003, "reflector", rings=4, seg=24)
patch(0.36, 0.60, 0.235, 0.055, 0.055, 3.4, 0.005, "lens")
# Bonnet vents, between the raised wings.
patch(1.05, 0.90, 0.13, 0.042, 0.0, 4.0, 0.0015, "dark")
for (m, grp), prims in OVER.items():
    vs, ns, fs, base = [], [], [], 0
    for v, n, f in prims:
        vs.append(v)
        ns.append(n)
        fs.append(f + base)
        base += len(v)
    V, N, FF = np.vstack(vs), np.vstack(ns), np.vstack(fs)
    gl = "door_l" if grp == "door" else "body"
    gr = "door_r" if grp == "door" else "body"
    parts.append(("over_%s_%s_l" % (m, grp), gl, M[m], V, N, FF))
    V2, N2 = V.copy(), N.copy()
    V2[:, 1] *= -1
    N2[:, 1] *= -1
    parts.append(("over_%s_%s_r" % (m, grp), gr, M[m], V2, N2, FF[:, [0, 2, 1]]))


# ---- Small helpers for everything that is not skin.
class Group:
    def __init__(self, group):
        self.group, self.by = group, {}

    def add(self, mat, v, n, f):
        self.by.setdefault(mat, []).append((np.asarray(v, float), np.asarray(n, float), np.asarray(f, int)))

    def finish(self, name):
        for mat, prims in self.by.items():
            vs, ns, fs, base = [], [], [], 0
            for v, n, f in prims:
                vs.append(v)
                ns.append(n)
                fs.append(f + base)
                base += len(v)
            parts.append((name + "_" + mat, self.group, M[mat], np.vstack(vs), np.vstack(ns), np.vstack(fs)))


def box(g, mat, c, s):
    c = np.asarray(c, float)
    h = np.asarray(s, float) * 0.5
    v, n, f = [], [], []
    for ax in range(3):
        for sg in (-1, 1):
            nr = np.zeros(3)
            nr[ax] = sg
            u, w = np.zeros(3), np.zeros(3)
            u[(ax + 1) % 3] = h[(ax + 1) % 3]
            w[(ax + 2) % 3] = h[(ax + 2) % 3]
            cc = c + nr * h
            base = len(v)
            v += [cc - u - w, cc + u - w, cc + u + w, cc - u + w]
            n += [nr] * 4
            f += [[base, base + 1, base + 2], [base, base + 2, base + 3]] if sg > 0 else [[base, base + 2, base + 1], [base, base + 3, base + 2]]
    g.add(mat, v, n, f)


def lathe(g, mat, a, b, prof, seg=32):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax = b - a
    L = np.linalg.norm(ax)
    ax = ax / L
    e1 = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(e1) < 1e-3:
        e1 = np.cross(ax, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    prof = [(0.0, 1e-4)] + list(prof) + [(1.0, 1e-4)]
    v, n, f = [], [], []
    for i, (t, r) in enumerate(prof):
        t0, t1 = prof[max(i - 1, 0)], prof[min(i + 1, len(prof) - 1)]
        slope = -(t1[1] - t0[1]) / max((t1[0] - t0[0]) * L, 1e-6)
        for j in range(seg):
            q = 2 * math.pi * j / seg
            d = e1 * math.cos(q) + e2 * math.sin(q)
            v.append(a + ax * (t * L) + d * r)
            nn = d + ax * slope
            n.append(nn / np.linalg.norm(nn))
    for i in range(len(prof) - 1):
        for j in range(seg):
            A, B = i * seg + j, i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, C, D], [A, D, B]]
    g.add(mat, v, n, f)


def cyl(g, mat, a, b, r, seg=24):
    lathe(g, mat, a, b, [(0.002, r), (0.998, r)], seg)


# ---- Inside and underneath: closes the car against the light, and something to see through the glass.
F = Group("frame")
box(F, "carpet", (2.35, 0.0, 0.26), (1.1, 1.30, 0.02))
box(F, "dark", (1.86, 0.0, 0.50), (0.34, 1.50, 0.26))
box(F, "dark", (2.40, 0.0, 0.40), (0.95, 0.20, 0.20))
for sg in (-1, 1):
    y = sg * 0.36
    box(F, "fabric", (2.46, y, 0.36), (0.50, 0.42, 0.10))
    box(F, "fabric", (2.76, y, 0.62), (0.13, 0.40, 0.56))
box(F, "liner", (2.95, 0.0, 0.62), (0.03, 1.60, 0.80))
box(F, "liner", (1.68, 0.0, 0.45), (0.03, 1.60, 0.60))
for cx0 in (FRONT, REAR):
    for sg in (-1, 1):
        # Half-round arch liners behind the openings.
        v, f = [], []
        th = np.linspace(math.radians(-32), math.radians(212), 40)
        for t in th:
            px, pz = cx0 + (ARCH_R + 0.01) * math.cos(t), AXLE_Z + (ARCH_R + 0.01) * math.sin(t)
            v.append((px, sg * 0.56, pz))
            v.append((px, sg * 1.05, pz))
        for i in range(len(th) - 1):
            a = i * 2
            f += [[a, a + 2, a + 3], [a, a + 3, a + 1]]
        base = len(v)
        v.append((cx0, sg * 0.56, AXLE_Z))
        for t in th:
            v.append((cx0 + (ARCH_R + 0.01) * math.cos(t), sg * 0.56, AXLE_Z + (ARCH_R + 0.01) * math.sin(t)))
        for i in range(len(th) - 1):
            f.append([base, base + 1 + i, base + 2 + i])
        v = np.array(v, float)
        n = np.tile([0.0, 0.0, -1.0], (len(v), 1))
        F.add("liner", v, n, f)
F.finish("inside")


# ---- Everything that closes the shell, so that no gap ever looks into the car: a floor from sill to sill
# with the wheel wells left open, and an end cap on the nose and on the tail.
def closing(xs, yfun, zfun, mat, name):
    v, n, f = [], [], []
    M_ = 12
    for x in xs:
        for j in range(M_ + 1):
            y = (j / M_ * 2 - 1) * float(yfun(x))
            v.append((x, y, float(zfun(x))))
            n.append((0, 0, -1))
    R_ = M_ + 1
    for i in range(len(xs) - 1):
        for j in range(M_):
            a, b = i * R_ + j, (i + 1) * R_ + j
            c, d = b + 1, a + 1
            xc = 0.5 * (xs[i] + xs[i + 1])
            yc = (j + 0.5) / M_ * 2 - 1
            yy = abs(yc * float(yfun(xc)))
            if yy > 0.56 and (abs(xc - FRONT) < ARCH_R + 0.012 or abs(xc - REAR) < ARCH_R + 0.012):
                continue  # the wheel well stays open downwards
            f += [[a, b, c], [a, c, d]]
    g_ = Group("frame")
    g_.add(mat, np.array(v, float), np.array(n, float), np.array(f))
    g_.finish(name)


closing(np.arange(0.0, 4.5001, 0.02), lambda x: wmax(np.array([x]))[0] * 0.965, lambda x: sill(np.array([x]))[0], "liner", "floor")


def end_cap(x, name):
    # A fan from the middle of the section to its outline, all the way round.
    ts = np.linspace(0.0, 1.0, 48)
    y, z = section(np.full_like(ts, x), ts)
    c = (x, 0.0, float(z.mean()))
    nrm = (-1.0, 0.0, 0.0) if x < 2 else (1.0, 0.0, 0.0)
    v = [c]
    n = [nrm]
    f = []
    outline = [(y[i], z[i]) for i in range(len(ts))] + [(-y[i], z[i]) for i in range(len(ts) - 1, -1, -1)]
    for yy, zz in outline:
        v.append((x, yy, zz))
        n.append(nrm)
    for i in range(1, len(outline)):
        f.append([0, i, i + 1])
    f.append([0, len(outline), 1])
    g_ = Group("frame")
    g_.add("carbon", np.array(v, float), np.array(n, float), np.array(f))
    g_.finish(name)


end_cap(0.0, "cap_nose")
end_cap(4.5, "cap_tail")


# Steering wheel: a flat-bottomed feel is left out, it is one smooth rim with a centre.
def torus(c, axis, R, r, major=96, minor=18):
    a = axis / np.linalg.norm(axis)
    e1 = np.cross(a, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    v, n, f = [], [], []
    for i in range(major):
        t = 2 * math.pi * i / major
        d = e1 * math.cos(t) + e2 * math.sin(t)
        for j in range(minor):
            p = 2 * math.pi * j / minor
            nn = d * math.cos(p) + a * math.sin(p)
            v.append(c + d * (R + r * math.cos(p)) + a * (r * math.sin(p)))
            n.append(nn)
    for i in range(major):
        for j in range(minor):
            A, B = i * minor + j, i * minor + (j + 1) % minor
            C, D = ((i + 1) % major) * minor + j, ((i + 1) % major) * minor + (j + 1) % minor
            f += [[A, C, D], [A, D, B]]
    return v, n, f


sw = np.array([2.10, 0.36, 0.63])
axis = np.array([-math.cos(math.radians(24)), 0, -math.sin(math.radians(24))])
R = Group("frame")
R.add("black", *torus(sw, axis, 0.165, 0.014))
R.finish("steerwheel_rim")
H = Group("frame")
cyl(H, "carbon", sw - axis * 0.012, sw + axis * 0.03, 0.07, 36)
e1 = np.cross(axis, [0.0, 1.0, 0.0])
e1 /= np.linalg.norm(e1)
for ang in (0.0, 120.0, 240.0):
    d = e1 * math.cos(math.radians(ang)) + np.cross(axis, e1) * math.sin(math.radians(ang))
    cyl(H, "carbon", sw + d * 0.06, sw + d * 0.16, 0.012, 12)
H.finish("steerwheel_hub")

# ---- Mirrors, the rear lights and the exhaust on the body.
B = Group("body")
for sg in (-1, 1):
    lathe(B, "carbon", (1.70, sg * 0.99, 0.87), (1.86, sg * 1.03, 0.88), [(0.05, 0.05), (0.5, 0.075), (0.95, 0.05)], 20)
    cyl(B, "carbon", (1.80, sg * 0.83, 0.80), (1.79, sg * 1.00, 0.87), 0.012, 10)
    # Rear lights: round, stacked, the car's signature from behind.
    for z, r in ((0.70, 0.075), (0.56, 0.058)):
        y = sg * 0.56
        cyl(B, "tail", (4.48, y, z), (4.522, y, z), r, 32)
        cyl(B, "lens", (4.522, y, z), (4.524, y, z), r * 0.5, 24)
    # Vents on the tail.
    box(B, "dark", (4.522, sg * 0.28, 0.42), (0.012, 0.40, 0.14))
# Tail panel and diffuser, in carbon.
box(B, "carbon", (4.49, 0.0, 0.33), (0.06, 1.36, 0.24))
box(B, "carbon", (4.40, 0.0, 0.205), (0.34, 1.34, 0.03))
for y in (-0.42, -0.21, 0.0, 0.21, 0.42):
    box(B, "dark", (4.42, y, 0.26), (0.30, 0.015, 0.11))
# The central exhaust, up high between the lights.
cyl(B, "alu", (4.46, 0.0, 0.62), (4.54, 0.0, 0.62), 0.075, 32)
cyl(B, "black", (4.54, 0.0, 0.62), (4.543, 0.0, 0.62), 0.060, 28)
# Front splitter.
# A blade following the nose's plan outline, instead of a plank across it.
_sx = np.linspace(0.015, 0.62, 18)
_sy = wmax(_sx) * 0.90
_v, _n, _f = [], [], []
for _z, _nz in ((0.155, 1.0), (0.135, -1.0)):
    for _x, _y in zip(_sx, _sy):
        _v += [(_x, _y, _z), (_x, -_y, _z)]
        _n += [(0, 0, _nz), (0, 0, _nz)]
for _i in range(len(_sx) - 1):
    _a = _i * 2
    _f += [[_a, _a + 2, _a + 3], [_a, _a + 3, _a + 1]]
    _a += len(_sx) * 2
    _f += [[_a, _a + 3, _a + 2], [_a, _a + 1, _a + 3]]
B.add("carbon", _v, _n, _f)
for sg in (-1, 1):
    # Canards, ahead of the front arches.
    box(B, "carbon", (0.40, sg * 0.90, 0.30), (0.16, 0.09, 0.010))
# Roof scoop over the engine.
box(B, "carbon", (3.08, 0.0, 1.075), (0.30, 0.34, 0.05))
box(B, "black", (2.94, 0.0, 1.07), (0.02, 0.30, 0.035))
B.finish("trim")

# ---- The wing: a two-element blade on swan-neck pylons, with end plates.
WNG = Group("wing")
for x0, x1, z, thick in ((4.12, 4.33, 1.06, 0.024), (4.35, 4.48, 1.09, 0.016)):
    box(WNG, "carbon", (0.5 * (x0 + x1), 0.0, z), (x1 - x0, 1.84, thick))
for sg in (-1, 1):
    box(WNG, "carbon", (4.30, sg * 0.93, 1.06), (0.46, 0.014, 0.20))
    box(WNG, "accent", (4.32, sg * 0.945, 1.12), (0.40, 0.004, 0.025))
    box(WNG, "carbon", (4.24, sg * 0.42, 0.955), (0.16, 0.024, 0.21))
WNG.finish("wing")

# ---- Drivetrain, for the X-ray view.
E = Group("engine")
box(E, "alu", (3.20, 0.0, 0.40), (0.52, 0.30, 0.34))
for sg in (-1, 1):
    box(E, "dark", (3.20, sg * 0.22, 0.56), (0.52, 0.18, 0.18))
    cyl(E, "rust", (3.46, sg * 0.28, 0.40), (3.62, sg * 0.42, 0.56), 0.05, 14)
    lathe(E, "metal", (3.50, sg * 0.32, 0.60), (3.58, sg * 0.32, 0.60), [(0.1, 0.09), (0.9, 0.09)], 18)
E.finish("engine")
G = Group("gearbox")
box(G, "alu", (3.74, 0.0, 0.34), (0.46, 0.34, 0.26))
G.finish("gearbox")
RD = Group("radiator")
box(RD, "dark", (0.47, 0.0, 0.36), (0.08, 0.96, 0.22))
RD.finish("radiator")
TK = Group("tank")
box(TK, "dark", (2.77, 0.0, 0.40), (0.20, 1.06, 0.40))
TK.finish("tank")

# ---- Wheels, as for the other car: local, axle along y, outboard +y. Carbon five-spokes.
WH = Group("wheel")
tyre = [(0.0, 0.250), (0.04, 0.268), (0.10, 0.312), (0.20, 0.331), (0.35, 0.338), (0.5, 0.340), (0.65, 0.338), (0.8, 0.331), (0.90, 0.312), (0.96, 0.268), (1.0, 0.250)]
lathe(WH, "rubber", (0, -0.1325, 0), (0, 0.1325, 0), tyre, 64)
for mat_, g_ in ((M["rubber"], 0),):
    pass
WH.finish("wheel_tyre")
parts[-1] = ("wheel_tyre", "wheel", M["rubber"], parts[-1][3], parts[-1][4], parts[-1][5])
BR = Group("wheel")
lathe(BR, "dark", (0, -0.125, 0), (0, 0.125, 0), [(0.02, 0.252), (0.10, 0.235), (0.80, 0.225), (0.98, 0.252)], 56)
BR.finish("wheel_barrel")
FC = Group("wheel")
for k in range(10):
    q = 2 * math.pi * k / 10
    offs = 0.06 * (1 if k % 2 else -1)
    r0 = np.array([math.cos(q) * 0.05, 0.095, math.sin(q) * 0.05])
    r1 = np.array([math.cos(q + offs * 5) * 0.242, 0.122, math.sin(q + offs * 5) * 0.242])
    cyl(FC, "carbon", r0, r1, 0.017, 8)
cyl(FC, "carbon", (0, 0.07, 0), (0, 0.112, 0), 0.062, 28)
cyl(FC, "accent", (0, 0.112, 0), (0, 0.116, 0), 0.028, 18)
lathe(FC, "alloy", (0, 0.115, 0), (0, 0.128, 0), [(0.05, 0.252), (0.95, 0.258)], 56)
FC.finish("wheel_face")
HB = Group("wheel")
cyl(HB, "metal", (0, -0.08, 0), (0, 0.10, 0), 0.075, 20)
HB.finish("wheel_hub")
DS = Group("wheel")
lathe(DS, "disc", (0, -0.02, 0), (0, 0.02, 0), [(0.02, 0.105), (0.08, 0.195), (0.92, 0.195), (0.98, 0.105)], 44)
DS.finish("wheel_disc")
K = Group("wheel_static")
box(K, "accent", (0.12, 0.0, 0.135), (0.13, 0.095, 0.11))
K.finish("caliper")


def write(path):
    with open(path, "wb") as fp:
        fp.write(b"XMSH")
        fp.write(struct.pack("<I", len(parts)))
        for name, group, mat, v, n, f in parts:
            fp.write(name.encode()[:39].ljust(40, b"\0"))
            fp.write(group.encode()[:23].ljust(24, b"\0"))
            fp.write(struct.pack("<III", mat, len(v), len(f)))
            fp.write(np.asarray(v, np.float32).tobytes())
            fp.write(np.asarray(n, np.float32).tobytes())
            fp.write(np.asarray(f, np.uint32).tobytes())
    print("wrote %s: %d parts, %d vertices, %d triangles" % (path, len(parts), sum(len(p[3]) for p in parts), sum(len(p[5]) for p in parts)))


if __name__ == "__main__":
    write(sys.argv[1] if len(sys.argv) > 1 else "assets/one1.mesh")
