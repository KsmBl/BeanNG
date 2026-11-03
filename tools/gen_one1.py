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
        "olive", "olive2", "track", "pearl", "carbon", "accent"]
M = {n: i for i, n in enumerate(MATS)}
parts = []

X = np.array([0.00, 0.12, 0.30, 0.55, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00, 2.30, 2.60, 2.90, 3.20, 3.45, 3.70, 3.95, 4.20, 4.40, 4.50])
ZTOP = np.array([0.42, 0.50, 0.58, 0.63, 0.66, 0.69, 0.74, 0.84, 0.98, 1.10, 1.15, 1.15, 1.12, 1.05, 0.98, 0.92, 0.88, 0.86, 0.84, 0.72])
W = np.array([0.62, 0.78, 0.90, 0.98, 1.015, 1.03, 1.00, 0.96, 0.94, 0.94, 0.96, 0.99, 1.02, 1.03, 1.03, 1.03, 1.02, 0.98, 0.90, 0.72])
ZBELT = np.array([0.34, 0.40, 0.50, 0.60, 0.66, 0.68, 0.66, 0.66, 0.70, 0.74, 0.74, 0.76, 0.80, 0.84, 0.87, 0.88, 0.86, 0.84, 0.82, 0.70])
ZRAIL = np.array([0.40, 0.46, 0.54, 0.60, 0.64, 0.68, 0.72, 0.80, 0.93, 1.05, 1.10, 1.10, 1.06, 0.99, 0.93, 0.89, 0.86, 0.85, 0.83, 0.71])
WRAIL = np.array([0.30, 0.42, 0.55, 0.66, 0.72, 0.74, 0.72, 0.66, 0.60, 0.55, 0.52, 0.52, 0.55, 0.62, 0.72, 0.80, 0.82, 0.80, 0.72, 0.55])
FRONT, REAR, AXLE_Z = 0.98, 3.64, 0.34
ARCH_R = 0.385

ztop, wmax, zbelt, zrail, wrail = [Pchip(X, a) for a in (ZTOP, W, ZBELT, ZRAIL, WRAIL)]


def sill(x):
    return np.full_like(x, 0.15) + 0.02 * np.exp(-((x - 0.0) / 0.25) ** 2) + 0.14 * np.clip((x - 4.0) / 0.5, 0, 1)


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
NX, NT = 420, 120
gx = np.linspace(0.0, 4.5, NX)
gt = np.linspace(0.0, 1.0, NT)
XX, TT = np.meshgrid(gx, gt, indexing="ij")
YY, ZZ = section(XX.ravel(), TT.ravel())
YY = YY.reshape(XX.shape)
ZZ = ZZ.reshape(XX.shape)
XX = XX.copy()
# Arches: the grid points nearest to the circle go exactly onto it.
cell = float(gx[1] - gx[0])
for cx in (FRONT, REAR):
    dx = XX - cx
    dz = ZZ - AXLE_Z
    r = np.hypot(dx, dz)
    near = np.abs(r - ARCH_R) < cell * 1.3
    near &= TT < 0.5
    k = ARCH_R / np.maximum(r, 1e-6)
    XX[near] = cx + dx[near] * k[near]
    ZZ[near] = AXLE_Z + dz[near] * k[near]



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
LAMP = [(0.10, 0.40), (0.62, 0.59), (0.70, 0.64), (0.50, 0.64), (0.14, 0.50)]
LOWER_INTAKE = [(0.00, 0.14), (0.40, 0.14), (0.42, 0.30), (0.00, 0.33)]



def lower_line(x):
    return 0.28 + 0.03 * np.clip((x - 2.9) / 1.0, 0, 1)


def densify(points, closed=True, step=0.004):
    pts = []
    seq = points + [points[0]] if closed else points
    for (x0, z0), (x1, z1) in zip(seq, seq[1:]):
        n = max(int(math.hypot(x1 - x0, z1 - z0) / step), 1)
        for k in range(n):
            t = k / n
            pts.append((x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))
    return np.array(pts)


def snap_side(curve, mask):
    """Grid points close to a curve in side view go exactly onto it: no stairs along part edges."""
    from scipy.spatial import cKDTree
    tree = cKDTree(curve)
    q = np.stack([XX[mask], ZZ[mask]], -1)
    d, i = tree.query(q, distance_upper_bound=cell * 0.75)
    ok = np.isfinite(d)
    xs, zs = XX[mask], ZZ[mask]
    xs[ok] = curve[i[ok], 0]
    zs[ok] = curve[i[ok], 1]
    XX[mask] = xs
    ZZ[mask] = zs


side_mask = TT < 0.70
for poly in (SIDE_WIN, DOOR, SCOOP):
    snap_side(densify(poly), side_mask)
_lx = np.arange(0.0, 4.5, 0.004)
snap_side(np.stack([_lx, lower_line(_lx)], -1), side_mask & (np.hypot(XX - FRONT, ZZ - AXLE_Z) > ARCH_R + cell) & (np.hypot(XX - REAR, ZZ - AXLE_Z) > ARCH_R + cell))
# The upper edge of the headlights, on the part of the surface facing up and out.
snap_side(densify(LAMP), (TT < 0.92) & (np.abs(YY) > 0.42))


def classify(cx, cy, cz, ct):
    """Part of every triangle, from where its centre lies. Returns an array of part keys."""
    out = np.full(cx.shape, "pearl", dtype=object)
    group = np.full(cx.shape, "body", dtype=object)
    side = ct < 0.72
    # The lower body is exposed carbon, with the line of the sill rising towards the tail.
    low = cz < lower_line(cx)
    out[low] = "carbon"
    # An accent line along the shoulder.
    accent = side & (np.abs(cz - (zbelt(cx) - 0.11)) < 0.009) & (cx > 0.5) & (cx < 4.2)
    out[accent] = "accent"
    # Rear deck, in carbon, with the engine vents.
    deck = (ct > 0.72) & (cx > 3.05) & (cx < 4.25)
    out[deck] = "carbon"
    # Windows.
    d_side = side & inside_poly(cx, cz, SIDE_WIN)
    out[d_side] = "glass"
    screen = (ct > 0.72) & (cx > 1.50) & (cx < 2.30) & (np.abs(cy) < 0.62)
    out[screen] = "glass"
    # The targa roof, in carbon, with a small rear window behind it.
    roof = (ct > 0.72) & (cx >= 2.30) & (cx < 2.95) & (np.abs(cy) < 0.50)
    out[roof] = "carbon"
    rear_glass = (ct > 0.72) & (cx >= 2.95) & (cx < 3.28) & (np.abs(cy) < 0.40)
    out[rear_glass] = "glass"
    # Door, with its shut line.
    in_door = side & inside_poly(cx, cz, DOOR)
    group[in_door] = "door"
    # Shut lines are dark gaps. This grid is fine enough to show them as thin ones.
    eps = 0.008
    shut = side & ~in_door & (inside_poly(cx - eps, cz, DOOR) | inside_poly(cx + eps, cz, DOOR) | inside_poly(cx, cz - eps, DOOR) | inside_poly(cx, cz + eps, DOOR))
    out[shut] = "black"
    # Side intake behind the door.
    out[side & inside_poly(cx, cz, SCOOP)] = "dark"
    # Headlights, sweeping back along the wing.
    lamp = (cz > 0.36) & inside_poly(cx, cz, LAMP) & (np.abs(cy) > 0.44)
    out[lamp] = "lens"
    # The hood's two vents, and the shut line of the front clam.
    vents = (ct > 0.72) & (cx > 0.90) & (cx < 1.22) & (np.abs(cy) > 0.16) & (np.abs(cy) < 0.40)
    out[vents] = "dark"
    out[(cx > 1.40) & (cx < 1.415) & (ct > 0.3)] = "black"
    # The nose's lower intake.
    out[(cz < 0.33) & (cx < 0.26) & (np.abs(cy) < 0.56)] = "dark"
    return out, group


cx = 0.25 * (XX[:-1, :-1] + XX[1:, :-1] + XX[:-1, 1:] + XX[1:, 1:]).ravel()
cy = 0.25 * (YY[:-1, :-1] + YY[1:, :-1] + YY[:-1, 1:] + YY[1:, 1:]).ravel()
cz = 0.25 * (ZZ[:-1, :-1] + ZZ[1:, :-1] + ZZ[:-1, 1:] + ZZ[1:, 1:]).ravel()
ct = 0.25 * (TT[:-1, :-1] + TT[1:, :-1] + TT[:-1, 1:] + TT[1:, 1:]).ravel()
mat_of, grp_of = classify(cx, cy, cz, ct)
arch = np.zeros_like(cx, bool)
for c0 in (FRONT, REAR):
    arch |= np.hypot(cx - c0, cz - AXLE_Z) < ARCH_R
arch &= ct < 0.5

# Smooth normals from the whole surface, shared by every part cut from it.
du = np.gradient(np.stack([XX, YY, ZZ], -1), axis=0)
dv = np.gradient(np.stack([XX, YY, ZZ], -1), axis=1)
NN = np.cross(dv, du)
NN /= np.linalg.norm(NN, axis=-1, keepdims=True) + 1e-12
if NN[NX // 2, NT // 4, 1] < 0:
    NN = -NN
PP = np.stack([XX, YY, ZZ], -1)


def add_skin(side_sign, name_suffix, door_group):
    keys = {}
    for k, (m, g) in enumerate(zip(mat_of, grp_of)):
        if arch[k]:
            continue
        keys.setdefault((m, g), []).append(k)
    for (m, g), cells in keys.items():
        cells = np.array(cells)
        i = cells // (NT - 1)
        j = cells % (NT - 1)
        idx = {}
        v, n, f = [], [], []

        def vert(a, b):
            key = (a, b)
            if key not in idx:
                idx[key] = len(v)
                p = PP[a, b].copy()
                q = NN[a, b].copy()
                p[1] *= side_sign
                q[1] *= side_sign
                v.append(p)
                n.append(q)
            return idx[key]

        for a, b in zip(i, j):
            A, B, C, D = vert(a, b), vert(a + 1, b), vert(a + 1, b + 1), vert(a, b + 1)
            f.append([A, B, C])
            f.append([A, C, D])
        group = door_group if g == "door" else "body"
        off = 0.0
        parts.append(("skin_%s_%s%s" % (m, g, name_suffix), group, M[m], np.array(v), np.array(n), np.array(f)))


add_skin(1, "_l", "door_l")
add_skin(-1, "_r", "door_r")


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
box(F, "carpet", (2.35, 0.0, 0.30), (1.1, 1.30, 0.02))
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
        th = np.linspace(math.radians(-6), math.radians(186), 28)
        for t in th:
            px, pz = cx0 + (ARCH_R + 0.01) * math.cos(t), AXLE_Z + (ARCH_R + 0.01) * math.sin(t)
            v.append((px, sg * 0.56, pz))
            v.append((px, sg * 1.0, pz))
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
# The underbody and the panels that close both ends.
box(F, "liner", (2.25, 0.0, 0.135), (4.1, 1.5, 0.012))
F.finish("inside")


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
    lathe(B, "carbon", (1.74, sg * 0.98, 0.86), (1.90, sg * 1.02, 0.87), [(0.05, 0.05), (0.5, 0.075), (0.95, 0.05)], 20)
    cyl(B, "carbon", (1.86, sg * 0.84, 0.84), (1.84, sg * 0.98, 0.86), 0.012, 10)
    # Rear lights: round, stacked, the car's signature from behind.
    for z, r in ((0.70, 0.075), (0.56, 0.058)):
        y = sg * 0.56
        cyl(B, "tail", (4.47, y, z), (4.50, y, z), r, 28)
    # Vents on the tail.
    box(B, "dark", (4.50, sg * 0.28, 0.42), (0.02, 0.40, 0.18))
# Tail panel and diffuser, in carbon.
box(B, "carbon", (4.48, 0.0, 0.34), (0.05, 1.36, 0.30))
box(B, "carbon", (4.40, 0.0, 0.205), (0.34, 1.34, 0.03))
for y in (-0.42, -0.21, 0.0, 0.21, 0.42):
    box(B, "dark", (4.42, y, 0.26), (0.30, 0.015, 0.11))
# The central exhaust, up high between the lights.
cyl(B, "alu", (4.44, 0.0, 0.64), (4.51, 0.0, 0.64), 0.075, 28)
cyl(B, "black", (4.51, 0.0, 0.64), (4.515, 0.0, 0.64), 0.058, 24)
# Front splitter.
box(B, "carbon", (0.20, 0.0, 0.125), (0.42, 1.30, 0.02))
for sg in (-1, 1):
    # Canards, ahead of the front arches.
    box(B, "carbon", (0.42, sg * 0.97, 0.30), (0.20, 0.08, 0.012))
    # Headlight projectors under the lens.
    for x in (0.34, 0.50):
        lathe(B, "reflector", (x, sg * (0.60 + x * 0.25), 0.50 + (x - 0.34) * 0.25), (x + 0.05, sg * (0.60 + x * 0.25), 0.50 + (x - 0.34) * 0.25), [(0.1, 0.03), (0.9, 0.03)], 14)
# Roof scoop over the engine.
box(B, "carbon", (3.08, 0.0, 1.075), (0.30, 0.34, 0.05))
box(B, "black", (2.94, 0.0, 1.07), (0.02, 0.30, 0.035))
B.finish("trim")

# ---- The wing: a two-element blade on swan-neck pylons, with end plates.
WNG = Group("wing")
for x0, x1, z, thick in ((4.12, 4.34, 1.13, 0.022), (4.34, 4.48, 1.17, 0.015)):
    box(WNG, "carbon", (0.5 * (x0 + x1), 0.0, z), (x1 - x0, 1.86, thick))
for sg in (-1, 1):
    box(WNG, "carbon", (4.30, sg * 0.93, 1.13), (0.46, 0.012, 0.24))
    box(WNG, "accent", (4.32, sg * 0.935, 1.20), (0.40, 0.006, 0.03))
    box(WNG, "carbon", (4.26, sg * 0.42, 1.02), (0.12, 0.02, 0.20))
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
