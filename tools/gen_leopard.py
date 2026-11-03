#!/usr/bin/env python3
"""Generates the Leopard 1 mesh (assets/leopard.mesh).

Coordinates as for the car: x from the hull's nose rearwards, y to the left, z up, ground at z = 0.
Hull, turret and gun are separate groups. They follow their own nodes in the game: the turret turns and
the gun elevates. Road wheels, sprockets and track links are drawn by the game between their nodes.
"""
import math
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent"]
M = {n: i for i, n in enumerate(MATS)}
parts = []


class Group:
    def __init__(self, group):
        self.group = group
        self.by = {}

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


def solid(g, mat, pts):
    """A convex plated solid with crisp edges: one set of vertices per face."""
    pts = np.asarray(pts, float)
    hull = ConvexHull(pts)
    # Merge the triangles of each flat face by normal, so that they shade as one plate.
    c = pts.mean(0)
    v, n, f = [], [], []
    for simplex, eq in zip(hull.simplices, hull.equations):
        a, b, d = pts[simplex]
        nn = eq[:3]
        if np.dot(np.cross(b - a, d - a), nn) < 0:
            b, d = d, b
        base = len(v)
        v += [a, b, d]
        n += [nn, nn, nn]
        f.append([base, base + 1, base + 2])
    g.add(mat, v, n, f)


def box(g, mat, c, s):
    c = np.asarray(c, float)
    s = np.asarray(s, float) * 0.5
    solid(g, mat, [c + s * [i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])


def lathe(g, mat, a, b, prof, seg=28):
    """prof: (t along a..b, radius). Smooth, with flat end caps."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax = b - a
    L = np.linalg.norm(ax)
    ax = ax / L
    e1 = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(e1) < 1e-3:
        e1 = np.cross(ax, [0.0, 1.0, 0.0])
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    prof = [(0.0, 1e-4)] + list(prof) + [(1.0, 1e-4)]
    v, n, f = [], [], []
    for i, (t, r) in enumerate(prof):
        t0 = prof[max(i - 1, 0)]
        t1 = prof[min(i + 1, len(prof) - 1)]
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


def cyl(g, mat, a, b, r, seg=20):
    lathe(g, mat, a, b, [(0.002, r), (0.998, r)], seg)


def loft(g, mat, sections, seg=64):
    """Smooth closed body through horizontal outlines: fn(angle) -> (x, y), at height z."""
    v = []
    rings = len(sections)
    for z, fn in sections:
        for j in range(seg):
            a = 2 * math.pi * j / seg
            x, y = fn(a)
            v.append((x, y, z))
    v = np.array(v, float)
    f = []
    for i in range(rings - 1):
        for j in range(seg):
            A, B = i * seg + j, i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, B, D], [A, D, C]]
    # Caps.
    base = len(v)
    tops = [v[:seg].mean(0), v[-seg:].mean(0)]
    v = np.vstack([v, tops])
    for j in range(seg):
        f.append([base, (j + 1) % seg, j])
        f.append([base + 1, (rings - 1) * seg + j, (rings - 1) * seg + (j + 1) % seg])
    f = np.array(f)
    n = np.zeros_like(v)
    fn_ = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn_)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    g.add(mat, v, n, f)


# ---- Hull.
H = Group("hull")
Ytop, Yspon, Ybot = 1.56, 1.60, 1.00
S = [(0.00, 0.72, 0.76, 0.82), (0.62, 0.46, 0.84, 1.08), (1.70, 0.46, 0.86, 1.30), (5.40, 0.46, 0.86, 1.32), (7.02, 0.68, 0.88, 1.24)]
for (x0, zb0, zs0, zt0), (x1, zb1, zs1, zt1) in zip(S, S[1:]):
    pts = []
    for x, zb, zs, zt in ((x0, zb0, zs0, zt0), (x1, zb1, zs1, zt1)):
        for sg in (-1, 1):
            pts += [(x, sg * Ybot, zb), (x, sg * Yspon, zs), (x, sg * Ytop, zt)]
    solid(H, "olive", pts)
# Long side skirts, the look of the later A1A1, hiding the upper half of the wheels.
for sg in (-1, 1):
    pts = []
    for x, zt, zb in ((0.25, 0.90, 0.72), (0.95, 0.96, 0.58), (6.35, 0.96, 0.58), (6.85, 0.92, 0.70)):
        for y in (1.635, 1.66):
            pts += [(x, sg * y, zt), (x, sg * y, zb)]
    solid(H, "olive2", pts)
    # Track guards running along the top of the sponsons.
    box(H, "olive2", (3.55, sg * 1.45, 1.015), (6.9, 0.44, 0.018))
    # Stowage boxes along the guards.
    for x, l in ((1.2, 0.8), (2.15, 0.9), (4.85, 1.1), (6.05, 0.8)):
        box(H, "olive2", (x, sg * 1.45, 1.13), (l, 0.34, 0.24))
    # Headlight in its guard.
    cyl(H, "dark", (0.42, sg * 1.28, 1.11), (0.28, sg * 1.28, 1.13), 0.085)
    cyl(H, "lens", (0.28, sg * 1.28, 1.13), (0.27, sg * 1.28, 1.13), 0.07)
    for k in (-1, 1):
        box(H, "metal", (0.32, sg * 1.28 + k * 0.1, 1.13), (0.2, 0.012, 0.2))
    # Exhaust grilles on the rear plate.
    box(H, "dark", (7.03, sg * 0.95, 1.0), (0.03, 0.6, 0.36))
    # Tow eyes.
    cyl(H, "metal", (-0.06, sg * 0.75, 0.76), (0.02, sg * 0.75, 0.76), 0.05, 14)
# Engine deck: cooling grilles and access plates.
box(H, "dark", (5.55, 0.62, 1.325), (1.7, 0.85, 0.014))
box(H, "dark", (5.55, -0.62, 1.325), (1.7, 0.85, 0.014))
for x in np.arange(4.8, 6.4, 0.13):
    box(H, "olive2", (x, 0.62, 1.335), (0.035, 0.84, 0.012))
    box(H, "olive2", (x, -0.62, 1.335), (0.035, 0.84, 0.012))
box(H, "olive2", (4.15, 0.0, 1.335), (0.5, 2.2, 0.02))
# Driver's hatch and periscopes on the glacis.
cyl(H, "olive2", (1.32, 0.48, 1.20), (1.32, 0.48, 1.27), 0.30, 28)
for k in (-1, 0, 1):
    box(H, "glass", (1.05, 0.48 + k * 0.17, 1.17), (0.03, 0.13, 0.04))
# Spare track links on the glacis, a small touch from the real thing.
for k in range(4):
    box(H, "track", (0.34, -0.55 + k * 0.30, 0.97), (0.34, 0.26, 0.02))
H.finish("hull")

# ---- Turret. Local frame of the hull at yaw zero: ring centre at design (2.90, 0, 1.34).
T = Group("turret")
C = np.array([2.90, 0.0, 1.34])


def turret_outline(scale_x, scale_y, bustle):
    def fn(a):
        # a measured with 0 towards the front of the tank; a pointed cast nose and a long bustle.
        c, s = math.cos(a), math.sin(a)
        front = c > 0
        rx = (1.30 if front else 1.30 + bustle) * scale_x
        ry = 1.42 * scale_y
        e = 2.0 / 2.35
        x = rx * abs(c) ** e * (1 if c >= 0 else -1)
        y = ry * abs(s) ** e * (1 if s >= 0 else -1)
        return (C[0] - x, y)
    return fn


sections = [
    (C[2] + 0.04, turret_outline(0.98, 0.98, 0.1)),
    (C[2] + 0.14, turret_outline(1.02, 1.02, 0.28)),
    (C[2] + 0.32, turret_outline(1.00, 1.00, 0.44)),
    (C[2] + 0.54, turret_outline(0.93, 0.92, 0.44)),
    (C[2] + 0.72, turret_outline(0.81, 0.80, 0.38)),
    (C[2] + 0.84, turret_outline(0.68, 0.68, 0.32)),
    (C[2] + 0.90, turret_outline(0.56, 0.56, 0.26)),
]
loft(T, "olive", sections)
# Optical rangefinder heads, sticking out of both turret sides: the mark of a Leopard 1.
for sg in (-1, 1):
    cyl(T, "olive2", (C[0] + 0.05, sg * 1.05, C[2] + 0.56), (C[0] + 0.05, sg * 1.44, C[2] + 0.53), 0.09, 20)
    cyl(T, "dark", (C[0] + 0.05, sg * 1.44, C[2] + 0.53), (C[0] + 0.05, sg * 1.47, C[2] + 0.53), 0.065, 20)
    # Smoke dischargers, in fours.
    for k in range(4):
        y = sg * (0.95 + 0.09 * k)
        cyl(T, "dark", (C[0] - 0.12, y, C[2] + 0.72), (C[0] - 0.42, y + sg * 0.05, C[2] + 0.80), 0.042, 12)
# Commander's cupola with periscopes, loader's hatch and his MG3.
cc = C + [0.45, -0.46, 0.88]
lathe(T, "olive2", cc, cc + [0, 0, 0.26], [(0.02, 0.36), (0.55, 0.33), (0.98, 0.30)], 36)
for k in range(8):
    q = 2 * math.pi * k / 8
    box(T, "glass", cc + [0.33 * math.cos(q), 0.33 * math.sin(q), 0.14], (0.07, 0.07, 0.04))
lc = C + [0.45, 0.48, 0.88]
cyl(T, "olive2", lc, lc + [0, 0, 0.09], 0.34, 32)
cyl(T, "dark", lc + [-0.25, 0.0, 0.22], lc + [-1.02, 0.0, 0.24], 0.013, 10)
cyl(T, "dark", lc + [-0.25, 0.0, 0.09], lc + [-0.25, 0.0, 0.22], 0.016, 10)
box(T, "dark", lc + [-0.12, 0.0, 0.24], (0.34, 0.06, 0.09))
# Gunner's sight head and radio antennas.
box(T, "olive2", C + [-0.32, -0.46, 0.92], (0.30, 0.28, 0.17))
for y in (0.25, -0.25):
    cyl(T, "dark", C + [0.95, y, 0.74], C + [1.00, y, 2.65], 0.007, 6)
# Stowage basket at the rear of the bustle.
for y in (-0.65, -0.3, 0.05, 0.4, 0.7):
    box(T, "dark", C + [1.92, y, 0.56], (0.34, 0.018, 0.26))
box(T, "dark", C + [2.08, 0.03, 0.56], (0.018, 1.42, 0.26))
box(T, "olive2", C + [1.86, 0.03, 0.56], (0.22, 1.2, 0.2))
T.finish("turret")

# ---- Gun: L7 105 mm in its mantlet. Trunnions at 0.92 ahead of the ring centre, 0.56 above it.
G = Group("gun")
tr = C + [-0.92, 0.0, 0.56]
# The barrel runs in -x, the direction the turret faces at yaw zero.
muzzle = tr + [-5.45, 0, 0]
lathe(G, "olive", tr + [0.08, 0, 0], tr + [-0.74, 0, 0], [(0.02, 0.27), (0.45, 0.26), (0.80, 0.20), (0.98, 0.16)], 36)
barrel = [
    (0.01, 0.105), (0.34, 0.098), (0.35, 0.115), (0.42, 0.115), (0.43, 0.092),
    (0.60, 0.088), (0.61, 0.145), (0.72, 0.145), (0.73, 0.085), (0.92, 0.078),
    (0.925, 0.092), (0.995, 0.09),
]
lathe(G, "olive2", tr + [-0.6, 0, 0], muzzle, barrel, 28)
cyl(G, "black", muzzle, muzzle + [-0.003, 0, 0], 0.055, 20)
box(G, "olive2", tr + [-0.42, 0.34, 0.12], (0.3, 0.1, 0.1))
G.finish("gun")


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
    write(sys.argv[1] if len(sys.argv) > 1 else "assets/leopard.mesh")
