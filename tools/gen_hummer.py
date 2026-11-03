#!/usr/bin/env python3
"""Generates the Hummer H1 mesh (assets/hummer.mesh).

A slab-sided truck, built the way the real one is: flat panels with crisp edges, flared wings with round
arches cut out of them, narrow windows, and everything bolted on in plain sight. The outlines are drawn in side
view and extruded, so that the arches are true circles.

Coordinates: x from the nose rearwards, y to the left, z up, metres. Ground at z = 0.
"""
import math
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull, Delaunay
from shapely import contains_xy
from shapely.geometry import Point, Polygon, box as sbox

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent", "sand"]
M = {n: i for i, n in enumerate(MATS)}
parts = []

FRONT, REAR, AXLE_Z, ARCH_R = 0.74, 4.04, 0.47, 0.60
BODY = "sand"


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


def solid(g, mat, pts):
    """A convex plated solid with crisp edges."""
    pts = np.asarray(pts, float)
    hull = ConvexHull(pts)
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
    h = np.asarray(s, float) * 0.5
    solid(g, mat, [c + h * [i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])


def lathe(g, mat, a, b, prof, seg=28):
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


def cyl(g, mat, a, b, r, seg=20):
    lathe(g, mat, a, b, [(0.002, r), (0.998, r)], seg)


def tube(g, mat, points, r, seg=14):
    """A bent tube through points: bull bar, rails."""
    for a, b in zip(points, points[1:]):
        cyl(g, mat, a, b, r, seg)
    for p in points[1:-1]:
        lathe(g, mat, np.asarray(p) - [0, 0, r], np.asarray(p) + [0, 0, r], [(t, r * math.sqrt(max(1 - (2 * t - 1) ** 2, 1e-4))) for t in np.linspace(0.02, 0.98, 8)], seg)


def extrude(g, mat, poly, y0, y1, h=0.05):
    """A side-view outline as a plate between y0 and y1: both faces and the edge, on a fine enough mesh."""
    rings = [np.array(poly.exterior.coords)[:-1]] + [np.array(i.coords)[:-1] for i in poly.interiors]
    pts = []
    for ring in rings:
        pts.extend(ring.tolist())
    minx, minz, maxx, maxz = poly.bounds
    gx, gz = np.meshgrid(np.arange(minx, maxx + h, h * 2), np.arange(minz, maxz + h, h * 2))
    gg = np.stack([gx.ravel(), gz.ravel()], -1)
    gg = gg[contains_xy(poly.buffer(-h * 0.5), gg[:, 0], gg[:, 1])]
    pts = np.array(pts + gg.tolist())
    tri = Delaunay(pts).simplices
    c = pts[tri].mean(1)
    tri = tri[contains_xy(poly, c[:, 0], c[:, 1])]
    v, n, f = [], [], []
    for y, ny in ((y1, 1.0), (y0, -1.0)):
        base = len(v)
        for x, z in pts:
            v.append((x, y, z))
            n.append((0, ny, 0))
        for t in tri:
            a, b, d = t
            pa, pb, pd = pts[a], pts[b], pts[d]
            cross = (pb[0] - pa[0]) * (pd[1] - pa[1]) - (pb[1] - pa[1]) * (pd[0] - pa[0])
            # Facing +y seen from outside means clockwise in (x, z).
            if (cross > 0) == (ny > 0):
                f.append([base + a, base + d, base + b])
            else:
                f.append([base + a, base + b, base + d])
    # The edge, as its own quads.
    for ring in rings:
        m = len(ring)
        for i in range(m):
            a, b = ring[i], ring[(i + 1) % m]
            d = b - a
            nn = np.array([d[1], 0.0, -d[0]])
            nn /= np.linalg.norm(nn) + 1e-12
            mid = 0.5 * (a + b) + np.array([nn[0], nn[2]]) * 1e-3
            if poly.contains(Point(mid)):
                nn = -nn
            base = len(v)
            v += [(a[0], y0, a[1]), (b[0], y0, b[1]), (b[0], y1, b[1]), (a[0], y1, a[1])]
            n += [nn, nn, nn, nn]
            f += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
    g.add(mat, v, n, f)


def arch(cx):
    return Point(cx, AXLE_Z).buffer(ARCH_R, 64)


def mirror(g_fn):
    for sg in (1, -1):
        g_fn(sg)


# ---- The body, in the sand colour of the desert trucks.
B = Group("body")
# Wings and lower body sides, with their round arches.
side = Polygon([(0.04, 0.66), (0.10, 1.10), (1.46, 1.16), (4.56, 1.16), (4.68, 1.02), (4.66, 0.66)])
side = side.difference(arch(FRONT)).difference(arch(REAR))
for sg in (1, -1):
    y0, y1 = sorted((sg * 1.04, sg * 1.10))
    extrude(B, BODY, side, y0, y1)
# Hood: a wide, nearly flat slab, narrowing a little towards the grille, with a raised centre.
solid(B, BODY, [(x, s * w, z) for x, w, z in ((0.05, 1.04, 1.02), (0.12, 1.08, 1.13), (1.46, 1.08, 1.27), (1.46, 1.08, 1.16), (0.05, 1.04, 0.88)) for s in (1, -1)])
solid(B, BODY, [(x, s * 0.52, z) for x, z in ((0.28, 1.15), (0.34, 1.20), (1.42, 1.32), (1.42, 1.25)) for s in (1, -1)])
# Grille: the nose plate and its slots.
solid(B, BODY, [(x, s * 1.02, z) for x, z in ((0.00, 0.66), (0.05, 0.66), (0.05, 1.04), (0.00, 0.94)) for s in (1, -1)])
for k in range(7):
    y = -0.54 + k * 0.18
    box(B, "black", (0.005, y, 0.83), (0.012, 0.09, 0.20))
# Headlights, round, in the nose.
for sg in (1, -1):
    y = sg * 0.78
    cyl(B, "black", (0.015, y, 0.84), (-0.002, y, 0.84), 0.115, 28)
    cyl(B, "lens", (-0.002, y, 0.84), (-0.006, y, 0.84), 0.092, 28)
    cyl(B, "amber", (-0.002, sg * 0.98, 0.84), (-0.006, sg * 0.98, 0.84), 0.038, 18)
    # Hood vent on top of each wing.
    box(B, "black", (0.86, sg * 0.82, 1.215), (0.40, 0.26, 0.012))
# Roof, pillars and the rear of the cab: flat planes on a narrow greenhouse.
solid(B, BODY, [(x, s * w, z) for x, w, z in ((1.84, 0.80, 1.90), (3.70, 0.80, 1.90), (3.72, 0.84, 1.84), (1.82, 0.84, 1.84)) for s in (1, -1)])
for sg in (1, -1):
    # Windscreen pillar, door frames, rear pillar.
    solid(B, BODY, [(1.46, sg * 1.04, 1.26), (1.46, sg * 0.98, 1.26), (1.86, sg * 0.82, 1.86), (1.82, sg * 0.86, 1.86), (1.56, sg * 1.04, 1.26), (1.92, sg * 0.84, 1.84)])
    solid(B, BODY, [(3.62, sg * 1.04, 1.16), (3.98, sg * 1.04, 1.16), (3.76, sg * 0.86, 1.86), (3.62, sg * 0.86, 1.86), (3.62, sg * 0.98, 1.16), (3.98, sg * 0.98, 1.16)])
    # Below the windows, up to the belt, and the wide shoulder over the wheels.
    solid(B, BODY, [(1.50, sg * 1.04, 1.16), (3.70, sg * 1.04, 1.16), (3.70, sg * 1.02, 1.32), (1.50, sg * 1.02, 1.32), (1.50, sg * 0.94, 1.32), (3.70, sg * 0.94, 1.32), (1.50, sg * 0.94, 1.16), (3.70, sg * 0.94, 1.16)])
    # Roof rail, cab side frame: door-sized frames with the glass inside.
    box(B, BODY, (2.72, sg * 0.86, 1.82), (1.90, 0.06, 0.07))
    box(B, BODY, (2.72, sg * 0.97, 1.35), (1.90, 0.08, 0.05))
    # B pillar.
    solid(B, BODY, [(2.68, sg * 1.01, 1.32), (2.80, sg * 1.01, 1.32), (2.68, sg * 0.97, 1.32), (2.80, sg * 0.97, 1.32), (2.68, sg * 0.84, 1.84), (2.80, sg * 0.84, 1.84), (2.68, sg * 0.80, 1.84), (2.80, sg * 0.80, 1.84)])
    # Side windows.
    solid(B, "glass", [(1.92, sg * 0.97, 1.35), (2.66, sg * 0.97, 1.35), (2.66, sg * 0.84, 1.80), (2.05, sg * 0.84, 1.80), (1.92, sg * 0.91, 1.35), (2.66, sg * 0.91, 1.35), (2.66, sg * 0.79, 1.80), (2.05, sg * 0.79, 1.80)])
    solid(B, "glass", [(2.82, sg * 0.97, 1.35), (3.60, sg * 0.97, 1.35), (3.60, sg * 0.84, 1.80), (2.82, sg * 0.84, 1.80), (2.82, sg * 0.91, 1.35), (3.60, sg * 0.91, 1.35), (3.60, sg * 0.79, 1.80), (2.82, sg * 0.79, 1.80)])
    # Rear door shut line, the running board and the rock rail below the doors.
    box(B, "black", (2.74, sg * 1.104, 0.92), (0.012, 0.004, 0.44))
    box(B, "black", (3.66, sg * 1.104, 0.92), (0.012, 0.004, 0.44))
    box(B, "black", (2.74, sg * 1.10, 0.68), (1.86, 0.06, 0.08))
    # Door handles.
    box(B, "black", (2.46, sg * 1.105, 1.08), (0.15, 0.015, 0.035))
    box(B, "black", (3.45, sg * 1.105, 1.08), (0.15, 0.015, 0.035))
# Split windscreen, each pane flat and upright, as on the real truck, with a centre bar.
for sg in (1, -1):
    solid(B, "glass", [(1.49, sg * 0.04, 1.30), (1.49, sg * 0.92, 1.30), (1.84, sg * 0.78, 1.82), (1.84, sg * 0.04, 1.82), (1.52, sg * 0.04, 1.30), (1.52, sg * 0.92, 1.30), (1.87, sg * 0.78, 1.82), (1.87, sg * 0.04, 1.82)])
solid(B, BODY, [(1.46, 0.045, 1.29), (1.46, -0.045, 1.29), (1.86, 0.045, 1.86), (1.86, -0.045, 1.86), (1.50, 0.045, 1.28), (1.50, -0.045, 1.28), (1.90, 0.045, 1.85), (1.90, -0.045, 1.85)])
box(B, BODY, (1.48, 0.0, 1.27), (0.09, 1.96, 0.05))
# The sloping tail, with its window.
solid(B, BODY, [(x, s * w, z) for x, w, z in ((3.72, 0.84, 1.86), (3.76, 0.82, 1.86), (4.46, 1.04, 1.24), (4.52, 1.04, 1.18)) for s in (1, -1)])
solid(B, BODY, [(x, s * 1.04, z) for x, z in ((3.98, 1.18), (4.60, 1.18), (4.68, 1.02), (4.66, 0.66), (3.98, 0.66)) for s in (1, -1)])
solid(B, "glass", [(3.92, 0.60, 1.74), (3.92, -0.60, 1.74), (4.36, 0.74, 1.34), (4.36, -0.74, 1.34), (3.885, 0.60, 1.71), (3.885, -0.60, 1.71), (4.33, 0.74, 1.31), (4.33, -0.74, 1.31)])
for sg in (1, -1):
    box(B, "tail", (4.67, sg * 0.86, 0.96), (0.015, 0.20, 0.12))
    box(B, "amber", (4.67, sg * 0.86, 0.84), (0.015, 0.20, 0.07))
    box(B, "lens", (4.67, sg * 0.60, 0.95), (0.012, 0.13, 0.09))
    # Mirrors, on long arms, very much a truck's.
    box(B, "black", (1.70, sg * 1.26, 1.50), (0.05, 0.22, 0.34))
    cyl(B, "black", (1.76, sg * 1.00, 1.34), (1.70, sg * 1.20, 1.46), 0.012, 10)
    cyl(B, "black", (1.76, sg * 0.98, 1.60), (1.70, sg * 1.20, 1.58), 0.012, 10)
# Underneath and inside: a closed floor, and the wheel wells.
box(B, "liner", (2.35, 0.0, 0.645), (4.60, 2.02, 0.03))
box(B, "liner", (1.50, 0.0, 0.95), (0.03, 1.96, 0.60))
box(B, "liner", (3.72, 0.0, 1.10), (0.03, 1.62, 0.90))
for cx in (FRONT, REAR):
    for sg in (1, -1):
        th = np.linspace(math.radians(-15), math.radians(195), 44)
        v, n, f = [], [], []
        for t in th:
            x, z = cx + (ARCH_R + 0.008) * math.cos(t), AXLE_Z + (ARCH_R + 0.008) * math.sin(t)
            nx, nz = -math.cos(t), -math.sin(t)
            for y in (sg * 0.50, sg * 1.09):
                v.append((x, y, z))
                n.append((nx, 0, nz))
        for i in range(len(th) - 1):
            a = i * 2
            f += [[a, a + 2, a + 3], [a, a + 3, a + 1]]
        B.add("liner", v, n, f)
        poly = Polygon([(cx + (ARCH_R + 0.012) * math.cos(t), AXLE_Z + (ARCH_R + 0.012) * math.sin(t)) for t in th])
        extrude(B, "liner", poly, sg * 0.50 - 0.005, sg * 0.50 + 0.005, 0.1)
# ---- What makes it look mean: bull bar with a winch, roof rack, snorkel, a spare on the tail.
for sg in (1, -1):
    tube(B, "black", [(-0.20, sg * 0.86, 0.62), (-0.20, sg * 0.86, 1.04), (0.05, sg * 0.86, 1.06)], 0.032)
    box(B, "black", (-0.08, sg * 0.86, 0.66), (0.28, 0.10, 0.08))
    # Tow hooks, and the fog lamps on the bar.
    cyl(B, "metal", (-0.20, sg * 0.62, 0.60), (-0.26, sg * 0.62, 0.60), 0.02, 12)
    cyl(B, "black", (-0.24, sg * 0.70, 1.02), (-0.20, sg * 0.70, 1.02), 0.075, 22)
    cyl(B, "lens", (-0.245, sg * 0.70, 1.02), (-0.24, sg * 0.70, 1.02), 0.065, 22)
tube(B, "black", [(-0.20, 0.86, 0.72), (-0.20, -0.86, 0.72)], 0.038)
tube(B, "black", [(-0.20, 0.86, 1.02), (-0.20, -0.86, 1.02)], 0.028)
box(B, "dark", (-0.10, 0.0, 0.70), (0.22, 0.58, 0.20))
cyl(B, "metal", (-0.10, 0.22, 0.70), (-0.10, -0.22, 0.70), 0.085, 22)
box(B, "metal", (-0.22, 0.0, 0.70), (0.04, 0.14, 0.05))
# The roof rack: tubes on feet, and a row of lights across the front of it.
rack = [(1.98, 0.74, 2.02), (3.62, 0.74, 2.02), (3.62, -0.74, 2.02), (1.98, -0.74, 2.02), (1.98, 0.74, 2.02)]
tube(B, "black", rack, 0.018, 10)
for x in (2.3, 2.62, 2.94, 3.26):
    cyl(B, "black", (x, 0.74, 2.02), (x, -0.74, 2.02), 0.012, 8)
for x in (1.98, 3.62):
    for sg in (1, -1):
        cyl(B, "black", (x, sg * 0.74, 1.90), (x, sg * 0.74, 2.02), 0.016, 8)
for k in range(4):
    y = -0.54 + k * 0.36
    cyl(B, "black", (1.97, y, 2.08), (1.91, y, 2.08), 0.075, 20)
    cyl(B, "lens", (1.91, y, 2.08), (1.905, y, 2.08), 0.065, 20)
    cyl(B, "black", (1.97, y, 2.02), (1.97, y, 2.08), 0.015, 8)
# Jerry cans and a box on the rack.
box(B, "olive", (3.32, 0.32, 2.12), (0.46, 0.34, 0.20))
box(B, "olive2", (3.32, -0.32, 2.12), (0.46, 0.34, 0.20))
# The snorkel, up the right-hand windscreen pillar.
tube(B, "black", [(1.48, -1.13, 1.18), (1.58, -1.13, 1.34), (1.72, -1.03, 1.90)], 0.05, 14)
box(B, "black", (1.70, -1.03, 1.96), (0.16, 0.13, 0.12))
# The spare wheel on a swing-out carrier at the back.
box(B, "black", (4.74, 0.0, 1.00), (0.10, 0.10, 0.60))
lathe(B, "rubber", (4.78, 0.0, 1.02), (5.06, 0.0, 1.02), [(0.02, 0.30), (0.1, 0.40), (0.3, 0.46), (0.7, 0.46), (0.9, 0.40), (0.98, 0.30)], 40)
cyl(B, "dark", (5.05, 0.0, 1.02), (5.07, 0.0, 1.02), 0.28, 28)
tube(B, "black", [(4.70, 0.9, 0.60), (4.86, 0.9, 0.60), (4.86, -0.9, 0.60), (4.70, -0.9, 0.60)], 0.034)
B.finish("body")

# ---- Inside, with the wide tunnel the engine and drivetrain leave down the middle.
F = Group("frame")
box(F, "fabric", (2.70, 0.0, 0.92), (2.1, 0.64, 0.50))
box(F, "dark", (1.70, 0.0, 1.18), (0.34, 1.88, 0.20))
for x in (2.25, 3.15):
    for sg in (1, -1):
        y = sg * 0.62
        box(F, "fabric", (x, y, 0.92), (0.50, 0.42, 0.12))
        box(F, "fabric", (x + 0.27, y, 1.26), (0.12, 0.40, 0.58))
F.finish("inside")


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
            v.append(c + d * (R + r * math.cos(p)) + a * (r * math.sin(p)))
            n.append(d * math.cos(p) + a * math.sin(p))
    for i in range(major):
        for j in range(minor):
            A, Bb = i * minor + j, i * minor + (j + 1) % minor
            C, D = ((i + 1) % major) * minor + j, ((i + 1) % major) * minor + (j + 1) % minor
            f += [[A, C, D], [A, D, Bb]]
    return v, n, f


sw = np.array([2.02, 0.62, 1.30])
axis = np.array([-math.cos(math.radians(24)), 0, -math.sin(math.radians(24))])
R = Group("frame")
R.add("black", *torus(sw, axis, 0.19, 0.016))
R.finish("steerwheel_rim")
H = Group("frame")
cyl(H, "dark", sw - axis * 0.01, sw + axis * 0.04, 0.08, 32)
e1 = np.cross(axis, [0.0, 1.0, 0.0])
e1 /= np.linalg.norm(e1)
for ang in (0.0, 180.0, 270.0):
    d = e1 * math.cos(math.radians(ang)) + np.cross(axis, e1) * math.sin(math.radians(ang))
    cyl(H, "black", sw + d * 0.07, sw + d * 0.18, 0.014, 10)
H.finish("steerwheel_hub")
C = Group("frame")
cyl(C, "black", sw + axis * 0.04, sw + axis * 0.36, 0.026, 16)
C.finish("column")

# Doors: the front door's panel below the window, as a part of its own that can come off.
for sg, name in ((1, "door_l"), (-1, "door_r")):
    D = Group(name)
    poly = Polygon([(1.86, 0.66), (2.72, 0.66), (2.72, 1.15), (1.86, 1.15)])
    y0, y1 = sorted((sg * 1.104, sg * 1.116))
    extrude(D, BODY, poly, y0, y1)
    box(D, "black", (1.86, sg * 1.118, 0.905), (0.012, 0.004, 0.49))
    box(D, "black", (2.46, sg * 1.122, 1.08), (0.15, 0.015, 0.035))
    D.finish(name)

# ---- Mechanicals, for the X-ray view.
E = Group("engine")
box(E, "metal", (0.86, 0.0, 0.96), (0.86, 0.50, 0.46))
for sg in (1, -1):
    box(E, "alu", (0.86, sg * 0.28, 1.14), (0.80, 0.14, 0.14))
E.finish("engine")
G = Group("gearbox")
box(G, "alu", (1.66, 0.0, 0.82), (0.72, 0.34, 0.32))
G.finish("gearbox")
RD = Group("radiator")
box(RD, "dark", (0.23, 0.0, 0.94), (0.10, 1.06, 0.30))
RD.finish("radiator")
TK = Group("tank")
box(TK, "dark", (3.50, 0.0, 0.83), (0.38, 1.20, 0.28))
TK.finish("tank")

# ---- Wheels, axle along y, outboard +y: 37 inch mud tyres with real lugs on beadlock rims.
TY = Group("wheel")
half = 0.165
tyre = [(0.0, 0.30), (0.04, 0.345), (0.12, 0.42), (0.22, 0.445), (0.5, 0.452), (0.78, 0.445), (0.88, 0.42), (0.96, 0.345), (1.0, 0.30)]
lathe(TY, "rubber", (0, -half, 0), (0, half, 0), tyre, 72)
# Chunky tread blocks, staggered, wrapping round onto the shoulders.
for row, (y, w) in enumerate(((-0.115, 0.095), (-0.03, 0.06), (0.03, 0.06), (0.115, 0.095))):
    n = 24
    for k in range(n):
        q = 2 * math.pi * (k + (0.5 if row % 2 else 0.0)) / n
        dx, dz = math.cos(q), math.sin(q)
        tx, tz = -dz, dx
        rO, rI = 0.470, 0.445
        wid = 0.065
        pts = []
        for r in (rO, rI):
            for s in (-1, 1):
                for yy in (y - w / 2, y + w / 2):
                    pts.append((dx * r + tx * s * wid, yy, dz * r + tz * s * wid))
        solid(TY, "rubber", pts)
TY.finish("wheel_tyre")
parts[-1] = ("wheel_tyre", "wheel", M["rubber"], parts[-1][3], parts[-1][4], parts[-1][5])
BR = Group("wheel")
lathe(BR, "dark", (0, -0.155, 0), (0, 0.155, 0), [(0.02, 0.302), (0.08, 0.285), (0.92, 0.285), (0.98, 0.302)], 48)
BR.finish("wheel_barrel")
FC = Group("wheel")
lathe(FC, "dark", (0, 0.10, 0), (0, 0.13, 0), [(0.02, 0.05), (0.2, 0.26), (0.8, 0.28), (0.98, 0.29)], 48)
# Beadlock ring and its bolts.
lathe(FC, "metal", (0, 0.145, 0), (0, 0.170, 0), [(0.05, 0.268), (0.1, 0.315), (0.9, 0.315), (0.95, 0.268)], 56)
for k in range(16):
    q = 2 * math.pi * k / 16
    cyl(FC, "alu", (math.cos(q) * 0.292, 0.168, math.sin(q) * 0.292), (math.cos(q) * 0.292, 0.178, math.sin(q) * 0.292), 0.012, 8)
# The portal hub in the middle.
cyl(FC, "metal", (0, 0.11, 0), (0, 0.19, 0), 0.105, 32)
for k in range(8):
    q = 2 * math.pi * k / 8
    cyl(FC, "alu", (math.cos(q) * 0.08, 0.188, math.sin(q) * 0.08), (math.cos(q) * 0.08, 0.198, math.sin(q) * 0.08), 0.014, 8)
FC.finish("wheel_face")
HB = Group("wheel")
cyl(HB, "metal", (0, -0.10, 0), (0, 0.11, 0), 0.09, 20)
HB.finish("wheel_hub")
DS = Group("wheel")
lathe(DS, "disc", (0, -0.02, 0), (0, 0.02, 0), [(0.02, 0.12), (0.1, 0.235), (0.9, 0.235), (0.98, 0.12)], 44)
DS.finish("wheel_disc")
K = Group("wheel_static")
box(K, "metal", (0.13, 0.0, 0.17), (0.14, 0.11, 0.13))
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
    write(sys.argv[1] if len(sys.argv) > 1 else "assets/hummer.mesh")
