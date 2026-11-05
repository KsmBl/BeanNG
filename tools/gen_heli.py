#!/usr/bin/env python3
"""Generates the helicopter, a heavy two-seat gunship: python3 gen_heli.py out.mesh

Coordinates as for the vehicles: x rearwards, y to the left, z up, skids on the ground at z = 0, the mast at x = 0.
Four groups, each moved rigidly by the game: the airframe, the main rotor, the tail rotor and the gatling's barrels.
"""
import math
import struct
import sys

import numpy as np

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent", "sand", "skin", "grass", "grass2", "dirt", "leaf"]
M = {n: i for i, n in enumerate(MATS)}
parts = []


def normals(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def add(group, mat, v, f, n=None):
    v = np.asarray(v, float)
    f = np.asarray(f, int)
    parts.append(("%s_%s_%d" % (group, mat, len(parts)), group, M[mat], v, normals(v, f) if n is None else n, f))


def section(x, z, top, bottom, w, seg, power=2.5, y0=0.0):
    """A ring across the body, rounder or squarer by power: flat sides, as a gunship has."""
    out = []
    for j in range(seg):
        a = 2 * math.pi * j / seg
        s, c = math.sin(a), math.cos(a)
        y = w * math.copysign(abs(s) ** (2 / power), s)
        h = top if c > 0 else bottom
        out.append((x, y0 + y, z + h * math.copysign(abs(c) ** (2 / power), c)))
    return out


def loft_mesh(rings, seg, power=2.5, y0=0.0):
    v, f = [], []
    for x, z, top, bottom, w in rings:
        v += section(x, z, top, bottom, w, seg, power, y0)
    n = len(rings)
    for i in range(n - 1):
        for j in range(seg):
            A, B = i * seg + j, i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, D, B], [A, C, D]]
    base = len(v)
    v.append(tuple(np.mean(v[:seg], axis=0)))
    v.append(tuple(np.mean(v[-seg - 1:-1], axis=0)))
    for j in range(seg):
        f.append([base, j, (j + 1) % seg])
        f.append([base + 1, (n - 1) * seg + (j + 1) % seg, (n - 1) * seg + j])
    return np.asarray(v, float), np.asarray(f, int)


def loft(group, mat, rings, seg=20, power=2.5, y0=0.0):
    v, f = loft_mesh(rings, seg, power, y0)
    add(group, mat, v, f)


def camo(group, rings, seg=36, power=2.5):
    """One smooth skin, painted in soft-edged blotches: the colours change from face to face, the surface does not."""
    v, f = loft_mesh(rings, seg, power)
    n = normals(v, f)
    pick = {"olive": [], "olive2": []}
    for tri in f:
        c = v[tri].mean(axis=0)
        u = math.sin(c[0] * 1.7 + c[2] * 2.3) + math.sin(c[0] * 0.8 - abs(c[1]) * 3.1 + c[2] * 1.1 + 1.3)
        pick["olive2" if u > 0.55 else "olive"].append(tri)
    for mat, tris in pick.items():
        if tris:
            add(group, mat, v, np.asarray(tris, int), n)


def box_mesh(corners):
    q = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    V, F = [], []
    for a, b, c, d in q:
        base = len(V)
        V += [corners[a], corners[b], corners[c], corners[d]]
        F += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
    return V, F


def box(group, mat, x0, x1, y0, y1, z0, z1):
    V, F = box_mesh([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)])
    add(group, mat, V, F)


def rod(group, mat, a, b, r, seg=10):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    u = np.cross(d, [0, 0, 1] if abs(d[2]) < 0.9 else [1, 0, 0])
    u /= np.linalg.norm(u)
    w = np.cross(d, u)
    v, f = [], []
    for p in (a, b):
        for j in range(seg):
            t = 2 * math.pi * j / seg
            v.append(tuple(p + (u * math.cos(t) + w * math.sin(t)) * r))
    for j in range(seg):
        k = (j + 1) % seg
        f += [[j, k, seg + k], [j, seg + k, seg + j]]
    base = len(v)
    v += [tuple(a), tuple(b)]
    for j in range(seg):
        k = (j + 1) % seg
        f += [[base, k, j], [base + 1, seg + j, seg + k]]
    add(group, mat, v, f)


def path(group, mat, points, r, seg=8):
    for a, b in zip(points, points[1:]):
        rod(group, mat, a, b, r, seg)


def plate(group, mat, outline, half):
    """A flat shape in the x-z plane, half thick to each side."""
    n = len(outline)
    V, F = [], []
    front = [(x, half, z) for x, z in outline]
    back = [(x, -half, z) for x, z in outline]
    V += front
    for k in range(1, n - 1):
        F.append([0, k + 1, k])
    base = len(V)
    V += back
    for k in range(1, n - 1):
        F.append([base, base + k, base + k + 1])
    for k in range(n):
        j = (k + 1) % n
        base = len(V)
        V += [front[k], front[j], back[j], back[k]]
        F += [[base, base + 2, base + 1], [base, base + 3, base + 2]]
    add(group, mat, V, F)


def wing(group, mat, x0, x1, tipx0, tipx1, y0, y1, z, zt, thick):
    for side in (-1, 1):
        c = [(x0, side * y0, z - thick), (x1, side * y0, z - thick), (tipx0, side * y1, zt - thick * 0.6), (tipx1, side * y1, zt - thick * 0.6),
             (x0, side * y0, z + thick), (x1, side * y0, z + thick), (tipx0, side * y1, zt + thick * 0.6), (tipx1, side * y1, zt + thick * 0.6)]
        if side < 0:
            c = [c[2], c[3], c[0], c[1], c[6], c[7], c[4], c[5]]
        V, F = box_mesh(c)
        add(group, mat, V, F)


def faceted(group, mat, rings, seg=10, power=1.7):
    """Flat panels, each face with its own corners: glass does not bend the way paint does."""
    v, f = loft_mesh(rings, seg, power)
    V, F = [], []
    for tri in f:
        base = len(V)
        V += [tuple(v[tri[0]]), tuple(v[tri[1]]), tuple(v[tri[2]])]
        F.append([base, base + 1, base + 2])
    add(group, mat, V, F)
    return v


def wheel(y, x, z, r, width):
    rod("body", "rubber", (x, y - width, z), (x, y + width, z), r, seg=20)
    rod("body", "metal", (x, y - width * 1.15, z), (x, y + width * 1.15, z), r * 0.55, seg=12)


# ---- The airframe: deep and slab-sided, the tail running out high.
BODY = [(-3.72, 1.05, 0.02, 0.02, 0.02), (-3.62, 1.05, 0.2, 0.24, 0.2), (-3.3, 1.05, 0.36, 0.42, 0.36), (-2.8, 1.05, 0.46, 0.5, 0.46), (-2.1, 1.08, 0.55, 0.58, 0.52),
        (-1.2, 1.15, 0.7, 0.62, 0.62), (-0.2, 1.25, 0.85, 0.62, 0.68), (0.8, 1.38, 0.82, 0.58, 0.66), (1.6, 1.55, 0.62, 0.46, 0.5), (2.4, 1.78, 0.42, 0.33, 0.34),
        (3.4, 1.98, 0.3, 0.26, 0.24), (4.8, 2.15, 0.24, 0.2, 0.18), (6.2, 2.3, 0.2, 0.16, 0.13), (7.05, 2.42, 0.16, 0.12, 0.09), (7.12, 2.43, 0.02, 0.02, 0.02)]
camo("body", BODY, seg=40, power=3.2)
# Panel lines round the skin.
for x in (-2.9, -2.0, -0.7, 0.3, 1.2, 2.2, 3.3, 4.6, 5.8):
    for k in range(len(BODY) - 1):
        a, b = BODY[k], BODY[k + 1]
        if a[0] <= x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            r = [a[i] + (b[i] - a[i]) * t for i in range(5)]
            loft("body", "dark", [(x - 0.012, r[1], r[2] + 0.005, r[3] + 0.005, r[4] + 0.005), (x + 0.012, r[1], r[2] + 0.005, r[3] + 0.005, r[4] + 0.005)], seg=40, power=3.2)
# Along the belly and the back of the boom, a strake and a spine.
path("body", "dark", [(2.2, 0, 2.2), (3.4, 0, 2.3), (4.8, 0, 2.41), (6.2, 0, 2.52)], 0.03, seg=6)
path("body", "dark", [(-1.4, 0, 0.53), (0.4, 0, 0.66), (1.6, 0, 1.08)], 0.035, seg=6)
for side in (-1, 1):
    # The side bays along the cabin.
    loft("body", "olive", [(-2.05, 0.98, 0.02, 0.02, 0.02), (-1.9, 0.98, 0.2, 0.22, 0.12), (-1.3, 0.98, 0.33, 0.33, 0.2), (0.5, 0.98, 0.36, 0.36, 0.22), (1.2, 1.08, 0.26, 0.26, 0.16), (1.5, 1.14, 0.02, 0.02, 0.02)], seg=20, power=3.2, y0=side * 0.68)
    for x in (-1.2, -0.6, 0.0, 0.5):
        box("body", "dark", x - 0.008, x + 0.008, side * 0.895 - 0.012, side * 0.895 + 0.012, 0.68, 1.3)
    path("body", "dark", [(-1.5, side * 0.895, 1.28), (0.8, side * 0.895, 1.32)], 0.012, seg=5)
    path("body", "dark", [(-1.5, side * 0.895, 0.68), (0.8, side * 0.895, 0.68)], 0.012, seg=5)
    for x in (-1.0, -0.8, -0.2, 0.2):
        box("body", "black", x, x + 0.1, side * 0.885 - 0.012, side * 0.885 + 0.012, 1.05, 1.2)

# ---- The canopy: gunner low in front, pilot high behind, both in flat panes.
GUNNER = [(-3.18, 1.28, 0.02, 0.1, 0.1), (-2.98, 1.33, 0.2, 0.08, 0.33), (-2.25, 1.5, 0.3, 0.1, 0.42), (-1.72, 1.56, 0.31, 0.08, 0.42), (-1.7, 1.56, 0.02, 0.02, 0.02)]
PILOT = [(-1.8, 1.7, 0.02, 0.02, 0.02), (-1.75, 1.72, 0.26, 0.14, 0.42), (-1.15, 1.92, 0.34, 0.2, 0.44), (-0.55, 2.05, 0.27, 0.22, 0.42), (-0.4, 2.07, 0.02, 0.02, 0.02)]
for rings in (GUNNER, PILOT):
    seg = 10
    v = faceted("body", "glass", rings, seg, 1.7)
    # A frame round every pane on the upper half.
    for k in range(1, len(rings) - 1):
        ring = [tuple(p) for p in v[k * seg:(k + 1) * seg]]
        path("body", "dark", [ring[m % seg] for m in range(-3, 4)], 0.022, seg=6)
    for m in range(-3, 4):
        path("body", "dark", [tuple(v[k * seg + m % seg]) for k in range(1, len(rings) - 1)], 0.018, seg=6)
# Crew: helmets, shoulders and a seat back behind each, seen through the glass.
for x, z in ((-2.1, 1.5), (-1.0, 1.95)):
    loft("body", "white", [(x - 0.14, z + 0.1, 0.01, 0.01, 0.01), (x - 0.11, z + 0.1, 0.11, 0.11, 0.1), (x + 0.11, z + 0.1, 0.11, 0.11, 0.1), (x + 0.14, z + 0.1, 0.01, 0.01, 0.01)], seg=12, power=2)
    loft("body", "olive2", [(x - 0.14, z - 0.2, 0.2, 0.2, 0.2), (x + 0.14, z - 0.2, 0.2, 0.2, 0.2)], seg=12, power=3)
    box("body", "black", x + 0.16, x + 0.24, -0.22, 0.22, z - 0.4, z + 0.18)
# A wiper and the bar for the sights.
rod("body", "black", (-2.95, 0.05, 1.32), (-2.75, -0.15, 1.5), 0.008, seg=4)

# ---- The nose: sensors in a turret at the tip, and the gatling in its ball under the chin.
loft("body", "dark", [(-3.98, 1.1, 0.02, 0.02, 0.02), (-3.94, 1.1, 0.17, 0.17, 0.17), (-3.66, 1.1, 0.2, 0.2, 0.2), (-3.52, 1.1, 0.14, 0.14, 0.14), (-3.48, 1.1, 0.02, 0.02, 0.02)], seg=18, power=2.4)
for y, r in ((0.09, 0.065), (-0.09, 0.05)):
    rod("body", "lens", (-3.98, y, 1.12), (-3.99, y, 1.12), r, seg=12)
    rod("body", "black", (-3.975, y, 1.12), (-3.985, y, 1.12), r * 1.3, seg=12)
rod("body", "metal", (-3.3, 0.3, 1.25), (-4.05, 0.3, 1.25), 0.008, seg=5)
rod("body", "metal", (-3.3, -0.3, 1.25), (-4.05, -0.3, 1.25), 0.008, seg=5)
loft("body", "dark", [(-3.34, 0.58, 0.02, 0.02, 0.02), (-3.28, 0.58, 0.16, 0.16, 0.16), (-3.05, 0.58, 0.27, 0.27, 0.27), (-2.78, 0.58, 0.25, 0.25, 0.25), (-2.6, 0.58, 0.12, 0.12, 0.12), (-2.55, 0.58, 0.02, 0.02, 0.02)], seg=22, power=2)
loft("gun", "metal", [(-3.44, 0.58, 0.09, 0.09, 0.09), (-3.3, 0.58, 0.12, 0.12, 0.12), (-3.25, 0.58, 0.02, 0.02, 0.02)], seg=14, power=2)
for k in range(6):
    a = 2 * math.pi * k / 6
    y, z = 0.075 * math.cos(a), 0.58 + 0.075 * math.sin(a)
    rod("gun", "metal", (-4.05, y, z), (-3.3, y, z), 0.019, seg=6)
for x in (-4.0, -3.66):
    loft("gun", "dark", [(x - 0.02, 0.58, 0.108, 0.108, 0.108), (x + 0.02, 0.58, 0.108, 0.108, 0.108)], seg=14, power=2)
    # Bolts on the clamp, for the eye to see it turn.
    for k in range(3):
        a = 2 * math.pi * k / 3
        box("gun", "white", x - 0.024, x + 0.024, 0.09 * math.cos(a) - 0.018, 0.09 * math.cos(a) + 0.018, 0.58 + 0.09 * math.sin(a) - 0.018, 0.58 + 0.09 * math.sin(a) + 0.018)

# ---- Two engines in their own nacelles beside the mast, each with an intake and a shrouded exhaust.
loft("body", "olive2", [(-0.5, 2.05, 0.02, 0.02, 0.02), (-0.42, 2.06, 0.2, 0.1, 0.3), (0.5, 2.15, 0.32, 0.12, 0.36), (1.2, 2.08, 0.24, 0.1, 0.28), (1.7, 1.98, 0.02, 0.02, 0.02)], seg=22, power=3)
for side in (-1, 1):
    y = side * 0.72
    loft("body", "olive", [(-0.42, 2.16, 0.24, 0.24, 0.22), (-0.3, 2.16, 0.3, 0.3, 0.28), (0.6, 2.16, 0.31, 0.31, 0.29), (1.3, 2.14, 0.26, 0.26, 0.24), (1.75, 2.12, 0.2, 0.2, 0.18), (1.78, 2.12, 0.02, 0.02, 0.02)], seg=22, power=2.3, y0=y)
    loft("body", "black", [(-0.44, 2.16, 0.02, 0.02, 0.02), (-0.43, 2.16, 0.2, 0.2, 0.18), (-0.41, 2.16, 0.2, 0.2, 0.18)], seg=18, power=2, y0=y)
    rod("body", "metal", (-0.46, y, 2.16), (-0.4, y, 2.16), 0.06, seg=10)
    rod("body", "dark", (1.7, y, 2.12), (2.05, y + side * 0.14, 2.15), 0.16, seg=14)
    rod("body", "black", (2.03, y + side * 0.13, 2.148), (2.06, y + side * 0.145, 2.152), 0.12, seg=14)
    for x in (0.1, 0.65):
        box("body", "dark", x - 0.008, x + 0.008, y - 0.3, y + 0.3, 2.18, 2.48)
    for x in (0.2, 0.32, 0.44):
        box("body", "black", x, x + 0.06, y + side * 0.285 - 0.012, y + side * 0.285 + 0.012, 2.06, 2.28)
    box("body", "olive2", -0.2, 1.1, side * 0.45 - 0.04, side * 0.45 + 0.04, 1.85, 2.0)
# The mast, with the radar dome on top of it that the rotor turns under.
loft("body", "olive", [(-0.55, 2.42, 0.02, 0.02, 0.02), (-0.45, 2.42, 0.22, 0.12, 0.2), (0.45, 2.42, 0.22, 0.12, 0.22), (0.55, 2.42, 0.02, 0.02, 0.02)], seg=16, power=2.2)
rod("body", "metal", (0, 0, 2.5), (0, 0, 3.55), 0.08, seg=14)
loft("body", "dark", [(-0.3, 3.62, 0.01, 0.01, 0.01), (-0.28, 3.62, 0.09, 0.09, 0.09), (-0.2, 3.62, 0.16, 0.14, 0.2), (0.0, 3.62, 0.18, 0.15, 0.29), (0.2, 3.62, 0.16, 0.14, 0.2), (0.28, 3.62, 0.09, 0.09, 0.09), (0.3, 3.62, 0.01, 0.01, 0.01)], seg=20, power=2)
# Aerials.
rod("body", "black", (3.0, 0, 2.26), (3.3, 0, 2.78), 0.01, seg=5)
rod("body", "black", (4.1, 0, 2.36), (4.35, 0, 2.78), 0.008, seg=5)
loft("body", "dark", [(-0.15, 0.56, 0.01, 0.01, 0.01), (-0.1, 0.5, 0.08, 0.05, 0.16), (0.3, 0.5, 0.08, 0.05, 0.16), (0.35, 0.5, 0.01, 0.01, 0.01)], seg=12, power=2)

# ---- The wings, each with a pod of rockets out at the end and four missiles on a rack inboard.
wing("body", "olive", -0.45, 0.55, -0.22, 0.28, 0.6, 1.95, 1.62, 1.5, 0.06)
for side in (-1, 1):
    loft("body", "dark", [(-0.22, 1.5, 0.05, 0.05, 0.02), (0.28, 1.5, 0.05, 0.05, 0.02)], seg=8, power=2, y0=side * 1.96)
    # The pod.
    y = side * 1.68
    box("body", "dark", -0.2, 0.25, y - 0.04, y + 0.04, 1.3, 1.5)
    loft("body", "olive2", [(-0.95, 1.08, 0.02, 0.02, 0.02), (-0.92, 1.08, 0.2, 0.2, 0.2), (0.45, 1.08, 0.22, 0.22, 0.22), (0.7, 1.08, 0.14, 0.14, 0.14), (0.74, 1.08, 0.02, 0.02, 0.02)], seg=20, power=2, y0=y)
    for k in range(12):
        ring = 0.15 if k >= 4 else 0.07
        a = 2 * math.pi * k / (8 if k >= 4 else 4)
        rod("body", "black", (-0.955, y + ring * math.cos(a), 1.08 + ring * math.sin(a)), (-0.93, y + ring * math.cos(a), 1.08 + ring * math.sin(a)), 0.032, seg=6)
    for x in (-0.6, 0.2):
        loft("body", "dark", [(x - 0.012, 1.08, 0.222, 0.222, 0.222), (x + 0.012, 1.08, 0.222, 0.222, 0.222)], seg=20, power=2, y0=y)
    # The missile rack.
    y2 = side * 1.1
    box("body", "dark", -0.1, 0.2, y2 - 0.04, y2 + 0.04, 1.3, 1.55)
    box("body", "dark", -0.65, 0.65, y2 - 0.17, y2 + 0.17, 1.25, 1.31)
    for dy, dz in ((-0.1, 0.0), (0.1, 0.0), (-0.1, -0.2), (0.1, -0.2)):
        yy, zz = y2 + dy, 1.14 + dz
        loft("body", "olive", [(-1.0, zz, 0.005, 0.005, 0.005), (-0.92, zz, 0.06, 0.06, 0.06), (-0.75, zz, 0.085, 0.085, 0.085), (0.65, zz, 0.085, 0.085, 0.085), (0.67, zz, 0.01, 0.01, 0.01)], seg=10, power=2, y0=yy)
        loft("body", "lens", [(-1.0, zz, 0.005, 0.005, 0.005), (-0.92, zz, 0.06, 0.06, 0.06), (-0.87, zz, 0.005, 0.005, 0.005)], seg=10, power=2, y0=yy)
        rod("body", "dark", (0.55, yy - 0.12, zz), (0.55, yy + 0.12, zz), 0.008, seg=4)
        rod("body", "dark", (0.55, yy, zz - 0.12), (0.55, yy, zz + 0.12), 0.008, seg=4)
    # At the very tip, a small missile for what flies.
    loft("body", "white", [(-0.6, 1.42, 0.005, 0.005, 0.005), (-0.5, 1.42, 0.04, 0.04, 0.04), (0.35, 1.42, 0.04, 0.04, 0.04), (0.36, 1.42, 0.005, 0.005, 0.005)], seg=8, power=2, y0=side * 1.99)

# ---- The tail: a big swept fin, a tailplane low on it, a fairing for the tail rotor's gearbox.
plate("body", "olive2", [(5.7, 2.42), (6.5, 3.3), (6.95, 3.72), (7.45, 3.74), (7.28, 3.2), (7.18, 2.42)], 0.05)
plate("body", "olive2", [(5.75, 2.25), (6.2, 1.5), (6.5, 0.72), (6.95, 0.72), (7.1, 2.3)], 0.05)
for z in (2.95, 3.3):
    box("body", "dark", 6.2, 7.3, -0.055, 0.055, z - 0.006, z + 0.006)
wing("body", "olive2", 6.45, 7.15, 6.75, 7.2, 0.05, 1.05, 2.1, 2.14, 0.03)
loft("body", "dark", [(6.42, 2.7, 0.02, 0.02, 0.02), (6.48, 2.7, 0.12, 0.12, 0.12), (6.9, 2.7, 0.12, 0.12, 0.12), (6.98, 2.7, 0.02, 0.02, 0.02)], seg=14, power=2)
loft("body", "amber", [(7.44, 3.74, 0.005, 0.005, 0.005), (7.46, 3.74, 0.03, 0.03, 0.03), (7.48, 3.74, 0.005, 0.005, 0.005)], seg=8, power=2)
for side, mat in ((-1, "tail"), (1, "lens")):
    rod("body", mat, (0.2, side * 2.0, 1.56), (0.28, side * 2.0, 1.56), 0.03, seg=8)

# ---- Gear: two main wheels on struts well forward, a little one under the tail.
for side in (-1, 1):
    y = side * 1.22
    wheel(y, -1.5, 0.34, 0.34, 0.1)
    path("body", "dark", [(-1.5, y - side * 0.1, 0.42), (-1.25, side * 0.78, 0.78)], 0.05, seg=10)
    path("body", "metal", [(-1.5, y - side * 0.1, 0.42), (-1.65, side * 0.8, 0.9)], 0.035, seg=8)
    path("body", "dark", [(-1.5, y - side * 0.14, 0.3), (-0.9, side * 0.72, 0.68)], 0.03, seg=8)
    box("body", "metal", -2.25, -1.85, y - 0.12, y + 0.12, 0.55, 0.59)
    path("body", "dark", [(-2.1, side * 0.78, 0.72), (-2.05, y, 0.57)], 0.015, seg=5)
wheel(0, 6.72, 0.2, 0.2, 0.07)
for side in (-1, 1):
    path("body", "dark", [(6.72, side * 0.1, 0.2), (6.72, side * 0.08, 0.8)], 0.035, seg=8)
loft("body", "dark", [(6.5, 0.82, 0.08, 0.08, 0.1), (6.95, 0.82, 0.08, 0.08, 0.1)], seg=10, power=3)

# ---- The main rotor: a heavy head, pitch links, and four long wide blades, white at the tips.
loft("rotor", "metal", [(-0.26, 3.2, 0.01, 0.01, 0.01), (-0.24, 3.2, 0.12, 0.08, 0.22), (0.24, 3.2, 0.12, 0.08, 0.22), (0.26, 3.2, 0.01, 0.01, 0.01)], seg=16, power=2)
for k in range(4):
    a = math.pi * 0.5 * k
    c, s = math.cos(a), math.sin(a)
    def turned(x, y, z):
        return (x * c - y * s, x * s + y * c, z)
    rod("rotor", "metal", turned(0.15, 0, 3.2), turned(0.72, 0, 3.17), 0.06, seg=10)
    rod("rotor", "dark", turned(0.34, 0, 3.2), turned(0.72, 0, 3.17), 0.075, seg=10)
    rod("rotor", "dark", turned(0.25, 0.12, 2.82), turned(0.46, 0.12, 3.15), 0.014, seg=5)
    stations = [(0.72, 0.13), (1.3, 0.24), (4.9, 0.24), (5.3, 0.21), (5.45, 0.08)]
    for i in range(len(stations) - 1):
        x0, w0 = stations[i]
        x1, w1 = stations[i + 1]
        corners = [turned(x0, -w0, 3.145), turned(x1, -w1, 3.145), turned(x0, w0 * 0.6, 3.16), turned(x1, w1 * 0.6, 3.16),
                   turned(x0, -w0, 3.165), turned(x1, -w1, 3.165), turned(x0, w0 * 0.6, 3.19), turned(x1, w1 * 0.6, 3.19)]
        V, F = box_mesh(corners)
        add("rotor", "white" if i >= 2 else "black", V, F)

# ---- The tail rotor, on the left of the fin at x = 6.7, z = 2.7: four blades, in two pairs set at an angle.
loft("tail", "dark", [(6.6, 2.7, 0.02, 0.02, 0.02), (6.62, 2.7, 0.09, 0.09, 0.09), (6.78, 2.7, 0.09, 0.09, 0.09), (6.8, 2.7, 0.02, 0.02, 0.02)], seg=12, power=2, y0=0.22)
rod("tail", "metal", (6.7, 0.1, 2.7), (6.7, 0.3, 2.7), 0.045, seg=8)
box("tail", "black", 6.63, 6.77, 0.2, 0.25, 1.8, 3.6)
box("tail", "black", 5.8, 7.6, 0.25, 0.3, 2.63, 2.77)
box("tail", "white", 6.63, 6.77, 0.195, 0.255, 1.8, 2.0)
box("tail", "white", 6.63, 6.77, 0.195, 0.255, 3.4, 3.6)
box("tail", "white", 5.8, 6.0, 0.245, 0.305, 2.63, 2.77)
box("tail", "white", 7.4, 7.6, 0.245, 0.305, 2.63, 2.77)

with open(sys.argv[1] if len(sys.argv) > 1 else "assets/heli.mesh", "wb") as fp:
    fp.write(b"XMSH")
    fp.write(struct.pack("<I", len(parts)))
    for name, group, mat, v, n, f in parts:
        fp.write(name.encode()[:39].ljust(40, b"\0"))
        fp.write(group.encode()[:23].ljust(24, b"\0"))
        fp.write(struct.pack("<III", mat, len(v), len(f)))
        fp.write(np.asarray(v, np.float32).tobytes())
        fp.write(np.asarray(n, np.float32).tobytes())
        fp.write(np.asarray(f, np.uint32).tobytes())
print("heli: %d parts, %d tris" % (len(parts), sum(len(p[5]) for p in parts)))
