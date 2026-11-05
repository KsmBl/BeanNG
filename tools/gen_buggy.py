#!/usr/bin/env python3
"""Generates the tracked buggy: python3 gen_buggy.py out.mesh

Coordinates as for the vehicles: x rearwards, y to the left, z up, tracks on the ground at z = 0, the middle at x = 0.
Open and low: a tube frame on two rubber tracks, a roll cage over the seat, an engine behind it and a machine gun on a
ring on top of the cage. Groups as on the tanks: the hull, the ring with the gun's mount, the gun, and the net that can be thrown over it, and whoever is driving.
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


def add(group, mat, v, f):
    v = np.asarray(v, float)
    f = np.asarray(f, int)
    parts.append(("%s_%s_%d" % (group, mat, len(parts)), group, M[mat], v, normals(v, f), f))


def box_faces(c):
    q = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    V, F = [], []
    for a, b, cc, d in q:
        base = len(V)
        V += [c[a], c[b], c[cc], c[d]]
        F += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
    return V, F


def box(group, mat, x0, x1, y0, y1, z0, z1):
    V, F = box_faces([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)])
    add(group, mat, V, F)


def wedge(group, mat, x0, x1, y0, y1, z00, z01, z10, z11):
    """A box whose bottom and top run from z00, z01 at x0 to z10, z11 at x1."""
    c = [(x0, y0, z00), (x1, y0, z10), (x0, y1, z00), (x1, y1, z10), (x0, y0, z01), (x1, y0, z11), (x0, y1, z01), (x1, y1, z11)]
    V, F = box_faces(c)
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


def path(group, mat, pts, r, seg=8):
    for a, b in zip(pts, pts[1:]):
        rod(group, mat, a, b, r, seg)
    # A ball at each bend, so that the tube stays closed there.
    for p in pts[1:-1]:
        rod(group, mat, (p[0], p[1] - r, p[2]), (p[0], p[1] + r, p[2]), r, seg)


def wheel(x, y, z, r, width, mat="rubber"):
    rod("hull", mat, (x, y - width, z), (x, y + width, z), r, seg=18)
    rod("hull", "alloy", (x, y - width - 0.01, z), (x, y + width + 0.01, z), r * 0.55, seg=12)
    rod("hull", "dark", (x, y - width - 0.02, z), (x, y + width + 0.02, z), r * 0.16, seg=8)


# ---- The tracks and wheels are the game's, as on any tank: they turn.
TRACK_Y = 0.92
for side in (-1, 1):
    y = side * TRACK_Y
    # A mudguard over the top of the track.
    # Just a short fender over the idler: the rest of the track lies open to see.
    wedge("hull", "olive", -2.55, -1.75, y - 0.3, y + 0.3, 0.84, 0.88, 1.09, 1.13)

# ---- The tub: a low open body between the tracks, with a sloping nose.
wedge("hull", "olive2", -2.3, -1.2, -0.62, 0.62, 0.32, 0.42, 0.28, 0.82)
box("hull", "olive2", -1.2, 1.5, -0.62, 0.62, 0.26, 0.5)
box("hull", "olive", -1.2, 0.75, -0.66, -0.62, 0.5, 0.82)
box("hull", "olive", -1.2, 0.75, 0.62, 0.66, 0.5, 0.82)
box("hull", "olive", -1.25, -1.18, -0.62, 0.62, 0.5, 0.88)
# Headlights behind a guard on the nose.
for y in (-0.42, 0.42):
    rod("hull", "lens", (-1.68, y, 0.83), (-1.71, y, 0.83), 0.09, seg=14)
    rod("hull", "chrome", (-1.62, y, 0.83), (-1.68, y, 0.83), 0.105, seg=14)
path("hull", "dark", [(-2.32, -0.6, 0.36), (-2.42, -0.5, 0.74), (-2.42, 0.5, 0.74), (-2.32, 0.6, 0.36)], 0.03)
# Seat, wheel-levers and a dash.
box("hull", "black", 0.1, 0.55, -0.27, 0.27, 0.5, 0.62)
wedge("hull", "black", 0.48, 0.66, -0.27, 0.27, 0.6, 0.62, 0.6, 1.25)
box("hull", "dark", -0.98, -0.82, -0.42, 0.42, 0.72, 0.92)
for y in (-0.16, 0.16):
    rod("hull", "dark", (-0.25, y, 0.55), (-0.45, y, 0.98), 0.016, seg=6)
    rod("hull", "rubber", (-0.46, y, 0.96), (-0.49, y, 1.08), 0.024, seg=6)
# A harness over the seat.
for y in (-0.14, 0.14):
    rod("hull", "accent", (0.47, y, 1.15), (0.2, y * 0.3, 0.68), 0.012, seg=4)

# ---- The engine out at the back, in the open, with its exhaust, and the fuel behind it.
box("hull", "dark", 0.75, 1.5, -0.42, 0.42, 0.5, 0.88)
for k in range(5):
    x = 0.84 + 0.13 * k
    box("hull", "metal", x, x + 0.05, -0.44, 0.44, 0.88, 0.94)
rod("hull", "metal", (1.0, 0.3, 0.98), (1.0, 0.3, 1.14), 0.1, seg=12)
for side in (-1, 1):
    path("hull", "metal", [(1.45, side * 0.2, 0.7), (1.72, side * 0.28, 0.72), (2.15, side * 0.3, 0.9)], 0.04)
    rod("hull", "black", (2.13, side * 0.3, 0.88), (2.17, side * 0.3, 0.92), 0.03, seg=8)
box("hull", "dark", 1.5, 1.75, -0.55, 0.55, 0.42, 0.72)

# ---- The roll cage: bent tube all over the seat, the first thing one sees.
B, F, H = 0.72, -0.95, 1.72
for side in (-1, 1):
    y0, y1 = side * 0.62, side * 0.5
    # Down from the roof to the sills, front and back, and a brace to the tail.
    path("hull", "white", [(F - 0.05, y0, 0.82), (F + 0.3, y1, H), (B - 0.1, y1, H), (B + 0.05, y0, 0.82)], 0.035)
    rod("hull", "white", (B - 0.1, y1, H), (1.58, y0 * 0.9, 0.84), 0.028)
    rod("hull", "white", (F - 0.05, y0, 0.82), (F - 0.75, y0 * 0.82, 0.45), 0.028)
    # A cross on the door side.
    rod("hull", "white", (F + 0.1, y0, 1.05), (B, y0, 1.05), 0.022)
    rod("hull", "white", (F + 0.3, y1, H), (B + 0.02, y0, 1.05), 0.02)
for x in (F + 0.3, B - 0.1):
    rod("hull", "white", (x, -0.5, H), (x, 0.5, H), 0.035)
rod("hull", "white", (B - 0.1, -0.5, H), (F + 0.3, 0.5, H), 0.022)
rod("hull", "white", (B + 0.02, -0.6, 1.05), (B + 0.02, 0.6, 1.05), 0.028)
# Two lamps on the roof bar.
for y in (-0.3, 0.3):
    rod("hull", "lens", (F + 0.24, y, H + 0.06), (F + 0.2, y, H + 0.06), 0.055, seg=10)
    rod("hull", "dark", (F + 0.3, y, H + 0.06), (F + 0.24, y, H + 0.06), 0.065, seg=10)
# An aerial and a small flag of nothing: just the aerial.
rod("hull", "black", (B + 0.05, 0.56, 0.95), (B + 0.3, 0.56, 2.6), 0.007, seg=4)

# ---- The gun: on a ring on the roof, with its ammunition box and a shield. All of it turns.
rod("turret", "dark", (-0.1, 0, H + 0.02), (-0.1, 0, H + 0.13), 0.1, seg=14)
rod("turret", "dark", (-0.1, 0, H + 0.13), (-0.1, 0, H + 0.28), 0.03, seg=8)
box("gun", "dark", -0.45, 0.25, -0.05, 0.05, H + 0.26, H + 0.38)
rod("gun", "metal", (-0.45, 0, H + 0.32), (-1.32, 0, H + 0.32), 0.022, seg=8)
rod("gun", "dark", (-0.55, 0, H + 0.32), (-0.95, 0, H + 0.32), 0.035, seg=10)
rod("gun", "dark", (-1.32, 0, H + 0.32), (-1.42, 0, H + 0.32), 0.04, seg=10)
box("gun", "olive", -0.3, 0.05, 0.05, 0.23, H + 0.2, H + 0.37)
box("gun", "dark", 0.22, 0.34, -0.012, 0.012, H + 0.22, H + 0.33)
for y in (-0.05, 0.05):
    rod("gun", "dark", (0.25, y, H + 0.3), (0.42, y, H + 0.3), 0.012, seg=5)
box("gun", "olive2", -0.66, -0.62, -0.26, 0.26, H + 0.14, H + 0.55)

# ---- Whoever sits in it, in the open under the cage: there only while there is someone.
box("driver", "olive", 0.18, 0.5, -0.2, 0.2, 0.62, 1.12)
box("driver", "olive2", 0.1, 0.42, -0.18, 0.18, 1.0, 1.14)
box("driver", "olive", -0.28, 0.2, -0.2, -0.06, 0.58, 0.72)
box("driver", "olive", -0.28, 0.2, 0.06, 0.2, 0.58, 0.72)
for y in (-0.27, 0.27):
    rod("driver", "olive", (0.34, y * 0.85, 1.1), (0.08, y * 0.75, 0.92), 0.05, seg=8)
    rod("driver", "olive", (0.08, y * 0.75, 0.92), (-0.38, y * 0.6, 1.04), 0.043, seg=8)
    rod("driver", "black", (-0.38, y * 0.6, 1.04), (-0.45, y * 0.58, 1.05), 0.05, seg=8)
rod("driver", "skin", (0.3, 0, 1.12), (0.3, 0, 1.2), 0.055, seg=10)
rod("driver", "skin", (0.28, 0, 1.2), (0.28, 0, 1.38), 0.105, seg=12)
rod("driver", "olive2", (0.3, 0, 1.3), (0.3, 0, 1.43), 0.128, seg=12)
box("driver", "dark", 0.14, 0.2, -0.08, 0.08, 1.26, 1.32)

# ---- The camouflage net, over the whole of it and down to the ground.
NET_N = 28
_v, _f = [], []
for i in range(NET_N + 1):
    for j in range(NET_N + 1):
        a, b = i / NET_N * 2 - 1, j / NET_N * 2 - 1
        h = 2.3 * max(0.0, 1.0 - (abs(a) ** 2.4 + abs(b) ** 2.4) ** 0.8) ** 0.55
        _v.append((a * 2.9, b * 1.55, 0.05 + h + 0.03 * math.sin(i * 1.9) * math.cos(j * 2.3)))
for i in range(NET_N):
    for j in range(NET_N):
        if (i * 7 + j * 3) % 11 == 0:
            continue
        p = i * (NET_N + 1) + j
        _f += [[p, p + NET_N + 1, p + 1], [p + 1, p + NET_N + 1, p + NET_N + 2]]
add("net", "olive2", _v, _f)

with open(sys.argv[1] if len(sys.argv) > 1 else "assets/buggy.mesh", "wb") as fp:
    fp.write(b"XMSH")
    fp.write(struct.pack("<I", len(parts)))
    for name, group, mat, v, n, f in parts:
        fp.write(name.encode()[:39].ljust(40, b"\0"))
        fp.write(group.encode()[:23].ljust(24, b"\0"))
        fp.write(struct.pack("<III", mat, len(v), len(f)))
        fp.write(np.asarray(v, np.float32).tobytes())
        fp.write(np.asarray(n, np.float32).tobytes())
        fp.write(np.asarray(f, np.uint32).tobytes())
print("buggy: %d parts, %d tris" % (len(parts), sum(len(p[5]) for p in parts)))
