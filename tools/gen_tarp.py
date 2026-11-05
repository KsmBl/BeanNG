#!/usr/bin/env python3
"""Generates the tarpaulin for cars: python3 gen_tarp.py out.mesh

One sheet thrown over a car of 4.6 by 2 metres and 1.6 high, down to the ground at the edges, in the colours of what lies
about. Centred on the origin with the ground at z = 0. The game stretches it to the car it covers.
"""
import math
import struct
import sys

import numpy as np

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent", "sand", "skin", "grass", "grass2", "dirt", "leaf"]
M = {n: i for i, n in enumerate(MATS)}
seed = 7171


def rnd():
    global seed
    seed = (seed * 1664525 + 1013904223) % (1 << 32)
    return (seed >> 16) / 65535.0


def height(x, y):
    # Over the roof, lower over bonnet and boot, and down to the ground all round.
    roof = 1.62 - 0.55 * min(1.0, max(0.0, abs(x) - 0.9) / 1.3) ** 2
    edge = max(abs(y) - 0.95, 0.0) / 0.35 + max(abs(x) - 2.25, 0.0) / 0.35
    return max(0.04, roof * (1.0 - min(edge * edge, 1.0)))


nx, ny = 30, 18
corners = {}
for i in range(nx + 1):
    for j in range(ny + 1):
        x, y = -2.7 + 5.4 * i / nx, -1.35 + 2.7 * j / ny
        corners[(i, j)] = (x + 0.03 * (rnd() - 0.5), y + 0.03 * (rnd() - 0.5), height(x, y) + 0.06 * (rnd() - 0.5))

cells = {"grass": [], "grass2": [], "dirt": []}
for i in range(nx):
    for j in range(ny):
        u = math.sin(i * 0.61 + j * 0.27) + math.sin(i * 0.23 - j * 0.71 + 1.7) + 0.7 * (rnd() - 0.5)
        cells["grass" if u < 0.35 else ("grass2" if u < 1.15 else "dirt")].append((i, j))

parts = []
for mat, cell in cells.items():
    v, f = [], []
    for i, j in cell:
        base = len(v)
        v += [corners[(i, j)], corners[(i + 1, j)], corners[(i + 1, j + 1)], corners[(i, j + 1)]]
        f += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
    v = np.asarray(v, float)
    f = np.asarray(f, int)
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    parts.append(("net_" + mat, "net", M[mat], v, n, f))

with open(sys.argv[1] if len(sys.argv) > 1 else "assets/tarp.mesh", "wb") as fp:
    fp.write(b"XMSH")
    fp.write(struct.pack("<I", len(parts)))
    for name, group, mat, v, n, f in parts:
        fp.write(name.encode()[:39].ljust(40, b"\0"))
        fp.write(group.encode()[:23].ljust(24, b"\0"))
        fp.write(struct.pack("<III", mat, len(v), len(f)))
        fp.write(np.asarray(v, np.float32).tobytes())
        fp.write(np.asarray(n, np.float32).tobytes())
        fp.write(np.asarray(f, np.uint32).tobytes())
