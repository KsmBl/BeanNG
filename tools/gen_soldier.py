#!/usr/bin/env python3
"""Generates the soldier: python3 gen_soldier.py out.mesh [cloth material] [darker cloth material].

Coordinates as for the vehicles: x rearwards, y to the left, z up, ground at z = 0, standing with arms down.
Each group is one rigid part of the body; the game moves it by four nodes that go with it. The weapons
stand at the right hand, pointing forward, and only the one in use is brought up from underground.
"""
import math
import struct
import sys

import numpy as np

MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust",
        "olive", "olive2", "track", "pearl", "carbon", "accent", "sand", "skin"]
M = {n: i for i, n in enumerate(MATS)}
CLOTH = sys.argv[2] if len(sys.argv) > 2 else "olive"
CLOTH2 = sys.argv[3] if len(sys.argv) > 3 else "olive2"
SKIN = "skin"
parts = []


def smooth_normals(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def add(group, mat, v, f):
    v = np.asarray(v, float)
    f = np.asarray(f, int)
    parts.append(("%s_%s_%d" % (group, mat, len(parts)), group, M[mat], v, smooth_normals(v, f), f))


def body(group, mat, rings, seg=20):
    """A smooth closed body through rings: (centre x, y, z, half depth in x, half width in y), stacked along z or any way."""
    v = []
    for cx, cy, cz, rx, ry in rings:
        for j in range(seg):
            a = 2 * math.pi * j / seg
            v.append((cx + rx * math.cos(a), cy + ry * math.sin(a), cz))
    f = []
    n = len(rings)
    for i in range(n - 1):
        for j in range(seg):
            A, B = i * seg + j, i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, B, D], [A, D, C]]
    base = len(v)
    v.append(np.mean(v[:seg], axis=0))
    v.append(np.mean(v[-seg - 1:-1], axis=0))
    for j in range(seg):
        f.append([base, (j + 1) % seg, j])
        f.append([base + 1, (n - 1) * seg + j, (n - 1) * seg + (j + 1) % seg])
    add(group, mat, v, f)


def box(group, mat, c, s):
    c = np.asarray(c, float)
    h = np.asarray(s, float) * 0.5
    v, f = [], []
    # One set of vertices per face, so that its edges stay crisp.
    for axis in range(3):
        for sg in (-1, 1):
            u, w = [(1, 2), (2, 0), (0, 1)][axis]
            base = len(v)
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = np.zeros(3)
                p[axis] = sg
                p[u] = a
                p[w] = b
                v.append(c + p * h)
            f += [[base, base + 1, base + 2], [base, base + 2, base + 3]] if sg > 0 else [[base, base + 2, base + 1], [base, base + 3, base + 2]]
    add(group, mat, v, f)


def rod(group, mat, a, b, prof, seg=14):
    """Turned between a and b: prof is (t along it, radius)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax = b - a
    L = np.linalg.norm(ax)
    ax = ax / L
    e1 = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(e1) < 1e-3:
        e1 = np.cross(ax, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    prof = [(prof[0][0], 1e-4)] + list(prof) + [(prof[-1][0], 1e-4)]
    v, f = [], []
    for t, r in prof:
        for j in range(seg):
            q = 2 * math.pi * j / seg
            v.append(a + ax * (t * L) + (e1 * math.cos(q) + e2 * math.sin(q)) * r)
    for i in range(len(prof) - 1):
        for j in range(seg):
            A, B = i * seg + j, i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, C, D], [A, D, B]]
    add(group, mat, v, f)


# Forward is -x in this frame. Heights, in metres, for a man of 1.78 m.
HIP, KNEE, ANKLE, SHOULDER, ELBOW, WRIST = 0.93, 0.50, 0.09, 1.42, 1.13, 0.87
HIP_Y, SHOULDER_Y = 0.10, 0.215

# Pelvis and belt.
body("pelvis", CLOTH2, [(0, 0, 0.84, 0.115, 0.15), (0, 0, 0.92, 0.125, 0.165), (0, 0, 1.03, 0.115, 0.155)])
body("pelvis", "dark", [(0, 0, 0.99, 0.128, 0.168), (0, 0, 1.035, 0.128, 0.168)])
for sg in (-1, 1):
    box("pelvis", "dark", (-0.03, sg * 0.175, 0.95), (0.13, 0.05, 0.13))  # pouches on the belt

# Trunk: jacket, with the plate carrier over it, and the pack behind.
body("torso", CLOTH, [(0, 0, 1.02, 0.115, 0.155), (0, 0, 1.2, 0.125, 0.17), (0, 0, 1.38, 0.125, 0.195), (0, 0, 1.46, 0.09, 0.14), (0, 0, 1.5, 0.05, 0.06)])
body("torso", CLOTH2, [(0, 0, 1.06, 0.14, 0.165), (0, 0, 1.22, 0.15, 0.18), (0, 0, 1.39, 0.145, 0.17), (0, 0, 1.42, 0.11, 0.13)])
for k, y in enumerate((-0.09, 0.0, 0.09)):
    box("torso", "dark", (-0.155, y, 1.14), (0.04, 0.075, 0.13))  # magazines across the chest
box("torso", "dark", (0.2, 0, 1.25), (0.15, 0.27, 0.36))        # the pack
box("torso", CLOTH2, (0.285, 0, 1.2), (0.03, 0.22, 0.2))
for sg in (-1, 1):
    rod("torso", "dark", (-0.09, sg * 0.1, 1.41), (-0.09, sg * 0.1, 1.2), [(0, 0.022), (1, 0.022)], 8)  # straps

# Head: neck, face, helmet with its rim, and goggles on the front of it.
HEAD = "head"
rod(HEAD, SKIN, (0, 0, 1.47), (0, 0, 1.56), [(0, 0.055), (1, 0.052)], 12)
body(HEAD, SKIN, [(-0.008, 0, 1.53, 0.075, 0.066), (-0.012, 0, 1.58, 0.092, 0.08), (-0.012, 0, 1.64, 0.098, 0.082), (-0.005, 0, 1.7, 0.09, 0.075)])
body(HEAD, CLOTH2, [(0, 0, 1.635, 0.127, 0.115), (0, 0, 1.655, 0.128, 0.116), (0, 0, 1.69, 0.122, 0.11), (0, 0, 1.735, 0.103, 0.093), (0, 0, 1.77, 0.07, 0.063), (0, 0, 1.785, 0.03, 0.028)], 22)
body(HEAD, "dark", [(0, 0, 1.632, 0.133, 0.12), (0, 0, 1.645, 0.133, 0.12)], 22)
box(HEAD, "black", (-0.098, 0, 1.615), (0.018, 0.135, 0.042))   # the eyes, behind goggles
box(HEAD, "dark", (-0.125, 0, 1.685), (0.03, 0.12, 0.03))       # goggles pushed up, or a mount

for name, sg in (("l", 1), ("r", -1)):
    y = sg * HIP_Y
    t, s, ua, fa = "thigh_" + name, "shin_" + name, "upper_" + name, "fore_" + name
    # Legs, with a knee pad and a proper boot with toe in front.
    rod(t, CLOTH, (0, y, HIP + 0.03), (0, y, KNEE), [(0, 0.07), (0.15, 0.092), (0.55, 0.08), (0.98, 0.058), (1, 0.05)])
    box(t, "dark", (-0.005, y + sg * 0.078, 0.74), (0.12, 0.05, 0.1))  # thigh pouch
    rod(s, CLOTH, (0, y, KNEE + 0.02), (0, y, ANKLE + 0.06), [(0, 0.056), (0.3, 0.064), (1, 0.043)])
    body(s, CLOTH2, [(-0.02, y, KNEE - 0.055, 0.055, 0.058), (-0.03, y, KNEE, 0.062, 0.062), (-0.02, y, KNEE + 0.055, 0.055, 0.058)], 12)
    body(s, "dark", [(-0.035, y, 0.0, 0.135, 0.058), (-0.035, y, 0.05, 0.135, 0.058), (-0.015, y, 0.09, 0.09, 0.053), (0, y, 0.19, 0.056, 0.05)], 14)
    # Arms: sleeve, a pad at the shoulder, glove.
    yS = sg * SHOULDER_Y
    rod(ua, CLOTH, (0, yS, SHOULDER + 0.02), (0, yS, ELBOW), [(0, 0.04), (0.12, 0.06), (0.6, 0.052), (1, 0.043)])
    rod(fa, CLOTH, (0, yS, ELBOW + 0.01), (0, yS, WRIST + 0.03), [(0, 0.043), (0.35, 0.046), (1, 0.034)])
    rod(fa, "dark", (0, yS, WRIST + 0.04), (0, yS, WRIST - 0.09), [(0, 0.036), (0.45, 0.042), (0.85, 0.036), (1, 0.02)], 10)

# ---- Weapons. Grip in the right hand at (0, -SHOULDER_Y, WRIST - 0.04), the muzzle forward.
GRIP = np.array([0.0, -SHOULDER_Y, WRIST - 0.04])


def w(x, y=0.0, z=0.0):
    """A point relative to the grip: x forward, z up."""
    return GRIP + np.array([-x, y, z])


# Combat knife.
box("gun_knife", "metal", w(0.14, 0, 0.0), (0.2, 0.006, 0.032))
box("gun_knife", "black", w(0.0, 0, 0.0), (0.11, 0.024, 0.03))
# Pistol.
box("gun_pistol", "black", w(0.06, 0, 0.055), (0.19, 0.028, 0.034))
box("gun_pistol", "black", w(0.0, 0, 0.0), (0.04, 0.028, 0.09))
# Assault rifle.
box("gun_ar", "black", w(0.08, 0, 0.05), (0.42, 0.045, 0.075))
box("gun_ar", "black", w(0.0, 0, 0.0), (0.04, 0.03, 0.09))
box("gun_ar", "dark", w(0.13, 0, -0.03), (0.06, 0.03, 0.14))    # magazine
box("gun_ar", "black", w(-0.24, 0, 0.03), (0.2, 0.035, 0.08))   # stock
rod("gun_ar", "black", w(0.28, 0, 0.06), w(0.62, 0, 0.06), [(0, 0.012), (1, 0.011)], 8)
box("gun_ar", "dark", w(0.28, 0, 0.06), (0.2, 0.04, 0.05))      # handguard
# The big rifle: as long as a man, with a scope, a muzzle brake and a bipod.
box("gun_sniper", "dark", w(0.15, 0, 0.05), (0.62, 0.055, 0.09))
box("gun_sniper", "black", w(0.0, 0, 0.0), (0.045, 0.03, 0.09))
box("gun_sniper", "dark", w(-0.28, 0, 0.03), (0.28, 0.04, 0.1))
rod("gun_sniper", "black", w(0.45, 0, 0.065), w(1.25, 0, 0.065), [(0, 0.021), (1, 0.019)], 10)
rod("gun_sniper", "dark", w(1.25, 0, 0.065), w(1.37, 0, 0.065), [(0, 0.034), (1, 0.034)], 10)
rod("gun_sniper", "black", w(-0.05, 0, 0.135), w(0.28, 0, 0.135), [(0, 0.03), (0.2, 0.022), (0.8, 0.022), (1, 0.034)], 10)
for sg in (-1, 1):
    rod("gun_sniper", "dark", w(0.5, 0, 0.05), w(0.5, sg * 0.14, -0.28), [(0, 0.008), (1, 0.008)], 6)
# Rocket launcher: a tube on the shoulder, with the warhead showing in front.
GZ = 0.17
rod("gun_rpg", "olive2", w(-0.55, 0, GZ), w(0.5, 0, GZ), [(0, 0.05), (0.08, 0.042), (0.92, 0.042), (1, 0.046)], 14)
rod("gun_rpg", "dark", w(0.5, 0, GZ), w(0.82, 0, GZ), [(0, 0.03), (0.3, 0.05), (0.75, 0.05), (1, 0.008)], 14)
box("gun_rpg", "black", w(0.0, 0, 0.0), (0.04, 0.03, 0.15))


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
    write(sys.argv[1] if len(sys.argv) > 1 else "assets/soldier.mesh")
