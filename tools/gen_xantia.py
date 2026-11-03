#!/usr/bin/env python3
"""Generates the Citroen Xantia mesh (assets/xantia.mesh).

The outer skin comes from ONE smooth surface, defined by cross-sections along the car. Every body
panel is a 2D outline in a "chart" (side view, top view, or a cylinder wrapped around the nose or
tail) that gets triangulated and projected onto that surface. So all panels fit together perfectly
and real shut lines with rolled edges appear between them.

Coordinates: x from the nose rearwards, y to the left, z up, metres. Ground at z = 0, normal ride.
"""
import math
import struct
import sys

import numpy as np
from scipy.interpolate import PchipInterpolator as Pchip, RegularGridInterpolator
from scipy.ndimage import gaussian_filter
from scipy.spatial import Delaunay
from shapely.geometry import LineString, Point, Polygon, box
from shapely.affinity import scale
from shapely.ops import unary_union

# ---- Materials, shared with src/carmesh.h.
MATS = ["paint", "glass", "black", "liner", "chrome", "lens", "tail", "amber", "rubber", "alloy",
        "fabric", "carpet", "plate", "metal", "alu", "lhm", "dark", "disc", "white", "reflector", "rust", "olive", "olive2", "track", "pearl", "carbon", "accent"]
M = {n: i for i, n in enumerate(MATS)}


def P(xs, ys):
    return Pchip(np.array(xs, float), np.array(ys, float), extrapolate=True)


# ---- Profile curves along the car. x measured from the nose.
ztop_c = P([0.02, 0.10, 0.28, 0.60, 0.90, 1.20, 1.33, 1.60, 1.88, 2.05, 2.30, 2.70, 3.00, 3.20, 3.55, 3.90, 3.98, 4.20, 4.40, 4.47, 4.52],
           [0.60, 0.660, 0.72, 0.785, 0.835, 0.895, 0.935, 1.095, 1.285, 1.360, 1.398, 1.405, 1.380, 1.335, 1.205, 1.090, 1.072, 1.058, 1.056, 1.030, 0.96])
zbelt_c = P([0.00, 0.10, 0.35, 0.70, 1.05, 1.35, 1.70, 2.30, 2.90, 3.40, 3.90, 4.30, 4.52],
            [0.54, 0.585, 0.665, 0.735, 0.790, 0.850, 0.895, 0.925, 0.945, 0.975, 0.995, 1.000, 0.93])
zrail_c = P([0.00, 0.28, 0.60, 0.90, 1.20, 1.33, 1.60, 1.90, 2.05, 2.30, 2.90, 3.20, 3.50, 3.80, 4.00, 4.20, 4.40, 4.52],
            [0.57, 0.690, 0.750, 0.800, 0.860, 0.905, 1.060, 1.255, 1.300, 1.325, 1.320, 1.285, 1.195, 1.115, 1.070, 1.045, 1.040, 0.95])
yrail_c = P([0.00, 0.20, 0.60, 1.10, 1.34, 1.70, 2.05, 2.70, 3.20, 3.60, 4.00, 4.35, 4.52],
            [0.66, 0.70, 0.76, 0.79, 0.80, 0.74, 0.665, 0.655, 0.665, 0.70, 0.745, 0.72, 0.66])
w_c = P([0.00, 0.60, 1.20, 3.40, 4.00, 4.52], [0.86, 0.87, 0.8775, 0.8775, 0.87, 0.84])
zb_c = P([0.00, 0.30, 1.00, 1.30, 3.20, 3.80, 4.52], [0.30, 0.25, 0.215, 0.205, 0.205, 0.235, 0.33])
# Front and rear face of the car seen from the side: x position as a function of height.
xf_c = P([0.20, 0.28, 0.40, 0.50, 0.56, 0.62, 0.70, 1.00], [0.14, 0.05, 0.005, 0.0, 0.015, 0.06, 0.14, 0.60])
xr_c = P([0.20, 0.30, 0.42, 0.55, 0.70, 0.85, 0.98, 1.06, 1.20], [4.30, 4.45, 4.515, 4.52, 4.51, 4.50, 4.475, 4.43, 4.30])

NSEC = 1200
NZ = 650
XS = np.linspace(-0.02, 4.56, NSEC)
ZS = np.linspace(0.15, 1.45, NZ)
YS = np.linspace(0.0, 0.95, 420)


def section_polyline(x):
    W = float(w_c(x))
    zb = float(zb_c(x))
    zt = float(ztop_c(x))
    zr = float(zrail_c(x))
    zbelt = min(float(zbelt_c(x)), zr - 0.02)
    yr = float(yrail_c(x))
    z3 = min(0.66, zbelt - 0.12)
    z2 = min(0.42, z3 - 0.10)
    z1 = min(zb + 0.05, z2 - 0.05)
    pts = [(0.78 * W, zb), (0.94 * W, z1), (0.985 * W, z2), (W, z3), (0.992 * W, zbelt - 0.05),
           (0.968 * W, zbelt), (0.5 * (0.968 * W + yr) + 0.005, 0.5 * (zbelt + zr)), (yr, zr),
           (yr - 0.07, zr + 0.5 * (zt - zr) * 1.25), (yr * 0.5, zt - 0.010), (0.0, zt)]
    pts = np.array(pts)
    # Catmull-Rom through the points, with a mirrored point beyond the centreline for a flat crown.
    ext = np.vstack([pts[0] + (pts[0] - pts[1]), pts, [-pts[-2, 0], pts[-2, 1]]])
    out = []
    for i in range(1, len(ext) - 2):
        a, b, c, d = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for t in np.linspace(0, 1, 24, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(0.5 * (2 * b + (c - a) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (3 * b - a - 3 * c + d) * t3))
    out.append(pts[-1])
    return np.array(out)


print("building surface tables...", flush=True)
Yt = np.zeros((NSEC, NZ))
Zt = np.zeros((NSEC, len(YS)))
for i, x in enumerate(XS):
    poly = section_polyline(x)
    order = np.argsort(poly[:, 1])
    Yt[i] = np.interp(ZS, poly[order, 1], poly[order, 0], left=poly[order[0], 0], right=0.0)
    # Upper surface z(y) from the part of the section above the widest point.
    k = int(np.argmax(poly[:, 0]))
    up = poly[k:]
    o2 = np.argsort(up[:, 0])
    Zt[i] = np.interp(YS, up[o2, 0], up[o2, 1])
Yt = gaussian_filter(Yt, sigma=(5, 1.2))
Zt = gaussian_filter(Zt, sigma=(5, 2.0))
Y_i = RegularGridInterpolator((XS, ZS), Yt, bounds_error=False, fill_value=None)
Z_i = RegularGridInterpolator((XS, YS), Zt, bounds_error=False, fill_value=None)


def plan(x, z):
    """Rounds the plan view at the nose and tail."""
    xf = xf_c(z)
    xr = xr_c(z)
    df = np.clip((x - xf) / 0.42, 0.0, 1.0)
    dr = np.clip((xr - x) / 0.34, 0.0, 1.0)
    pf = (1.0 - (1.0 - df) ** 2.6) ** (1.0 / 2.6)
    pr = (1.0 - (1.0 - dr) ** 3.0) ** (1.0 / 3.0)
    return pf * pr


def Ysurf(x, z):
    return Y_i(np.stack([x, z], -1)) * plan(x, z)


def normal_implicit(p):
    """Gradient of the implicit surface |y| - Y(x, z) on the left side."""
    h = 0.006
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    dx = (Ysurf(x + h, z) - Ysurf(x - h, z)) / (2 * h)
    dz = (Ysurf(x, z + h) - Ysurf(x, z - h)) / (2 * h)
    n = np.stack([-dx, np.sign(y + 1e-9), -dz], -1)
    return n / np.linalg.norm(n, axis=1, keepdims=True)


class SideChart:
    """(x, z) -> side of the car, left half."""

    def map(self, uv):
        x, z = uv[:, 0], uv[:, 1]
        p = np.stack([x, Ysurf(x, z), z], -1)
        return p, normal_implicit(p)


def ztop_surface(x, y):
    z = Z_i(np.stack([x, np.abs(y)], -1))
    pl = np.clip(plan(x, z), 0.3, 1.0)
    return Z_i(np.stack([x, np.abs(y) / pl], -1))


class TopChart:
    """(x, y) -> top of the car."""

    def map(self, uv):
        x, y = uv[:, 0], uv[:, 1]
        z = ztop_surface(x, y)
        p = np.stack([x, y, z], -1)
        h = 0.008
        dx = (ztop_surface(x + h, y) - ztop_surface(x - h, y)) / (2 * h)
        dy = (ztop_surface(x, y + h) - ztop_surface(x, y - h)) / (2 * h)
        # At the centreline |y| is symmetric, so the numerical slope is exactly right there too.
        n = np.stack([-dx, -dy, np.ones_like(dx)], -1)
        return p, n / np.linalg.norm(n, axis=1, keepdims=True)


class CylChart:
    """(u, z), u = angle * radius, around a vertical axis: wraps the nose or the tail."""

    def __init__(self, cx, radius, front):
        self.cx, self.r, self.front = cx, radius, front

    def map(self, uv):
        ang = uv[:, 0] / self.r
        z = uv[:, 1]
        dx = -np.cos(ang) if self.front else np.cos(ang)
        dy = np.sin(ang)
        lo = np.zeros_like(ang)
        hi = np.full_like(ang, 2.6)
        for _ in range(38):
            m = 0.5 * (lo + hi)
            x = self.cx + dx * m
            y = np.abs(dy) * m
            inside = y < Ysurf(x, z)
            inside &= (x >= xf_c(z)) & (x <= xr_c(z))
            lo = np.where(inside, m, lo)
            hi = np.where(inside, hi, m)
        m = 0.5 * (lo + hi)
        x = self.cx + dx * m
        y = np.abs(dy) * m
        p = np.stack([x, y, z], -1)
        n = normal_implicit(p)
        # Near the centreline blend towards straight ahead, |y| would otherwise make a crease.
        axial = np.stack([dx, np.zeros_like(dx), np.zeros_like(dx)], -1)
        t = np.clip(np.abs(ang) / 0.25, 0, 1)[:, None]
        blend = axial * 0.85 + np.stack([np.zeros_like(dx), dy, np.zeros_like(dx)], -1) * 0.5 + np.stack([np.zeros_like(dx), np.zeros_like(dx), n[:, 2]], -1)
        n[:, 1] = n[:, 1]
        n = n * t + blend * (1 - t)
        n[:, 1] = np.abs(n[:, 1]) * np.sign(dy + 1e-12)
        p[:, 1] = dy * m
        return p, n / np.linalg.norm(n, axis=1, keepdims=True)


SIDE = SideChart()
TOP = TopChart()
NOSE = CylChart(0.92, 0.90, True)
TAIL = CylChart(3.88, 0.90, False)

# ---- Mesh output.
parts = []


def add_part(name, group, mat, v, n, f, flags=0):
    parts.append((name, group, M[mat], np.asarray(v, np.float32), np.asarray(n, np.float32), np.asarray(f, np.uint32), flags))


def mesh_normals(v, f):
    n = np.zeros_like(v)
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    fn = np.cross(b - a, c - a)
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def sample_ring(coords, step):
    coords = np.asarray(coords)
    if np.allclose(coords[0], coords[-1]):
        coords = coords[:-1]
    ring = LineString(np.vstack([coords, coords[0]]))
    n = max(int(ring.length / step), 6)
    pts = [ring.interpolate(i / n, normalized=True).coords[0] for i in range(n)]
    return np.array(pts)


def rings_of(poly):
    geoms = poly.geoms if hasattr(poly, "geoms") else [poly]
    out = []
    for g in geoms:
        out.append(g.exterior.coords)
        out.extend(i.coords for i in g.interiors)
    return out


def panel(name, group, mat, poly, chart, h=0.045, roll=0.008, flange=0.022, offset=0.0, mirror=False, edge=0.018, flip_group=None, flip_name=None):
    """Triangulates an outline in chart space and projects it onto the surface."""
    if poly.is_empty:
        return
    poly = poly.buffer(0)
    pts = []
    dist = []
    boundary_rings = []
    # Boundary and inset rings give a clean rolled edge, whatever the interior grid does.
    insets = [0.0]
    if roll > 0:
        insets += [roll * 0.25, roll * 0.6, roll, roll * 1.8]
    base = len(pts)
    for ins in insets:
        p2 = poly if ins == 0 else poly.buffer(-ins)
        if p2.is_empty:
            continue
        for coords in rings_of(p2):
            s = sample_ring(coords, edge)
            if ins == 0:
                boundary_rings.append((len(pts), len(s)))
            pts.extend(s.tolist())
            dist.extend([ins] * len(s))
    # Interior grid.
    margin = max(roll * 1.8, 0.001) + h * 0.4
    inner = poly.buffer(-margin)
    if not inner.is_empty:
        minx, miny, maxx, maxy = inner.bounds
        gx = np.arange(minx, maxx + h, h)
        gy = np.arange(miny, maxy + h, h)
        gg = np.array([(x, y) for x in gx for y in gy])
        from shapely import contains_xy
        keep = contains_xy(inner, gg[:, 0], gg[:, 1])
        gg = gg[keep]
        pts.extend(gg.tolist())
        dist.extend([1.0] * len(gg))
    pts = np.array(pts)
    dist = np.array(dist)
    tri = Delaunay(pts).simplices
    cen = pts[tri].mean(axis=1)
    from shapely import contains_xy
    tri = tri[contains_xy(poly, cen[:, 0], cen[:, 1])]

    P3, N3 = chart.map(pts)
    if offset != 0:
        P3 = P3 + N3 * offset
    # Rolled edge: the sheet curves away from the surface at its boundary.
    if roll > 0:
        d = np.clip(dist, 0, roll)
        depth = roll - np.sqrt(np.maximum(roll * roll - (roll - d) ** 2, 0))
        P3 = P3 - N3 * depth[:, None]
    # Consistent winding.
    a, b, c = P3[tri[:, 0]], P3[tri[:, 1]], P3[tri[:, 2]]
    fn = np.cross(b - a, c - a)
    wrong = (fn * N3[tri[:, 0]]).sum(1) < 0
    tri[wrong] = tri[wrong][:, [0, 2, 1]]

    V = P3
    F = tri
    NN = N3.copy()
    if roll > 0:
        mn = mesh_normals(V, F)
        near = dist < roll * 2.0
        NN[near] = mn[near]
    # Flange: the sheet folds inwards so that the gap between panels reads as a real, deep gap.
    if flange > 0:
        vs, ns, fs = [V], [NN], [F]
        count = len(V)
        for start, n in boundary_rings:
            top = np.arange(start, start + n)
            bot = np.arange(count, count + n)
            vs.append(V[top] - N3[top] * flange)
            ns.append(NN[top])
            nxt = np.roll(top, -1)
            nbt = np.roll(bot, -1)
            quad = np.vstack([np.stack([top, nxt, nbt], 1), np.stack([top, nbt, bot], 1)])
            # Face away from the panel: find which side of this ring lies outside the outline.
            m2 = 0.5 * (pts[top[0]] + pts[nxt[0]])
            e2 = pts[nxt[0]] - pts[top[0]]
            perp = np.array([-e2[1], e2[0]]) / (np.linalg.norm(e2) + 1e-12)
            if poly.contains(Point(*(m2 + perp * 5e-4))):
                perp = -perp
            pa, _ = chart.map(np.array([m2]))
            pb, _ = chart.map(np.array([m2 + perp * 0.002]))
            out3 = pb[0] - pa[0]
            q0 = V[quad[0, 0]]
            q1 = V[quad[0, 1]]
            q2 = (V[top[0]] - N3[top[0]] * flange)
            fn0 = np.cross(q1 - q0, q2 - q0)
            fs.append(quad if fn0 @ out3 > 0 else quad[:, [0, 2, 1]])
            count += n
        V, NN, F = np.vstack(vs), np.vstack(ns), np.vstack(fs)
    add_part(name, group, mat, V, NN, F)
    if mirror:
        V2 = V * np.array([1, -1, 1])
        N2 = NN * np.array([1, -1, 1])
        add_part(flip_name or name + "_r", flip_group or group, mat, V2, N2, F[:, [0, 2, 1]])


def poly_fn(fn, x0, x1, n=40):
    return [(x, float(fn(x))) for x in np.linspace(x0, x1, n)]


def outline(*segments):
    pts = []
    for s in segments:
        pts.extend(s)
    return Polygon(pts).buffer(0)


# ---- Wheel arches and side-view shut lines.
FA = Point(0.90, 0.312).buffer(0.368, 64)
RA = Point(3.64, 0.312).buffer(0.362, 64)
GAP = 0.0045


def rail(x):
    return float(zrail_c(x)) - 0.006


print("body panels...", flush=True)
# Window openings in side view.
win_f = Polygon([(1.685, 0.925), (2.455, 0.948), (2.455, rail(2.455) - 0.032), (2.08, rail(2.08) - 0.036), (1.99, 1.235)]).buffer(0.018).buffer(-0.006)
win_r = Polygon([(2.555, 0.950), (3.23, 0.970), (3.27, 1.03), (3.06, rail(3.06) - 0.04), (2.555, rail(2.555) - 0.032)]).buffer(0.018).buffer(-0.006)
win_q = Polygon([(3.33, 0.982), (3.60, 1.000), (3.335, 1.178)]).buffer(0.012).buffer(-0.004)

# Doors.
door_fl = outline([(1.50, 0.30), (2.492, 0.30), (2.492, rail(2.492))], poly_fn(rail, 2.47, 1.66, 28), [(1.50, 0.915)]).buffer(0.012).buffer(-0.012)
door_rl_outline = outline([(2.508, 0.30), (3.255, 0.30)], [(3.262, 0.37), (3.285, 0.46), (3.33, 0.58), (3.372, 0.72), (3.385, 0.97), (3.30, 1.09), (3.16, rail(3.16) - 0.012)], poly_fn(rail, 3.13, 2.53, 20)).buffer(0.012).buffer(-0.012)
door_rl = door_rl_outline.difference(RA.buffer(0.014))
panel("door_fl", "door_fl", "paint", door_fl.difference(win_f.buffer(0.022)), SIDE, mirror=True, flip_group="door_fr", flip_name="door_fr")
panel("door_rl", "door_rl", "paint", door_rl.difference(win_r.buffer(0.022)), SIDE, mirror=True, flip_group="door_rr", flip_name="door_rr")

# Black window surrounds on the doors, as on the real car.
for nm, gp, w in (("fl", "door_fl", win_f), ("rl", "door_rl", win_r)):
    frame = w.buffer(0.026).difference(w.buffer(-0.002))
    panel("frame_" + nm, gp, "black", frame, SIDE, offset=0.0015, roll=0.002, flange=0, h=0.03, mirror=True, flip_group=gp.replace("l", "r"), flip_name="frame_" + nm[0] + "r")
    panel("glass_" + nm, gp, "glass", w.buffer(0.004), SIDE, offset=-0.008, roll=0, flange=0, h=0.05, mirror=True, flip_group=gp.replace("l", "r"), flip_name="glass_" + nm[0] + "r")
panel("glass_q", "body", "glass", win_q.buffer(0.004), SIDE, offset=-0.006, roll=0, flange=0, h=0.05, mirror=True, flip_name="glass_qr")
panel("frame_q", "body", "black", win_q.buffer(0.018).difference(win_q.buffer(-0.002)), SIDE, offset=0.0015, roll=0.002, flange=0, h=0.03, mirror=True, flip_name="frame_qr")

# Front wing: from the headlight back to the door, around the arch, up to the bonnet shut line.
yh = P([0.16, 0.5, 0.9, 1.33], [0.62, 0.745, 0.775, 0.785])
xs_h = np.linspace(0.34, 1.495, 50)


def z_on_surface(x, y):
    zz = np.linspace(0.3, 1.3, 800)
    yy = Ysurf(np.full_like(zz, x), zz)
    i = np.where(yy >= y)[0]
    return float(zz[i[-1]]) if len(i) else 0.5


zh = [(x, z_on_surface(x, float(yh(x)) + 0.006)) for x in xs_h]
wing = outline([(0.34, 0.552), (0.60, 0.53)], [(1.20, 0.30), (1.487, 0.30), (1.487, 0.90)], zh[::-1]).difference(FA)
wing = wing.difference(box(0.0, 0.0, 0.62, 0.545)).buffer(0)
panel("wing_fl", "body", "paint", wing, SIDE, mirror=True, flip_name="wing_fr")

# Rear quarter with C pillar, around the rear arch and down to the bumper line.
qt_top = poly_fn(rail, 4.17, 3.17, 40)
quarter = outline([(3.27, 0.285), (4.17, 0.56)], qt_top, [(3.30, 1.09), (3.385, 0.97), (3.372, 0.72), (3.33, 0.58), (3.285, 0.46), (3.262, 0.37)])
quarter = unary_union([quarter, box(3.262, 0.285, 3.32, 0.46), box(3.27, 0.53, 4.17, 0.70)]).difference(RA).difference(door_rl_outline.buffer(GAP * 0.5))
quarter = quarter.difference(box(3.9, 0.0, 4.3, 0.555)).difference(win_q.buffer(0.016)).buffer(0)
if hasattr(quarter, "geoms"):
    quarter = max(quarter.geoms, key=lambda g: g.area)
# Above the rear lights the quarter runs on towards the lip, closing the corner up to the tailgate edge.
quarter = unary_union([quarter, outline([(4.15, 0.90), (4.44, 0.90)], poly_fn(rail, 4.44, 4.15, 16))]).buffer(0)
panel("quarter_l", "body", "paint", quarter, SIDE, mirror=True, flip_name="quarter_r")

# Sills, in dark plastic.
sill = box(1.255, 0.214, 3.27, 0.292).difference(FA).difference(RA)
panel("sill_l", "body", "black", sill, SIDE, mirror=True, flip_name="sill_r", h=0.05)

# A post foot between wing and door.
apost = Polygon([(1.492, 0.85), (1.495, 0.915), (1.66, rail(1.66) - 0.004), (1.54, rail(1.54) + 0.1)]).buffer(0)

# Rubbing strips along the doors and quarter, typical of the car.
for nm, gp, x0, x1 in (("fl", "door_fl", 1.505, 2.487), ("rl", "door_rl", 2.513, 3.20)):
    strip = box(x0, 0.500, x1, 0.542)
    panel("rub_" + nm, gp, "dark", strip, SIDE, offset=0.008, roll=0.006, flange=0.012, h=0.04, mirror=True, flip_group=gp.replace("l", "r"), flip_name="rub_" + nm[0] + "r")
    panel("rubc_" + nm, gp, "chrome", box(x0, 0.522, x1, 0.530), SIDE, offset=0.0105, roll=0.0, flange=0.0, h=0.05, mirror=True, flip_group=gp.replace("l", "r"), flip_name="rubc_" + nm[0] + "r")

# Bonnet in top view.
hood_side = [(x, float(yh(x))) for x in np.linspace(0.20, 1.30, 40)]
hood = Polygon([(0.17, 0.0), (0.17, 0.60)] + hood_side + [(1.315, 0.0)] + [(1.30, -y) for x, y in []])
hood = Polygon(hood_side + [(1.315, 0.0)] + [(x, -y) for x, y in reversed(hood_side)] + [(0.155, 0.0)]).buffer(0.015).buffer(-0.015)
panel("hood", "hood", "paint", hood, TOP, h=0.05)

# Roof with A posts: a top-view outline around windscreen and rear window.
def yrail_inner(x):
    return float(yrail_c(x)) - 0.004


def ws_half(x):
    # Half width of the windscreen in top view, slightly tapering upwards.
    return float(P([1.36, 1.7, 2.04], [0.735, 0.655, 0.585])(x))


xroof = np.linspace(1.33, 3.21, 60)
roof_outer = Polygon([(x, yrail_inner(x)) for x in xroof] + [(x, -yrail_inner(x)) for x in xroof[::-1]])
windscreen = Polygon([(1.37, 0.0), (1.38, 0.5), (1.39, ws_half(1.40))] + [(x, ws_half(x)) for x in np.linspace(1.42, 2.02, 18)] + [(2.05, 0.45), (2.055, 0.0)]
                     + [(2.05, -0.45)] + [(x, -ws_half(x)) for x in np.linspace(2.02, 1.42, 18)] + [(1.39, -ws_half(1.40)), (1.38, -0.5)]).buffer(0.03).buffer(-0.03)
cowl = box(1.27, -0.78, 1.385, 0.78)
roof = roof_outer.difference(windscreen.buffer(0.012)).difference(box(1.25, -0.9, 1.40, 0.9))
panel("roof", "body", "paint", roof, TOP, h=0.05)
panel("glass_ws", "windscreen", "glass", windscreen.buffer(0.006), TOP, offset=-0.004, roll=0, flange=0, h=0.05)
panel("frame_ws", "windscreen", "black", windscreen.buffer(0.014).difference(windscreen.buffer(-0.016)), TOP, offset=-0.0005, roll=0.002, flange=0, h=0.03)
panel("cowl", "windscreen", "black", Polygon([(1.315, 0.0), (1.30, 0.78), (1.40, 0.76), (1.40, -0.76), (1.30, -0.78)]), TOP, offset=-0.002, roll=0.002, flange=0, h=0.03)

# Tailgate: rear window and deck in top view, tail panel in the tail chart.
xl = np.linspace(3.225, 4.465, 50)


def gate_half_width(x):
    # The tail rounds off in plan view, so the deck has to narrow with it towards the lip.
    pl = float(plan(np.array([x]), np.array([1.04]))[0])
    return (yrail_inner(x) - 0.006) * min(pl, 1.0)


gate_top = Polygon([(x, gate_half_width(x)) for x in xl] + [(x, -gate_half_width(x)) for x in xl[::-1]]).buffer(0.01).buffer(-0.01)
rw_half = P([3.27, 3.6, 3.92], [0.575, 0.625, 0.655])
rear_win = Polygon([(x, float(rw_half(x))) for x in np.linspace(3.27, 3.92, 16)] + [(x, -float(rw_half(x))) for x in np.linspace(3.92, 3.27, 16)]).buffer(0.04).buffer(-0.04)
panel("gate_top", "tailgate", "paint", gate_top.difference(rear_win.buffer(0.02)), TOP, h=0.05)
panel("glass_rw", "tailgate", "glass", rear_win.buffer(0.006), TOP, offset=-0.004, roll=0, flange=0, h=0.05)
panel("frame_rw", "tailgate", "black", rear_win.buffer(0.018).difference(rear_win.buffer(-0.018)), TOP, offset=-0.0005, roll=0.002, flange=0, h=0.03)

print("nose and tail...", flush=True)
# ---- Front: chart u = angle * 0.9 m.
uB = 1.17
bump_f = box(-uB, 0.235, uB, 0.527)
intake = Polygon([(-0.56, 0.285), (0.56, 0.285), (0.60, 0.415), (-0.60, 0.415)]).buffer(0.015)
fog_l = Point(0.72, 0.345).buffer(0.052)
fog_r = Point(-0.72, 0.345).buffer(0.052)


def arch_in_chart(circle, chart):
    """A round opening drawn in side view, as it appears in a chart that wraps around the corner."""
    x, z = np.array(circle.exterior.coords).T
    y = Ysurf(x, z)
    u = chart.r * np.arctan2(y, chart.cx - x if chart.front else x - chart.cx)
    return Polygon(np.stack([u, z], -1)).buffer(0)


# Without this the panels end in a straight vertical edge across the wheel opening.
arch_l = arch_in_chart(FA, NOSE)
arch_r = Polygon([(-a, b) for a, b in arch_l.exterior.coords])
arches_f = unary_union([arch_l, arch_r])
bump_f = bump_f.difference(arches_f).buffer(-0.014).buffer(0.014)  # no sharp slivers left at the arch
panel("bumper_f", "bumper_f", "paint", bump_f.difference(intake).difference(fog_l).difference(fog_r), NOSE, h=0.05, flange=0.03)
panel("intake", "bumper_f", "dark", intake.buffer(0.004), NOSE, offset=-0.022, roll=0, flange=0, h=0.05)
for i in range(3):
    z = 0.312 + i * 0.038
    panel("intake_bar%d" % i, "bumper_f", "black", box(-0.56, z, 0.56, z + 0.012), NOSE, offset=-0.012, roll=0.002, flange=0.008, h=0.04)
panel("fog_l", "bumper_f", "reflector", fog_l.buffer(0.002), NOSE, offset=-0.012, roll=0, flange=0, h=0.02)
panel("fog_r", "bumper_f", "reflector", fog_r.buffer(0.002), NOSE, offset=-0.012, roll=0, flange=0, h=0.02)
# Chrome strip on top of the bumper, running around the nose as on the Activa.
panel("strip_f", "bumper_f", "chrome", box(-uB + 0.01, 0.527, uB - 0.01, 0.541).difference(arches_f.buffer(0.006)), NOSE, offset=0.006, roll=0.004, flange=0.012, h=0.04, edge=0.012)

# Number plate.
panel("plate_f", "bumper_f", "plate", box(-0.26, 0.44, 0.26, 0.55) if False else box(-0.26, 0.42, 0.26, 0.5), NOSE, offset=0.008, roll=0.001, flange=0.006, h=0.05)

# Grille between the headlights: two slats and the double chevron.
grille = Polygon([(-0.40, 0.548), (0.40, 0.548), (0.40, 0.632), (-0.40, 0.632)])
panel("grille", "bumper_f", "dark", grille, NOSE, offset=-0.012, roll=0, flange=0, h=0.04)
panel("grille_slat1", "bumper_f", "chrome", box(-0.40, 0.568, 0.40, 0.581), NOSE, offset=-0.004, roll=0.003, flange=0.006, h=0.04)
panel("grille_slat2", "bumper_f", "chrome", box(-0.40, 0.600, 0.40, 0.613), NOSE, offset=-0.004, roll=0.003, flange=0.006, h=0.04)
for z0 in (0.566, 0.596):
    chev = Polygon([(-0.085, z0), (0.0, z0 + 0.03), (0.085, z0), (0.085, z0 + 0.015), (0.0, z0 + 0.045), (-0.085, z0 + 0.015)])
    panel("chevron%d" % int(z0 * 1000), "bumper_f", "chrome", chev, NOSE, offset=0.004, roll=0.002, flange=0.006, h=0.02)

# Headlights and indicators, wrapping around the corner.
lamp = Polygon([(0.405, 0.548), (0.98, 0.548), (0.975, 0.615), (0.84, 0.645), (0.405, 0.645)]).buffer(0.012).buffer(-0.012)
ind = Polygon([(0.985, 0.548), (1.11, 0.548), (1.10, 0.585), (0.98, 0.615)]).buffer(0.006).buffer(-0.006)
for nm, s in (("l", 1.0), ("r", -1.0)):
    lp = Polygon([(s * x, y) for x, y in lamp.exterior.coords])
    id2 = Polygon([(s * x, y) for x, y in ind.exterior.coords])
    panel("lamp_refl_" + nm, "bumper_f", "reflector", lp.buffer(-0.002), NOSE, offset=-0.03, roll=0, flange=0, h=0.03)
    for u0 in (0.52, 0.76):
        panel("lamp_bowl%d_%s" % (int(u0 * 100), nm), "bumper_f", "dark", Point(s * u0, 0.597).buffer(0.042), NOSE, offset=-0.024, roll=0, flange=0, h=0.02)
        panel("lamp_bulb%d_%s" % (int(u0 * 100), nm), "bumper_f", "chrome", Point(s * u0, 0.597).buffer(0.018), NOSE, offset=-0.016, roll=0, flange=0, h=0.012)
    panel("lamp_" + nm, "bumper_f", "lens", lp, NOSE, offset=-0.002, roll=0.003, flange=0.03, h=0.03)
    panel("ind_" + nm, "bumper_f", "amber", id2, NOSE, offset=-0.002, roll=0.003, flange=0.02, h=0.025)
    # Dark nose surround between lamp, wing and bumper.
surround = box(-1.17, 0.541, 1.17, 0.66).difference(lamp).difference(Polygon([(-x, y) for x, y in lamp.exterior.coords])).difference(ind).difference(Polygon([(-x, y) for x, y in ind.exterior.coords])).difference(grille).difference(arches_f)
panel("nose_surround", "body", "paint", surround.buffer(-0.003), NOSE, offset=-0.003, roll=0.002, flange=0.01, h=0.04)

# ---- Rear.
uR = 1.20
bump_r = box(-uR, 0.285, uR, 0.555)
panel("bumper_r", "bumper_r", "paint", bump_r, TAIL, h=0.05, flange=0.03)
panel("strip_r", "bumper_r", "chrome", box(-uR + 0.02, 0.555, uR - 0.02, 0.569), TAIL, offset=0.006, roll=0.004, flange=0.012, h=0.04, edge=0.012)
panel("bumper_r_low", "bumper_r", "dark", box(-uR + 0.05, 0.27, uR - 0.05, 0.37), TAIL, offset=0.003, roll=0.004, flange=0.01, h=0.05)

# Tail panel (moves with the tailgate), corners (fixed to the body), split rear lights.
gate_half = 0.62
tail_lid = box(-gate_half, 0.575, gate_half, 1.028)
corner_l = Polygon([(gate_half + 0.006, 0.575), (uR - 0.05, 0.575), (uR - 0.03, 0.985), (gate_half + 0.006, 1.03)])
lt_in = box(0.31, 0.745, gate_half - 0.018, 0.89).buffer(0.008).buffer(-0.008)
lt_out = Polygon([(gate_half + 0.018, 0.745), (0.98, 0.745), (0.98, 0.88), (gate_half + 0.018, 0.89)]).buffer(0.008).buffer(-0.008)
holes = [lt_in, Polygon([(-x, y) for x, y in lt_in.exterior.coords])]
panel("tail_lid", "tailgate", "paint", tail_lid.difference(unary_union(holes)), TAIL, h=0.05)
panel("tail_corner_l", "body", "paint", corner_l.difference(lt_out), TAIL, h=0.05, offset=0.0015)
panel("tail_corner_r", "body", "paint", scale(corner_l.difference(lt_out), xfact=-1, yfact=1, origin=(0, 0)), TAIL, h=0.05, offset=0.0015)
for s, nm in ((1, "l"), (-1, "r")):
    li = Polygon([(s * x, y) for x, y in lt_in.exterior.coords])
    lo = Polygon([(s * x, y) for x, y in lt_out.exterior.coords])
    panel("tail_in_" + nm, "tailgate", "tail", li, TAIL, offset=-0.001, roll=0.003, flange=0.02, h=0.03)
    panel("tail_out_" + nm, "body", "tail", lo, TAIL, offset=-0.001, roll=0.003, flange=0.02, h=0.03)
    panel("tail_rev_" + nm, "tailgate", "white", box(min(s * 0.34, s * 0.46), 0.83, max(s * 0.34, s * 0.46), 0.875), TAIL, offset=0.001, roll=0.001, flange=0, h=0.03)
panel("plate_r", "tailgate", "plate", box(-0.26, 0.74, 0.26, 0.85), TAIL, offset=0.008, roll=0.001, flange=0.006, h=0.05)
panel("chevron_r", "tailgate", "chrome", Polygon([(-0.05, 0.90), (0, 0.925), (0.05, 0.90), (0.05, 0.915), (0, 0.94), (-0.05, 0.915)]), TAIL, offset=0.004, roll=0.002, flange=0.004, h=0.02)


# ---- Primitive solids, used for all the mechanicals. All have outward normals and hard edges.
def prim_box(c, s, R=None):
    """Axis-aligned box with per-face vertices, optionally rotated by matrix R."""
    c = np.asarray(c, float)
    hs = np.asarray(s, float) * 0.5
    v, n, f = [], [], []
    for ax in range(3):
        for sg in (-1, 1):
            nr = np.zeros(3)
            nr[ax] = sg
            u = np.zeros(3)
            w = np.zeros(3)
            u[(ax + 1) % 3] = hs[(ax + 1) % 3]
            w[(ax + 2) % 3] = hs[(ax + 2) % 3]
            cc = nr * hs
            base = len(v)
            corners = [cc - u - w, cc + u - w, cc + u + w, cc - u + w]
            v += corners
            n += [nr] * 4
            f += ([[base, base + 1, base + 2], [base, base + 2, base + 3]] if sg > 0 else [[base, base + 2, base + 1], [base, base + 3, base + 2]])
    v = np.array(v)
    n = np.array(n)
    if R is not None:
        v = v @ R.T
        n = n @ R.T
    return v + c, n, np.array(f)


def lathe(profile, axis_from, axis_to, seg=24, cap=True):
    """Solid of revolution from a (t along axis 0..1, radius) profile."""
    a = np.asarray(axis_from, float)
    b = np.asarray(axis_to, float)
    ax = b - a
    L = np.linalg.norm(ax)
    ax = ax / L
    h = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 0, 1.0])
    u = np.cross(ax, h)
    u /= np.linalg.norm(u)
    w = np.cross(ax, u)
    prof = np.array(profile, float)
    v, f = [], []
    n_rows = len(prof)
    ang = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    for t, r in prof:
        for an in ang:
            v.append(a + ax * (t * L) + (u * np.cos(an) + w * np.sin(an)) * r)
    for i in range(n_rows - 1):
        for j in range(seg):
            p = i * seg + j
            q = i * seg + (j + 1) % seg
            r_ = (i + 1) * seg + (j + 1) % seg
            s = (i + 1) * seg + j
            f += [[p, s, r_], [p, r_, q]]
    v = np.array(v)
    f = np.array(f)
    center = a + ax * (L * 0.5)
    # Make winding face outwards from the axis.
    tc = v[f].mean(1)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    rad = tc - (a + np.outer((tc - a) @ ax, ax))
    # Faces perpendicular to the axis have no radial part: use the axial direction from the centre.
    out = rad + (tc - center) * 0.001
    wrong = (fn * out).sum(1) < 0
    f[wrong] = f[wrong][:, [0, 2, 1]]
    return v, mesh_normals(v, f), f


def cyl(a, b, r, seg=18, r2=None):
    r2 = r if r2 is None else r2
    return lathe([(0, 0.0005), (0.0005, r), (0.9995, r2), (1, 0.0005)], a, b, seg)


def ball(c, r, seg=20):
    c = np.asarray(c, float)
    prof = [(t, r * math.sqrt(max(1 - (2 * t - 1) ** 2, 1e-5))) for t in np.linspace(0.002, 0.998, 12)]
    return lathe(prof, c - [0, 0, r], c + [0, 0, r], seg)


def ellipsoid(c, r, seg=20):
    v, n, f = ball((0, 0, 0), 1.0, seg)
    r = np.asarray(r, float)
    v2 = v * r + np.asarray(c, float)
    n2 = n / r
    n2 /= np.linalg.norm(n2, axis=1, keepdims=True)
    return v2, n2, f


def rotY(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotX(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rotZ(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Comp:
    """Collects primitives per material for one rigid or bound group."""

    def __init__(self, group):
        self.group = group
        self.by_mat = {}

    def add(self, mat, prim):
        self.by_mat.setdefault(mat, []).append(prim)
        return self

    def box(self, mat, c, s, R=None):
        return self.add(mat, prim_box(c, s, R))

    def cyl(self, mat, a, b, r, seg=18, r2=None):
        return self.add(mat, cyl(a, b, r, seg, r2))

    def ball(self, mat, c, r, seg=20):
        return self.add(mat, ball(c, r, seg))

    def finish(self, name):
        for mat, prims in self.by_mat.items():
            vs, ns, fs = [], [], []
            base = 0
            for v, n, f in prims:
                vs.append(v)
                ns.append(n)
                fs.append(f + base)
                base += len(v)
            add_part(name + "_" + mat, self.group, mat, np.vstack(vs), np.vstack(ns), np.vstack(fs))


print("engine, gearbox and mechanicals...", flush=True)
# ---- PSA ES9 V6, transverse: crank along y, 60 degree banks towards front and rear.
E = Comp("engine")
ex, ez = 0.70, 0.40  # centre of the crank
E.box("alu", (ex, -0.06, ez + 0.02), (0.30, 0.46, 0.26))             # crankcase
E.box("metal", (ex, -0.06, ez - 0.13), (0.33, 0.44, 0.10))           # sump
for bank, sgn in (("f", -1), ("r", 1)):
    R = rotY(sgn * math.radians(30))
    d = np.array([sgn * math.sin(math.radians(30)), 0, math.cos(math.radians(30))])
    head = np.array([ex, -0.06, ez + 0.02]) + d * 0.17
    E.add("alu", prim_box(head, (0.17, 0.42, 0.13), R))                 # cylinder block
    E.add("metal", prim_box(head + d * 0.09, (0.19, 0.43, 0.07), R))    # head
    E.add("black", prim_box(head + d * 0.14, (0.17, 0.42, 0.045), R))  # cam cover
    for i in range(3):
        y = -0.06 + (i - 1) * 0.125
        c = head + d * 0.168 + np.array([0, y + 0.06, 0])
        E.add("chrome", prim_box(c, (0.095, 0.085, 0.012), R))          # the three ovals
    # Exhaust manifold on the outer side of each bank.
    for i in range(3):
        y = -0.06 + (i - 1) * 0.125
        p0 = head + np.array([sgn * 0.11, y + 0.06, 0.02])
        E.cyl("rust", p0, p0 + np.array([sgn * 0.05, 0, -0.14]), 0.022, 10)
# Intake plenum across the top, in the valley between the banks.
E.box("alu", (ex, -0.06, ez + 0.30), (0.15, 0.40, 0.07))
for i in range(6):
    y = -0.06 + (i - 2.5) * 0.065
    sg = 1 if i % 2 else -1
    E.cyl("alu", (ex + 0.0, y, ez + 0.29), (ex + sg * 0.11, y, ez + 0.23), 0.018, 10)
# Crank pulley, belts and auxiliaries on the timing side (right side of the car).
E.cyl("metal", (ex, -0.32, ez), (ex, -0.29, ez), 0.085, 26)
E.cyl("dark", (ex, -0.325, ez), (ex, -0.31, ez), 0.09, 26)
E.cyl("metal", (ex - 0.17, -0.31, ez + 0.17), (ex - 0.17, -0.20, ez + 0.17), 0.058, 20)   # alternator
E.cyl("alu", (ex - 0.17, -0.20, ez + 0.17), (ex - 0.17, -0.10, ez + 0.17), 0.062, 20)
E.cyl("dark", (ex + 0.17, -0.31, ez - 0.03), (ex + 0.17, -0.19, ez - 0.03), 0.05, 18)     # HP pump
E.cyl("metal", (ex + 0.17, -0.19, ez - 0.03), (ex + 0.17, -0.09, ez - 0.03), 0.042, 18)
E.box("dark", (ex, -0.305, ez + 0.14), (0.19, 0.02, 0.44))                                 # timing cover
# High-pressure accumulator sphere sits on the regulator on the engine.
E.ball("lhm", (ex + 0.21, -0.28, ez + 0.02), 0.058)
E.cyl("metal", (ex + 0.21, -0.28, ez - 0.06), (ex + 0.21, -0.28, ez - 0.03), 0.035, 12)
E.finish("engine")

# Gearbox: bell housing and case on the left of the engine.
G = Comp("gearbox")
G.cyl("alu", (ex, 0.17, ez), (ex, 0.26, ez), 0.145, 28, r2=0.13)
G.box("alu", (ex - 0.01, 0.34, ez - 0.03), (0.24, 0.17, 0.26))
G.cyl("alu", (ex - 0.03, 0.26, ez + 0.05), (ex - 0.03, 0.47, ez + 0.05), 0.075, 18)
G.cyl("metal", (ex + 0.08, 0.28, ez - 0.14), (ex + 0.08, 0.42, ez - 0.14), 0.09, 22)       # differential
for i in range(5):
    G.box("alu", (ex - 0.13, 0.30 + i * 0.022, ez - 0.03), (0.012, 0.008, 0.2))              # cooling ribs
G.box("dark", (ex - 0.03, 0.36, ez + 0.11), (0.09, 0.07, 0.03))                              # shift tower
G.finish("gearbox")

# Radiator, fan and cross member.
R_ = Comp("radiator")
R_.box("dark", (0.30, 0.0, 0.47), (0.03, 1.10, 0.42))
R_.box("metal", (0.285, 0.0, 0.47), (0.004, 1.04, 0.36))
R_.cyl("black", (0.33, 0.18, 0.47), (0.385, 0.18, 0.47), 0.17, 24)
R_.cyl("black", (0.33, -0.25, 0.47), (0.385, -0.25, 0.47), 0.14, 24)
R_.cyl("black", (0.36, 0.18, 0.47), (0.385, 0.18, 0.47), 0.05, 14)
R_.cyl("black", (0.36, -0.25, 0.47), (0.385, -0.25, 0.47), 0.05, 14)
R_.finish("radiator")

# Battery and air box on the left of the bay, as in the reference photo.
B_ = Comp("battery")
B_.box("black", (1.04, 0.50, 0.58), (0.19, 0.28, 0.19))
B_.box("dark", (1.04, 0.50, 0.68), (0.20, 0.29, 0.015))
B_.cyl("metal", (1.0, 0.42, 0.685), (1.0, 0.42, 0.71), 0.012, 8)
B_.cyl("rust", (1.0, 0.58, 0.685), (1.0, 0.58, 0.71), 0.012, 8)
B_.finish("battery")
AB = Comp("frame")
AB.box("black", (0.62, 0.50, 0.66), (0.34, 0.22, 0.13))
AB.box("dark", (0.45, 0.40, 0.66), (0.06, 0.08, 0.09))
AB.cyl("black", (0.80, 0.42, 0.68), (0.95, 0.15, 0.78), 0.05, 14)
# Reservoir of LHM fluid, the bright green tell-tale of every hydraulic Citroen.
AB.cyl("lhm", (1.13, -0.42, 0.56), (1.13, -0.42, 0.72), 0.075, 18)
AB.cyl("black", (1.13, -0.42, 0.72), (1.13, -0.42, 0.75), 0.05, 14)
AB.finish("bay_parts")

# Bulkhead, inner wings and underbody: what you see through a missing panel.
IW = Comp("frame")
IW.box("liner", (1.40, 0.0, 0.54), (0.02, 1.40, 0.68))                       # firewall
IW.box("metal", (0.27, 0.0, 0.40), (0.03, 0.56, 0.22))                       # radiator support


def sheet_x(comp, mat, xs, y, zlo, zhi):
    """Vertical sheet along the car at constant y, with its top edge given by zhi(x)."""
    v, f = [], []
    for x in xs:
        v.append((x, y, zlo))
        v.append((x, y, zhi(x)))
    for i in range(len(xs) - 1):
        a = i * 2
        f += [[a, a + 2, a + 3], [a, a + 3, a + 1]]
    v = np.array(v)
    f = np.array(f)
    comp.add(mat, (v, mesh_normals(v, f), f))


for sg in (-1, 1):
    # Inner wing: follows the underside of the bonnet at a safe distance.
    sheet_x(IW, "liner", np.linspace(0.42, 1.39, 16), sg * 0.70, 0.30,
            lambda x: float(ztop_surface(np.array([x]), np.array([0.70]))[0]) - 0.05)
    IW.cyl("liner", (0.92, sg * 0.60, 0.56), (0.92, sg * 0.60, 0.76), 0.085, 18)  # strut tower
    IW.box("metal", (0.85, sg * 0.40, 0.29), (1.10, 0.08, 0.09))                  # chassis rail
IW.box("metal", (0.45, 0.0, 0.30), (0.08, 0.72, 0.06))                            # front cross member
IW.box("liner", (2.35, 0.0, 0.213), (2.1, 1.28, 0.012))                           # floor
IW.box("metal", (2.35, 0.0, 0.25), (2.0, 0.22, 0.06))                             # tunnel
for x in (1.70, 2.30, 2.85):
    IW.box("metal", (x, 0.0, 0.228), (0.06, 1.26, 0.02))                          # cross members
for sg in (-1, 1):
    IW.box("metal", (2.3, sg * 0.70, 0.27), (2.0, 0.10, 0.09))                    # sill box
IW.box("liner", (3.85, 0.0, 0.32), (1.05, 1.10, 0.02))                            # boot floor
IW.box("liner", (3.25, 0.0, 0.62), (0.02, 1.36, 0.80))                            # seat back bulkhead
IW.box("liner", (4.0, 0.0, 1.03), (0.62, 1.30, 0.012))                             # parcel shelf


def arch_liner(comp, cx, sg, r=0.392, y0=0.62, y1=0.872):
    """Half-round wheel arch: a curved roof over the wheel and an inner wall."""
    v, f = [], []
    th = np.linspace(math.radians(-8), math.radians(188), 26)
    for t in th:
        px, pz = cx + r * math.cos(t), 0.312 + r * math.sin(t)
        v.append((px, sg * y0, pz))
        v.append((px, sg * y1, pz))
    for i in range(len(th) - 1):
        a = i * 2
        f += [[a, a + 2, a + 3], [a, a + 3, a + 1]]
    base = len(v)
    # Inner wall.
    v.append((cx, sg * y0, 0.312))
    for t in th:
        v.append((cx + r * math.cos(t), sg * y0, 0.312 + r * math.sin(t)))
    for i in range(len(th) - 1):
        f.append([base, base + 1 + i, base + 2 + i])
    v = np.array(v, float)
    f = np.array(f)
    comp.add("liner", (v, mesh_normals(v, f), f))


for cx in (0.90, 3.64):
    for sg in (-1, 1):
        arch_liner(IW, cx, sg)
# Boot tub: closes the car from the inside, so that no gap ever looks straight through it.
IW.box("liner", (4.36, 0.0, 0.66), (0.012, 1.14, 0.68))
for sg in (-1, 1):
    IW.box("liner", (4.14, sg * 0.57, 0.66), (0.44, 0.012, 0.68))
IW.finish("shell_inner")

# Interior.
IN = Comp("frame")
IN.box("dark", (1.60, 0.0, 0.84), (0.34, 1.36, 0.12))                         # dashboard top
IN.box("dark", (1.53, 0.0, 0.60), (0.22, 1.34, 0.38))
IN.box("black", (1.68, 0.0, 0.89), (0.1, 1.30, 0.02))
IN.box("carpet", (1.95, 0.0, 0.37), (0.60, 0.18, 0.26))                        # console
IN.box("metal", (1.76, 0.0, 0.52), (0.16, 0.12, 0.03))
IN.box("carpet", (2.2, 0.0, 0.226), (2.0, 1.26, 0.012))                        # carpet
IN.box("fabric", (2.52, 0.0, 1.345), (0.92, 1.04, 0.012))                      # headlining
for sg in (-1, 1):
    y = sg * 0.38
    for xs in (2.20, ):
        IN.box("fabric", (xs, y, 0.40), (0.50, 0.46, 0.13))
        IN.add("fabric", prim_box((xs + 0.25, y, 0.70), (0.13, 0.44, 0.56), rotY(math.radians(13))))
        IN.add("fabric", prim_box((xs + 0.33, y, 1.04), (0.09, 0.25, 0.18), rotY(math.radians(10))))
        IN.box("black", (xs, y, 0.30), (0.46, 0.40, 0.10))
IN.box("fabric", (3.03, 0.0, 0.42), (0.52, 1.30, 0.15))                        # rear bench
IN.add("fabric", prim_box((3.28, 0.0, 0.72), (0.14, 1.30, 0.56), rotY(math.radians(16))))
IN.finish("interior")

# Driver's controls: left-hand drive.
ST = Comp("frame")
sw = np.array([1.93, 0.38, 0.87])  # well clear of the dashboard, which ends at x = 1.77
st_axis = np.array([-math.cos(math.radians(24)), 0, -math.sin(math.radians(24))])
ST.cyl("black", sw + st_axis * 0.03, sw + st_axis * 0.30, 0.022, 24)
ST.cyl("dark", sw + st_axis * 0.05, sw + st_axis * 0.30, 0.048, 28)              # column shroud


def torus(c, axis, R, r, major=96, minor=20, flatten=1.0):
    """A closed, smooth ring. flatten < 1 makes the section oval, thicker in the driver's hands."""
    a = axis / np.linalg.norm(axis)
    e1 = np.cross(a, [0.0, 1.0, 0.0])
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    v, n, f = [], [], []
    for i in range(major):
        t = 2 * math.pi * i / major
        d = e1 * math.cos(t) + e2 * math.sin(t)
        for j in range(minor):
            p = 2 * math.pi * j / minor
            nn = d * math.cos(p) / flatten + a * math.sin(p)
            v.append(c + d * (R + r * flatten * math.cos(p)) + a * (r * math.sin(p)))
            n.append(nn / np.linalg.norm(nn))
    for i in range(major):
        for j in range(minor):
            A, B = i * minor + j, i * minor + (j + 1) % minor
            C, D = ((i + 1) % major) * minor + j, ((i + 1) % major) * minor + (j + 1) % minor
            f += [[A, C, D], [A, D, B]]
    return np.array(v, float), np.array(n, float), np.array(f)


def dome(c, axis, r0, h, rings=10, seg=48):
    """A shallow padded cap over the wheel's centre, facing against the axis."""
    a = axis / np.linalg.norm(axis)
    e1 = np.cross(a, [0.0, 1.0, 0.0])
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    v, n, f = [c - a * h], [-a], []
    for i in range(1, rings + 1):
        q = i / rings
        for j in range(seg):
            t = 2 * math.pi * j / seg
            d = e1 * math.cos(t) + e2 * math.sin(t)
            v.append(c + d * (r0 * q) - a * (h * (1 - q * q)))
            nn = -a + d * (2 * h * q / r0)
            n.append(nn / np.linalg.norm(nn))
    for j in range(seg):
        f.append([0, 1 + j, 1 + (j + 1) % seg])
    for i in range(rings - 1):
        for j in range(seg):
            A, B = 1 + i * seg + j, 1 + i * seg + (j + 1) % seg
            C, D = A + seg, B + seg
            f += [[A, C, D], [A, D, B]]
    return np.array(v, float), np.array(n, float), np.array(f)


# Everything named steerwheel_* turns about the column axis in the renderer. The rim is its own part:
# its vertices are evenly spread, so that their centre is exactly the point on the column.
SWR = Comp("frame")
SWR.add("black", torus(sw, st_axis, 0.182, 0.0135, flatten=1.12))
SW = Comp("frame")
sw_e1 = np.cross(st_axis, [0.0, 1.0, 0.0])
sw_e1 = sw_e1 / np.linalg.norm(sw_e1)
sw_e2 = np.cross(st_axis, sw_e1)
SW.add("dark", dome(sw - st_axis * 0.012, st_axis, 0.078, 0.024))
SW.cyl("black", sw - st_axis * 0.012, sw + st_axis * 0.03, 0.078, 48)
for ang in (0.0, 180.0, 270.0):
    d = sw_e1 * math.cos(math.radians(ang)) + sw_e2 * math.sin(math.radians(ang))
    w = np.cross(st_axis, d) * 0.018
    # Spokes are flat and wide, meeting the rim smoothly.
    SW.add("black", prim_box(sw + d * 0.125, (0.12, 0.05, 0.012), np.column_stack([d, np.cross(st_axis, d), st_axis])))
SWR.finish("steerwheel_rim")
SW.finish("steerwheel_hub")
STEER_AXIS = st_axis
STEER_CENTRE = sw
ST.finish("steering")

# Underbody: exhaust, fuel tank, rear beam.
EX = Comp("frame")
EX.cyl("rust", (1.05, 0.12, 0.30), (1.4, 0.05, 0.215), 0.03, 12)
EX.cyl("rust", (1.4, 0.05, 0.215), (2.3, 0.05, 0.22), 0.03, 12)
EX.cyl("rust", (1.6, 0.05, 0.215), (2.0, 0.05, 0.22), 0.065, 16)
EX.cyl("rust", (2.3, 0.05, 0.22), (3.2, 0.25, 0.27), 0.03, 12)
EX.cyl("rust", (3.2, 0.25, 0.27), (3.95, 0.30, 0.32), 0.03, 12)
EX.cyl("rust", (3.95, 0.02, 0.36), (3.95, 0.52, 0.36), 0.085, 18)
EX.cyl("metal", (3.98, 0.44, 0.34), (4.44, 0.44, 0.31), 0.028, 12)
EX.finish("exhaust")
FT = Comp("tank")
FT.box("dark", (3.25, 0.0, 0.31), (0.60, 1.08, 0.17))
FT.finish("fuel_tank")

# The rest of the ten spheres: four corners, two Hydractive regulators, two Activa, brake reserve.
SPH = Comp("frame")
for sg in (-1, 1):
    SPH.ball("lhm", (1.02, sg * 0.60, 0.78), 0.062)                 # front corner, on the strut top
    SPH.cyl("metal", (1.02, sg * 0.60, 0.70), (1.02, sg * 0.60, 0.74), 0.028, 12)
    SPH.ball("lhm", (3.72, sg * 0.56, 0.42), 0.062)                 # rear corner, next to the arm
    SPH.cyl("metal", (3.72, sg * 0.46, 0.42), (3.72, sg * 0.51, 0.42), 0.028, 12)
SPH.ball("lhm", (1.15, 0.15, 0.74), 0.062)                           # front Hydractive regulator
SPH.ball("lhm", (3.66, 0.22, 0.42), 0.062)                          # rear Hydractive regulator
SPH.ball("lhm", (0.43, 0.26, 0.30), 0.06)                            # front Activa sphere, by the bar
SPH.ball("lhm", (3.66, -0.22, 0.42), 0.056)                          # rear Activa sphere
SPH.ball("lhm", (1.20, -0.20, 0.74), 0.05)                           # brake reserve
# Hydraulic lines from the bay along the floor.
for sg, c in ((1, "metal"), (-1, "metal")):
    SPH.cyl(c, (1.02, sg * 0.60, 0.72), (1.25, sg * 0.25, 0.25), 0.005, 6)
    SPH.cyl(c, (1.25, sg * 0.25, 0.25), (3.25, sg * 0.32, 0.25), 0.005, 6)
    SPH.cyl(c, (3.25, sg * 0.32, 0.25), (3.65, sg * 0.46, 0.42), 0.005, 6)
SPH.finish("spheres")
# Exterior details bound to the doors.
for side, sg, gp_f, gp_r in (("l", 1, "door_fl", "door_rl"), ("r", -1, "door_fr", "door_rr")):
    for gp, xh in ((gp_f, 2.40), (gp_r, 3.14)):
        p = SIDE.map(np.array([[xh, 0.80]]))[0][0]
        DH = Comp(gp)
        DH.box("black", (xh, sg * (p[1] + 0.004), 0.80), (0.14, 0.018, 0.035))
        DH.finish("handle_" + gp)
    # Door mirror.
    MR = Comp(gp_f)
    MR.add("paint", ellipsoid((1.675, sg * 0.965, 0.985), (0.05, 0.10, 0.062)))
    MR.add("dark", prim_box((1.722, sg * 0.965, 0.985), (0.004, 0.165, 0.098)))
    MR.add("black", ellipsoid((1.69, sg * 0.89, 0.965), (0.035, 0.05, 0.025)))
    MR.finish("mirror_" + side)
    # Door cards.
    DC = Comp(gp_f)
    DC.box("dark", (2.0, sg * 0.79, 0.62), (0.92, 0.03, 0.56))
    DC.finish("card_f" + side)
    DC = Comp(gp_r)
    DC.box("dark", (2.88, sg * 0.79, 0.62), (0.68, 0.03, 0.56))
    DC.finish("card_r" + side)
# Wipers on the cowl.
WP = Comp("windscreen")
for y0 in (0.65, 0.05):
    z = float(ztop_surface(np.array([1.40]), np.array([y0]))[0])
    WP.add("black", prim_box((1.42, y0 - 0.30, z + 0.012), (0.012, 0.62, 0.012), rotZ(math.radians(-6))))
WP.finish("wipers")
# Rear wiper.
WR = Comp("tailgate")
z = float(ztop_surface(np.array([3.70]), np.array([0.0]))[0])
WR.add("black", prim_box((3.74, 0.0, z + 0.02), (0.012, 0.40, 0.012), rotZ(math.radians(50))))
WR.finish("wiper_r")

print("wheels...", flush=True)


def wheel_parts():
    """Wheel in local coordinates: axle along y (outboard is +y), metres. Spins as a rigid part."""
    tyre = [(0, 0.195), (0.05, 0.215), (0.10, 0.262), (0.2, 0.296), (0.35, 0.307), (0.5, 0.312), (0.65, 0.307), (0.8, 0.296), (0.90, 0.262), (0.95, 0.215), (1, 0.195)]
    t = lathe(tyre, (0, -0.1025, 0), (0, 0.1025, 0), 56)
    add_part("wheel_tyre", "wheel", "rubber", *t)
    # Tread grooves as darker rings.
    # Rim barrel, inner face and the 5-spoke alloy face of the V6.
    barrel = lathe([(0, 0.198), (0.04, 0.188), (0.12, 0.178), (0.65, 0.17), (0.9, 0.178), (1, 0.196)], (0, -0.095, 0), (0, 0.095, 0), 48)
    add_part("wheel_barrel", "wheel", "alloy", barrel[0], -barrel[1], barrel[2][:, [0, 2, 1]])
    verts, norms, faces = [], [], []
    y_face = 0.062
    nA, nR = 120, 16
    rs = np.linspace(0.0, 0.192, nR)
    for j in range(nR):
        for i in range(nA):
            a = 2 * math.pi * i / nA
            r = rs[j]
            sp = abs(((a / (2 * math.pi) * 5) % 1.0) - 0.5) * 2.0   # 0 at spoke centre, 1 between spokes
            width = 0.05 / max(r, 0.04)
            hole = (r > 0.075 and r < 0.165 and sp > 0.28 + 0.15 * (r / 0.17))
            dish = 0.018 * (1 - min(r / 0.19, 1.0)) ** 0.5
            y = y_face - (0.060 if hole else 0.0) + dish * 0.8
            verts.append((r * math.cos(a), y, r * math.sin(a)))
    verts = np.array(verts)
    for j in range(nR - 1):
        for i in range(nA):
            p = j * nA + i
            q = j * nA + (i + 1) % nA
            r_ = (j + 1) * nA + (i + 1) % nA
            s = (j + 1) * nA + i
            faces += [[p, q, r_], [p, r_, s]]
    faces = np.array(faces)
    fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
    wrong = fn[:, 1] < 0
    faces[wrong] = faces[wrong][:, [0, 2, 1]]
    add_part("wheel_face", "wheel", "alloy", verts, mesh_normals(verts, faces), faces)
    C = Comp("wheel")
    C.cyl("dark", (0, 0.066, 0), (0, 0.078, 0), 0.03, 16)
    for i in range(4):
        a = 2 * math.pi * i / 4
        C.cyl("metal", (0.054 * math.cos(a), 0.058, 0.054 * math.sin(a)), (0.054 * math.cos(a), 0.072, 0.054 * math.sin(a)), 0.012, 8)
    C.finish("wheel_hub")
    D = Comp("wheel")
    D.add("disc", lathe([(0, 0.08), (0.02, 0.135), (0.98, 0.135), (1, 0.08)], (0, -0.012, 0), (0, 0.012, 0), 40))
    D.add("dark", cyl((0, 0.012, 0), (0, 0.045, 0), 0.075, 20))
    D.finish("wheel_disc")
    K = Comp("wheel_static")
    K.box("metal", (0.10, 0.0, 0.08), (0.09, 0.075, 0.09), rotY(math.radians(40)))
    K.box("rust", (0.10, 0.016, 0.08), (0.07, 0.03, 0.07), rotY(math.radians(40)))
    K.finish("caliper")


wheel_parts()


def write(path):
    total_v = sum(len(p[3]) for p in parts)
    total_t = sum(len(p[5]) for p in parts)
    with open(path, "wb") as fp:
        fp.write(b"XMSH")
        fp.write(struct.pack("<I", len(parts)))
        for name, group, mat, v, n, f, flags in parts:
            fp.write(name.encode()[:39].ljust(40, b"\0"))
            fp.write(group.encode()[:23].ljust(24, b"\0"))
            fp.write(struct.pack("<III", mat, len(v), len(f)))
            fp.write(np.asarray(v, np.float32).tobytes())
            fp.write(np.asarray(n, np.float32).tobytes())
            fp.write(np.asarray(f, np.uint32).tobytes())
    print("wrote %s: %d parts, %d vertices, %d triangles" % (path, len(parts), total_v, total_t))


if __name__ == "__main__":
    write(sys.argv[1] if len(sys.argv) > 1 else "assets/xantia.mesh")
