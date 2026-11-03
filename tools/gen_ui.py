#!/usr/bin/env python3
"""Generates assets/ui.atlas: a signed-distance atlas of a real typeface and of the HUD's small icons.

Distance fields stay crisp at any size on screen, and need nothing but one texture at runtime.
Format, little endian: "UIAT", u32 width, height, count; count * (u32 code, 8 floats: u0 v0 u1 v1
xoff yoff w h, float advance), all metrics relative to a font size of 1; then width*height bytes.
"""
import math
import struct
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import distance_transform_edt

BIG = 96          # size each glyph is drawn at
PAD = 12          # distance field margin at that size
SCALE = 3         # how much it is shrunk into the atlas
ATLAS = 1024


def find_font():
    for query in ("Inter:bold", "Noto Sans:bold", "DejaVu Sans:bold", "sans:bold"):
        try:
            path = subprocess.check_output(["fc-match", "-f", "%{file}", query], text=True).strip()
            if path:
                return path
        except (OSError, subprocess.CalledProcessError):
            pass
    return None


def sdf(mask):
    inside = distance_transform_edt(mask)
    outside = distance_transform_edt(~mask)
    d = np.clip((inside - outside) / PAD * 0.5 + 0.5, 0, 1)
    return d


entries = []
atlas = np.zeros((ATLAS, ATLAS), np.uint8)
pen_x = pen_y = 1
row_h = 0


def place(code, mask, xoff, yoff, advance):
    """mask: boolean bitmap with PAD margin all round. Offsets in units of the font size."""
    global pen_x, pen_y, row_h
    d = sdf(mask)
    h, w = d.shape
    sw, sh = max(w // SCALE, 1), max(h // SCALE, 1)
    small = np.asarray(Image.fromarray((d * 255).astype(np.uint8)).resize((sw, sh), Image.LANCZOS))
    if pen_x + sw + 1 >= ATLAS:
        pen_x = 1
        pen_y += row_h + 1
        row_h = 0
    if pen_y + sh >= ATLAS:
        raise SystemExit("atlas full")
    atlas[pen_y:pen_y + sh, pen_x:pen_x + sw] = small
    entries.append((code, pen_x / ATLAS, pen_y / ATLAS, (pen_x + sw) / ATLAS, (pen_y + sh) / ATLAS,
                    xoff, yoff, w / BIG, h / BIG, advance))
    pen_x += sw + 1
    row_h = max(row_h, sh)


def glyphs(font_path):
    font = ImageFont.truetype(font_path, BIG)
    ascent, _ = font.getmetrics()
    chars = [chr(c) for c in range(32, 127)] + list("äöüÄÖÜß°·–")
    for ch in chars:
        adv = font.getlength(ch) / BIG
        l, t, r, b = font.getbbox(ch)
        w, h = max(r - l, 1), max(b - t, 1)
        im = Image.new("L", (w + 2 * PAD, h + 2 * PAD), 0)
        ImageDraw.Draw(im).text((PAD - l, PAD - t), ch, font=font, fill=255)
        mask = np.asarray(im) > 127
        place(ord(ch), mask, (l - PAD) / BIG, (t - PAD - ascent * 0.0) / BIG, adv)


# ---- Icons: drawn on a 96 px canvas, code from 0xE000.
def icon(code, draw_fn):
    S = BIG
    im = Image.new("L", (S + 2 * PAD, S + 2 * PAD), 0)
    d = ImageDraw.Draw(im)

    class C:
        def poly(self, pts, fill=255):
            d.polygon([(PAD + x * S, PAD + y * S) for x, y in pts], fill=fill)

        def circle(self, x, y, r, fill=255):
            d.ellipse([PAD + (x - r) * S, PAD + (y - r) * S, PAD + (x + r) * S, PAD + (y + r) * S], fill=fill)

        def rect(self, x0, y0, x1, y1, fill=255, radius=0.0):
            d.rounded_rectangle([PAD + x0 * S, PAD + y0 * S, PAD + x1 * S, PAD + y1 * S], radius * S, fill=fill)

        def ring(self, x, y, r, w, fill=255):
            d.ellipse([PAD + (x - r) * S, PAD + (y - r) * S, PAD + (x + r) * S, PAD + (y + r) * S], outline=fill, width=max(int(w * S), 1))

        def arc(self, x, y, r, a0, a1, w, fill=255):
            d.arc([PAD + (x - r) * S, PAD + (y - r) * S, PAD + (x + r) * S, PAD + (y + r) * S], a0, a1, fill=fill, width=max(int(w * S), 1))

        def line(self, x0, y0, x1, y1, w, fill=255):
            d.line([PAD + x0 * S, PAD + y0 * S, PAD + x1 * S, PAD + y1 * S], fill=fill, width=max(int(w * S), 1))

    draw_fn(C())
    mask = np.asarray(im) > 127
    place(code, mask, -PAD / BIG, -PAD / BIG, 1.0)


def thermometer(c):
    # Coolant: a thermometer standing in waves.
    c.rect(0.44, 0.05, 0.58, 0.60, radius=0.07)
    c.rect(0.585, 0.12, 0.72, 0.17)
    c.rect(0.585, 0.26, 0.72, 0.31)
    c.rect(0.585, 0.40, 0.72, 0.45)
    c.circle(0.51, 0.66, 0.13)
    for k, y in enumerate((0.84, 0.95)):
        pts = [(0.08 + i * 0.84 / 24, y + 0.03 * math.sin(i * 0.8)) for i in range(25)]
        for a, b in zip(pts, pts[1:]):
            c.line(a[0], a[1], b[0], b[1], 0.05)


def oilcan(c):
    c.poly([(0.20, 0.42), (0.68, 0.42), (0.78, 0.74), (0.20, 0.74)])
    c.poly([(0.68, 0.46), (0.97, 0.30), (0.97, 0.37), (0.74, 0.60)])
    c.rect(0.32, 0.30, 0.46, 0.42)
    c.rect(0.26, 0.26, 0.52, 0.31)
    c.poly([(0.20, 0.46), (0.04, 0.34), (0.02, 0.44), (0.20, 0.62)])
    c.circle(0.97, 0.56, 0.035)
    c.poly([(0.97, 0.47), (1.0, 0.56), (0.94, 0.56)])


def fuel(c):
    c.rect(0.16, 0.08, 0.64, 0.92, radius=0.05)
    c.rect(0.25, 0.18, 0.55, 0.42, fill=0)
    c.rect(0.09, 0.86, 0.72, 0.96)
    c.line(0.64, 0.40, 0.80, 0.50, 0.05)
    c.line(0.80, 0.50, 0.80, 0.76, 0.05)
    c.arc(0.87, 0.75, 0.07, 0, 180, 0.05)
    c.line(0.94, 0.75, 0.94, 0.28, 0.05)
    c.line(0.94, 0.28, 0.76, 0.13, 0.05)


def wrench(c):
    c.line(0.25, 0.75, 0.66, 0.34, 0.15)
    c.circle(0.72, 0.28, 0.20)
    c.poly([(0.70, 0.30), (0.74, 0.02), (1.0, 0.02), (1.0, 0.26)], fill=0)
    c.circle(0.20, 0.80, 0.13)


def engine(c):
    c.rect(0.22, 0.32, 0.74, 0.74, radius=0.02)
    c.rect(0.38, 0.16, 0.60, 0.32)
    c.rect(0.30, 0.10, 0.68, 0.17)
    c.poly([(0.74, 0.42), (0.86, 0.42), (0.86, 0.32), (0.98, 0.32), (0.98, 0.74), (0.86, 0.74), (0.86, 0.64), (0.74, 0.64)])
    c.rect(0.04, 0.42, 0.22, 0.64)
    c.rect(0.02, 0.36, 0.08, 0.70)
    c.rect(0.32, 0.74, 0.62, 0.86)


def tyre(c):
    c.ring(0.5, 0.5, 0.42, 0.14)
    c.circle(0.5, 0.5, 0.12)
    for k in range(5):
        a = 2 * math.pi * k / 5 - math.pi / 2
        c.line(0.5, 0.5, 0.5 + 0.34 * math.cos(a), 0.5 + 0.34 * math.sin(a), 0.05)


def warning(c):
    c.poly([(0.5, 0.04), (0.98, 0.92), (0.02, 0.92)])
    c.poly([(0.45, 0.32), (0.55, 0.32), (0.53, 0.66), (0.47, 0.66)], fill=0)
    c.circle(0.5, 0.77, 0.055, fill=0)


def brake_disc(c):
    c.ring(0.5, 0.5, 0.42, 0.09)
    c.ring(0.5, 0.5, 0.26, 0.1)
    c.arc(0.5, 0.5, 0.50, 125, 235, 0.06)
    c.arc(0.5, 0.5, 0.50, 305, 55, 0.06)


def skid(c):
    # The stability lamp: a car with two wavy tracks behind it.
    c.poly([(0.32, 0.05), (0.68, 0.05), (0.74, 0.26), (0.84, 0.32), (0.84, 0.55), (0.16, 0.55), (0.16, 0.32), (0.26, 0.26)])
    c.rect(0.36, 0.11, 0.64, 0.25, fill=0)
    for x0 in (0.26, 0.74):
        pts = [(x0 + 0.075 * math.sin(i * 0.55), 0.60 + i * 0.038) for i in range(11)]
        for a, b in zip(pts, pts[1:]):
            c.line(a[0], a[1], b[0], b[1], 0.07)


def camera(c):
    c.rect(0.06, 0.28, 0.68, 0.76, radius=0.07)
    c.poly([(0.72, 0.46), (0.96, 0.30), (0.96, 0.74), (0.72, 0.58)])


def clock(c):
    c.ring(0.5, 0.5, 0.42, 0.1)
    c.line(0.5, 0.5, 0.5, 0.22, 0.08)
    c.line(0.5, 0.5, 0.70, 0.60, 0.08)


def crosshair_shell(c):
    # Shell, for the gun's reload state.
    c.poly([(0.36, 0.34), (0.5, 0.04), (0.64, 0.34)])
    c.rect(0.36, 0.36, 0.64, 0.86)
    c.rect(0.32, 0.88, 0.68, 0.96)


def turret(c):
    c.circle(0.42, 0.56, 0.28)
    c.rect(0.0, 0.46, 0.96, 0.56)
    c.rect(0.10, 0.70, 0.90, 0.88, radius=0.08)
    c.rect(0.0, 0.46, 0.45, 0.56)


def controller(c):
    c.rect(0.08, 0.32, 0.92, 0.74, radius=0.2)
    c.rect(0.20, 0.47, 0.40, 0.55, fill=0)
    c.rect(0.26, 0.41, 0.34, 0.61, fill=0)
    c.circle(0.68, 0.46, 0.05, fill=0)
    c.circle(0.78, 0.57, 0.05, fill=0)


ICONS = [thermometer, oilcan, fuel, wrench, engine, tyre, warning, brake_disc, skid, camera, clock, crosshair_shell, turret, controller]


def main(path):
    font = find_font()
    if not font:
        raise SystemExit("no font found (fc-match failed)")
    print("font:", font)
    glyphs(font)
    for i, fn in enumerate(ICONS):
        icon(0xE000 + i, fn)
    with open(path, "wb") as fp:
        fp.write(b"UIAT")
        fp.write(struct.pack("<III", ATLAS, ATLAS, len(entries)))
        for e in entries:
            fp.write(struct.pack("<I9f", *e))
        fp.write(atlas.tobytes())
    print("wrote %s: %d glyphs, rows up to %d" % (path, len(entries), pen_y + row_h))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "assets/ui.atlas")
