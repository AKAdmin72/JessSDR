#!/usr/bin/env python3
"""JessSDR boot splash (480x320): Jessica, the cat the project is named after.

Drawing her from scratch never looked like her, so the splash is built from
her real face: tools/jessica_cutout.png (cut out of a 2026-11-10 photo with
cv2.grabCut, eyes filled back in, only the cat kept), stylized five ways:
  1 photo    the photo itself, warm light behind her
  2 cartoon  edge-preserving smoothing + few colours + ink lines
  3 oil      oil painting (cv2.xphoto.oilPainting)
  4 pencil   colour pencil sketch (cv2.pencilSketch)
  5 pixel    pixel art (1/5 resolution, 24 colours)

Usage:
  python3 esp32/tools/gen_splash.py sheet  -> esp32/tools/splash_v1..5.png + splash_sheet.png
  python3 esp32/tools/gen_splash.py N      -> style N: esp32/main/splash_img.h + esp32/tools/splash.png
splash_img.h = palette (<= 256 colours, RGB565 big-endian) + (count, index) runs.
"""
import math, os, random, sys
import cairo
import cv2
import numpy as np
from PIL import Image, ImageDraw

W, H = 480, 320
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_H = os.path.join(HERE, "..", "main", "splash_img.h")
CUTOUT = os.path.join(HERE, "jessica_cutout.png")
STYLES = {1: "photo", 2: "cartoon", 3: "oil", 4: "pencil", 5: "pixel"}
BG0, BG1 = (0x0B, 0x14, 0x30), (0x1D, 0x2B, 0x5C)
CAT_W, CAT_X, CAT_Y = 292, 2, 44           # placement of the cutout on the screen


# ---------------------------------------------------------------------------
def surface_to_pil(surf):
    surf.flush()
    arr = np.frombuffer(bytes(surf.get_data()), np.uint8).reshape(H, W, 4)
    return Image.fromarray(arr[..., [2, 1, 0]].copy(), "RGB")


def pil_to_surface(img):
    arr = np.array(img.convert("RGB"))
    bgra = np.dstack([arr[..., 2], arr[..., 1], arr[..., 0], np.full(arr.shape[:2], 255, np.uint8)])
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, W, H)
    surf.get_data()[:] = bgra.tobytes()
    surf.mark_dirty()
    return surf


def background(glow=False):
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, W, H)
    c = cairo.Context(surf)
    g = cairo.LinearGradient(0, 0, 0, H)
    g.add_color_stop_rgb(0, *(x / 255 for x in BG0))
    g.add_color_stop_rgb(1, *(x / 255 for x in BG1))
    c.set_source(g)
    c.paint()
    r = random.Random(7)
    for _ in range(40):
        x, y = r.uniform(0, W), r.uniform(0, H * 0.7)
        c.set_source_rgba(1, 1, 1, r.uniform(0.25, 0.7))
        c.arc(x, y, r.uniform(0.6, 1.3), 0, 2 * math.pi)
        c.fill()
    if glow:
        g = cairo.RadialGradient(150, 160, 20, 150, 160, 170)
        g.add_color_stop_rgba(0, 1.0, 0.75, 0.45, 0.35)
        g.add_color_stop_rgba(1, 1.0, 0.75, 0.45, 0.0)
        c.set_source(g)
        c.paint()
    r = random.Random(3)
    pts = []
    for x in range(0, W + 1, 3):
        y = 306 + r.gauss(0, 1.6)
        for f0, a in ((330, 18), (382, 9), (430, 13), (300, 7)):
            y -= a * math.exp(-((x - f0) / 4.0) ** 2)
        pts.append((x, y))
    c.set_line_width(1.6)
    c.set_source_rgba(0.35, 0.85, 1.0, 0.85)
    c.move_to(*pts[0])
    for p in pts[1:]:
        c.line_to(*p)
    c.stroke_preserve()
    c.line_to(W, H); c.line_to(0, H); c.close_path()
    c.set_source_rgba(0.35, 0.85, 1.0, 0.12)
    c.fill()
    return surface_to_pil(surf)


def title(img, mono=False):
    surf = pil_to_surface(img)
    c = cairo.Context(surf)
    fam = "DejaVu Sans Mono" if mono else "DejaVu Sans"

    def text(s, x, y, size, color, bold=True, alpha=1.0):
        c.select_font_face(fam, cairo.FONT_SLANT_NORMAL,
                           cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
        c.set_font_size(size)
        c.move_to(x, y)
        c.set_source_rgba(*color, alpha)
        c.show_text(s)
        return c.text_extents(s)
    tx, ty = 300, 150
    e = text("Jess", tx + 2, ty + 3, 37, (0, 0, 0), alpha=0.5)
    text("SDR", tx + 2 + e.x_advance + 1, ty + 3, 37, (0, 0, 0), alpha=0.5)
    e = text("Jess", tx, ty, 37, (0.95, 0.71, 0.43))
    text("SDR", tx + e.x_advance + 1, ty, 37, (0.50, 0.83, 1.0))
    text("software defined radio", tx + 2, ty + 24, 12.5, (0.85, 0.89, 1.0), bold=False)
    return surface_to_pil(surf)


# ---------------------------------------------------------------------------
def load_cutout():
    rgba = cv2.imread(CUTOUT, cv2.IMREAD_UNCHANGED)
    bgr, a = rgba[..., :3].copy(), rgba[..., 3]
    # transparent area -> background colour, so filters do not pull in junk
    bgr[a < 8] = np.array([87, 40, 27], np.uint8)
    # the photo crop ends straight on the right: fade it out softly
    h, w = a.shape
    fade = np.clip((w - 1 - np.arange(w)) / 45.0, 0, 1)[None, :]
    a = (a * fade).astype(np.uint8)
    return bgr, a


def saturate(bgr, k):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(float)
    hsv[..., 1] = np.clip(hsv[..., 1] * k, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def kmeans_colors(img, k):
    Z = img.reshape(-1, 3).astype(np.float32)
    _, lab, ctr = cv2.kmeans(Z, k, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0),
                             3, cv2.KMEANS_PP_CENTERS)
    return ctr[lab.flatten()].reshape(img.shape).astype(np.uint8)


def stylize(n, bgr):
    if n == 1:
        out = bgr.astype(float) * np.array([0.96, 1.0, 1.06]) * 1.04 + 4    # a bit warmer
        return np.clip(out, 0, 255).astype(np.uint8)
    if n == 2:
        sm = bgr
        for _ in range(4):
            sm = cv2.bilateralFilter(sm, 9, 40, 7)
        sm = saturate(cv2.convertScaleAbs(sm, alpha=1.08, beta=6), 1.45)
        post = kmeans_colors(sm, 10)
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(sm, cv2.COLOR_BGR2GRAY), (0, 0), 2.0), 40, 90)
        edges = cv2.GaussianBlur(edges, (0, 0), 0.7)
        post = post.astype(float) * (1 - 0.55 * (edges[..., None] / 255.0))
        return post.clip(0, 255).astype(np.uint8)
    if n == 3:
        return saturate(cv2.xphoto.oilPainting(cv2.convertScaleAbs(bgr, alpha=1.06, beta=6), 6, 1), 1.35)
    if n == 4:
        _, col = cv2.pencilSketch(bgr, sigma_s=40, sigma_r=0.06, shade_factor=0.04)
        return saturate(cv2.addWeighted(col, 0.5, cv2.stylization(bgr, sigma_s=30, sigma_r=0.3), 0.5, 12), 1.5)
    if n == 5:
        h, w = bgr.shape[:2]
        small = kmeans_colors(saturate(cv2.resize(bgr, (w // 5, h // 5), interpolation=cv2.INTER_AREA), 1.3), 24)
        return cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)
    raise ValueError(n)


def render(n):
    bgr, a = load_cutout()
    st = stylize(n, bgr)
    if n == 5:                                    # blocky alpha too
        h, w = a.shape
        a = cv2.resize(cv2.resize(a, (w // 5, h // 5), interpolation=cv2.INTER_AREA), (w, h),
                       interpolation=cv2.INTER_NEAREST)
        a = np.where(a > 110, 255, 0).astype(np.uint8)
    h, w = a.shape
    size = (CAT_W, round(h * CAT_W / w))
    cat = Image.fromarray(np.dstack([st[..., ::-1], a]), "RGBA").resize(
        size, Image.NEAREST if n == 5 else Image.LANCZOS)
    img = background(glow=(n == 1)).convert("RGBA")
    img.alpha_composite(cat, (CAT_X, CAT_Y))
    return title(img.convert("RGB"), mono=(n == 5))


# ---------------------------------------------------------------------------
def rgb565_be(r, g, b):
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return ((v >> 8) | (v << 8)) & 0xFFFF


def export(img, n):
    q = img.quantize(colors=256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    pal = q.getpalette()[:768]
    idx = list(q.get_flattened_data()) if hasattr(q, "get_flattened_data") else list(q.getdata())
    runs, i = [], 0
    while i < len(idx):
        j = i
        while j < len(idx) and idx[j] == idx[i] and j - i < 255:
            j += 1
        runs.append((j - i, idx[i]))
        i = j
    with open(OUT_H, "w") as f:
        f.write("// Generated by esp32/tools/gen_splash.py - do not edit.\n")
        f.write(f"// JessSDR boot splash {W}x{H}, style {n} ({STYLES[n]}): {len(runs)} runs.\n")
        f.write("#pragma once\n#include <stdint.h>\n")
        f.write(f"#define SPLASH_W {W}\n#define SPLASH_H {H}\n")
        f.write("static const uint16_t splash_pal[256] = {\n")
        for k in range(256):
            r, g_, b = pal[3 * k:3 * k + 3] if 3 * k + 2 < len(pal) else (0, 0, 0)
            f.write(f"0x{rgb565_be(r, g_, b):04X},{chr(10) if k % 12 == 11 else ''}")
        f.write("\n};\n")
        f.write(f"#define SPLASH_NRUNS {len(runs)}\n")
        f.write("static const uint8_t splash_rle[2 * SPLASH_NRUNS] = {  // (count, index)\n")
        for k, (cnt, v) in enumerate(runs):
            f.write(f"{cnt},{v},{chr(10) if k % 16 == 15 else ''}")
        f.write("\n};\n")
    img.save(os.path.join(HERE, "splash.png"))
    print(f"style {n} ({STYLES[n]}): {OUT_H}, {len(runs)} runs ({2 * len(runs)} bytes)")


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else "sheet"
    if arg == "sheet":
        sheet = Image.new("RGB", (2 * W + 30, 3 * H + 40), (40, 40, 40))
        d = ImageDraw.Draw(sheet)
        for k, n in enumerate(STYLES):
            im = render(n)
            im.save(os.path.join(HERE, f"splash_v{n}.png"))
            x, y = 10 + (k % 2) * (W + 10), 10 + (k // 2) * (H + 10)
            sheet.paste(im, (x, y))
            d.rectangle((x, y, x + 34, y + 28), fill=(255, 255, 255))
            d.text((x + 10, y + 3), str(n), fill=(0, 0, 0), font_size=22)
        sheet.save(os.path.join(HERE, "splash_sheet.png"))
        print("splash_v1..5.png, splash_sheet.png")
    else:
        export(render(int(arg)), int(arg))
