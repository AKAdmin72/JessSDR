#!/usr/bin/env python3
"""Draws the physical perfboard layout with every wire (docs/wiring_layout.svg
+ .png). Uses the pin tables from gen_wiring_sch.py, so it can never disagree
with the KiCad schematic. Not to exact scale - for orientation and wiring.
    python3 hardware/gen_layout_svg.py
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_wiring_sch as W  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_SVG = os.path.join(ROOT, "docs", "wiring_layout.svg")
OUT_PNG = os.path.join(ROOT, "docs", "wiring_layout.png")

g = 26                     # one 2.54 mm perfboard step, px
LX = 600                   # Tang J5 (left row) x
RX = LX + 8 * g            # Tang J6 (right row) x
Y20 = 470                  # y of pin 20 (top) of both rows


def ypin(n):               # header pin number 1..20 -> y (pin 20 at the top)
    return Y20 + (20 - n) * g


COL = {
    "ADC_CLK": "#e00000", "ADC_OTR": "#ff8c00",
    "+5V": "#c00000", "+3V3": "#ff6a00", "GND": "#000000",
}
ADC_DATA_COL = "#1f77b4"
TFT_COL = "#8e24aa"
ESP_COL = "#00897b"

svg = []


def add(s):
    svg.append(s)


def text(x, y, s, size=12, anchor="start", color="#000", weight="normal", rot=None):
    tr = f' transform="rotate({rot} {x} {y})"' if rot is not None else ""
    s = s.replace("&", "&amp;").replace("<", "&lt;")
    add(f'<text x="{x}" y="{y}" font-family="DejaVu Sans, sans-serif" font-size="{size}" '
        f'text-anchor="{anchor}" fill="{color}" font-weight="{weight}"{tr}>{s}</text>')


def rect(x, y, w, h, fill, stroke="#333", rx=6, sw=2, op=1.0):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
        f'fill-opacity="{op}" stroke="{stroke}" stroke-width="{sw}"/>')


def pin(x, y, used=True):
    add(f'<circle cx="{x}" cy="{y}" r="6" fill="{"#d4af37" if used else "#999"}" stroke="#333" stroke-width="1.5"/>')


def wire(pts, color, w=3.5):
    d = " ".join(f"{x},{y}" for x, y in pts)
    add(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{w}" '
        f'stroke-linejoin="round" stroke-linecap="round" opacity="0.9"/>')


def net_color(net):
    if net in COL:
        return COL[net]
    if net.startswith("ADC_D"):
        return ADC_DATA_COL
    if net.startswith("TFT_"):
        return TFT_COL
    if net.startswith("ESP_"):
        return ESP_COL
    return "#555"


W_, H_ = 1500, 1290
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_}" height="{H_}" viewBox="0 0 {W_} {H_}">')
rect(0, 0, W_, H_, "#ffffff", stroke="none", rx=0)
# perfboard
rect(60, 90, W_ - 120, H_ - 290, "#2e7d32", stroke="#1b5e20", rx=4, op=0.18)
text(W_ / 2, 40, "Tang Nano 20K + AD9226 + ILI9341 (+ ESP32-S3 reserved) - wiring layout, view from the component side",
     18, "middle", weight="bold")
text(W_ / 2, 64, "Generated from hardware/gen_wiring_sch.py (same tables as the KiCad schematic and adc_test.cst). Not to exact scale.",
     12, "middle", "#444")

# ---------------------------------------------------------------- Tang
rect(LX - 22, Y20 - 70, RX - LX + 44, 19 * g + 120, "#212121", stroke="#000", rx=10)
rect(LX + 40, Y20 - 64, RX - LX - 80, 46, "#bdbdbd", stroke="#555", rx=3)
text((LX + RX) / 2, Y20 - 36, "HDMI (top)", 12, "middle")
rect(LX + 60, Y20 + 19 * g + 18, RX - LX - 120, 30, "#bdbdbd", stroke="#555", rx=8)
text((LX + RX) / 2, Y20 + 19 * g + 38, "USB-C", 12, "middle")
text((LX + RX) / 2 + 5, Y20 + 9.5 * g, "TANG NANO 20K  (chip side up)", 13, "middle", "#fff", "bold", rot=-90)
text(LX, Y20 - 82, "J5", 14, "middle", "#000", "bold")
text(RX, Y20 - 82, "J6", 14, "middle", "#000", "bold")

for n in range(1, 21):
    y = ypin(n)
    lbl5, _ = W.J5[n - 1]
    lbl6, _ = W.J6[n - 1]
    net5 = W.TANG_NETS.get(("J5", n))
    net6 = W.TANG_NETS.get(("J6", n))
    pin(LX, y, net5 is not None)
    pin(RX, y, net6 is not None)
    text(LX + 12, y + 4, f"{n} \u00b7 {lbl5}", 10, "start", "#fff")
    text(RX - 12, y + 4, f"{lbl6} \u00b7 {n}", 10, "end", "#fff")

# ---------------------------------------------------------------- ESP (left)
EX = LX - 9 * g
esp_pins = [(n, net) for (h, n), net in W.TANG_NETS.items() if h == "J5" and net.startswith("ESP_")]
esp_pins.append((15, "GND"))
rect(EX - 230, ypin(20) - 40, 230 + 18, ypin(4) - ypin(20) + 80, "#e0f2f1", stroke=ESP_COL, rx=8)
text(EX - 115, ypin(20) - 14, "ESP32-S3 board", 15, "middle", ESP_COL, "bold")
text(EX - 115, ypin(4) + 32, "(reserved - GPIO numbers", 10, "middle", "#555")
text(EX - 115, ypin(4) + 45, "picked to match its header)", 10, "middle", "#555")
for n, net in sorted(esp_pins, reverse=True):
    y = ypin(n)
    c = net_color(net)
    wire([(LX, y), (EX, y)], c)
    pin(EX, y)
    text(EX - 12, y + 4, net.replace("ESP_", ""), 11, "end", c, "bold")
    text((EX + LX) / 2, y - 5, f"J5-{n}", 9, "middle", "#333")

# ---------------------------------------------------------------- ADC (right)
AX = RX + 8 * g            # outer column of P2 (column nearest the Tang)
AX2 = AX + g               # inner column
YM = ypin(14) + 3 * g      # y of module row 1 (+5V row) -> rows 3..9 face J6-14..J6-1
rect(AX - 22, YM - 3 * g, 330, 16 * g, "#263238", stroke="#000", rx=8)
text(AX + 170, YM + 2 * g, "AD9226 module", 15, "middle", "#fff", "bold")
text(AX + 170, YM + 2 * g + 20, "12-bit 65 MSPS", 11, "middle", "#ccc")
text(AX + 170, YM + 6 * g, "5V", 12, "middle", "#ccc")
rect(AX + 290, YM + 9 * g - 14, 70, 28, "#d4af37", stroke="#7a5c00", rx=4)
text(AX + 325, YM + 9 * g + 4, "SMA IN", 11, "middle", "#000", "bold")
text(AX - 2, YM - 3 * g + 18, "P2 header faces the Tang, +5V row at the top", 10, "start", "#fff")
p2_pos = {}
for r in range(10):
    y = YM + r * g
    for col, names, x in ((0, W.ADC_OUTER, AX), (1, W.ADC_INNER, AX2)):
        name = names[r]
        p2_pos[(col, r)] = (x, y, name)
        pin(x, y)
        if col == 1:
            text(x + 12, y + 4, name, 10, "start", "#fff")

# wires Tang J6 -> P2, routed in lanes between the boards
adc = []
for (h, n), net in W.TANG_NETS.items():
    if h == "J6" and net.startswith("ADC_"):
        adc.append((n, net))
adc.sort(reverse=True)     # J6-14 first (top)
for i, (n, net) in enumerate(adc):
    name = W.adc_net  # noqa
    tgt = next((v for v in p2_pos.values() if W.adc_net(v[2]) == net), None)
    x2, y2, pname = tgt
    y1 = ypin(n)
    c = net_color(net)
    lane = RX + g * 1.2 + i * (g * 0.38)
    if x2 == AX2:          # inner column: come in just below the outer pin
        pts = [(RX, y1), (lane, y1), (lane, y2 + 9), (AX + 10, y2 + 9), (x2, y2)]
    else:
        pts = [(RX, y1), (lane, y1), (lane, y2), (x2, y2)]
    wire(pts, c, 5 if net == "ADC_CLK" else 3.5)
    text(RX + 14, y1 - 5, net.replace("ADC_", ""), 10, "start", c, "bold")
    if x2 == AX:
        text(x2 - 10, y2 - 7, pname, 10, "end", c, "bold")
# module power: +5V/GND from J5-1/J5-2 (under the Tang)
y5, yg = ypin(1), ypin(2)
yb = ypin(1) + 2.6 * g
wire([(LX, y5), (LX - g * 0.8, y5), (LX - g * 0.8, yb), (AX - g * 1.6, yb), (AX - g * 1.6, YM), (AX, YM)], COL["+5V"], 4)
wire([(LX, yg), (LX - g * 1.3, yg), (LX - g * 1.3, yb + 16), (AX - g * 2.1, yb + 16),
      (AX - g * 2.1, YM + g + 9), (AX + 10, YM + g + 9), (AX2, YM + g)], COL["GND"], 4)
text((LX + AX) / 2, yb - 6, "+5V  (J5-1 -> P2 +5V, runs under the Tang)", 11, "middle", COL["+5V"], "bold")
text((LX + AX) / 2, yb + 32, "GND  (J5-2 -> P2 GND)", 11, "middle", "#000", "bold")

# ---------------------------------------------------------------- TFT (top)
TY = Y20 - 6 * g           # y of the TFT header pins
TX1 = LX - 1 * g           # x of TFT pin 1
rect(TX1 - 40, TY - 175, 15 * g + 60, 160, "#d32f2f", stroke="#7f0000", rx=8)
rect(TX1 + 20, TY - 160, 15 * g - 60, 110, "#111", stroke="#555", rx=2)
text(TX1 + 7 * g, TY - 100, "ILI9341 2.8\" TFT", 15, "middle", "#fff", "bold")
text(TX1 + 7 * g, TY - 80, "(header along its bottom edge)", 10, "middle", "#ccc")
tpos = {}
for k, name in enumerate(W.TFT, start=1):
    x = TX1 + (k - 1) * g
    tpos[name] = x
    pin(x, TY, W.TFT_NETS.get(name) is not None)
    text(x + 4, TY - 12, name, 9, "start", "#fff", rot=-60)
# LED -> VCC bridge on the TFT side
wire([(tpos["LED"], TY), (tpos["LED"], TY - 14), (tpos["VCC"], TY - 14), (tpos["VCC"], TY)], COL["+3V3"], 3)
tft_links = [("VCC", "J6", 19), ("GND", "J6", 20), ("CS", "J5", 18), ("RESET", "J6", 18),
             ("DC", "J6", 17), ("SDI(MOSI)", "J6", 16), ("SCK", "J6", 15)]
for j, (name, h, n) in enumerate(tft_links):
    net = W.TANG_NETS[(h, n)]
    c = net_color(net)
    x0 = LX if h == "J5" else RX
    y0 = ypin(n)
    xt = tpos[name]
    if h == "J5":
        lane_x = LX - g * 0.6
        lane_y = TY + g * 1.2
        pts = [(x0, y0), (lane_x, y0), (lane_x, lane_y), (xt, lane_y), (xt, TY)]
    else:
        lane_x = RX + g * (0.55 + 0.33 * j)
        lane_y = TY + g * (3.4 - 0.33 * j)
        pts = [(x0, y0), (lane_x, y0), (lane_x, lane_y), (xt, lane_y), (xt, TY)]
    wire(pts, c, 3.5)
    text(xt + 4, TY + 22, net.replace("TFT_", ""), 9, "start", c, "bold", rot=60)

# ---------------------------------------------------------------- legend
ly = H_ - 160
items = [("ADC data D0..D11", ADC_DATA_COL), ("ADC CLK (keep short!)", COL["ADC_CLK"]),
         ("ADC OTR", COL["ADC_OTR"]), ("TFT signals", TFT_COL), ("ESP32 link (reserved)", ESP_COL),
         ("+5V", COL["+5V"]), ("+3V3", COL["+3V3"]), ("GND", COL["GND"])]
for i, (lbl, c) in enumerate(items):
    x = 70 + (i % 4) * 340
    y = ly + (i // 4) * 26
    wire([(x, y), (x + 40, y)], c, 5)
    text(x + 50, y + 4, lbl, 13)
notes = [
    "Tang mounted HDMI end UP: J5 = left row (ESP side), J6 = right row (ADC side). Pin text \"20 \u00b7 IO52\" = header pin 20, FPGA pin 52.",
    "AD9226 P2 order is from the seller photo - VERIFY before powering. Measure the module's 3V3 pad (logic must be 3.3 V).",
    "TFT: LED pin bridged to VCC on the TFT side; SDO and touch pins unconnected.  microSD slot on the Tang must stay EMPTY.",
    "Grey pins = not connected (76/86 BL616, 49 LCD backlight, 51/54/55/56 audio amp, 72 spare).",
]
for i, s in enumerate(notes):
    text(70, ly + 70 + i * 18, s, 12, "start", "#333")
add("</svg>")

os.makedirs(os.path.dirname(OUT_SVG), exist_ok=True)
with open(OUT_SVG, "w") as f:
    f.write("\n".join(svg))
subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", OUT_SVG, OUT_PNG], check=True)
print("written", OUT_SVG, OUT_PNG)
