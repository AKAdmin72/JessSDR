#!/usr/bin/env python3
"""JessSDR - the ONE schematic of the whole receiver, as built (KiCad 9):
    python3 hardware/gen_schematic.py   ->  hardware/jesssdr/ (.kicad_sch + .pdf)

One A2 sheet: antenna -> 40m BPF -> AD9226 (+ its AD8138 input stage with
the IW5ALZ gain mod) -> Tang Nano 20K <-> ESP32-S3 DevKitC -> 4" ST7796S,
optical encoder LPD3806 + pull-ups, buttons, ESP supply filter.

This file is the single source of truth for every pin: Tang header tables
(official schematic docs/Tang_Nano_20K_3923_Schematics.pdf), the .cst files,
ESP pins = esp32/main/{lcd,fpga,input}.c. Edit here, re-run, then
    kicad-cli sch erc hardware/jesssdr/jesssdr.kicad_sch
Replaces the old wiring/ (Tang-only build with ILI9341 - see tag
working-2026-11-07), system/ and frontend_mod/ drawings.
"""
import os
import re
import subprocess
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "jesssdr")
PROJECT = "jesssdr"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "jesssdr/schematic"))
KLIB = "/usr/share/kicad/symbols"
P = 2.54
PL = 5.08  # pin length of the module boxes (room for "J6-10"-style numbers)
FONT = "(effects (font (size 1.27 1.27)))"
HIDE = "(effects (font (size 1.27 1.27)) (hide yes))"
_ctr = [0]


def uid():
    _ctr[0] += 1
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"jesssdr/schematic/{_ctr[0]}"))


def esc(s):
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


# ===========================================================================
# PIN TABLES
# ---------------------------------------------------------------------------
# Tang Nano 20K headers, pin 1 (USB-C end) .. 20, from the official schematic
# sheet 1 (J6 / J5, Conn_01x20). (pin label, onboard use); IOnn = FPGA pin nn.
J6 = [
    ("IO73", ""), ("IO74", ""), ("IO75", "BL616 SPI_MISO"), ("IO85", "SD D1 10k pull-up"),
    ("IO77", "RGB-LCD CK"), ("IO15", "LED0"), ("IO16", "LED1"), ("IO27", "RGB-LCD B7"),
    ("IO28", "RGB-LCD B6"), ("IO25", "RGB-LCD HS"), ("IO26", "HDMI HPD divider"), ("IO29", "RGB-LCD B5"),
    ("IO30", "RGB-LCD B4"), ("IO31", "RGB-LCD B3"), ("IO17", "LED2"), ("IO20", "LED5"),
    ("IO19", "LED4"), ("IO18", "LED3"), ("3V3", ""), ("GND", ""),
]
J5 = [
    ("5V", ""), ("GND", ""), ("IO76", "BL616 SPI_MOSI"), ("IO80", "SD D2 10k pull-up"),
    ("IO42", "RGB-LCD R3"), ("IO41", "RGB-LCD R4"), ("IO56", "I2S BCLK (amp)"), ("IO54", "I2S DIN (amp)"),
    ("IO51", "PA_EN (amp)"), ("IO48", "RGB-LCD DE"), ("IO55", "I2S LRCK (amp)"), ("IO49", "LCD BL boost EN"),
    ("IO86", "BL616 SPI_DIR"), ("IO79", "WS2812"), ("GND", ""), ("3V3", ""),
    ("IO72", ""), ("IO71", ""), ("IO53", "HDMI DDC SCL"), ("IO52", "HDMI DDC SDA"),
]

# Tang mounted HDMI end UP: J5 = left row (faces the ESP), J6 = right row
# (faces the AD9226 P2 header), pin 20 of both rows at the top.
# AD9226: all 14 signals on J6-14..J6-1 in the P2 header's own order
# (D0..D6, CLK, OTR, D7..D11); CLK/OTR on the LED1/LED0 pins.
TANG_NETS = {
    ("J6", 14): "ADC_D0", ("J6", 13): "ADC_D1", ("J6", 12): "ADC_D2", ("J6", 11): "ADC_D3",
    ("J6", 10): "ADC_D4", ("J6", 9): "ADC_D5", ("J6", 8): "ADC_D6",
    ("J6", 7): "ADC_CLK", ("J6", 6): "ADC_OTR",
    ("J6", 5): "ADC_D7", ("J6", 4): "ADC_D8", ("J6", 3): "ADC_D9", ("J6", 2): "ADC_D10",
    ("J6", 1): "ADC_D11",
    ("J5", 1): "+5V", ("J5", 2): "GND", ("J5", 15): "GND", ("J6", 20): "GND",
}

# ESP32-S3 link: ESP left of the Tang, its right row J3 faces Tang J5 and is
# 2 pins longer at the top -> J5-n sits opposite J3-(23-n): every link wire is
# one straight jumper. J5-1 (+5V) faces J3-22 (GND) - never bridge!
ESP_LINK = {
    ("J5", 19): "ESP_SCK", ("J5", 18): "ESP_MISO", ("J5", 17): "ESP_I2S_WS",
    ("J5", 14): "ESP_MOSI", ("J5", 6): "ESP_I2S_BCK", ("J5", 5): "ESP_I2S_DATA",
    ("J5", 4): "ESP_CS_N",
}
ESP_GPIO = {"ESP_SCK": "GPIO1", "ESP_MISO": "GPIO2", "ESP_I2S_WS": "GPIO42", "ESP_MOSI": "GPIO39",
            "ESP_I2S_BCK": "GPIO47", "ESP_I2S_DATA": "GPIO21", "ESP_CS_N": "GPIO20"}
TANG_NETS.update(ESP_LINK)

# AD9226 module 2x10 header P2 (seller photo order, confirmed by the working
# build, BIT_REVERSE=0). Row 1 = "+5V" end. outer column / inner column.
ADC_OUTER = ["+5V", "GND", "D0", "D2", "D4", "D6", "D8", "D10", "OTR", "GND"]
ADC_INNER = ["+5V", "GND", "CLK", "D1", "D3", "D5", "D7", "D9", "D11", "GND"]


def adc_net(name):
    return name if name in ("+5V", "GND") else "ADC_" + name


# ESP32-S3-DevKitC N16R8, rows top to bottom (antenna end up).
# N16R8 = octal PSRAM: GPIO33..37 are taken.
ESP_J1 = ["3V3", "3V3", "RST", "GPIO4", "GPIO5", "GPIO6", "GPIO7", "GPIO15", "GPIO16", "GPIO17",
          "GPIO18", "GPIO8", "GPIO3", "GPIO46", "GPIO9", "GPIO10", "GPIO11", "GPIO12", "GPIO13",
          "GPIO14", "5V", "GND"]
ESP_J3 = ["GND", "GPIO43 TX", "GPIO44 RX", "GPIO1", "GPIO2", "GPIO42", "GPIO41", "GPIO40",
          "GPIO39", "GPIO38", "GPIO37", "GPIO36", "GPIO35", "GPIO0 BOOT", "GPIO45", "GPIO48 RGB",
          "GPIO47", "GPIO21", "GPIO20", "GPIO19", "GND", "GND"]
ESP_NETS = {
    "3V3": "+3V3_ESP", "5V": "+5V_ESP", "GND": "GND",
    **{g: n for n, g in ESP_GPIO.items()},                    # FPGA link (fpga.c)
    "GPIO4": "LCD_CS", "GPIO5": "LCD_RST", "GPIO6": "LCD_DC",  # display (lcd.c)
    "GPIO7": "LCD_MOSI", "GPIO15": "LCD_SCK", "GPIO16": "LCD_LED",
    "GPIO17": "ENC_A", "GPIO18": "ENC_B", "GPIO8": "BTN_SEL",  # input.c
    "GPIO12": "BTN_BACK", "GPIO13": "BTN_PREV", "GPIO14": "BTN_NEXT",
}

TFT = ["VCC", "GND", "CS", "RESET", "DC/RS", "SDI(MOSI)", "SCK", "LED",
       "SDO(MISO)", "T_CLK", "T_CS", "T_DIN", "T_DO", "T_IRQ"]
TFT_NETS = {"VCC": "+3V3_ESP", "GND": "GND", "CS": "LCD_CS", "RESET": "LCD_RST",
            "DC/RS": "LCD_DC", "SDI(MOSI)": "LCD_MOSI", "SCK": "LCD_SCK", "LED": "LCD_LED"}


# ===========================================================================
# KiCad writer
# ---------------------------------------------------------------------------
def lib_get(fname, name):
    s = open(os.path.join(KLIB, fname)).read()
    i = s.index(f'(symbol "{name}"')
    d = 0
    for j in range(i, len(s)):
        d += s[j] == "("
        d -= s[j] == ")"
        if d == 0:
            return s[i:j + 1]


def lib_symbol(name, ref, value, pins, half_w, half_h, prefix):
    """Module box; pins = (number, name, side 'L'/'R', row), row 0 at top."""
    body = (f'(rectangle (start {-half_w:.2f} {half_h:.2f}) (end {half_w:.2f} {-half_h:.2f})'
            ' (stroke (width 0.254) (type default)) (fill (type background)))')
    ps = []
    for num, pname, side, row in pins:
        y = half_h - P * (row + 1)
        x, a = (-half_w - PL, 0) if side == "L" else (half_w + PL, 180)
        ps.append(f'(pin passive line (at {x:.2f} {y:.2f} {a}) (length {PL})'
                  f' (name "{esc(pname)}" {FONT}) (number "{esc(num)}" {FONT}))')
    return (f'(symbol "{prefix}{name}" (pin_names (offset 1.016)) (exclude_from_sim no)'
            f' (in_bom yes) (on_board yes)'
            f' (property "Reference" "{ref}" (at 0 {half_h + 1.27:.2f} 0) {FONT})'
            f' (property "Value" "{esc(value)}" (at 0 {-half_h - 1.27:.2f} 0) {FONT})'
            f' (property "Footprint" "" (at 0 0 0) {HIDE})'
            f' (property "Datasheet" "" (at 0 0 0) {HIDE})'
            f' (symbol "{name}_0_1" {body})'
            f' (symbol "{name}_1_1" {" ".join(ps)}))')


class Sheet:
    def __init__(self):
        self.items, self.libs, self.libfile, self.lib_ids = [], [], [], set()

    def _instance(self, lib_id, ref, value, ax, ay, rot, pin_nums, ref_at, val_at):
        pin_u = " ".join(f'(pin "{esc(n)}" (uuid "{uid()}"))' for n in pin_nums)
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {ax:.2f} {ay:.2f} {rot}) (unit 1)'
            f' (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}")'
            f' (property "Reference" "{ref}" (at {ref_at[0]:.2f} {ref_at[1]:.2f} {rot % 180}) {FONT})'
            f' (property "Value" "{esc(value)}" (at {val_at[0]:.2f} {val_at[1]:.2f} {rot % 180}) {FONT})'
            f' (property "Footprint" "" (at {ax:.2f} {ay:.2f} 0) {HIDE})'
            f' (property "Datasheet" "" (at {ax:.2f} {ay:.2f} 0) {HIDE})'
            f' {pin_u}'
            f' (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (reference "{ref}") (unit 1)))))')

    def stub(self, px, py, dx, dy, net, n=3):
        """Wire n grid steps out of a pin end, then a label (or a no-connect)."""
        if net is None:
            self.items.append(f'(no_connect (at {px:.2f} {py:.2f}) (uuid "{uid()}"))')
            return
        ex, ey = px + dx * n * P, py + dy * n * P
        self.wire((px, py), (ex, ey))
        ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(dx, dy)]
        just = "right" if (dx, dy) in ((-1, 0), (0, 1)) else "left"
        self.label(net, ex, ey, ang, just)

    def box(self, libname, ref, value, at, pins, half_w, half_h, nets, direct=()):
        """Module drawn as a box; pins = (number, name, side, row). Pins in
        `direct` get no stub (the caller wires them); returns pin end points."""
        self.libs.append(lib_symbol(libname, ref, value, pins, half_w, half_h, PROJECT + ":"))
        self.libfile.append(lib_symbol(libname, ref, value, pins, half_w, half_h, ""))
        ax, ay = at
        self._instance(f"{PROJECT}:{libname}", ref, value, ax, ay, 0, [p[0] for p in pins],
                       (ax, ay - half_h - 1.27), (ax, ay + half_h + 1.27))
        ends = {}
        for num, _, side, row in pins:
            y = ay - (half_h - P * (row + 1))
            x = ax - half_w - PL if side == "L" else ax + half_w + PL
            ends[num] = (x, y)
            if num not in direct:
                self.stub(x, y, -1 if side == "L" else 1, 0, nets.get(num))
        return ends

    def part(self, lib_id, ref, value, at, rot=0, nets=None, ref_at=None, val_at=None, stub_n=2):
        """KiCad library symbol. nets: pin number -> net (labelled stub), or
        absent = the caller wires that pin itself. Returns pin positions."""
        lib, name = lib_id.split(":")
        t = lib_get(lib + ".kicad_sym", name)
        if lib_id not in self.lib_ids:
            self.lib_ids.add(lib_id)
            self.libs.append(t.replace(f'(symbol "{name}"', f'(symbol "{lib_id}"', 1))
        ax, ay = at
        pos = {}
        for x, y, a, num in re.findall(
                r'\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\)(?:.|\n)*?\(number "([^"]+)"', t):
            u, v, a = float(x), -float(y), (int(a) + rot) % 360   # screen offset, y down
            for _ in range(rot // 90):
                u, v = v, -u
            # pin angle = direction from the pin end into the body; stub goes the other way
            d = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[a]
            pos[num] = (ax + u, ay + v, d)
        self._instance(lib_id, ref, value, ax, ay, rot, list(pos),
                       ref_at or (ax + 2.54, ay - 3.81), val_at or (ax + 2.54, ay + 3.81))
        for num, net in (nets or {}).items():
            px, py, (dx, dy) = pos[num]
            self.stub(px, py, dx, dy, net, stub_n)
        return {k: (x, y) for k, (x, y, _) in pos.items()}

    def hres(self, lib_id, ref, value, cx, y):
        """Horizontal R/C centred at cx, text above; returns (left, right) pin ends."""
        p = self.part(lib_id, ref, value, (cx, y), rot=90,
                      ref_at=(cx, y - 4.45), val_at=(cx, y - 2.67))
        return tuple(sorted(p.values()))

    def vres(self, lib_id, ref, value, x, cy, net_bottom, text_dx=6.35):
        """Vertical R/C centred at cy, bottom pin -> labelled stub; returns top pin end."""
        p = self.part(lib_id, ref, value, (x, cy), ref_at=(x + text_dx, cy - 0.89),
                      val_at=(x + text_dx, cy + 0.89))
        top, bot = sorted(p.values(), key=lambda q: q[1])
        self.stub(bot[0], bot[1], 0, 1, net_bottom, 1)
        return top

    def wire(self, *pts):
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            self.items.append(f'(wire (pts (xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f}))'
                              f' (stroke (width 0) (type default)) (uuid "{uid()}"))')

    def junction(self, x, y):
        self.items.append(f'(junction (at {x:.2f} {y:.2f}) (diameter 0) (color 0 0 0 0) (uuid "{uid()}"))')

    def label(self, net, x, y, ang=0, just="left"):
        self.items.append(f'(label "{esc(net)}" (at {x:.2f} {y:.2f} {ang})'
                          f' (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid "{uid()}"))')

    def text(self, s, x, y, size=1.5, bold=False, color=None):
        b = " (bold yes)" if bold else ""
        c = f" (color {color[0]} {color[1]} {color[2]} 1)" if color else ""
        self.items.append(f'(text "{esc(s)}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0)'
                          f' (effects (font (size {size} {size}){b}{c}) (justify left top)) (uuid "{uid()}"))')

    def frame(self, x1, y1, x2, y2, title, color):
        self.items.append(f'(rectangle (start {x1:.2f} {y1:.2f}) (end {x2:.2f} {y2:.2f})'
                          f' (stroke (width 0.3) (type dash) (color {color[0]} {color[1]} {color[2]} 1))'
                          f' (fill (type none)) (uuid "{uid()}"))')
        self.text(title, x1 + 1.27, y1 + 1.27, 2.0, True, color)

    def render(self):
        return (f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0")'
                f' (uuid "{ROOT_UUID}") (paper "A2")'
                f' (title_block (title "JessSDR - 40m direct-sampling SDR, полная схема")'
                f' (date "2026-11-14") (rev "4")'
                f' (comment 1 "Generated by hardware/gen_schematic.py - edit the script, not this file")'
                f' (comment 2 "Same net label = one wire. Tang pins: IOnn = FPGA pin nn"))'
                f' (lib_symbols {" ".join(self.libs)})'
                f' {" ".join(self.items)}'
                f' (sheet_instances (path "/" (page "1"))))\n')


BLUE, GREEN, RED = (0, 0, 160), (0, 110, 0), (180, 0, 0)


# ===========================================================================
# drawing
# ---------------------------------------------------------------------------
def draw_system(sh):
    # Drawn as on the perfboard (Screenshots/photo_2026-11-08_22-27-08.jpg):
    # controls | ESP32-S3 | Tang 20K | AD9226, TFT above. The Tang sits one
    # grid step lower than the ESP so ESP J3-(23-n) and Tang J5-n are level:
    # the link is drawn as real straight wires, like the jumpers on the board.
    EX, EY = 115.57, 133.35                  # ESP centre
    TX, TY = 195.58, EY + P                  # Tang centre
    sh.text("UI: ESP32-S3 DevKitC + 4\" ST7796S, энкодер, кнопки", 20.32, 17.78, 2.0, True, BLUE)
    sh.text("Радиотракт: Tang Nano 20K + AD9226", 165.1, 96.52, 2.0, True, RED)

    # ===== ESP32-S3 =====
    pins, nets = [], {}
    for i, n in enumerate(ESP_J1):
        num = f"J1-{i + 1}"
        pins.append((num, n, "L", i))
        nets[num] = ESP_NETS.get(n)
    for i, n in enumerate(ESP_J3):
        num = f"J3-{i + 1}"
        pins.append((num, n, "R", i))
        nets[num] = ESP_NETS.get(n.split()[0])
    link_j3 = {f"J3-{23 - n}" for (_, n) in ESP_LINK} | {"J3-21"}
    esp = sh.box("ESP32S3_DevKitC", "U3", "ESP32-S3-DevKitC (N16R8, 44 pin)", (EX, EY),
                 pins, 11.43, 29.21, nets, direct=link_j3)

    # ===== Tang Nano 20K =====
    pins, nets = [], {}
    for i, (lbl, use) in enumerate(J5):
        num = f"J5-{i + 1}"
        pins.append((num, f"{lbl} {use}".strip(), "L", 19 - i))
        nets[num] = TANG_NETS.get(("J5", i + 1))
    for i, (lbl, use) in enumerate(J6):
        num = f"J6-{i + 1}"
        pins.append((num, f"{use} {lbl}".strip(), "R", 19 - i))
        nets[num] = TANG_NETS.get(("J6", i + 1))
    link_j5 = {f"J5-{n}" for (_, n) in ESP_LINK} | {"J5-2"}
    tang = sh.box("TangNano20K", "U1", "Sipeed Tang Nano 20K (GW2AR-18)", (TX, TY),
                  pins, 27.94, 26.67, nets, direct=link_j5)

    # straight link wires J3-(23-n) -> J5-n, net name on each
    for n, net in sorted([(n, v) for (_, n), v in ESP_LINK.items()] + [(2, "GND")]):
        (x1, y1), (x2, y2) = esp[f"J3-{23 - n}"], tang[f"J5-{n}"]
        assert abs(y1 - y2) < 0.01, (n, y1, y2)
        sh.wire((x1, y1), (x2, y2))
        sh.label(net, x1 + 2 * P, y1)

    # ===== display, above the boards =====
    pins, nets = [], {}
    for i, n in enumerate(TFT):
        num = str(i + 1)
        pins.append((num, n, "L", i))
        nets[num] = TFT_NETS.get(n)
    sh.box("ST7796S_TFT", "U4", "4in 480x320 SPI TFT, ST7796S", (196.85, 52.07), pins, 11.43, 19.05, nets)

    # optical encoder LPD3806-600BM-G5-24C: 5-24V, A/B = NPN open collector ->
    # pull-ups to 3V3 (never to 5V), small caps against cable pickup
    sh.box("LPD3806", "U5", "LPD3806-600BM-G5-24C", (35.56, 38.1),
           [("1", "VCC (красный)", "R", 0), ("2", "GND (чёрный)", "R", 1),
            ("3", "A (зелёный)", "R", 2), ("4", "B (белый)", "R", 3), ("5", "экран", "R", 4)],
           11.43, 7.62, {"1": "+5V_ESP", "2": "GND", "3": "ENC_A", "4": "ENC_B", "5": "GND"})
    for k, (r, c, net) in enumerate((("R1", "C3", "ENC_A"), ("R2", "C4", "ENC_B"))):
        x = 71.12 + k * 12.7
        pr = sh.part("Device:R", r, "4.7k", (x, 43.18), nets={"1": "+3V3_ESP"},
                     ref_at=(x + 2.54, 41.91), val_at=(x + 2.54, 44.45))
        pc = sh.part("Device:C", c, "1nF", (x, 58.42), nets={"2": "GND"},
                     ref_at=(x + 2.54, 57.15), val_at=(x + 2.54, 59.69))
        yn = 50.8
        sh.wire(pr["2"], (x, yn), pc["1"])
        sh.wire((x, yn), (x + 5.08, yn))
        sh.junction(x, yn)
        sh.label(net, x + 5.08, yn)
    sh.text("энкодер: 600 имп/об, питание 5 В,\nподтяжки только на 3V3", 20.32, 53.34, 1.3)

    # 4 buttons, all to GND (internal pull-ups on the ESP)
    for k, (ref, net) in enumerate((("SW1", "BTN_SEL"), ("SW2", "BTN_BACK"), ("SW3", "BTN_PREV"),
                                    ("SW4", "BTN_NEXT"))):
        y = 66.04 + k * 11.43
        sh.part("Switch:SW_Push", ref, net[4:], (43.18, y), nets={"1": net, "2": "GND"},
                ref_at=(40.64, y - 3.81), val_at=(40.64, y + 2.54))

    # ESP supply: Tang +5V -> D1 (no back-feed between the two USB ports) -> L1/C1/C2 -> ESP 5V
    x0, y0 = 30.48, 200.66
    sh.label("+5V", x0 - 2.54, y0, 180, "right")
    sh.wire((x0 - 2.54, y0), (x0, y0))
    d = sh.part("Device:D_Schottky", "D1", "SS14", (x0 + 3.81, y0), rot=180,
                ref_at=(x0 + 1.27, y0 - 5.08), val_at=(x0 + 1.27, y0 + 3.81))
    l1 = sh.part("Device:L", "L1", "10uH 1A", (x0 + 16.51, y0), rot=90,
                 ref_at=(x0 + 13.97, y0 - 5.08), val_at=(x0 + 13.97, y0 + 3.81))
    sh.wire(d["1"], l1["1"])
    xn = x0 + 27.94
    sh.wire(l1["2"], (xn, y0), (xn + 12.7, y0))      # segments end at the nodes
    c1 = sh.part("Device:C_Polarized", "C1", "100uF", (xn, y0 + 7.62), nets={"2": "GND"},
                 ref_at=(xn + 3.81, y0 + 6.35), val_at=(xn + 3.81, y0 + 8.89))
    c2 = sh.part("Device:C", "C2", "100nF", (xn + 12.7, y0 + 7.62), nets={"2": "GND"},
                 ref_at=(xn + 16.51, y0 + 6.35), val_at=(xn + 16.51, y0 + 8.89))
    sh.wire((xn, y0), c1["1"])
    sh.wire((xn + 12.7, y0), c2["1"])
    sh.junction(xn, y0)
    sh.wire((xn + 12.7, y0), (xn + 20.32, y0))
    sh.label("+5V_ESP", xn + 20.32, y0)
    sh.junction(xn + 12.7, y0)
    sh.text("питание ESP", x0 - 2.54, y0 - 10.16, 1.5, True)

    # ===== RF: antenna -> BPF -> AD9226 SMA (top right) =====
    sh.frame(256.54, 15.24, 406.4, 76.2, "RF: антенна -> ДПФ 40 м -> AD9226", GREEN)
    ant = sh.part("Connector:Conn_Coaxial", "J2", "ANT (SMA/BNC)", (275.59, 40.64),
                  nets={"2": "GND"}, ref_at=(271.78, 33.02), val_at=(266.7, 50.8))
    sh.box("BPF_40m", "FL1", "40m BPF 7.0-7.2 MHz", (322.58, 40.64),
           [("IN", "IN", "L", 0), ("OUT", "OUT", "R", 0), ("G", "GND", "R", 1)],
           8.89, 5.08, {"IN": "RF_ANT", "OUT": "RF_BPF", "G": "GND"})
    sh.wire(ant["1"], (ant["1"][0] - 5.08, ant["1"][1]))
    sh.label("RF_ANT", ant["1"][0] - 5.08, ant["1"][1], 180, "right")

    # AD9226 module: P2 2x10 + SMA input, right of the Tang (P2 faces J6)
    pins, nets = [], {}
    for r in range(10):
        for col, names in ((0, ADC_OUTER), (1, ADC_INNER)):
            num = str(2 * r + 1 + col)
            pins.append((num, names[r], "L" if col == 0 else "R", r))
            nets[num] = adc_net(names[r])
    pins.append(("SMA", "SMA IN", "R", 11))
    nets["SMA"] = "RF_BPF"
    sh.box("AD9226_Module", "U2", "AD9226 module (вход: см. справа)", (325.12, TY),
           pins, 8.89, 16.51, nets)


def draw_frontend(sh):
    """AD9226 module input stage (U6 = AD8138) as built 2026-11-07 with the
    IW5ALZ gain mod (hackaday.io/project/186329, logs 213303/213304).
    Topology reconstructed from his formulas; board R-numbers unverified."""
    A, AY = 508.0, 66.04           # AD8138 centre
    yT, yB = AY - 5.08, AY + 5.08  # +IN / -IN rows
    NS, NG, NB, NO = A - 55.88, A - 25.4, A - 45.72, A + 20.32
    sh.frame(424.18, 15.24, 568.96, 116.84,
             "Вход AD9226 (на плате U2): AD8138, мод IW5ALZ +25.4 дБ - как собрано", RED)
    sh.text("SMA модуля = RF_BPF. Номинал (было ...) = сток до переделки.", 426.72, 21.59, 1.5)

    sh.wire((A - 78.74, yT), (A - 72.39, yT))
    sh.label("RF_BPF", A - 78.74, yT, 180, "right")
    cl, cr = sh.hres("Device:C", "CIN1", "100n (вместо Rser 200)", A - 68.58, yT)
    sl, sr = sh.hres("Device:R", "RG1", "360 (было 200)", A - 38.1, yT)
    sh.wire(cr, (NS, yT), sl)
    sh.junction(NS, yT)
    sh.wire((NS, yT), sh.vres("Device:R", "RSH1", "62", NS, yT + 7.62, "GND", -5.08))
    ad = sh.box("AD8138", "U6", "AD8138", (A, AY),
                [("8", "+IN", "L", 0), ("3", "V+", "L", 1), ("2", "VOCM", "L", 3), ("1", "-IN", "L", 4),
                 ("5", "-OUT", "R", 0), ("6", "V-", "R", 1), ("7", "NC", "R", 3), ("4", "+OUT", "R", 4)],
                7.62, 7.62, {"3": "+5V", "2": "CML", "6": "GND", "7": None},
                direct={"8", "1", "5", "4"})
    sh.wire(sr, (NG, yT), ad["8"])
    sh.junction(NG, yT)
    gl, gr = sh.hres("Device:R", "RG2", "360 (было 200)", A - 38.1, yB)
    sh.wire(gr, (NG, yB), ad["1"])
    sh.junction(NG, yB)
    sh.wire(gl, (NB, yB), sh.vres("Device:R", "RBAL1", "22 (было 50)", NB, yB + 7.62, "GND", 10.16))
    # feedback: RF1 -OUT -> +IN (above), RF2 +OUT -> -IN (below)
    for k, (yy, ry, out, lbl) in enumerate(((yT, yT - 10.16, "5", "VINA"), (yB, yB + 10.16, "4", "VINB"))):
        fl, fr = sh.hres("Device:R", f"RF{k + 1}", "2.2k (было 200)", A, ry)
        sh.wire((NG, yy), (NG, ry), fl)
        sh.wire(fr, (NO, ry), (NO, yy))
        sh.wire(ad[out], (NO, yy), (NO + 7.62, yy))
        sh.junction(NO, yy)
        sh.label(lbl, NO + 7.62, yy)
    sh.box("AD9226_ADC", "U7", "AD9226 (чип)", (A + 53.34, AY),
           [("VINA", "VINA", "L", 0), ("VINB", "VINB", "L", 1), ("CML", "CML", "L", 2)],
           5.08, 5.08, {"VINA": "VINA", "VINB": "VINB", "CML": "CML"})
    sh.text("между U6 и U7 - RC-фильтр\n(на плате, не трогали)", A + 33.02, AY + 15.24, 1.27)

    sh.text(
        "Усиление: было (200||62)/(200+200||62) x (200/200+1) = 0.38 (-8.4 дБ), полная шкала ~ +24 dBm, Zвх ~ 247 Ом\n"
        "          стало 2200/360+1 = 7.11 (+17.0 дБ): итого +25.4 дБ, полная шкала ~ -1.5 dBm, Zвх 62||360 = 53 Ом\n"
        "CIN 100n: развязка по постоянке (антенна с КЗ по DC не сбивает смещение). RG и RF меняются ПАРАМИ.\n"
        "RBAL балансирует -IN: было 62||(200+50) = 49.7 -> 50, стало 62||50 = 27.7 -> 22 (у IW5ALZ RG 390).\n"
        "AD8138 SOIC-8 (ключ = 1): 1 -IN, 2 VOCM, 3 V+, 4 +OUT, 5 -OUT, 6 V-, 7 NC, 8 +IN.\n"
        "Как найти на плате (прозвонка): RF - ножки 1<->4 / 8<->5; RG - от 1/8 к RSH/RBAL; RBAL - RG2 -> земля.\n"
        "Позиционные номера резисторов модуля не проверены.",
        426.72, 93.98, 1.4)


NOTES = (
    "ПРИМЕЧАНИЯ\n"
    "1. Одинаковая метка = один провод. Фаза 3 (PLAN.md): дисплей, энкодер, меню, FFT/водопад на ESP; Tang = АЦП -> DDC -> SSB/AM -> AGC -> звук.\n"
    "2. Tang J5 <-> ESP J3 (правый ряд ESP напротив левого ряда Tang): прямые перемычки, J5-n напротив J3-(23-n).\n"
    "   SPI (ESP master, mode 0): SCK J5-19/53 - GPIO1, MOSI J5-14/79 - GPIO39, MISO J5-18/71 - GPIO2, CS_N J5-4/80 - GPIO20;\n"
    "   I2S (Tang master, 105.54 кГц): WS J5-17/72 - GPIO42, BCK J5-6/41 - GPIO47, DATA J5-5/42 - GPIO21; GND J5-2 - J3-21.\n"
    "   ВНИМАНИЕ: J5-1 (+5V Tang) стоит напротив J3-22 (GND ESP) - эти два НЕ соединять. GPIO20 = USB D+ ESP: родной USB\n"
    "   ESP не используется, прошивка и консоль через порт COM (CH343). 79 = WS2812 на Tang (fpga.c гасит его после каждой посылки).\n"
    "3. Дисплей 4\" ST7796S на левом ряду ESP (J1), выводы подряд в порядке разъёма TFT: CS/RST/DC/MOSI/SCK/LED =\n"
    "   GPIO 4/5/6/7/15/16 (LED = ШИМ подсветки). VCC от 3V3 ESP (J1-1), GND J1-22. SDO и тач (T_*) не подключены.\n"
    "   Если на модуле есть перемычка J1 (обход LDO) - для 3.3 В её замкнуть.\n"
    "4. Оптический энкодер LPD3806-600BM: красный +5V_ESP, чёрный и экран GND, A (зелёный) GPIO17, B (белый) GPIO18.\n"
    "   Выходы - открытый коллектор: R1/R2 4.7k ТОЛЬКО на 3V3 (не на 5V!), C3/C4 1nF по желанию. Считает PCNT, 2400 отсч./об.\n"
    "   Кнопки на GND (внутренние подтяжки): SELECT GPIO8 (коротко - шаг/вход, долго - меню), BACK/PREV/NEXT GPIO 12/13/14.\n"
    "   Кнопки Tang S1 (88) / S2 (87) дублируют энкодер: S1 коротко/долго = NEXT/SELECT, S2 коротко = PREV, долго = BACK.\n"
    "5. Питание: Tang от своего USB-C; +5V (J5-1) -> AD9226 и через D1 + L1/C1/C2 -> 5V ESP. D1 не даёт двум USB питать друг друга,\n"
    "   с ним можно держать оба кабеля. Без D1 при двух USB-кабелях 5V между платами НЕ соединять. Wi-Fi на ESP выключен.\n"
    "6. Звук: встроенный MAX98357A на Tang (FPGA 56/55/54/51), динамик на разъём платы Tang. microSD - слот ПУСТОЙ\n"
    "   (ADC_D8 и ESP_CS_N сидят на линиях SD D1/D2).\n"
    "7. AD9226: P2 смотрит на J6, ряд +5V сверху; 14 сигналов на J6-14..J6-1 в порядке разъёма (D0..D6, CLK, OTR, D7..D11).\n"
    "   CLK/OTR на ножках LED1/LED0 (LED0 горит при OTR=0 - норма). Порядок P2 с фото продавца, подтверждён работой (BIT_REVERSE=0).\n"
    "   Вход модуля переделан (мод AD8138, справа вверху).\n"
    "8. Tang, свободно: J6-15..19 (LED2..5, 3V3), J5-16 (3V3). Не занимать: 76/86 (BL616), 49 (буст подсветки), 54/55/56/51 (усилитель).\n"
    "   Старая сборка без ESP (тег working-2026-11-07, ветка tang-only, sdr/): ILI9341 2.8\" на Tang -\n"
    "   RST/DC/MOSI/SCK = J6-18/17/16/15 (FPGA 18/19/20/17), CS = J5-18 (71), VCC J6-19, GND J6-20, LED -> 3V3.\n"
    "9. ESP N16R8: 33..37 заняты octal PSRAM; не занимать 0/3/45/46 (strapping), 43/44 (COM-порт), 48 (RGB LED), 19 (USB D-).\n"
    "   Свободны: 9, 10, 11, 38, 40, 41."
)


def main():
    sh = Sheet()
    draw_system(sh)
    draw_frontend(sh)
    sh.text(NOTES, 15.24, 243.84, 1.6)

    os.makedirs(OUT_DIR, exist_ok=True)
    sch = os.path.join(OUT_DIR, PROJECT + ".kicad_sch")
    with open(sch, "w") as f:
        f.write(sh.render())
    with open(os.path.join(OUT_DIR, PROJECT + ".kicad_sym"), "w") as f:
        f.write('(kicad_symbol_lib (version 20241209) (generator "gen_schematic") (generator_version "9.0") '
                + " ".join(sh.libfile) + ")\n")
    with open(os.path.join(OUT_DIR, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table (version 7)\n  (lib (name "%s") (type "KiCad") (uri "${KIPRJMOD}/%s.kicad_sym")'
                ' (options "") (descr "project symbols"))\n)\n' % (PROJECT, PROJECT))
    pro = os.path.join(OUT_DIR, PROJECT + ".kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w") as f:
            f.write('{\n  "meta": {"filename": "%s.kicad_pro", "version": 3},\n'
                    '  "sheets": [["%s", "Root"]]\n}\n' % (PROJECT, ROOT_UUID))
    subprocess.run(["kicad-cli", "sch", "export", "pdf", "-o", os.path.join(OUT_DIR, PROJECT + ".pdf"), sch],
                   check=True, capture_output=True)
    print("written", OUT_DIR)


if __name__ == "__main__":
    main()
