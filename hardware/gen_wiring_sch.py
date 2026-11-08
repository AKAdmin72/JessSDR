#!/usr/bin/env python3
"""Generates hardware/wiring/wiring.kicad_sch (KiCad 9) - the wiring diagram
Tang Nano 20K <-> AD9226 module <-> ILI9341 TFT.

Single source of truth for the pinout is the PINS table below; it must match
adc_test/adc_test.cst. Re-run after any change:
    python3 hardware/gen_wiring_sch.py
"""
import os
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "wiring")
PROJECT = "wiring"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "tang20k_adc_9226/wiring"))

_ctr = [0]


def uid():
    _ctr[0] += 1
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"tang20k_adc_9226/{_ctr[0]}"))


# ---------------------------------------------------------------------------
# Tang Nano 20K headers, top (USB-C end) to bottom, from the official
# schematic Tang_Nano_20K_3923_Schematics.pdf sheet 1 (J6 / J5, Conn_01x20).
# (pin label, onboard use) ; FPGA pin number in the label.
J6 = [  # left row (USB up, chip side up)
    ("IO73", ""), ("IO74", ""), ("IO75", "BL616 SPI_MISO"), ("IO85", "SD D1 10k pull-up"),
    ("IO77", "RGB-LCD CK"), ("IO15", "LED0"), ("IO16", "LED1"), ("IO27", "RGB-LCD B7"),
    ("IO28", "RGB-LCD B6"), ("IO25", "RGB-LCD HS"), ("IO26", "HDMI HPD divider"), ("IO29", "RGB-LCD B5"),
    ("IO30", "RGB-LCD B4"), ("IO31", "RGB-LCD B3"), ("IO17", "LED2"), ("IO20", "LED5"),
    ("IO19", "LED4"), ("IO18", "LED3"), ("3V3", ""), ("GND", ""),
]
J5 = [  # right row
    ("5V", ""), ("GND", ""), ("IO76", "BL616 SPI_MOSI"), ("IO80", "SD D2 10k pull-up"),
    ("IO42", "RGB-LCD R3"), ("IO41", "RGB-LCD R4"), ("IO56", "I2S BCLK (amp)"), ("IO54", "I2S DIN (amp)"),
    ("IO51", "PA_EN (amp)"), ("IO48", "RGB-LCD DE"), ("IO55", "I2S LRCK (amp)"), ("IO49", "LCD BL boost EN"),
    ("IO86", "BL616 SPI_DIR"), ("IO79", "WS2812"), ("GND", ""), ("3V3", ""),
    ("IO72", ""), ("IO71", ""), ("IO53", "HDMI DDC SCL"), ("IO52", "HDMI DDC SDA"),
]

# Net per Tang header pin (header, index 1..20) -> net name.
# Layout (2026-11-01, chosen for the shortest wires):
#     [ TFT, above the Tang's top-right corner ]
#     ESP32-S3  ::  Tang  ::  AD9226 (SMA pointing right)
# Tang mounted HDMI end UP, USB-C down: J5 = LEFT row (faces the ESP),
# J6 = RIGHT row (faces the ADC module's P2 header), pin 20 of both at the top.
# - ADC: all 14 signals on J6-14..J6-1, in the same top-to-bottom order as the
#   module's P2 header (+5V row at the top): D0..D6, CLK, OTR, D7..D11.
#   Data only on clean pins; CLK/OTR on the two LED pins of that stretch
#   (an output / a static level don't mind the 510R+LED load).
# - TFT: the 4 LED pins at the top of J6 + CS on J5-18 + 3V3/GND from J6-19/20.
# - ESP: all 7 link signals on J5.
TANG_NETS = {
    # AD9226, straight across J6-14..J6-1
    ("J6", 14): "ADC_D0", ("J6", 13): "ADC_D1", ("J6", 12): "ADC_D2", ("J6", 11): "ADC_D3",
    ("J6", 10): "ADC_D4", ("J6", 9): "ADC_D5", ("J6", 8): "ADC_D6",
    ("J6", 7): "ADC_CLK", ("J6", 6): "ADC_OTR",
    ("J6", 5): "ADC_D7", ("J6", 4): "ADC_D8", ("J6", 3): "ADC_D9", ("J6", 2): "ADC_D10",
    ("J6", 1): "ADC_D11",
    ("J5", 1): "+5V", ("J5", 2): "GND", ("J5", 15): "GND", ("J5", 16): "+3V3",
    # TFT, from the top of the Tang
    ("J6", 20): "GND", ("J6", 19): "+3V3",
    ("J6", 18): "TFT_RST", ("J6", 17): "TFT_DC", ("J6", 16): "TFT_MOSI", ("J6", 15): "TFT_SCK",
    ("J5", 18): "TFT_CS",
}

# ESP32-S3 link (SPI slave regs + I2S I/Q out), revised 2026-11-08 for the
# perfboard photo Screenshots/photo_2026-11-08_22-27-08.jpg: ESP32-S3-DevKitC
# (N16R8) left of the Tang, antenna end up; its right row J3 faces Tang J5 and
# is 2 pins longer at the top, so J5-n sits exactly opposite ESP J3-(23-n) ->
# every link wire is one straight horizontal jumper. Pairs not used: 52/RX
# (ESP console), 3V3/GPIO41, GND/GPIO40 (power), 49,55,48,51,54,56 opposite
# 37/36/35/0/45/48 (octal PSRAM / strapping / RGB LED), 86/GPIO38 (86 = BL616
# IO0 SPI_DIR, may be driven), 76/GPIO19 (BL616 SPI_MOSI), 5V/GND (J5-1 faces
# ESP GND - never bridge!).
#   J5-19 IO53 <-> GPIO1  SCK   (ESP -> FPGA)
#   J5-18 IO71 <-> GPIO2  MISO  (FPGA -> ESP; the old Tang TFT build drives 71
#                                as TFT CS - harmless into an ESP input)
#   J5-17 IO72 <-> GPIO42 I2S WS
#   J5-14 IO79 <-> GPIO39 MOSI  (WS2812 DIN: the RGB LED may light randomly)
#   J5-6  IO41 <-> GPIO47 I2S BCK
#   J5-5  IO42 <-> GPIO21 I2S DATA
#   J5-4  IO80 <-> GPIO20 CS_N  (ESP native-USB D+ given up; 10k pull-up on
#                                the Tang keeps CS high while the ESP boots)
#   J5-2  GND  <-> J3-21 GND
ESP_LINK = {
    ("J5", 19): "ESP_SCK", ("J5", 18): "ESP_MISO", ("J5", 17): "ESP_I2S_WS",
    ("J5", 14): "ESP_MOSI", ("J5", 6): "ESP_I2S_BCK", ("J5", 5): "ESP_I2S_DATA",
    ("J5", 4): "ESP_CS_N",
}
ESP_GPIO = {"ESP_SCK": "GPIO1", "ESP_MISO": "GPIO2", "ESP_I2S_WS": "GPIO42", "ESP_MOSI": "GPIO39",
            "ESP_I2S_BCK": "GPIO47", "ESP_I2S_DATA": "GPIO21", "ESP_CS_N": "GPIO20"}
# phase 1/2 drawing: the Tang still drives the ILI9341 (CS on J5-18), the
# link is reserved on the remaining pins
TANG_NETS.update({k: v for k, v in ESP_LINK.items() if k not in TANG_NETS})

# AD9226 module 2x10 header P2, read from the seller photo silkscreen
# (UNVERIFIED - check on the real board). Row 1 = end next to "+5V" label.
# outer column / inner column
ADC_OUTER = ["+5V", "GND", "D0", "D2", "D4", "D6", "D8", "D10", "OTR", "GND"]
ADC_INNER = ["+5V", "GND", "CLK", "D1", "D3", "D5", "D7", "D9", "D11", "GND"]

TFT = ["VCC", "GND", "CS", "RESET", "DC", "SDI(MOSI)", "SCK", "LED",
       "SDO(MISO)", "T_CLK", "T_CS", "T_DIN", "T_DO", "T_IRQ"]
# ESP32-S3 board: only the GPIOs used by the link (esp32/main/fpga.c) + GND.
# (ESP_MISO is left out here: its Tang pin J5-18 is still the TFT CS in this drawing)
ESP = [(ESP_GPIO[n], n) for n in ("ESP_SCK", "ESP_I2S_WS", "ESP_MOSI", "ESP_I2S_BCK",
                                   "ESP_I2S_DATA", "ESP_CS_N") if n in TANG_NETS.values()] + [("GND", "GND")]

TFT_NETS = {"VCC": "+3V3", "GND": "GND", "CS": "TFT_CS", "RESET": "TFT_RST",
            "DC": "TFT_DC", "SDI(MOSI)": "TFT_MOSI", "SCK": "TFT_SCK", "LED": "+3V3"}


def adc_net(name):
    if name in ("+5V", "GND"):
        return name
    if name == "CLK":
        return "ADC_CLK"
    if name == "OTR":
        return "ADC_OTR"
    return "ADC_" + name


# ---------------------------------------------------------------------------
P = 2.54
PL = 5.08  # pin length (room for "J6-10"-style numbers)
FONT = "(effects (font (size 1.27 1.27)))"


def esc(s):
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def pin_sexpr(num, name, x, y, ang, length=PL):
    return (f'(pin passive line (at {x:.2f} {y:.2f} {ang}) (length {length})'
            f' (name "{esc(name)}" {FONT}) (number "{esc(num)}" {FONT}))')


def lib_symbol(name, ref, value, pins, half_w, half_h, prefix=PROJECT + ":"):
    """pins: list of (number, name, side 'L'/'R', row_index). Row 0 at top."""
    body = (f'(rectangle (start {-half_w:.2f} {half_h:.2f}) (end {half_w:.2f} {-half_h:.2f})'
            ' (stroke (width 0.254) (type default)) (fill (type background)))')
    ps = []
    for num, pname, side, row in pins:
        y = half_h - P * (row + 1)
        if side == "L":
            ps.append(pin_sexpr(num, pname, -half_w - PL, y, 0))
        else:
            ps.append(pin_sexpr(num, pname, half_w + PL, y, 180))
    return (f'(symbol "{prefix}{name}" (pin_names (offset 1.016)) (exclude_from_sim no)'
            f' (in_bom yes) (on_board yes)'
            f' (property "Reference" "{ref}" (at 0 {half_h + 1.27:.2f} 0) {FONT})'
            f' (property "Value" "{esc(value)}" (at 0 {-half_h - 1.27:.2f} 0) {FONT})'
            f' (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f' (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f' (symbol "{name}_0_1" {body})'
            f' (symbol "{name}_1_1" {" ".join(ps)}))')


class Sheet:
    def __init__(self):
        self.items = []
        self.libs = []
        self.libfile = []

    def place(self, libname, ref, value, at, pins, half_w, half_h, nets):
        """nets: pin number -> net name (None = no-connect)."""
        self.libs.append(lib_symbol(libname, ref, value, pins, half_w, half_h))
        self.libfile.append(lib_symbol(libname, ref, value, pins, half_w, half_h, prefix=""))
        ax, ay = at
        pin_uuids = " ".join(f'(pin "{esc(n)}" (uuid "{uid()}"))' for n, *_ in pins)
        self.items.append(
            f'(symbol (lib_id "{PROJECT}:{libname}") (at {ax:.2f} {ay:.2f} 0) (unit 1)'
            f' (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}")'
            f' (property "Reference" "{ref}" (at {ax:.2f} {ay - half_h - 1.27:.2f} 0) {FONT})'
            f' (property "Value" "{esc(value)}" (at {ax:.2f} {ay + half_h + 1.27:.2f} 0) {FONT})'
            f' (property "Footprint" "" (at {ax:.2f} {ay:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f' (property "Datasheet" "" (at {ax:.2f} {ay:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f' {pin_uuids}'
            f' (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (reference "{ref}") (unit 1)))))')
        for num, pname, side, row in pins:
            y = ay - (half_h - P * (row + 1))   # schematic y grows downward
            if side == "L":
                px, dx, just = ax - half_w - PL, -1, "right"
            else:
                px, dx, just = ax + half_w + PL, 1, "left"
            net = nets.get(num)
            if net is None:
                self.items.append(f'(no_connect (at {px:.2f} {y:.2f}) (uuid "{uid()}"))')
                continue
            ex = px + dx * 3 * P
            self.items.append(
                f'(wire (pts (xy {px:.2f} {y:.2f}) (xy {ex:.2f} {y:.2f}))'
                f' (stroke (width 0) (type default)) (uuid "{uid()}"))')
            ang = 0 if dx > 0 else 180
            self.items.append(
                f'(label "{esc(net)}" (at {ex:.2f} {y:.2f} {ang})'
                f' (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid "{uid()}"))')

    def text(self, s, x, y, size=1.27):
        self.items.append(
            f'(text "{esc(s)}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0)'
            f' (effects (font (size {size} {size})) (justify left top)) (uuid "{uid()}"))')

    def render(self):
        return (f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0")'
                f' (uuid "{ROOT_UUID}") (paper "A3")'
                f' (title_block (title "Tang Nano 20K + AD9226 + ILI9341 - wiring")'
                f' (date "2026-10-31") (rev "1") (comment 1 "Generated by hardware/gen_wiring_sch.py - edit the script, not this file"))'
                f' (lib_symbols {" ".join(self.libs)})'
                f' {" ".join(self.items)}'
                f' (sheet_instances (path "/" (page "1"))))\n')


def main():
    sh = Sheet()

    # --- U1: Tang Nano 20K, drawn as mounted: HDMI end up, chip side up ->
    # J5 on the left, J6 on the right, pin 20 of both rows at the top ---
    pins, nets = [], {}
    for i, (lbl, use) in enumerate(J5):
        num = f"J5-{i + 1}"
        pins.append((num, f"{lbl} {use}".strip(), "L", 19 - i))
        nets[num] = TANG_NETS.get(("J5", i + 1))
    for i, (lbl, use) in enumerate(J6):
        num = f"J6-{i + 1}"
        pins.append((num, f"{use} {lbl}".strip(), "R", 19 - i))
        nets[num] = TANG_NETS.get(("J6", i + 1))
    sh.place("TangNano20K", "U1", "Sipeed Tang Nano 20K (GW2AR-18)", (210.82, 132.08),
             pins, 27.94, 26.67, nets)

    # --- U2: AD9226 module, P2 2x10 (KiCad odd/even numbering) ---
    pins, nets = [], {}
    for r in range(10):
        for col, names in ((0, ADC_OUTER), (1, ADC_INNER)):
            num = str(2 * r + 1 + col)
            pins.append((num, names[r], "L" if col == 0 else "R", r))
            nets[num] = adc_net(names[r])
    sh.place("AD9226_Module", "U2", "AD9226 12bit 65MSPS module (P2 2x10, 5V)", (299.72, 138.43),
             pins, 7.62, 13.97, nets)

    # --- U3: ILI9341 2.8in SPI TFT (14-pin header) ---
    pins, nets = [], {}
    for i, n in enumerate(TFT):
        num = str(i + 1)
        pins.append((num, n, "L", i))
        nets[num] = TFT_NETS.get(n)
    # TFT above the Tang (top-right corner)
    sh.place("ILI9341_TFT", "U3", "ILI9341 2.8in 320x240 SPI TFT", (299.72, 63.50),
             pins, 10.16, 19.05, nets)

    # --- U4: ESP32-S3 board (link signals only) ---
    pins, nets = [], {}
    for i, (g, net) in enumerate(ESP):
        num = str(i + 1)
        pins.append((num, g, "R", i))
        nets[num] = net
    sh.place("ESP32S3_Link", "U4", "ESP32-S3 board (FPGA link pins only)", (99.06, 132.08),
             pins, 10.16, 11.43, nets)

    sh.text("WIRING NOTES\n"
            "1. Same net label = one wire. Tang 20K pin labels: IOnn = FPGA pin nn (see adc_test/adc_test.cst).\n"
            "   Perfboard layout: ESP32-S3 | Tang | AD9226 (SMA to the right), TFT above the Tang's top-right corner.\n"
            "   Tang drawn as mounted: HDMI end UP, USB-C down -> J5 = left row (ESP side), J6 = right row (ADC side).\n"
            "2. AD9226: module P2 header faces J6, +5V row at the top. All 14 signals sit on J6-14..J6-1 in the header's\n"
            "   own top-to-bottom order (D0..D6, CLK, OTR, D7..D11) -> short, nearly straight wires. CLK (J6-7) and OTR\n"
            "   (J6-6) are on the LED1/LED0 pins (LED0 is lit while OTR=0, normal). Data pins have no LED/pull-up load\n"
            "   except D3 (HDMI HPD 2.2k+5.1k divider) and D8 (microSD 10k pull-up) - both light.\n"
            "   P2 pin order was read from the seller photo - VERIFY on the real board. Bit order: BIT_REVERSE parameter.\n"
            "3. AD9226 power: +5V/GND from Tang J5-1/J5-2 for bring-up (wire under the Tang); a separate clean 5V is\n"
            "   better for the SDR. Its logic outputs are 3.3V - measure the module's 3V3 pad before connecting data.\n"
            "4. TFT: RESET/DC/MOSI/SCK from J6-18..J6-15 (LED3/4/5/2 pins - they flicker), CS from J5-18,\n"
            "   VCC from J6-19, GND from J6-20, LED (backlight) -> 3V3. SDO and touch pins unconnected.\n"
            "5. Pins NOT used on purpose: 76/86 (driven by onboard BL616), 49 (LCD backlight boost EN),\n"
            "   54/55/56/51 (onboard audio amp - reserved for SDR audio).\n"
            "6. ESP32-S3 link (2026-11-08), straight jumpers J5-n <-> ESP J3-(23-n): SCK 53/GPIO1, MISO 71/GPIO2\n"
            "   (phase 3, J5-18 is TFT CS until the TFT moves to the ESP), I2S WS 72/GPIO42, MOSI 79/GPIO39 (WS2812 DIN),\n"
            "   I2S BCK 41/GPIO47, I2S DATA 42/GPIO21, CS_N 80/GPIO20, GND J5-2/J3-21. See hardware/system.\n"
            "7. microSD slot must stay EMPTY (ADC_D8 and ESP_I2S_BCK use SD D1/D2 lines).",
            20.32, 200.66)

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"{PROJECT}.kicad_sch"), "w") as f:
        f.write(sh.render())
    with open(os.path.join(OUT_DIR, f"{PROJECT}.kicad_sym"), "w") as f:
        f.write('(kicad_symbol_lib (version 20241209) (generator "gen_wiring_sch") (generator_version "9.0") '
                + " ".join(sh.libfile) + ")\n")
    with open(os.path.join(OUT_DIR, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table (version 7)\n  (lib (name "%s") (type "KiCad") (uri "${KIPRJMOD}/%s.kicad_sym") (options "") (descr "project symbols"))\n)\n' % (PROJECT, PROJECT))
    pro = os.path.join(OUT_DIR, f"{PROJECT}.kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w") as f:
            f.write('{\n  "meta": {"filename": "%s.kicad_pro", "version": 3},\n'
                    '  "sheets": [["%s", "Root"]]\n}\n' % (PROJECT, ROOT_UUID))
    print("written", OUT_DIR)


if __name__ == "__main__":
    main()
