#!/usr/bin/env python3
"""Generates hardware/system/system.kicad_sch (+ .pdf) (KiCad 9): the whole
Phase 3 receiver (PLAN.md) - antenna -> 40m BPF -> AD9226 -> Tang Nano 20K
(radio path, onboard I2S amp) <-> ESP32-S3 DevKitC (UI) -> 4" ST7796S TFT,
encoder, buttons, ESP supply filter.

Tang header tables and the ADC/ESP-link nets come from gen_wiring_sch.py
(the phase 1/2 wiring, source of truth for the .cst files). ESP pins as in
esp32/main/{lcd,fpga,input}.c. Re-run after changes:
    python3 hardware/gen_system_sch.py
"""
import os
import re
import subprocess
import uuid

import gen_wiring_sch as W
from gen_frontend_mod_sch import lib_entry, lib_get

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "system")
PROJECT = "system"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "tang20k_adc_9226/system"))
P, PL, FONT, esc = W.P, W.PL, W.FONT, W.esc
HIDE = "(effects (font (size 1.27 1.27)) (hide yes))"
_ctr = [0]


def uid():
    _ctr[0] += 1
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"tang20k_adc_9226/system/{_ctr[0]}"))


# ---------------------------------------------------------------------------
# Tang Nano 20K: phase 1/2 nets minus the old ILI9341 (moved to the ESP).
TANG_NETS = {k: v for k, v in W.TANG_NETS.items() if not v.startswith("TFT_")}
TANG_NETS.pop(("J6", 19))     # 3V3 out, unused now
TANG_NETS.pop(("J5", 16))
TANG_NETS[("J6", 20)] = "GND"
TANG_NETS.update(W.ESP_LINK)  # incl. ESP_MISO on J5-18 (the old TFT CS)

# ESP32-S3-DevKitC (44 pins, N16R8 on the perfboard), rows top to bottom
# (antenna end up). Placement (Screenshots/photo_2026-11-08_22-27-08.jpg):
# ESP left of the Tang, J3 (right row) faces Tang J5 -> link on J3, straight
# jumpers (gen_wiring_sch.ESP_LINK); TFT + controls on J1 (left row).
# N16R8 = octal PSRAM: GPIO33..37 are taken (35/36/37 are on J3).
ESP_J1 = ["3V3", "3V3", "RST", "GPIO4", "GPIO5", "GPIO6", "GPIO7", "GPIO15", "GPIO16", "GPIO17",
          "GPIO18", "GPIO8", "GPIO3", "GPIO46", "GPIO9", "GPIO10", "GPIO11", "GPIO12", "GPIO13",
          "GPIO14", "5V", "GND"]
ESP_J3 = ["GND", "GPIO43 TX", "GPIO44 RX", "GPIO1", "GPIO2", "GPIO42", "GPIO41", "GPIO40",
          "GPIO39", "GPIO38", "GPIO37", "GPIO36", "GPIO35", "GPIO0 BOOT", "GPIO45", "GPIO48 RGB",
          "GPIO47", "GPIO21", "GPIO20", "GPIO19", "GND", "GND"]
ESP_NETS = {
    "3V3": "+3V3_ESP", "5V": "+5V_ESP", "GND": "GND",
    # FPGA link (fpga.c), right row J3, straight across to Tang J5
    **{g: n for n, g in W.ESP_GPIO.items()},
    # display (lcd.c), left row J1 top: same order as the TFT header pins 3..8
    "GPIO4": "LCD_CS", "GPIO5": "LCD_RST", "GPIO6": "LCD_DC",
    "GPIO7": "LCD_MOSI", "GPIO15": "LCD_SCK", "GPIO16": "LCD_LED",
    # encoder + buttons (input.c), left row J1
    "GPIO17": "ENC_A", "GPIO18": "ENC_B", "GPIO8": "ENC_SW",
    "GPIO12": "BTN_BACK", "GPIO13": "BTN_PREV", "GPIO14": "BTN_NEXT",
}
# ESP J3 GND pins: only J3-21 (opposite Tang J5-2 GND) carries the link GND;
# J3-22 sits opposite Tang J5-1 = +5V - never bridge those two.

TFT = ["VCC", "GND", "CS", "RESET", "DC/RS", "SDI(MOSI)", "SCK", "LED",
       "SDO(MISO)", "T_CLK", "T_CS", "T_DIN", "T_DO", "T_IRQ"]
TFT_NETS = {"VCC": "+3V3_ESP", "GND": "GND", "CS": "LCD_CS", "RESET": "LCD_RST",
            "DC/RS": "LCD_DC", "SDI(MOSI)": "LCD_MOSI", "SCK": "LCD_SCK", "LED": "LCD_LED"}


# ---------------------------------------------------------------------------
class Sheet:
    def __init__(self):
        self.items, self.libs, self.libfile, self.lib_ids = [], [], [], set()

    def _instance(self, lib_id, ref, value, ax, ay, rot, pin_nums, ref_at, val_at, val_hide=False):
        pin_u = " ".join(f'(pin "{esc(n)}" (uuid "{uid()}"))' for n in pin_nums)
        vh = " (hide yes)" if val_hide else ""
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {ax:.2f} {ay:.2f} {rot}) (unit 1)'
            f' (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}")'
            f' (property "Reference" "{ref}" (at {ref_at[0]:.2f} {ref_at[1]:.2f} {rot % 180}) {FONT})'
            f' (property "Value" "{esc(value)}" (at {val_at[0]:.2f} {val_at[1]:.2f} {rot % 180})'
            f' (effects (font (size 1.27 1.27)){vh}))'
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
        self.libs.append(W.lib_symbol(libname, ref, value, pins, half_w, half_h, prefix=PROJECT + ":"))
        self.libfile.append(W.lib_symbol(libname, ref, value, pins, half_w, half_h, prefix=""))
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
        fname, name = {"Device": "Device.kicad_sym", "Switch": "Switch.kicad_sym",
                       "Connector": "Connector.kicad_sym"}[lib_id.split(":")[0]], lib_id.split(":")[1]
        if lib_id not in self.lib_ids:
            self.lib_ids.add(lib_id)
            self.libs.append(lib_entry(fname, lib_id.split(":")[0], name))
        t = lib_get(fname, name)
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
                f' (uuid "{ROOT_UUID}") (paper "A3")'
                f' (title_block (title "40m SDR - Phase 3 system")'
                f' (date "2026-11-08") (rev "2")'
                f' (comment 1 "Generated by hardware/gen_system_sch.py - edit the script, not this file")'
                f' (comment 2 "Same net label = one wire. Tang pins: IOnn = FPGA pin nn"))'
                f' (lib_symbols {" ".join(self.libs)})'
                f' {" ".join(self.items)}'
                f' (sheet_instances (path "/" (page "1"))))\n')


BLUE, GREEN, RED, GREY = (0, 0, 160), (0, 110, 0), (180, 0, 0), (90, 90, 90)


def main():
    sh = Sheet()

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
    link_j3 = {f"J3-{23 - n}" for (_, n) in W.ESP_LINK} | {"J3-21"}
    esp = sh.box("ESP32S3_DevKitC", "U3", "ESP32-S3-DevKitC (N16R8, 44 pin)", (EX, EY),
                 pins, 11.43, 29.21, nets, direct=link_j3)

    # ===== Tang Nano 20K =====
    pins, nets = [], {}
    for i, (lbl, use) in enumerate(W.J5):
        num = f"J5-{i + 1}"
        pins.append((num, f"{lbl} {use}".strip(), "L", 19 - i))
        nets[num] = TANG_NETS.get(("J5", i + 1))
    for i, (lbl, use) in enumerate(W.J6):
        num = f"J6-{i + 1}"
        pins.append((num, f"{use} {lbl}".strip(), "R", 19 - i))
        nets[num] = TANG_NETS.get(("J6", i + 1))
    link_j5 = {f"J5-{n}" for (_, n) in W.ESP_LINK} | {"J5-2"}
    tang = sh.box("TangNano20K", "U1", "Sipeed Tang Nano 20K (GW2AR-18)", (TX, TY),
                  pins, 27.94, 26.67, nets, direct=link_j5)

    # straight link wires J3-(23-n) -> J5-n, net name on each
    for n, net in sorted([(n, v) for (_, n), v in W.ESP_LINK.items()] + [(2, "GND")]):
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

    # encoder + 3 buttons, all to GND (internal pull-ups on the ESP)
    sh.part("Device:RotaryEncoder_Switch", "SW1", "EC11 encoder", (43.18, 40.64),
            nets={"A": "ENC_A", "B": "ENC_B", "C": "GND", "S1": "ENC_SW", "S2": "GND"},
            ref_at=(38.1, 33.02), val_at=(38.1, 48.26))
    for k, (ref, net) in enumerate((("SW2", "BTN_BACK"), ("SW3", "BTN_PREV"), ("SW4", "BTN_NEXT"))):
        y = 63.5 + k * 12.7
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
        for col, names in ((0, W.ADC_OUTER), (1, W.ADC_INNER)):
            num = str(2 * r + 1 + col)
            pins.append((num, names[r], "L" if col == 0 else "R", r))
            nets[num] = W.adc_net(names[r])
    pins.append(("SMA", "SMA IN", "R", 11))
    nets["SMA"] = "RF_BPF"
    sh.box("AD9226_Module", "U2", "AD9226 module, AD8138 gain mod +25 dB", (325.12, TY),
           pins, 8.89, 16.51, nets)

    sh.text("ПРИМЕЧАНИЯ\n"
            "1. Одинаковая метка = один провод. Фаза 3 (PLAN.md): дисплей, энкодер, меню, FFT/водопад на ESP; Tang = АЦП -> DDC -> SSB -> AGC -> звук.\n"
            "2. Tang J5 <-> ESP J3 (правый ряд ESP напротив левого ряда Tang): прямые перемычки, J5-n напротив J3-(23-n).\n"
            "   SPI (ESP master, mode 0): SCK J5-19/53 - GPIO1, MOSI J5-14/79 - GPIO39, MISO J5-18/71 - GPIO2, CS_N J5-4/80 - GPIO20;\n"
            "   I2S (Tang master, 105.54 кГц): WS J5-17/72 - GPIO42, BCK J5-6/41 - GPIO47, DATA J5-5/42 - GPIO21; GND J5-2 - J3-21.\n"
            "   ВНИМАНИЕ: J5-1 (+5V Tang) стоит напротив J3-22 (GND ESP) - эти два НЕ соединять. GPIO20 = USB D+ ESP: родной USB\n"
            "   ESP не используется, прошивка и консоль через порт COM (CH343). 79 = WS2812 на Tang (RGB LED может мигать).\n"
            "3. Дисплей 4\" ST7796S на левом ряду ESP (J1), выводы подряд в порядке разъёма TFT: CS/RST/DC/MOSI/SCK/LED =\n"
            "   GPIO 4/5/6/7/15/16 (LED = ШИМ подсветки). VCC от 3V3 ESP (J1-1), GND J1-22. SDO и тач (T_*) не подключены.\n"
            "   Если на модуле есть перемычка J1 (обход LDO) - для 3.3 В её замкнуть.\n"
            "4. Энкодер A/B/SW = GPIO 17/18/8, кнопки BACK/PREV/NEXT = GPIO 12/13/14, всё на GND (внутренние подтяжки). Без энкодера - BOOT.\n"
            "5. Питание: Tang от своего USB-C; +5V (J5-1) -> AD9226 и через D1 + L1/C1/C2 -> 5V ESP. D1 не даёт двум USB питать друг друга,\n"
            "   с ним можно держать оба кабеля. Без D1 при двух USB-кабелях 5V между платами НЕ соединять. Wi-Fi на ESP выключен.\n"
            "6. Звук: встроенный MAX98357A на Tang (FPGA 56/55/54/51), динамик на разъём платы Tang. microSD - слот ПУСТОЙ.\n"
            "7. На Tang свободны после переноса дисплея: J6-15..18 (бывший ILI9341), 3V3 (J5-16, J6-19).\n"
            "8. ESP N16R8: 33..37 заняты octal PSRAM; не занимать 0/3/45/46 (strapping), 43/44 (COM-порт), 48 (RGB LED), 19 (USB D-).\n"
            "   Свободны: 9, 10, 11, 38, 40, 41.\n"
            "9. RF: антенна -> ДПФ 40 м -> SMA AD9226 (мод AD8138: полная шкала ~ -1.5 dBm, Zвх ~ 53 Ом, см. hardware/frontend_mod).\n"
            "   Порядок P2 AD9226 взят с фото продавца, на плате подтверждён работой (BIT_REVERSE=0).",
            15.24, 243.84, 1.6)

    os.makedirs(OUT_DIR, exist_ok=True)
    sch = os.path.join(OUT_DIR, PROJECT + ".kicad_sch")
    with open(sch, "w") as f:
        f.write(sh.render())
    with open(os.path.join(OUT_DIR, PROJECT + ".kicad_sym"), "w") as f:
        f.write('(kicad_symbol_lib (version 20241209) (generator "gen_system_sch") (generator_version "9.0") '
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
