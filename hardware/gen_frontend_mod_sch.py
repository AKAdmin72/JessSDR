#!/usr/bin/env python3
"""Generates hardware/frontend_mod/frontend_mod.kicad_sch (KiCad 9):
AD9226 module input stage (AD8138) - stock vs IW5ALZ gain mod (+24.9 dB; built with RG 360 -> +25.4 dB).

Topology reconstructed from IW5ALZ's logs (hackaday.io/project/186329,
logs 213303/213304): stock gain = (200||62)/(200+200||62) * (200/200+1) = 0.38,
mod = (2200/390+1) = 6.64 (as built: 2200/360+1 = 7.11). Board reference designators NOT verified.
    python3 hardware/gen_frontend_mod_sch.py
"""
import os
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "frontend_mod")
PROJECT = "frontend_mod"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "tang20k_adc_9226/frontend_mod"))
KLIB = "/usr/share/kicad/symbols"
G = 2.54
_ctr = [0]


def uid():
    _ctr[0] += 1
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"tang20k_adc_9226/fe/{_ctr[0]}"))


def esc(s):
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def lib_get(fname, name):
    s = open(os.path.join(KLIB, fname)).read()
    i = s.index(f'(symbol "{name}"')
    d = 0
    for j in range(i, len(s)):
        d += s[j] == "("
        d -= s[j] == ")"
        if d == 0:
            return s[i:j + 1]


def lib_entry(fname, lib, name):
    t = lib_get(fname, name)
    return t.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)


FONT = "(effects (font (size 1.27 1.27)))"
HIDE = "(effects (font (size 1.27 1.27)) (hide yes))"

# AD8138 (SOIC-8) custom symbol: body x +-7.62, y +-5.08; pin ends at x +-10.16 / y +-7.62
AD8138_PINS = [  # num, name, x, y, angle (symbol coords, y up)
    ("8", "+IN", -10.16, 2.54, 0), ("1", "-IN", -10.16, -2.54, 0),
    ("5", "-OUT", 10.16, 2.54, 180), ("4", "+OUT", 10.16, -2.54, 180),
    ("3", "V+", 0, 7.62, 270), ("2", "VOCM", 0, -7.62, 90), ("6", "V-", 2.54, -7.62, 90),
]


def ad8138_lib():
    ps = " ".join(
        f'(pin passive line (at {x} {y} {a}) (length 2.54) (name "{n}" {FONT}) (number "{num}" {FONT}))'
        for num, n, x, y, a in AD8138_PINS)
    return ('(symbol "fe:AD8138" (pin_names (offset 0.762)) (exclude_from_sim no) (in_bom yes) (on_board yes)'
            f' (property "Reference" "U" (at 0 6.35 0) {FONT}) (property "Value" "AD8138" (at 0 -6.35 0) {FONT})'
            f' (property "Footprint" "" (at 0 0 0) {HIDE}) (property "Datasheet" "" (at 0 0 0) {HIDE})'
            ' (symbol "AD8138_0_1" (rectangle (start -7.62 5.08) (end 7.62 -5.08)'
            ' (stroke (width 0.254) (type default)) (fill (type background))))'
            f' (symbol "AD8138_1_1" {ps}))')


class Sheet:
    def __init__(self):
        self.items = []

    def sym(self, lib_id, ref, value, x, y, rot=0, mirror=False, pins=("1", "2"),
            ref_at=None, val_at=None, show_ref=True, show_val=True, just="left"):
        rx, ry = ref_at or (x + 2.54, y - 1.27)
        vx, vy = val_at or (x + 2.54, y + 1.27)
        rhide = "" if show_ref else " (hide yes)"
        vhide = "" if show_val else " (hide yes)"
        pin_u = " ".join(f'(pin "{p}" (uuid "{uid()}"))' for p in pins)
        mir = " (mirror y)" if mirror else ""
        fa = rot  # field angle = symbol rotation -> text stays horizontal
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {x:.2f} {y:.2f} {rot}){mir} (unit 1)'
            f' (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}")'
            f' (property "Reference" "{ref}" (at {rx:.2f} {ry:.2f} {fa}) (effects (font (size 1.27 1.27)) (justify {just}){rhide}))'
            f' (property "Value" "{esc(value)}" (at {vx:.2f} {vy:.2f} {fa}) (effects (font (size 1.27 1.27)) (justify {just}){vhide}))'
            f' (property "Footprint" "" (at {x:.2f} {y:.2f} 0) {HIDE})'
            f' (property "Datasheet" "" (at {x:.2f} {y:.2f} 0) {HIDE})'
            f' {pin_u}'
            f' (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (reference "{ref}") (unit 1)))))')

    def hres(self, lib_id, ref, value, cx, y, below=False):
        """Horizontal R/C (rot 90), pins at cx -+ 3.81. Text above (or below)."""
        if below:
            ra, va = (cx - 3.5, y + 2.8), (cx - 3.5, y + 4.6)
        else:
            ra, va = (cx - 3.5, y - 4.6), (cx - 3.5, y - 2.8)
        # rot 90 flips justification: 'right' here renders left-aligned
        self.sym(lib_id, ref, value, cx, y, rot=90, ref_at=ra, val_at=va, just="right")

    def wire(self, *pts):
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            self.items.append(
                f'(wire (pts (xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f}))'
                f' (stroke (width 0) (type default)) (uuid "{uid()}"))')

    def junction(self, x, y):
        self.items.append(f'(junction (at {x:.2f} {y:.2f}) (diameter 0) (color 0 0 0 0) (uuid "{uid()}"))')

    def label(self, name, x, y, ang=0, just="left"):
        self.items.append(
            f'(label "{esc(name)}" (at {x:.2f} {y:.2f} {ang})'
            f' (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid "{uid()}"))')

    def text(self, s, x, y, size=1.27, color=None, bold=False):
        col = f" (color {color[0]} {color[1]} {color[2]} 1)" if color else ""
        b = " (bold yes)" if bold else ""
        self.items.append(
            f'(text "{esc(s)}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0)'
            f' (effects (font (size {size} {size}){b}{col}) (justify left top)) (uuid "{uid()}"))')

    def box(self, x1, y1, x2, y2, color):
        self.items.append(
            f'(rectangle (start {x1:.2f} {y1:.2f}) (end {x2:.2f} {y2:.2f})'
            f' (stroke (width 0.4) (type dash) (color {color[0]} {color[1]} {color[2]} 1))'
            f' (fill (type none)) (uuid "{uid()}"))')

    def gnd(self, x, y, n):
        self.sym("power:GND", f"#PWR{n:03d}", "GND", x, y, pins=("1",), show_ref=False, show_val=False)

    def render(self, libs):
        return (f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0")'
                f' (uuid "{ROOT_UUID}") (paper "A3")'
                f' (title_block (title "AD9226 module: AD8138 input stage - stock vs IW5ALZ mod")'
                f' (date "2026-11-07") (rev "1")'
                f' (comment 1 "Generated by hardware/gen_frontend_mod_sch.py - edit the script, not this file")'
                f' (comment 2 "Topology from IW5ALZ hackaday.io/project/186329 logs 213303/213304; board R-numbers unverified"))'
                f' (lib_symbols {" ".join(libs)})'
                f' {" ".join(self.items)}'
                f' (sheet_instances (path "/" (page "1"))))\n')


RED = (200, 0, 0)
BLUE = (0, 0, 160)


def stage(sh, X0, Y0, mod, n0, s):
    """One input stage, X0/Y0 in grid units; s = reference suffix."""
    g = lambda v: v * G  # noqa: E731
    yT, yB = Y0 - 1, Y0 + 1
    A = X0 + 36          # AD8138 centre; input pins at A-4, outputs at A+4
    NS, NG, NO = X0 + 14, X0 + 27, A + 8   # shunt node, gain node, output node
    # SMA, mirrored: centre pin right (X0+2), shield down (X0, yT+2)
    sh.sym("Connector:Conn_Coaxial", f"J1{s}", "SMA", g(X0), g(yT), mirror=True,
           ref_at=(g(X0) - 3, g(yT) - 5), val_at=(g(X0) - 3, g(yT) - 3.2))
    sh.wire((g(X0), g(yT + 2)), (g(X0), g(yT + 3)))
    sh.gnd(g(X0), g(yT + 3), n0)
    # series element (centre X0+8, pins X0+6.5 / X0+9.5)
    if mod:
        sh.hres("Device:C", f"CIN{s}", "100n", g(X0 + 8), g(yT))
        sh.box(g(X0 + 5.5), g(yT) - 6.2, g(X0 + 11), g(yT) + 1.8, RED)
        sh.text("вместо Rser 200", g(X0 + 5.5), g(yT) - 9.5, 1.27, RED)
    else:
        sh.hres("Device:R", f"RSER{s}", "200", g(X0 + 8), g(yT))
    sh.wire((g(X0 + 2), g(yT)), (g(X0 + 6.5), g(yT)))
    sh.wire((g(X0 + 9.5), g(yT)), (g(NS), g(yT)), (g(X0 + 18.5), g(yT)))
    sh.junction(g(NS), g(yT))
    # shunt 62 -> GND (vertical, pins at yT / yT+3)
    sh.sym("Device:R", f"RSH{s}", "62" + ("  (оставить)" if mod else ""), g(NS), g(yT + 1.5),
           ref_at=(g(NS) - 2.2, g(yT + 1.0)), val_at=(g(NS) - 2.2, g(yT + 2.0)), just="right")
    sh.wire((g(NS), g(yT + 3)), (g(NS), g(yT + 4)))
    sh.gnd(g(NS), g(yT + 4), n0 + 1)
    rg = "360  (было 200)" if mod else "200"
    rf = "2.2k  (было 200)" if mod else "200"
    # RG1 -> +IN (centre X0+20, pins X0+18.5 / X0+21.5)
    sh.hres("Device:R", f"RG1{s}", rg, g(X0 + 20), g(yT))
    sh.wire((g(X0 + 21.5), g(yT)), (g(NG), g(yT)), (g(A - 4), g(yT)))
    sh.junction(g(NG), g(yT))
    # RG2 -> -IN, other end to GND
    sh.hres("Device:R", f"RG2{s}", rg, g(X0 + 20), g(yB), below=True)
    sh.wire((g(X0 + 21.5), g(yB)), (g(NG), g(yB)), (g(A - 4), g(yB)))
    sh.junction(g(NG), g(yB))
    # RBAL: balances the -IN side to the source impedance seen by RG1
    # (stock 62 || (200 + 50) = 49.7 -> 50; mod 62 || 50 = 27.7 -> 22, closer than 39)
    sh.wire((g(X0 + 18.5), g(yB)), (g(X0 + 17), g(yB)), (g(X0 + 17), g(yB + 2.5)))
    sh.sym("Device:R", f"RBAL{s}", "22" if mod else "50", g(X0 + 17), g(yB + 4),
           ref_at=(g(X0 + 17) + 2.2, g(yB + 3.6)), val_at=(g(X0 + 17) + 2.2, g(yB + 4.5)))
    sh.wire((g(X0 + 17), g(yB + 5.5)), (g(X0 + 17), g(yB + 6.5)))
    sh.gnd(g(X0 + 17), g(yB + 6.5), n0 + 2)
    if mod:
        sh.box(g(X0 + 16.5), g(yT) - 6.2, g(X0 + 26), g(yB) + 6.2, RED)
        sh.box(g(X0 + 15.8), g(yB + 2.5) + 0.3, g(X0 + 21.5), g(yB + 5.5) + 0.3, RED)
        sh.text("было 50 ->", g(X0 + 11), g(yB + 4.6), 1.27, RED)
    # AD8138
    sh.sym("fe:AD8138", f"U3{s}", "AD8138", g(A), g(Y0),
           pins=tuple(p[0] for p in AD8138_PINS),
           ref_at=(g(A) + 2, g(Y0) - 6.6), val_at=(g(A) - 11, g(Y0) - 6.6))
    # RF1: -OUT -> +IN (top), RF2: +OUT -> -IN (bottom); centre A, pins A-1.5/A+1.5
    for yy, ry, k, below in ((yT, yT - 6, 1, False), (yB, yB + 6, 2, True)):
        sh.hres("Device:R", f"RF{k}{s}", rf, g(A), g(ry), below=below)
        sh.wire((g(NG), g(yy)), (g(NG), g(ry)), (g(A - 1.5), g(ry)))
        sh.wire((g(A + 1.5), g(ry)), (g(NO), g(ry)), (g(NO), g(yy)))
        sh.wire((g(A + 4), g(yy)), (g(NO), g(yy)), (g(NO + 5), g(yy)))
        sh.junction(g(NO), g(yy))
    if mod:
        sh.box(g(A - 3), g(yT - 6) - 6.2, g(A + 7), g(yT - 6) + 1.8, RED)
        sh.box(g(A - 3), g(yB + 6) - 1.8, g(A + 7), g(yB + 6) + 6.2, RED)
    sfx = "_mod" if mod else ""
    sh.label("VINA" + sfx, g(NO + 5), g(yT))
    sh.label("VINB" + sfx, g(NO + 5), g(yB))
    sh.text("-> RC фильтр -> AD9226  (не трогать)", g(NO + 5), g(yB) + 1.5)
    # supplies
    sh.wire((g(A), g(Y0 - 3)), (g(A), g(Y0 - 4)))
    sh.label("+5V", g(A), g(Y0 - 4), 90)
    sh.wire((g(A), g(Y0 + 3)), (g(A), g(Y0 + 4)))
    sh.label("CML", g(A), g(Y0 + 4), 270, "right")
    sh.gnd(g(A + 1), g(Y0 + 3), n0 + 3)


def main():
    libs = [lib_entry("Device.kicad_sym", "Device", "R"),
            lib_entry("Device.kicad_sym", "Device", "C"),
            lib_entry("power.kicad_sym", "power", "GND"),
            lib_entry("power.kicad_sym", "power", "PWR_FLAG"),
            lib_entry("Connector.kicad_sym", "Connector", "Conn_Coaxial"),
            ad8138_lib()]
    sh = Sheet()
    X0 = 8
    sh.text("БЫЛО (сток)", g0(X0) - 6, g0(14), 3.0, BLUE, True)
    sh.text("усиление 0.38 (-8.4 дБ), полная шкала ~ +24 dBm, Zвх ~ 247 Ом",
            g0(X0) - 6, g0(16.5), 1.8, BLUE)
    stage(sh, X0, 32, False, 1, "")
    sh.text("СТАЛО (мод IW5ALZ)   красная рамка = перепаять", g0(X0) - 6, g0(50), 3.0, RED, True)
    sh.text("усиление 7.11 (+17.0 дБ), итого +25.4 дБ;  полная шкала ~ -1.5 dBm, Zвх ~ 53 Ом",
            g0(X0) - 6, g0(52.5), 1.8, RED)
    stage(sh, X0, 68, True, 11, "M")
    sh.sym("power:PWR_FLAG", "#FLG001", "PWR_FLAG", g0(8), g0(90), pins=("1",), show_ref=False)
    sh.wire((g0(8), g0(90)), (g0(8), g0(92)))
    sh.gnd(g0(8), g0(92), 30)

    nx = g0(80)
    sh.text("ЧТО ПЕРЕПАЯТЬ (вокруг U3 = AD8138)", nx, g0(14), 3.0, RED, True)
    sh.text(
        "1. Rser 200 Ом (последовательно от центра SMA)\n"
        "     ->  ВЫПАЯТЬ, поставить конденсатор 100 нФ\n"
        "     (развязка по постоянке: антенна с КЗ по DC не посадит смещение)\n"
        "2. Rsh 62 Ом (на землю после Rser)  ->  ОСТАВИТЬ\n"
        "     Zвх станет 62 || 360 = 53 Ом\n"
        "3. RG1, RG2 = 200 Ом (вход -> ножки 8 и 1 U3)  ->  360 Ом, ОБА (у IW5ALZ 390)\n"
        "4. RF1, RF2 = 200 Ом (обр. связь 5->8 и 4->1 U3)  ->  2.2 кОм, ОБА\n"
        "     RG и RF менять парами, иначе разбаланс (чётные гармоники)\n"
        "5. RBAL 50 Ом (RG2 -> RBAL -> земля)  ->  22 Ом (идеал 27.7; 22 ближе, чем 39)\n"
        "     балансирует -IN под сопротивление источника со стороны +IN:\n"
        "     было 62 || (200 + 50) = 49.7 Ом (отсюда 50), стало 62 || 50 = 27.7 Ом.\n"
        "     Оставить 50 можно - работает, но разбаланс ~5% (чуть хуже чётные гармоники)\n"
        "6. RC-фильтр U3 -> AD9226, VOCM (CML), питание  ->  НЕ ТРОГАТЬ\n"
        "\n"
        "Усиление (формула IW5ALZ):\n"
        "  было:  делитель (200||62)/(200 + 200||62) = 0.19,  x (200/200 + 1) = 0.38\n"
        "  стало: делителя нет,  2200/360 + 1 = 7.11  (у IW5ALZ 2200/390 + 1 = 6.64)",
        nx, g0(18), 1.8)
    sh.text("КАК НАЙТИ (питание ВЫКЛ., прозвонка)", nx, g0(50), 3.0, BLUE, True)
    sh.text(
        "Rser : один вывод звонится на центр SMA\n"
        "Rsh  : один вывод на Rser, другой на землю\n"
        "RF   : соединяет ножки U3  1 <-> 4  или  8 <-> 5\n"
        "RG   : один вывод на ножку 1 или 8 U3, другой к Rsh или к RBAL\n"
        "RBAL : между RG2 и землёй (прозвонено: 50 Ом)\n"
        "\n"
        "По фото маркировка 2000 (= 200 Ом): R2, R6, резистор справа от R6,\n"
        "R14, резистор над C14 - кандидаты в Rser / RG / RF.\n"
        "R16, R8, R17 (у SMA) и R9, R11 на фото не читаются.\n"
        "ПОЗИЦИОННЫЕ НОМЕРА НЕ ПРОВЕРЕНЫ - СНАЧАЛА ПРОЗВОНИТЬ!\n"
        "\n"
        "Цоколёвка AD8138, SOIC-8 (точка/ключ = ножка 1):\n"
        "  1 -IN     2 VOCM    3 V+     4 +OUT\n"
        "  5 -OUT    6 V-      7 NC     8 +IN\n"
        "\n"
        "Какой вход (+IN или -IN) получает сигнал - неважно,\n"
        "каскад симметричный, замена номиналов та же.\n"
        "Детали: 0603/0805, 1%;  C 100 нФ X7R.",
        nx, g0(54), 1.8)

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, PROJECT + ".kicad_sch"), "w") as f:
        f.write(sh.render(libs))
    with open(os.path.join(OUT_DIR, "fe.kicad_sym"), "w") as f:
        f.write('(kicad_symbol_lib (version 20241209) (generator "gen_frontend_mod_sch")'
                f' (generator_version "9.0") {ad8138_lib().replace("fe:AD8138", "AD8138")})\n')
    with open(os.path.join(OUT_DIR, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table (version 7)\n  (lib (name "fe") (type "KiCad") (uri "${KIPRJMOD}/fe.kicad_sym")'
                ' (options "") (descr "project symbols"))\n)\n')
    pro = os.path.join(OUT_DIR, PROJECT + ".kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w") as f:
            f.write('{"meta": {"filename": "frontend_mod.kicad_pro", "version": 1}}\n')
    print("wrote", OUT_DIR)


def g0(v):
    return v * G


if __name__ == "__main__":
    main()
