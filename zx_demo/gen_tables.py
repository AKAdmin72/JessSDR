#!/usr/bin/env python3
"""Generates the ROM tables of the ZX-style demo -> tables.v

  font_rom    8x8 glyphs, ASCII 32..95 (from the Linux console VGA8 font)
  text_rom    scroller message (512 chars, wraps)
  title_rom   32-char title line
  sin_rom     sin, 256 steps/turn, signed Q1.7
  pose_rom    dancer key poses (angles: 256/turn, 0=right 64=down 128=left 192=up)
  bone_rom    skeleton: how each bone is built and drawn
  dance_rom   pose sequence: 4 dance sections x 8 beats

Poses/dance are original. Re-run after editing: python3 gen_tables.py
"""
import gzip, math, os

HERE = os.path.dirname(os.path.abspath(__file__))

out = []
def emit(s=""):
    out.append(s)

def case_rom(name, in_w, out_w, items, default=0, comment=""):
    emit(f"// {comment}" if comment else "")
    # function + continuous assign (not always @*): iverilog never evaluates
    # an always @* whose input stays at its time-0 value
    emit(f"module {name} (input wire [{in_w-1}:0] a, output wire [{out_w-1}:0] d);")
    emit(f"    function [{out_w-1}:0] f(input [{in_w-1}:0] v);")
    emit("        case (v)")
    for k, v in items:
        emit(f"            {in_w}'d{k}: f = {out_w}'h{v & ((1 << out_w) - 1):x};")
    emit(f"            default: f = {out_w}'h{default:x};")
    emit("        endcase")
    emit("    endfunction")
    emit("    assign d = f(a);")
    emit("endmodule")
    emit()

# ---------------- font ----------------
d = gzip.open("/usr/share/consolefonts/Lat15-VGA8.psf.gz").read()
assert d[0] == 0x36 and d[1] == 0x04 and d[3] == 8
items = []
for c in range(32, 96):
    g = d[4 + c * 8: 4 + c * 8 + 8]
    for r in range(8):
        items.append(((c - 32) << 3 | r, g[r]))
case_rom("font_rom", 9, 8, items, comment="a = {char-32 (6b), row (3b)} -> 8 pixels, MSB = left")

# ---------------- texts ----------------
MSG = ("      *** HELLO SPECTRUM FANS! ***   THIS LITTLE DEMO RUNS ON A SIPEED TANG NANO 20K "
       "FPGA - NO CPU, NO PROGRAM, JUST LOGIC GATES.   THE GIRL IS DRAWN BY FORWARD KINEMATICS "
       "INTO A 256X192 SCREEN WITH ATTRIBUTES AND SENT TO A 2.8 INCH ILI9341 OVER SPI.   GREETINGS TO EVERY SINCLAIR SCENER OUT THERE "
       "- KEEP THE BORDER FLASHING!   ")
assert len(MSG) <= 512, len(MSG)
MSG = MSG + " " * (512 - len(MSG))
case_rom("text_rom", 9, 6, [(i, ord(ch) - 32) for i, ch in enumerate(MSG)],
         comment="scroller text, ASCII-32")
TITLE = "   TANG NANO 20K  ZX DEMO 2026  "
assert len(TITLE) == 32
case_rom("title_rom", 5, 6, [(i, ord(ch) - 32) for i, ch in enumerate(TITLE)],
         comment="title line, ASCII-32")

# ---------------- sine ----------------
case_rom("sin_rom", 8, 8, [(i, round(127 * math.sin(2 * math.pi * i / 256)) & 0xFF) for i in range(256)],
         comment="sin(a*2pi/256)*127, signed")

# ---------------- dancer ----------------
# fields: torso uaL faL uaR faR thL shL thR shR hair dy flare   (L = screen-left limb)
F = ["torso", "uaL", "faL", "uaR", "faR", "thL", "shL", "thR", "shR", "hair", "dy", "flare"]
POSES = [
    dict(torso=192, uaL=74,  faL=70,  uaR=54,  faR=58,  thL=67, shL=64, thR=61, shR=64, hair=0,   dy=0,  flare=0),   # 0 stand
    dict(torso=192, uaL=165, faL=160, uaR=219, faR=224, thL=70, shL=64, thR=58, shR=64, hair=-6,  dy=-3, flare=4),   # 1 arms up V
    dict(torso=189, uaL=104, faL=34,  uaR=214, faR=196, thL=68, shL=64, thR=58, shR=66, hair=6,   dy=0,  flare=2),   # 2 hand on hip, wave
    dict(torso=195, uaL=104, faL=34,  uaR=214, faR=232, thL=68, shL=64, thR=58, shR=66, hair=-6,  dy=0,  flare=2),   # 3 wave other way
    dict(torso=194, uaL=134, faL=146, uaR=250, faR=238, thL=64, shL=64, thR=10, shR=70, hair=-4,  dy=-4, flare=8),   # 4 right knee up
    dict(torso=190, uaL=134, faL=146, uaR=250, faR=238, thL=118, shL=58, thR=64, shR=64, hair=4,  dy=-4, flare=8),   # 5 left knee up
    dict(torso=192, uaL=112, faL=150, uaR=16,  faR=234, thL=94, shL=46, thR=34, shR=82, hair=8,   dy=10, flare=6),   # 6 crouch
    dict(torso=192, uaL=204, faL=236, uaR=180, faR=148, thL=66, shL=64, thR=62, shR=64, hair=16,  dy=-4, flare=14),  # 7 twirl, hands over head
    dict(torso=196, uaL=92,  faL=92,  uaR=212, faR=212, thL=76, shL=64, thR=56, shR=62, hair=-4,  dy=0,  flare=2),   # 8 disco point up
    dict(torso=188, uaL=160, faL=160, uaR=36,  faR=36,  thL=72, shL=66, thR=56, shR=64, hair=4,   dy=2,  flare=2),   # 9 disco point down
    dict(torso=186, uaL=48,  faL=20,  uaR=82,  faR=104, thL=82, shL=70, thR=56, shR=60, hair=8,   dy=2,  flare=6),   # 10 step left
    dict(torso=198, uaL=46,  faL=24,  uaR=80,  faR=108, thL=72, shL=68, thR=46, shR=58, hair=-8,  dy=2,  flare=6),   # 11 step right
]
items = []
for p, pose in enumerate(POSES):
    for f, name in enumerate(F):
        items.append((p << 4 | f, pose[name] & 0xFF))
case_rom("pose_rom", 8, 8, items, comment="a = {pose (4b), field (4b)}; fields " + " ".join(F))

DANCE = [
    [0, 0, 2, 3, 2, 3, 0, 1],      # 0 intro: wave
    [10, 11, 10, 11, 4, 5, 4, 5],  # 1 part A: steps + knees
    [8, 9, 8, 9, 1, 6, 1, 7],      # 2 part B: disco
    [7, 7, 1, 1, 6, 6, 0, 0],      # 3 breakdown: slow
]
case_rom("dance_rom", 5, 4, [(s << 3 | b, p) for s, seq in enumerate(DANCE) for b, p in enumerate(seq)],
         comment="a = {section (2b), beat (3b)} -> pose")

# bones: parent point, out point, angle var, angle offset, spread mult k, length, brush radius (0 = not drawn)
# points: 0 hip 1 neck 2 head 3 shL 4 elL 5 hdL 6 shR 7 elR 8 hdR 9 hpL 10 knL 11 ftL
#         12 hpR 13 knR 14 ftR 15 waist 16 hairL 17 hairR 18 scratch
# angle vars: 0 zero 1 torso 2 uaL 3 faL 4 uaR 5 faR 6 thL 7 shL 8 thR 9 shR 10 hair
BONES = [
    (0, 15, 1, 0, 0, 8, 0),     # waist point (hip + 8 along torso)
    (0, 1, 1, 0, 0, 40, 4),     # torso hip -> neck
    (15, 18, 1, 0, 0, 16, 0),   # chest point
    (18, 18, 0, 0, 0, 0, 6),    # chest
    (1, 2, 1, 0, 0, 13, 2),     # neck
    (2, 2, 0, 0, 0, 0, 10),     # head
    (1, 3, 1, -64, 0, 9, 3),    # shoulder L
    (1, 6, 1, 64, 0, 9, 3),     # shoulder R
    (3, 4, 2, 0, 0, 24, 3),     # upper arm L
    (4, 5, 3, 0, 0, 22, 2),     # forearm L
    (5, 5, 0, 0, 0, 0, 3),      # hand L
    (6, 7, 4, 0, 0, 24, 3),
    (7, 8, 5, 0, 0, 22, 2),
    (8, 8, 0, 0, 0, 0, 3),
    (0, 9, 1, -64, 0, 6, 5),    # hip L
    (0, 12, 1, 64, 0, 6, 5),
    (9, 10, 6, 0, 0, 32, 4),    # thigh L
    (10, 11, 7, 0, 0, 32, 3),   # shin L
    (11, 18, 0, 128, 0, 7, 2),  # foot L
    (12, 13, 8, 0, 0, 32, 4),
    (13, 14, 9, 0, 0, 32, 3),
    (14, 18, 0, 0, 0, 7, 2),
] + [(15, 18, 0, 64, k, 30, 3) for k in range(-3, 4)] + [   # skirt fan from the waist
    (2, 18, 10, 100, 0, 24, 3),  # pigtails from the head centre
    (2, 18, 10, 28, 0, 24, 3),
]
NBONES = len(BONES)
assert NBONES <= 32
items = []
for i, (pa, po, av, off, k, ln, r) in enumerate(BONES):
    v = (pa << 34) | (po << 29) | (av << 25) | ((off & 0xFF) << 17) | ((k & 7) << 14) | (ln << 8) | r
    items.append((i, v))
case_rom("bone_rom", 5, 39, items,
         comment=f"{NBONES} bones: {{parent 5, out 5, avar 4, off 8, k 3 (signed), len 6, pad 2, r 4}}")

hdr = (f"// GENERATED by gen_tables.py - do not edit.\n"
       f"// {NBONES} bones\n")
with open(os.path.join(HERE, "tables.v"), "w") as f:
    f.write(hdr + "\n".join(out))
print(f"tables.v written: bones={NBONES}, msg={len(MSG)}")
