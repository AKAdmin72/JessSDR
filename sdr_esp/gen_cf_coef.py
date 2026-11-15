#!/usr/bin/env python3
"""Taps for ssb_demod.v's programmable sideband filter (255 complex taps,
Q1.17, {re, im} per 36-bit word). Same formula as esp32/main/fpga.c
fpga_set_filter() - keep them in sync.

  python3 gen_cf_coef.py                    -> cf_coef.hex (power-up: LSB 300..2700 Hz)
  python3 gen_cf_coef.py LO HI out.hex      -> passband LO..HI Hz (negative = LSB side)
"""
import sys
import numpy as np

FS = 64.8e6 / 1229 / 4       # 13181.4 Hz, ssb_demod's output rate
N = 255


def taps(lo, hi):
    fc, f0 = (hi - lo) / 2, (hi + lo) / 2
    n = np.arange(N) - (N - 1) // 2
    h = 2 * fc / FS * np.sinc(2 * fc / FS * n) * np.blackman(N) * np.exp(2j * np.pi * f0 * n / FS)
    re = np.clip(np.round(h.real * 2 ** 17), -131072, 131071).astype(int)
    im = np.clip(np.round(h.imag * 2 ** 17), -131072, 131071).astype(int)
    return re, im


def write_hex(path, re, im):
    with open(path, "w") as f:
        for r, i in zip(list(re) + [0], list(im) + [0]):      # 256th word = 0
            f.write("%09x\n" % (((r & 0x3FFFF) << 18) | (i & 0x3FFFF)))


if __name__ == "__main__":
    if len(sys.argv) == 4:
        write_hex(sys.argv[3], *taps(float(sys.argv[1]), float(sys.argv[2])))
    else:
        write_hex("cf_coef.hex", *taps(-2700.0, -300.0))
