#!/usr/bin/env python3
"""Taps for audio_up4.v: x4 interpolation 13.18 -> 52.73 kHz before
audio_i2s.v's linear interpolation to 48.21 kHz (that alone left images of
the audio around 13.18 kHz only ~21 dB down: 3 kHz -> 10.2 kHz).
96 taps (24 per phase), Kaiser low-pass, pass <= 4.5 kHz, stop >= 7.2 kHz
(13.18 - 6 kHz, the AM decimator's edge), gain 4 (zero stuffing), Q2.16.
Prints the Verilog case body for audio_up4.v's coef() and the response.

  python3 gen_up4_coef.py > /dev/null   (paste the case lines by hand)
"""
import numpy as np
from scipy import signal

FS = 64.8e6 / 1229        # 52.727 kHz
NT = 96
h = signal.firwin(NT, 5850, fs=FS, window=("kaiser", 7.5)) * 4
q = np.round(h * 2 ** 16).astype(int)
assert q.max() < 2 ** 17 and q.min() >= -2 ** 17

if __name__ == "__main__":
    import sys
    for k, v in enumerate(q):
        print("            7'd%d: coef = %s18'sd%d;" % (k, "-" if v < 0 else " ", abs(v)))
    w, H = signal.freqz(q / 2 ** 16 / 4, worN=8192, fs=FS)
    db = 20 * np.log10(abs(H) + 1e-12)
    for f in (1000, 3000, 4500, 6000, 7200, 10000, 13000, 20000):
        sys.stderr.write("%6d Hz %7.2f dB\n" % (f, db[np.argmin(abs(w - f))]))
    sys.stderr.write("stop >= 7.2k max %.1f dB, phase sums %s\n" % (
        db[w >= 7200].max(), [int(q[p::4].sum()) for p in range(4)]))
