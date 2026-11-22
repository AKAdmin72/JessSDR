#!/usr/bin/env python3
"""Spectrum of sim/up4_out.txt (tb_audio_up4): 3 kHz tone gain and the
strongest image (>= 7 kHz), relative to the tone. Exit 1 on failure."""
import sys
import numpy as np
FS = 64.8e6 / 1229
y = np.loadtxt("sim/up4_out.txt")[400:]          # skip the start-up
y = y[:4096]
w = np.blackman(len(y))
Y = np.abs(np.fft.rfft(y * w)) / (w.sum() / 2)
f = np.fft.rfftfreq(len(y), 1 / FS)
tone = Y[np.argmin(abs(f - 3000)) - 3:np.argmin(abs(f - 3000)) + 4].max()
img = Y[f >= 7000].max()
g = 20 * np.log10(tone / 16000)
r = 20 * np.log10(img / tone)
print("tone 3 kHz: %.2f dB gain, worst image >= 7 kHz: %.1f dB (at %.0f Hz)" % (g, r, f[f >= 7000][np.argmax(Y[f >= 7000])]))
ok = abs(g) < 0.2 and r < -70
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
