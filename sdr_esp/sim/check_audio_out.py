#!/usr/bin/env python3
"""sim/audio_out48.txt (tb_audio_out): 3 kHz tone through audio_up4 +
audio_i2s at 48.214 kHz. Prints the gain and the worst spur 200 Hz..24 kHz
away from the tone; PASS if gain within 0.6 dB and spurs < -45 dB.
'old' = the same tone through linear interpolation only, for comparison."""
import sys
import numpy as np
FS = 64.8e6 / 1344
y = np.loadtxt("sim/audio_out48.txt")[2000:][:8192]
def analyse(y):
    w = np.blackman(len(y))
    Y = np.abs(np.fft.rfft(y * w)) / (w.sum() / 2)
    f = np.fft.rfftfreq(len(y), 1 / FS)
    near = abs(f - 3000) < 200
    tone = Y[near].max()
    sp = Y[~near & (f > 200)]
    return 20 * np.log10(tone / 16000), 20 * np.log10(sp.max() / tone), f[~near & (f > 200)][np.argmax(sp)]
g, s, fs_ = analyse(y)
print("new: gain %.2f dB, worst spur %.1f dB at %.0f Hz" % (g, s, fs_))
# old path model: linear interpolation of the 13.18 kHz samples at 48.21 kHz
t = np.arange(len(y) + 4000) / FS
fin = 64.8e6 / 4916
x = 16000 * np.sin(2 * np.pi * 3000 * np.arange(int(t[-1] * fin) + 2) / fin)
yo = np.interp(t * fin, np.arange(len(x)), x)[2000:][:8192]
g0, s0, f0 = analyse(yo)
print("old (linear only, model): gain %.2f dB, worst spur %.1f dB at %.0f Hz" % (g0, s0, f0))
ok = abs(g) < 0.6 and s < -45      # -0.46: mostly FFT bin scalloping (3 kHz is off-bin)
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
