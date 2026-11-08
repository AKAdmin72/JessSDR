#!/usr/bin/env python3
"""PC side of sdr/ (40m SDR v1) over the BL616 UART (1 Mbaud).

  python3 tools/sdr.py info
  python3 tools/sdr.py freq 7101000        tune (Hz)
  python3 tools/sdr.py mode lsb|usb
  python3 tools/sdr.py iq   [-o f.npy]     4096 DDC I/Q samples @ 52.726 kHz: spectrum peaks
  python3 tools/sdr.py audio [-o f.wav]    AGC audio samples @ 13.181 kHz: tone + wav
  python3 tools/sdr.py spec [-o f.npy|f.png] one on-screen spectrum line (320 bins, L8) -> peaks/floor
  python3 tools/sdr.py eq 0|1              automatic noise floor off/on
  python3 tools/sdr.py base SB WB          display bases (L8 at spectrum bottom / waterfall black)
  python3 tools/sdr.py gain DB             front-end gain for the S-meter (25 = AD8138 mod)
  python3 tools/sdr.py zoom 0|1|2          spectrum scale 1 / 1.5 / 2 px per dB
  python3 tools/sdr.py wide 0|1            spectrum span +-16.5 / +-33 kHz
"""
import argparse, os, struct, sys, wave
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adc65 import Link

FS_IQ = 64.8e6 / 1229
FS_AUD = FS_IQ / 4
BIN_HZ = FS_IQ / 512
# L8 -> dBm per bin (approx.): full-scale ADC tone (+24 dBm, STOCK front end; subtract
# FE_GAIN_DB for the AD8138 gain mod) gives |z| = 2^19 at the
# DDC, |X| = 2^19 * 512 * 0.35875 (Blackman-Harris) -> L8 = 16*log2|X|
FE_GAIN_DB = 25   # AD8138 gain mod 2026-11-07 (estimate, re-measure)
SPEC_DBM0 = 24 - FE_GAIN_DB - 16 * np.log2(2 ** 19 * 512 * 0.35875) * 0.3763


def cmd_info(l):
    l.write(b"?")
    r = l.read(4)
    return dict(version=r[1], level=r[2], usb=r[3] & 1, wide=(r[3] >> 1) & 1,
                level_dbm=round(-90.7 + 6.02 * r[2] / 8 - FE_GAIN_DB, 1))


def dump(l, c, nbytes):
    l.write(c)
    h = l.read(3)
    n = h[1] | h[2] << 8
    return l.read(n * nbytes, timeout=15), n


def peaks(x, fs, k=5, complex_=True):
    n = len(x)
    w = np.blackman(n)
    X = np.fft.fftshift(np.fft.fft((x - x.mean()) * w)) if complex_ else np.fft.rfft((x - x.mean()) * w)
    f = np.fft.fftshift(np.fft.fftfreq(n, 1 / fs)) if complex_ else np.fft.rfftfreq(n, 1 / fs)
    p = 20 * np.log10(np.abs(X) / (w.sum() / (1 if complex_ else 2)) + 1e-9)
    idx = []
    for i in np.argsort(p)[::-1]:
        if all(abs(i - j) > 8 for j in idx):
            idx.append(i)
        if len(idx) == k:
            break
    return [(f[i], p[i]) for i in idx]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["info", "freq", "mode", "iq", "audio", "spec", "base", "eq", "gain", "zoom", "wide"])
    ap.add_argument("value", nargs="*")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    l = Link()
    l.sync()
    try:
        if a.cmd == "info":
            print(cmd_info(l))
        elif a.cmd == "freq":
            f = int(float(a.value[0]))
            l.write(b"f" + struct.pack("<I", f))
            assert l.read(1) == b"F"
            print(f"tuned to {f} Hz")
        elif a.cmd == "mode":
            l.write(b"m" + bytes([1 if a.value[0].lower() == "usb" else 0]))
            assert l.read(1) == b"M"
            print(f"mode {a.value[0].upper()}")
        elif a.cmd == "iq":
            raw, n = dump(l, b"q", 6)
            b = np.frombuffer(raw, np.uint8).reshape(n, 6).astype(np.int32)
            def s24(lo, mid, hi):
                v = lo | mid << 8 | hi << 16
                return np.where(v & 0x800000, v - (1 << 24), v)
            i = s24(b[:, 0], b[:, 1], b[:, 2]); q = s24(b[:, 3], b[:, 4], b[:, 5])
            z = i + 1j * q
            print(f"{n} I/Q samples: rms {np.sqrt(np.mean(np.abs(z - z.mean())**2)):.1f} LSB, "
                  f"max |I|,|Q| {np.abs(i).max()},{np.abs(q).max()}")
            for f, p in peaks(z, FS_IQ):
                print(f"  {f:+9.1f} Hz  {p:6.1f} dB(LSB)")
            if a.out:
                np.save(a.out, z)
        elif a.cmd == "audio":
            raw, n = dump(l, b"a", 2)
            x = np.frombuffer(raw, "<i2").astype(float)
            print(f"{n} audio samples: rms {x.std():.0f}, peak {np.abs(x).max():.0f}")
            for f, p in peaks(x, FS_AUD, complex_=False):
                print(f"  {f:8.1f} Hz  {p:6.1f} dB")
            if a.out:
                with wave.open(a.out, "wb") as w:
                    w.setnchannels(1); w.setsampwidth(2); w.setframerate(int(round(FS_AUD)))
                    w.writeframes(x.astype("<i2").tobytes())
                print(f"saved {a.out}")
        elif a.cmd == "gain":
            l.write(b"g" + bytes([int(a.value[0]) & 255]))
            assert l.read(1) == b"G"
            print(f"S-meter front-end gain {int(a.value[0])} dB")
        elif a.cmd == "wide":
            l.write(b"w" + bytes([int(a.value[0]) & 1]))
            assert l.read(1) == b"W"
            print("spectrum span " + ("+-33 kHz" if int(a.value[0]) else "+-16.5 kHz"))
        elif a.cmd == "zoom":
            l.write(b"z" + bytes([int(a.value[0]) & 3]))
            assert l.read(1) == b"Z"
            print(f"spectrum zoom {a.value[0]}")
        elif a.cmd == "eq":
            l.write(b"e" + bytes([int(a.value[0]) & 1]))
            assert l.read(1) == b"E"
            print("auto floor", "on" if int(a.value[0]) & 1 else "off")
        elif a.cmd == "base":
            sb, wb = int(a.value[0]), int(a.value[1])
            l.write(b"b" + bytes([sb, wb]))
            assert l.read(1) == b"B"
            print(f"spectrum base {sb}, waterfall base {wb} (L8 units, 0.376 dB)")
        elif a.cmd == "spec":
            bin_hz = BIN_HZ * (2 if cmd_info(l)["wide"] else 1)   # wide span: fs = 64.8M/614
            raw, n = dump(l, b"s", 2)
            l8 = np.frombuffer(raw, "<u2").astype(float)
            db = l8 * 10 * np.log10(2) / 8
            f = (np.arange(n) - 160) * bin_hz
            fl = np.median(l8)
            print(f"{n} bins, {bin_hz:.2f} Hz each; floor (median) L8 {fl:.0f} = {fl * 0.3763:.1f} dB"
                  f" -> ~{SPEC_DBM0 + fl * 0.3763:.1f} dBm per bin")
            for i in np.argsort(l8)[::-1][:8]:
                print(f"  {f[i]:+9.1f} Hz  L8 {l8[i]:.0f}  {db[i] - fl * 0.3763:+5.1f} dB over floor")
            if a.out and a.out.endswith(".npy"):
                np.save(a.out, l8); print(f"saved {a.out}")
            elif a.out:
                import matplotlib; matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                plt.figure(figsize=(10, 4)); plt.plot(f / 1e3, db - db.max()); plt.grid(True)
                plt.xlabel("kHz from dial"); plt.ylabel("dB"); plt.savefig(a.out, dpi=100)
                print(f"saved {a.out}")
    finally:
        l.close()


if __name__ == "__main__":
    main()
