#!/usr/bin/env python3
"""PC side of adc65/ (AD9226 @ 64.8 MSPS, UART dump through the BL616).

  python3 tools/adc65.py info
  python3 tools/adc65.py cap [-p PHASE] [-o file.npy]   capture + analysis
  python3 tools/adc65.py sweep                           try all 16 clock phases
  python3 tools/adc65.py phase N                         just set the phase

Analysis: mean / rms noise in LSB, min/max, OTR count, strongest tone
(frequency, dBFS), SINAD/ENOB around it (Blackman-Harris window), noise
floor in dBFS/bin and dBm/Hz (50-ohm input, full scale assumed +-5 V =
+24 dBm sine - change FS_DBM if measured differently), and a "glitch"
count (samples far off the local trend) that exposes bad capture timing.
Talks libftdi directly (interface B) because the udev rule unbinds ftdi_sio.
"""
import argparse, ctypes, sys, time
import numpy as np

VID, PID, IFACE_B = 0x0403, 0x6010, 2
BAUD = 1000000
FS_HZ = 64.8e6
FS_DBM = 24.0          # full-scale sine at the SMA input (stock module, +-5 V on 50 ohm)


class Link:
    def __init__(self, baud=BAUD):
        self.lib = lib = ctypes.CDLL("libftdi1.so.2")
        lib.ftdi_new.restype = ctypes.c_void_p
        lib.ftdi_get_error_string.restype = ctypes.c_char_p
        self.ctx = ctypes.c_void_p(lib.ftdi_new())
        self.chk(lib.ftdi_set_interface(self.ctx, IFACE_B), "set_interface")
        self.chk(lib.ftdi_usb_open(self.ctx, VID, PID), "usb_open (board connected? term.py closed?)")
        self.chk(lib.ftdi_set_baudrate(self.ctx, baud), "set_baudrate")
        self.chk(lib.ftdi_set_line_property(self.ctx, 8, 0, 0), "8N1")
        self.chk(lib.ftdi_setflowctrl(self.ctx, 0), "flowctrl")
        self.chk(lib.ftdi_setdtr_rts(self.ctx, 1, 1), "dtr_rts")
        lib.ftdi_set_latency_timer(self.ctx, 2)
        lib.ftdi_usb_purge_buffers(self.ctx)
        self.buf = ctypes.create_string_buffer(65536)

    def chk(self, r, what):
        if r < 0:
            sys.exit(f"{what} failed ({r}): {self.lib.ftdi_get_error_string(self.ctx).decode()}")

    def write(self, data: bytes):
        self.chk(self.lib.ftdi_write_data(self.ctx, data, len(data)), "write")

    def read(self, n, timeout=10.0):
        out = bytearray()
        t0 = time.time()
        while len(out) < n:
            r = self.lib.ftdi_read_data(self.ctx, self.buf, min(65536, n - len(out)))
            if r < 0:
                self.chk(r, "read")
            if r > 0:
                out += self.buf.raw[:r]
                t0 = time.time()
            elif time.time() - t0 > timeout:
                sys.exit(f"timeout: got {len(out)} of {n} bytes")
            else:
                time.sleep(0.001)
        return bytes(out)

    def flush_in(self):
        time.sleep(0.05)
        while self.lib.ftdi_read_data(self.ctx, self.buf, 65536) > 0:
            pass

    def sync(self):
        """The BL616 bridge drops/holds the first bytes after the port is
        opened: ping with '?' until a reply comes back, then drain."""
        for _ in range(20):
            self.write(b"?")
            t0 = time.time()
            got = b""
            while time.time() - t0 < 0.3:
                r = self.lib.ftdi_read_data(self.ctx, self.buf, 4096)
                if r > 0:
                    got += self.buf.raw[:r]
                    if b"I" in got:
                        self.flush_in()
                        return
                time.sleep(0.005)
        sys.exit("no reply from the FPGA (adc65 bitstream loaded?)")

    def close(self):
        self.lib.ftdi_usb_close(self.ctx)


def info(link):
    link.write(b"?")
    r = link.read(4)
    if r[0:1] != b"I":
        sys.exit(f"bad info reply {r!r}")
    return dict(phase=r[1], clk_mhz=r[2], version=r[3])


def set_phase(link, p):
    link.write(bytes([ord("p"), p & 15]))
    r = link.read(2)
    if r != bytes([ord("P"), p & 15]):
        sys.exit(f"bad phase reply {r!r}")


def capture(link):
    link.write(b"c")
    h = link.read(5)
    if h[:2] != b"AD":
        sys.exit(f"bad capture header {h!r}")
    n = h[2] | h[3] << 8
    raw = np.frombuffer(link.read(2 * n, timeout=10), dtype="<u2")
    code = (raw & 0x0FFF).astype(np.int32)
    otr = (raw >> 15) & 1
    return code, otr, h[4]


def glitches(code):
    """Samples that jump far away from both neighbours' average."""
    x = code.astype(float)
    d = x[1:-1] - 0.5 * (x[:-2] + x[2:])
    mad = np.median(np.abs(d - np.median(d))) + 0.5
    return int(np.sum(np.abs(d) > 12 * mad + 40))


def analyse(code, otr, verbose=True):
    n = len(code)
    x = code - 2048.0
    res = dict(n=n, mean=x.mean() + 2048, rms=x.std(), min=int(code.min()), max=int(code.max()),
               otr=int(otr.sum()), glitches=glitches(code))
    w = np.kaiser(n, 20)                                           # main lobe ~+-7 bins, very low sidelobes
    X = np.fft.rfft((x - x.mean()) * w)
    p = np.abs(X) ** 2
    p_fs = (2048.0 * w.sum() / 2) ** 2                             # full-scale sine peak bin power
    pdb = 10 * np.log10(p / p_fs + 1e-30)
    p[:8] = 0                                                      # DC leakage
    k = int(np.argmax(p))
    span = 10
    sig = p[max(k - span, 0):k + span + 1].sum()
    noise_bins = np.ones(len(p), bool)
    noise_bins[:8] = False
    noise_bins[max(k - span, 0):k + span + 1] = False
    nb = noise_bins.sum()
    noise = p[noise_bins].sum() * (len(p) - 8) / nb
    enbw = n * np.sum(w ** 2) / np.sum(w) ** 2                     # equivalent noise BW in bins
    floor_bin = np.median(pdb[noise_bins])                         # dBFS per bin
    floor_dbm_hz = FS_DBM + floor_bin - 10 * np.log10(enbw * FS_HZ / n) - 3.0
    # SNR: also exclude the tone's harmonics (aliased) - a square-wave source
    # (NanoVNA, Si5351) has ~19% of its power in odd harmonics
    nh = noise_bins.copy()
    for h in range(2, 64):
        kh = (h * k) % n
        kh = n - kh if kh > n // 2 else kh
        nh[max(kh - span, 0):kh + span + 1] = False
    noise_h = p[nh].sum() * (len(p) - 8) / max(nh.sum(), 1)
    res["snr"] = 10 * np.log10(sig / noise_h) if noise_h > 0 else 0
    res.update(f_tone=k * FS_HZ / n, tone_dbfs=10 * np.log10(sig / p_fs),
               sinad=10 * np.log10(sig / noise) if noise > 0 else 0,
               floor_dbfs_bin=floor_bin, floor_dbm_hz=floor_dbm_hz)
    res["enob"] = (res["sinad"] - 1.76) / 6.02
    if verbose:
        print(f"  {n} samples: mean {res['mean']:.1f}  rms {res['rms']:.2f} LSB  "
              f"min {res['min']}  max {res['max']}  OTR {res['otr']}  glitches {res['glitches']}")
        print(f"  strongest tone {res['f_tone']/1e6:.4f} MHz  {res['tone_dbfs']:.1f} dBFS "
              f"(~{FS_DBM + res['tone_dbfs']:.1f} dBm at the input)")
        print(f"  SNR {res['snr']:.1f} dB (harmonics excluded)  SINAD {res['sinad']:.1f} dB  ENOB {res['enob']:.2f}  "
              f"noise floor {floor_bin:.1f} dBFS/bin = {floor_dbm_hz:.1f} dBm/Hz")
        top = np.argsort(pdb * noise_bins)[-6:][::-1]
        print("  largest spurs: " + ", ".join(f"{i*FS_HZ/n/1e6:.3f} MHz {pdb[i]:.0f} dBFS" for i in top))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["info", "cap", "sweep", "phase"])
    ap.add_argument("value", nargs="?", type=int)
    ap.add_argument("-p", "--phase", type=int)
    ap.add_argument("-o", "--out")
    ap.add_argument("--baud", type=int, default=BAUD)
    a = ap.parse_args()
    link = Link(a.baud)
    link.sync()
    try:
        if a.cmd == "info":
            print(info(link))
        elif a.cmd == "phase":
            set_phase(link, a.value)
            print(f"phase {a.value}")
        elif a.cmd == "cap":
            if a.phase is not None:
                set_phase(link, a.phase)
            t = time.time()
            code, otr, ph = capture(link)
            print(f"capture: phase {ph}, {time.time() - t:.2f} s")
            analyse(code, otr)
            if a.out:
                np.save(a.out, code | (otr.astype(np.int32) << 15))
                print(f"saved {a.out}")
        elif a.cmd == "sweep":
            print("phase  glitches  rms(LSB)  SINAD  floor dBFS/bin")
            best = None
            for ph in range(16):
                set_phase(link, ph)
                time.sleep(0.01)
                code, otr, _ = capture(link)
                r = analyse(code, otr, verbose=False)
                print(f"  {ph:2d}   {r['glitches']:6d}   {r['rms']:8.2f}  {r['sinad']:5.1f}  {r['floor_dbfs_bin']:6.1f}")
                score = (r["glitches"], r["floor_dbfs_bin"])
                if best is None or score < best[0]:
                    best = (score, ph)
            print(f"best phase: {best[1]}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
