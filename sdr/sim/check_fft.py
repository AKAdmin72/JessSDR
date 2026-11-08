import numpy as np
x = np.loadtxt("sim/fft_in.txt"); y = np.loadtxt("sim/fft_out.txt")
z = x[:, 0] + 1j * x[:, 1]
w = np.array([int(l, 16) for l in open("fft_win.hex")]) / 32768
ref = np.fft.fft(np.floor(z.real * w * 32768 / 32768) + 1j * np.floor(z.imag * w))
got = y[:, 0] + 1j * y[:, 1]
err = np.abs(got - ref)
print("peak bins:", np.argsort(np.abs(got))[-3:], "max |X| %.0f" % np.abs(got).max())
print("max abs err %.1f, rms err %.2f, rel to peak %.1f dB" % (err.max(), np.sqrt(np.mean(err**2)),
      20 * np.log10(err.max() / np.abs(ref).max())))
print("bin -100 (412): got %.0f ref %.0f" % (abs(got[412]), abs(ref[412])))
fl = np.median(np.abs(got)); print("median bin |X| (noise) %.1f" % fl)
assert err.max() < np.abs(ref).max() * 10 ** (-80 / 20), "FFT mismatch"
print("ok")
