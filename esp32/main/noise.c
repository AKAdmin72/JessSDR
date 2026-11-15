// Noise test: is the receiver (ADC + front end) or the band the limit?
//
// Measure once with a 50 ohm load on the antenna input (the BPF stays in
// the chain), once with the antenna; the difference is how far the band
// noise sits above the receiver's own noise. 10 dB or more = the band
// limits, a better ADC would not hear anything new (PLAN.md / 2026-11-15).
//
// Method: Hann-windowed FFT of the wide I/Q (105.54 kHz, fixed DDC gain,
// no AGC on this path), power per bin averaged over NOISE_FRAMES blocks,
// then the 10th percentile over the central +-26 kHz (DC +-2 bins skipped)
// so carriers and SSB stations in the band don't lift the result. The CIC
// droop near the edges is the same in both measurements, so it cancels in
// the difference.
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "dsps_fft2r.h"
#include "spectrum.h"
#include "fpga.h"
#include "noise.h"

#define N            SPEC_FFT_N
#define NOISE_FRAMES 200     // ~2s at 103 blocks/s; spread of the result ~0.1 dB
#define SKIP         5
#define PCT          0.1f    // low percentile: works while stations fill < ~80% of the bins

static float buf[2 * N] __attribute__((aligned(16)));
static float acc[N], tmp[N], win[N];
static float wsum2;          // sum of w^2 (noise gain of the window)
static int nfr, skip;

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

void noise_reset(void)
{
    if (wsum2 == 0) {        // dsps_fft2r_init_fc32 was done by spectrum_init
        for (int n = 0; n < N; n++) {
            win[n] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n / N);
            wsum2 += win[n] * win[n];
        }
    }
    memset(acc, 0, sizeof(acc));
    nfr = 0;
    skip = SKIP;
}

bool noise_add(const float *iq, float *db)
{
    if (skip > 0) { skip--; return false; }
    for (int n = 0; n < N; n++) {
        buf[2 * n] = iq[2 * n] * win[n];
        buf[2 * n + 1] = iq[2 * n + 1] * win[n];
    }
    dsps_fft2r_fc32(buf, N);
    dsps_bit_rev_fc32(buf, N);
    for (int k = 0; k < N; k++) acc[k] += buf[2 * k] * buf[2 * k] + buf[2 * k + 1] * buf[2 * k + 1];
    if (++nfr < NOISE_FRAMES) return false;

    int m = 0;
    for (int k = 0; k < N; k++) {
        int f = k < N / 2 ? k : N - k;              // |bin| from the LO
        if (f < 3 || f > N / 4) continue;
        tmp[m++] = acc[k] / nfr;
    }
    qsort(tmp, m, sizeof(float), cmp_f);
    // E|X|^2 = sigma^2 * sum(w^2), density = sigma^2 / fs
    float p = tmp[(int)(m * PCT)] / (wsum2 * FPGA_FS_HZ);
    *db = 10.0f * log10f(p + 1e-30f);
    memset(acc, 0, sizeof(acc));
    nfr = 0;
    return true;
}
