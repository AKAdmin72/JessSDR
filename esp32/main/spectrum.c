// Main screen: header, ruler, spectrum and waterfall (PLAN.md Phase 3),
// ported from ../tang_nano_9k_projects/FpgaRX-SDR to the 480x320 ST7796S.
//
//   y   0..47   header: dial frequency, mode/step, ADC/speed readout
//   y  48..65   frequency ruler (absolute kHz)
//   y  66..319  panel, by the view (menu: Display > View):
//                 SPECTRUM (default) = spectrum 254 px,
//                 WATERFALL          = waterfall 254 px (dashed red dial line),
//                 BOTH               = spectrum 160/140/120 px (menu: View,
//                                      default 160) + waterfall 94/114/134 px
//   spectrum = heatmap-filled, grid, passband, red dial mark
// Input is either the real I/Q block from the FPGA (I2S, the wide DDC
// output, 105.54kHz complex, centred on the dial) or a synthetic stream
// like it: a few LSB "voices", CW carriers and noise. FFT 1024, window
// OFF by default (the user prefers stations as broad shapes; menu Window
// HANN / BH). Only the middle +-35kHz of the +-52.7kHz are shown: further
// out the wide CIC lets aliases through (-27dBc at 33kHz, -10dBc at 45kHz,
// 0dBc at the edge - ddc.v) and droops up to 16dB. The header shows the
// spectrum rate and where the time goes.
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "dsps_fft2r.h"
#include "lcd.h"
#include "spectrum.h"
#include "fpga.h"

static const char *TAG = "spectrum";

// ---- layout ----
#define HDR_H      48
#define RULER_Y    48
#define RULER_H    18
#define PANEL_Y    66
#define PANEL_H    (LCD_H - PANEL_Y)    // 254: spectrum + waterfall share it
#define SPEC_Y     PANEL_Y
#define WF_MAX     PANEL_H              // waterfall ring rows (the tallest view)

static int spec_h_px = PANEL_H;         // spectrum panel height, 0 = hidden
static int spec_h_both = 160;           // spectrum height in BOTH (spectrum_set_layout)
static int wf_y = LCD_H, wf_h = 0;      // waterfall panel, wf_h 0 = hidden

// ---- signal ----
#define FS         FPGA_FS_HZ           // wide DDC output, 64.8MHz / 614
#define FFT_N      1024
#define BIN_HZ     (FS / FFT_N)            // 103.06Hz
#define SPAN_HZ    70000.0f                // shown: +-35kHz around the LO
#define HZ_PER_PX  (SPAN_HZ / LCD_W)       // 145.8Hz
#define BINS_PX    (HZ_PER_PX / BIN_HZ)    // 1.415 bins per pixel

static uint32_t dial_hz = 7100000;
static float lo_shift_hz = 0;           // spectrum centre (the LO) - dial: CW -pitch, else 0
static int step_idx = 3;                // 1 = 10Hz ... 6 = 1MHz (tune_ctrl.v numbering)
static char mode_label[4] = "LSB";     // header, set by main.c (LSB/USB/AM/CW)
static char bw_label[6] = "";           // filter width in the header ("2.4k")
static bool demod_auto = true;          // chosen by the band plan
static float pb_lo = -2700, pb_hi = -300;   // passband shading, Hz relative to the dial

#define C_BG       RGB565(0, 0, 0)
#define C_HDR_BG   RGB565(0, 0, 40)
#define C_TEXT     RGB565(255, 255, 255)
#define C_DIM      RGB565(150, 150, 170)
#define C_GRID     RGB565(45, 45, 45)
#define C_PASS     RGB565(30, 30, 70)
#define C_DIAL     RGB565(255, 0, 0)
#define C_RULER_BG RGB565(20, 20, 20)

static uint16_t heat_pal[256];

// PicoRX heatmap (same as stage1/heatmap.v): black-blue-cyan-green-yellow-red-white
static void make_palette(void)
{
    for (int v = 0; v < 256; v++) {
        int v6 = v * 6, sec = v6 >> 8, f = v6 & 255, r, g, b;
        switch (sec) {
        case 0:  r = 0;   g = 0;       b = f;       break;
        case 1:  r = 0;   g = f;       b = 255;     break;
        case 2:  r = 0;   g = 255;     b = 255 - f; break;
        case 3:  r = f;   g = 255;     b = 0;       break;
        case 4:  r = 255; g = 255 - f; b = 0;       break;
        default: r = 255; g = f;       b = f;       break;
        }
        heat_pal[v] = RGB565(r, g, b);
    }
}

static int freq_to_x(float hz)  // absolute Hz -> screen x (center = LO)
{
    float center = dial_hz + lo_shift_hz;
    return LCD_W / 2 + (int)lrintf((hz - center) / HZ_PER_PX);
}

// ---------------------------------------------------------------------------
// Synthetic I/Q
// ---------------------------------------------------------------------------
typedef struct { float re, im, cr, ci, amp, target; } osc_t;

#define VOICE_TONES 6
typedef struct { float f0; float level; osc_t t[VOICE_TONES]; } voice_t;   // f0 = suppressed carrier, offset from LO
typedef struct { float f; float amp; int key_period, key_on; osc_t o; } cw_t;

static voice_t voices[] = {
    { .f0 = 0,          .level = 0.006f },   // the station on the dial
    { .f0 = -16000.0f,  .level = 0.010f },
    { .f0 = +9500.0f,   .level = 0.003f },
    { .f0 = +23000.0f,  .level = 0.0015f },
    { .f0 = -38000.0f,  .level = 0.004f },
};
static cw_t cws[] = {
    { .f = +18000.0f, .amp = 0.02f,   .key_period = 9 },
    { .f = -10500.0f, .amp = 0.0006f, .key_period = 0 },   // weak steady carrier
    { .f = +41000.0f, .amp = 0.003f,  .key_period = 5 },
};
static uint32_t rng = 12345;
static int frame_no;

static inline float frand(void)  // 0..1
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return (rng >> 8) * (1.0f / 16777216.0f);
}

static void osc_set_freq(osc_t *o, float hz)
{
    float w = 2.0f * (float)M_PI * hz / FS;
    o->cr = cosf(w);
    o->ci = sinf(w);
    if (o->re == 0 && o->im == 0) o->re = 1;
}

static inline void osc_step(osc_t *o, float *i, float *q)
{
    *i += o->re * o->amp;
    *q += o->im * o->amp;
    float re = o->re * o->cr - o->im * o->ci;
    o->im = o->re * o->ci + o->im * o->cr;
    o->re = re;
}

static void osc_renorm(osc_t *o)
{
    float m = 1.0f / sqrtf(o->re * o->re + o->im * o->im);
    o->re *= m;
    o->im *= m;
}

static void siggen_init(void)
{
    for (int v = 0; v < sizeof(voices) / sizeof(voices[0]); v++)
        for (int k = 0; k < VOICE_TONES; k++)
            osc_set_freq(&voices[v].t[k], voices[v].f0 - 300 - 400 * k);
    for (int c = 0; c < sizeof(cws) / sizeof(cws[0]); c++) {
        osc_set_freq(&cws[c].o, cws[c].f);
        cws[c].o.amp = cws[c].amp;
    }
}

// Once per block: move the voice "formants" around and key the CW.
static void siggen_update(void)
{
    frame_no++;
    for (int v = 0; v < sizeof(voices) / sizeof(voices[0]); v++) {
        voice_t *vc = &voices[v];
        bool talking = ((frame_no / 40 + v) % 4) != 0;   // pauses between "sentences"
        for (int k = 0; k < VOICE_TONES; k++) {
            osc_t *o = &vc->t[k];
            if (frand() < 0.15f) {
                o->target = talking ? vc->level * frand() * (k < 3 ? 1.0f : 0.5f) : 0;
                osc_set_freq(o, vc->f0 - 300 - 2400 * frand());
            }
            o->amp += (o->target - o->amp) * 0.3f;
            osc_renorm(o);
        }
    }
    for (int c = 0; c < sizeof(cws) / sizeof(cws[0]); c++) {
        cw_t *cw = &cws[c];
        if (cw->key_period) cw->o.amp = (((frame_no / cw->key_period) * 7) % 5 < 3) ? cw->amp : 0;
        osc_renorm(&cw->o);
    }
}

// Fills buf with FFT_N complex samples (interleaved re, im).
static void siggen_block(float *buf)
{
    const float noise = 0.002f;
    siggen_update();
    for (int n = 0; n < FFT_N; n++) {
        float i = (frand() + frand() - 1.0f) * noise;
        float q = (frand() + frand() - 1.0f) * noise;
        for (int v = 0; v < sizeof(voices) / sizeof(voices[0]); v++)
            for (int k = 0; k < VOICE_TONES; k++)
                osc_step(&voices[v].t[k], &i, &q);
        for (int c = 0; c < sizeof(cws) / sizeof(cws[0]); c++)
            osc_step(&cws[c].o, &i, &q);
        buf[2 * n] = i;
        buf[2 * n + 1] = q;
    }
}

// ---------------------------------------------------------------------------
// Spectrum processing
// ---------------------------------------------------------------------------
static float fft_buf[2 * FFT_N] __attribute__((aligned(16)));
static float cap_acc[FFT_N];   // debug capture: power per bin, fftshifted
// S-meter calibration (console "sm"): over N frames, mean passband power
// (linear, not peak-held), min/max per frame, noise in the same passband
static int sm_n, sm_left;
static double sm_p, sm_nz;
static float sm_min, sm_max;
static int cap_left, cap_n;
static float px_db[LCD_W];     // this frame, per pixel
static float px_avg[LCD_W];    // averaged over frames (stage1/level_averager.v)
static float px_wf[LCD_W];     // lightly averaged, for the waterfall
static float est[LCD_W];       // per-pixel floor estimate (stage1/spectrum_eq.v)
static float env[LCD_W];       // lower envelope of est across frequency
static int warm;               // frames since start / floor reset (stops counting at EQ_WARM)
static uint8_t spec_h[LCD_W];  // spectrum column height 0..spec_h_px
static uint8_t spec_lv[LCD_W]; // spectrum column colour level
static uint8_t line_lo[LCD_W], line_hi[LCD_W];   // white trace span per column (0 = none)
static float rel_db[LCD_W];    // dB above the floor, per pixel
static bool view_smooth = true; // [1 2 1]/4 across frequency on the spectrum (menu: Smooth)
static uint8_t (*wf)[LCD_W];   // WF_MAX rows, PSRAM; always filled, the view shows the newest wf_h
static int wf_head;            // index of the newest waterfall row
// Retune without smear: the per-pixel state (px_avg, px_wf, est) belongs to
// the spectrum centre st_center (within half a pixel of the real one). When
// the dial moves, only the level ABOVE the floor (px_avg - env, px_wf -
// env) is shifted by whole pixels; the floor itself (est/env) stays where
// it is on the screen - its shape is the CIC droop and the window, tied to
// the screen position, not to the frequency (shifting it made hills: a
// column from the drooping edge carried its low floor inwards). Columns
// that come in at the edge start at the floor and fade in with the normal
// averaging. A jump of half a screen or more restarts the floor like at
// power-up (another stretch of band, maybe another noise level).
// Each waterfall row keeps the centre it was made at and is drawn shifted,
// so the history stays put; where a shifted row has no data the edge is
// filled with that row's own noise (wf_cap), not black.
// Menu Display > Retune: SCROLL (this) or STATIC (the old way: no shift,
// the averages forget the old picture).
static double st_center;       // Hz, centre the state arrays are aligned to
static bool st_valid;          // st_center set (first frame seen)
static uint8_t wf_cap[WF_MAX]; // per row: ~p75 of its levels (noise, no stations) for edge fill
static bool scroll_on = true;  // Retune: SCROLL / STATIC
static int32_t wf_ctr[WF_MAX]; // Hz, centre each waterfall row was made at
// Waterfall speed (menu Display > WF speed): one row per wf_div frames, the
// row = mean of their levels over the floor (not a skipped frame: weak
// signals add up). Partial sum in screen coordinates, shifted on retune.
static int wf_div = 1, wf_n;
static float wf_acc[LCD_W];
static bool src_sim;           // synthetic input: stations move with the LO, so don't shift

// Squelch measurement (main.c decides): SNR in the demodulator's passband =
// mean FFT bin power inside pb_lo..pb_hi over the mean noise power per bin.
// Noise per bin = 10th percentile of the bins within +-SQ_NOISE_K (about
// +-26kHz, same as noise.c) / 0.10536 (the p10 of an exponential = 0.10536
// x its mean), so it holds while up to ~half the band is busy (then it
// reads high = the squelch opens harder, never easier).
#define SQ_NOISE_K   256
#define SQ_SIG_A     0.5f              // per frame (~30/s): ~60ms attack
#define SQ_NOISE_A   0.05f             // ~0.7s
static float sq_sig = -1, sq_noise = -1;   // linear, -1 = restart
static float sq_snr_db;
static bool sq_fresh;
static float sq_tmp[2 * SQ_NOISE_K + 1];
static float view_range = 40;  // dB over the spectrum panel (menu: Range)
static int view_win;           // window before the FFT: SPEC_WIN_* (menu: Window)
static float win_tab[SPEC_WIN_N][FFT_N];
static float win_pow[SPEC_WIN_N];   // sum of w^2 (Parseval: power of the signal)
// pixel -> FFT bins (fractional overlap, 1.415 bins per pixel)
static int16_t px_k0[LCD_W];        // first bin offset from the LO
static uint8_t px_nk[LCD_W];        // bins touched (<= 3)
static float px_w[LCD_W][3];        // overlap of each, sum = BINS_PX
// S-meter (dBm) from the FFT power inside the demodulator passband
// (was the FPGA's level_meter on the +-6kHz decimator stream: a strong
// neighbour 4kHz away showed S9 while the filter had nothing).
static float pb_dbm = -140;

// Same numbers as the Tang build (where 1 level = 0.4dB, 206Hz per bin,
// 206 FFT frames/s), converted to dB, pixels and seconds:
#define AVG_A      0.125f                 // level_averager AVG_SHIFT=3
#define WF_A       0.5f
#define EQ_RISE    1.6f                   // dB/s, est creep up (RISE=5)
// Start-up: the first EQ_SKIP blocks are only used to prime the averages
// (the first block after the link self-test can be stale or near silent),
// then for EQ_WARM frames (~1s) est follows px_avg both ways. Before, est
// came from one noisy frame, sat 10-15dB under the real floor and took
// ~10s at EQ_RISE to climb: the spectrum started high and sank to 0.
#define EQ_SKIP    4
#define EQ_WARM    36
// Envelope slope: was 1.94dB/kHz (Tang SLOPE=1 level/bin); a station
// lifts the floor under it by slope x half its width, so an AM station
// (9kHz) lost ~8.7dB, SSB ~2.6dB. 1dB/kHz halves that and still follows
// the CIC droop (<= 0.35dB/kHz within +-35kHz).
#define EQ_SLOPE   (1.0f * HZ_PER_PX / 1000.0f)  // dB per pixel
// FpgaRX had a 1/f + LO-leakage hump around the LO and a steeper envelope
// there (DC_W/DC_SLOPE). The direct-sampling DDC has none, and the dial is
// at the centre now, so that zone would only hide the station listened to.
static float slope_at(int x) { return EQ_SLOPE; }

static double cur_center(void) { return (double)dial_hz + lo_shift_hz; }

// Shift the state to the current centre (whole pixels). Tuning up moves the
// stations left: new[x] = old[x + n].
static void state_follow(void)
{
    double c = cur_center();
    if (!st_valid || src_sim || !scroll_on) { st_center = c; st_valid = true; return; }
    int n = (int)lrint((c - st_center) / HZ_PER_PX);
    if (n == 0) return;
    st_center += n * (double)HZ_PER_PX;
    if (abs(n) >= LCD_W / 2) warm = 0;        // mostly new band: floor from scratch
    // level above the floor, shifted; new columns = 0 dB (at the floor)
    static float r_avg[LCD_W], r_wf[LCD_W], r_acc[LCD_W];
    for (int x = 0; x < LCD_W; x++) {
        int xs = x + n;
        bool in = xs >= 0 && xs < LCD_W;
        r_avg[x] = in ? px_avg[xs] - env[xs] : 0;
        r_wf[x]  = in ? px_wf[xs] - env[xs] : 0;
        r_acc[x] = in ? wf_acc[xs] : 0;
    }
    for (int x = 0; x < LCD_W; x++) {
        px_avg[x] = env[x] + r_avg[x];
        px_wf[x]  = env[x] + r_wf[x];
        wf_acc[x] = r_acc[x];
        if (est[x] > px_avg[x]) est[x] = px_avg[x];   // as if the frame had come in
    }
}

static void spectrum_render(bool new_row);

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

static inline float bin_pow(int k)   // k = bin offset from the LO, signed
{
    int i = k & (FFT_N - 1);
    return fft_buf[2 * i] * fft_buf[2 * i] + fft_buf[2 * i + 1] * fft_buf[2 * i + 1];
}

static void sq_measure(void)
{
    const float bin_hz = FS / FFT_N;
    int k0 = (int)ceilf((pb_lo - lo_shift_hz) / bin_hz), k1 = (int)floorf((pb_hi - lo_shift_hz) / bin_hz);
    if (k1 < k0) k0 = k1 = (int)lrintf((pb_lo + pb_hi) * 0.5f / bin_hz - lo_shift_hz / bin_hz);
    float sig = 0;
    for (int k = k0; k <= k1; k++) sig += bin_pow(k);
    // S-meter: Parseval - signal power = sum |X|^2 / (N sum w^2), in units
    // of the I/Q floats (18-bit sample / 2^17) -> LSB^2 -> dBm. Peak-reading
    // like the old level_meter: up at once, down 20dB/s.
    {
        static int64_t t_prev;
        int64_t now = esp_timer_get_time();
        float dt = t_prev ? (now - t_prev) * 1e-6f : 0;
        t_prev = now;
        float p_lsb = sig / (FFT_N * win_pow[view_win]) * 17179869184.0f;   // x 2^34
        float dbm = FPGA_DBM_1LSB + 10.0f * log10f(p_lsb + 1e-12f);
        pb_dbm = dbm > pb_dbm - 20.0f * dt ? dbm : pb_dbm - 20.0f * dt;
        if (sm_left > 0) {
            sm_p += p_lsb;
            if (dbm < sm_min) sm_min = dbm;
            if (dbm > sm_max) sm_max = dbm;
        }
    }
    int nb = k1 - k0 + 1;
    sig /= nb;
    int n = 0;
    for (int k = -SQ_NOISE_K; k <= SQ_NOISE_K; k++) sq_tmp[n++] = bin_pow(k);
    qsort(sq_tmp, n, sizeof(float), cmp_float);
    float noise = sq_tmp[n / 10] / 0.10536f + 1e-30f;
    if (sm_left > 0) {   // noise power the passband would hold, same units as sm_p
        sm_nz += noise * nb / (FFT_N * win_pow[view_win]) * 17179869184.0f;
        sm_left--;
    }
    sq_sig = sq_sig < 0 ? sig : sq_sig + (sig - sq_sig) * SQ_SIG_A;
    sq_noise = sq_noise < 0 ? noise : sq_noise + (noise - sq_noise) * SQ_NOISE_A;
    sq_snr_db = 10.0f * log10f(sq_sig / sq_noise + 1e-12f);
    sq_fresh = true;
}

static void spectrum_process(void)
{
    static int64_t t_prev;
    int64_t now = esp_timer_get_time();
    float dt = t_prev ? (now - t_prev) * 1e-6f : 0;
    t_prev = now;
    if (dt > 0.2f) dt = 0.2f;

    if (view_win != SPEC_WIN_OFF) {
        const float *w = win_tab[view_win];
        for (int n = 0; n < FFT_N; n++) { fft_buf[2 * n] *= w[n]; fft_buf[2 * n + 1] *= w[n]; }
    }
    dsps_fft2r_fc32(fft_buf, FFT_N);
    dsps_bit_rev_fc32(fft_buf, FFT_N);
    sq_measure();

    // bins -> pixels (the middle SPAN_HZ), mean power per pixel weighted by
    // how much of each bin the pixel covers (less noise variance than the max)
    for (int x = 0; x < LCD_W; x++) {
        float m = 0;
        for (int j = 0; j < px_nk[x]; j++) m += px_w[x][j] * bin_pow(px_k0[x] + j);
        px_db[x] = 10.0f * log10f(m / BINS_PX + 1e-20f);
    }

    // debug capture (console "s"): mean power per FFT bin over N frames
    if (cap_left > 0) {
        for (int b = 0; b < FFT_N; b++) {
            int k = (b + FFT_N / 2) & (FFT_N - 1);
            cap_acc[b] += fft_buf[2 * k] * fft_buf[2 * k] + fft_buf[2 * k + 1] * fft_buf[2 * k + 1];
        }
        cap_left--;
    }

    // 1. averaging between frames, in dB like level_averager.v
    // 2. per-pixel floor: snaps down at once, creeps up at EQ_RISE
    state_follow();
    // start-up / floor reset: EQ_SKIP frames prime the averages, then a
    // running mean (until 1/AVG_A frames) and est = px_avg both ways
    for (int x = 0; x < LCD_W; x++) {
        if (warm < EQ_SKIP) {
            px_avg[x] = px_wf[x] = est[x] = px_db[x];
            continue;
        }
        int n = warm - EQ_SKIP + 1;
        float a = n < (int)(1.0f / AVG_A) ? 1.0f / (n + 1) : AVG_A;
        px_avg[x] += (px_db[x] - px_avg[x]) * a;
        px_wf[x]  += (px_db[x] - px_wf[x]) * WF_A;
        if (warm < EQ_WARM) est[x] = px_avg[x];
        else est[x] = px_avg[x] < est[x] ? px_avg[x] : est[x] + EQ_RISE * dt;
    }
    if (warm < EQ_WARM) warm++;
    spectrum_render(true);
}

// Levels -> panel heights/colours (+ a waterfall row if new_row). Also used
// without a new frame while the I/Q is stale after a retune.
static void spectrum_render(bool new_row)
{
    // 3. lower envelope across frequency: min over k of est[x+k] + |k|*SLOPE.
    // Follows the wide center hump and the edge roll-off, but a station
    // cannot lift it by more than SLOPE per pixel of its half-width.
    env[0] = est[0];
    for (int x = 1; x < LCD_W; x++) env[x] = fminf(est[x], env[x - 1] + slope_at(x));
    for (int x = LCD_W - 2; x >= 0; x--) env[x] = fminf(env[x], env[x + 1] + slope_at(x));

    // 4. output = level - envelope: flat noise along the bottom. view_range
    // dB fill the panel (was a fixed 60dB, waterfall 45dB: a 3-6dB station
    // was only 5-10px high and nearly black in the waterfall).
    if (new_row) {   // add this frame to the row sum; a row every wf_div frames
        for (int x = 0; x < LCD_W; x++) wf_acc[x] += px_wf[x] - env[x];
        new_row = ++wf_n >= wf_div;
    }
    float k_acc = 1.0f;
    if (new_row) {
        k_acc = 1.0f / wf_n;
        wf_head = (wf_head + WF_MAX - 1) % WF_MAX;
        wf_ctr[wf_head] = (int32_t)lrint(st_center);
    }
    float k_h = spec_h_px / view_range, k_lv = 255.0f / (view_range * 0.92f), k_wf = 255.0f / (view_range * 0.75f);
    for (int x = 0; x < LCD_W; x++) rel_db[x] = px_avg[x] - env[x];   // dB above the floor
    // Smooth ([1 2 1]/4 in dB): less noise "grass". A lone 1-px peak (CW)
    // loses about half its height and gets 3 px wide; a voice station hardly changes.
    float prev = rel_db[0];
    if (view_smooth)
        for (int x = 1; x < LCD_W - 1; x++) {
            float cur = rel_db[x];
            rel_db[x] = (prev + 2.0f * cur + rel_db[x + 1]) * 0.25f;
            prev = cur;
        }
    for (int x = 0; x < LCD_W; x++) {
        float rel = rel_db[x];
        int h = (int)((rel + 1.0f) * k_h);
        spec_h[x] = h < 0 ? 0 : h > spec_h_px ? spec_h_px : h;
        int lv = (int)((rel + 1.0f) * k_lv);
        spec_lv[x] = lv < 0 ? 0 : lv > 255 ? 255 : lv;
        if (new_row) {
            int w = (int)((wf_acc[x] * k_acc + 1.0f) * k_wf);
            wf[wf_head][x] = w < 0 ? 0 : w > 255 ? 255 : w;
            wf_acc[x] = 0;
        }
    }
    if (new_row) {   // edge fill level: ~75th percentile of the row (noise, stations are above)
        wf_n = 0;
        uint16_t hist[256] = { 0 };
        for (int x = 0; x < LCD_W; x++) hist[wf[wf_head][x]]++;
        int acc = 0, v = 0;
        while (v < 255 && (acc += hist[v]) < LCD_W * 3 / 4) v++;
        wf_cap[wf_head] = v;
    }
    // Trace: column x is white from the midpoint towards its left neighbour's
    // top to the midpoint towards its right one (and its own top), so the tops
    // join into one unbroken line instead of loose dots.
    for (int x = 0; x < LCD_W; x++) {
        int h = spec_h[x];
        int hl = x > 0 ? (h + spec_h[x - 1] + 1) / 2 : h;
        int hr = x < LCD_W - 1 ? (h + spec_h[x + 1] + 1) / 2 : h;
        int lo = h, hi = h;
        if (hl < lo) lo = hl;
        if (hr < lo) lo = hr;
        if (hl > hi) hi = hl;
        if (hr > hi) hi = hr;
        if (lo < 1) lo = 1;
        line_lo[x] = hi ? lo : 0;
        line_hi[x] = hi;
    }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
static uint8_t col_flags[LCD_W];   // bit0 = grid line, bit1 = passband, bit2 = dial
#define F_GRID 1
#define F_PASS 2
#define F_DIAL 4

// Text into an off-screen buffer (stride = buffer width), transparent background.
static void buf_text(uint16_t *b, int stride, int x, int y, const font_t *f, const char *s, uint16_t fg)
{
    int bpr = (f->w + 7) / 8;
    for (; *s; s++, x += f->w) {
        const uint8_t *g = f->bits + (*s - 32) * f->h * bpr;
        for (int yy = 0; yy < f->h; yy++)
            for (int xx = 0; xx < f->w; xx++)
                if (x + xx >= 0 && x + xx < stride && (g[yy * bpr + xx / 8] & (0x80 >> (xx & 7))))
                    b[(y + yy) * stride + x + xx] = fg;
    }
}

// Ruler + column flags; redrawn when the dial changes.
static void draw_ruler(void)
{
    float center = dial_hz + lo_shift_hz;
    float lo = center - LCD_W / 2 * HZ_PER_PX, hi = center + LCD_W / 2 * HZ_PER_PX;

    memset(col_flags, 0, sizeof(col_flags));
    uint16_t *b = lcd_buf_get();
    for (int i = 0; i < LCD_W * RULER_H; i++) b[i] = C_RULER_BG;
    for (long f = ((long)lo / 1000 + 1) * 1000; f < hi; f += 1000) {
        int x = freq_to_x(f);
        if (x < 0 || x >= LCD_W) continue;
        int len = (f % 10000 == 0) ? 3 : (f % 5000 == 0) ? 2 : 1;
        for (int y = RULER_H - len; y < RULER_H; y++) b[y * LCD_W + x] = C_DIM;
        if (f % 5000 == 0) col_flags[x] |= F_GRID;
        if (f % 10000 == 0) {
            char s[16];
            snprintf(s, sizeof(s), "%ld", f / 1000);
            buf_text(b, LCD_W, x - (int)strlen(s) * font_small.w / 2, 0, &font_small, s, C_TEXT);
        }
    }
    lcd_buf_send(0, RULER_Y, LCD_W, RULER_H);

    // passband (what the demodulator hears) and the dial marker
    float p0 = pb_lo, p1 = pb_hi;
    for (int x = freq_to_x(dial_hz + p0); x <= freq_to_x(dial_hz + p1); x++)
        if (x >= 0 && x < LCD_W) col_flags[x] |= F_PASS;
    int xd = freq_to_x(dial_hz);
    if (xd >= 0 && xd < LCD_W) col_flags[xd] |= F_DIAL;
}

#define FREQ_X     6
#define MODE_X     236                  // right of the 10 big digits (6 + 210)
#define STEP_X     (MODE_X + 50)
#define ROW2_Y     26                   // second header row (font_mid)

// Dial readout, always 10 characters (leading blank below 10MHz), with the
// digit that the encoder moves underlined.
static void draw_freq(void)
{
    static const int pos_from_right[] = { 0, 1, 2, 4, 5, 6, 8 };   // per step_idx, skipping the dots
    static const char *step_name[] = { "", "10", "100", "1k", "10k", "100k", "1M" };
    char s[16];
    snprintf(s, sizeof(s), "%2lu.%03lu.%03lu", dial_hz / 1000000, dial_hz / 1000 % 1000, dial_hz % 1000);
    lcd_text(FREQ_X, 3, &font_big, s, C_TEXT, C_HDR_BG);
    lcd_fill(FREQ_X, 41, 10 * font_big.w, 3, C_HDR_BG);
    lcd_fill(FREQ_X + (9 - pos_from_right[step_idx]) * font_big.w + 1, 41, font_big.w - 2, 3, RGB565(255, 220, 0));
    snprintf(s, sizeof(s), "STEP %-4s", step_name[step_idx]);
    lcd_text(STEP_X, 4, &font_mid, s, C_DIM, C_HDR_BG);
}

// Mode: yellow = picked by the band plan (AUTO), cyan = set by hand
#define BW_X       (STEP_X + 100)       // filter width, right of "STEP 100k"
static void draw_mode(void)
{
    char s[8];
    snprintf(s, sizeof(s), "%-3s", mode_label);
    lcd_text(MODE_X, 4, &font_mid, s, demod_auto ? RGB565(255, 220, 0) : RGB565(0, 220, 255), C_HDR_BG);
    snprintf(s, sizeof(s), "%-5s", bw_label);
    lcd_text(BW_X, 4, &font_mid, s, RGB565(0, 220, 255), C_HDR_BG);
}

// ---- S-meter / volume slot: header row 2, right of the ADC/FPS slot ----
// "S9+20" + a bar: S0..S9 (6dB per S unit, S9 = -73dBm) on the first 80px
// (green), S9..S9+60 on the last 40px (red). While the encoder sets the
// volume the slot shows "VOL nn" and a bar instead.
#define SM_X       (MODE_X + 64)
#define SM_BAR_X   (SM_X + 60)
#define SM_BAR_W   120
static float sm_dbm = -200;
static int vol_shown = -1;              // >= 0: volume display instead of the S-meter
static int sql_shown = -1;              // >= 0: squelch threshold (0 = OFF) + live SNR bar
static bool sql_closed;                 // squelch shut: S-meter drawn dim
#define SQ_BAR_DB  30                   // squelch bar: 0..30dB SNR over SM_BAR_W
static int sm_drawn_px = -1, sm_drawn_kind = -1;
static char sm_drawn_txt[8];
static void draw_smeter(bool force)
{
    char t[16];
    int px, split;
    uint16_t c_lo, c_hi;
    int kind = vol_shown >= 0 ? 1 : sql_shown >= 0 ? 2 : sql_closed ? 3 : 0;
    if (kind == 2) {
        // "SQ 12" / "SQOFF", bar = SNR in the passband now, tick = threshold
        if (sql_shown) snprintf(t, sizeof(t), "SQ%3d", sql_shown);
        else snprintf(t, sizeof(t), "SQOFF");
        float v = sq_snr_db < 0 ? 0 : sq_snr_db;
        px = (int)(v * SM_BAR_W / SQ_BAR_DB);
        if (px > SM_BAR_W) px = SM_BAR_W;
        split = sql_shown ? sql_shown * SM_BAR_W / SQ_BAR_DB : SM_BAR_W;
        c_lo = RGB565(90, 90, 120); c_hi = RGB565(0, 220, 80);   // under / over the threshold
    } else if (kind == 1) {
        snprintf(t, sizeof(t), "VOL%2d", vol_shown);
        px = vol_shown * SM_BAR_W / 23;
        split = SM_BAR_W * 20 / 23;     // 0 dB mark
        c_lo = RGB565(0, 200, 255); c_hi = RGB565(255, 160, 0);
    } else {
        float s = (sm_dbm + 127.0f) / 6.0f;          // S units, S0 = -127dBm
        if (s < 0) s = 0;
        if (s <= 9.0f) { snprintf(t, sizeof(t), "S%d", (int)s); px = (int)(s * 80.0f / 9.0f); }
        else {
            float over = sm_dbm + 73.0f;
            snprintf(t, sizeof(t), "S9+%d", (int)(over / 10.0f) * 10);
            px = 80 + (int)(over * 40.0f / 60.0f);
        }
        if (px > SM_BAR_W) px = SM_BAR_W;
        split = 80;
        if (kind == 3) { c_lo = RGB565(70, 70, 90); c_hi = RGB565(110, 60, 60); }   // squelch shut
        else { c_lo = RGB565(0, 220, 80); c_hi = RGB565(255, 40, 40); }
    }
    char s[8];
    snprintf(s, sizeof(s), "%-5.5s", t);
    if (force || kind != sm_drawn_kind || strcmp(s, sm_drawn_txt) != 0) {
        strcpy(sm_drawn_txt, s);
        uint16_t fg = kind == 1 ? RGB565(0, 200, 255) : kind == 2 ? RGB565(255, 220, 0) : kind == 3 ? C_DIM : C_TEXT;
        lcd_text(SM_X, ROW2_Y, &font_mid, s, fg, C_HDR_BG);
    }
    if (force || px != sm_drawn_px || kind != sm_drawn_kind || kind == 2) {
        uint16_t *b = lcd_buf_get();
        for (int y = 0; y < 12; y++)
            for (int x = 0; x < SM_BAR_W; x++) {
                bool on = x < px && y >= 2 && y < 10;
                bool tick = (y >= 10) && ((kind == 1 || kind == 2) ? (x == split || x == split - 1) : (x % 9 == 0 && x <= 80) || (x > 80 && (x - 80) % 13 == 0));
                b[y * SM_BAR_W + x] = on ? (x < split ? c_lo : c_hi) : tick ? C_DIM : RGB565(10, 10, 60);
            }
        lcd_buf_send(SM_BAR_X, ROW2_Y + 4, SM_BAR_W, 12);
        sm_drawn_px = px;
    }
    sm_drawn_kind = kind;
}

void spectrum_set_smeter(float dbm)
{
    sm_dbm = dbm;
    draw_smeter(false);
}

void spectrum_show_volume(int v)
{
    vol_shown = v;
    draw_smeter(false);
}

void spectrum_show_squelch(int thr_db)
{
    sql_shown = thr_db;
    draw_smeter(false);
}

void spectrum_set_squelch_closed(bool closed)
{
    sql_closed = closed;
    draw_smeter(false);
}

static void draw_speed(float fps);
static bool show_fps;   // menu: FPS (off by default)
static float last_fps;
static struct { int clip; float pk_db; } adc_st = { 0, -100.0f };   // no peak until the FPGA reports one
static char speed_shown[7];   // what the slot holds now ("" = redraw)
static void draw_header(void)
{
    lcd_fill(0, 0, LCD_W, HDR_H, C_HDR_BG);
    speed_shown[0] = 0;
    draw_speed(last_fps);
    draw_mode();
    draw_freq();
    draw_smeter(true);
}

void spectrum_set_mode(const char *name, const char *bw, bool is_auto, float lo_hz, float hi_hz)
{
    bool moved = lo_hz != pb_lo || hi_hz != pb_hi;
    snprintf(mode_label, sizeof(mode_label), "%s", name);
    snprintf(bw_label, sizeof(bw_label), "%s", bw);
    demod_auto = is_auto;
    pb_lo = lo_hz;
    pb_hi = hi_hz;
    draw_mode();
    if (moved) draw_ruler();
}

void spectrum_set_view(int range_db, int window)
{
    view_range = range_db;
    view_win = window >= 0 && window < SPEC_WIN_N ? window : SPEC_WIN_OFF;
}

float spectrum_smeter_dbm(void)
{
    return pb_dbm;
}

void spectrum_set_smooth(bool on)
{
    view_smooth = on;
}

void spectrum_set_scroll(bool on)
{
    scroll_on = on;
}

void spectrum_floor_reset(void)
{
    warm = 0;
}

void spectrum_set_wf_speed(int div)
{
    wf_div = div < 1 ? 1 : div;
    wf_n = 0;
    memset(wf_acc, 0, sizeof(wf_acc));
}

void spectrum_set_layout(int view, int both_spec_h)
{
    if (both_spec_h >= 40 && both_spec_h <= PANEL_H - 40) spec_h_both = both_spec_h;
    switch (view) {
    case SPEC_VIEW_WATERFALL: spec_h_px = 0;           break;
    case SPEC_VIEW_BOTH:      spec_h_px = spec_h_both; break;
    default:                  spec_h_px = PANEL_H;     break;
    }
    wf_y = PANEL_Y + spec_h_px;
    wf_h = LCD_H - wf_y;
    memset(spec_h, 0, sizeof(spec_h));   // heights were scaled for the old panel
    memset(line_lo, 0, sizeof(line_lo));
    memset(line_hi, 0, sizeof(line_hi));
}

void spectrum_set_dial(uint32_t hz, int step, float lo_offset_hz)
{
    float shift = lo_offset_hz;
    bool moved = hz != dial_hz || shift != lo_shift_hz;
    dial_hz = hz;
    lo_shift_hz = shift;
    step_idx = step;
    if (moved) sq_sig = -1;      // the old station's level must not hold the squelch
    draw_freq();
    if (moved) draw_ruler();
}


// Header slot under the mode, 6 characters. ADC overload (adc_peak.v,
// adc_clip.v) takes priority over the frame rate: white "CLIP" on red =
// steady clipping within the last second, on orange = only rare clipped
// samples (single peaks); yellow "-3.2dB" = peak above -6dBFS
// (little headroom left); otherwise the FPS if switched on, or blank.
static void draw_speed(float fps)
{
    char t[16], s[7];
    uint16_t fg = RGB565(0, 255, 120), bg = C_HDR_BG;
    last_fps = fps;
    if (adc_st.clip) {
        snprintf(t, sizeof(t), " CLIP ");
        fg = C_TEXT; bg = adc_st.clip >= 2 ? RGB565(220, 0, 0) : RGB565(230, 120, 0);
    } else if (adc_st.pk_db > -6.0f) {
        snprintf(t, sizeof(t), "%4.1fdB", adc_st.pk_db);
        fg = RGB565(255, 220, 0);
    } else if (show_fps) snprintf(t, sizeof(t), "%2.0ffps", fps);
    else t[0] = 0;
    snprintf(s, sizeof(s), "%-6.6s", t);
    static uint16_t bg_shown;
    if (strcmp(s, speed_shown) == 0 && bg == bg_shown) return;   // orange <-> red: same text
    strcpy(speed_shown, s);
    bg_shown = bg;
    lcd_text(MODE_X, ROW2_Y, &font_mid, s, fg, bg);
}

void spectrum_show_fps(bool on)
{
    show_fps = on;
    draw_speed(0);   // clears it at once when switched off; the value follows within 1s
}

void spectrum_set_adc(int clip, float peak_dbfs)
{
    adc_st.clip = clip;
    adc_st.pk_db = peak_dbfs;
    draw_speed(last_fps);
}

static void draw_spectrum(void)
{
    for (int y0 = 0; y0 < spec_h_px; y0 += LCD_CHUNK) {
        int n = spec_h_px - y0 < LCD_CHUNK ? spec_h_px - y0 : LCD_CHUNK;
        uint16_t *b = lcd_buf_get();
        for (int l = 0; l < n; l++) {
            int y = y0 + l;                      // 0 = top of panel
            int from_bottom = spec_h_px - y;     // 1..spec_h_px
            bool grid_row = (y % 20) == 0;
            uint16_t *row = b + l * LCD_W;
            for (int x = 0; x < LCD_W; x++) {
                uint8_t fl = col_flags[x];
                uint16_t c;
                if (fl & F_DIAL)                       c = C_DIAL;
                else if (from_bottom >= line_lo[x] && from_bottom <= line_hi[x]) c = C_TEXT;
                else if (from_bottom <= spec_h[x])     c = heat_pal[spec_lv[x]];
                else if (fl & F_PASS)                  c = C_PASS;
                else if ((fl & F_GRID) || grid_row)    c = C_GRID;
                else                                   c = C_BG;
                row[x] = c;
            }
        }
        lcd_buf_send(0, SPEC_Y + y0, LCD_W, n);
    }
}

static void draw_waterfall(void)
{
    // no spectrum above it -> the dial is a dashed red line through the waterfall
    int xd = -1;
    double c = st_valid ? st_center : cur_center();   // what the rows are aligned to now
    if (spec_h_px == 0)
        for (int x = 0; x < LCD_W; x++) if (col_flags[x] & F_DIAL) xd = x;
    for (int y0 = 0; y0 < wf_h; y0 += LCD_CHUNK) {
        int n = wf_h - y0 < LCD_CHUNK ? wf_h - y0 : LCD_CHUNK;
        uint16_t *b = lcd_buf_get();
        for (int l = 0; l < n; l++) {
            int r = (wf_head + y0 + l) % WF_MAX;
            const uint8_t *src = wf[r];
            uint16_t *row = b + l * LCD_W;
            // row x shows freq wf_ctr + (x - W/2) * HZ_PER_PX -> screen x + off
            int off = src_sim || !scroll_on ? 0 : (int)lrint((wf_ctr[r] - c) / HZ_PER_PX);
            for (int x = 0; x < LCD_W; x++) {
                int xs = x - off;
                if (xs >= 0 && xs < LCD_W) { row[x] = heat_pal[src[xs]]; continue; }
                // no data here: a pseudo-random pixel of the same row, capped
                // at its noise level -> looks like the noise around it
                uint32_t h = ((uint32_t)x * 2654435761u) ^ ((uint32_t)r * 40503u);
                uint8_t v = src[(h >> 16) % LCD_W];
                row[x] = heat_pal[v > wf_cap[r] ? wf_cap[r] : v];
            }
            if (xd >= 0 && ((y0 + l) & 4)) row[xd] = C_DIAL;
        }
        lcd_buf_send(0, wf_y + y0, LCD_W, n);
    }
}

static int frames;
static int64_t t_sec, sum_gen, sum_fft, sum_draw;

void spectrum_init(void)
{
    make_palette();
    wf = heap_caps_calloc(WF_MAX, LCD_W, MALLOC_CAP_SPIRAM);
    if (!wf) wf = heap_caps_calloc(WF_MAX, LCD_W, MALLOC_CAP_8BIT);
    assert(wf);
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_N));
    for (int n = 0; n < FFT_N; n++) {
        float t = 2.0f * (float)M_PI * n / FFT_N;
        win_tab[SPEC_WIN_OFF][n] = 1.0f;
        win_tab[SPEC_WIN_HANN][n] = 0.5f - 0.5f * cosf(t);
        win_tab[SPEC_WIN_BH][n] = 0.35875f - 0.48829f * cosf(t) + 0.14128f * cosf(2 * t) - 0.01168f * cosf(3 * t);
    }
    for (int w = 0; w < SPEC_WIN_N; w++) {
        win_pow[w] = 0;
        for (int n = 0; n < FFT_N; n++) win_pow[w] += win_tab[w][n] * win_tab[w][n];
    }
    // pixel x covers [x - W/2 - 0.5, x - W/2 + 0.5) pixels = u0..u0+BINS_PX
    // bins from the LO; bin k covers [k - 0.5, k + 0.5)
    for (int x = 0; x < LCD_W; x++) {
        float u0 = (x - LCD_W / 2 - 0.5f) * BINS_PX, u1 = u0 + BINS_PX;
        int k = (int)floorf(u0 + 0.5f);
        px_k0[x] = k;
        px_nk[x] = 0;
        for (; k - 0.5f < u1 && px_nk[x] < 3; k++) {
            float a = fmaxf(u0, k - 0.5f), b = fminf(u1, k + 0.5f);
            px_w[x][px_nk[x]++] = b > a ? b - a : 0;
        }
    }
    siggen_init();
}

void spectrum_draw_static(void)
{
    draw_header();
    draw_ruler();
    frames = 0;
    sum_gen = sum_fft = sum_draw = 0;
    t_sec = esp_timer_get_time();
}

void spectrum_step(const float *iq, const char *src_name)
{
    int64_t t0 = esp_timer_get_time();
    src_sim = !iq;
    if (iq) memcpy(fft_buf, iq, sizeof(fft_buf));
    else    siggen_block(fft_buf);
    int64_t t1 = esp_timer_get_time();
    spectrum_process();
    int64_t t2 = esp_timer_get_time();
    draw_spectrum();
    draw_waterfall();
    int64_t t3 = esp_timer_get_time();
    sum_gen += t1 - t0;
    sum_fft += t2 - t1;
    sum_draw += t3 - t2;
    frames++;

    if (t3 - t_sec >= 1000000) {
        float fps = frames * 1e6f / (t3 - t_sec);
        draw_speed(fps);
        lcd_text(LCD_W - 4 - 4 * font_small.w, 4, &font_small, src_name, C_DIM, C_HDR_BG);
        ESP_LOGI(TAG, "%s %.1f FPS: input %lld us, fft+levels %lld us, draw %lld us",
                 src_name, fps, sum_gen / frames, sum_fft / frames, sum_draw / frames);
        frames = 0;
        sum_gen = sum_fft = sum_draw = 0;
        t_sec = t3;
    }
}

bool spectrum_sq_snr(float *snr_db, float *sig_db, float *noise_db)
{
    if (snr_db) *snr_db = sq_snr_db;
    if (sig_db) *sig_db = 10.0f * log10f(sq_sig > 0 ? sq_sig : 1e-30f);
    if (noise_db) *noise_db = 10.0f * log10f(sq_noise > 0 ? sq_noise : 1e-30f);
    bool f = sq_fresh;
    sq_fresh = false;
    return f;
}

void spectrum_feed(const float *iq)
{
    src_sim = false;
    memcpy(fft_buf, iq, sizeof(fft_buf));
    spectrum_process();
}

// No usable frame (the I/Q straddles a retune): redraw the old picture,
// shifted to the new dial, without adding a waterfall row.
void spectrum_hold(void)
{
    if (!st_valid) return;
    state_follow();
    spectrum_render(false);
    draw_spectrum();
    draw_waterfall();
}

// ---- S-meter calibration (main.c "sm") ----
void spectrum_sm_start(int frames)
{
    sm_p = sm_nz = 0;
    sm_min = 1e9f; sm_max = -1e9f;
    sm_n = sm_left = frames;
}

// "SM <dial> <pb_lo>..<pb_hi> Hz: <dBm mean> (min .. max) noise <dBm> snr <dB> n <frames>"
bool spectrum_sm_dump(void)
{
    if (sm_n == 0 || sm_left > 0) return false;
    float p = 10.0f * log10f(sm_p / sm_n + 1e-12) + FPGA_DBM_1LSB;
    float nz = 10.0f * log10f(sm_nz / sm_n + 1e-12) + FPGA_DBM_1LSB;
    printf("SM %lu %.0f..%.0f Hz: %.2f dBm (%.2f .. %.2f) noise %.2f dBm snr %.1f dB n %d\n",
           (unsigned long)dial_hz, pb_lo, pb_hi, p, sm_min, sm_max, nz, p - nz, sm_n);
    fflush(stdout);
    sm_n = 0;
    return true;
}

// ---- debug capture for the serial console (main.c "s") ----
void spectrum_capture_start(int frames)
{
    memset(cap_acc, 0, sizeof(cap_acc));
    cap_n = cap_left = frames;
}

// One line: "SPEC <dial Hz> <LO offset Hz> <window> <frames> <bin Hz>:" and
// FFT_N levels in dB (0.1dB), bin 0 = -fs/2 ... bin FFT_N/2 = the LO.
bool spectrum_capture_dump(void)
{
    if (cap_n == 0 || cap_left > 0) return false;
    printf("SPEC %lu %.1f %d %d %.4f:", (unsigned long)dial_hz, lo_shift_hz, view_win, cap_n, FS / FFT_N);
    for (int b = 0; b < FFT_N; b++)
        printf(" %d", (int)lroundf(100.0f * log10f(cap_acc[b] / cap_n + 1e-20f)));
    printf("\n");
    fflush(stdout);
    cap_n = 0;
    return true;
}
