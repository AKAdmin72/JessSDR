// Main screen: header, ruler, spectrum and waterfall (PLAN.md Phase 3),
// ported from ../tang_nano_9k_projects/FpgaRX-SDR to the 480x320 ST7796S.
//
//   y   0..47   header: dial frequency, mode/step, ADC/speed readout
//   y  48..65   frequency ruler (absolute kHz)
//   y  66..319  panel, by the view (menu: Display > View):
//                 SPECTRUM (default) = spectrum 254 px,
//                 WATERFALL          = waterfall 254 px (dashed red dial line),
//                 BOTH               = spectrum 120 px (66..185) + waterfall 134 px
//   spectrum = heatmap-filled, grid, passband, red dial mark
// Input is either the real I/Q block from the FPGA (I2S, the wide DDC
// output, 105.54kHz complex, centred on the dial) or a synthetic stream
// like it: a few LSB "voices", CW carriers and noise. FFT 1024, no window
// (the user prefers stations as broad shapes). The header shows the
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
#define SPEC_H_BOTH 120
#define WF_MAX     PANEL_H              // waterfall ring rows (the tallest view)

static int spec_h_px = PANEL_H;         // spectrum panel height, 0 = hidden
static int wf_y = LCD_H, wf_h = 0;      // waterfall panel, wf_h 0 = hidden

// ---- signal ----
#define FS         FPGA_FS_HZ           // wide DDC output, 64.8MHz / 614
#define FFT_N      1024
#define HZ_PER_PX  (FS / LCD_W)

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
static float view_range = 40;  // dB over the spectrum panel (menu: Range)
static bool view_win;          // Hann window before the FFT (menu: Window)
static float hann[FFT_N];

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
#define EQ_SLOPE   (0.4f * HZ_PER_PX / 206.0f)   // dB per pixel (SLOPE=1 level/bin)
// FpgaRX had a 1/f + LO-leakage hump around the LO and a steeper envelope
// there (DC_W/DC_SLOPE). The direct-sampling DDC has none, and the dial is
// at the centre now, so that zone would only hide the station listened to.
static float slope_at(int x) { return EQ_SLOPE; }

static void spectrum_process(void)
{
    static int64_t t_prev;
    int64_t now = esp_timer_get_time();
    float dt = t_prev ? (now - t_prev) * 1e-6f : 0;
    t_prev = now;
    if (dt > 0.2f) dt = 0.2f;

    if (view_win)
        for (int n = 0; n < FFT_N; n++) { fft_buf[2 * n] *= hann[n]; fft_buf[2 * n + 1] *= hann[n]; }
    dsps_fft2r_fc32(fft_buf, FFT_N);
    dsps_bit_rev_fc32(fft_buf, FFT_N);

    // bins -> pixels, fftshift (negative frequencies on the left), mean power
    // per pixel (less noise variance than the max)
    for (int x = 0; x < LCD_W; x++) {
        int b0 = x * FFT_N / LCD_W, b1 = (x + 1) * FFT_N / LCD_W;
        float m = 0;
        for (int b = b0; b < b1; b++) {
            int k = (b + FFT_N / 2) & (FFT_N - 1);
            m += fft_buf[2 * k] * fft_buf[2 * k] + fft_buf[2 * k + 1] * fft_buf[2 * k + 1];
        }
        px_db[x] = 10.0f * log10f(m / (b1 - b0) + 1e-20f);
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
    for (int x = 0; x < LCD_W; x++) {
        if (warm < EQ_SKIP) {
            px_avg[x] = px_wf[x] = est[x] = px_db[x];
            continue;
        }
        px_avg[x] += (px_db[x] - px_avg[x]) * AVG_A;
        px_wf[x]  += (px_db[x] - px_wf[x]) * WF_A;
        if (warm < EQ_WARM) est[x] = px_avg[x];
        else est[x] = px_avg[x] < est[x] ? px_avg[x] : est[x] + EQ_RISE * dt;
    }
    if (warm < EQ_WARM) warm++;

    // 3. lower envelope across frequency: min over k of est[x+k] + |k|*SLOPE.
    // Follows the wide center hump and the edge roll-off, but a station
    // cannot lift it by more than SLOPE per pixel of its half-width.
    env[0] = est[0];
    for (int x = 1; x < LCD_W; x++) env[x] = fminf(est[x], env[x - 1] + slope_at(x));
    for (int x = LCD_W - 2; x >= 0; x--) env[x] = fminf(env[x], env[x + 1] + slope_at(x));

    // 4. output = level - envelope: flat noise along the bottom. view_range
    // dB fill the panel (was a fixed 60dB, waterfall 45dB: a 3-6dB station
    // was only 5-10px high and nearly black in the waterfall).
    wf_head = (wf_head + WF_MAX - 1) % WF_MAX;
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
        int w = (int)((px_wf[x] - env[x] + 1.0f) * k_wf);
        wf[wf_head][x] = w < 0 ? 0 : w > 255 ? 255 : w;
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
static int sm_drawn_px = -1, sm_drawn_kind = -1;
static char sm_drawn_txt[8];
static void draw_smeter(bool force)
{
    char t[16];
    int px, split;
    uint16_t c_lo, c_hi;
    if (vol_shown >= 0) {
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
        c_lo = RGB565(0, 220, 80); c_hi = RGB565(255, 40, 40);
    }
    int kind = vol_shown >= 0;
    char s[8];
    snprintf(s, sizeof(s), "%-5.5s", t);
    if (force || kind != sm_drawn_kind || strcmp(s, sm_drawn_txt) != 0) {
        strcpy(sm_drawn_txt, s);
        lcd_text(SM_X, ROW2_Y, &font_mid, s, kind ? RGB565(0, 200, 255) : C_TEXT, C_HDR_BG);
    }
    if (force || px != sm_drawn_px || kind != sm_drawn_kind) {
        uint16_t *b = lcd_buf_get();
        for (int y = 0; y < 12; y++)
            for (int x = 0; x < SM_BAR_W; x++) {
                bool on = x < px && y >= 2 && y < 10;
                bool tick = (y >= 10) && (kind ? (x == split) : (x % 9 == 0 && x <= 80) || (x > 80 && (x - 80) % 13 == 0));
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

static void draw_speed(float fps);
static bool show_fps;   // menu: FPS (off by default)
static float last_fps;
static struct { bool clip; float pk_db; } adc_st = { false, -100.0f };   // no peak until the FPGA reports one
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

void spectrum_set_view(int range_db, bool window)
{
    view_range = range_db;
    view_win = window;
}

void spectrum_set_smooth(bool on)
{
    view_smooth = on;
}

void spectrum_floor_reset(void)
{
    warm = 0;
}

void spectrum_set_layout(int view)
{
    switch (view) {
    case SPEC_VIEW_WATERFALL: spec_h_px = 0;           break;
    case SPEC_VIEW_BOTH:      spec_h_px = SPEC_H_BOTH; break;
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
    draw_freq();
    if (moved) draw_ruler();
}


// Header slot under the mode, 6 characters. ADC overload (adc_peak.v)
// takes priority over the frame rate: white "CLIP" on red = the ADC
// clipped within the last second; yellow "-3.2dB" = peak above -6dBFS
// (little headroom left); otherwise the FPS if switched on, or blank.
static void draw_speed(float fps)
{
    char t[16], s[7];
    uint16_t fg = RGB565(0, 255, 120), bg = C_HDR_BG;
    last_fps = fps;
    if (adc_st.clip) {
        snprintf(t, sizeof(t), " CLIP ");
        fg = C_TEXT; bg = RGB565(220, 0, 0);
    } else if (adc_st.pk_db > -6.0f) {
        snprintf(t, sizeof(t), "%4.1fdB", adc_st.pk_db);
        fg = RGB565(255, 220, 0);
    } else if (show_fps) snprintf(t, sizeof(t), "%2.0ffps", fps);
    else t[0] = 0;
    snprintf(s, sizeof(s), "%-6.6s", t);
    if (strcmp(s, speed_shown) == 0) return;
    strcpy(speed_shown, s);
    lcd_text(MODE_X, ROW2_Y, &font_mid, s, fg, bg);
}

void spectrum_show_fps(bool on)
{
    show_fps = on;
    draw_speed(0);   // clears it at once when switched off; the value follows within 1s
}

void spectrum_set_adc(bool clip, float peak_dbfs)
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
    if (spec_h_px == 0)
        for (int x = 0; x < LCD_W; x++) if (col_flags[x] & F_DIAL) xd = x;
    for (int y0 = 0; y0 < wf_h; y0 += LCD_CHUNK) {
        int n = wf_h - y0 < LCD_CHUNK ? wf_h - y0 : LCD_CHUNK;
        uint16_t *b = lcd_buf_get();
        for (int l = 0; l < n; l++) {
            const uint8_t *src = wf[(wf_head + y0 + l) % WF_MAX];
            uint16_t *row = b + l * LCD_W;
            for (int x = 0; x < LCD_W; x++) row[x] = heat_pal[src[x]];
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
    for (int n = 0; n < FFT_N; n++) hann[n] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n / FFT_N);
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
