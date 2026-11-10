// Main screen: header, ruler, spectrum and waterfall (PLAN.md Phase 3),
// ported from ../tang_nano_9k_projects/FpgaRX-SDR to the 480x320 ST7796S.
//
//   y   0..47   header: dial frequency, mode/step, ADC/speed readout
//   y  48..65   frequency ruler (absolute kHz)
//   y  66..185  spectrum (heatmap-filled, grid, passband, red dial mark)
//   y 186..319  waterfall
// Input is either the real I/Q block from the FPGA (I2S, the wide DDC
// output, 105.54kHz complex, centred on the dial) or a synthetic stream
// like it: a few LSB "voices", CW carriers and noise. FFT 1024, no window
// (the user prefers stations as broad shapes). The header shows the
// spectrum rate and where the time goes.
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "dsps_fft2r.h"
#include "lcd.h"
#include "spectrum.h"
#include "fpga.h"

static const char *TAG = "spectrum";

// ---- layout ----
#define HDR_H      48
#define RULER_Y    48
#define RULER_H    18
#define SPEC_Y     66
#define SPEC_H     120                  // multiple of LCD_CHUNK
#define WF_Y       186
#define WF_H       (LCD_H - WF_Y)

// ---- signal ----
#define FS         FPGA_FS_HZ           // wide DDC output, 64.8MHz / 614
#define FFT_N      1024
#define HZ_PER_PX  (FS / LCD_W)

static uint32_t dial_hz = 7100000;
static float lo_shift_hz = 0;           // K * fs/16; K = 0 here (DDC centred on the dial)
static int step_idx = 3;                // 1 = 10Hz ... 6 = 1MHz (tune_ctrl.v numbering)
static int demod = FPGA_MODE_LSB;       // FPGA_MODE_*
static bool demod_auto = true;          // chosen by the band plan

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
static bool primed;
static uint8_t spec_h[LCD_W];  // spectrum column height 0..SPEC_H
static uint8_t spec_lv[LCD_W]; // spectrum column colour level
static uint8_t wf[WF_H][LCD_W];
static int wf_head;            // index of the newest waterfall row
static float view_range = 40;  // dB over the spectrum panel (menu: Range)
static bool view_win;          // Hann window before the FFT (menu: Window)
static float hann[FFT_N];

// Same numbers as the Tang build (where 1 level = 0.4dB, 206Hz per bin,
// 206 FFT frames/s), converted to dB, pixels and seconds:
#define AVG_A      0.125f                 // level_averager AVG_SHIFT=3
#define WF_A       0.5f
#define EQ_RISE    1.6f                   // dB/s, est creep up (RISE=5)
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
        if (!primed) {
            px_avg[x] = px_wf[x] = est[x] = px_db[x];
            continue;
        }
        px_avg[x] += (px_db[x] - px_avg[x]) * AVG_A;
        px_wf[x]  += (px_db[x] - px_wf[x]) * WF_A;
        est[x] = px_avg[x] < est[x] ? px_avg[x] : est[x] + EQ_RISE * dt;
    }
    primed = true;

    // 3. lower envelope across frequency: min over k of est[x+k] + |k|*SLOPE.
    // Follows the wide center hump and the edge roll-off, but a station
    // cannot lift it by more than SLOPE per pixel of its half-width.
    env[0] = est[0];
    for (int x = 1; x < LCD_W; x++) env[x] = fminf(est[x], env[x - 1] + slope_at(x));
    for (int x = LCD_W - 2; x >= 0; x--) env[x] = fminf(env[x], env[x + 1] + slope_at(x));

    // 4. output = level - envelope: flat noise along the bottom. view_range
    // dB fill the panel (was a fixed 60dB, waterfall 45dB: a 3-6dB station
    // was only 5-10px high and nearly black in the waterfall).
    wf_head = (wf_head + WF_H - 1) % WF_H;
    float k_h = SPEC_H / view_range, k_lv = 255.0f / (view_range * 0.92f), k_wf = 255.0f / (view_range * 0.75f);
    for (int x = 0; x < LCD_W; x++) {
        float rel = px_avg[x] - env[x];                 // dB above the floor
        int h = (int)((rel + 1.0f) * k_h);
        spec_h[x] = h < 0 ? 0 : h > SPEC_H ? SPEC_H : h;
        int lv = (int)((rel + 1.0f) * k_lv);
        spec_lv[x] = lv < 0 ? 0 : lv > 255 ? 255 : lv;
        int w = (int)((px_wf[x] - env[x] + 1.0f) * k_wf);
        wf[wf_head][x] = w < 0 ? 0 : w > 255 ? 255 : w;
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
    float p0 = demod == FPGA_MODE_AM ? -5000.0f : demod == FPGA_MODE_USB ? 300.0f : -2700.0f;
    float p1 = demod == FPGA_MODE_AM ?  5000.0f : demod == FPGA_MODE_USB ? 2700.0f : -300.0f;
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
static void draw_mode(void)
{
    static const char *name[] = { "LSB", "USB", "AM " };
    lcd_text(MODE_X, 4, &font_mid, name[demod], demod_auto ? RGB565(255, 220, 0) : RGB565(0, 220, 255), C_HDR_BG);
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
}

void spectrum_set_mode(int mode, bool is_auto)
{
    bool moved = mode != demod;
    demod = mode;
    demod_auto = is_auto;
    draw_mode();
    if (moved) draw_ruler();
}

void spectrum_set_view(int range_db, bool window)
{
    view_range = range_db;
    view_win = window;
}

void spectrum_set_dial(uint32_t hz, int step, int lo_shift_k)
{
    float shift = lo_shift_k * (FS / 16);
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
    for (int y0 = 0; y0 < SPEC_H; y0 += LCD_CHUNK) {
        uint16_t *b = lcd_buf_get();
        for (int l = 0; l < LCD_CHUNK; l++) {
            int y = y0 + l;                      // 0 = top of panel
            int from_bottom = SPEC_H - y;        // 1..SPEC_H
            bool grid_row = (y % 20) == 0;
            uint16_t *row = b + l * LCD_W;
            for (int x = 0; x < LCD_W; x++) {
                uint8_t fl = col_flags[x];
                uint16_t c;
                if (fl & F_DIAL)                       c = C_DIAL;
                else if (from_bottom <= spec_h[x])     c = (from_bottom == spec_h[x]) ? C_TEXT : heat_pal[spec_lv[x]];
                else if (fl & F_PASS)                  c = C_PASS;
                else if ((fl & F_GRID) || grid_row)    c = C_GRID;
                else                                   c = C_BG;
                row[x] = c;
            }
        }
        lcd_buf_send(0, SPEC_Y + y0, LCD_W, LCD_CHUNK);
    }
}

static void draw_waterfall(void)
{
    for (int y0 = 0; y0 < WF_H; y0 += LCD_CHUNK) {
        int n = WF_H - y0 < LCD_CHUNK ? WF_H - y0 : LCD_CHUNK;
        uint16_t *b = lcd_buf_get();
        for (int l = 0; l < n; l++) {
            const uint8_t *src = wf[(wf_head + y0 + l) % WF_H];
            uint16_t *row = b + l * LCD_W;
            for (int x = 0; x < LCD_W; x++) row[x] = heat_pal[src[x]];
        }
        lcd_buf_send(0, WF_Y + y0, LCD_W, n);
    }
}

static int frames;
static int64_t t_sec, sum_gen, sum_fft, sum_draw;

void spectrum_init(void)
{
    make_palette();
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
