// JessSDR: 40m SDR UI firmware for ESP32-S3 + 4" ST7796S 480x320 (PLAN.md Phase 3),
// ported from ../tang_nano_9k_projects/FpgaRX-SDR/esp32.
//
// Screens: the main spectrum/waterfall screen, a menu tree, and the link
// test screens in MENU > SYSTEM. Controls (input.h):
//   main screen: turn = tune by the step (faster turn = bigger jumps),
//                SELECT = next step, hold SELECT = menu
//   menu:        turn = move, SELECT or hold = enter, BACK = up one level
// The BOOT button and Tang S1/S2 still work too: short press = NEXT
// (tune up / move down), hold = HOLD (menu / enter).
//
// The ESP owns the dial and the demodulator mode: it sends them to the FPGA
// (spi_regs FREQ, MODE), keeps them in NVS across power cycles, and re-sends
// them if the FPGA was restarted. Mode AUTO picks LSB/USB/AM from the band
// plan (band_mode()) as the dial moves.
//
// At power-up a short link self-test (SPI, LO retune, I2S) runs and logs
// its result to the serial console. If the FPGA does not answer with its ID
// the ESP runs on its own: SIM source, no register polling.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lcd.h"
#include "splash.h"
#include "spectrum.h"
#include "fpga.h"
#include "input.h"
#include "noise.h"

static const char *TAG = "jesssdr";   // (the NVS namespace stays "fpgarx": saved settings)

#define C_BG       RGB565(0, 0, 0)
#define C_PANEL    RGB565(10, 10, 30)
#define C_TITLE_BG RGB565(0, 60, 120)
#define C_TEXT     RGB565(255, 255, 255)
#define C_DIM      RGB565(150, 150, 170)
#define C_SEL_BG   RGB565(255, 200, 0)
#define C_EDIT_BG  RGB565(0, 200, 255)   // menu item whose value the encoder changes
#define C_SEL_FG   RGB565(0, 0, 0)
#define C_OK       RGB565(0, 230, 100)
#define C_FAIL     RGB565(255, 60, 60)

#define AREA_Y     48                // menus and tests use everything below the header
#define AREA_H     (LCD_H - AREA_Y)
#define ROW_H      26                // font_mid is 19 high
#define TEST_ROW_H 24
#define TEST_VAL_X 230

// ---------------------------------------------------------------------------
// Menu tree
// ---------------------------------------------------------------------------
typedef enum { ACT_NONE, ACT_SUBMENU, ACT_BACK, ACT_EXIT, ACT_MODE, ACT_VIEW, ACT_RANGE, ACT_WINDOW, ACT_SMOOTH, ACT_SCROLL, ACT_WFSPD, ACT_FPS, ACT_NR, ACT_NR_LEVEL, ACT_FILTER, ACT_SHIFT, ACT_AGC, ACT_VOLUME, ACT_SQL, ACT_PPM, ACT_SOURCE, ACT_TEST_SPI, ACT_TEST_I2S, ACT_TEST_NOISE } action_t;
typedef struct menu menu_t;
typedef struct { const char *label; action_t act; const menu_t *sub; } item_t;
struct menu { const char *title; const item_t *items; int n; const menu_t *parent; };

static const menu_t menu_main, menu_system, menu_display, menu_audio;
static const item_t audio_items[] = {
    { .label = "NR", .act = ACT_NR },               // ": OFF" / ": NR" / ": ANF"
    { .label = "Level", .act = ACT_NR_LEVEL },      // ": 1".."4"
    { .label = "Filter", .act = ACT_FILTER },       // ": 2.4k" / CW ": 500"
    { .label = "Shift", .act = ACT_SHIFT },         // IF shift, SSB, edited with the encoder
    { .label = "AGC", .act = ACT_AGC },             // ": FAST" / ": MED" / ": SLOW"
    { .label = "Volume", .act = ACT_VOLUME },       // edited with the encoder
    { .label = "Squelch", .act = ACT_SQL },         // ": OFF" / ": 8 dB", edited with the encoder
    { .label = "Back", .act = ACT_BACK },
};
static const item_t display_items[] = {
    { .label = "View", .act = ACT_VIEW },       // ": SPECTRUM" / ": WATERFALL" / ": BOTH 160/94" ...
    { .label = "Range", .act = ACT_RANGE },     // ": 40 dB"
    { .label = "Window", .act = ACT_WINDOW },   // ": OFF" / ": HANN" / ": BH"
    { .label = "Smooth", .act = ACT_SMOOTH },   // ": ON" / ": OFF"
    { .label = "Retune", .act = ACT_SCROLL },   // ": SCROLL" / ": STATIC"
    { .label = "WF speed", .act = ACT_WFSPD },  // ": 1/1" / ": 1/2" / ": 1/3" rows per frame
    { .label = "FPS", .act = ACT_FPS },         // ": ON" / ": OFF"
    { .label = "Back", .act = ACT_BACK },
};
static const item_t system_items[] = {
    { .label = "SPI link test", .act = ACT_TEST_SPI },
    { .label = "I2S link test", .act = ACT_TEST_I2S },
    { .label = "Noise test", .act = ACT_TEST_NOISE },   // 50 ohm vs antenna
    { .label = "Freq cal", .act = ACT_PPM },        // LO correction, ppm, edited with the encoder
    { .label = "Back", .act = ACT_BACK },
};
static const item_t main_items[] = {
    { .label = "Mode", .act = ACT_MODE },       // label gets ": AUTO (LSB)" / ": USB" ...
    { .label = "Audio", .act = ACT_SUBMENU, .sub = &menu_audio },
    { .label = "Display", .act = ACT_SUBMENU, .sub = &menu_display },
    { .label = "System", .act = ACT_SUBMENU, .sub = &menu_system },
    { .label = "Source", .act = ACT_SOURCE },   // label gets ": SIM" / ": FPGA"
    { .label = "Exit", .act = ACT_EXIT },
};
static const menu_t menu_main   = { "MENU",   main_items,   6, NULL };
static const menu_t menu_system = { "SYSTEM", system_items, 5, &menu_main };
static const menu_t menu_display = { "DISPLAY", display_items, 8, &menu_main };
static const menu_t menu_audio   = { "AUDIO",   audio_items,   8, &menu_main };

static bool src_fpga = true;   // spectrum input: false = synthetic, true = FPGA I2S (kept in NVS)
// Display (kept in NVS): dB over the spectrum panel, Hann window on/off
static const uint8_t range_db[] = { 20, 30, 40, 60 };
static int range_idx = 2;
static int view_sel = SPEC_VIEW_SPECTRUM;   // main panel: spectrum / waterfall / both
static const char *const view_name[] = { "SPECTRUM", "WATERFALL", "BOTH" };
// BOTH: spectrum / waterfall heights (panel 254 px, grid 20 px), kept in NVS
static const uint8_t split_spec[] = { 160, 140, 120 };
static int split_idx = 0;
// waterfall: one row per wf_div[] frames (row = their mean), kept in NVS
static const uint8_t wf_div[] = { 1, 2, 3 };
static int wfspd_idx = 0;
static int win_sel = SPEC_WIN_OFF;   // FFT window: OFF / HANN / BH (OFF = user preference)
static const char *const win_name[] = { "OFF", "HANN", "BH" };
static bool fpga_present;      // the FPGA answered with FPGA_ID at power-up
static bool smooth_on = true;  // spectrum smoothing across frequency, kept in NVS
static bool scroll_on = true; // retune: shift the picture with the dial, kept in NVS
static bool fps_on = false;    // frame rate in the header, kept in NVS
// FPGA LMS noise reduction (audio_nr.v): FPGA_NR_*, level 0..3 (shown 1..4,
// 4 = strongest). The FPGA's level field is the other way round, hence
// 3 - nr_level. NR: Level 1/2/3/4 = residual e/2, e/4, e/8, none mixed back
// (1 = mildest, keeps the highs; 4 = the old out = y), mu fixed 2^-11.
// ANF: Level = adaptation speed, mu = 2^-(10+field). Default Level 2.
// NVS key "nrl3" since 2026-11-15: the NR meaning changed, the old key is ignored.
static int nr_mode = FPGA_NR_OFF, nr_level = 1;
static const char *nr_name[] = { "OFF", "NR", "ANF" };
static uint8_t nr_reg(void) { return nr_mode | (3 - nr_level) << 2; }
// No I/Q correction: the DDC I/Q are digital, so perfectly balanced.
static uint8_t ctrl_base(void) { return 0; }

// ADC overload: the FPGA keeps, per 0.26s window, the max |code-2048|
// (PEAK) and, from VERSION 8, the number of clipped samples (CLIP, code at
// a rail or OTR, log2-coded); read every 100ms so no window is missed.
// Header for 1s after the window: red CLIP = steady clipping (>= 16
// samples per window), orange = rare single peaks (1..15, harmless in
// practice), else the highest peak of the last second if above -6dBFS.
// An older FPGA (no CLIP register): red CLIP when PEAK >= -0.6dBFS.
#define CLIP_STEADY_LOG 5     // CLIP >= 5: >= 16 clipped samples in 0.26s
static void adc_status(int64_t now)
{
    static int64_t t_clip = -10000000, t_pk, t_log, t_ver = -10000000;
    static uint8_t pk_hold, ver, cl_hold;
    if (now - t_ver > 1000000) { t_ver = now; ver = fpga_read(FPGA_REG_VERSION); }   // follows an FPGA reload
    uint8_t p = fpga_read(FPGA_REG_PEAK);
    uint8_t cl = ver >= 8 ? fpga_read(FPGA_REG_CLIP) : p >= FPGA_PEAK_CLIP ? CLIP_STEADY_LOG : 0;
    if (cl) {
        if (now - t_clip > 1000000 && now - t_log > 1000000) {
            t_log = now;
            if (ver >= 8)
                ESP_LOGW(TAG, "ADC CLIP: %u..%u samples / 0.26s (peak %u/255)%s", 1u << (cl - 1), (1u << cl) - 1, p,
                         cl >= CLIP_STEADY_LOG ? " - too much gain / signal" : " - rare peaks");
            else ESP_LOGW(TAG, "ADC CLIP (peak %u/255) - too much gain / signal", p);
        }
        if (cl >= cl_hold || now - t_clip > 1000000) cl_hold = cl;
        t_clip = now;
    }
    if (p >= pk_hold || now - t_pk > 1000000) { pk_hold = p; t_pk = now; }
    int lvl = now - t_clip >= 1000000 ? 0 : cl_hold >= CLIP_STEADY_LOG ? 2 : 1;
    spectrum_set_adc(lvl, 20.0f * log10f((pk_hold + 0.5f) / 256.0f));
    // S-meter: FFT power in the passband (spectrum.c); the FPGA's LEVEL
    // register measures the whole +-6kHz decimator stream (neighbours too)
    spectrum_set_smeter(spectrum_smeter_dbm());
}

// ---------------------------------------------------------------------------
// Dial
// ---------------------------------------------------------------------------
#define F_MIN      100000
#define F_MAX      30000000
static const uint32_t step_hz[] = { 0, 10, 100, 1000, 10000, 100000, 1000000 };
static uint32_t dial_hz = 7100000;
static int step_idx = 3;            // 1kHz
static int lo_shift_k = 1;          // LO offset K of cur_mode (fpga_mode_k)
static bool nvs_dirty;
static int64_t t_last_tune;

// Demodulator: mode_sel = FPGA_MODE_* (set by hand) or MODE_AUTO;
// cur_mode = what the FPGA runs now.
// MODE_CW is UI-only: the FPGA runs USB with the LO CW_PITCH below the
// dial, so a carrier on the dial is heard at CW_PITCH, through a narrow
// filter around it.
#define MODE_AUTO  3
#define MODE_CW    4
#define CW_PITCH   700
static int mode_sel = MODE_AUTO;
static int cur_mode = -1;
static const char *mode_name[] = { "LSB", "USB", "AM", "AUTO", "CW" };
static int fpga_mode_of(int m) { return m == MODE_CW ? FPGA_MODE_USB : m; }

// Demodulator filter (sdr_esp ssb_demod.v, taps computed by fpga_set_filter):
// audio passband per SSB width, CW width around CW_PITCH; IF shift moves the
// SSB passband (audio Hz, + = higher). AM uses the fixed +-6kHz decimator.
static const struct { int lo, hi; const char *name; } ssb_bw[] = {
    { 300, 2100, "1.8k" }, { 300, 2700, "2.4k" }, { 200, 2900, "2.7k" }, { 100, 3100, "3.0k" },
};
static const struct { int w; const char *name; } cw_bw[] = { { 1000, "1k" }, { 500, "500" }, { 250, "250" } };
#define N_SSB_BW   (int)(sizeof(ssb_bw) / sizeof(ssb_bw[0]))
#define N_CW_BW    (int)(sizeof(cw_bw) / sizeof(cw_bw[0]))
static int ssb_bw_idx = 1, cw_bw_idx = 1;
static int if_shift = 0;                 // Hz, -1000..+1000, step 50
static int agc_speed = FPGA_AGC_MED;
static const char *agc_name[] = { "FAST", "MED", "SLOW" };
// Volume: 0..23 in 2dB steps, 20 = 0dB (the level before VOL existed), 0 = mute
#define VOL_MAX    23
static int vol_idx = 20;
static uint8_t vol_user_reg(void)
{
    if (vol_idx <= 0) return 0;
    long r = lrintf(128.0f * powf(10.0f, (vol_idx - 20) * 0.1f));
    return r > 255 ? 255 : r;
}
// Squelch (docs/squelch_idea.md, variant 2): opens when the SNR in the
// filter passband (spectrum.c, dB over the noise per FFT bin) reaches
// sql_db, closes SQ_HANG_US after it fell below sql_db - SQ_HYST (but not
// below SQ_CLOSE_MIN, the noise itself wanders ~+-1dB). Muting = VOL 0,
// ramped so it doesn't click. 0 = off. Kept in NVS ("sql").
#define SQL_MAX      30
#define SQ_HYST      2.0f
#define SQ_CLOSE_MIN 1.5f
#define SQ_HANG_US   800000
static int sql_db = 0;
static bool sq_muted;
static uint8_t vol_reg(void) { return sq_muted ? 0 : vol_user_reg(); }
// LO calibration: the 27MHz crystal is off by ppm -> program f / (1 + ppm)
static int ppm_x10 = 0;                  // 0.1 ppm, -1000..+1000
static uint32_t lo_hz(void)
{
    double f = (double)dial_hz + (cur_mode == MODE_CW ? -CW_PITCH : 0);
    return (uint32_t)llround(f / (1.0 + ppm_x10 * 1e-7));
}
// audio passband of the current mode
static void audio_band(float *lo, float *hi)
{
    if (cur_mode == MODE_CW) {
        *lo = CW_PITCH - cw_bw[cw_bw_idx].w / 2;
        *hi = CW_PITCH + cw_bw[cw_bw_idx].w / 2;
    } else {
        *lo = ssb_bw[ssb_bw_idx].lo + if_shift;
        *hi = ssb_bw[ssb_bw_idx].hi + if_shift;
        if (*lo < 50) *lo = 50;
    }
}
static void filter_apply(void)
{
    if (cur_mode == FPGA_MODE_AM) return;          // AM does not use the filter
    float lo, hi;
    audio_band(&lo, &hi);
    fpga_set_filter(-hi, -lo);                     // the FPGA filter is always on the LSB side
}
// header + passband shading
static void mode_show(void)
{
    float lo, hi, plo, phi;
    const char *bw = "";
    audio_band(&lo, &hi);
    if (cur_mode == FPGA_MODE_AM)       { plo = -6000; phi = 6000; }   // ssb_demod's decimator, fc 6kHz
    else if (cur_mode == MODE_CW)       { plo = lo - CW_PITCH; phi = hi - CW_PITCH; bw = cw_bw[cw_bw_idx].name; }
    else if (cur_mode == FPGA_MODE_USB) { plo = lo; phi = hi; bw = ssb_bw[ssb_bw_idx].name; }
    else                                { plo = -hi; phi = -lo; bw = ssb_bw[ssb_bw_idx].name; }
    spectrum_set_mode(mode_name[cur_mode], bw, mode_sel == MODE_AUTO, plo, phi);
    spectrum_set_dial(dial_hz, step_idx, cur_mode == MODE_CW ? -CW_PITCH : 0);
}

// Band plan for AUTO (IARU region 1): AM on the LW/MW and shortwave
// broadcast bands and CB, else the amateur convention - LSB below 10MHz,
// USB above (and on 60m).
static int band_mode(uint32_t hz)
{
    static const uint32_t am[][2] = {
        {   148500,  1705000 },   // LW + MW
        {  2300000,  2495000 }, {  3200000,  3400000 }, {  3900000,  4000000 },
        {  4750000,  5060000 }, {  5900000,  6200000 }, {  7200000,  7450000 },
        {  9400000,  9900000 }, { 11600000, 12100000 }, { 13570000, 13870000 },
        { 15100000, 15800000 }, { 17480000, 17900000 }, { 18900000, 19020000 },
        { 21450000, 21850000 }, { 25670000, 26100000 },
        { 26965000, 27405000 },   // CB
    };
    for (int i = 0; i < sizeof(am) / sizeof(am[0]); i++)
        if (hz >= am[i][0] && hz <= am[i][1]) return FPGA_MODE_AM;
    if (hz >= 5351500 && hz <= 5366500) return FPGA_MODE_USB;   // 60m
    return hz < 10000000 ? FPGA_MODE_LSB : FPGA_MODE_USB;
}

// Brings the FPGA (MODE -> LO offset K) and the screen in line with
// mode_sel and the dial. force = send even if unchanged.
static void mode_apply(bool force)
{
    int m = mode_sel == MODE_AUTO ? band_mode(dial_hz) : mode_sel;
    if (m != cur_mode || force) {
        fpga_write(FPGA_REG_MODE, fpga_mode_of(m));
        cur_mode = m;
        lo_shift_k = fpga_mode_k(m);
        filter_apply();
        ESP_LOGI(TAG, "mode %s%s, K=%d", mode_name[m], mode_sel == MODE_AUTO ? " (auto)" : "", lo_shift_k);
    }
    fpga_set_freq(lo_hz());              // CW offset and calibration
    mode_show();
}

static void dial_load(void)
{
    nvs_handle_t h;
    if (nvs_open("fpgarx", NVS_READONLY, &h) != ESP_OK) return;
    uint32_t f;
    uint8_t st;
    if (nvs_get_u32(h, "dial", &f) == ESP_OK && f >= F_MIN && f <= F_MAX) dial_hz = f;
    if (nvs_get_u8(h, "step", &st) == ESP_OK && st >= 1 && st <= 6) step_idx = st;
    if (nvs_get_u8(h, "src", &st) == ESP_OK) src_fpga = st;
    if (nvs_get_u8(h, "mode", &st) == ESP_OK && st <= MODE_CW) mode_sel = st;
    if (nvs_get_u8(h, "ssbbw", &st) == ESP_OK && st < N_SSB_BW) ssb_bw_idx = st;
    if (nvs_get_u8(h, "cwbw", &st) == ESP_OK && st < N_CW_BW) cw_bw_idx = st;
    if (nvs_get_u8(h, "agc", &st) == ESP_OK && st <= FPGA_AGC_SLOW) agc_speed = st;
    if (nvs_get_u8(h, "vol", &st) == ESP_OK && st <= VOL_MAX) vol_idx = st;
    if (nvs_get_u8(h, "sql", &st) == ESP_OK && (st == 0 || (st >= 3 && st <= SQL_MAX))) sql_db = st;
    int16_t i16;
    if (nvs_get_i16(h, "shift", &i16) == ESP_OK && i16 >= -1000 && i16 <= 1000) if_shift = i16;
    if (nvs_get_i16(h, "ppm10", &i16) == ESP_OK && i16 >= -1000 && i16 <= 1000) ppm_x10 = i16;
    if (nvs_get_u8(h, "view", &st) == ESP_OK && st < SPEC_VIEW_N) view_sel = st;
    if (nvs_get_u8(h, "split", &st) == ESP_OK && st < sizeof(split_spec)) split_idx = st;
    if (nvs_get_u8(h, "wfspd", &st) == ESP_OK && st < sizeof(wf_div)) wfspd_idx = st;
    if (nvs_get_u8(h, "range", &st) == ESP_OK && st < sizeof(range_db)) range_idx = st;
    if (nvs_get_u8(h, "win", &st) == ESP_OK && st < SPEC_WIN_N) win_sel = st;
    if (nvs_get_u8(h, "fps", &st) == ESP_OK) fps_on = st;
    if (nvs_get_u8(h, "smooth", &st) == ESP_OK) smooth_on = st;
    if (nvs_get_u8(h, "rtscr", &st) == ESP_OK) scroll_on = st;
    if (nvs_get_u8(h, "nrm", &st) == ESP_OK && st <= FPGA_NR_ANF) nr_mode = st;
    if (nvs_get_u8(h, "nrl3", &st) == ESP_OK && st <= 3) nr_level = st;
    nvs_close(h);
}

static void dial_save(void)
{
    nvs_handle_t h;
    if (nvs_open("fpgarx", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "dial", dial_hz);
    nvs_set_u8(h, "step", step_idx);
    nvs_set_u8(h, "src", src_fpga);
    nvs_set_u8(h, "mode", mode_sel);
    nvs_set_u8(h, "ssbbw", ssb_bw_idx);
    nvs_set_u8(h, "cwbw", cw_bw_idx);
    nvs_set_u8(h, "agc", agc_speed);
    nvs_set_u8(h, "vol", vol_idx);
    nvs_set_u8(h, "sql", sql_db);
    nvs_set_i16(h, "shift", if_shift);
    nvs_set_i16(h, "ppm10", ppm_x10);
    nvs_set_u8(h, "view", view_sel);
    nvs_set_u8(h, "split", split_idx);
    nvs_set_u8(h, "wfspd", wfspd_idx);
    nvs_set_u8(h, "range", range_idx);
    nvs_set_u8(h, "win", win_sel);
    nvs_set_u8(h, "fps", fps_on);
    nvs_set_u8(h, "smooth", smooth_on);
    nvs_set_u8(h, "rtscr", scroll_on);
    nvs_set_u8(h, "nrm", nr_mode);
    nvs_set_u8(h, "nrl3", nr_level);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "saved dial %lu Hz, step %lu Hz", (unsigned long)dial_hz, (unsigned long)step_hz[step_idx]);
}

static void dial_apply(void)
{
    mode_apply(false);                   // sends the LO too
    nvs_dirty = true;
    t_last_tune = esp_timer_get_time();
}

static void dial_tune(int dir)
{
    int64_t f = (int64_t)dial_hz + dir * (int64_t)step_hz[step_idx];
    if (f < F_MIN) f = F_MIN;
    if (f > F_MAX) f = F_MAX;
    dial_hz = f;
    dial_apply();
}

typedef enum { ST_MAIN, ST_MENU, ST_TEST_SPI, ST_TEST_I2S, ST_TEST_NOISE } state_t;
static state_t state = ST_MAIN;
static const menu_t *cur_menu = &menu_main;
static int cursor;
static bool editing;        // menu: the encoder changes the selected item's value
static void fpga_sync_all(void);
static bool vol_mode;       // main screen: the encoder sets the volume
static bool sql_mode;       // main screen: the encoder sets the squelch threshold
static void sql_mode_set(bool on)
{
    sql_mode = on;
    spectrum_show_squelch(on ? sql_db : -1);
}
static void vol_mode_set(bool on)
{
    vol_mode = on;
    spectrum_show_volume(on ? vol_idx : -1);
}

// Items whose value the encoder edits: ACT_SHIFT, ACT_VOLUME, ACT_PPM
// (SELECT enters / leaves editing)
static void edit_adjust(action_t a, int d)
{
    if (a == ACT_VOLUME) {
        vol_idx += d;
        if (vol_idx < 0) vol_idx = 0;
        if (vol_idx > VOL_MAX) vol_idx = VOL_MAX;
        fpga_write(FPGA_REG_VOL, vol_reg());
    } else if (a == ACT_SQL) {
        // OFF, 3..30 dB (one step from OFF lands on 3)
        int v = sql_db + d;
        if (sql_db == 0 && d > 0) v = 2 + d;
        if (v < 3) v = 0;
        if (v > SQL_MAX) v = SQL_MAX;
        sql_db = v;
    } else if (a == ACT_SHIFT) {
        if_shift += 50 * d;
        if (if_shift < -1000) if_shift = -1000;
        if (if_shift > 1000) if_shift = 1000;
        filter_apply();
    } else if (a == ACT_PPM) {
        ppm_x10 += d;
        if (ppm_x10 < -1000) ppm_x10 = -1000;
        if (ppm_x10 > 1000) ppm_x10 = 1000;
        fpga_set_freq(lo_hz());
    }
}

// Squelch: VOL to 0 / back in a few steps (~10ms) instead of a click.
static void sq_set_muted(bool m)
{
    if (m == sq_muted) return;
    sq_muted = m;
    spectrum_set_squelch_closed(m);
    if (!fpga_present) return;
    uint8_t v = vol_user_reg();
    static const uint8_t close_q[] = { 3, 2, 1, 0 }, open_q[] = { 2, 4 };   // quarters
    const uint8_t *q = m ? close_q : open_q;
    int n = m ? 4 : 2;
    for (int i = 0; i < n; i++) {
        fpga_write(FPGA_REG_VOL, v * q[i] / 4);
        if (i < n - 1) vTaskDelay(pdMS_TO_TICKS(m ? 4 : 2));
    }
}

// Once per spectrum frame (main screen). Elsewhere (menu, tests) the
// squelch keeps its state.
static bool squelch_update(int64_t now)   // true = there was a new frame
{
    static int64_t t_above;
    float snr;
    if (!spectrum_sq_snr(&snr, NULL, NULL)) return false;
    if (sql_db == 0) { sq_set_muted(false); return true; }
    float close_at = sql_db - SQ_HYST;
    if (close_at < SQ_CLOSE_MIN) close_at = SQ_CLOSE_MIN;
    if (snr >= sql_db) { t_above = now; sq_set_muted(false); }
    else if (!sq_muted) {
        if (snr >= close_at) t_above = now;                  // hysteresis band: stay open
        else if (now - t_above > SQ_HANG_US) sq_set_muted(true);
    }
    return true;
}

// Tang buttons S1/S2 (sdr_esp buttons.v): the FPGA counts short/long
// presses (2-bit counters), polled every 30ms. The dial and the menu
// cursor are on the encoder only (2026-11-22; no ESP buttons are wired).
//   main screen: S1 short = step (10Hz..1MHz round)  S1 long = volume (encoder, 3s)
//                S2 short = squelch (encoder, 5s)     S2 long = menu
//                in volume / squelch: S1 short = done
//   elsewhere:   S1 short / long = SELECT            S2 short = BACK
//                S2 long = exit to the main screen
static input_ev_t tang_buttons(int64_t now)
{
    static int64_t t_poll;
    static uint8_t last;
    static bool have;
    static input_ev_t pend[12];
    static int np;
    if (np) return pend[--np];
    if (now - t_poll < 30000) return EV_NONE;
    t_poll = now;
    uint8_t b = fpga_read(FPGA_REG_BTN);
    if (!have) { last = b; have = true; return EV_NONE; }
    // {S1 short, S1 long, S2 short, S2 long}; main: SELECT = step, BACK = volume
    static const input_ev_t map_main[4] = { EV_SELECT, EV_BACK, EV_SQL, EV_HOLD };
    static const input_ev_t map_menu[4] = { EV_SELECT, EV_SELECT, EV_BACK, EV_EXIT };
    const input_ev_t *map = state == ST_MAIN ? map_main : map_menu;
    for (int i = 3; i >= 0; i--)
        for (int d = ((b >> (2 * i)) - (last >> (2 * i))) & 3; d > 0; d--) pend[np++] = map[i];
    last = b;
    return np ? pend[--np] : EV_NONE;
}

static void draw_title(const char *title)
{
    lcd_fill(0, AREA_Y, LCD_W, ROW_H, C_TITLE_BG);
    lcd_text(8, AREA_Y + 3, &font_mid, title, C_TEXT, C_TITLE_BG);
}

static void draw_menu(void)
{
    draw_title(cur_menu->title);
    lcd_fill(0, AREA_Y + ROW_H, LCD_W, AREA_H - ROW_H, C_PANEL);
    for (int i = 0; i < cur_menu->n; i++) {
        const item_t *it = &cur_menu->items[i];
        char s[32];
        if (it->act == ACT_SOURCE)
            snprintf(s, sizeof(s), "%s: %s", it->label, !src_fpga ? "SIM" : fpga_present ? "FPGA" : "FPGA (absent: SIM)");
        else if (it->act == ACT_MODE && mode_sel == MODE_AUTO)
            snprintf(s, sizeof(s), "%s: AUTO (%s)", it->label, mode_name[cur_mode < 0 ? 0 : cur_mode]);
        else if (it->act == ACT_VIEW && view_sel == SPEC_VIEW_BOTH)
            snprintf(s, sizeof(s), "%s: BOTH %d/%d", it->label, split_spec[split_idx], 254 - split_spec[split_idx]);
        else if (it->act == ACT_VIEW) snprintf(s, sizeof(s), "%s: %s", it->label, view_name[view_sel]);
        else if (it->act == ACT_WFSPD) snprintf(s, sizeof(s), "%s: 1/%d", it->label, wf_div[wfspd_idx]);
        else if (it->act == ACT_RANGE) snprintf(s, sizeof(s), "%s: %d dB", it->label, range_db[range_idx]);
        else if (it->act == ACT_NR) snprintf(s, sizeof(s), "%s: %s", it->label, nr_name[nr_mode]);
        else if (it->act == ACT_NR_LEVEL) snprintf(s, sizeof(s), "%s: %d", it->label, nr_level + 1);
        else if (it->act == ACT_SMOOTH) snprintf(s, sizeof(s), "%s: %s", it->label, smooth_on ? "ON" : "OFF");
        else if (it->act == ACT_SCROLL) snprintf(s, sizeof(s), "%s: %s", it->label, scroll_on ? "SCROLL" : "STATIC");
        else if (it->act == ACT_FPS) snprintf(s, sizeof(s), "%s: %s", it->label, fps_on ? "ON" : "OFF");
        else if (it->act == ACT_WINDOW) snprintf(s, sizeof(s), "%s: %s", it->label, win_name[win_sel]);
        else if (it->act == ACT_MODE) snprintf(s, sizeof(s), "%s: %s", it->label, mode_name[mode_sel]);
        else if (it->act == ACT_FILTER) {
            if (cur_mode == FPGA_MODE_AM) snprintf(s, sizeof(s), "%s: AM (fixed)", it->label);
            else if (cur_mode == MODE_CW) snprintf(s, sizeof(s), "%s: CW %s", it->label, cw_bw[cw_bw_idx].name);
            else snprintf(s, sizeof(s), "%s: SSB %s", it->label, ssb_bw[ssb_bw_idx].name);
        }
        else if (it->act == ACT_SHIFT) snprintf(s, sizeof(s), "%s: %+d Hz%s", it->label, if_shift,
                                                (cur_mode == MODE_CW || cur_mode == FPGA_MODE_AM) ? " (SSB only)" : "");
        else if (it->act == ACT_AGC) snprintf(s, sizeof(s), "%s: %s", it->label, agc_name[agc_speed]);
        else if (it->act == ACT_VOLUME) {
            if (vol_idx == 0) snprintf(s, sizeof(s), "%s: MUTE", it->label);
            else snprintf(s, sizeof(s), "%s: %d (%+d dB)", it->label, vol_idx, (vol_idx - 20) * 2);
        }
        else if (it->act == ACT_SQL) {
            if (sql_db == 0) snprintf(s, sizeof(s), "%s: OFF", it->label);
            else snprintf(s, sizeof(s), "%s: %d dB over noise", it->label, sql_db);
        }
        else if (it->act == ACT_PPM) snprintf(s, sizeof(s), "%s: %+.1f ppm", it->label, ppm_x10 / 10.0f);
        else snprintf(s, sizeof(s), "%s%s", it->label, it->act == ACT_SUBMENU ? " >" : "");
        bool sel = (i == cursor);
        // 30 px pitch; 8 rows squeeze to 27 so the last one clears the hint line
        int pitch = cur_menu->n > 7 ? ROW_H + 1 : ROW_H + 4;
        int y = AREA_Y + ROW_H + 8 + i * pitch;
        uint16_t bg = sel ? (editing ? C_EDIT_BG : C_SEL_BG) : C_PANEL;
        lcd_fill(12, y, LCD_W - 24, ROW_H, bg);
        lcd_text(24, y + 3, &font_mid, s, sel ? C_SEL_FG : C_TEXT, bg);
    }
    lcd_text(8, LCD_H - 16, &font_small, editing ? "turn = change value, S1 / S2 = done           "
                                                  : "turn = move, S1 = select, S2 = back (hold: exit)", C_DIM, C_PANEL);
}

// A test screen line: label in the left column, value in the right column.
static void test_line(int row, const char *label, const char *val, uint16_t color)
{
    int y = AREA_Y + ROW_H + 8 + row * TEST_ROW_H;
    lcd_text(12, y, &font_mid, label, C_DIM, C_PANEL);
    char v[24];
    snprintf(v, sizeof(v), "%-16s", val);   // pad, so a shorter value erases the old one
    lcd_text(TEST_VAL_X, y, &font_mid, v, color, C_PANEL);
}

static void test_screen(const char *title)
{
    draw_title(title);
    lcd_fill(0, AREA_Y + ROW_H, LCD_W, AREA_H - ROW_H, C_PANEL);
    lcd_text(8, LCD_H - 16, &font_small, "S2 = back, hold S2 = main screen", C_DIM, C_PANEL);
}

// ---------------------------------------------------------------------------
// SPI link test: ID/VERSION, then endless write/read-back of SCRATCH
// ---------------------------------------------------------------------------
static uint32_t spi_ok, spi_err;
static int64_t spi_t0;

static void spi_test_enter(void)
{
    test_screen("SPI LINK TEST");
    spi_ok = spi_err = 0;
    spi_t0 = esp_timer_get_time();
}

static void spi_test_step(void)
{
    fpga_led_hold(true);
    uint8_t id = fpga_read(FPGA_REG_ID), ver = fpga_read(FPGA_REG_VERSION);
    for (int k = 0; k < 200; k++) {
        uint8_t v = esp_random();
        fpga_write(FPGA_REG_SCRATCH, v);
        if (fpga_read(FPGA_REG_SCRATCH) == v) spi_ok++;
        else spi_err++;
    }
    fpga_led_hold(false);
    float secs = (esp_timer_get_time() - spi_t0) / 1e6f;
    char s[24];
    snprintf(s, sizeof(s), "0x%02X", id);
    test_line(0, "FPGA ID", s, id == FPGA_ID ? C_OK : C_FAIL);
    snprintf(s, sizeof(s), "%u", ver);
    test_line(1, "Version", s, C_TEXT);
    snprintf(s, sizeof(s), "%lu", (unsigned long)spi_ok);
    test_line(2, "Write+read OK", s, C_TEXT);
    snprintf(s, sizeof(s), "%lu", (unsigned long)spi_err);
    test_line(3, "Errors", s, spi_err ? C_FAIL : C_OK);
    snprintf(s, sizeof(s), "%.0f /s", (spi_ok + spi_err) / secs);
    test_line(4, "Rate", s, C_TEXT);
    bool pass = id == FPGA_ID && spi_err == 0 && spi_ok > 0;
    test_line(6, "Result", pass ? "PASS" : "FAIL", pass ? C_OK : C_FAIL);
}

// ---------------------------------------------------------------------------
// I2S link test: FPGA sends a counter pattern, the receive task checks it
// ---------------------------------------------------------------------------
static void i2s_test_enter(void)
{
    test_screen("I2S LINK TEST");
    fpga_write(FPGA_REG_CTRL, ctrl_base() | FPGA_CTRL_I2S_TEST);
    fpga_i2s_test_mode(true);
    vTaskDelay(pdMS_TO_TICKS(100));     // let the old blocks drain
    fpga_i2s_stats_reset();
}

static void i2s_test_leave(void)
{
    fpga_write(FPGA_REG_CTRL, ctrl_base());
    fpga_i2s_test_mode(false);
}

static bool i2s_rate_ok(float rate) { return rate > FPGA_FS_HZ * 0.99f && rate < FPGA_FS_HZ * 1.01f; }

static void i2s_test_step(void)
{
    fpga_i2s_stats_t s;
    fpga_i2s_stats(&s);
    float rate = fpga_i2s_rate(&s);
    char v[24];
    snprintf(v, sizeof(v), "%.0f Hz", rate);
    test_line(0, "Frame rate", v, i2s_rate_ok(rate) ? C_OK : C_FAIL);
    snprintf(v, sizeof(v), "%lu", (unsigned long)s.frames);
    test_line(1, "Frames", v, C_TEXT);
    snprintf(v, sizeof(v), "%lu", (unsigned long)s.pattern_err);
    test_line(2, "Pattern errors", v, s.pattern_err ? C_FAIL : C_OK);
    snprintf(v, sizeof(v), "%lu", (unsigned long)s.count_jumps);
    test_line(3, "Counter jumps", v, s.count_jumps ? C_FAIL : C_OK);
    snprintf(v, sizeof(v), "%08lX", (unsigned long)s.last_left);
    test_line(4, "Last L", v, C_TEXT);
    snprintf(v, sizeof(v), "%08lX", (unsigned long)s.last_right);
    test_line(5, "Last R", v, C_TEXT);
    bool pass = i2s_rate_ok(rate) && s.frames > 0 && !s.pattern_err && !s.count_jumps;
    test_line(6, "Result", pass ? "PASS" : "FAIL", pass ? C_OK : C_FAIL);
}

// ---------------------------------------------------------------------------
// Noise test (noise.c): band noise vs receiver noise, no PC needed.
// Two slots kept in NVS: "50 ohm" (load on the antenna input, BPF in the
// chain) and "Antenna". Turn / NEXT / PREV = pick the slot, SELECT = store
// the current 2s value in it, BACK or hold = leave. Console "nz" prints them.
// ---------------------------------------------------------------------------
static int nz_slot;                  // 0 = 50 ohm, 1 = antenna
static int16_t nz_val[2] = { INT16_MIN, INT16_MIN };   // dB/Hz x10, MIN = empty
static uint32_t nz_hz[2];
static float nz_live;
static int nz_count;                 // finished 2s measurements since entering
static uint8_t nz_peak;              // max ADC PEAK since the last result
static const char *const nz_key_v[2] = { "nz_load", "nz_ant" }, *const nz_key_f[2] = { "nz_lf", "nz_af" };

static void nz_load(void)
{
    nvs_handle_t h;
    if (nvs_open("fpgarx", NVS_READONLY, &h) != ESP_OK) return;
    for (int i = 0; i < 2; i++) {
        nvs_get_i16(h, nz_key_v[i], &nz_val[i]);
        nvs_get_u32(h, nz_key_f[i], &nz_hz[i]);
    }
    nvs_close(h);
}

static void nz_store(int i)
{
    nz_val[i] = (int16_t)lroundf(nz_live * 10.0f);
    nz_hz[i] = dial_hz;
    nvs_handle_t h;
    if (nvs_open("fpgarx", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i16(h, nz_key_v[i], nz_val[i]);
    nvs_set_u32(h, nz_key_f[i], nz_hz[i]);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "noise %s: %.1f dBFS/Hz at %lu Hz", i ? "antenna" : "50 ohm", nz_val[i] / 10.0f,
             (unsigned long)nz_hz[i]);
}

static void nz_print(void)
{
    for (int i = 0; i < 2; i++) {
        if (nz_val[i] == INT16_MIN) printf("NZ %s: -\n", i ? "antenna" : "load");
        else printf("NZ %s: %.1f dBFS/Hz at %lu Hz\n", i ? "antenna" : "load", nz_val[i] / 10.0f,
                    (unsigned long)nz_hz[i]);
    }
    if (nz_val[0] != INT16_MIN && nz_val[1] != INT16_MIN)
        printf("NZ antenna - load: %+.1f dB\n", (nz_val[1] - nz_val[0]) / 10.0f);
}

static void nz_draw(void)
{
    char v[24];
    snprintf(v, sizeof(v), "%.1f kHz", dial_hz / 1000.0f);
    test_line(0, "Dial", v, C_TEXT);
    float pk = 20.0f * log10f((nz_peak + 0.5f) / 256.0f);
    if (nz_peak >= FPGA_PEAK_CLIP) snprintf(v, sizeof(v), "CLIP");
    else snprintf(v, sizeof(v), "%.0f dBFS", pk);
    test_line(1, "ADC peak", v, nz_peak >= FPGA_PEAK_CLIP ? C_FAIL : C_TEXT);
    if (nz_count == 0) snprintf(v, sizeof(v), "measuring...");
    else snprintf(v, sizeof(v), "%.1f  #%d", nz_live, nz_count);
    test_line(2, "Now, dBFS/Hz", v, C_OK);
    static const char *const lbl[2][2] = { { "  50 ohm load", "> 50 ohm load" }, { "  Antenna", "> Antenna" } };
    for (int i = 0; i < 2; i++) {
        if (nz_val[i] == INT16_MIN) snprintf(v, sizeof(v), "-");
        else snprintf(v, sizeof(v), "%.1f @%luk", nz_val[i] / 10.0f, (unsigned long)(nz_hz[i] / 1000));
        test_line(3 + i, lbl[i][nz_slot == i], v, nz_slot == i ? C_SEL_BG : C_TEXT);
    }
    const char *verdict = "measure both slots";
    uint16_t col = C_DIM;
    if (nz_val[0] != INT16_MIN && nz_val[1] != INT16_MIN) {
        float d = (nz_val[1] - nz_val[0]) / 10.0f;
        snprintf(v, sizeof(v), "%+.1f dB", d);
        test_line(5, "Antenna - load", v, d >= 10 ? C_OK : d >= 3 ? C_SEL_BG : C_FAIL);
        if (d >= 10)     { verdict = "band noise >> receiver: a better ADC adds no sensitivity"; col = C_OK; }
        else if (d >= 3) { verdict = "receiver noise close to the band: some room to improve "; col = C_SEL_BG; }
        else             { verdict = "receiver limits (or no antenna): ADC / LNA would help  "; col = C_FAIL; }
    } else test_line(5, "Antenna - load", "-", C_DIM);
    char line[64];
    snprintf(line, sizeof(line), "%-58s", verdict);
    lcd_text(12, AREA_Y + ROW_H + 8 + 6 * TEST_ROW_H + 6, &font_small, line, col, C_PANEL);
}

static void noise_test_enter(void)
{
    test_screen("NOISE TEST");
    lcd_text(8, LCD_H - 16, &font_small, "turn = slot, S1 = save value, S2 = leave      ", C_DIM, C_PANEL);
    noise_reset();
    nz_count = 0;
    nz_peak = 0;
    if (!fpga_present) {
        test_line(2, "Now, dBFS/Hz", "no FPGA", C_FAIL);
        return;
    }
    nz_draw();
}

// ---------------------------------------------------------------------------
// Power-up self-test (serial log only)
// ---------------------------------------------------------------------------
static bool link_selftest(void)   // true = all PASS
{
    fpga_led_hold(true);
    uint8_t id = fpga_read(FPGA_REG_ID), ver = fpga_read(FPGA_REG_VERSION);
    int err = 0;
    for (int k = 0; k < 1000; k++) {
        uint8_t v = esp_random();
        fpga_write(FPGA_REG_SCRATCH, v);
        if (fpga_read(FPGA_REG_SCRATCH) != v) err++;
    }
    ESP_LOGI(TAG, "selftest SPI: id=0x%02X ver=%u scratch errors=%d/1000 -> %s",
             id, ver, err, (id == FPGA_ID && !err) ? "PASS" : "FAIL");

    // LO retune: set a few dials, check FREQ read-back and the phase
    // increment the FPGA computed (freq_to_phase_inc takes ~2us)
    lo_shift_k = (int8_t)fpga_read(FPGA_REG_LO_SHIFT);
    static const uint32_t probe[] = { 100000, 7074000, 14250000, 29999990 };
    int lo_err = 0;
    for (int i = 0; i < 4; i++) {
        fpga_set_freq(probe[i]);
        vTaskDelay(1);
        uint32_t f = fpga_get_freq(), fw = fpga_read_u32(FPGA_REG_LO_FW);
        uint32_t want = fpga_expected_lo_fw(probe[i], lo_shift_k);
        int d = (int)(fw - want);
        if (f != probe[i] || d < -1 || d > 1) {
            lo_err++;
            ESP_LOGW(TAG, "  LO %lu Hz: read %lu Hz, fw %08lX expected %08lX",
                     (unsigned long)probe[i], (unsigned long)f, (unsigned long)fw, (unsigned long)want);
        }
    }
    ESP_LOGI(TAG, "selftest LO: K=%d, 4 dials, errors %d -> %s", lo_shift_k, lo_err, lo_err ? "FAIL" : "PASS");

    // demod modes: each must switch the LO offset K (and the phase increment)
    int mode_err = 0;
    for (int m = FPGA_MODE_LSB; m <= FPGA_MODE_AM; m++) {
        fpga_write(FPGA_REG_MODE, m);
        fpga_set_freq(7074000);
        vTaskDelay(1);
        int k = (int8_t)fpga_read(FPGA_REG_LO_SHIFT);
        uint32_t fw = fpga_read_u32(FPGA_REG_LO_FW), want = fpga_expected_lo_fw(7074000, fpga_mode_k(m));
        int d = (int)(fw - want);
        bool ok = fpga_read(FPGA_REG_MODE) == m && k == fpga_mode_k(m) && d >= -1 && d <= 1;
        if (!ok) mode_err++;
        ESP_LOGI(TAG, "  mode %s: K=%d fw %08lX expected %08lX%s", mode_name[m], k,
                 (unsigned long)fw, (unsigned long)want, ok ? "" : "  <- wrong");
    }
    fpga_write(FPGA_REG_MODE, FPGA_MODE_LSB);
    ESP_LOGI(TAG, "selftest modes: errors %d -> %s", mode_err, mode_err ? "FAIL" : "PASS");

    fpga_write(FPGA_REG_CTRL, ctrl_base() | FPGA_CTRL_I2S_TEST);
    fpga_i2s_test_mode(true);
    vTaskDelay(pdMS_TO_TICKS(100));
    fpga_i2s_stats_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
    fpga_i2s_stats_t s;
    fpga_i2s_stats(&s);
    float rate = fpga_i2s_rate(&s);
    fpga_write(FPGA_REG_CTRL, ctrl_base());
    fpga_i2s_test_mode(false);
    ESP_LOGI(TAG, "selftest I2S: %.0f Hz, %lu frames, pattern errors %lu, counter jumps %lu, "
             "last L=%08lX R=%08lX -> %s", rate, (unsigned long)s.frames,
             (unsigned long)s.pattern_err, (unsigned long)s.count_jumps,
             (unsigned long)s.last_left, (unsigned long)s.last_right,
             (i2s_rate_ok(rate) && !s.pattern_err && !s.count_jumps) ? "PASS" : "FAIL");
    fpga_led_hold(false);
    return id == FPGA_ID && !err && !lo_err && !mode_err &&
           i2s_rate_ok(rate) && !s.pattern_err && !s.count_jumps;
}

// The board's RGB LED is on GPIO48 (checked 2026-10-19); keep it dark.
static void rgb_led_off(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = 48,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt = { .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000 };
    led_strip_handle_t strip;
    if (led_strip_new_rmt_device(&cfg, &rmt, &strip) == ESP_OK) {
        led_strip_clear(strip);
        led_strip_del(strip);
    }
}

// ---------------------------------------------------------------------------
static void enter_main(void)
{
    state = ST_MAIN;
    lcd_fill(0, AREA_Y, LCD_W, AREA_H, C_BG);
    spectrum_draw_static();
}

static void enter_menu(const menu_t *m, int pos)
{
    state = ST_MENU;
    editing = false;
    cur_menu = m;
    cursor = pos;
    draw_menu();
}

static void menu_select(void)
{
    const item_t *it = &cur_menu->items[cursor];
    switch (it->act) {
    case ACT_SUBMENU:  enter_menu(it->sub, 0); break;
    case ACT_BACK:     enter_menu(cur_menu->parent, 0); break;
    case ACT_EXIT:     enter_main(); break;
    case ACT_MODE:     // AUTO -> LSB -> USB -> AM -> AUTO
        {   // AUTO -> LSB -> USB -> CW -> AM -> AUTO
            static const int next[] = { FPGA_MODE_USB, MODE_CW, MODE_AUTO, FPGA_MODE_LSB, FPGA_MODE_AM };
            mode_sel = next[mode_sel];
        }
        mode_apply(false);   // redraws the ruler under the menu title,
        dial_save();
        draw_menu();         // so the menu goes back on top
        break;
    case ACT_VIEW:     // SPECTRUM -> WATERFALL -> BOTH 160 -> 140 -> 120 -> SPECTRUM
        if (view_sel != SPEC_VIEW_BOTH) { view_sel++; split_idx = 0; }
        else if (++split_idx >= (int)sizeof(split_spec)) { view_sel = SPEC_VIEW_SPECTRUM; split_idx = 0; }
        spectrum_set_layout(view_sel, split_spec[split_idx]);   // seen on Exit (enter_main clears the panel)
        dial_save();
        draw_menu();
        break;
    case ACT_WFSPD:    // 1/1 -> 1/2 -> 1/3 -> 1/1
        wfspd_idx = (wfspd_idx + 1) % sizeof(wf_div);
        spectrum_set_wf_speed(wf_div[wfspd_idx]);
        dial_save();
        draw_menu();
        break;
    case ACT_RANGE:    // 20 -> 30 -> 40 -> 60 -> 20 dB
        range_idx = (range_idx + 1) % sizeof(range_db);
        spectrum_set_view(range_db[range_idx], win_sel);
        dial_save();
        draw_menu();
        break;
    case ACT_WINDOW:
        win_sel = (win_sel + 1) % SPEC_WIN_N;   // OFF -> HANN -> BH -> OFF
        spectrum_set_view(range_db[range_idx], win_sel);
        dial_save();
        draw_menu();
        break;
    case ACT_NR:          // OFF -> NR -> ANF -> OFF
        nr_mode = (nr_mode + 1) % 3;
        fpga_write(FPGA_REG_NR, nr_reg());
        dial_save();
        draw_menu();
        break;
    case ACT_NR_LEVEL:    // 1 -> 2 -> 3 -> 4 -> 1
        nr_level = (nr_level + 1) % 4;
        fpga_write(FPGA_REG_NR, nr_reg());
        dial_save();
        draw_menu();
        break;
    case ACT_SMOOTH:
        smooth_on = !smooth_on;
        spectrum_set_smooth(smooth_on);
        dial_save();
        draw_menu();
        break;
    case ACT_SCROLL:
        scroll_on = !scroll_on;
        spectrum_set_scroll(scroll_on);
        dial_save();
        draw_menu();
        break;
    case ACT_FPS:
        fps_on = !fps_on;
        spectrum_show_fps(fps_on);
        dial_save();
        draw_menu();
        break;
    case ACT_FILTER:      // cycles the widths of the current mode
        if (cur_mode == MODE_CW) cw_bw_idx = (cw_bw_idx + 1) % N_CW_BW;
        else if (cur_mode != FPGA_MODE_AM) ssb_bw_idx = (ssb_bw_idx + 1) % N_SSB_BW;
        filter_apply();
        mode_show();         // header + passband (draws the ruler under the menu title)
        dial_save();
        draw_menu();
        break;
    case ACT_AGC:         // FAST -> MED -> SLOW
        agc_speed = (agc_speed + 1) % 3;
        fpga_write(FPGA_REG_AGC, agc_speed);
        dial_save();
        draw_menu();
        break;
    case ACT_SHIFT:
    case ACT_VOLUME:
    case ACT_SQL:
    case ACT_PPM:
        editing = !editing;
        if (!editing) { mode_show(); dial_save(); }
        draw_menu();
        break;
    case ACT_SOURCE:   src_fpga = !src_fpga; spectrum_floor_reset(); dial_save(); draw_menu(); break;
    case ACT_TEST_SPI: state = ST_TEST_SPI; spi_test_enter(); break;
    case ACT_TEST_I2S: state = ST_TEST_I2S; i2s_test_enter(); break;
    case ACT_TEST_NOISE: state = ST_TEST_NOISE; noise_test_enter(); break;
    default: break;
    }
}

// Debug console over the COM port (UART0 via the CH343, same port as
// flashing and the log; the native USB is gone - GPIO20 is the FPGA SPI CS):
//   f <Hz>       tune the dial
//   s <frames>   average the spectrum, print it as one "SPEC ..." line
//   lcd m <hex>  ST7796 MADCTL (orientation, RGB/BGR), lcd i 0|1 inversion,
//   lcd          print both - for the display bring-up
// Lets a PC sweep the dial and look at the spectrum (spur hunting).
static void console_poll(void)
{
    static char line[48];
    static int len;
    uint8_t c;
    while (uart_read_bytes(UART_NUM_0, &c, 1, 0) == 1) {
        if (c != '\n' && c != '\r') {
            if (len < (int)sizeof(line) - 1) line[len++] = c;
            continue;
        }
        line[len] = 0;
        len = 0;
        long v = 0;
        if (sscanf(line, "f %ld", &v) == 1 && v >= F_MIN && v <= F_MAX) {
            dial_hz = v;
            dial_apply();
            printf("OK f %lu\n", (unsigned long)dial_hz);
        } else if (strncmp(line, "sm", 2) == 0 && (line[2] == 0 || line[2] == ' ')) {   // "sm [frames]", default 64 (~2 s)
            if (sscanf(line, "sm %ld", &v) != 1 || v < 1 || v > 10000) v = 64;
            spectrum_sm_start(v);
        } else if (sscanf(line, "s %ld", &v) == 1 && v > 0 && v <= 1000) {
            spectrum_capture_start(v);
        } else if (sscanf(line, "lcd m %lx", &v) == 1 && v >= 0 && v <= 0xFF) {
            lcd_set_madctl(v);
            enter_main();
            printf("OK lcd m %02X\n", lcd_get_madctl());
        } else if (sscanf(line, "lcd i %ld", &v) == 1) {
            lcd_set_invert(v != 0);
            printf("OK lcd i %d\n", lcd_get_invert());
        } else if (strcmp(line, "defaults") == 0) {   // menu settings -> defaults, dial/step kept
            src_fpga = true; mode_sel = MODE_AUTO; range_idx = 2; view_sel = SPEC_VIEW_SPECTRUM;
            ssb_bw_idx = 1; cw_bw_idx = 1; if_shift = 0; agc_speed = FPGA_AGC_MED; vol_idx = 20; sql_db = 0;   // ppm kept
            split_idx = 0; wfspd_idx = 0; spectrum_set_wf_speed(wf_div[wfspd_idx]);
            spectrum_set_layout(view_sel, split_spec[split_idx]); win_sel = SPEC_WIN_OFF; fps_on = false; smooth_on = true;
            spectrum_set_smooth(smooth_on);
            scroll_on = true; spectrum_set_scroll(scroll_on);
            nr_mode = FPGA_NR_OFF; nr_level = 1;
            spectrum_set_view(range_db[range_idx], win_sel);
            spectrum_show_fps(fps_on);
            if (fpga_present) fpga_sync_all();
            else mode_apply(true);
            dial_save();
            enter_main();
            printf("OK defaults\n");
        } else if (strcmp(line, "sq") == 0) {   // squelch: passband level, noise, SNR, state
            float snr, sig, nz;
            spectrum_sq_snr(&snr, &sig, &nz);
            printf("sq thr %d snr %.1f sig %.1f noise %.1f %s\n", sql_db, snr, sig, nz,
                   sql_db == 0 ? "OFF" : sq_muted ? "CLOSED" : "OPEN");
        } else if (strcmp(line, "nz") == 0) {
            nz_print();
        } else if (strcmp(line, "lcd") == 0) {
            printf("lcd m %02X i %d\n", lcd_get_madctl(), lcd_get_invert());
        } else if (line[0]) printf("ERR %s\n", line);
    }
}

// Registers the FPGA forgets on a restart (reconfiguration from flash).
// SCRATCH holds SYNC_MARK while they are in place; the link test overwrites
// it, so after the test everything is simply sent again.
#define SYNC_MARK  0xC5
static void fpga_sync_all(void)
{
    fpga_led_hold(true);
    fpga_write(FPGA_REG_CTRL, ctrl_base());
    fpga_write(FPGA_REG_NR, nr_reg());
    fpga_write(FPGA_REG_VOL, vol_reg());
    fpga_write(FPGA_REG_AGC, agc_speed);
    mode_apply(true);                    // MODE, filter taps, LO
    fpga_write(FPGA_REG_SCRATCH, SYNC_MARK);
    fpga_led_hold(false);
}

void app_main(void)
{
    static float iq[2 * SPEC_FFT_N];

    rgb_led_off();
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    dial_load();
    nz_load();
    lcd_init();
    splash_show();                     // Jessica, while the link self-test runs
    lcd_backlight(100);
    int64_t t_splash = esp_timer_get_time();
    spectrum_init();
    spectrum_set_view(range_db[range_idx], win_sel);
    spectrum_set_layout(view_sel, split_spec[split_idx]);
    spectrum_set_wf_speed(wf_div[wfspd_idx]);
    spectrum_set_smooth(smooth_on);
    spectrum_set_scroll(scroll_on);
    fpga_init();
    input_init();
    if (uart_driver_install(UART_NUM_0, 256, 4096, 0, NULL, 0) == ESP_OK) uart_vfs_dev_use_driver(UART_NUM_0);
    uint8_t id = fpga_read(FPGA_REG_ID);
    fpga_present = id == FPGA_ID;
    if (fpga_present) {
        link_selftest();                          // result in the serial log
        uint8_t ver = fpga_read(FPGA_REG_VERSION);
        if (ver < 6) ESP_LOGW(TAG, "FPGA version %u < 6: no VOL/AGC/filter registers - flash sdr_esp", ver);
    } else {
        ESP_LOGW(TAG, "no FPGA (ID read 0x%02X, want 0x%02X) - standalone, SIM source", id, FPGA_ID);
    }
    int64_t shown = esp_timer_get_time() - t_splash;   // keep the splash up >= 2.5s
    if (shown < 2500000) vTaskDelay(pdMS_TO_TICKS((2500000 - shown) / 1000));
    lcd_fill(0, 0, LCD_W, LCD_H, C_BG);
    spectrum_show_fps(fps_on);
    if (fpga_present) fpga_sync_all();       // CTRL, NR, VOL, AGC, mode + filter + LO
    else mode_apply(true);
    enter_main();
    ESP_LOGI(TAG, "dial %lu Hz, step %lu Hz", (unsigned long)dial_hz, (unsigned long)step_hz[step_idx]);

    int64_t t_test = 0, t_check = 0, t_adc = 0, t_vol = 0;
    while (1) {
        input_ev_t ev = input_get();
        int64_t now = esp_timer_get_time();
        if (ev == EV_NONE && fpga_present) ev = tang_buttons(now);
        int enc = input_enc_steps(state == ST_MAIN && !vol_mode && !sql_mode);   // dial or menu steps

        // save the dial 2s after the last change (spares the flash while turning)
        if (nvs_dirty && now - t_last_tune > 2000000) {
            nvs_dirty = false;
            dial_save();
        }
        // once a second: if the FPGA was restarted it lost the dial - resend
        if (fpga_present && now - t_check > 1000000 && state != ST_TEST_SPI) {
            t_check = now;
            fpga_led_hold(true);
            if (state != ST_TEST_I2S && fpga_read(FPGA_REG_SCRATCH) != SYNC_MARK) {
                ESP_LOGW(TAG, "FPGA restarted (or link test ran) - sending all settings");
                fpga_sync_all();
                if (state == ST_MENU) draw_menu();   // the ruler redraw lands under the menu title
            }
            if (fpga_get_freq() != lo_hz()) {
                ESP_LOGW(TAG, "FPGA dial differs (restarted?) - resending %lu Hz", (unsigned long)lo_hz());
                fpga_set_freq(lo_hz());
            }
            if (fpga_read(FPGA_REG_VOL) != vol_reg()) fpga_write(FPGA_REG_VOL, vol_reg());
            if (fpga_read(FPGA_REG_AGC) != agc_speed) fpga_write(FPGA_REG_AGC, agc_speed);
            uint8_t ctrl = ctrl_base() | (state == ST_TEST_I2S ? FPGA_CTRL_I2S_TEST : 0);
            if (fpga_read(FPGA_REG_CTRL) != ctrl) {
                ESP_LOGW(TAG, "FPGA CTRL differs (restarted?) - resending");
                fpga_write(FPGA_REG_CTRL, ctrl);
            }
            if (fpga_read(FPGA_REG_NR) != nr_reg()) {
                ESP_LOGW(TAG, "FPGA NR differs (restarted?) - resending");
                fpga_write(FPGA_REG_NR, nr_reg());
            }
            if (fpga_read(FPGA_REG_MODE) != fpga_mode_of(cur_mode)) {
                ESP_LOGW(TAG, "FPGA mode differs (restarted?) - resending %s", mode_name[cur_mode]);
                fpga_write(FPGA_REG_MODE, fpga_mode_of(cur_mode));
            }
            fpga_led_hold(false);
        }

        switch (state) {
        case ST_MAIN:
            if (ev == EV_HOLD) { vol_mode_set(false); sql_mode_set(false); enter_menu(&menu_main, 0); break; }
            // BACK (Tang S1 long): the encoder sets the volume (header shows it)
            // until BACK again, SELECT (S1 short) or 3s without a change
            if (ev == EV_BACK) { sql_mode_set(false); vol_mode_set(!vol_mode); t_vol = now; }
            // Tang S2 short: the encoder sets the squelch threshold (header:
            // "SQ nn" + live SNR bar, tick = threshold) until S2 short again,
            // SELECT (S1 short), BACK or 5s without a change
            if (ev == EV_SQL) { vol_mode_set(false); sql_mode_set(!sql_mode); t_vol = now; }
            if (sql_mode) {
                int d = enc + (ev == EV_NEXT) - (ev == EV_PREV);
                if (d) {
                    edit_adjust(ACT_SQL, d);
                    nvs_dirty = true;
                    t_last_tune = t_vol = now;
                }
                if (ev == EV_SELECT || now - t_vol > 5000000) { sql_mode_set(false); ev = EV_NONE; }
            } else if (vol_mode) {
                int d = enc + (ev == EV_NEXT) - (ev == EV_PREV);
                if (d) {
                    edit_adjust(ACT_VOLUME, d);
                    spectrum_show_volume(vol_idx);
                    nvs_dirty = true;
                    t_last_tune = t_vol = now;
                }
                if (ev == EV_SELECT || now - t_vol > 3000000) { vol_mode_set(false); ev = EV_NONE; }   // SELECT = done, not a step change
            } else {
                if (ev == EV_NEXT) dial_tune(+1);
                if (ev == EV_PREV) dial_tune(-1);
                if (enc) dial_tune(enc);
            }
            if (ev == EV_SELECT) {
                step_idx = step_idx % 6 + 1;
                spectrum_set_dial(dial_hz, step_idx, cur_mode == MODE_CW ? -CW_PITCH : 0);
                nvs_dirty = true;
                t_last_tune = now;
            }
            if (fpga_present && now - t_adc > 100000) { t_adc = now; adc_status(now); }
            console_poll();
            spectrum_capture_dump();
            spectrum_sm_dump();
            if (src_fpga && fpga_present) {
                int r = fpga_iq_block(iq, SPEC_FFT_N);
                if (r == FPGA_IQ_OK) spectrum_step(iq, "FPGA");
                else if (r == FPGA_IQ_STALE) spectrum_hold();
                else vTaskDelay(1);
            } else {
                spectrum_step(NULL, "SIM ");
            }
            if (squelch_update(now) && sql_mode) spectrum_show_squelch(sql_db);   // live bar
            break;

        case ST_MENU:
            // frames keep coming without drawing: squelch, S-meter, waterfall history
            if (src_fpga && fpga_present && fpga_iq_block(iq, SPEC_FFT_N) == FPGA_IQ_OK) spectrum_feed(iq);
            squelch_update(now);
            if (editing) {
                action_t a = cur_menu->items[cursor].act;
                int d = enc + (ev == EV_NEXT) - (ev == EV_PREV);
                if (d) { edit_adjust(a, d); draw_menu(); }
                if (ev == EV_SELECT || ev == EV_HOLD || ev == EV_BACK || ev == EV_EXIT) {
                    editing = false;
                    mode_show();   // passband after a shift (the ruler sits under the title)
                    dial_save();
                    draw_menu();
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            if (ev == EV_NEXT) { cursor = (cursor + 1) % cur_menu->n; draw_menu(); }
            else if (ev == EV_PREV) { cursor = (cursor + cur_menu->n - 1) % cur_menu->n; draw_menu(); }
            else if (ev == EV_SELECT || ev == EV_HOLD) menu_select();
            else if (ev == EV_BACK) {
                if (cur_menu->parent) enter_menu(cur_menu->parent, 0);
                else enter_main();
            }
            else if (ev == EV_EXIT) enter_main();
            else if (enc) {
                cursor = ((cursor + enc) % cur_menu->n + cur_menu->n) % cur_menu->n;
                draw_menu();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case ST_TEST_SPI:
        case ST_TEST_I2S:
            if (ev == EV_SELECT || ev == EV_HOLD || ev == EV_BACK || ev == EV_EXIT) {
                if (state == ST_TEST_I2S) i2s_test_leave();
                if (ev == EV_EXIT) enter_main();
                else enter_menu(&menu_system, state == ST_TEST_SPI ? 0 : 1);
                break;
            }
            if (esp_timer_get_time() - t_test > 300000) {
                t_test = esp_timer_get_time();
                if (state == ST_TEST_SPI) spi_test_step();
                else i2s_test_step();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case ST_TEST_NOISE:
            if (ev == EV_HOLD || ev == EV_BACK) { enter_menu(&menu_system, 2); break; }
            if (ev == EV_EXIT) { enter_main(); break; }
            console_poll();
            if (!fpga_present) { vTaskDelay(pdMS_TO_TICKS(10)); break; }
            if (enc || ev == EV_NEXT || ev == EV_PREV) { nz_slot ^= 1; nz_draw(); }
            if (ev == EV_SELECT && nz_count > 0) { nz_store(nz_slot); nz_draw(); }
            if (now - t_adc > 100000) {
                t_adc = now;
                uint8_t p = fpga_read(FPGA_REG_PEAK);
                if (p > nz_peak) nz_peak = p;
            }
            if (fpga_iq_block(iq, SPEC_FFT_N) == FPGA_IQ_OK) {
                float db;
                if (noise_add(iq, &db)) {
                    nz_live = db;
                    nz_count++;
                    nz_draw();
                    nz_peak = 0;
                }
            } else vTaskDelay(1);
            break;
        }
    }
}
