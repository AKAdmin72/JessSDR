// JessSDR: 40m SDR UI firmware for ESP32-S3 + 4" ST7796S 480x320 (PLAN.md Phase 3),
// ported from ../tang_nano_9k_projects/FpgaRX-SDR/esp32.
//
// Screens: the main spectrum/waterfall screen, a menu tree, and the link
// test screens in MENU > SYSTEM. Controls (input.h):
//   main screen: turn = tune by the step, push = next step, hold = menu
//   menu:        turn = move, push or hold = enter, BACK = up one level
// While the encoder is not wired, the BOOT button does it: short press =
// NEXT (tune up / move down), hold = HOLD (menu / enter).
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

static const char *TAG = "jesssdr";   // (the NVS namespace stays "fpgarx": saved settings)

#define C_BG       RGB565(0, 0, 0)
#define C_PANEL    RGB565(10, 10, 30)
#define C_TITLE_BG RGB565(0, 60, 120)
#define C_TEXT     RGB565(255, 255, 255)
#define C_DIM      RGB565(150, 150, 170)
#define C_SEL_BG   RGB565(255, 200, 0)
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
typedef enum { ACT_NONE, ACT_SUBMENU, ACT_BACK, ACT_EXIT, ACT_MODE, ACT_RANGE, ACT_WINDOW, ACT_FPS, ACT_NR, ACT_NR_LEVEL, ACT_SOURCE, ACT_TEST_SPI, ACT_TEST_I2S } action_t;
typedef struct menu menu_t;
typedef struct { const char *label; action_t act; const menu_t *sub; } item_t;
struct menu { const char *title; const item_t *items; int n; const menu_t *parent; };

static const menu_t menu_main, menu_system, menu_display, menu_audio;
static const item_t audio_items[] = {
    { .label = "NR", .act = ACT_NR },               // ": OFF" / ": NR" / ": ANF"
    { .label = "Level", .act = ACT_NR_LEVEL },      // ": 1".."4"
    { .label = "Back", .act = ACT_BACK },
};
static const item_t display_items[] = {
    { .label = "Range", .act = ACT_RANGE },     // ": 40 dB"
    { .label = "Window", .act = ACT_WINDOW },   // ": OFF" / ": HANN"
    { .label = "FPS", .act = ACT_FPS },         // ": ON" / ": OFF"
    { .label = "Back", .act = ACT_BACK },
};
static const item_t system_items[] = {
    { .label = "SPI link test", .act = ACT_TEST_SPI },
    { .label = "I2S link test", .act = ACT_TEST_I2S },
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
static const menu_t menu_system = { "SYSTEM", system_items, 3, &menu_main };
static const menu_t menu_display = { "DISPLAY", display_items, 4, &menu_main };
static const menu_t menu_audio   = { "AUDIO",   audio_items,   3, &menu_main };

static bool src_fpga = true;   // spectrum input: false = synthetic, true = FPGA I2S (kept in NVS)
// Display (kept in NVS): dB over the spectrum panel, Hann window on/off
static const uint8_t range_db[] = { 20, 30, 40, 60 };
static int range_idx = 2;
static bool win_hann;
static bool fpga_present;      // the FPGA answered with FPGA_ID at power-up
static bool fps_on = false;    // frame rate in the header, kept in NVS
// FPGA LMS noise reduction (audio_nr.v): FPGA_NR_*, level 0..3 (shown 1..4,
// 4 = strongest = fastest adaptation). The FPGA's level field is the other
// way round (mu = 2^-(10+field)), hence 3 - nr_level. Default 3 = mu 2^-11,
// WDSP's two_mu (1e-4 at 48kHz) at our 13.2kHz.
static int nr_mode = FPGA_NR_OFF, nr_level = 2;
static const char *nr_name[] = { "OFF", "NR", "ANF" };
static uint8_t nr_reg(void) { return nr_mode | (3 - nr_level) << 2; }
// No I/Q correction: the DDC I/Q are digital, so perfectly balanced.
static uint8_t ctrl_base(void) { return 0; }

// ADC overload (adc_peak.v): the FPGA keeps the max |I|,|Q| of the raw
// samples over 0.31s windows; read every 100ms so no window is missed. The
// header shows CLIP for 1s after a clipped window, else the highest peak of
// the last second if it is above -6dBFS.
static void adc_status(int64_t now)
{
    static int64_t t_clip = -10000000, t_pk, t_log;
    static uint8_t pk_hold;
    uint8_t p = fpga_read(FPGA_REG_PEAK);
    if (p >= FPGA_PEAK_CLIP) {
        if (now - t_clip > 1000000 && now - t_log > 1000000) {
            t_log = now;
            ESP_LOGW(TAG, "ADC CLIP (peak %u/255) - too much gain / signal", p);
        }
        t_clip = now;
    }
    if (p >= pk_hold || now - t_pk > 1000000) { pk_hold = p; t_pk = now; }
    spectrum_set_adc(now - t_clip < 1000000, 20.0f * log10f((pk_hold + 0.5f) / 256.0f));
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
#define MODE_AUTO  3
static int mode_sel = MODE_AUTO;
static int cur_mode = -1;
static const char *mode_name[] = { "LSB", "USB", "AM" };

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
        fpga_write(FPGA_REG_MODE, m);
        cur_mode = m;
        lo_shift_k = fpga_mode_k(m);
        ESP_LOGI(TAG, "mode %s%s, K=%d", mode_name[m], mode_sel == MODE_AUTO ? " (auto)" : "", lo_shift_k);
    }
    spectrum_set_mode(cur_mode, mode_sel == MODE_AUTO);
    spectrum_set_dial(dial_hz, step_idx, lo_shift_k);
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
    if (nvs_get_u8(h, "mode", &st) == ESP_OK && st <= MODE_AUTO) mode_sel = st;
    if (nvs_get_u8(h, "range", &st) == ESP_OK && st < sizeof(range_db)) range_idx = st;
    if (nvs_get_u8(h, "win", &st) == ESP_OK) win_hann = st;
    if (nvs_get_u8(h, "fps", &st) == ESP_OK) fps_on = st;
    if (nvs_get_u8(h, "nrm", &st) == ESP_OK && st <= FPGA_NR_ANF) nr_mode = st;
    if (nvs_get_u8(h, "nrl2", &st) == ESP_OK && st <= 3) nr_level = st;
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
    nvs_set_u8(h, "range", range_idx);
    nvs_set_u8(h, "win", win_hann);
    nvs_set_u8(h, "fps", fps_on);
    nvs_set_u8(h, "nrm", nr_mode);
    nvs_set_u8(h, "nrl2", nr_level);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "saved dial %lu Hz, step %lu Hz", (unsigned long)dial_hz, (unsigned long)step_hz[step_idx]);
}

static void dial_apply(void)
{
    fpga_set_freq(dial_hz);
    mode_apply(false);
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

typedef enum { ST_MAIN, ST_MENU, ST_TEST_SPI, ST_TEST_I2S } state_t;
static state_t state = ST_MAIN;
static const menu_t *cur_menu = &menu_main;
static int cursor;

// Tang buttons S1/S2 as a stand-in for the encoder (sdr_esp buttons.v):
// the FPGA counts short/long presses (2-bit counters), polled every 30ms.
//   S1 short = NEXT (tune up / move down)   S1 long = SELECT (next step / enter)
//   S2 short = PREV (tune down / move up)   S2 long = menu on the main screen, else BACK
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
    const input_ev_t map[4] = { EV_NEXT, EV_SELECT, EV_PREV, state == ST_MAIN ? EV_HOLD : EV_BACK };
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
        else if (it->act == ACT_RANGE) snprintf(s, sizeof(s), "%s: %d dB", it->label, range_db[range_idx]);
        else if (it->act == ACT_NR) snprintf(s, sizeof(s), "%s: %s", it->label, nr_name[nr_mode]);
        else if (it->act == ACT_NR_LEVEL) snprintf(s, sizeof(s), "%s: %d", it->label, nr_level + 1);
        else if (it->act == ACT_FPS) snprintf(s, sizeof(s), "%s: %s", it->label, fps_on ? "ON" : "OFF");
        else if (it->act == ACT_WINDOW) snprintf(s, sizeof(s), "%s: %s", it->label, win_hann ? "HANN" : "OFF");
        else if (it->act == ACT_MODE) snprintf(s, sizeof(s), "%s: %s", it->label, mode_name[mode_sel]);
        else snprintf(s, sizeof(s), "%s%s", it->label, it->act == ACT_SUBMENU ? " >" : "");
        bool sel = (i == cursor);
        int y = AREA_Y + ROW_H + 8 + i * (ROW_H + 4);
        uint16_t bg = sel ? C_SEL_BG : C_PANEL;
        lcd_fill(12, y, LCD_W - 24, ROW_H, bg);
        lcd_text(24, y + 3, &font_mid, s, sel ? C_SEL_FG : C_TEXT, bg);
    }
    lcd_text(8, LCD_H - 16, &font_small, "BOOT: press = next, hold = enter", C_DIM, C_PANEL);
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
    lcd_text(8, LCD_H - 16, &font_small, "hold BOOT (or BACK) to leave", C_DIM, C_PANEL);
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
        mode_sel = mode_sel == MODE_AUTO ? FPGA_MODE_LSB : mode_sel == FPGA_MODE_AM ? MODE_AUTO : mode_sel + 1;
        mode_apply(false);   // redraws the ruler under the menu title,
        dial_save();
        draw_menu();         // so the menu goes back on top
        break;
    case ACT_RANGE:    // 20 -> 30 -> 40 -> 60 -> 20 dB
        range_idx = (range_idx + 1) % sizeof(range_db);
        spectrum_set_view(range_db[range_idx], win_hann);
        dial_save();
        draw_menu();
        break;
    case ACT_WINDOW:
        win_hann = !win_hann;
        spectrum_set_view(range_db[range_idx], win_hann);
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
    case ACT_FPS:
        fps_on = !fps_on;
        spectrum_show_fps(fps_on);
        dial_save();
        draw_menu();
        break;
    case ACT_SOURCE:   src_fpga = !src_fpga; dial_save(); draw_menu(); break;
    case ACT_TEST_SPI: state = ST_TEST_SPI; spi_test_enter(); break;
    case ACT_TEST_I2S: state = ST_TEST_I2S; i2s_test_enter(); break;
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
        } else if (sscanf(line, "s %ld", &v) == 1 && v > 0 && v <= 1000) {
            spectrum_capture_start(v);
        } else if (sscanf(line, "lcd m %lx", &v) == 1 && v >= 0 && v <= 0xFF) {
            lcd_set_madctl(v);
            enter_main();
            printf("OK lcd m %02X\n", lcd_get_madctl());
        } else if (sscanf(line, "lcd i %ld", &v) == 1) {
            lcd_set_invert(v != 0);
            printf("OK lcd i %d\n", lcd_get_invert());
        } else if (strcmp(line, "lcd") == 0) {
            printf("lcd m %02X i %d\n", lcd_get_madctl(), lcd_get_invert());
        } else if (line[0]) printf("ERR %s\n", line);
    }
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
    lcd_init();
    splash_show();                     // Jessica, while the link self-test runs
    lcd_backlight(100);
    int64_t t_splash = esp_timer_get_time();
    spectrum_init();
    spectrum_set_view(range_db[range_idx], win_hann);
    fpga_init();
    input_init();
    if (uart_driver_install(UART_NUM_0, 256, 4096, 0, NULL, 0) == ESP_OK) uart_vfs_dev_use_driver(UART_NUM_0);
    uint8_t id = fpga_read(FPGA_REG_ID);
    fpga_present = id == FPGA_ID;
    if (fpga_present) {
        link_selftest();                          // result in the serial log
        fpga_set_freq(dial_hz);
        fpga_write(FPGA_REG_CTRL, ctrl_base());
        fpga_write(FPGA_REG_NR, nr_reg());
    } else {
        ESP_LOGW(TAG, "no FPGA (ID read 0x%02X, want 0x%02X) - standalone, SIM source", id, FPGA_ID);
    }
    int64_t shown = esp_timer_get_time() - t_splash;   // keep the splash up >= 2.5s
    if (shown < 2500000) vTaskDelay(pdMS_TO_TICKS((2500000 - shown) / 1000));
    lcd_fill(0, 0, LCD_W, LCD_H, C_BG);
    spectrum_show_fps(fps_on);
    mode_apply(true);
    enter_main();
    ESP_LOGI(TAG, "dial %lu Hz, step %lu Hz", (unsigned long)dial_hz, (unsigned long)step_hz[step_idx]);

    int64_t t_test = 0, t_check = 0, t_adc = 0;
    while (1) {
        input_ev_t ev = input_get();
        int64_t now = esp_timer_get_time();
        if (ev == EV_NONE && fpga_present) ev = tang_buttons(now);

        // save the dial 2s after the last change (spares the flash while turning)
        if (nvs_dirty && now - t_last_tune > 2000000) {
            nvs_dirty = false;
            dial_save();
        }
        // once a second: if the FPGA was restarted it lost the dial - resend
        if (fpga_present && now - t_check > 1000000 && state != ST_TEST_SPI) {
            t_check = now;
            fpga_led_hold(true);
            if (fpga_get_freq() != dial_hz) {
                ESP_LOGW(TAG, "FPGA dial differs (restarted?) - resending %lu Hz", (unsigned long)dial_hz);
                fpga_set_freq(dial_hz);
            }
            uint8_t ctrl = ctrl_base() | (state == ST_TEST_I2S ? FPGA_CTRL_I2S_TEST : 0);
            if (fpga_read(FPGA_REG_CTRL) != ctrl) {
                ESP_LOGW(TAG, "FPGA CTRL differs (restarted?) - resending");
                fpga_write(FPGA_REG_CTRL, ctrl);
            }
            if (fpga_read(FPGA_REG_NR) != nr_reg()) {
                ESP_LOGW(TAG, "FPGA NR differs (restarted?) - resending");
                fpga_write(FPGA_REG_NR, nr_reg());
            }
            if (fpga_read(FPGA_REG_MODE) != cur_mode) {
                ESP_LOGW(TAG, "FPGA mode differs (restarted?) - resending %s", mode_name[cur_mode]);
                fpga_write(FPGA_REG_MODE, cur_mode);
            }
            fpga_led_hold(false);
        }

        switch (state) {
        case ST_MAIN:
            if (ev == EV_HOLD) { enter_menu(&menu_main, 0); break; }
            if (ev == EV_NEXT) dial_tune(+1);
            if (ev == EV_PREV) dial_tune(-1);
            if (ev == EV_SELECT) {
                step_idx = step_idx % 6 + 1;
                spectrum_set_dial(dial_hz, step_idx, lo_shift_k);
                nvs_dirty = true;
                t_last_tune = now;
            }
            if (fpga_present && now - t_adc > 100000) { t_adc = now; adc_status(now); }
            console_poll();
            spectrum_capture_dump();
            if (src_fpga && fpga_present) {
                if (fpga_iq_block(iq, SPEC_FFT_N)) spectrum_step(iq, "FPGA");
                else vTaskDelay(1);
            } else {
                spectrum_step(NULL, "SIM ");
            }
            break;

        case ST_MENU:
            if (ev == EV_NEXT) { cursor = (cursor + 1) % cur_menu->n; draw_menu(); }
            else if (ev == EV_PREV) { cursor = (cursor + cur_menu->n - 1) % cur_menu->n; draw_menu(); }
            else if (ev == EV_SELECT || ev == EV_HOLD) menu_select();
            else if (ev == EV_BACK) {
                if (cur_menu->parent) enter_menu(cur_menu->parent, 0);
                else enter_main();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case ST_TEST_SPI:
        case ST_TEST_I2S:
            if (ev == EV_SELECT || ev == EV_HOLD || ev == EV_BACK) {
                if (state == ST_TEST_I2S) i2s_test_leave();
                enter_menu(&menu_system, state == ST_TEST_SPI ? 0 : 1);
                break;
            }
            if (esp_timer_get_time() - t_test > 300000) {
                t_test = esp_timer_get_time();
                if (state == ST_TEST_SPI) spi_test_step();
                else i2s_test_step();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
    }
}
