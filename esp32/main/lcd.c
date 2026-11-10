#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "lcd.h"

static const char *TAG = "lcd";

// Pins as in FpgaRX (PLAN.md Phase 3); the 4" module has the same header
// as the 2.8" ILI9341 one. MISO (SDO) is not used.
// left row J1, consecutive, same order as the TFT header pins 3..8
// (N16R8: 35..37 belong to the octal PSRAM)
#define PIN_LCD_CS    4
#define PIN_LCD_RST   5
#define PIN_LCD_DC    6
#define PIN_LCD_MOSI  7
#define PIN_LCD_SCK   15
#define PIN_LCD_LED   16

#define LCD_HOST      SPI2_HOST
#define LCD_PCLK_HZ   (80 * 1000 * 1000)    // 80 MHz / n: 80, 40, 26.7. Tang tft_test ran this panel at 54-67 MHz

// MADCTL bits: MY 0x80, MX 0x40, MV 0x20 (swap x/y = landscape), BGR 0x08.
// MV|MX|MY|BGR = the same landscape as the ILI9341 build (header on the
// left). Unchecked on this panel - the console "lcd m <hex>" tries others.
#define MADCTL_DEFAULT 0x28       // 2026-11-10: 0xE8 was upside down on the board -> MX, MY flipped (180 deg)
#define INVERT_DEFAULT 0          // some ST7796 modules need INVON (0x21)

static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t free_bufs;          // counts buffers not in flight
static uint16_t *bufs[LCD_NBUF];             // ring, sent in the order got
static int cur;                              // buffer handed out by lcd_buf_get
static uint8_t madctl = MADCTL_DEFAULT;
static int invert = INVERT_DEFAULT;

static bool on_trans_done(esp_lcd_panel_io_handle_t h, esp_lcd_panel_io_event_data_t *ev, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(free_bufs, &woken);
    return woken == pdTRUE;
}

static void cmd(uint8_t c, const uint8_t *p, int n)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io, c, p, n));
}

uint16_t *lcd_buf_get(void)
{
    xSemaphoreTake(free_bufs, portMAX_DELAY);
    cur = (cur + 1) % LCD_NBUF;
    return bufs[cur];
}

// CASET/RASET wait for the queued colour transfers (esp_lcd does that in
// tx_param), then RAMWR goes out by DMA and frees the buffer when done.
void lcd_buf_send_buf(const uint16_t *b, int x, int y, int w, int h)
{
    int x1 = x + w - 1, y1 = y + h - 1;
    cmd(0x2A, (uint8_t[]){ x >> 8, x, x1 >> 8, x1 }, 4);
    cmd(0x2B, (uint8_t[]){ y >> 8, y, y1 >> 8, y1 }, 4);
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io, 0x2C, b, w * h * sizeof(uint16_t)));
}

void lcd_buf_send(int x, int y, int w, int h) { lcd_buf_send_buf(bufs[cur], x, y, w, h); }

void lcd_fill(int x, int y, int w, int h, uint16_t c)
{
    int lines = LCD_W * LCD_CHUNK / w;
    while (h > 0) {
        int n = h < lines ? h : lines;
        uint16_t *b = lcd_buf_get();
        for (int i = 0; i < w * n; i++) b[i] = c;
        lcd_buf_send(x, y, w, n);
        y += n;
        h -= n;
    }
}

int lcd_text(int x, int y, const font_t *f, const char *s, uint16_t fg, uint16_t bg)
{
    int len = strlen(s), w = len * f->w, bpr = (f->w + 7) / 8;
    if (w == 0 || w * f->h > LCD_W * LCD_CHUNK) return 0;
    uint16_t *b = lcd_buf_get();
    for (int i = 0; i < len; i++) {
        int c = (unsigned char)s[i];
        if (c < 32 || c > 126) c = '?';
        const uint8_t *g = f->bits + (c - 32) * f->h * bpr;
        for (int yy = 0; yy < f->h; yy++)
            for (int xx = 0; xx < f->w; xx++)
                b[yy * w + i * f->w + xx] =
                    (g[yy * bpr + xx / 8] & (0x80 >> (xx & 7))) ? fg : bg;
    }
    lcd_buf_send(x, y, w, f->h);
    return w;
}

void lcd_backlight(int percent)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, percent * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void lcd_set_madctl(uint8_t v) { madctl = v; cmd(0x36, &madctl, 1); }
void lcd_set_invert(int on)    { invert = on; cmd(on ? 0x21 : 0x20, NULL, 0); }
uint8_t lcd_get_madctl(void)   { return madctl; }
int lcd_get_invert(void)       { return invert; }

// ST7796S power-on sequence (the common one from the module vendors'
// demo code / TFT_eSPI): unlock the command set 2, RGB565, 1-dot inversion,
// power and gamma, lock again.
static void st7796_init(void)
{
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x01, NULL, 0);                                  // SWRESET
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x11, NULL, 0);                                  // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0xF0, (uint8_t[]){ 0xC3 }, 1);                   // CSCON: enable cmd set 2 part 1
    cmd(0xF0, (uint8_t[]){ 0x96 }, 1);                   //        part 2
    cmd(0x36, &madctl, 1);                               // MADCTL
    cmd(0x3A, (uint8_t[]){ 0x55 }, 1);                   // COLMOD 16 bit
    cmd(0xB4, (uint8_t[]){ 0x01 }, 1);                   // 1-dot inversion
    cmd(0xB6, (uint8_t[]){ 0x80, 0x02, 0x3B }, 3);       // display function control
    cmd(0xE8, (uint8_t[]){ 0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33 }, 8);
    cmd(0xC1, (uint8_t[]){ 0x06 }, 1);                   // power control 2
    cmd(0xC2, (uint8_t[]){ 0xA7 }, 1);                   // power control 3
    cmd(0xC5, (uint8_t[]){ 0x18 }, 1);                   // VCOM
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0xE0, (uint8_t[]){ 0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F,
                           0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B }, 14);
    cmd(0xE1, (uint8_t[]){ 0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B,
                           0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B }, 14);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0xF0, (uint8_t[]){ 0x3C }, 1);                   // lock cmd set 2
    cmd(0xF0, (uint8_t[]){ 0x69 }, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(invert ? 0x21 : 0x20, NULL, 0);                  // INVON / INVOFF
    cmd(0x29, NULL, 0);                                  // DISPON
    ESP_LOGI(TAG, "ST7796S %dx%d, MADCTL %02X, invert %d, SPI %d MHz",
             LCD_W, LCD_H, madctl, invert, LCD_PCLK_HZ / 1000000);
}

void lcd_init(void)
{
    free_bufs = xSemaphoreCreateCounting(LCD_NBUF, LCD_NBUF);
    for (int i = 0; i < LCD_NBUF; i++) {
        bufs[i] = heap_caps_malloc(LCD_W * LCD_CHUNK * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(bufs[i]);
    }

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t ch = {
        .gpio_num = PIN_LCD_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_config_t rst = { .pin_bit_mask = 1ULL << PIN_LCD_RST, .mode = GPIO_MODE_OUTPUT };
    ESP_ERROR_CHECK(gpio_config(&rst));
    gpio_set_level(PIN_LCD_RST, 1);

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * 20 * sizeof(uint16_t),  // S3 DMA max is 32 KB; esp_lcd splits a buffer
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 2 * LCD_NBUF,
        .on_color_trans_done = on_trans_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));
    st7796_init();
}
