#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "fpga.h"

static const char *TAG = "fpga";

// ---- pins: right row J3, straight jumpers to Tang J5 (hardware/system) ----
#define PIN_SPI_SCK   1    // J3-4  <-> Tang J5-19, FPGA 53
#define PIN_SPI_MOSI  39   // J3-9  <-> Tang J5-14, FPGA 79
#define PIN_SPI_MISO  2    // J3-5  <-> Tang J5-18, FPGA 71
#define PIN_SPI_CS    20   // J3-19 <-> Tang J5-4,  FPGA 80 (USB D+ used as GPIO)
#define PIN_I2S_DIN   21   // J3-18 <-> Tang J5-5,  FPGA 42
#define PIN_I2S_WS    42   // J3-6  <-> Tang J5-17, FPGA 72
#define PIN_I2S_BCK   47   // J3-17 <-> Tang J5-6,  FPGA 41

#define SPI_HZ        (1000 * 1000)   // the FPGA samples SCK with 27MHz
#define I2S_BLOCK     1024            // frames per read

static spi_device_handle_t spi;
static spi_device_handle_t led;        // no CS: MOSI only, the FPGA ignores it (CS high)

// Tang pin 79 (our MOSI) is also the DIN of the board's WS2812 RGB LED, so
// the register traffic sets it to random (mostly bright) colours. After
// every transaction a "black" WS2812 frame follows on MOSI, sent through a
// second, CS-less device at 2.5MHz (0.4us per SPI bit): reset (>= 280us
// low), 24 "0" bits as 100 patterns (0.4us high, 0.8us low), reset again so
// it latches. The garbage colour lives ~0.3ms per transaction.
#define LED_RST_BYTES 96               // 96 * 3.2us = 307us low
static uint8_t *led_frame;             // DMA-capable
static int led_len;
static int led_hold;                   // > 0: bursts, black once at the end
static bool led_dirty;

static void led_black(void)
{
    if (led_hold) { led_dirty = true; return; }
    spi_transaction_t t = { .length = 8 * led_len, .tx_buffer = led_frame };
    ESP_ERROR_CHECK(spi_device_polling_transmit(led, &t));
    led_dirty = false;
}

void fpga_led_hold(bool on)
{
    if (on) led_hold++;
    else if (led_hold > 0 && --led_hold == 0 && led_dirty) led_black();
}

static void xfer(spi_transaction_t *t)
{
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, t));
    led_black();
}
static i2s_chan_handle_t rx;

uint8_t fpga_read(uint8_t addr)
{
    uint8_t tx[2] = { 0x80 | addr, 0 }, rxb[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rxb };
    xfer(&t);
    return rxb[1];
}

void fpga_write(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { addr & 0x7F, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    xfer(&t);
}

uint32_t fpga_read_u32(uint8_t addr)
{
    uint8_t tx[5] = { 0x80 | addr }, rxb[5] = { 0 };
    spi_transaction_t t = { .length = 40, .tx_buffer = tx, .rx_buffer = rxb };
    xfer(&t);
    return (uint32_t)rxb[1] << 24 | (uint32_t)rxb[2] << 16 | (uint32_t)rxb[3] << 8 | rxb[4];
}

void fpga_set_freq(uint32_t hz)
{
    uint32_t bcd = 0;
    for (int i = 0; i < 8; i++, hz /= 10) bcd |= (hz % 10) << (4 * i);
    // one burst: the FPGA latches all four bytes when the last one arrives
    uint8_t tx[5] = { FPGA_REG_FREQ, bcd >> 24, bcd >> 16, bcd >> 8, bcd };
    spi_transaction_t t = { .length = 40, .tx_buffer = tx };
    xfer(&t);
}

uint32_t fpga_get_freq(void)
{
    uint32_t bcd = fpga_read_u32(FPGA_REG_FREQ), hz = 0;
    for (int i = 7; i >= 0; i--) hz = hz * 10 + ((bcd >> (4 * i)) & 0xF);
    return hz;
}

int fpga_mode_k(int mode)
{
    (void)mode;
    return 0;
}

uint32_t fpga_expected_lo_fw(uint32_t hz, int k)
{
    // sdr_esp_top.v: 64.8MHz NCO, fw = (f * 69500145) >> 20 (~ f * 2^32 / 64.8MHz);
    // no low-IF offset (K = 0)
    (void)k;
    return (uint32_t)(((uint64_t)hz * 69500145u) >> 20);
}

// ---- I2S receive task ----
static volatile bool test_on;
static fpga_i2s_stats_t st;
static portMUX_TYPE st_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool reset_req;

static float iq_buf[2][2 * I2S_BLOCK];
static volatile int iq_ready = -1;     // index of the newest complete block
static volatile bool iq_new;

void fpga_i2s_test_mode(bool on) { test_on = on; }
void fpga_i2s_stats_reset(void) { reset_req = true; }

void fpga_i2s_stats(fpga_i2s_stats_t *s)
{
    taskENTER_CRITICAL(&st_lock);
    *s = st;
    taskEXIT_CRITICAL(&st_lock);
}

float fpga_i2s_rate(const fpga_i2s_stats_t *s)
{
    int64_t dt = s->t_last_us - s->t_reset_us;
    return dt > 0 ? s->frames * 1e6f / dt : 0;
}

bool fpga_iq_block(float *dst, int n)
{
    if (!iq_new || iq_ready < 0) return false;
    iq_new = false;
    memcpy(dst, iq_buf[iq_ready], n * 2 * sizeof(float));
    return true;
}

static void i2s_task(void *arg)
{
    static int32_t buf[2 * I2S_BLOCK];
    uint32_t prev_cnt = 0;
    bool have_prev = false;
    int fill = 0;
    while (1) {
        size_t got = 0;
        if (i2s_channel_read(rx, buf, sizeof(buf), &got, pdMS_TO_TICKS(200)) != ESP_OK || got == 0)
            continue;
        int n = got / 8;
        int64_t now = esp_timer_get_time();
        if (reset_req) {
            // Start counting at this block boundary; the block itself is not
            // counted, so frames / (t_last - t_reset) is exact.
            reset_req = false;
            have_prev = false;
            taskENTER_CRITICAL(&st_lock);
            memset(&st, 0, sizeof(st));
            st.t_reset_us = st.t_last_us = now;
            taskEXIT_CRITICAL(&st_lock);
            continue;
        }

        uint32_t perr = 0, jumps = 0;
        if (test_on) {
            // left = {cnt, 0}, right = {~cnt, 0}; cnt + 1 per frame
            for (int k = 0; k < n; k++) {
                uint32_t l = (uint32_t)buf[2 * k] >> 1, r = (uint32_t)buf[2 * k + 1] >> 1;
                if (r != (~l & 0x7FFFFFFF)) perr++;
                if (have_prev && l != ((prev_cnt + 1) & 0x7FFFFFFF)) jumps++;
                prev_cnt = l;
                have_prev = true;
            }
        } else {
            have_prev = false;
            float *d = iq_buf[fill];
            for (int k = 0; k < n && k < I2S_BLOCK; k++) {
                d[2 * k]     = buf[2 * k]     * (1.0f / 2147483648.0f);
                d[2 * k + 1] = buf[2 * k + 1] * (1.0f / 2147483648.0f);
            }
            if (n == I2S_BLOCK) {
                iq_ready = fill;
                iq_new = true;
                fill ^= 1;
            }
        }
        taskENTER_CRITICAL(&st_lock);
        st.frames += n;
        st.t_last_us = now;
        st.pattern_err += perr;
        st.count_jumps += jumps;
        st.last_left = buf[2 * (n - 1)];
        st.last_right = buf[2 * (n - 1) + 1];
        taskEXIT_CRITICAL(&st_lock);
    }
}

void fpga_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_HZ,
        .mode = 0,
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 2,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev, &spi));
    spi_device_interface_config_t ldev = {
        .clock_speed_hz = 2500 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &ldev, &led));
    static const uint8_t black[9] = { 0x92, 0x49, 0x24, 0x92, 0x49, 0x24, 0x92, 0x49, 0x24 };
    led_len = 2 * LED_RST_BYTES + sizeof(black);
    led_frame = heap_caps_calloc(1, led_len, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    memcpy(led_frame + LED_RST_BYTES, black, sizeof(black));
    led_black();

    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan.dma_desc_num = 8;
    chan.dma_frame_num = 480;          // max 511
    ESP_ERROR_CHECK(i2s_new_channel(&chan, NULL, &rx));
    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)FPGA_FS_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCK,
            .ws = PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = PIN_I2S_DIN,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx, &std));
    ESP_ERROR_CHECK(i2s_channel_enable(rx));
    reset_req = true;
    xTaskCreatePinnedToCore(i2s_task, "i2s_rx", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "link up: SPI %d kHz, I2S slave %.3f kHz", SPI_HZ / 1000, FPGA_FS_HZ / 1000);
}
