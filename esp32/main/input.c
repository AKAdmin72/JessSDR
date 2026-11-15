#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <stdatomic.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "input.h"

#define PIN_BOOT   0
#define PIN_ENC_A  17
#define PIN_ENC_B  18
#define PIN_SEL    8      // separate SELECT button (was the encoder push)
#define PIN_BACK   12
#define PIN_PREV   13
#define PIN_NEXT   14

#define POLL_US      1000
#define DEBOUNCE_MS  20
#define LONG_MS      500

// Encoder: 600 lines x4 = 2400 counts/rev. Swap ENC_DIR if it tunes backwards.
#define ENC_DIR       (-1)
#define ENC_PER_REV   2400
#define ENC_TUNE_DIV  (ENC_PER_REV / 50)     // counts per dial step (50/rev)
#define ENC_MENU_DIV  (ENC_PER_REV / 20)     // counts per menu step
#define ENC_PERIOD_MS 10                     // speed is measured over this
#define ENC_LIMIT     10000                  // PCNT range, accumulated past it

static QueueHandle_t q;

typedef struct { int pin; int stable, cnt; int held_ms; bool long_sent; } btn_t;
static btn_t btns[] = {
    { .pin = PIN_BOOT, .stable = 1 }, { .pin = PIN_SEL, .stable = 1 },
    { .pin = PIN_BACK, .stable = 1 }, { .pin = PIN_PREV, .stable = 1 }, { .pin = PIN_NEXT, .stable = 1 },
};
static pcnt_unit_handle_t enc_unit;
static int enc_last, enc_ms;
static int tune_acc, menu_acc;                 // counts not yet turned into steps
static atomic_int tune_steps, menu_steps;

static void send(input_ev_t e)
{
    xQueueSend(q, &e, 0);   // esp_timer callbacks run in a task, not an ISR
}

// Returns +1 on a debounced press, -1 on a release, 0 otherwise.
static int debounce(btn_t *b)
{
    int lv = gpio_get_level(b->pin);
    if (lv == b->stable) { b->cnt = 0; return 0; }
    if (++b->cnt < DEBOUNCE_MS * 1000 / POLL_US) return 0;
    b->stable = lv;
    b->cnt = 0;
    return lv ? -1 : +1;
}

// Button with a short and a long action: the short one on release, the long
// one once while still held.
static void short_long(btn_t *b, input_ev_t ev_short, input_ev_t ev_long)
{
    int e = debounce(b);
    if (e > 0) { b->held_ms = 0; b->long_sent = false; }
    if (!b->stable && !b->long_sent && ++b->held_ms >= LONG_MS) {
        b->long_sent = true;
        send(ev_long);
    }
    if (e < 0 && !b->long_sent) send(ev_short);
}

// Every ENC_PERIOD_MS: new counts -> dial steps (with acceleration by the
// speed over this period) and menu steps. Remainders are kept, so slow
// turning never loses counts; a direction change drops them.
static void enc_poll(void)
{
    int c;
    if (pcnt_unit_get_count(enc_unit, &c) != ESP_OK) return;
    int d = (c - enc_last) * ENC_DIR;
    enc_last = c;
    // speed, smoothed over ~80 ms so the multiplier doesn't jump between
    // polls; Q4 counts per 10 ms, 24 = 1 rev/s
    static int spd;
    int a = d < 0 ? -d : d;
    spd += (a * 16 - spd) / 8;
    if (d == 0) return;
    int rps4 = spd * 4 / (16 * ENC_PER_REV / 100);   // speed in 1/4 rev/s
    int mul = rps4 < 4 ? 1 : rps4 < 8 ? 2 : 4;       // <1 rev/s x1, <2 x2, else x4
    if ((d > 0) != (tune_acc > 0) && tune_acc) tune_acc = 0;
    if ((d > 0) != (menu_acc > 0) && menu_acc) menu_acc = 0;
    tune_acc += d * mul;
    menu_acc += d;
    int t = tune_acc / ENC_TUNE_DIV, m = menu_acc / ENC_MENU_DIV;   // truncates toward 0
    tune_acc -= t * ENC_TUNE_DIV;
    menu_acc -= m * ENC_MENU_DIV;
    if (t) atomic_fetch_add(&tune_steps, t);
    if (m) atomic_fetch_add(&menu_steps, m);
}

static void poll_cb(void *arg)
{
    short_long(&btns[0], EV_NEXT, EV_HOLD);     // BOOT
    short_long(&btns[1], EV_SELECT, EV_HOLD);   // SELECT button
    if (debounce(&btns[2]) > 0) send(EV_BACK);
    if (debounce(&btns[3]) > 0) send(EV_PREV);
    if (debounce(&btns[4]) > 0) send(EV_NEXT);

    if (++enc_ms >= ENC_PERIOD_MS * 1000 / POLL_US) {
        enc_ms = 0;
        enc_poll();
    }
}

// Quadrature x4 on one PCNT unit, two channels (as the IDF rotary example):
// every edge of A and of B counts, its sign set by the level of the other.
static void enc_init(void)
{
    pcnt_unit_config_t uc = { .low_limit = -ENC_LIMIT, .high_limit = ENC_LIMIT, .flags.accum_count = 1 };
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &enc_unit));
    // 1 us glitch filter: a fast spin (5 rev/s) has 80 us between edges
    pcnt_glitch_filter_config_t fc = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc_unit, &fc));
    pcnt_chan_config_t ca = { .edge_gpio_num = PIN_ENC_A, .level_gpio_num = PIN_ENC_B };
    pcnt_chan_config_t cb = { .edge_gpio_num = PIN_ENC_B, .level_gpio_num = PIN_ENC_A };
    pcnt_channel_handle_t ha, hb;
    ESP_ERROR_CHECK(pcnt_new_channel(enc_unit, &ca, &ha));
    ESP_ERROR_CHECK(pcnt_new_channel(enc_unit, &cb, &hb));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(hb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(hb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // accum_count needs the limits as watch points
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc_unit, -ENC_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc_unit, ENC_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(enc_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(enc_unit));
    // weak internal pull-ups on top of the external 4.7k, in case those are missing
    gpio_pullup_en(PIN_ENC_A);
    gpio_pullup_en(PIN_ENC_B);
    enc_last = 0;
}

void input_init(void)
{
    q = xQueueCreate(16, sizeof(input_ev_t));
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BOOT) | (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B) |
                        (1ULL << PIN_SEL) | (1ULL << PIN_BACK) | (1ULL << PIN_PREV) |
                        (1ULL << PIN_NEXT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    enc_init();

    const esp_timer_create_args_t args = { .callback = poll_cb, .name = "input" };
    esp_timer_handle_t t;
    ESP_ERROR_CHECK(esp_timer_create(&args, &t));
    ESP_ERROR_CHECK(esp_timer_start_periodic(t, POLL_US));
}

input_ev_t input_get(void)
{
    input_ev_t e;
    return xQueueReceive(q, &e, 0) ? e : EV_NONE;
}

int input_enc_steps(bool tune)
{
    int t = atomic_exchange(&tune_steps, 0), m = atomic_exchange(&menu_steps, 0);
    return tune ? t : m;
}

