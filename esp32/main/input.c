#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "input.h"

#define PIN_BOOT   0
#define PIN_ENC_A  17
#define PIN_ENC_B  18
#define PIN_ENC_SW 8
#define PIN_BACK   12
#define PIN_PREV   13
#define PIN_NEXT   14

#define POLL_US      1000
#define DEBOUNCE_MS  20
#define LONG_MS      500

static QueueHandle_t q;

typedef struct { int pin; int stable, cnt; int held_ms; bool long_sent; } btn_t;
static btn_t btns[] = {
    { .pin = PIN_BOOT, .stable = 1 }, { .pin = PIN_ENC_SW, .stable = 1 },
    { .pin = PIN_BACK, .stable = 1 }, { .pin = PIN_PREV, .stable = 1 }, { .pin = PIN_NEXT, .stable = 1 },
};
static int enc_state, enc_acc;

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

static void poll_cb(void *arg)
{
    short_long(&btns[0], EV_NEXT, EV_HOLD);     // BOOT
    short_long(&btns[1], EV_SELECT, EV_HOLD);   // encoder push
    if (debounce(&btns[2]) > 0) send(EV_BACK);
    if (debounce(&btns[3]) > 0) send(EV_PREV);
    if (debounce(&btns[4]) > 0) send(EV_NEXT);

    // encoder: Gray-code state machine, one event per detent (4 steps)
    static const int8_t tbl[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    int s = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    enc_acc += tbl[(enc_state << 2) | s];
    enc_state = s;
    if (enc_acc >= 4)  { enc_acc = 0; send(EV_NEXT); }
    if (enc_acc <= -4) { enc_acc = 0; send(EV_PREV); }
}

void input_init(void)
{
    q = xQueueCreate(16, sizeof(input_ev_t));
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BOOT) | (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B) |
                        (1ULL << PIN_ENC_SW) | (1ULL << PIN_BACK) | (1ULL << PIN_PREV) |
                        (1ULL << PIN_NEXT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    enc_state = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);

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
