// User input: encoder + buttons (PLAN.md Stage 4), and the board's BOOT
// button as a stand-in while the encoder is not wired yet.
//   BOOT:    short press = NEXT, hold 0.5s = HOLD
//   encoder: turn = NEXT/PREV, short push = SELECT, hold 0.5s = HOLD
//   buttons: GPIO8 = BACK, GPIO13 = PREV, GPIO14 = NEXT
#pragma once

typedef enum { EV_NONE, EV_NEXT, EV_PREV, EV_SELECT, EV_HOLD, EV_BACK } input_ev_t;

void       input_init(void);
input_ev_t input_get(void);   // non-blocking
