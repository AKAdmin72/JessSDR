// User input: optical encoder + buttons (PLAN.md Stage 4), and the board's
// BOOT button as a spare.
//   encoder: LPD3806-600BM (600 lines, A/B on GPIO17/18, NPN open collector,
//            4.7k pull-ups to 3V3, powered from 5V). Counted in hardware
//            (PCNT, x4 = 2400 counts/rev), read with input_enc_steps().
//   SELECT:  separate button on GPIO8: short = SELECT, hold 0.5s = HOLD
//   BOOT:    short press = NEXT, hold 0.5s = HOLD
//   buttons: GPIO12 = BACK, GPIO13 = PREV, GPIO14 = NEXT
#pragma once
#include <stdbool.h>

typedef enum { EV_NONE, EV_NEXT, EV_PREV, EV_SELECT, EV_HOLD, EV_BACK } input_ev_t;

void       input_init(void);
input_ev_t input_get(void);   // button events, non-blocking

// Encoder steps since the last call (signed, + = clockwise).
//   tune = true:  dial steps, 50 per revolution; faster turning multiplies
//                 them (speed smoothed ~80 ms: x2 above 1 rev/s, x4 above 2)
//   tune = false: menu steps, 20 per revolution, no acceleration
// Each call clears both kinds, so switching screens never jumps.
int        input_enc_steps(bool tune);
