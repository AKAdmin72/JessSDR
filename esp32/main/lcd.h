// ST7796S 480x320 landscape (4" SPI module, 74LVC245A buffer) on SPI2 (pins
// in lcd.c, see PLAN.md Phase 3). Drawing goes through two DMA line buffers:
// while one is being sent, the CPU fills the other.
#pragma once
#include <stdint.h>
#include "font.h"

#define LCD_W        480
#define LCD_H        320
#define LCD_CHUNK    20                      // lines per DMA buffer

// ST7796 wants big-endian RGB565; esp_lcd sends the buffer bytes as is.
#define RGB565_(v)    ((uint16_t)(((v) >> 8) | ((v) << 8)))
#define RGB565(r, g, b) RGB565_((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

void      lcd_init(void);
void      lcd_backlight(int percent);
uint16_t *lcd_buf_get(void);                 // waits for a free buffer
void      lcd_buf_send(int x, int y, int w, int h);  // sends the last buffer got
void      lcd_fill(int x, int y, int w, int h, uint16_t c);
// Text on a solid background; returns the width drawn.
int       lcd_text(int x, int y, const font_t *f, const char *s, uint16_t fg, uint16_t bg);

// Bring-up knobs (serial console "lcd"): MADCTL = orientation + RGB/BGR,
// colour inversion on/off. Applied at once; the defaults are in lcd.c.
void      lcd_set_madctl(uint8_t v);
void      lcd_set_invert(int on);
uint8_t   lcd_get_madctl(void);
int       lcd_get_invert(void);
