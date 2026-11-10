// JessSDR boot splash: the picture from tools/gen_splash.py (palette + RLE,
// splash_img.h), decoded straight into the LCD DMA buffers, LCD_CHUNK lines
// at a time.
#include "lcd.h"
#include "splash.h"
#include "splash_img.h"

void splash_show(void)
{
    int run = 0, left = 0;
    uint16_t col = 0;
    for (int y0 = 0; y0 < SPLASH_H; y0 += LCD_CHUNK) {
        int n = SPLASH_H - y0 < LCD_CHUNK ? SPLASH_H - y0 : LCD_CHUNK;
        uint16_t *b = lcd_buf_get();
        for (int i = 0; i < SPLASH_W * n; i++) {
            if (left == 0 && run < SPLASH_NRUNS) {
                left = splash_rle[2 * run];
                col = splash_pal[splash_rle[2 * run + 1]];
                run++;
            }
            b[i] = col;
            if (left) left--;
        }
        lcd_buf_send(0, y0, SPLASH_W, n);
    }
}
