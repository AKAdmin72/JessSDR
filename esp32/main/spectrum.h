// Main screen: header, ruler, spectrum, waterfall.
#pragma once

#define SPEC_FFT_N 1024

void spectrum_init(void);
#include <stdbool.h>
#include <stdint.h>

void spectrum_draw_static(void);   // header + ruler (after the menu was shown)
// Dial in Hz, step 1..6 (10Hz..1MHz), LO offset K (LO = dial + K*fs/16).
// Redraws the readout, and the ruler/markers when the dial moved.
void spectrum_set_dial(uint32_t hz, int step, int lo_shift_k);
// Demodulator (FPGA_MODE_*) for the header and the passband shading;
// is_auto = picked by the band plan (shown in another colour).
void spectrum_set_mode(int mode, bool is_auto);
// Display: range_db = dB from the noise floor to the top of the spectrum
// panel (the waterfall uses 3/4 of it), window = Hann before the FFT.
void spectrum_set_view(int range_db, bool window);
// Frame rate readout in the header on/off.
void spectrum_show_fps(bool on);
// ADC overload: clip = clipped within the last second; peak in dBFS (yellow above -6)
void spectrum_set_adc(bool clip, float peak_dbfs);
// One frame. iq = SPEC_FFT_N interleaved I/Q floats, or NULL for the built-in
// synthetic signal. src_name (4 chars) is shown in the header corner.
void spectrum_step(const float *iq, const char *src_name);

// Debug (serial console): average N frames of FFT bin power, then print them.
void spectrum_capture_start(int frames);
bool spectrum_capture_dump(void);     // true once, when the capture is printed
