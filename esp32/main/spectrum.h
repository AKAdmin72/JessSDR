// Main screen: header, ruler, spectrum, waterfall.
#pragma once

#define SPEC_FFT_N 1024

void spectrum_init(void);
#include <stdbool.h>
#include <stdint.h>

void spectrum_draw_static(void);   // header + ruler (after the menu was shown)
// Dial in Hz, step 1..6 (10Hz..1MHz), LO offset = spectrum centre - dial
// (CW: -pitch, else 0). Redraws the readout, and the ruler/markers when moved.
void spectrum_set_dial(uint32_t hz, int step, float lo_offset_hz);
// Mode label ("LSB", "CW"...), filter label ("2.4k"), is_auto = picked by
// the band plan (other colour), passband lo..hi in Hz relative to the dial.
void spectrum_set_mode(const char *name, const char *bw, bool is_auto, float lo_hz, float hi_hz);
// S-meter in dBm (header bar, S9 = -73dBm).
void spectrum_set_smeter(float dbm);
// Volume 0..23 shown in the S-meter slot instead (-1 = back to the S-meter).
void spectrum_show_volume(int v);
// Squelch threshold in dB (0 = OFF) + live SNR bar in the S-meter slot
// (-1 = back to the S-meter); closed = S-meter drawn dim (squelch shut).
void spectrum_show_squelch(int thr_db);
void spectrum_set_squelch_closed(bool closed);
// Squelch measurement, updated per spectrum frame: SNR in the passband (dB
// over the noise per bin), signal and noise per bin in dB (FFT units).
// Returns true once per new frame.
bool spectrum_sq_snr(float *snr_db, float *sig_db, float *noise_db);
// Display: range_db = dB from the noise floor to the top of the spectrum
// panel (the waterfall uses 3/4 of it), window before the FFT (SPEC_WIN_*).
enum { SPEC_WIN_OFF, SPEC_WIN_HANN, SPEC_WIN_BH, SPEC_WIN_N };
void spectrum_set_view(int range_db, int window);
// S-meter, dBm: power in the demodulator passband (FFT), peak-reading.
float spectrum_smeter_dbm(void);
// Main panel layout: spectrum only (default), waterfall only, or both.
// Takes effect with the next frame; call enter_main/spectrum_draw_static after.
enum { SPEC_VIEW_SPECTRUM, SPEC_VIEW_WATERFALL, SPEC_VIEW_BOTH, SPEC_VIEW_N };
void spectrum_set_layout(int view);
// Restart the noise-floor estimate (fast ~1s settle), e.g. after a source change.
void spectrum_floor_reset(void);
// [1 2 1]/4 smoothing across frequency on the spectrum trace (not the waterfall).
void spectrum_set_smooth(bool on);
// Retune: true = shift the averages/waterfall with the dial (SCROLL), false =
// the old way, no shift (STATIC: the picture smears for a moment).
void spectrum_set_scroll(bool on);
// Frame rate readout in the header on/off.
void spectrum_show_fps(bool on);
// ADC overload: clip = clipped within the last second; peak in dBFS (yellow above -6)
void spectrum_set_adc(bool clip, float peak_dbfs);
// One frame. iq = SPEC_FFT_N interleaved I/Q floats, or NULL for the built-in
// synthetic signal. src_name (4 chars) is shown in the header corner.
void spectrum_step(const float *iq, const char *src_name);
// Frame processed without drawing (menu open): keeps the squelch, the
// S-meter and the waterfall history going.
void spectrum_feed(const float *iq);
// No fresh frame (I/Q stale after a retune): redraw the last picture shifted
// to the current dial, no new waterfall row.
void spectrum_hold(void);

// Debug (serial console): average N frames of FFT bin power, then print them.
void spectrum_capture_start(int frames);
bool spectrum_capture_dump(void);     // true once, when the capture is printed
