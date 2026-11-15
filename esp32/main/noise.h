// Noise test (MENU > System > Noise test): band noise vs receiver noise.
#pragma once
#include <stdbool.h>

// Start a new measurement (drops the first few blocks: they can be stale).
void noise_reset(void);
// Feed one I/Q block (SPEC_FFT_N interleaved floats, full scale = 1).
// Every NOISE_FRAMES blocks (~2s) it returns true and sets *db to the noise
// floor in dBFS/Hz of the wide I/Q (0 dBFS = |I+jQ| = 1), then starts over.
bool noise_add(const float *iq, float *db);
