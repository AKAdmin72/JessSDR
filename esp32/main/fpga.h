// Link to the Tang Nano 20K (PLAN.md Phase 3).
//   SPI (ESP = master, 1MHz, mode 0): control registers, spi_regs.v
//   I2S (FPGA = master, 105.54kHz, 32-bit stereo): wide DDC I/Q, iq_i2s_tx.v
// Register map = FpgaRX's, as implemented by sdr_esp/spi_regs.v (Tang side:
// sdr_esp/sdr_esp_top.v). IQC_* read 0, LO_SHIFT reads 0, LEVEL added.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define FPGA_REG_ID       0x00   // reads FPGA_ID
#define FPGA_REG_VERSION  0x01
#define FPGA_REG_SCRATCH  0x02   // free byte for the link test
#define FPGA_REG_CTRL     0x03   // bit0 = I2S test pattern, bit1 = I/Q correction
#define FPGA_REG_FREQ     0x04   // 4 bytes, dial as 8 packed BCD digits, MSB first
#define FPGA_REG_LO_FW    0x08   // 4 bytes RO, DDC NCO phase increment (64.8MHz NCO)
#define FPGA_REG_LO_SHIFT 0x0C   // RO, signed, low-IF: LO = dial + K * fs/16
#define FPGA_REG_MODE     0x0D   // demodulator, FPGA_MODE_*; also sets K
#define FPGA_REG_IQC_C1   0x0E   // 2 bytes RO, iq_corr c1, signed Q0.15 = -(A_Q/A_I) sin(phi)
#define FPGA_REG_IQC_C2   0x10   // 2 bytes RO, iq_corr c2, signed Q1.14 = (A_Q/A_I) cos(phi)
#define FPGA_REG_IQC_STAT 0x12   // RO, bit0 = coefficients valid, bit1 = signal present now
#define FPGA_REG_NR       0x13   // audio_nr: bits 1:0 FPGA_NR_*, bits 3:2 level (mu = 2^-(10+level))
#define FPGA_REG_PEAK     0x14   // RO, max |ADC code - 2048| >> 3 over 0.26s, 255 = full scale / OTR
#define FPGA_REG_LEVEL    0x15   // RO, S-meter level (level_meter.v, 6.02dB per 8)
#define FPGA_REG_BTN      0x16   // RO, Tang S1/S2 event counters {s2_long, s2_short, s1_long, s1_short}, 2 bits each
#define FPGA_PEAK_CLIP    240    // >= -0.6dBFS: the ADC clipped
#define FPGA_NR_OFF       0
#define FPGA_NR_NR        1
#define FPGA_NR_ANF       2
#define FPGA_ID           0x5A
#define FPGA_CTRL_I2S_TEST 0x01
#define FPGA_CTRL_IQ_CORR  0x02

// Demodulator modes (spi_regs MODE). FpgaRX moved its LO by K * fs/16 per
// mode (low IF); the direct-sampling DDC is centred on the dial: K = 0.
#define FPGA_MODE_LSB     0
#define FPGA_MODE_USB     1
#define FPGA_MODE_AM      2
int fpga_mode_k(int mode);

#define FPGA_FS_HZ        105537.46f   // 64.8MHz / 614

void    fpga_init(void);
// Every transaction is followed by a WS2812 "black" frame (Tang pin 79 =
// MOSI = RGB LED DIN). Around bursts: hold(true) ... hold(false) sends it once.
void    fpga_led_hold(bool on);
uint8_t fpga_read(uint8_t addr);
void    fpga_write(uint8_t addr, uint8_t val);
uint32_t fpga_read_u32(uint8_t addr);        // 4-byte register, MSB first

void     fpga_set_freq(uint32_t hz);         // dial frequency -> FREQ (BCD)
uint32_t fpga_get_freq(void);                // FREQ read back, in Hz
// Phase increment the FPGA should compute for this dial (sdr_esp_top.v
// formula); compare with FPGA_REG_LO_FW.
uint32_t fpga_expected_lo_fw(uint32_t hz, int lo_shift_k);

// I2S receive statistics (updated by the receive task on core 1).
typedef struct {
    uint32_t frames;        // frames received since the last reset
    uint32_t pattern_err;   // test mode: right != ~left
    uint32_t count_jumps;   // test mode: counter did not advance by exactly 1
    uint32_t last_left, last_right;
    int64_t  t_reset_us;    // block boundary where counting started
    int64_t  t_last_us;     // end of the last counted block
} fpga_i2s_stats_t;

// Frame rate over the counted blocks (0 until two blocks have arrived).
float fpga_i2s_rate(const fpga_i2s_stats_t *s);

void fpga_i2s_test_mode(bool on);         // checks the pattern while on
void fpga_i2s_stats_reset(void);
void fpga_i2s_stats(fpga_i2s_stats_t *s);

// Latest I/Q block as SPEC_FFT_N interleaved floats (full scale = 1.0).
// Returns false if no new block arrived since the last call.
bool fpga_iq_block(float *dst, int n);
