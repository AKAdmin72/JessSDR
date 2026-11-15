# JessSDR — plan

## Phase 1 — ADC tests (`adc_test/`)
1. [done] Pinout for 20K, KiCad wiring schematic, scope bitstream builds:
   AD9226 @ 13.5 MSPS (27 MHz / 2, no PLL) → trigger → waveform on TFT.
   S1/S2 = timebase slower/faster (decimation 1..32768). Red trace = OTR.
2. Hardware bring-up: verify module header order + 3.3V logic level, power,
   generator sine → clean waveform; find bit order (BIT_REVERSE) and
   whether the display waveform is inverted.
3. UART sample dump (pins 69/70, ~2–3 Mbaud) of a full-rate block → PC
   script: histogram (missing codes / stuck bits), FFT, SNR/ENOB, noise
   floor with input terminated in 50 Ω, measured input full-scale.
4. PLL → 54–65 MSPS with adjustable capture phase; repeat 3.

## Phase 2 — 40m SDR (direct sampling)
Front end: PicoRX LPF + LTC6226 (+15 dB), optionally the AD8138 gain mod on
the module; spare LNA as reserve. FPGA: NCO + I/Q mixer at ~7.1 MHz →
CIC decimation → FIR → SSB demod → onboard I2S amplifier (pins 54/55/56/51)
→ speaker; spectrum/waterfall on TFT; encoder on the ESP32-S3. Reuse DSP
blocks from ../tang_nano_9k_projects/FpgaRX-SDR where they fit.

## Wiring (Tang Nano 20K header pin → device)
Perfboard layout (chosen for the shortest wires):
```
              [ TFT ]            <- above the Tang's top-right corner
  ESP32-S3  |  Tang 20K  |  AD9226 (SMA →)
            HDMI end up, USB-C down
```
**J5 = left row** (ESP side), **J6 = right row** (ADC side), pin 20 of both
rows at the top. Full diagram: `hardware/jesssdr/jesssdr.pdf`.

| AD9226 P2 (+5V row at top) | Tang | FPGA |
|---|---|---|
| D0 / D1 / D2 / D3 | J6-14 / 13 / 12 / 11 | 31 / 30 / 29 / 26 |
| D4 / D5 / D6 | J6-10 / 9 / 8 | 25 / 28 / 27 |
| CLK | J6-7 | 16 |
| OTR | J6-6 | 15 |
| D7 / D8 / D9 / D10 / D11 | J6-5 / 4 / 3 / 2 / 1 | 77 / 85 / 75 / 74 / 73 |
| +5V / GND | J5-1 / J5-2 (under the Tang) | — |

| TFT pin | Tang | FPGA |
|---|---|---|
| 1 VCC, 8 LED | J6-19 (3V3) | — |
| 2 GND | J6-20 | — |
| 3 CS | J5-18 | 71 |
| 4 RESET | J6-18 | 18 |
| 5 D/C | J6-17 | 19 |
| 6 SDI(MOSI) | J6-16 | 20 |
| 7 SCK | J6-15 | 17 |
| 9–14 | not connected | — |

| ESP32-S3-DevKitC (N16R8), right row J3 | Tang | FPGA |
|---|---|---|
| SPI SCK = GPIO1 (J3-4) | J5-19 | 53 |
| SPI MISO = GPIO2 (J3-5) | J5-18 | 71 (TFT CS until the TFT moves to the ESP) |
| I2S WS = GPIO42 (J3-6) | J5-17 | 72 |
| SPI MOSI = GPIO39 (J3-9) | J5-14 | 79 |
| I2S BCK = GPIO47 (J3-17) | J5-6 | 41 |
| I2S DATA = GPIO21 (J3-18) | J5-5 | 42 |
| SPI CS_N = GPIO20 (J3-19) | J5-4 | 80 |
| GND (J3-21) | J5-2 | — |

Revised 2026-11-08 from the perfboard photo: the ESP's right row J3 faces
J5 and is 2 pins longer at the top, so J5-n sits opposite J3-(23-n) - every
link wire is a straight jumper. J5-1 (+5V) faces J3-22 (GND): never bridge.
GPIO20 = native USB D+ -> the ESP console moved to the COM port (UART0).
microSD slot must stay empty (ADC D8 and ESP BCK use SD lines).

## Phase 3 — ESP32-S3 UI (decided 2026-11-07)
Same split as ../tang_nano_9k_projects/FpgaRX-SDR stage 4: the Tang keeps the whole radio path
(ADC → DDC → SSB → NR/ANF → AGC → I2S amp); the ESP32-S3 does the display,
encoder/buttons, menu (NVS), FFT spectrum + waterfall. Reason: FpgaRX's ESP
spectrum/menu/NR were much nicer, and the Tang's BSRAM is 41/46 full mostly
because of the FFT/waterfall/TFT.

Hardware: a new ESP32-S3 board of the same type as FpgaRX's (44-pin
DevKitC, N8R2); display **4" SPI 480x320** (assumed ST7796S - check the
marking; ILI9488 would need RGB666), same header pinout as the 2.8"
ILI9341. ESP pins (2026-11-08, board is N16R8 = octal PSRAM on 33..37):
LCD CS/RST/DC/MOSI/SCK/LED = 4/5/6/7/15/16 (left row J1, header order),
FPGA link on the right row J3 (table above), optical encoder A/B 17/18 (LPD3806-600BM, pull-ups to 3V3), SELECT 8,
buttons BACK/PREV/NEXT 12/13/14. Debug console = COM port (UART0).
ESP gets its own LC-filtered 5V; Wi-Fi off.

Link:
- SPI (ESP master, mode 0): register file `spi_regs.v` (from FpgaRX, sampled
  with the 64.8 MHz clock). Registers: ID, VERSION, SCRATCH, CTRL (bit0 I2S
  test pattern), FREQ (dial, 4 bytes), MODE, NR, PEAK (ADC peak), GAIN.
- I2S (FPGA master, 32-bit stereo): the **wide** DDC output, 105.54 kHz
  (64.8 MHz / 614) → ESP FFT 2048 (51.5 Hz bins), spans ±12/±25/±37 kHz.
  BCK = 64 × fs = 64.8 MHz × 64/614 via a fractional divider (±1 clk jitter).
  No LO offset (K = 0): the DDC is centred on the dial.

Steps (each one leaves a working receiver; check the noise floor after 4):
1. Wiring: ESP + 4" display, filtered supply. Schematic: `hardware/jesssdr/jesssdr.pdf`
   (`hardware/gen_schematic.py`).
2. ESP standalone: FpgaRX firmware ported to 480x320 (ST7796), SIM source.
   The Tang keeps its own TFT build meanwhile (display unplugged).
   Status 2026-11-08: works on hardware (N16R8, display OK, MADCTL E8).
   Was: 2026-11-07 done in `esp32/`, builds (IDF 6.0). Display = ST7796S (user confirmed; 74LVC245A buffer on the
   module). Own minimal driver in lcd.c (esp_lcd panel IO + CASET/RASET/
   RAMWR), MADCTL 0xE8 / no inversion assumed - console `lcd m <hex>`,
   `lcd i 0|1` to fix on the board. FS = 105.54 kHz, K = 0, no DC zone in
   the floor envelope, I/Q correction removed. No FPGA ID at boot ->
   standalone SIM, no register polling.
3. SPI: spi_regs.v on the Tang; the encoder tunes; tb_spi_regs first.
4. I/Q: iq_i2s_tx.v (105.54 kHz) + ESP I2S slave; spectrum/waterfall on the
   ESP. Compare the noise floor with and without the ESP running.
5. FPGA clean-up: drop TFT/FFT/waterfall/floor_eq, record freed BSRAM.
6. NR/ANF: port ../tang_nano_9k_projects/FpgaRX-SDR/stage1/audio_nr.v (menu Audio > NR/Level).
7. Later: filter width, CW, USB audio + CAT (WSJT-X), Wi-Fi.

Status 2026-11-10: steps 3-6 written as `sdr_esp/` (new top, the Tang-only
`sdr/` stays as it is): spi_regs + BCD tuning, I2S I/Q link (105.54 kHz,
BCK 6.75 MHz), TFT/FFT/waterfall/floor_eq dropped (BSRAM 41 -> 18/46),
audio_nr + am_demod added. All tbs pass, Gowin build clean (Fmax 81.8 MHz).
Next: flash both, check the ESP boot self-test log (SPI/LO/modes/I2S PASS),
then the noise floor with/without the ESP.
