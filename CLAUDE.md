# JessSDR — notes for Claude

Project renamed 2026-11-10 from `tang20k_adc_9226` (after Jessica, the
user's cat) and moved from `~/Development/tang_nano_9k_projects/` to
`~/Development/JessSDR`; sibling projects are now under
`../tang_nano_9k_projects/`. Boot splash: `esp32/tools/gen_splash.py N`
style 6 = the user's logo `esp32/tools/jessica_logo.png` (2026-11-15,
drawn head + radio waves; cut out onto the dark starry background, outer
contour + waves recoloured light blue - user disliked the cream one) -
current. Styles 1-5 = her photo cutout (`jessica_cutout.png`, cv2.grabCut)
as photo/cartoon/oil/pencil/pixel. -> `esp32/main/splash_img.h` (palette +
RLE, ~90 KB), shown by splash.c during the link self-test (>= 2.5 s).
Display turned 180 deg 2026-11-10: MADCTL 0x28 (was 0xE8).
Noise test (2026-11-15, NOT yet run on hardware): MENU > System > Noise test
(noise.c): wide-I/Q floor in dBFS/Hz (Hann FFT 1024, 200 frames, 10th
percentile of +-26 kHz), slots "50 ohm"/"Antenna" in NVS (nz_load/nz_ant),
difference + verdict on screen, console `nz`. No PC needed at the antenna.
Main panel view (2026-11-15, menu Display > View, NVS "view"): SPECTRUM
(default, 254 px) / WATERFALL (254 px, dashed red dial line) / BOTH (120+134);
waterfall ring is 254 rows in PSRAM, always filled -> no history lost on switch.
Retune without smear (2026-11-19): spectrum.c
state_follow() shifts px_avg/px_wf/est by whole pixels to the new centre
(st_center), edge columns "cold" until a fresh frame; waterfall rows keep
wf_ctr[] and are drawn offset (history stays put). fpga_iq_block() returns
FPGA_IQ_STALE for blocks ending < 20 ms after fpga_set_freq -> main.c calls
spectrum_hold() (shifted redraw, no wf row). SIM input: no shift.
2026-11-21 review rework (ESP; flashed + user-approved 2026-11-22, release
working-2026-11-22; next: docs/flat_spectrum_plan.md = CFIR in FPGA): only the middle +-35 kHz shown (SPAN_HZ;
the wide CIC4 aliases -27 dBc at 33k, 0 dBc at 52.7k; fractional bin->pixel
weights), scroll shifts only the level ABOVE the floor (px_avg-env), est/env
stay in screen coordinates (floor shape = CIC droop, tied to the screen; a
jump >= half a screen restarts the floor), waterfall edges = row noise
(wf_cap). Menu Display > Retune SCROLL/STATIC (NVS "rtscr"). EQ_SLOPE 1 dB/kHz
(was 1.94: AM lost ~8.7 dB). Window OFF/HANN/BH (default OFF = user choice).
S-meter = FFT passband power (Parseval, FPGA_DBM_1LSB -115.7) peak-held,
not the FPGA LEVEL (+-6 kHz incl. neighbours). Menu open: spectrum_feed()
keeps squelch/S-meter/waterfall running. fpga.c: block time = arrival of
its last sample (s_end), IQ_CLEAN_US 16 ms. AM shading +-6 kHz.
Squelch (2026-11-19, variant 2 of docs/squelch_idea.md, flashed, "sq" console
read ~0 +-2 dB on noise): spectrum.c sq_measure() = mean bin power in pb_lo..
pb_hi / (p10 of bins +-26 kHz / 0.10536); main.c squelch_update(): open >= thr,
close < max(thr-2, 1.5) after 0.8 s hang, mute = VOL 0 ramped (sq_muted ->
vol_reg()). Tang S1 long on main = EV_SQL -> encoder sets thr 5 s (header
"SQ nn" + SNR bar); Audio > Squelch OFF/3..30 dB, NVS "sql". Shut = dim S-meter.

Goal: Sipeed Tang Nano 20K + AD9226 ADC module (+ ILI9341 TFT). Phase 1 =
ADC bring-up/tests (`adc_test/`), phase 2 = 40m (7.0–7.2 MHz) direct-sampling
SDR. See `PLAN.md`.

All Verilog/Gowin conventions from `../tang_nano_9k_projects/FpgaRX-SDR/CLAUDE.md` apply here
(one `always` per reg, no reset network, single-cycle pulses, BSRAM
inference pattern, signed-concat pitfall, iverilog tb per module, `#1` in
tb blocking drives). Reused modules are copied verbatim from
`../tang_nano_9k_projects/oscilloscope/` (trigger, ring_buffer, waveform_draw, waveform_framebuffer,
line_draw, ili9341_driver) — don't "improve" them casually, they're proven.

## Git / releases
Git repo since 2026-11-08 (branch `main`). Hardware-verified builds go to
`releases/` (+ README table, sha256) with a tag `working-YYYY-MM-DD`, same
scheme as ../tang_nano_9k_projects/FpgaRX-SDR. Tang-only receiver (no ESP32): tag
`working-2026-11-07` / branch `tang-only`, `./flash_tang_only [sram]`.

## Board: Tang Nano 20K (NOT 9K)
- Part `GW2AR-LV18QN88C8/I7`, `-device_version C`; 27 MHz clock on pin 4.
- Flash with `openFPGALoader -b tangnano20k` (`./flash`, `./flash flash`).
- UART to PC: FPGA pins 69 (TX) / 70 (RX) via the onboard BL616; the
  udev rule unbinds ftdi_sio, so use `../tang_nano_9k_projects/term.py` (libftdi, interface B).
- Buttons S1=88, S2=87 are active-HIGH.
- Official schematic: `docs/Tang_Nano_20K_3923_Schematics.pdf` (sheet 1 has
  headers J5/J6). Pin choice rules and every excluded pin are documented in
  `adc_test/adc_test.cst` — check there before picking a new pin.

## Schematic = single source of truth (one file, one sheet)
`hardware/gen_schematic.py` -> `hardware/jesssdr/` (.kicad_sch + .pdf, A2):
the whole receiver as built - antenna/BPF, AD9226 + its AD8138 input stage
(IW5ALZ mod, as built), Tang 20K, ESP32-S3 DevKitC, 4" ST7796S, encoder,
buttons, ESP supply D1/LC. All pin tables live there (Tang J5/J6, P2, ESP
J1/J3 = esp32/main/*.c); the .cst files must match. After changing pins: edit
the script, re-run it, `kicad-cli sch erc hardware/jesssdr/jesssdr.kicad_sch`
(must be 0 violations). The old separate drawings (wiring/ = Tang-only with
ILI9341, system/, frontend_mod/, docs/wiring_layout.png) were merged into it
2026-11-14; the Tang-only wiring is in tag working-2026-11-07.

## AD9226 module facts / open questions
- Black single-channel "AD9226 12-BIT 65MSPS" board, 5V, SMA input, 2x10
  header P2 (order read from seller photo — UNVERIFIED).
- Front end U3 is most likely AD8138, stock gain ~0.38 → ±5V input range
  (IW5ALZ, hackaday.io/project/186329 logs 213303/213304; his mod: gain
  6.64, +24.9 dB). Needed for 40m sensitivity — measure stock gain first.
- Bit order (MSB side) unknown → `adc_test_top` parameter `BIT_REVERSE`.
- Gain mod (IW5ALZ: Rser 200→C 100n, RG 200→360 (as built; IW5ALZ 390), RF
  200→2.2k, RBAL 50→22): drawn as built in the frame "Вход AD9226" of
  `hardware/jesssdr` (draw_frontend()). Topology from his formulas. User probed 2026-11-07:
  RG2 → RBAL 50 Ω → GND (balances 62||(200+50)=49.7); board R-numbers unverified.
  MOD DONE on the real board 2026-11-07 (RF 2k2, RG 360, RBAL 22, C 100n):
  user reports reception much better. Expected gain +25.4 dB vs stock -> the
  -121 dBm/Hz floor and sdr dBm constant (-135.7) are now stale - re-measure.

## Build/sim
`tft_test/` = standalone TFT tests, SRAM only: `./build bars`, `./build mandel
[27|54|67]` (PLL MHz = SPI SCK = compute clock), `./flash` same args.
mandel = 4x mandel_engine (Q4.32, 1 iter/clk, 3 MULT36X36 each -> DSP 24/24),
round-robin dispatch, no framebuffer; Fmax ~69-72 MHz. tft_test's
ili9341_driver.v copy has a CLK_KHZ param (init delays). Full-system tb:
`sim/tb_mandel_top.v` (+ sim/rpll_sim.v, -DFAST_SIM) checks frame 0 vs a
reference. TFT module has J1 shorted (662K LDO bypassed) -> VCC 3.3V only.
`zx_demo/` = ZX-Spectrum-style dancer demo (no sound - user didn't want
music): `./build && ./flash` (SRAM). 67.5 MHz PLL; beat_gen (125 BPM) ->
dancer.v (FK skeleton, brush_line) into double-buffered 1bpp fb_ram (BSRAM),
attribute plasma/floor/scroller composed per pixel in zx_demo_top.v. Poses,
skeleton, font (console VGA8), texts: gen_tables.py -> tables.v. ROMs must
be function+assign, not always @* (iverilog never evaluates an always @*
whose inputs stay at their time-0 value). tb: sim/tb_dancer.v (pose sheet),
sim/tb_zx_demo.v (full frame -> sim/frame.ppm, -DFAST_SIM + sim/rpll_sim.v).
`adc65/` = AD9226 at 64.8 MSPS (rPLL 27*12/5; ADC CLK = CLKOUTP with
dynamic phase PSDA, DUTYDA=PSDA+8 keeps 50%) + 16K-sample BSRAM dump over
UART 1 Mbaud (pins 69/70). PC tool: `python3 tools/adc65.py info|cap|sweep|phase`
(numpy analysis: noise, tone, SNR w/o harmonics, dBm/Hz). Phase sweep
2026-11-03: capture errors only at phases 5-6 -> PHASE_INIT 13. BL616 bridge
drops/holds the first bytes after the port opens -> tool pings '?' until a
reply (Link.sync). Measured noise floor ~-121 dBm/Hz (stock front end, 50 ohm
input, FS ~+24 dBm). NanoVNA output is a square wave (SINAD ~6.3 dB is just
its harmonics) - use CW mode, not sweep, for SNR.
`sdr/` = 40m SDR v1 (2026-11-03, works on hardware): ddc.v (NCO 4096x16
BSRAM ROM nco_sin.hex, I=x*cos Q=-x*sin, CIC4 R=1229 -> 52.726 kHz, >>47)
-> ssb_demod.v + audio_agc.v copied verbatim from ../tang_nano_9k_projects/FpgaRX-SDR/stage1 (built
for 52.734 kHz) -> audio_i2s.v (linear interp to 48.214 kHz, BCLK clk/21) ->
onboard MAX98357A (pins 56 BCLK/55 LRCK/54 DIN/51 PA_EN; needs
`set_option -use_sspi_as_gpio 1`). USB = negate Q. level_meter (pipelined -
was the 59 MHz critical path). UI: S1/S2 short = +-step, S1 long = step,
S2 long = LSB/USB. UART: tools/sdr.py info|freq|mode|iq|audio. tbs:
sim/tb_ddc, tb_chain (87 dB opposite sideband), tb_audio_i2s, tb_misc,
tb_sdr_top (screen.ppm). Fmax 93 MHz @ 64.8.
v2 (same day): spectrum + waterfall. spectrum.v wraps fft512.v (512-pt
radix-2 DIT, in-place 28-bit, one shared 28x17 multiplier, Blackman-Harris
fft_win.hex + Q14 fft_tw.hex from gen_tables.py; ~25k clk/FFT) on the DDC
I/Q -> |X| max+3/8min -> L8 = 16*log2|X| (0.376 dB) -> mean of 8 frames ->
line (x 0..319 = bins -160..159, 102.98 Hz, dial at x 160). sdr_top: spm
(L8 per column, px = (L8-sbase)*3/8 = 1 dB/px, 76 px) + wfm 32768x8
(102 rows x 320, v = L8-wbase, palette() in sdr_top), newest row on top.
UART 's' = spectrum line dump, 'b' sb wb = bases (tools/sdr.py spec|base);
dBm per bin ~ -135.7 + 0.3763*L8. Capture buffer now 2048. Fmax 82.6 MHz,
BSRAM 38/46. tbs: sim/tb_fft512 (+check_fft.py vs numpy), tb_spectrum.
Written to flash 2026-11-03 (replaces adc_test there).
v3: floor_eq.v = automatic noise floor (FpgaRX-SDR spectrum_eq ported: per-
bin est snaps down / rises RISE=2 L8/line, +-1/bin lower envelope, out =
L - env + BASE 94) between spectrum and spm/wfm (user: LNA lifted the floor
1/3 screen). Recovers from +20 dB in ~2 s. UART 'e' 0/1 toggles it; 's'
still dumps raw L8. Waterfall row latched at line start (e_row/e_top)
because floor_eq output lags 2 clk. tb: sim/tb_floor_eq. Fmax 91, BSRAM 87%.
v3 written to flash 2026-11-03.
v4 (2026-11-07, after the AD8138 gain mod): S-meter offset GAIN_DB=25 is now
runtime reg gain_db (UART 'g', sdr.py gain); spectrum scale zoom 0/1/2 =
1/1.5/2 px/dB (UART 'z', sdr.py zoom, default 1). sdr.py FE_GAIN_DB=25 in
SPEC_DBM0. Wide span (default, UART 'w', sdr.py wide): ddc.v 2nd output from
the same integrators, R2=614 -> 105.54 kHz, own 66-bit combs, >>43; spectrum
+-33 kHz (205.96 Hz/bin), NAVG 16 (navg_half=~wide), scale_rom 128 chars
{wide,char}, pband 147..158/162..173. '?' byte3 = {wide,usb}. glyph_q
register cut the font path (Fmax 69.7 -> 78.9). tb_ddc_wide (+20k/+5k tones),
tb_spectrum, tb_sdr_top (+10 kHz tone on the tick) ok. BSRAM 41/46.
v4 written to flash 2026-11-07.
`sdr_esp/` = SDR v5 for the ESP32-S3 UI (2026-11-10, PLAN.md phase 3 steps
3-6; builds + sims pass, NOT yet hardware-tested). No TFT/FFT/waterfall/
buttons on the Tang (pin 71 = ESP MISO). Shared DSP files stay in ../sdr
(build adds them from there, copies nco_sin.hex). New: spi_regs.v (FpgaRX
copy + VERSION param 5 + 0x15 LEVEL; FREQ BCD -> binary in the top, LO_FW =
64.8 MHz phase_inc = (f*69500145)>>20, LO_SHIFT/IQC = 0), i2s_link_clk.v
(BCK = clk*64/614 fractional, 4/5-clk halves, frame = exactly 614 clk = ddc
wide rate) + iq_i2s_tx.v (FpgaRX verbatim) on the wide I/Q, DATA 2 clk after
the BCK fall; am_demod.v + audio_nr.v (FpgaRX verbatim; AM = MODE 2/3 on
ssb_demod's dec I/Q, NR after AGC). PEAK = max|code-2048|>>3 per 2^24 clk,
OTR -> 255. UART subset ?/f/m/q/a + 'v' (wide I/Q dump). Fmax 81.8, BSRAM
18/46, DSP 8/24. tbs: sim/tb_spi_regs, tb_i2s_link, tb_am_demod,
tb_audio_nr, tb_sdr_esp_top (~10 min). ESP fpga.c expected LO_FW updated.
NR rework 2026-11-15 (user: NR too aggressive, highs gone, weak stations
inaudible; Python model confirmed: out=y lost -16/-21dB at 1-2/2-2.7kHz vs
noise -13dB, and 6-15dB loudness): NR out = y + residual mix (Level 1..4 =
e/2,e/4,e/8,0; mu fixed 2^-11; ANF unchanged) + 2nd audio_agc (G_MAX 8x)
after NR/ANF (bypassed when off). ESP NVS key nrl3, default Level 2.
v7 (2026-11-21, VERSION 7): audio_up4.v (x4 polyphase FIR 96 taps,
gen_up4_coef.py, >= 77 dB stop from 7.2 kHz) between VOL and audio_i2s
(IN_PERIOD 1229): images of the audio at 13.18k+-f were only ~21 dB down,
now worst spur -49 dB (sim/tb_audio_up4 + check_up4.py, tb_audio_out +
check_audio_out.py). Fmax 83.6, BSRAM 19/46, DSP 10.5/24.
v7 + ESP written to flash 2026-11-22, user approved -> working-2026-11-22.
v6 (2026-11-15, VERSION 6): sdr_esp forks ssb_demod.v (programmable 255-tap
complex filter in BSRAM, written by the ESP: spi_regs CF_ADDR 0x19 + 5-byte
CF_DATA 0x1A..0x1E bursts; power-up cf_coef.hex = LSB 300-2700 from
gen_cf_coef.py; same formula as fpga.c fpga_set_filter) and audio_agc.v
(speed port: fast/med/slow); VOL 0x17 (Q1.7 before audio_i2s), AGC 0x18.
build uses these forks, not ../sdr's. tb_ssb_filter (LSB -109 dB opposite,
CW500 -84 dB @800 Hz). NR output stage pipelined (Fmax 65.8 -> 81).
ESP: CW mode (UI-only: FPGA USB, LO = dial - 700, filter 1k/500/250 round
700 Hz), SSB filters 1.8/2.4/2.7/3.0k + IF shift +-1 kHz, AGC menu, volume
(BACK on main = encoder volume 3 s, menu Audio > Volume; 2 dB steps, 20 = 0 dB),
Freq cal ppm (System; LO = f/(1+ppm)), S-meter in the header (LEVEL ->
dBm approx -90.7+6.02L/8-25, S9 = -73). SCRATCH = 0xC5 marks "settings
sent"; a mismatch (FPGA restart / link test) resends all incl. filter taps.
Flashed (both) 2026-11-10, ESP self-test all PASS. Later same day: Tang S1/S2
= encoder stand-in (buttons.v -> spi_regs 0x16 BTN, 2-bit short/long counters,
ESP main.c tang_buttons() polls 30 ms: S1 short/long = NEXT/SELECT, S2 short =
PREV, S2 long = menu/BACK; tb_buttons). WS2812 on pin 79 (= MOSI): fpga.c sends
a black WS2812 frame (CS-less 2.5 MHz device) after each SPI txn. ESP TFT now
80 MHz SPI + 40-line DMA ring (lcd.c from ../eps32/tft_4inch), -O2, 1 kHz
tick: 19 -> 35 FPS.
`esp32/` = Phase 3 UI (ESP32-S3 + 4" ST7796S 480x320), ported from
../tang_nano_9k_projects/FpgaRX-SDR/esp32: `./build.sh`, `./flash.sh [monitor]`. Status in PLAN.md.
ESP pins revised 2026-11-08 from Screenshots/photo_2026-11-08_22-27-08.jpg
(perfboard: ESP32-S3-DevKitC **N16R8** left of the Tang, both antenna/HDMI
up): link on ESP right row J3, J5-n opposite J3-(23-n) -> straight jumpers
(gen_schematic.ESP_LINK: SCK 53/G1, MISO 71/G2, WS 72/G42, MOSI 79/G39,
BCK 41/G47, DATA 42/G21, CS_N 80/G20, GND J5-2/J3-21; J5-1 +5V faces ESP
GND - never bridge). TFT on left row J1: CS/RST/DC/MOSI/SCK/LED =
4/5/6/7/15/16; encoder A/B 17/18 (LPD3806-600BM optical, 5V, NPN OC -> 4.7k pull-ups to 3V3, PCNT x4 = 2400/rev, input_enc_steps(): 50 dial steps/rev with gentle x2/x4 acceleration (smoothed speed), ENC_DIR -1, 20 menu steps/rev), SELECT button 8 (was encoder push), buttons 12/13/14. GPIO20 = USB D+ -> debug
console moved from USB-Serial-JTAG to UART0 (COM port). 86/G38 avoided
(86 = BL616 IO0). sdkconfig = N16R8 (octal PSRAM 8MB AP_3v3, 16MB flash).
Flashed 2026-11-08 via COM (CH343 1a86:55d3, /dev/ttyACM0): boots, SIM 16 FPS,
console OK; user confirmed the picture (MADCTL E8, invert 0; now 0x28, see top). hardware/jesssdr draws ESP|Tang side by side with real wires.
```
cd adc_test && ./build && ./flash
iverilog -g2012 -o sim/tb_adc_capture sim/tb_adc_capture.v adc_capture.v && vvp sim/tb_adc_capture
```
