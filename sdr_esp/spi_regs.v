// SPI slave register file: the ESP32-S3 (SPI master, mode 0, MSB first)
// controls the FPGA through it. Copied from ../tang_nano_9k_projects/FpgaRX-SDR/stage1/spi_regs.v;
// changes for JessSDR sdr_esp/: VERSION is a parameter, register
// 0x15 LEVEL, 0x16 BTN, 0x17 VOL, 0x18 AGC, 0x19..0x1E filter taps added, and clk is the 64.8 MHz DSP clock (sdr_esp_top.v ties
// IQC_* to 0 and LO_SHIFT_K to 0 - the DDC is centred on the dial; LO_FW =
// the 64.8 MHz NCO phase increment).
//
// Transaction (CS low for the whole transaction):
//   byte 0  = {rw, addr[6:0]}   rw = 1 read, 0 write
//   byte 1+ = data; the address auto-increments after every data byte, so a
//             multi-byte register can be read or written in one go.
// On a read, MISO carries reg[addr] during byte 1, reg[addr+1] during byte
// 2, and so on. MISO is 0 during byte 0.
//
// SCK/CS/MOSI come from another chip and are sampled by the 27MHz clock
// through 2-flop synchronizers. SCK must stay well below clk/8; the ESP
// uses 1MHz. With mode 0 the master samples MISO on the SCK rising edge,
// and MISO changes here ~3 clk (110ns) after the falling edge, which leaves
// almost a full half period of setup time at 1MHz.
//
// Registers (multi-byte values MSB first, i.e. at the lowest address):
//   0x00 ID        RO  0x5A
//   0x01 VERSION   RO  0x06
//   0x02 SCRATCH   RW  free byte for the link test
//   0x03 CTRL      RW  bit0 = I2S test pattern (iq_i2s_tx.v)
//                      bit1 = I/Q imbalance correction on (iq_corr.v)
//   0x04..0x07 FREQ RW dial frequency, 8 packed BCD digits (tune_ctrl.v
//                  format, 0x07100000 = 7.100.000 Hz). Bytes 0x04..0x06 are
//                  staged; writing 0x07 latches all four at once and pulses
//                  freq_set, so the LO never sees a half-written value.
//   0x08..0x0B LO_FW RO the LO phase increment freq_to_phase_inc.v computed
//                  (lets the ESP check that the LO really was retuned)
//   0x0C LO_SHIFT_K RO low-IF offset, signed: the LO runs K*fs/16 above the
//                  dial (K < 0: below). Depends on MODE: LSB 1, USB -1, AM 4.
//   0x0D MODE      RW  demodulator: 0 = LSB, 1 = USB, 2 = AM (3 acts as AM).
//                  A write pulses mode_set (the LO offset changes with it).
//   0x0E..0x0F IQC_C1 RO iq_corr c1 >> 2, signed Q0.15 (= -(A_Q/A_I) sin(phi))
//   0x10..0x11 IQC_C2 RO iq_corr c2 >> 2, signed Q1.14 (= (A_Q/A_I) cos(phi))
//                  -> phase error phi = atan2(-c1, c2), gain A_Q/A_I = |c1, c2|
//   0x12 IQC_STAT  RO  bit0 = coefficients valid (a block with signal seen)
//                      bit1 = signal present (the last block was used)
//   0x13 NR        RW  audio_nr.v: bits 1:0 mode (0 off, 1 NR, 2 ANF),
//                      bits 3:2 level (mu = 2^-(10+level))
//   0x14 PEAK      RO  adc_peak.v: max |I|,|Q| of the raw ADC samples over the
//                      last 0.31s, |x| >> 9 (255 = full scale, >= 240 = clipped)
//   0x15 LEVEL     RO  level_meter.v L8 (S-meter)
//   0x16 BTN       RO  Tang S1/S2 press counters (buttons.v)
//   0x17 VOL       RW  audio gain, unsigned Q1.7 (0x80 = 1.0, 0 = mute)
//   0x18 AGC       RW  bits 1:0 audio_agc speed: 0 fast, 1 medium, 2 slow
//   0x19 CF_ADDR   RW  ssb_demod filter tap index for the next CF_DATA write
//   0x1A..0x1E CF_DATA WO one complex tap, 40 bits MSB first, low 36 used =
//                  {re[17:0], im[17:0]} Q1.17. 0x1A..0x1D are staged; writing
//                  0x1E writes the tap at CF_ADDR and increments CF_ADDR, so
//                  the ESP sets CF_ADDR = 0 once and sends 255 5-byte bursts.
//   0x1F CLIP      RO  adc_clip.v: clipped ADC samples (code 0/4095 or OTR) per
//                  PEAK window, log2-coded: 0 none, n = 2^(n-1)..2^n-1 (VERSION 8)
//   others        RO  0x00
module spi_regs #(
    parameter [7:0] VERSION = 8'h06
) (
    input  wire       clk,
    input  wire       spi_sck,
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output reg        spi_miso = 1'b0,

    output reg  [7:0] ctrl = 8'h00,
    output reg [31:0] freq_bcd = 32'h07100000,
    output reg        freq_set = 1'b0,     // single-cycle pulse, freq_bcd already new
    output reg  [1:0] mode = 2'd0,
    output reg        mode_set = 1'b0,     // single-cycle pulse, mode already new
    output reg  [3:0] nr = 4'd0,
    output reg  [7:0] vol = 8'h80,
    output reg  [1:0] agc = 2'd1,
    output reg        cf_we = 1'b0,        // single-cycle pulse
    output reg  [7:0] cf_waddr = 8'd0,
    output reg [35:0] cf_wdata = 36'd0,
    input  wire [31:0] lo_fw,
    input  wire signed [3:0] lo_shift_k,
    input  wire [17:0] iqc_c1,
    input  wire [17:0] iqc_c2,
    input  wire [1:0]  iqc_stat,
    input  wire [7:0]  adc_peak,
    input  wire [7:0]  adc_clip,
    input  wire [7:0]  level,
    input  wire [7:0]  btn
);

    localparam [7:0] ID      = 8'h5A;

    // ---- synchronizers ----
    reg [2:0] sck_s  = 3'b000;
    reg [1:0] cs_s   = 2'b11;
    reg [1:0] mosi_s = 2'b00;
    always @(posedge clk) sck_s  <= {sck_s[1:0], spi_sck};
    always @(posedge clk) cs_s   <= {cs_s[0], spi_cs_n};
    always @(posedge clk) mosi_s <= {mosi_s[0], spi_mosi};

    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire sck_fall = (sck_s[2:1] == 2'b10);
    wire cs_n     = cs_s[1];
    wire mosi     = mosi_s[1];

    reg [7:0] scratch = 8'h00;

    function [7:0] reg_read(input [6:0] a);
        case (a)
            7'h00:   reg_read = ID;
            7'h01:   reg_read = VERSION;
            7'h02:   reg_read = scratch;
            7'h03:   reg_read = ctrl;
            7'h04:   reg_read = freq_bcd[31:24];
            7'h05:   reg_read = freq_bcd[23:16];
            7'h06:   reg_read = freq_bcd[15:8];
            7'h07:   reg_read = freq_bcd[7:0];
            7'h08:   reg_read = lo_fw[31:24];
            7'h09:   reg_read = lo_fw[23:16];
            7'h0A:   reg_read = lo_fw[15:8];
            7'h0B:   reg_read = lo_fw[7:0];
            7'h0C:   reg_read = {{4{lo_shift_k[3]}}, lo_shift_k};
            7'h0D:   reg_read = {6'd0, mode};
            7'h0E:   reg_read = iqc_c1[17:10];
            7'h0F:   reg_read = iqc_c1[9:2];
            7'h10:   reg_read = iqc_c2[17:10];
            7'h11:   reg_read = iqc_c2[9:2];
            7'h12:   reg_read = {6'd0, iqc_stat};
            7'h13:   reg_read = {4'd0, nr};
            7'h14:   reg_read = adc_peak;
            7'h15:   reg_read = level;
            7'h16:   reg_read = btn;
            7'h17:   reg_read = vol;
            7'h18:   reg_read = {6'd0, agc};
            7'h19:   reg_read = cf_addr;
            7'h1F:   reg_read = adc_clip;
            default: reg_read = 8'h00;
        endcase
    endfunction

    // ---- receive side ----
    reg [2:0] bit_cnt = 3'd0;    // bits of the current byte received
    reg [6:0] rx      = 7'd0;    // first 7 bits of the current byte
    reg       have_cmd = 1'b0;   // byte 0 done
    reg       rw       = 1'b0;
    reg [6:0] addr     = 7'd0;

    wire [7:0] byte_in   = {rx, mosi};               // valid on the 8th rise
    wire       byte_done = sck_rise && bit_cnt == 3'd7;
    wire       wr_strobe = byte_done && have_cmd && !rw;

    always @(posedge clk)
        if (cs_n)          bit_cnt <= 3'd0;
        else if (sck_rise) bit_cnt <= bit_cnt + 3'd1;

    always @(posedge clk)
        if (sck_rise) rx <= {rx[5:0], mosi};

    always @(posedge clk)
        if (cs_n)           have_cmd <= 1'b0;
        else if (byte_done) have_cmd <= 1'b1;

    always @(posedge clk)
        if (byte_done && !have_cmd) rw <= byte_in[7];

    always @(posedge clk)
        if (byte_done) addr <= have_cmd ? addr + 7'd1 : byte_in[6:0];

    always @(posedge clk)
        if (wr_strobe && addr == 7'h02) scratch <= byte_in;

    always @(posedge clk)
        if (wr_strobe && addr == 7'h03) ctrl <= byte_in;

    reg [23:0] freq_stage = 24'd0;
    always @(posedge clk)
        if (wr_strobe && addr == 7'h04) freq_stage[23:16] <= byte_in;
        else if (wr_strobe && addr == 7'h05) freq_stage[15:8] <= byte_in;
        else if (wr_strobe && addr == 7'h06) freq_stage[7:0] <= byte_in;

    always @(posedge clk)
        if (wr_strobe && addr == 7'h07) freq_bcd <= {freq_stage, byte_in};

    always @(posedge clk) begin
        freq_set <= 1'b0;
        if (wr_strobe && addr == 7'h07) freq_set <= 1'b1;
    end

    always @(posedge clk)
        if (wr_strobe && addr == 7'h0D) mode <= byte_in[1:0];

    always @(posedge clk)
        if (wr_strobe && addr == 7'h13) nr <= byte_in[3:0];

    always @(posedge clk)
        if (wr_strobe && addr == 7'h17) vol <= byte_in;

    always @(posedge clk)
        if (wr_strobe && addr == 7'h18) agc <= byte_in[1:0];

    // filter taps: CF_ADDR, then 5-byte CF_DATA bursts
    reg [7:0]  cf_addr  = 8'd0;
    reg [31:0] cf_stage = 32'd0;
    wire       cf_commit = wr_strobe && addr == 7'h1E;
    always @(posedge clk)
        if (wr_strobe && addr == 7'h19) cf_addr <= byte_in;
        else if (cf_commit)             cf_addr <= cf_addr + 8'd1;

    always @(posedge clk)
        if (wr_strobe && addr == 7'h1A)      cf_stage[31:24] <= byte_in;
        else if (wr_strobe && addr == 7'h1B) cf_stage[23:16] <= byte_in;
        else if (wr_strobe && addr == 7'h1C) cf_stage[15:8]  <= byte_in;
        else if (wr_strobe && addr == 7'h1D) cf_stage[7:0]   <= byte_in;

    always @(posedge clk) cf_we <= cf_commit;
    always @(posedge clk) if (cf_commit) cf_waddr <= cf_addr;
    always @(posedge clk) if (cf_commit) cf_wdata <= {cf_stage[27:0], byte_in};

    always @(posedge clk) begin
        mode_set <= 1'b0;
        if (wr_strobe && addr == 7'h0D) mode_set <= 1'b1;
    end

    // ---- transmit side ----
    // Loaded on the last rise of a byte with the register the NEXT byte
    // reads; shifted out on the falling edges.
    reg [7:0] tx = 8'h00;
    wire [6:0] next_addr = have_cmd ? addr + 7'd1 : byte_in[6:0];
    wire       next_rd   = have_cmd ? rw : byte_in[7];

    always @(posedge clk)
        if (cs_n)           tx <= 8'h00;
        else if (byte_done) tx <= next_rd ? reg_read(next_addr) : 8'h00;
        else if (sck_fall)  tx <= {tx[6:0], 1'b0};

    always @(posedge clk)
        if (cs_n)          spi_miso <= 1'b0;
        else if (sck_fall) spi_miso <= tx[7];

endmodule
