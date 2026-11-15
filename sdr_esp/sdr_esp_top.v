// 40m direct-sampling SDR, v5 (PLAN.md phase 3, steps 3-6): Tang Nano 20K
// + AD9226 (64.8 MSPS) + onboard MAX98357A, UI on the ESP32-S3 (esp32/).
// The Tang keeps the whole radio path; the TFT/FFT/waterfall of ../sdr are
// gone (the ESP draws the spectrum from the I2S I/Q).
//
//   ADC -> ddc (NCO + CIC/1229 -> I/Q 52.7 kHz) -> [USB: Q negated]
//       -> ssb_demod (LSB filter -2700..-300 Hz, 13.18 kHz)   \
//          ssb_demod's decimated I/Q -> am_demod (envelope)    -> by MODE
//       -> audio_agc -> audio_nr (LMS NR / ANF) [-> audio_agc #2 when NR/ANF on]
//       -> audio_i2s (48.2 kHz) -> MAX98357A
//   ddc wide output (105.54 kHz I/Q) -> iq_i2s_tx -> ESP (I2S, FPGA = master)
//   ESP -> spi_regs (SPI slave): FREQ (BCD, converted here), MODE, NR, CTRL
//   ADC peak (|code-2048| max over 0.26 s, OTR = 255) and the S-meter level
//   -> spi_regs PEAK / LEVEL; buttons S1/S2 -> spi_regs BTN (event counters)
//
// ESP link pins (hardware/gen_schematic.py ESP_LINK): SCK 53, MISO 71,
// MOSI 79, CS_N 80, I2S BCK 41, WS 72, DATA 42.
//
// UART 1 Mbaud (pins 69/70), tools/sdr.py subset (debug without the ESP):
//   '?'            -> 'I' VERSION level {5'b0, am, 1, usb}
//   'f' f0 f1 f2 f3 (Hz, little endian) -> 'F'   (the ESP re-sends its own
//                     dial only if the spi_regs FREQ read-back differs - it
//                     does not, so a UART tune sticks until the ESP tunes)
//   'm' b          (0 LSB, 1 USB, 2 AM) -> 'M'
//   'q'            -> 'Q' n_lo n_hi, n x (I 24-bit LE, Q 24-bit LE): DDC output
//   'v'            -> same as 'q' ('Q' reply), but the wide (105.5 kHz) output
//   'a'            -> 'A' n_lo n_hi, n x 16-bit LE: audio (as played: after NR + make-up AGC, 13.18 kHz)
module sdr_esp_top #(
    parameter integer CLK_KHZ    = 64800,
    parameter integer BAUD       = 1000000,
    parameter [3:0]   PHASE      = 4'd13,     // ADC clock phase (tools/adc65.py sweep)
    parameter [24:0]  FREQ_INIT  = 25'd7100000,
    parameter integer PEAK_WIN   = 24         // peak window 2^24 samples = 0.26 s
) (
    input  wire        clk,                   // 27 MHz
    input  wire        btn_s1,                // onboard buttons -> spi_regs BTN (the ESP
    input  wire        btn_s2,                //   tunes with them while no encoder is wired)
    output wire        adc_clk,
    input  wire [11:0] adc_d,
    input  wire        adc_otr,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        i2s_bclk,              // onboard amplifier
    output wire        i2s_lrck,
    output wire        i2s_din,
    output wire        pa_en,
    input  wire        esp_sck,               // ESP link
    input  wire        esp_cs_n,
    input  wire        esp_mosi,
    output wire        esp_miso,
    output wire        esp_bck,
    output wire        esp_ws,
    output wire        esp_data
);
    localparam [7:0] VERSION = 8'd6;   // 6: VOL, AGC, programmable filter (2026-11-15)
    assign pa_en = 1'b1;                       // MAX98357A SD_MODE high = on (left)

    // ================= clock =================
    wire sclk, pll_lock;
    rPLL u_pll (
        .CLKOUT(sclk), .LOCK(pll_lock), .CLKOUTP(adc_clk), .CLKOUTD(), .CLKOUTD3(),
        .RESET(1'b0), .RESET_P(1'b0), .CLKIN(clk), .CLKFB(1'b0),
        .FBDSEL(6'd0), .IDSEL(6'd0), .ODSEL(6'd0),
        .PSDA(PHASE), .DUTYDA(PHASE + 4'd8), .FDLY(4'b1111)
    );
    defparam u_pll.FCLKIN        = "27";
    defparam u_pll.DYN_IDIV_SEL  = "false";
    defparam u_pll.IDIV_SEL      = 4;
    defparam u_pll.DYN_FBDIV_SEL = "false";
    defparam u_pll.FBDIV_SEL     = 11;
    defparam u_pll.DYN_ODIV_SEL  = "false";
    defparam u_pll.ODIV_SEL      = 16;
    defparam u_pll.PSDA_SEL      = "0000";
    defparam u_pll.DYN_DA_EN     = "true";
    defparam u_pll.DUTYDA_SEL    = "1000";
    defparam u_pll.CLKOUT_FT_DIR = 1'b1;
    defparam u_pll.CLKOUTP_FT_DIR = 1'b1;
    defparam u_pll.CLKOUT_DLY_STEP = 0;
    defparam u_pll.CLKOUTP_DLY_STEP = 0;
    defparam u_pll.CLKFB_SEL     = "internal";
    defparam u_pll.CLKOUT_BYPASS = "false";
    defparam u_pll.CLKOUTP_BYPASS = "false";
    defparam u_pll.CLKOUTD_BYPASS = "false";
    defparam u_pll.DYN_SDIV_SEL  = 2;
    defparam u_pll.CLKOUTD_SRC   = "CLKOUT";
    defparam u_pll.CLKOUTD3_SRC  = "CLKOUT";
    defparam u_pll.DEVICE        = "GW2A-18C";

    // ================= ADC =================
    reg [11:0] d_q = 12'd2048;
    reg        otr_q = 1'b0;
    always @(posedge sclk) d_q   <= adc_d;
    always @(posedge sclk) otr_q <= adc_otr;

    // peak: max |code - 2048| >> 3 per window (2048 -> 255); OTR -> 255
    wire [10:0] pk_pos = d_q[10:0];                          // code >= 2048: code - 2048
    wire [10:0] pk_neg = ~d_q[10:0];                         // code <  2048: 2047 - code
    wire [10:0] pk_mag = d_q[11] ? pk_pos : pk_neg;          // 0..2047 (|x| - 0.5 for x < 0)
    reg  [7:0]  pk_m8 = 8'd0;
    always @(posedge sclk) pk_m8 <= otr_q ? 8'd255 : pk_mag[10:3];
    reg  [PEAK_WIN-1:0] pk_cnt = {PEAK_WIN{1'b0}};
    reg  [7:0]  pk_cur = 8'd0, adc_peak = 8'd0;
    wire        pk_last = &pk_cnt;
    wire [7:0]  pk_nx = (pk_m8 > pk_cur) ? pk_m8 : pk_cur;
    always @(posedge sclk) pk_cnt <= pk_cnt + 1'b1;
    always @(posedge sclk) pk_cur <= pk_last ? 8'd0 : pk_nx;
    always @(posedge sclk) if (pk_last) adc_peak <= pk_nx;

    // ================= ESP SPI registers =================
    wire [7:0]  esp_ctrl;
    wire [31:0] esp_freq_bcd;
    wire        esp_freq_set;
    wire [1:0]  esp_mode;
    wire        esp_mode_set;
    wire [3:0]  esp_nr;
    wire [7:0]  esp_vol;
    wire [1:0]  esp_agc;
    wire        cf_we;
    wire [7:0]  cf_waddr;
    wire [35:0] cf_wdata;
    wire [7:0]  level;
    wire [7:0]  btn;
    buttons u_btn (.clk(sclk), .s1(btn_s1), .s2(btn_s2), .btn(btn));
    reg  [31:0] phase_inc = 32'd0;
    spi_regs #(.VERSION(VERSION)) u_regs (
        .clk(sclk), .spi_sck(esp_sck), .spi_cs_n(esp_cs_n), .spi_mosi(esp_mosi), .spi_miso(esp_miso),
        .ctrl(esp_ctrl), .freq_bcd(esp_freq_bcd), .freq_set(esp_freq_set),
        .mode(esp_mode), .mode_set(esp_mode_set), .nr(esp_nr),
        .vol(esp_vol), .agc(esp_agc), .cf_we(cf_we), .cf_waddr(cf_waddr), .cf_wdata(cf_wdata),
        .lo_fw(phase_inc), .lo_shift_k(4'sd0),
        .iqc_c1(18'd0), .iqc_c2(18'd0), .iqc_stat(2'd0),
        .adc_peak(adc_peak), .level(level), .btn(btn)
    );

    // FREQ (8 BCD digits) -> binary Hz: acc = acc * 10 + digit, MSD first
    reg [31:0] b2b_sr = 32'd0;
    reg [3:0]  b2b_n = 4'd0;                   // digits left, 0 = idle
    reg [26:0] b2b_acc = 27'd0;
    reg        b2b_done = 1'b0;
    always @(posedge sclk)
        if (esp_freq_set)       b2b_sr <= esp_freq_bcd;
        else if (b2b_n != 4'd0) b2b_sr <= {b2b_sr[27:0], 4'd0};
    always @(posedge sclk)
        if (esp_freq_set)       b2b_n <= 4'd8;
        else if (b2b_n != 4'd0) b2b_n <= b2b_n - 4'd1;
    always @(posedge sclk)
        if (esp_freq_set)       b2b_acc <= 27'd0;
        else if (b2b_n != 4'd0) b2b_acc <= {b2b_acc[23:0], 3'd0} + {b2b_acc[25:0], 1'b0} + b2b_sr[31:28];
    always @(posedge sclk) b2b_done <= (b2b_n == 4'd1);

    // ================= tuning / mode =================
    reg        u_set_f = 1'b0, u_set_m = 1'b0;   // UART-requested changes
    reg [31:0] u_arg = 32'd0;
    localparam [24:0] F_MIN = 25'd100000, F_MAX = 25'd32000000;
    function [24:0] clampf(input [31:0] f);
        clampf = (f < F_MIN) ? F_MIN : (f > F_MAX) ? F_MAX : f[24:0];
    endfunction
    reg [24:0] freq = FREQ_INIT;
    always @(posedge sclk)
        if (b2b_done)     freq <= clampf({5'd0, b2b_acc});
        else if (u_set_f) freq <= clampf(u_arg);

    reg [1:0] mode = 2'd0;                     // 0 LSB, 1 USB, 2/3 AM
    always @(posedge sclk)
        if (esp_mode_set) mode <= esp_mode;
        else if (u_set_m) mode <= u_arg[1:0];
    wire usb    = (mode == 2'd1);
    wire mode_am = mode[1];

    // phase_inc = freq * 2^32 / 64.8e6 = (freq * 69500145) >> 20, pipelined
    reg [51:0] pi_prod = 52'd0;
    always @(posedge sclk) pi_prod   <= freq * 27'd69500145;
    always @(posedge sclk) phase_inc <= pi_prod[51:20];

    // ================= DSP chain =================
    wire               ddc_v, wd_v;
    wire signed [17:0] ddc_i, ddc_q, wd_i, wd_q;
    ddc u_ddc (.clk(sclk), .adc_code(d_q), .phase_inc(phase_inc),
               .out_valid(ddc_v), .out_i(ddc_i), .out_q(ddc_q),
               .w_valid(wd_v), .w_i(wd_i), .w_q(wd_q));

    // USB: mirror the spectrum so the LSB filter selects the upper sideband
    wire signed [17:0] dm_q = usb ? ((ddc_q == -18'sd131072) ? 18'sd131071 : -ddc_q) : ddc_q;

    wire               lsb_v, dec_v;
    wire signed [17:0] lsb_aud, dec_i, dec_q;
    ssb_demod u_ssb (.clk(sclk), .in_valid(ddc_v), .in_i(ddc_i), .in_q(dm_q),
                     .out_valid(lsb_v), .out_audio(lsb_aud),
                     .dec_valid(dec_v), .dec_i(dec_i), .dec_q(dec_q),
                     .cf_we(cf_we), .cf_waddr(cf_waddr), .cf_wdata(cf_wdata));
    wire               am_v;
    wire signed [17:0] am_aud;
    am_demod u_am (.clk(sclk), .in_valid(dec_v), .in_i(dec_i), .in_q(dec_q),
                   .out_valid(am_v), .out_audio(am_aud));
    wire               dem_v   = mode_am ? am_v : lsb_v;
    wire signed [17:0] dem_aud = mode_am ? am_aud : lsb_aud;

    wire               agc_v;
    wire signed [15:0] agc_out;
    audio_agc u_agc (.clk(sclk), .speed(esp_agc), .in_valid(dem_v), .in_audio(dem_aud), .out_valid(agc_v), .out_audio(agc_out));
    wire               nr_v;
    wire signed [15:0] nr_out;
    audio_nr u_nr (.clk(sclk), .mode(esp_nr[1:0]), .level(esp_nr[3:2]),
                   .in_valid(agc_v), .in_audio(agc_out), .out_valid(nr_v), .out_audio(nr_out));
    // Make-up gain after NR/ANF: NR takes 6-15dB of loudness, more the weaker
    // the station (audio_nr.v header), ANF a removed carrier's share. A 2nd
    // AGC with the same target restores it; G_MAX 8x (18dB) so long pauses
    // keep most of what NR took off the noise. Bypassed when NR is off.
    wire               nr_on = (esp_nr[1:0] != 2'd0);
    wire               agc2_v;
    wire signed [15:0] agc2_out;
    audio_agc #(.G_MAX(24'd32768)) u_agc2 (.clk(sclk), .speed(esp_agc), .in_valid(nr_v), .in_audio({{2{nr_out[15]}}, nr_out}),
                                           .out_valid(agc2_v), .out_audio(agc2_out));
    wire               aud_v   = nr_on ? agc2_v   : nr_v;
    wire signed [15:0] aud_out = nr_on ? agc2_out : nr_out;
    // volume (VOL, Q1.7) right before the speaker; the UART 'a' dump stays pre-volume
    reg signed [24:0] vol_p = 25'sd0;
    reg               vol_v = 1'b0, vol_v2 = 1'b0;
    reg signed [15:0] spk = 16'sd0;
    always @(posedge sclk) if (aud_v) vol_p <= aud_out * $signed({1'b0, esp_vol});
    always @(posedge sclk) vol_v <= aud_v;
    wire signed [17:0] vol_s = vol_p >>> 7;
    always @(posedge sclk) if (vol_v) spk <= (vol_s > 18'sd32767) ? 16'sd32767 : (vol_s < -18'sd32768) ? -16'sd32768 : vol_s[15:0];
    always @(posedge sclk) vol_v2 <= vol_v;
    audio_i2s u_i2s (.clk(sclk), .in_valid(vol_v2), .in_audio(spk),
                     .bclk(i2s_bclk), .lrck(i2s_lrck), .din(i2s_din));

    level_meter u_lvl (.clk(sclk), .in_valid(dec_v), .in_i(dec_i), .in_q(dec_q), .level(level));

    // ================= I/Q link to the ESP (I2S, 105.54 kHz) =================
    wire lk_bck, lk_ws;
    i2s_link_clk #(.DIV(614)) u_lkclk (.clk(sclk), .bck(lk_bck), .ws(lk_ws));
    wire lk_data;
    iq_i2s_tx u_iqtx (.clk(sclk), .bck(lk_bck), .lrck(lk_ws), .test_mode(esp_ctrl[0]),
                      .in_valid(wd_v), .in_i(wd_i), .in_q(wd_q), .data(lk_data));
    // BCK/WS go to the pins straight from their registers; iq_i2s_tx changes
    // DATA 1 clk after the BCK fall, one more register makes it 2 clk
    // (31 ns): ~2.5 clk setup before the rise, ~6.5 clk hold after it - the
    // ESP slave samples a little after the rise (input synchronizer).
    reg data_o = 1'b0;
    always @(posedge sclk) data_o <= lk_data;
    assign esp_bck  = lk_bck;
    assign esp_ws   = lk_ws;
    assign esp_data = data_o;

    // ================= UART =================
    localparam integer DIV = (CLK_KHZ * 1000 + BAUD / 2) / BAUD;
    wire [7:0] rx_data;
    wire       rx_valid;
    uart_rx #(.DIV(DIV)) u_rx (.clk(sclk), .rx(uart_rx), .data(rx_data), .valid(rx_valid));
    reg        tx_start = 1'b0;
    reg  [7:0] tx_data = 8'd0;
    wire       tx_busy;
    uart_tx #(.DIV(DIV)) u_tx (.clk(sclk), .start(tx_start), .data(tx_data), .tx(uart_tx), .busy(tx_busy));

    // capture buffer: 2048 x 36 ({I, Q} or {audio, 0})
    localparam integer NCAP = 2048;
    reg  [11:0] cwa = 12'd0, cra = 12'd0;
    reg         cwe = 1'b0;
    reg  [35:0] cwd = 36'd0;
    reg  [35:0] cmem [0:NCAP-1];
    reg  [35:0] crd = 36'd0;
    always @(posedge sclk) begin
        if (cwe)
            cmem[cwa] <= cwd;
        crd <= cmem[cra];
    end

    localparam C_IDLE = 3'd0, C_ARG = 3'd1, C_REPLY = 3'd2, C_CAP = 3'd3, C_HDR = 3'd4,
               C_RD = 3'd5, C_BYTE = 3'd6;
    localparam [11:0] CLAST = NCAP - 1;
    reg [2:0]  cst = C_IDLE;
    reg [7:0]  cmd = 8'd0;
    reg [2:0]  nargs = 3'd0;
    reg [7:0]  rb0 = 0, rb1 = 0, rb2 = 0, rb3 = 0;
    reg [2:0]  rlen = 3'd0, ri = 3'd0;
    reg        kind_q = 1'b0;                  // 1 = I/Q dump, 0 = audio dump
    reg        kind_w = 1'b0;                  // I/Q dump of the wide output
    reg [2:0]  bi = 3'd0;                      // byte index within a sample
    reg [1:0]  rdw = 2'd0;
    reg        sent = 1'b0;
    wire can_issue = !tx_busy && !sent;
    wire byte_done = sent && !tx_busy && !tx_start;
    wire [7:0] reply_byte = (ri == 3'd0) ? rb0 : (ri == 3'd1) ? rb1 : (ri == 3'd2) ? rb2 : rb3;
    wire [23:0] ci24 = {{6{crd[35]}}, crd[35:18]};
    wire [23:0] cq24 = {{6{crd[17]}}, crd[17:0]};
    reg  [7:0]  data_byte;
    always @(*)
        case (bi)
            3'd0: data_byte = kind_q ? ci24[7:0]   : crd[27:20];
            3'd1: data_byte = kind_q ? ci24[15:8]  : crd[35:28];
            3'd2: data_byte = ci24[23:16];
            3'd3: data_byte = cq24[7:0];
            3'd4: data_byte = cq24[15:8];
            default: data_byte = cq24[23:16];
        endcase
    wire [2:0] last_bi = kind_q ? 3'd5 : 3'd1;
    wire       cap_v = kind_q ? (kind_w ? wd_v : ddc_v) : aud_v;
    wire [35:0] cap_d = kind_q ? (kind_w ? {wd_i, wd_q} : {ddc_i, ddc_q}) : {aud_out, 20'd0};

    always @(posedge sclk) begin
        tx_start <= 1'b0;
        cwe      <= 1'b0;
        u_set_f  <= 1'b0;
        u_set_m  <= 1'b0;
        case (cst)
            C_IDLE:
                if (rx_valid) begin
                    cmd <= rx_data;
                    if (rx_data == "?") begin
                        rb0 <= "I"; rb1 <= VERSION; rb2 <= level; rb3 <= {5'd0, mode_am, 1'b1, usb};
                        rlen <= 3'd4; ri <= 3'd0; cst <= C_REPLY;
                    end else if (rx_data == "f") begin
                        nargs <= 3'd4; cst <= C_ARG;
                    end else if (rx_data == "m") begin
                        nargs <= 3'd1; cst <= C_ARG;
                    end else if (rx_data == "q" || rx_data == "v" || rx_data == "a") begin
                        kind_q <= (rx_data != "a"); kind_w <= (rx_data == "v");
                        cwa <= 12'd0; cst <= C_CAP;
                    end
                end
            C_ARG:
                if (rx_valid) begin
                    u_arg <= {rx_data, u_arg[31:8]};
                    nargs <= nargs - 3'd1;
                    if (nargs == 3'd1) begin
                        if (cmd == "f") begin u_set_f <= 1'b1; rb0 <= "F"; end
                        else begin u_set_m <= 1'b1; rb0 <= "M"; end
                        rlen <= 3'd1; ri <= 3'd0; cst <= C_REPLY;
                    end
                end
            C_REPLY:
                if (can_issue) begin
                    tx_data <= reply_byte; tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0;
                    if (ri == rlen - 3'd1) cst <= C_IDLE;
                    else ri <= ri + 3'd1;
                end
            C_CAP: begin                       // one write per sample, NCAP samples
                if (cwe) begin
                    cwa <= cwa + 12'd1;
                    if (cwa == CLAST) begin
                        rb0 <= kind_q ? "Q" : "A"; rb1 <= NCAP[7:0]; rb2 <= NCAP[15:8];
                        ri <= 3'd0; cst <= C_HDR;
                    end
                end
                if (cap_v && !(cwe && cwa == CLAST)) begin
                    cwe <= 1'b1;
                    cwd <= cap_d;
                end
            end
            C_HDR:
                if (can_issue) begin
                    tx_data <= reply_byte; tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0;
                    if (ri == 3'd2) begin cra <= 12'd0; rdw <= 2'd2; bi <= 3'd0; cst <= C_RD; end
                    else ri <= ri + 3'd1;
                end
            C_RD:
                if (rdw != 2'd0) rdw <= rdw - 2'd1;
                else cst <= C_BYTE;
            C_BYTE:
                if (can_issue) begin
                    tx_data <= data_byte; tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0;
                    if (bi != last_bi) bi <= bi + 3'd1;
                    else if (cra == CLAST) cst <= C_IDLE;
                    else begin bi <= 3'd0; cra <= cra + 12'd1; rdw <= 2'd2; cst <= C_RD; end
                end
            default: cst <= C_IDLE;
        endcase
    end
endmodule
