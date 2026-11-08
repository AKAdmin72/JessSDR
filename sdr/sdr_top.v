// 40m direct-sampling SDR, v2: Tang Nano 20K + AD9226 (64.8 MSPS) + ILI9341
// + onboard MAX98357A.
//
//   ADC -> ddc (NCO + CIC/1229 -> I/Q 52.7 kHz) -> [USB: Q negated]
//       -> ssb_demod (from FpgaRX-SDR: LSB filter -2700..-300 Hz, 13.18 kHz)
//       -> audio_agc (FpgaRX-SDR) -> audio_i2s (48.2 kHz) -> MAX98357A
//   ssb_demod's decimated I/Q -> level_meter -> S-meter bar
//   ddc I/Q -> spectrum (fft512, Blackman-Harris, 102.98 Hz bins, mean of
//       8 frames) -> spectrum trace (+-33 kHz wide (default) or +-16.5 kHz, 1.5 px/dB default, UART zoom) + waterfall
//       (102 rows, ~12 lines/s, newest on top); floor_eq (auto noise floor,
//       from FpgaRX-SDR spectrum_eq) flattens the line before display
//
// Screen: y 0..23 frequency (3x), mode, step underline; y 30..47 S-meter;
//         y 50..125 spectrum (SSB passband shaded, dial = red line x 160);
//         y 126..137 ticks + kHz offsets; y 138..239 waterfall.
//
// Controls: S1 short = up one step, S2 short = down one step,
//           S1 long  = next step (10 Hz / 100 Hz / 1 kHz / 10 kHz / 100 kHz),
//           S2 long  = LSB <-> USB. Step digit is underlined.
// UART 1 Mbaud (pins 69/70), for tools/sdr.py:
//   '?'            -> 'I' VERSION level usb
//   'f' f0 f1 f2 f3 (Hz, little endian) -> 'F'
//   'm' b          (0 LSB, 1 USB) -> 'M'
//   'q'            -> 'Q' n_lo n_hi, n x (I 24-bit LE, Q 24-bit LE): DDC output
//   'a'            -> 'A' n_lo n_hi, n x 16-bit LE: AGC output (13.18 kHz)
//   's'            -> 'S' 320 0x01, 320 x 16-bit LE: next spectrum line (L8,
//                     0.376 dB units, x = 0..319 = bins -160..159)
//   'b' sb wb      -> 'B': spectrum base (L8 at the bottom) / waterfall base
//   'e' b          -> 'E': automatic noise floor (floor_eq) 1 = on (default), 0 = off
//   'g' db         -> 'G': front-end gain in dB (S-meter offset, 0..255)
//   'z' n          -> 'Z': spectrum scale 0 = 1 px/dB, 1 = 1.5 px/dB, 2 = 2 px/dB
//   'w' b          -> 'W': spectrum span 1 = +-33 kHz (ddc wide output, 16-frame mean),
//                     0 = +-16.5 kHz. '?' reply byte 3 = {wide, usb}.
module sdr_top #(
    parameter integer CLK_KHZ    = 64800,
    parameter integer BAUD       = 1000000,
    parameter [3:0]   PHASE      = 4'd13,     // ADC clock phase (tools/adc65.py sweep)
    parameter [24:0]  FREQ_INIT  = 25'd7100000,
    parameter [7:0]   GAIN_DB    = 8'd25,     // front-end gain before the ADC (S-meter offset);
                                              // 25 = AD8138 gain mod (2026-11-07), UART 'g' changes it
    parameter [1:0]   ZOOM_INIT  = 2'd1,      // spectrum px/dB: 0 = 1, 1 = 1.5, 2 = 2 (UART 'z')
    parameter         WIDE_INIT  = 1'b1,      // spectrum span: 1 = +-33 kHz (206 Hz/px), 0 = +-16.5 kHz
    parameter [7:0]   SBASE_INIT = 8'd78,     // spectrum: L8 at the baseline (noise ~10 px up)
    parameter [7:0]   WBASE_INIT = 8'd88      // waterfall: L8 of palette index 0
) (
    input  wire        clk,                   // 27 MHz
    input  wire        btn_s1,
    input  wire        btn_s2,
    output wire        adc_clk,
    input  wire [11:0] adc_d,
    input  wire        adc_otr,
    output wire        tft_sck,
    output wire        tft_mosi,
    output wire        tft_cs,
    output wire        tft_dc,
    output wire        tft_rst,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        i2s_bclk,
    output wire        i2s_lrck,
    output wire        i2s_din,
    output wire        pa_en
);
    localparam [7:0] VERSION = 8'd3;
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
    always @(posedge sclk) d_q <= adc_d;

    // ================= tuning state =================
    reg [24:0] freq = FREQ_INIT;
    reg [2:0]  step_idx = 3'd2;               // 0:10 1:100 2:1k 3:10k 4:100k
    reg        usb = 1'b0;
    function [24:0] step_hz(input [2:0] s);
        case (s)
            3'd0: step_hz = 25'd10;    3'd1: step_hz = 25'd100;
            3'd2: step_hz = 25'd1000;  3'd3: step_hz = 25'd10000;
            default: step_hz = 25'd100000;
        endcase
    endfunction

    // phase_inc = freq * 2^32 / 64.8e6 = (freq * 69500145) >> 20, pipelined
    reg [51:0] pi_prod = 52'd0;
    reg [31:0] phase_inc = 32'd0;
    always @(posedge sclk) pi_prod   <= freq * 27'd69500145;
    always @(posedge sclk) phase_inc <= pi_prod[51:20];

    // ================= buttons =================
    reg [1:0] s1_sync = 2'b00, s2_sync = 2'b00;
    always @(posedge sclk) s1_sync <= {s1_sync[0], btn_s1};
    always @(posedge sclk) s2_sync <= {s2_sync[0], btn_s2};
    reg [18:0] deb_div = 19'd0;               // 2^19 / 64.8 MHz = 8.1 ms
    always @(posedge sclk) deb_div <= deb_div + 19'd1;
    wire deb_tick = (deb_div == 19'd0);
    reg s1_st = 1'b0, s2_st = 1'b0, s1_prev = 1'b0, s2_prev = 1'b0;
    always @(posedge sclk) if (deb_tick) s1_st <= s1_sync[1];
    always @(posedge sclk) if (deb_tick) s2_st <= s2_sync[1];
    always @(posedge sclk) s1_prev <= s1_st;
    always @(posedge sclk) s2_prev <= s2_st;
    localparam [6:0] HOLD_LONG = 7'd62;       // ~0.5 s
    reg [6:0] s1_hold = 7'd0, s2_hold = 7'd0;
    always @(posedge sclk)
        if (deb_tick) s1_hold <= !s1_st ? 7'd0 : (s1_hold == 7'd127) ? s1_hold : s1_hold + 7'd1;
    always @(posedge sclk)
        if (deb_tick) s2_hold <= !s2_st ? 7'd0 : (s2_hold == 7'd127) ? s2_hold : s2_hold + 7'd1;
    wire s1_short = !s1_st && s1_prev && (s1_hold < HOLD_LONG);
    wire s2_short = !s2_st && s2_prev && (s2_hold < HOLD_LONG);
    wire s1_long  = deb_tick && s1_st && (s1_hold == HOLD_LONG - 7'd1);
    wire s2_long  = deb_tick && s2_st && (s2_hold == HOLD_LONG - 7'd1);

    // UART-requested changes (from the command FSM below)
    reg        u_set_f = 1'b0, u_set_m = 1'b0;
    reg [31:0] u_arg = 32'd0;

    localparam [24:0] F_MIN = 25'd100000, F_MAX = 25'd32000000;
    always @(posedge sclk)
        if (u_set_f)
            freq <= (u_arg[24:0] < F_MIN) ? F_MIN : (u_arg[31:0] > F_MAX) ? F_MAX : u_arg[24:0];
        else if (s1_short)
            freq <= (freq + step_hz(step_idx) > F_MAX) ? F_MAX : freq + step_hz(step_idx);
        else if (s2_short)
            freq <= (freq < F_MIN + step_hz(step_idx)) ? F_MIN : freq - step_hz(step_idx);
    always @(posedge sclk)
        if (s1_long) step_idx <= (step_idx == 3'd4) ? 3'd0 : step_idx + 3'd1;
    always @(posedge sclk)
        if (u_set_m)      usb <= u_arg[0];
        else if (s2_long) usb <= ~usb;

    // ================= DSP chain =================
    wire               ddc_v;
    wire signed [17:0] ddc_i, ddc_q;
    wire               wd_v;
    wire signed [17:0] wd_i, wd_q;
    ddc u_ddc (.clk(sclk), .adc_code(d_q), .phase_inc(phase_inc),
               .out_valid(ddc_v), .out_i(ddc_i), .out_q(ddc_q),
               .w_valid(wd_v), .w_i(wd_i), .w_q(wd_q));

    // USB: mirror the spectrum so the LSB filter selects the upper sideband
    wire signed [17:0] dm_q = usb ? ((ddc_q == -18'sd131072) ? 18'sd131071 : -ddc_q) : ddc_q;

    wire               aud_v, dec_v;
    wire signed [17:0] aud, dec_i, dec_q;
    ssb_demod u_ssb (.clk(sclk), .in_valid(ddc_v), .in_i(ddc_i), .in_q(dm_q),
                     .out_valid(aud_v), .out_audio(aud),
                     .dec_valid(dec_v), .dec_i(dec_i), .dec_q(dec_q));
    wire               agc_v;
    wire signed [15:0] agc_out;
    audio_agc u_agc (.clk(sclk), .in_valid(aud_v), .in_audio(aud), .out_valid(agc_v), .out_audio(agc_out));
    audio_i2s u_i2s (.clk(sclk), .in_valid(agc_v), .in_audio(agc_out),
                     .bclk(i2s_bclk), .lrck(i2s_lrck), .din(i2s_din));

    wire [7:0] level;
    level_meter u_lvl (.clk(sclk), .in_valid(dec_v), .in_i(dec_i), .in_q(dec_q), .level(level));

    // ================= spectrum / waterfall =================
    localparam integer WF_ROWS = 102;              // waterfall rows (y 138..239)
    localparam integer SP_H    = 76;               // spectrum height (y 50..125)
    localparam [7:0]   SP_Y0 = 8'd50, SP_Y1 = 8'd125, WF_Y0 = 8'd138;
    wire       ln_we, ln_done;
    wire [8:0] ln_x, ln_l8;
    wire [6:0] wf_row, wf_top;
    reg        wide = WIDE_INIT;
    // wide: 105.5 kHz I/Q, 16 frames per line (same ~12 lines/s as narrow with 8)
    spectrum #(.NAVG_LOG2(4), .WF_ROWS(WF_ROWS)) u_spec (
        .clk(sclk), .in_valid(wide ? wd_v : ddc_v), .in_i(wide ? wd_i : ddc_i),
        .in_q(wide ? wd_q : ddc_q), .navg_half(~wide),
        .line_we(ln_we), .line_x(ln_x), .line_l8(ln_l8),
        .wf_row(wf_row), .wf_top(wf_top), .line_done(ln_done));
    // automatic noise floor (per-bin flattening): e_* feeds the display,
    // the raw ln_* line goes to the UART 's' dump (absolute levels)
    reg        eq_en = 1'b1;
    wire       e_we, e_done;
    wire [8:0] e_x, e_l8;
    floor_eq u_eq (.clk(sclk), .en(eq_en), .in_we(ln_we), .in_x(ln_x), .in_l8(ln_l8),
                   .line_done(ln_done), .out_we(e_we), .out_x(e_x), .out_l8(e_l8), .done_out(e_done));
    reg [6:0] e_row = 7'd0, e_top = 7'd0;     // row being written / newest complete row
    always @(posedge sclk) if (ln_we && ln_x == 9'd0) e_row <= wf_row;
    always @(posedge sclk) if (e_done) e_top <= e_row;

    // ================= UART =================
    localparam integer DIV = (CLK_KHZ * 1000 + BAUD / 2) / BAUD;
    wire [7:0] rx_data;
    wire       rx_valid;
    uart_rx #(.DIV(DIV)) u_rx (.clk(sclk), .rx(uart_rx), .data(rx_data), .valid(rx_valid));
    reg        tx_start = 1'b0;
    reg  [7:0] tx_data = 8'd0;
    wire       tx_busy;
    uart_tx #(.DIV(DIV)) u_tx (.clk(sclk), .start(tx_start), .data(tx_data), .tx(uart_tx), .busy(tx_busy));

    reg  [7:0]  sbase = SBASE_INIT, wbase = WBASE_INIT;
    reg  [7:0]  gain_db = GAIN_DB;
    reg  [1:0]  zoom = ZOOM_INIT;

    // capture buffer: 2048 x 36 ({I, Q}, {audio, 0} or {L8, 0})
    localparam integer NCAP = 2048;
    localparam [11:0]  NSPEC = 12'd320;
    reg  [11:0] cwa = 12'd0, cra = 12'd0, clast = 12'd0;
    reg         cwe = 1'b0;
    reg  [35:0] cwd = 36'd0;
    reg  [35:0] cmem [0:NCAP-1];
    reg         kind_s = 1'b0;                   // spectrum dump
    reg         s_arm = 1'b0;                    // spectrum dump started at x = 0
    reg  [35:0] crd = 36'd0;
    always @(posedge sclk) begin
        if (cwe)
            cmem[cwa] <= cwd;
        crd <= cmem[cra];
    end

    localparam C_IDLE = 4'd0, C_ARG = 4'd1, C_REPLY = 4'd2, C_CAP = 4'd3, C_HDR = 4'd4,
               C_RD = 4'd5, C_BYTE = 4'd6;
    reg [3:0]  cst = C_IDLE;
    reg [7:0]  cmd = 8'd0;
    reg [2:0]  nargs = 3'd0;
    reg [7:0]  rb0 = 0, rb1 = 0, rb2 = 0, rb3 = 0;
    reg [2:0]  rlen = 3'd0, ri = 3'd0;
    reg        kind_q = 1'b0;                  // 1 = I/Q dump, 0 = audio dump
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
                        rb0 <= "I"; rb1 <= VERSION; rb2 <= level; rb3 <= {6'd0, wide, usb};
                        rlen <= 3'd4; ri <= 3'd0; cst <= C_REPLY;
                    end else if (rx_data == "f") begin
                        nargs <= 3'd4; cst <= C_ARG;
                    end else if (rx_data == "m") begin
                        nargs <= 3'd1; cst <= C_ARG;
                    end else if (rx_data == "b") begin
                        nargs <= 3'd2; cst <= C_ARG;
                    end else if (rx_data == "e" || rx_data == "g" || rx_data == "z" || rx_data == "w") begin
                        nargs <= 3'd1; cst <= C_ARG;
                    end else if (rx_data == "q" || rx_data == "a" || rx_data == "s") begin
                        kind_q <= (rx_data == "q"); kind_s <= (rx_data == "s");
                        clast <= (rx_data == "s") ? NSPEC - 12'd1 : NCAP - 1;
                        cwa <= 12'd0; s_arm <= 1'b0; cst <= C_CAP;
                    end
                end
            C_ARG:
                if (rx_valid) begin
                    u_arg <= {rx_data, u_arg[31:8]};
                    nargs <= nargs - 3'd1;
                    if (nargs == 3'd1) begin
                        if (cmd == "f") begin u_set_f <= 1'b1; rb0 <= "F"; end
                        else if (cmd == "b") begin
                            sbase <= u_arg[31:24]; wbase <= rx_data; rb0 <= "B";
                        end else if (cmd == "e") begin
                            eq_en <= rx_data[0]; rb0 <= "E";
                        end else if (cmd == "g") begin
                            gain_db <= rx_data; rb0 <= "G";
                        end else if (cmd == "w") begin
                            wide <= rx_data[0]; rb0 <= "W";
                        end else if (cmd == "z") begin
                            zoom <= (rx_data[1:0] == 2'd3) ? 2'd2 : rx_data[1:0]; rb0 <= "Z";
                        end else begin u_set_m <= 1'b1; rb0 <= "M"; end
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
            C_CAP:                             // one write per sample, clast+1 samples
                if (kind_s) begin               // one full spectrum line, x = 0..319
                    if (ln_we && (ln_x == 9'd0 || s_arm)) begin
                        s_arm <= 1'b1; cwe <= 1'b1; cwa <= {3'd0, ln_x}; cwd <= {7'd0, ln_l8, 20'd0};
                        if (ln_x == 9'd319) begin
                            rb0 <= "S"; rb1 <= NSPEC[7:0]; rb2 <= {4'd0, NSPEC[11:8]};
                            ri <= 3'd0; cst <= C_HDR;
                        end
                    end
                end else begin
                    if (cwe) begin
                        cwa <= cwa + 12'd1;
                        if (cwa == clast) begin
                            rb0 <= kind_q ? "Q" : "A"; rb1 <= NCAP[7:0]; rb2 <= NCAP[15:8];
                            ri <= 3'd0; cst <= C_HDR;
                        end
                    end
                    if ((kind_q ? ddc_v : agc_v) && !(cwe && cwa == clast)) begin
                        cwe <= 1'b1;
                        cwd <= kind_q ? {ddc_i, ddc_q} : {agc_out, 20'd0};
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
                    else if (cra == clast) cst <= C_IDLE;
                    else begin bi <= 3'd0; cra <= cra + 12'd1; rdw <= 2'd2; cst <= C_RD; end
                end
            default: cst <= C_IDLE;
        endcase
    end

    // spectrum line: mean L8 per column (0.376 dB units)
    reg [8:0] spm [0:511];
    reg [8:0] sp_rd = 9'd0;
    always @(posedge sclk) if (e_we) spm[e_x] <= e_l8;
    // waterfall: palette index v = L8 - wbase (0..255) per pixel, row-major
    wire [9:0] wv   = {1'b0, e_l8} - {2'b00, wbase};
    wire [7:0] wf_v = wv[9] ? 8'd0 : wv[8] ? 8'd255 : wv[7:0];
    reg [7:0]  wfm [0:32767];
    reg        wf_we = 1'b0;
    reg [14:0] wf_wa = 15'd0;
    reg [7:0]  wf_wd = 8'd0;
    always @(posedge sclk) wf_we <= e_we;
    always @(posedge sclk) wf_wa <= {e_row, 8'd0} + {e_row, 6'd0} + e_x;
    always @(posedge sclk) wf_wd <= wf_v;
    always @(posedge sclk) if (wf_we) wfm[wf_wa] <= wf_wd;

    // 8-bit level -> RGB565: black, blue, cyan, green, yellow, red, white
    function [15:0] palette(input [7:0] v);
        reg [4:0] f;
        begin
            f = v[4:0];
            case (v[7:5])
                3'd0: palette = {5'd0, 6'd0, 1'b0, f[4:1]};
                3'd1: palette = {5'd0, 1'b0, f, 1'b1, f[4:1]};
                3'd2: palette = {5'd0, 1'b1, f, ~f};
                3'd3: palette = {f, 6'd63, 5'd0};
                3'd4: palette = {5'd31, ~f, 1'b1, 5'd0};
                3'd5: palette = {5'd31, f, 1'b0, f};
                default: palette = 16'hFFFF;
            endcase
        end
    endfunction

    // ================= display =================
    wire [31:0] bcd;
    bin2bcd u_bcd (.clk(sclk), .bin(freq), .bcd(bcd));

    reg        ili_start_init  = 1'b0;
    reg        ili_set_window  = 1'b0;
    reg        ili_pixel_valid = 1'b0;
    reg [15:0] ili_pixel_data  = 16'd0;
    wire       ili_init_done, ili_window_done, ili_pixel_ready;
    ili9341_driver #(.CLK_KHZ(CLK_KHZ)) u_ili (
        .clk(sclk),
        .start_init(ili_start_init), .init_done(ili_init_done),
        .set_window(ili_set_window),
        .win_x0(9'd0), .win_x1(9'd319), .win_y0(8'd0), .win_y1(8'd239),
        .window_done(ili_window_done),
        .pixel_data(ili_pixel_data), .pixel_valid(ili_pixel_valid), .pixel_ready(ili_pixel_ready),
        .tft_sck(tft_sck), .tft_mosi(tft_mosi), .tft_cs(tft_cs), .tft_dc(tft_dc), .tft_rst(tft_rst)
    );

    // values frozen per frame (no tearing inside a frame)
    reg [31:0] f_bcd = 32'd0;
    reg [2:0]  f_step = 3'd2;
    reg        f_usb = 1'b0;
    reg        f_wide = 1'b0;
    reg [8:0]  f_bar = 9'd0;
    reg [6:0]  f_wtop = 7'd0;
    // bar length: x = (dBm + 127) * 288/114, dBm = -90.7 + 6.02*level/8 - gain_db
    // (2.527 px/dB = 647/256)
    wire [17:0] bar_raw = 18'd92 * 256 + level * 18'd487;
    reg  [9:0]  gain_px = 10'd0;
    always @(posedge sclk) gain_px <= ({8'd0, gain_db} * 18'd647) >> 8;
    wire signed [12:0] bar_s = $signed({1'b0, bar_raw[17:8]}) - $signed({3'b000, gain_px});
    wire [8:0] bar_px = (bar_s < 0) ? 9'd0 : (bar_s > 288) ? 9'd288 : bar_s[8:0];

    localparam [15:0] C_BLACK = 16'h0000, C_WHITE = 16'hFFFF, C_YELLOW = 16'hFFE0,
                      C_CYAN = 16'h07FF, C_GREEN = 16'h07E0, C_RED = 16'hF800,
                      C_GREY = 16'h4208, C_DGREY = 16'h2104, C_LGREY = 16'hA514,
                      C_SPEC = 16'h04DF, C_PB = 16'h0849, C_CENT = 16'h6000;

    // pixel pipeline: P0 region decode, P1 glyph code, PW1/PW2 wait for the
    // waterfall/spectrum memories, P2 colour, P3 push
    reg [8:0] x = 9'd0;
    reg [7:0] y = 8'd0;
    reg [4:0] fcx = 5'd0;                      // x within a 24-px frequency char
    reg [3:0] fidx = 4'd0;                     // frequency char index
    localparam [1:0] R_TXT = 2'd0, R_SPEC = 2'd1, R_WF = 2'd2;
    // P0 results
    reg [1:0] p_reg = R_TXT;
    reg       p_txt = 1'b0;
    reg [5:0] p_cidx = 6'd0;
    reg [1:0] p_line = 2'd0;                   // 0 mode, 1 freq, 2 meter labels, 3 kHz scale
    reg [2:0] p_row = 3'd0, p_col = 3'd0;
    reg [15:0] p_fg = 16'd0, p_bg = 16'd0;
    reg [6:0] p_h = 7'd0;                      // spectrum: height above the baseline
    // P1
    reg [1:0] q_reg = R_TXT;
    reg [5:0] q_code = 6'd0;
    reg [2:0] q_row = 3'd0, q_col = 3'd0;
    reg       q_txt = 1'b0;
    reg [15:0] q_fg = 16'd0, q_bg = 16'd0;
    reg [6:0] q_h = 7'd0;
    // P2
    reg [15:0] r_color = 16'd0;

    wire [5:0] m_code, sc_code;
    meter_rom u_m  (.a(p_cidx), .d(m_code));
    scale_rom u_sc (.a({f_wide, p_cidx}), .d(sc_code));   // narrow | wide labels
    wire [7:0] glyph;
    font_rom  u_font (.a({q_code, q_row}), .d(glyph));
    // q_code/q_row are set in S_P1 and held through PW1/PW2, so the font row
    // can be registered once (cuts the q_code -> font -> r_color path)
    reg  [7:0] glyph_q = 8'd0;
    always @(posedge sclk) glyph_q <= glyph;

    // spectrum column height: (L8 - sbase) * k/16 px, 0..SP_H; L8 = 0.376 dB:
    // zoom 0: k = 6 (1.00 dB/px), 1: k = 9 (1.5 px/dB), 2: k = 12 (2 px/dB)
    wire signed [10:0] sd = {2'b00, sp_rd} - {3'b000, sbase};
    wire signed [14:0] s3 = (zoom == 2'd0) ? sd * 6 : (zoom == 2'd1) ? sd * 9 : sd * 12;
    wire signed [14:0] s8 = s3 >>> 4;
    reg  [6:0] sp_px = 7'd0;
    always @(posedge sclk) sp_rd <= spm[x];
    always @(posedge sclk) sp_px <= (sd < 0) ? 7'd0 : (s8 > SP_H) ? SP_H : s8[6:0];
    // waterfall row: newest line (f_wtop) at the top
    wire [6:0] wr_r = y - WF_Y0;
    reg  [6:0] wf_br = 7'd0;
    reg  [14:0] wf_ra = 15'd0;
    reg  [7:0] wf_rd = 8'd0;
    reg  [15:0] wf_rgb = 16'd0;
    always @(posedge sclk) wf_br  <= (f_wtop >= wr_r) ? f_wtop - wr_r : f_wtop + WF_ROWS - wr_r;
    always @(posedge sclk) wf_ra  <= {wf_br, 8'd0} + {wf_br, 6'd0} + x;
    always @(posedge sclk) wf_rd  <= wfm[wf_ra];
    always @(posedge sclk) wf_rgb <= palette(wf_rd);

    function [5:0] dig(input [3:0] v);         // BCD digit -> font code
        dig = 6'd16 + {2'b00, v};               // '0' = 48 - 32
    endfunction
    // frequency line: " d6.d5d4d3.d2d1d0" over 10 chars of 24 px
    reg [5:0] f_code;
    always @(*)
        case (p_cidx[3:0])
            4'd0: f_code = (f_bcd[31:28] == 4'd0) ? 6'd0 : dig(f_bcd[31:28]);
            4'd1: f_code = (f_bcd[31:24] == 8'd0) ? 6'd0 : dig(f_bcd[27:24]);
            4'd2: f_code = (f_bcd[31:24] == 8'd0) ? 6'd0 : 6'd14;        // '.'
            4'd3: f_code = dig(f_bcd[23:20]);
            4'd4: f_code = dig(f_bcd[19:16]);
            4'd5: f_code = dig(f_bcd[15:12]);
            4'd6: f_code = 6'd14;
            4'd7: f_code = dig(f_bcd[11:8]);
            4'd8: f_code = dig(f_bcd[7:4]);
            default: f_code = dig(f_bcd[3:0]);
        endcase
    wire [5:0] mode_code = (p_cidx[1:0] == 2'd0) ? (f_usb ? 6'd53 : 6'd44) :  // 'U' / 'L'
                           (p_cidx[1:0] == 2'd1) ? 6'd51 : 6'd34;             // 'S'  'B'
    // underlined digit for the step
    function [3:0] step_char(input [2:0] s);
        case (s)
            3'd0: step_char = 4'd8; 3'd1: step_char = 4'd7; 3'd2: step_char = 4'd5;
            3'd3: step_char = 4'd4; default: step_char = 4'd3;
        endcase
    endfunction
    wire [8:0] ux0 = step_char(f_step) * 5'd24;
    wire under = (y == 8'd25 || y == 8'd26) && (x >= ux0 + 9'd2) && (x <= ux0 + 9'd21);
    // 5 kHz ticks (48.55 px), dial at x = 160
    wire tick   = (x == 9'd14) || (x == 9'd63) || (x == 9'd111) || (x == 9'd209) ||
                  (x == 9'd257) || (x == 9'd306);
    wire center = (x == 9'd160);
    wire [7:0] sh  = SP_Y1 - y;
    wire hgrid  = (sh == 8'd10) || (sh == 8'd20) || (sh == 8'd30) || (sh == 8'd40) ||
                  (sh == 8'd50) || (sh == 8'd60) || (sh == 8'd70);
    // SSB passband -2700..-300 Hz (LSB) / +300..+2700 Hz (USB): 26..3 bins
    // (wide: 13..2 bins of 206 Hz); ticks every 5 kHz (wide: 10 kHz) = 48.55 px
    wire pband  = f_wide ? (f_usb ? (x >= 9'd162 && x <= 9'd173) : (x >= 9'd147 && x <= 9'd158))
                         : (f_usb ? (x >= 9'd163 && x <= 9'd186) : (x >= 9'd134 && x <= 9'd157));
    wire in_bar   = (y >= 8'd30) && (y <= 8'd37) && (x >= 9'd16) && (x <= 9'd303);
    wire bar_edge = (y == 8'd30) || (y == 8'd37) || (x == 9'd16) || (x == 9'd303);
    wire bar_fill = (x - 9'd16) < f_bar;
    wire [7:0] y11 = y * 4'd11;                // frequency row = y/3 for y < 24
    wire [8:0] fcx11 = fcx * 4'd11;            // frequency col = fcx/3

    localparam [3:0] S_RESET = 4'd0, S_INIT = 4'd1, S_INIT_WAIT = 4'd2, S_WIN = 4'd3,
                     S_WIN_WAIT = 4'd4, S_P0 = 4'd5, S_P1 = 4'd6, S_P2 = 4'd7, S_P3 = 4'd8,
                     S_FEND = 4'd9, S_PW1 = 4'd10, S_PW2 = 4'd11;
    reg [3:0] dst = S_RESET;

    always @(posedge sclk) begin
        ili_start_init  <= 1'b0;
        ili_set_window  <= 1'b0;
        ili_pixel_valid <= 1'b0;
        case (dst)
            S_RESET:     if (pll_lock) dst <= S_INIT;
            S_INIT:      begin ili_start_init <= 1'b1; dst <= S_INIT_WAIT; end
            S_INIT_WAIT: if (ili_init_done) dst <= S_WIN;
            S_WIN:
                if (ili_pixel_ready) begin
                    ili_set_window <= 1'b1;
                    x <= 9'd0; y <= 8'd0; fcx <= 5'd0; fidx <= 4'd0;
                    f_bcd <= bcd; f_step <= step_idx; f_usb <= usb; f_wide <= wide; f_bar <= bar_px;
                    f_wtop <= e_top;
                    dst <= S_WIN_WAIT;
                end
            S_WIN_WAIT: if (ili_window_done) dst <= S_P0;
            S_P0: begin
                p_reg <= R_TXT; p_txt <= 1'b0; p_bg <= C_BLACK; p_fg <= C_WHITE;
                p_h <= sh[6:0];
                if (y < 8'd24 && x < 9'd240) begin                        // frequency, 3x
                    p_txt <= 1'b1; p_line <= 2'd1; p_cidx <= {2'b00, fidx};
                    p_row <= y11[7:5]; p_col <= fcx11[7:5]; p_fg <= C_YELLOW;
                end else if (y >= 8'd4 && y < 8'd20 && x >= 9'd256 && x < 9'd304) begin  // mode, 2x
                    p_txt <= 1'b1; p_line <= 2'd0; p_cidx <= {4'd0, x[5:4]};
                    p_row <= (y - 8'd4) >> 1; p_col <= x[3:1]; p_fg <= C_CYAN;
                end else if (under) begin
                    p_bg <= C_YELLOW;
                end else if (in_bar) begin
                    p_bg <= bar_edge ? C_GREY : bar_fill ? ((x < 9'd152) ? C_GREEN : C_RED) : C_DGREY;
                end else if (y >= 8'd40 && y < 8'd48) begin             // S-meter labels
                    p_txt <= 1'b1; p_line <= 2'd2; p_cidx <= x[8:3];
                    p_row <= y - 8'd40; p_col <= x[2:0]; p_fg <= C_LGREY;
                end else if (y >= SP_Y0 && y <= SP_Y1) begin            // spectrum
                    p_reg <= R_SPEC;
                    p_bg  <= center ? C_CENT : (hgrid || tick) ? C_DGREY : pband ? C_PB : C_BLACK;
                end else if (y >= 8'd126 && y <= 8'd129) begin          // ticks
                    p_bg <= center ? C_RED : tick ? C_GREY : C_BLACK;
                end else if (y >= 8'd130 && y < WF_Y0) begin            // kHz scale
                    p_txt <= 1'b1; p_line <= 2'd3; p_cidx <= x[8:3];
                    p_row <= y - 8'd130; p_col <= x[2:0]; p_fg <= C_LGREY;
                end else if (y >= WF_Y0) begin
                    p_reg <= R_WF;
                end
                dst <= S_P1;
            end
            S_P1: begin
                q_code <= (p_line == 2'd0) ? mode_code :
                          (p_line == 2'd1) ? f_code :
                          (p_line == 2'd2) ? m_code : sc_code;
                q_row <= p_row; q_col <= p_col; q_txt <= p_txt; q_fg <= p_fg; q_bg <= p_bg;
                q_reg <= p_reg; q_h <= p_h;
                dst <= S_PW1;
            end
            S_PW1: dst <= S_PW2;
            S_PW2: dst <= S_P2;
            S_P2: begin
                case (q_reg)
                    R_SPEC:  r_color <= (q_h + 7'd1 == sp_px) ? C_WHITE : (q_h < sp_px) ? C_SPEC : q_bg;
                    R_WF:    r_color <= wf_rgb;
                    default: r_color <= (q_txt && glyph_q[3'd7 - q_col]) ? q_fg : q_bg;
                endcase
                dst <= S_P3;
            end
            S_P3:
                if (ili_pixel_ready) begin
                    ili_pixel_data  <= r_color;
                    ili_pixel_valid <= 1'b1;
                    if (x == 9'd319) begin
                        x <= 9'd0; fcx <= 5'd0; fidx <= 4'd0;
                        if (y == 8'd239) dst <= S_FEND;   // 1-cycle gap: driver drops ready
                        else begin y <= y + 8'd1; dst <= S_P0; end
                    end else begin
                        x <= x + 9'd1;
                        if (fcx == 5'd23) begin fcx <= 5'd0; fidx <= fidx + 4'd1; end
                        else fcx <= fcx + 5'd1;
                        dst <= S_P0;
                    end
                end
            S_FEND: dst <= S_WIN;
            default: dst <= S_RESET;
        endcase
    end
endmodule
