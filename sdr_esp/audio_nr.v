// LMS noise reduction / automatic notch (improvement-from-UHSDR.md item 8;
// the leaky NLMS line enhancer of WDSP's ANR as used by mcHF/UHSDR).
//
// An adaptive FIR predicts the current sample from the past:
//     d[k]   = x[n - DELAY - k],  k = 0..TAPS-1
//     y      = sum w[k] d[k]                  prediction
//     e      = x[n] - y                       prediction error
//     w[k]  += mu * e * d[k] / sigma - w[k] / 2^LEAK,   sigma = sum d[k]^2
// What is predictable over DELAY samples (voice harmonics, carriers, whistles)
// ends up in y, white-ish noise mostly in e. mode 1 = NR: out = y (the noise
// is suppressed); mode 2 = ANF: out = e (steady tones removed); mode 0: out =
// x (the filter keeps adapting, so switching is instant).
//
// Step size as WDSP (two_mu = 1e-4 at 48kHz, i.e. ~3.6e-4 at our 13.2kHz):
// mu = 2^-(10+level) = 9.8e-4..1.2e-4. A strong steady tone (most of the
// power in one eigen-mode) still converges in ~2^(10+level) samples
// (0.1-0.6s), while white noise, spread over all 64 modes, adapts 64x slower,
// so the filter does not chase it. (The first version used 2^-(3+level), 150-
// 1250x larger: the filter followed the noise itself - "bubbling", and ANF
// ate voice pitch harmonics.)
//
// Placed AFTER audio_agc: the level there is roughly constant, which keeps
// the fixed-point normalization simple. Python model (white noise, voice-like
// harmonic signal, 2026-10-24): output SNR +5..7dB for mu = 2^-4..2^-6.
//
// Fixed point: x, d 16-bit audio (sign-extended to 18), weights 32-bit with
// 24 fractional bits; the filter uses them rounded to 18 bits (16 fractional,
// saturated to +-2). sigma ~ 2^L (L = leading one) turns the division into a
// right shift: dw = (e*d) >> (L + MU - 24), MU = 10 + level, rounded to
// nearest (a truncating shift would bias every update by -1/2 LSB, which
// the slow adaptation would integrate). The shift is never negative - for
// input with sigma < 2^(24-MU) (rms < ~2 LSB) the step just gets smaller.
//
// One multiplier, time-shared: per sample 64 x (w*d, d*d) for the filter, then
// 64 x (e*d) for the update, 6 cycles per tap, ~780 cycles in all; inputs
// come every 2048. out_valid ~390 cycles after in_valid (right after the
// filter pass). Two BSRAMs: the delay line (128 x 18) and the weights
// (64 x 32).
module audio_nr #(
    parameter TAPS  = 64,
    parameter DELAY = 16,
    parameter LEAK  = 20      // weights decay with time constant 2^20 samples
) (
    input  wire               clk,
    input  wire [1:0]         mode,        // 0 off, 1 NR, 2 ANF (3 = ANF)
    input  wire [1:0]         level,       // mu = 2^-(10+level)
    input  wire               in_valid,
    input  wire signed [15:0] in_audio,
    output reg                out_valid = 1'b0,
    output reg  signed [15:0] out_audio = 16'sd0
);

    integer i;
    // ---- delay line, 128 x 18 (single-port BSRAM pattern) ----
    reg  [6:0]         d_addr = 7'd0;
    reg                d_we   = 1'b0;
    reg  signed [17:0] d_din  = 18'sd0;
    reg  signed [17:0] d_dout = 18'sd0;
    reg  signed [17:0] d_mem [0:127];
    initial for (i = 0; i < 128; i = i + 1) d_mem[i] = 18'sd0;
    always @(posedge clk) begin
        if (d_we) d_mem[d_addr] <= d_din;
        d_dout <= d_mem[d_addr];
    end

    // ---- weights, 64 x 32 ----
    reg  [5:0]         w_addr = 6'd0;
    reg                w_we   = 1'b0;
    reg  signed [31:0] w_din  = 32'sd0;
    reg  signed [31:0] w_dout = 32'sd0;
    reg  signed [31:0] w_mem [0:63];
    initial for (i = 0; i < 64; i = i + 1) w_mem[i] = 32'sd0;
    always @(posedge clk) begin
        if (w_we) w_mem[w_addr] <= w_din;
        w_dout <= w_mem[w_addr];
    end

    // ---- the multiplier ----
    reg  signed [17:0] ma = 18'sd0, mb = 18'sd0;
    reg  signed [35:0] p  = 36'sd0;
    always @(posedge clk) p <= ma * mb;

    // weight rounded to 18 bits, 16 fractional, saturated
    wire signed [23:0] w_hi = w_dout >>> 8;
    wire signed [17:0] w18  = (w_hi >  24'sd131071) ?  18'sd131071 :
                              (w_hi < -24'sd131072) ? -18'sd131072 : w_hi[17:0];

    // ---- FSM ----
    localparam [1:0] S_IDLE = 2'd0, S_FILT = 2'd1, S_OUT = 2'd2, S_UPD = 2'd3;
    reg  [1:0]  st   = S_IDLE;
    reg  [2:0]  t    = 3'd0;        // cycle within a tap, 0..5
    reg  [5:0]  k    = 6'd0;        // tap
    reg  [6:0]  wp   = 7'd0;        // newest sample in d_mem
    reg  signed [17:0] x = 18'sd0;
    reg  signed [41:0] yacc = 42'sd0;
    reg  [37:0] sacc = 38'd0;       // sigma = sum d^2 < 64 * 2^30
    reg  signed [17:0] e = 18'sd0;
    reg  [5:0]  sh = 6'd0;
    reg  signed [31:0] w_old = 32'sd0;

    wire [6:0]  tap_addr = wp - DELAY[6:0] - {1'b0, k};

    wire signed [25:0] y_full = yacc >>> 16;
    wire signed [17:0] y_sat  = (y_full >  26'sd131071) ?  18'sd131071 :
                                (y_full < -26'sd131072) ? -18'sd131072 : y_full[17:0];
    wire signed [18:0] e_full = x - y_sat;
    wire signed [17:0] e_sat  = (e_full >  19'sd131071) ?  18'sd131071 :
                                (e_full < -19'sd131072) ? -18'sd131072 : e_full[17:0];

    // L = position of sigma's leading one; sh = max(L + MU - 24, 0)
    function [5:0] lead1(input [37:0] v);
        integer b;
        begin
            lead1 = 6'd0;
            for (b = 0; b < 38; b = b + 1) if (v[b]) lead1 = b;
        end
    endfunction
    wire [5:0] mu_sh = 6'd10 + {4'd0, level};
    wire [6:0] l_mu  = {1'b0, lead1(sacc)} + {1'b0, mu_sh};
    wire [5:0] sh_new = (l_mu > 7'd24) ? l_mu[5:0] - 6'd24 : 6'd0;

    // update: dw = round((e*d) >> sh), w_new = w + dw - w >> LEAK, saturated
    wire signed [35:0] dw_h = p >>> (sh - 6'd1);              // one bit more
    wire signed [35:0] dw   = (sh == 6'd0) ? p : ((dw_h + 36'sd1) >>> 1);
    reg  signed [35:0] dw_r = 36'sd0;                          // pipeline: shift | add
    always @(posedge clk) dw_r <= dw;
    wire signed [36:0] w_nx = w_old + dw_r - (w_old >>> LEAK);
    wire signed [31:0] w_new = (w_nx >  37'sd2147483647) ?  32'sd2147483647 :
                               (w_nx < -37'sd2147483648) ? -32'sd2147483648 : w_nx[31:0];

    function signed [15:0] sat16(input signed [17:0] v);
        sat16 = (v > 18'sd32767) ? 16'sd32767 : (v < -18'sd32768) ? -16'sd32768 : v[15:0];
    endfunction

    always @(posedge clk) begin
        out_valid <= 1'b0;
        d_we      <= 1'b0;
        w_we      <= 1'b0;
        case (st)
            S_IDLE: if (in_valid) begin
                        x      <= in_audio;
                        d_addr <= wp + 7'd1;           // store the new sample
                        d_din  <= in_audio;
                        d_we   <= 1'b1;
                        wp     <= wp + 7'd1;
                        yacc   <= 42'sd0;
                        sacc   <= 38'd0;
                        k      <= 6'd0;
                        t      <= 3'd0;
                        st     <= S_FILT;
                    end
            // per tap: t0 addresses, t2 data -> w*d, t3 d*d, t4 y +=, t5 sigma +=
            S_FILT: begin
                        case (t)
                            3'd0: begin d_addr <= tap_addr; w_addr <= k; end
                            3'd2: begin ma <= w18;    mb <= d_dout; end
                            3'd3: begin ma <= d_dout; mb <= d_dout; end
                            3'd4: yacc <= yacc + p;
                            3'd5: sacc <= sacc + p[35:0];
                            default: ;
                        endcase
                        if (t == 3'd5) begin
                            t <= 3'd0;
                            k <= k + 6'd1;
                            if (k == TAPS - 1) st <= S_OUT;
                        end else t <= t + 3'd1;
                    end
            S_OUT:  begin
                        e         <= e_sat;
                        sh        <= sh_new;
                        out_audio <= (mode == 2'd0) ? sat16(x) : (mode == 2'd1) ? sat16(y_sat) : sat16(e_sat);
                        out_valid <= 1'b1;
                        k         <= 6'd0;
                        t         <= 3'd0;
                        st        <= S_UPD;
                    end
            // per tap: t0 addresses, t2 e*d + old weight, t4 dw_r, t5 write the
            // new one (lands at the next t0 edge, w_addr is still k then)
            S_UPD:  begin
                        case (t)
                            3'd0: begin d_addr <= tap_addr; w_addr <= k; end
                            3'd2: begin ma <= e; mb <= d_dout; w_old <= w_dout; end
                            3'd5: begin w_din <= w_new; w_we <= 1'b1; end
                            default: ;
                        endcase
                        if (t == 3'd5) begin
                            t <= 3'd0;
                            k <= k + 6'd1;
                            if (k == TAPS - 1) st <= S_IDLE;
                        end else t <= t + 3'd1;
                    end
        endcase
    end

endmodule
