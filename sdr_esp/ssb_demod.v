// JessSDR sdr_esp fork of ../sdr/ssb_demod.v (2026-11-15): the sideband /
// audio filter is PROGRAMMABLE - 255 taps (was 127, fixed LSB 300..2700 Hz)
// in a BSRAM the ESP writes over SPI (cf_we/cf_waddr/cf_wdata), so filter
// width, IF shift and CW filters need no new bitstream. The ESP computes
// the taps exactly like the old offline generator: Blackman-windowed sinc
// low-pass (fc = half the width), shifted to the passband centre, Q1.17:
//   h[k] = 2fc/fs sinc(2fc/fs (k-127)) blackman(k) exp(j 2pi f0 (k-127)/fs)
// The passband is always on the NEGATIVE side (LSB); USB/CW negate Q before
// this module. cf_coef.hex = power-up contents (LSB 300..2700 Hz, from
// gen_cf_coef.py). Steps below as in the original, with 255 taps.
//
// LSB demodulator: zero-IF I/Q at fs = 52.734kHz in, real audio at
// fs/4 = 13.184kHz out. Fed from the same I/Q pair as the FFT (after
// hum_hpf and IQ_SWAP, tuned frequency at 0Hz, positive = above the dial).
//
//   1. Decimate by 4: 63-tap Blackman-windowed-sinc low-pass (fc 6kHz) on I
//      and Q, computed only for every 4th input. Everything that would
//      alias into +-3.3kHz is down >87dB.
//   2. Sideband selection + audio filter in one: a 127-tap COMPLEX FIR, the
//      low-pass prototype (fc 1350Hz) shifted to -1500Hz, so it passes
//      -2700..-300Hz (LSB, within 0.5dB) and rejects the upper sideband
//      +300..+2700Hz by 76-96dB. The real part of its output is the audio:
//        audio = sum(h_re[k]*I[n-k]) - sum(h_im[k]*Q[n-k])
//      A tone at dial-f comes out as audio at f.
//
// Coefficients are Q1.17 (18-bit signed), generated offline (Python,
// Blackman window) - the ROM tables below. Both FIRs run on the same two
// 18x18 multipliers (one per I/Q lane), one tap per clock, pipelined:
// issue (history address) -> BSRAM read + coefficient -> multiply ->
// accumulate. Histories are two circular buffers in BSRAM ({I,Q} per
// word): 64 deep at 52.7kHz, 256 deep at 13.2kHz.
//
// Timing: every 4th input triggers ~330 cycles of work (63 + 255 taps +
// pipeline drain); inputs arriving while busy are dropped, so inputs must
// be >= ~340 cycles apart (real pairs arrive every 1229).
// out_valid: single-cycle pulse per audio sample.
// dec_valid/dec_i/dec_q: the decimated complex stream (13.184kHz, +-6kHz
// wide) for am_demod.v; dec_valid pulses once per decimated sample, the
// values stay until the next one.
module ssb_demod (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    output reg                out_valid = 1'b0,
    output reg  signed [17:0] out_audio = 18'sd0,
    output reg                dec_valid = 1'b0,
    output reg  signed [17:0] dec_i = 18'sd0,
    output reg  signed [17:0] dec_q = 18'sd0,
    input  wire               cf_we,      // write one complex tap
    input  wire        [7:0]  cf_waddr,
    input  wire        [35:0] cf_wdata    // {re[17:0], im[17:0]}
);

    function signed [17:0] dec_coef(input [5:0] k);
        case (k)
            6'd0: dec_coef =  18'sd0;
            6'd1: dec_coef =  18'sd1;
            6'd2: dec_coef =  18'sd5;
            6'd3: dec_coef =  18'sd12;
            6'd4: dec_coef =  18'sd11;
            6'd5: dec_coef = -18'sd10;
            6'd6: dec_coef = -18'sd52;
            6'd7: dec_coef = -18'sd91;
            6'd8: dec_coef = -18'sd87;
            6'd9: dec_coef = -18'sd3;
            6'd10: dec_coef =  18'sd153;
            6'd11: dec_coef =  18'sd311;
            6'd12: dec_coef =  18'sd347;
            6'd13: dec_coef =  18'sd155;
            6'd14: dec_coef = -18'sd264;
            6'd15: dec_coef = -18'sd743;
            6'd16: dec_coef = -18'sd980;
            6'd17: dec_coef = -18'sd689;
            6'd18: dec_coef =  18'sd200;
            6'd19: dec_coef =  18'sd1387;
            6'd20: dec_coef =  18'sd2243;
            6'd21: dec_coef =  18'sd2063;
            6'd22: dec_coef =  18'sd492;
            6'd23: dec_coef = -18'sd2120;
            6'd24: dec_coef = -18'sd4633;
            6'd25: dec_coef = -18'sd5442;
            6'd26: dec_coef = -18'sd3149;
            6'd27: dec_coef =  18'sd2713;
            6'd28: dec_coef =  18'sd11245;
            6'd29: dec_coef =  18'sd20310;
            6'd30: dec_coef =  18'sd27237;
            6'd31: dec_coef =  18'sd29829;
            6'd32: dec_coef =  18'sd27237;
            6'd33: dec_coef =  18'sd20310;
            6'd34: dec_coef =  18'sd11245;
            6'd35: dec_coef =  18'sd2713;
            6'd36: dec_coef = -18'sd3149;
            6'd37: dec_coef = -18'sd5442;
            6'd38: dec_coef = -18'sd4633;
            6'd39: dec_coef = -18'sd2120;
            6'd40: dec_coef =  18'sd492;
            6'd41: dec_coef =  18'sd2063;
            6'd42: dec_coef =  18'sd2243;
            6'd43: dec_coef =  18'sd1387;
            6'd44: dec_coef =  18'sd200;
            6'd45: dec_coef = -18'sd689;
            6'd46: dec_coef = -18'sd980;
            6'd47: dec_coef = -18'sd743;
            6'd48: dec_coef = -18'sd264;
            6'd49: dec_coef =  18'sd155;
            6'd50: dec_coef =  18'sd347;
            6'd51: dec_coef =  18'sd311;
            6'd52: dec_coef =  18'sd153;
            6'd53: dec_coef = -18'sd3;
            6'd54: dec_coef = -18'sd87;
            6'd55: dec_coef = -18'sd91;
            6'd56: dec_coef = -18'sd52;
            6'd57: dec_coef = -18'sd10;
            6'd58: dec_coef =  18'sd11;
            6'd59: dec_coef =  18'sd12;
            6'd60: dec_coef =  18'sd5;
            6'd61: dec_coef =  18'sd1;
            6'd62: dec_coef =  18'sd0;
            default: dec_coef = 18'sd0;
        endcase
    endfunction

    // ---- programmable complex filter: 256 x {re, im} Q1.17, written by the
    // ESP (spi_regs 0x19..0x1E); power-up contents = LSB 300..2700 Hz ----
    reg        [35:0] cf_mem [0:255];
    initial $readmemh("cf_coef.hex", cf_mem);
    reg        [7:0]  cf_raddr = 8'd0;
    reg        [35:0] cf_dout  = 36'd0;
    always @(posedge clk) if (cf_we) cf_mem[cf_waddr] <= cf_wdata;
    always @(posedge clk) cf_dout <= cf_mem[cf_raddr];

    // ---- history buffers (BSRAM single-port sync-read pattern) ----
    reg        [5:0]  d_addr = 6'd0;
    reg               d_we   = 1'b0;
    reg        [35:0] d_din  = 36'd0;
    reg        [35:0] d_dout = 36'd0;
    reg        [35:0] d_mem [0:63];
    always @(posedge clk) begin
        if (d_we) d_mem[d_addr] <= d_din;
        d_dout <= d_mem[d_addr];
    end

    reg        [7:0]  c_addr = 8'd0;
    reg               c_we   = 1'b0;
    reg        [35:0] c_din  = 36'd0;
    reg        [35:0] c_dout = 36'd0;
    reg        [35:0] c_mem [0:255];
    // zero histories as in the hardware (BSRAM powers up 0); in simulation the
    // 256-deep CF history would otherwise give X audio for the first ~19 ms
    integer ii;
    initial for (ii = 0; ii < 256; ii = ii + 1) c_mem[ii] = 36'd0;
    initial for (ii = 0; ii < 64; ii = ii + 1) d_mem[ii] = 36'd0;
    always @(posedge clk) begin
        if (c_we) c_mem[c_addr] <= c_din;
        c_dout <= c_mem[c_addr];
    end

    // ---- control ----
    localparam [2:0] S_IDLE = 3'd0, S_DEC = 3'd1, S_DEC_DRAIN = 3'd2,
                     S_CF = 3'd3, S_CF_DRAIN = 3'd4;
    reg [2:0] state = S_IDLE;
    reg [1:0] phase = 2'd0;        // input count mod 4
    reg [5:0] d_wp  = 6'd0;        // next d_mem write slot
    reg [7:0] c_wp  = 8'd0;        // next c_mem write slot
    reg [5:0] d_new = 6'd0;        // slot of the newest d_mem sample
    reg [7:0] c_new = 8'd0;        // slot of the newest c_mem sample
    reg [7:0] k     = 8'd0;        // tap being issued
    reg       mode_cf = 1'b0;      // 0 = decimator taps, 1 = LSB taps

    // pipeline: v0 issue -> v1 data+coef ready -> v2 products ready
    reg        v0 = 1'b0, v1 = 1'b0, v2 = 1'b0;
    reg  [7:0] k0 = 8'd0;
    reg signed [17:0] coef_a = 18'sd0, coef_b = 18'sd0;
    reg signed [35:0] p_a = 36'sd0, p_b = 36'sd0;
    reg signed [43:0] acc_a = 44'sd0, acc_b = 44'sd0;

    wire [35:0] hist = mode_cf ? c_dout : d_dout;
    wire signed [17:0] hist_i = hist[35:18];
    wire signed [17:0] hist_q = hist[17:0];

    // round(acc >> 17), saturated to 18 bits
    function signed [17:0] scale_sat(input signed [43:0] a);
        reg signed [43:0] r;
        begin
            r = (a + 44'sd65536) >>> 17;
            if (r > 44'sd131071)       scale_sat =  18'sd131071;
            else if (r < -44'sd131072) scale_sat = -18'sd131072;
            else                       scale_sat = r[17:0];
        end
    endfunction

    wire signed [17:0] dec_i_out = scale_sat(acc_a);
    wire signed [17:0] dec_q_out = scale_sat(acc_b);

    always @(posedge clk) begin
        d_we      <= 1'b0;
        c_we      <= 1'b0;
        out_valid <= 1'b0;
        dec_valid <= 1'b0;
        v0        <= 1'b0;

        case (state)
            S_IDLE: begin
                if (in_valid) begin
                    d_addr <= d_wp;
                    d_din  <= {in_i, in_q};
                    d_we   <= 1'b1;
                    d_new  <= d_wp;
                    d_wp   <= d_wp + 6'd1;
                    phase  <= phase + 2'd1;
                    if (phase == 2'd3) begin
                        mode_cf <= 1'b0;
                        k       <= 8'd0;
                        acc_a   <= 44'sd0;
                        acc_b   <= 44'sd0;
                        state   <= S_DEC;
                    end
                end
            end
            S_DEC: begin
                d_addr <= d_new - k[5:0];
                k0     <= k;
                v0     <= 1'b1;
                if (k == 8'd62) state <= S_DEC_DRAIN;
                else            k     <= k + 8'd1;
            end
            S_DEC_DRAIN: begin
                if (!v0 && !v1 && !v2) begin
                    // decimated sample -> LSB filter history
                    c_addr  <= c_wp;
                    c_din   <= {dec_i_out, dec_q_out};
                    c_we    <= 1'b1;
                    dec_i     <= dec_i_out;
                    dec_q     <= dec_q_out;
                    dec_valid <= 1'b1;
                    c_new   <= c_wp;
                    c_wp    <= c_wp + 8'd1;
                    mode_cf <= 1'b1;
                    k       <= 8'd0;
                    acc_a   <= 44'sd0;
                    state   <= S_CF;
                end
            end
            S_CF: begin
                c_addr   <= c_new - k;
                cf_raddr <= k;                 // tap k's coefficient, same latency as c_mem
                k0       <= k;
                v0       <= 1'b1;
                if (k == 8'd254) state <= S_CF_DRAIN;
                else             k     <= k + 8'd1;
            end
            S_CF_DRAIN: begin
                if (!v0 && !v1 && !v2) begin
                    out_audio <= dec_i_out;   // same round/saturate of acc_a
                    out_valid <= 1'b1;
                    state     <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
        endcase

        // ---- MAC pipeline (runs alongside the FSM) ----
        v1 <= v0;
        if (v0) begin
            coef_a <= dec_coef(k0[5:0]);
            coef_b <= dec_coef(k0[5:0]);
        end
        v2 <= v1;
        if (v1) begin
            // CF taps come straight from cf_mem (read issued with the history read)
            p_a <= hist_i * (mode_cf ? $signed(cf_dout[35:18]) : coef_a);
            p_b <= hist_q * (mode_cf ? $signed(cf_dout[17:0])  : coef_b);
        end
        if (v2) begin
            if (mode_cf) acc_a <= acc_a + p_a - p_b;
            else begin
                acc_a <= acc_a + p_a;
                acc_b <= acc_b + p_b;
            end
        end
    end

endmodule
