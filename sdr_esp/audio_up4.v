// x4 audio interpolator (2026-11-21): 13.18 kHz -> 52.73 kHz in front of
// ../sdr/audio_i2s.v (instantiated with IN_PERIOD = PERIOD), whose linear
// interpolation to 48.21 kHz alone left images of the audio around
// 13.18 kHz only ~21 dB down (a 3 kHz tone -> 10.2 kHz at -21 dB, AM 5 kHz
// -> 8.2 kHz at -15 dB: "sand" on the highs). Now: 96-tap polyphase
// low-pass (gen_up4_coef.py: flat to 4.5 kHz, >= 77 dB down from 7.2 kHz,
// gain 4, Q2.16), 24 taps per output; the linear step then only has to
// move 52.7 -> 48.2 kHz (its images land >= 49 dB down for 3 kHz audio).
//
// Input: one sample every 4*PERIOD clocks EXACTLY (the demodulator rate);
// output: 4 samples per input, PERIOD clocks apart, the first ~28 clk
// after in_valid -> exactly periodic, as audio_i2s needs.
//   y[4n+p] = sum_j h[4j+p] * x[n-j],  p = 0..3, j = 0..23
// One multiplier, one tap per clock (issue -> product -> accumulate).
module audio_up4 #(
    parameter integer PERIOD = 1229
) (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [15:0] in_audio,
    output reg                out_valid = 1'b0,
    output reg  signed [15:0] out_audio = 16'sd0
);
    function signed [17:0] coef(input [6:0] k);
        case (k)
            7'd0: coef =  18'sd6;
            7'd1: coef =  18'sd9;
            7'd2: coef =  18'sd5;
            7'd3: coef = -18'sd10;
            7'd4: coef = -18'sd31;
            7'd5: coef = -18'sd47;
            7'd6: coef = -18'sd38;
            7'd7: coef =  18'sd3;
            7'd8: coef =  18'sd69;
            7'd9: coef =  18'sd127;
            7'd10: coef =  18'sd135;
            7'd11: coef =  18'sd60;
            7'd12: coef = -18'sd88;
            7'd13: coef = -18'sd249;
            7'd14: coef = -18'sd329;
            7'd15: coef = -18'sd246;
            7'd16: coef =  18'sd15;
            7'd17: coef =  18'sd364;
            7'd18: coef =  18'sd628;
            7'd19: coef =  18'sd625;
            7'd20: coef =  18'sd267;
            7'd21: coef = -18'sd356;
            7'd22: coef = -18'sd973;
            7'd23: coef = -18'sd1236;
            7'd24: coef = -18'sd894;
            7'd25: coef =  18'sd37;
            7'd26: coef =  18'sd1205;
            7'd27: coef =  18'sd2036;
            7'd28: coef =  18'sd1984;
            7'd29: coef =  18'sd845;
            7'd30: coef = -18'sd1048;
            7'd31: coef = -18'sd2866;
            7'd32: coef = -18'sd3611;
            7'd33: coef = -18'sd2606;
            7'd34: coef =  18'sd63;
            7'd35: coef =  18'sd3406;
            7'd36: coef =  18'sd5819;
            7'd37: coef =  18'sd5756;
            7'd38: coef =  18'sd2541;
            7'd39: coef = -18'sd3070;
            7'd40: coef = -18'sd8866;
            7'd41: coef = -18'sd11826;
            7'd42: coef = -18'sd9244;
            7'd43: coef =  18'sd81;
            7'd44: coef =  18'sd15100;
            7'd45: coef =  18'sd32569;
            7'd46: coef =  18'sd47968;
            7'd47: coef =  18'sd56979;
            7'd48: coef =  18'sd56979;
            7'd49: coef =  18'sd47968;
            7'd50: coef =  18'sd32569;
            7'd51: coef =  18'sd15100;
            7'd52: coef =  18'sd81;
            7'd53: coef = -18'sd9244;
            7'd54: coef = -18'sd11826;
            7'd55: coef = -18'sd8866;
            7'd56: coef = -18'sd3070;
            7'd57: coef =  18'sd2541;
            7'd58: coef =  18'sd5756;
            7'd59: coef =  18'sd5819;
            7'd60: coef =  18'sd3406;
            7'd61: coef =  18'sd63;
            7'd62: coef = -18'sd2606;
            7'd63: coef = -18'sd3611;
            7'd64: coef = -18'sd2866;
            7'd65: coef = -18'sd1048;
            7'd66: coef =  18'sd845;
            7'd67: coef =  18'sd1984;
            7'd68: coef =  18'sd2036;
            7'd69: coef =  18'sd1205;
            7'd70: coef =  18'sd37;
            7'd71: coef = -18'sd894;
            7'd72: coef = -18'sd1236;
            7'd73: coef = -18'sd973;
            7'd74: coef = -18'sd356;
            7'd75: coef =  18'sd267;
            7'd76: coef =  18'sd625;
            7'd77: coef =  18'sd628;
            7'd78: coef =  18'sd364;
            7'd79: coef =  18'sd15;
            7'd80: coef = -18'sd246;
            7'd81: coef = -18'sd329;
            7'd82: coef = -18'sd249;
            7'd83: coef = -18'sd88;
            7'd84: coef =  18'sd60;
            7'd85: coef =  18'sd135;
            7'd86: coef =  18'sd127;
            7'd87: coef =  18'sd69;
            7'd88: coef =  18'sd3;
            7'd89: coef = -18'sd38;
            7'd90: coef = -18'sd47;
            7'd91: coef = -18'sd31;
            7'd92: coef = -18'sd10;
            7'd93: coef =  18'sd5;
            7'd94: coef =  18'sd9;
            7'd95: coef =  18'sd6;
            default: coef = 18'sd0;
        endcase
    endfunction

    // ---- history: x[n] .. x[n-23] ----
    reg signed [15:0] hist [0:23];
    integer hi;
    initial for (hi = 0; hi < 24; hi = hi + 1) hist[hi] = 16'sd0;
    always @(posedge clk) if (in_valid) hist[0] <= in_audio;
    genvar g;
    generate
        for (g = 1; g < 24; g = g + 1) begin : g_hist
            always @(posedge clk) if (in_valid) hist[g] <= hist[g - 1];
        end
    endgenerate

    // ---- phase timer: start pulses at +1, +PERIOD+1, +2*PERIOD+1, +3*PERIOD+1 ----
    reg [10:0] tcnt = 11'd0;
    reg [1:0]  ph = 2'd3;                     // 3 = idle until the first input
    reg        start = 1'b0;
    wire       tick = (tcnt == PERIOD - 1) && (ph != 2'd3);
    always @(posedge clk)
        if (in_valid)  tcnt <= 11'd0;
        else if (tick) tcnt <= 11'd0;
        else if (tcnt != 11'h7FF) tcnt <= tcnt + 11'd1;
    always @(posedge clk)
        if (in_valid)  ph <= 2'd0;
        else if (tick) ph <= ph + 2'd1;
    always @(posedge clk) start <= in_valid || tick;

    // ---- MAC ----
    reg        busy = 1'b0;
    reg [4:0]  j = 5'd0;
    wire       j_last = (j == 5'd23);
    always @(posedge clk)
        if (start)                busy <= 1'b1;
        else if (busy && j_last)  busy <= 1'b0;
    always @(posedge clk)
        if (start)     j <= 5'd0;
        else if (busy) j <= j + 5'd1;

    reg signed [15:0] h_a = 16'sd0;
    reg signed [17:0] c_a = 18'sd0;
    reg               v1 = 1'b0, l1 = 1'b0;
    always @(posedge clk) h_a <= hist[j];
    always @(posedge clk) c_a <= coef({j, ph});       // 4j + p
    always @(posedge clk) v1  <= busy;
    always @(posedge clk) l1  <= busy && j_last;

    reg signed [33:0] prod = 34'sd0;
    reg               v2 = 1'b0, l2 = 1'b0;
    always @(posedge clk) prod <= h_a * c_a;
    always @(posedge clk) v2   <= v1;
    always @(posedge clk) l2   <= l1;

    reg signed [39:0] acc = 40'sd0;
    reg               l3 = 1'b0;
    always @(posedge clk)
        if (start)   acc <= 40'sd0;
        else if (v2) acc <= acc + {{6{prod[33]}}, prod};
    always @(posedge clk) l3 <= l2;

    // round(acc >> 16), saturated to 16 bits
    wire signed [39:0] r = (acc + 40'sd32768) >>> 16;
    always @(posedge clk)
        if (l3) out_audio <= (r > 40'sd32767) ? 16'sd32767 : (r < -40'sd32768) ? -16'sd32768 : r[15:0];
    always @(posedge clk) out_valid <= l3;
endmodule
