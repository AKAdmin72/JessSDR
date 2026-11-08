// Audio out to the onboard MAX98357A (I2S, mono sample in both slots).
//
// Input: 16-bit samples every IN_PERIOD clocks exactly (ssb_demod/agc at
// 64.8 MHz / (1229*4) = 13.181 kHz). Output: Philips I2S, BCLK = clk/21
// (3.086 MHz, 10/11-cycle halves), 64 BCLK per frame -> fs = 48.214 kHz
// (0.45 % above 48 kHz; the amp has no MCLK and locks to BCLK/LRCK).
// Resampling 13.18 -> 48.21 kHz by linear interpolation between the last
// two input samples, using the clock count since the newest one as the
// fraction (one input period of latency, no rate drift: both rates come
// from the same clock).
module audio_i2s #(
    parameter integer IN_PERIOD = 4916
) (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [15:0] in_audio,
    output reg                bclk = 1'b0,
    output wire               lrck,
    output reg                din = 1'b0
);
    // ---- interpolator ----
    reg signed [15:0] a0 = 16'sd0, a1 = 16'sd0;   // previous, newest
    reg [12:0] t = 13'd0;
    always @(posedge clk) if (in_valid) a0 <= a1;
    always @(posedge clk) if (in_valid) a1 <= in_audio;
    always @(posedge clk)
        if (in_valid) t <= 13'd0;
        else if (t != 13'h1FFF) t <= t + 13'd1;
    // frac = t / IN_PERIOD in Q16: t * round(2^27 / IN_PERIOD) >> 11
    localparam integer KF = (134217728 + IN_PERIOD / 2) / IN_PERIOD;   // 27303 for 4916
    wire [31:0] frac_w = t * KF;
    wire [16:0] frac   = (frac_w[31:11] > 21'd65535) ? 17'd65535 : {1'b0, frac_w[26:11]};
    wire signed [16:0] diff = a1 - a0;
    wire signed [34:0] prod = diff * $signed({1'b0, frac});
    wire signed [16:0] istep  = prod[32:16];      // |diff*frac/2^16| <= |diff|: 17 bits
    wire signed [16:0] isum   = a0 + istep;       // lies between a0 and a1 -> fits 16 bits
    wire signed [15:0] interp = isum[15:0];

    // ---- I2S ----
    reg [4:0] bdiv = 5'd0;                        // 0..20
    reg [5:0] bcnt = 6'd0;
    reg [15:0] smp = 16'd0;
    wire fall = (bdiv == 5'd20);                  // BCLK goes low here
    wire [5:0] bnext = bcnt + 6'd1;
    assign lrck = bcnt[5];
    always @(posedge clk) bdiv <= fall ? 5'd0 : bdiv + 5'd1;
    always @(posedge clk)
        if (bdiv == 5'd9) bclk <= 1'b1;
        else if (fall)    bclk <= 1'b0;
    always @(posedge clk) if (fall) bcnt <= bnext;
    always @(posedge clk) if (fall && bnext == 6'd0) smp <= interp;
    always @(posedge clk)
        if (fall)
            din <= (bnext[4:0] >= 5'd1 && bnext[4:0] <= 5'd16) ? smp[5'd16 - bnext[4:0]] : 1'b0;
endmodule
