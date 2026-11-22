// ADC clip counter (JessSDR sdr_esp, VERSION 8, 2026-11-22).
// Counts the samples that hit a rail (code 0 or 4095) or have OTR set
// within one peak window (win_last = its last clock, the same window as
// PEAK) and reports the count log2-coded:
//   clip_log = 0            no clipped sample
//   clip_log = n (1..W+1)   2^(n-1) <= count < 2^n
// PEAK only says "some sample came near full scale"; this tells a rare
// single peak (1..15 samples per 0.26 s) from steady clipping.
module adc_clip #(
    parameter integer W = 24                  // window 2^W samples
) (
    input  wire        clk,
    input  wire [11:0] d,                     // registered ADC code
    input  wire        otr,                   // registered OTR
    input  wire        win_last,              // 1 on the window's last clock
    output reg  [7:0]  clip_log = 8'd0
);
    wire clip_s = otr | (d == 12'd0) | (d == 12'hFFF);

    reg  [W:0] cnt = {(W+1){1'b0}};           // up to 2^W samples
    reg  [W:0] lat = {(W+1){1'b0}};           // count of the last window
    wire [W:0] cnt_nx = cnt + {{W{1'b0}}, clip_s};
    always @(posedge clk) cnt <= win_last ? {(W+1){1'b0}} : cnt_nx;
    always @(posedge clk) if (win_last) lat <= cnt_nx;

    // bit length of lat; lat is stable for a whole window
    function [7:0] bitlen(input [W:0] v);
        integer i;
        begin
            bitlen = 8'd0;
            for (i = 0; i <= W; i = i + 1)
                if (v[i]) bitlen = i + 1;
        end
    endfunction
    always @(posedge clk) clip_log <= bitlen(lat);
endmodule
