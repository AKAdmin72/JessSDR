// Digital down-converter for the AD9226 at 64.8 MSPS.
//
//   NCO: 32-bit phase accumulator, 4096 x 16-bit sine table (nco_sin.hex,
//        half-sample offset, two BSRAM ROMs for sin/cos), ~-72 dBc spurs
//   mixer: I = x*cos, Q = -x*sin  (x = ADC code as signed 12-bit), so a
//        tone at f_lo + df comes out at +df (positive = above the dial)
//   CIC: 4th order, decimate by R = 1229 -> 52.726 kHz (FpgaRX-SDR's
//        ssb_demod/audio chain is built for 52.734 kHz - 0.015 % off).
//        Gain R^4 = 2^41.05; registers 28 + 42 = 70 bits. The four combs
//        run as a 4-cycle pipeline at the output rate.
//   output: 18-bit signed, >> 47: a full-scale ADC sine would give ~2^19,
//        i.e. saturation above ~-12 dBFS (+12 dBm at the SMA, far above
//        any 40m signal); the stock module's band noise is ~7 LSB rms.
// out_valid pulses once every R clocks.
// Wide output (spectrum overview): the same integrators sampled every
// R2 = 614 clocks -> 105.54 kHz, own comb pipeline, >> SHIFT2 = 43
// (614^4 = 2^37.06, same scale as the narrow output to within 0.1 dB).
// CIC4 droop at +-33 kHz -5.6 dB (floor_eq flattens it), aliases >= -33 dB.
module ddc #(
    parameter integer R  = 1229,
    parameter integer R2 = 614
) (
    input  wire               clk,
    input  wire [11:0]        adc_code,      // offset binary, registered by the caller
    input  wire [31:0]        phase_inc,
    output reg                out_valid = 1'b0,
    output reg  signed [17:0] out_i = 18'sd0,
    output reg  signed [17:0] out_q = 18'sd0,
    output reg                w_valid = 1'b0,
    output reg  signed [17:0] w_i = 18'sd0,
    output reg  signed [17:0] w_q = 18'sd0
);
    localparam integer W = 70;
    localparam integer SHIFT = 47;

    // ---- NCO ----
    reg [31:0] ph = 32'd0;
    reg [31:0] inc_q = 32'd0;
    always @(posedge clk) inc_q <= phase_inc;
    always @(posedge clk) ph <= ph + inc_q;

    reg [15:0] rom_s [0:4095];
    reg [15:0] rom_c [0:4095];
    initial $readmemh("nco_sin.hex", rom_s);
    initial $readmemh("nco_sin.hex", rom_c);
    wire [11:0] a_s = ph[31:20];
    wire [11:0] a_c = ph[31:20] + 12'd1024;       // cos = sin(+90 deg)
    reg  signed [15:0] s = 16'sd0, c = 16'sd0;
    always @(posedge clk) s <= rom_s[a_s];
    always @(posedge clk) c <= rom_c[a_c];

    // ADC sample delayed to line up with the ROM output (1 cycle)
    reg signed [11:0] x0 = 12'sd0, x1 = 12'sd0;
    always @(posedge clk) x0 <= {~adc_code[11], adc_code[10:0]};
    always @(posedge clk) x1 <= x0;

    // ---- mixer ----
    reg signed [27:0] mi = 28'sd0, mq = 28'sd0;
    always @(posedge clk) mi <= x1 * c;
    always @(posedge clk) mq <= -(x1 * s);

    // ---- CIC integrators (full rate) ----
    reg signed [W-1:0] ii1 = 0, ii2 = 0, ii3 = 0, ii4 = 0;
    reg signed [W-1:0] qi1 = 0, qi2 = 0, qi3 = 0, qi4 = 0;
    always @(posedge clk) ii1 <= ii1 + {{(W-28){mi[27]}}, mi};
    always @(posedge clk) ii2 <= ii2 + ii1;
    always @(posedge clk) ii3 <= ii3 + ii2;
    always @(posedge clk) ii4 <= ii4 + ii3;
    always @(posedge clk) qi1 <= qi1 + {{(W-28){mq[27]}}, mq};
    always @(posedge clk) qi2 <= qi2 + qi1;
    always @(posedge clk) qi3 <= qi3 + qi2;
    always @(posedge clk) qi4 <= qi4 + qi3;

    // ---- decimation + comb pipeline ----
    reg [10:0] dcnt = 11'd0;
    wire       dhit = (dcnt == R - 1);
    always @(posedge clk) dcnt <= dhit ? 11'd0 : dcnt + 11'd1;
    reg [4:0] stg = 5'd0;                          // one-hot-ish pipeline valid
    always @(posedge clk) stg <= {stg[3:0], dhit};

    reg signed [W-1:0] ix = 0, qx = 0;             // sampled integrator output
    reg signed [W-1:0] ic1 = 0, ic2 = 0, ic3 = 0, ic4 = 0;
    reg signed [W-1:0] qc1 = 0, qc2 = 0, qc3 = 0, qc4 = 0;
    reg signed [W-1:0] id0 = 0, id1 = 0, id2 = 0, id3 = 0;   // comb delays (M = 1)
    reg signed [W-1:0] qd0 = 0, qd1 = 0, qd2 = 0, qd3 = 0;
    always @(posedge clk) if (dhit)   ix  <= ii4;
    always @(posedge clk) if (dhit)   qx  <= qi4;
    always @(posedge clk) if (stg[0]) ic1 <= ix - id0;
    always @(posedge clk) if (stg[0]) id0 <= ix;
    always @(posedge clk) if (stg[0]) qc1 <= qx - qd0;
    always @(posedge clk) if (stg[0]) qd0 <= qx;
    always @(posedge clk) if (stg[1]) ic2 <= ic1 - id1;
    always @(posedge clk) if (stg[1]) id1 <= ic1;
    always @(posedge clk) if (stg[1]) qc2 <= qc1 - qd1;
    always @(posedge clk) if (stg[1]) qd1 <= qc1;
    always @(posedge clk) if (stg[2]) ic3 <= ic2 - id2;
    always @(posedge clk) if (stg[2]) id2 <= ic2;
    always @(posedge clk) if (stg[2]) qc3 <= qc2 - qd2;
    always @(posedge clk) if (stg[2]) qd2 <= qc2;
    always @(posedge clk) if (stg[3]) ic4 <= ic3 - id3;
    always @(posedge clk) if (stg[3]) id3 <= ic3;
    always @(posedge clk) if (stg[3]) qc4 <= qc3 - qd3;
    always @(posedge clk) if (stg[3]) qd3 <= qc3;

    function signed [17:0] sat18(input signed [W-1:0] v);
        reg signed [W-1:0] sh;
        begin
            sh = v >>> SHIFT;
            if (sh > 131071)       sat18 = 18'sd131071;
            else if (sh < -131072) sat18 = -18'sd131072;
            else                   sat18 = sh[17:0];
        end
    endfunction
    always @(posedge clk) if (stg[4]) out_i <= sat18(ic4);
    always @(posedge clk) if (stg[4]) out_q <= sat18(qc4);
    always @(posedge clk) out_valid <= stg[4];

    // ---- wide output: decimate by R2, 66-bit combs (28 + 4*log2(614) = 65.1) ----
    localparam integer W2 = 66;
    localparam integer SHIFT2 = 43;
    reg [9:0] wcnt = 10'd0;
    wire      whit = (wcnt == R2 - 1);
    always @(posedge clk) wcnt <= whit ? 10'd0 : wcnt + 10'd1;
    reg [4:0] wstg = 5'd0;
    always @(posedge clk) wstg <= {wstg[3:0], whit};

    reg signed [W2-1:0] wix = 0, wqx = 0;
    reg signed [W2-1:0] wic1 = 0, wic2 = 0, wic3 = 0, wic4 = 0;
    reg signed [W2-1:0] wqc1 = 0, wqc2 = 0, wqc3 = 0, wqc4 = 0;
    reg signed [W2-1:0] wid0 = 0, wid1 = 0, wid2 = 0, wid3 = 0;
    reg signed [W2-1:0] wqd0 = 0, wqd1 = 0, wqd2 = 0, wqd3 = 0;
    always @(posedge clk) if (whit)    wix  <= ii4[W2-1:0];
    always @(posedge clk) if (whit)    wqx  <= qi4[W2-1:0];
    always @(posedge clk) if (wstg[0]) wic1 <= wix - wid0;
    always @(posedge clk) if (wstg[0]) wid0 <= wix;
    always @(posedge clk) if (wstg[0]) wqc1 <= wqx - wqd0;
    always @(posedge clk) if (wstg[0]) wqd0 <= wqx;
    always @(posedge clk) if (wstg[1]) wic2 <= wic1 - wid1;
    always @(posedge clk) if (wstg[1]) wid1 <= wic1;
    always @(posedge clk) if (wstg[1]) wqc2 <= wqc1 - wqd1;
    always @(posedge clk) if (wstg[1]) wqd1 <= wqc1;
    always @(posedge clk) if (wstg[2]) wic3 <= wic2 - wid2;
    always @(posedge clk) if (wstg[2]) wid2 <= wic2;
    always @(posedge clk) if (wstg[2]) wqc3 <= wqc2 - wqd2;
    always @(posedge clk) if (wstg[2]) wqd2 <= wqc2;
    always @(posedge clk) if (wstg[3]) wic4 <= wic3 - wid3;
    always @(posedge clk) if (wstg[3]) wid3 <= wic3;
    always @(posedge clk) if (wstg[3]) wqc4 <= wqc3 - wqd3;
    always @(posedge clk) if (wstg[3]) wqd3 <= wqc3;

    function signed [17:0] sat18w(input signed [W2-1:0] v);
        reg signed [W2-1:0] sh;
        begin
            sh = v >>> SHIFT2;
            if (sh > 131071)       sat18w = 18'sd131071;
            else if (sh < -131072) sat18w = -18'sd131072;
            else                   sat18w = sh[17:0];
        end
    endfunction
    always @(posedge clk) if (wstg[4]) w_i <= sat18w(wic4);
    always @(posedge clk) if (wstg[4]) w_q <= sat18w(wqc4);
    always @(posedge clk) w_valid <= wstg[4];
endmodule
