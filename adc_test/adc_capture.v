// AD9226 parallel-bus capture, bring-up version (no PLL).
//
// ADC clock: adc_clk toggles every system clock -> 27MHz/2 = 13.5MSPS.
// The AD9226 samples on the RISING edge of its CLK and its output bus
// changes tOD (a few ns) after a rising edge (the value belongs to a sample
// taken 7 clocks earlier - pipeline latency, irrelevant for a scope).
// We latch the bus on the system-clock edge where adc_clk falls, i.e. half
// an ADC period (37ns) after the last rising edge - far from the bus
// transition in both directions, so no IO-delay tuning is needed at this
// rate. (65MSPS later needs a PLL and a phase-adjusted capture clock.)
//
// Output format: AD9226 is straight (offset) binary, 0 = -FS, 4095 = +FS.
// Converted to two's complement and left-justified into 18 bits
// (sample = code_signed << 6), i.e. the same format the oscilloscope
// project's i2s_rx delivered, so trigger.v / waveform_draw.v work unchanged.
//
// BIT_REVERSE=1 swaps the bus (adc_d[0] <-> adc_d[11]) in case the module's
// silkscreen D0 turns out to be the converter's MSB (AD9226 datasheet names
// its pins BIT1=MSB .. BIT12=LSB, boards differ in how they relabel them).
//
// Decimation: one `valid` pulse every 2^decim_sel ADC samples (plain
// sample-dropping, no filtering - it's a timebase for the scope view).
module adc_capture #(
    parameter BIT_REVERSE = 0
) (
    input  wire        clk,
    output reg          adc_clk = 1'b0,
    input  wire [11:0] adc_d,
    input  wire        adc_otr,
    input  wire [3:0]  decim_sel,          // 0..15 -> decimation 1..32768

    output reg  signed [17:0] sample = 18'sd0,
    output reg          valid = 1'b0,
    output reg          otr = 1'b0,         // OTR flag of the sample presented with `valid`
    output reg  [11:0]  raw = 12'd0         // last latched bus value, unconverted (debug)
);

    always @(posedge clk)
        adc_clk <= ~adc_clk;

    // Input registers (one per reg, latched once per ADC period).
    reg [11:0] d_q = 12'd0;
    reg        otr_q = 1'b0;
    reg        cap_strobe = 1'b0;   // 1 cycle after the latch - d_q is valid
    always @(posedge clk) begin
        if (adc_clk) begin          // adc_clk is about to fall - mid-period
            d_q   <= adc_d;
            otr_q <= adc_otr;
        end
    end
    always @(posedge clk)
        cap_strobe <= adc_clk;

    function [11:0] rev12(input [11:0] v);
        integer i;
        begin
            for (i = 0; i < 12; i = i + 1)
                rev12[i] = v[11 - i];
        end
    endfunction

    wire [11:0] code  = BIT_REVERSE ? rev12(d_q) : d_q;
    wire [11:0] code_s = {~code[11], code[10:0]};   // offset binary -> two's complement

    reg [15:0] dec_cnt = 16'd0;
    wire [15:0] dec_max = (16'd1 << decim_sel) - 16'd1;
    wire        dec_hit = (dec_cnt >= dec_max);

    always @(posedge clk) begin
        if (cap_strobe)
            dec_cnt <= dec_hit ? 16'd0 : dec_cnt + 16'd1;
    end

    always @(posedge clk) begin
        valid <= cap_strobe && dec_hit;
        if (cap_strobe && dec_hit) begin
            sample <= {code_s, 6'b0};
            otr    <= otr_q;
        end
    end

    always @(posedge clk)
        if (cap_strobe)
            raw <= code;

endmodule
