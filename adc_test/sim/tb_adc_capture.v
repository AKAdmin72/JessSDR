// Self-checking tb for adc_capture.v: a behavioural AD9226 drives a code
// ramp (bus changes 5ns after each adc_clk rising edge); checks the
// offset-binary -> two's complement conversion, the <<6 justification,
// BIT_REVERSE, OTR pass-through and the decimation rate.
`timescale 1ns/1ps
module tb_adc_capture;
    reg clk = 0;
    always #18.518 clk = ~clk;            // 27MHz

    wire adc_clk_a, adc_clk_b;
    reg  [11:0] bus = 12'd0;
    reg         otr_bus = 1'b0;
    reg  [3:0]  decim_sel = 4'd0;

    // behavioural ADC: new code 5ns after every rising edge of its clock
    reg [11:0] next_code = 12'd100;
    always @(posedge adc_clk_a) begin
        #5;
        bus       <= next_code;
        otr_bus   <= (next_code == 12'd4095);
        next_code <= next_code + 12'd1;
    end

    wire signed [17:0] s_a, s_b;
    wire v_a, v_b, o_a;
    adc_capture #(.BIT_REVERSE(0)) dut_a (.clk(clk), .adc_clk(adc_clk_a), .adc_d(bus), .adc_otr(otr_bus),
        .decim_sel(decim_sel), .sample(s_a), .valid(v_a), .otr(o_a), .raw());
    function [11:0] rev(input [11:0] v); integer i; begin for (i=0;i<12;i=i+1) rev[i]=v[11-i]; end endfunction
    adc_capture #(.BIT_REVERSE(1)) dut_b (.clk(clk), .adc_clk(adc_clk_b), .adc_d(rev(bus)), .adc_otr(otr_bus),
        .decim_sel(decim_sel), .sample(s_b), .valid(v_b), .otr(), .raw());

    integer n = 0, errors = 0;
    reg signed [12:0] prev = 0;
    reg have_prev = 0;
    reg signed [11:0] got;
    always @(posedge clk) if (v_a) begin
        got = s_a[17:6];
        if (s_a[5:0] != 0) begin errors = errors + 1; $display("FAIL: low bits not zero %h", s_a); end
        if (s_b !== s_a) begin errors = errors + 1; $display("FAIL: BIT_REVERSE mismatch %h vs %h", s_b, s_a); end
        if (have_prev && ((got - prev[11:0]) & 12'hFFF) != ((1 << decim_sel) & 12'hFFF)) begin
            errors = errors + 1; $display("FAIL: step %0d -> %0d (decim %0d)", prev, got, 1 << decim_sel);
        end
        if (o_a != (got == 12'sd2047)) begin errors = errors + 1; $display("FAIL: otr %b for %0d", o_a, got); end
        prev = got; have_prev = 1; n = n + 1;
    end

    initial begin
        // code 100 offset-binary == -1948 two's complement
        @(posedge v_a); #1;
        if ($signed(s_a[17:6]) > -1940 || $signed(s_a[17:6]) < -1950) begin
            $display("FAIL: first sample %0d not ~-1948", $signed(s_a[17:6])); errors = errors + 1;
        end
        repeat (5000) @(posedge clk);
        decim_sel = 4'd3; have_prev = 0;
        repeat (5000) @(posedge clk);
        if (n < 100) begin $display("FAIL: only %0d samples", n); errors = errors + 1; end
        if (errors == 0) $display("PASS: adc_capture (%0d samples checked)", n);
        else $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
