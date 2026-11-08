// ddc wide output (R2 = 614): tone at dial + 20 kHz -> w_i/w_q rotate at
// +20 kHz (fs 105.54 kHz) with the same amplitude as the narrow output for a
// tone inside both bands (+5 kHz). Dumps sim/ddc_wide.txt for numpy.
`timescale 1ns/1ps
module tb_ddc_wide;
    reg clk = 0;
    always #7.716 clk = ~clk;
    reg [11:0] code = 12'd2048;
    real t = 0.0, foff = 20000.0;
    wire [31:0] inc = 32'd470590552;              // 7.1 MHz * 2^32 / 64.8 MHz
    always @(posedge clk) begin
        code <= 2048 + $rtoi(200.0 * $cos(2.0 * 3.14159265358979 * (7.1e6 + foff) * t) + 4096.5) - 4096;
        t = t + 1.0 / 64.8e6;
    end
    wire v, wv;
    wire signed [17:0] oi, oq, wi, wq;
    ddc dut (.clk(clk), .adc_code(code), .phase_inc(inc), .out_valid(v), .out_i(oi), .out_q(oq),
             .w_valid(wv), .w_i(wi), .w_q(wq));
    integer f, nw = 0, nn = 0;
    initial begin
        f = $fopen("sim/ddc_wide.txt", "w");
        #(3000000);                              // 3 ms at +20 kHz
        foff = 5000.0;
        #(3000000);                              // 3 ms at +5 kHz
        $fclose(f);
        $display("wide samples %0d, narrow %0d", nw, nn);
        $finish;
    end
    always @(posedge clk) if (wv) begin $fdisplay(f, "w %0d %0d %0f", wi, wq, foff); nw = nw + 1; end
    always @(posedge clk) if (v)  begin $fdisplay(f, "n %0d %0d %0f", oi, oq, foff); nn = nn + 1; end
endmodule
