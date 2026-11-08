// bin2bcd on a few values; level_meter monotonic/log check.
`timescale 1ns/1ps
module tb_misc;
    reg clk = 0; always #5 clk = ~clk;
    reg [24:0] bin = 0; wire [31:0] bcd;
    bin2bcd u (.clk(clk), .bin(bin), .bcd(bcd));
    reg v = 0; reg signed [17:0] i = 0, q = 0; wire [7:0] lvl;
    level_meter lm (.clk(clk), .in_valid(v), .in_i(i), .in_q(q), .level(lvl));
    integer errors = 0, k;
    task chk(input [24:0] b, input [31:0] e);
        begin bin = b; repeat (40) @(posedge clk); #1;
            if (bcd !== e) begin errors = errors + 1; $display("FAIL %0d -> %h", b, bcd); end end
    endtask
    task lev(input integer a, input integer e);
        begin i = a; q = 0; @(negedge clk) v = 1; @(negedge clk) v = 0; repeat (6) @(posedge clk); #1;
            if (lvl != e) begin errors = errors + 1; $display("FAIL level(%0d) = %0d exp %0d", a, lvl, e); end end
    endtask
    initial begin
        chk(25'd7100000, 32'h07100000);
        chk(25'd7074000, 32'h07074000);
        chk(25'd33554431, 32'h33554431);
        chk(25'd0, 32'h0);
        chk(25'd123456, 32'h00123456);
        lev(1, 0);  lev(8, 24); lev(12, 28); lev(-1000, 79); lev(100000, 132);
        if (errors == 0) $display("PASS"); else $display("FAILED %0d", errors);
        $finish;
    end
endmodule
