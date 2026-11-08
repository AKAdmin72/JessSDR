// floor_eq: lines of noise (floor 103 +-6 units, CIC-like edge droop) with a
// 10-bin "station" +60; at line 100 the floor jumps +53 (LNA on, +20 dB).
// Checks the flattened noise mean stays near BASE+~9 and the station height
// survives, and that the floor recovers after the jump.
`timescale 1ns/1ps
module tb_floor_eq;
    reg clk = 0;
    always #5 clk = ~clk;
    reg we = 0, done = 0; reg [8:0] x = 0, l = 0;
    wire owe, odone; wire [8:0] ox, ol;
    floor_eq dut (.clk(clk), .en(1'b1), .in_we(we), .in_x(x), .in_l8(l), .line_done(done),
                  .out_we(owe), .out_x(ox), .out_l8(ol), .done_out(odone));
    integer t, i, seed = 5, fl, nsum, ssum, fails = 0, dx;
    always @(posedge clk) if (owe) begin
        if (ox >= 150 && ox < 160) ssum = ssum + ol;
        else nsum = nsum + ol;
    end
    initial begin
        for (t = 0; t < 200; t = t + 1) begin
            nsum = 0; ssum = 0;
            fl = (t < 100) ? 103 : 156;
            for (i = 0; i < 320; i = i + 1) begin
                dx = (i < 160) ? 160 - i : i - 159;
                l = fl + ($random(seed) % 7) - (dx * dx) / 1700       // ~15 units droop at the edges
                    + ((i >= 150 && i < 160) ? 60 : 0);
                x = i; we = 1; @(posedge clk); #1 we = 0;
                repeat (8) @(posedge clk); #1;
            end
            done = 1; @(posedge clk); #1 done = 0;
            repeat (1400) @(posedge clk); #1;
            if (t == 99 || t == 100 || t == 105 || t == 110 || t == 130 || t == 199)
                $display("line %0d: noise mean %0d, station mean %0d (height %0d)", t, nsum / 310, ssum / 10,
                         ssum / 10 - nsum / 310);
            if ((t == 99 || t == 199) && (nsum / 310 < 94 || nsum / 310 > 112 || ssum / 10 - nsum / 310 < 50))
                fails = fails + 1;
        end
        if (fails) $display("FAIL"); else $display("ok");
        $finish;
    end
endmodule
