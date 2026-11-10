`timescale 1ns/1ps
// buttons.v with DEB_BITS = 6 (tick every 64 clk, long = 62 ticks = 3968 clk):
// short S1 x3, long S1, short S2, long S2 (held long - one event only), bounce.
module tb_buttons;
    reg clk = 0; always #5 clk = ~clk;
    reg s1 = 0, s2 = 0;
    wire [7:0] btn;
    buttons #(.DEB_BITS(6)) dut (.clk(clk), .s1(s1), .s2(s2), .btn(btn));
    integer errors = 0, k;
    task press(input integer which, input integer clks);
        begin
            for (k = 0; k < 5; k = k + 1) begin                         // contact bounce
                if (which == 1) s1 = 1; else s2 = 1; #30;
                s1 = 0; s2 = 0; #30;
            end
            if (which == 1) s1 = 1; else s2 = 1;
            #(clks * 10); s1 = 0; s2 = 0; #(2000 * 10);
        end
    endtask
    task chk(input [7:0] want, input [8*12-1:0] what);
        if (btn !== want) begin $display("FAIL: %0s btn=%b expected %b", what, btn, want); errors = errors + 1; end
    endtask
    initial begin
        #(1000 * 10);
        chk(8'b00_00_00_00, "idle");
        press(1, 500); press(1, 500); press(1, 500);
        chk(8'b00_00_00_11, "3 x S1 short");
        press(1, 6000);
        chk(8'b00_00_01_11, "S1 long");
        press(2, 500);
        chk(8'b00_01_01_11, "S2 short");
        press(2, 30000);
        chk(8'b01_01_01_11, "S2 held long");
        press(1, 500);
        chk(8'b01_01_01_00, "S1 short wrap");
        if (errors == 0) $display("PASS: buttons - short/long counters, bounce, one long per hold, wrap");
        $finish;
    end
endmodule
