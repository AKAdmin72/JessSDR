// fft512: two complex tones + noise -> bins dumped to sim/fft_out.txt,
// inputs to sim/fft_in.txt; sim/check_fft.py compares against numpy.
`timescale 1ns/1ps
module tb_fft512;
    reg clk = 0;
    always #5 clk = ~clk;
    reg start = 0, iv = 0;
    reg signed [17:0] xi = 0, xq = 0;
    wire busy, done;
    reg  [8:0] ra = 0;
    wire signed [27:0] re, im;
    fft512 dut (.clk(clk), .start(start), .in_valid(iv), .in_i(xi), .in_q(xq),
                .busy(busy), .done(done), .rd_addr(ra), .rd_re(re), .rd_im(im));
    integer n, fi, fo, seed = 7, cyc = 0;
    real ph1 = 0.0, ph2 = 0.0;
    always @(posedge clk) cyc = cyc + 1;
    initial begin
        fi = $fopen("sim/fft_in.txt", "w");
        fo = $fopen("sim/fft_out.txt", "w");
        @(posedge clk); #1 start = 1; @(posedge clk); #1 start = 0;
        for (n = 0; n < 512; n = n + 1) begin
            // tone A: bin +37.3, amp 100000; tone B: bin -100, amp 300; noise +-8
            xi = $rtoi(100000.0 * $cos(ph1) + 300.0 * $cos(ph2) + 1e6) - 1000000 + ($random(seed) % 9);
            xq = $rtoi(100000.0 * $sin(ph1) + 300.0 * $sin(ph2) + 1e6) - 1000000 + ($random(seed) % 9);
            ph1 = ph1 + 2.0 * 3.14159265358979 * 37.3 / 512.0;
            ph2 = ph2 - 2.0 * 3.14159265358979 * 100.0 / 512.0;
            $fwrite(fi, "%0d %0d\n", xi, xq);
            iv = 1; @(posedge clk); #1 iv = 0;
            repeat (20) @(posedge clk); #1;
        end
        n = cyc;
        wait (done); @(posedge clk); #1;
        $display("fft cycles after last sample: %0d", cyc - n);
        for (n = 0; n < 512; n = n + 1) begin
            ra = n; @(posedge clk); #1;
            $fwrite(fo, "%0d %0d\n", re, im);
        end
        $fclose(fi); $fclose(fo);
        $finish;
    end
endmodule
