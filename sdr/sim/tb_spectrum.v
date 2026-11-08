// spectrum: complex tone at +20 bins (amp 8000) + noise (~7 rms) at the DDC
// rate (here every 40 clocks) -> one averaged line; checks the tone column
// (x = 180), the noise floor, and that wf_top advances.
`timescale 1ns/1ps
module tb_spectrum;
    reg clk = 0;
    always #5 clk = ~clk;
    reg iv = 0;
    reg signed [17:0] xi = 0, xq = 0;
    wire lwe, ldone; wire [8:0] lx, ll8; wire [6:0] wrow, wtop;
    spectrum dut (.clk(clk), .in_valid(iv), .in_i(xi), .in_q(xq), .navg_half(1'b0), .line_we(lwe), .line_x(lx),
                  .line_l8(ll8), .wf_row(wrow), .wf_top(wtop), .line_done(ldone));
    integer seed = 3, c = 0;
    real ph = 0.0;
    always @(posedge clk) begin
        c = c + 1;
        if (c % 40 == 0) begin
            xi <= $rtoi(8000.0 * $cos(ph) + 1e6) - 1000000 + ($random(seed) % 12);
            xq <= $rtoi(8000.0 * $sin(ph) + 1e6) - 1000000 + ($random(seed) % 12);
            ph = ph + 2.0 * 3.14159265358979 * 20.0 / 512.0;
            iv <= 1;
        end else iv <= 0;
    end
    reg [8:0] line [0:319];
    integer i, mx, mxx, nlo;
    initial begin
        @(posedge ldone); #1;
        mx = 0; mxx = 0; nlo = 0;
        for (i = 0; i < 320; i = i + 1) begin
            if (line[i] > mx) begin mx = line[i]; mxx = i; end
            if (i < 140 || i > 220) nlo = nlo + line[i];
        end
        $display("peak L8 %0d at x %0d, mean floor L8 %0d, wf_row %0d wf_top %0d", mx, mxx, nlo / 220, wrow, wtop);
        if (mxx != 180 || wtop != 1 || mx < 300 || nlo / 220 > 130) $display("FAIL"); else $display("ok");
        $finish;
    end
    always @(posedge clk) if (lwe) line[lx] <= ll8;
endmodule
