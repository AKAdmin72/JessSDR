// Self-checking testbench for adc_clip.v: window 2^6 = 64 samples.
// Window 1: no clip -> 0. Window 2: 1 sample at 4095 -> 1. Window 3: 3
// samples (0, 4095, OTR) -> 2. Window 4: all 64 at 0 -> 7 (2^6 <= 64 < 2^7).
// Window 5: 5 clipped, one of them on the window's last clock -> 3.
`timescale 1ns/1ps
module tb_adc_clip;
    localparam W = 6, N = 1 << W;
    reg clk = 0;
    always #5 clk = ~clk;
    reg [11:0] d = 12'd2048;
    reg otr = 0;
    reg [W-1:0] pos = {W{1'b1}};   // 0 on the first sampled edge: sample k = window position k
    wire win_last = &pos;
    always @(posedge clk) pos <= pos + 1'b1;
    wire [7:0] clip_log;
    adc_clip #(.W(W)) dut (.clk(clk), .d(d), .otr(otr), .win_last(win_last), .clip_log(clip_log));

    integer errors = 0, w, k, nwin = -1;   // -1: the empty window ending on the first edge
    // checker: clip_log is valid 2 clocks after the window's last clock
    reg c1 = 0, c2 = 0;
    always @(posedge clk) begin
        c1 <= win_last; c2 <= c1;
        if (c2 && nwin < 0) nwin = 0;
        else if (c2 && nwin < 5) begin
            if (clip_log !== want[nwin]) begin
                $display("FAIL: window %0d clip_log %0d, expected %0d", nwin, clip_log, want[nwin]);
                errors = errors + 1;
            end else $display("window %0d clip_log %0d", nwin, clip_log);
            nwin = nwin + 1;
        end
    end
    reg [7:0] want [0:4];
    initial begin
        want[0] = 0; want[1] = 1; want[2] = 2; want[3] = 7; want[4] = 3;
        for (w = 0; w < 5; w = w + 1) begin
            for (k = 0; k < N; k = k + 1) begin
                @(negedge clk);
                d = 12'd2048 + (k % 7) * 100; otr = 0;
                case (w)
                1: if (k == 10) d = 12'hFFF;
                2: begin if (k == 3) d = 12'd0; if (k == 20) d = 12'hFFF; if (k == 40) otr = 1; end
                3: d = 12'd0;
                4: if (k == 1 || k == 2 || k == 30 || k == 50 || k == N - 1) d = 12'hFFF;
                endcase
            end
        end
        repeat (4) @(posedge clk);   // the last window's result
        if (nwin != 5) begin $display("FAIL: %0d windows checked", nwin); errors = errors + 1; end
        if (errors == 0) $display("PASS: adc_clip");
        $finish;
    end
endmodule
