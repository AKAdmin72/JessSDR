`timescale 1ns/1ns
// tb for audio_up4.v: 3 kHz tone (amp 16000) in at 13.18 kHz (every 4916
// clk), writes the 52.7 kHz output to sim/up4_out.txt; checks that outputs
// come exactly 1229 clk apart and 4 per input. sim/check_up4.py checks the
// spectrum (gain, image at 10.18 kHz).
module tb_audio_up4;
    reg clk = 0;
    always #1 clk = ~clk;
    reg vin = 0;
    reg signed [15:0] xin = 0;
    wire vout;
    wire signed [15:0] yout;
    audio_up4 #(.PERIOD(1229)) dut (.clk(clk), .in_valid(vin), .in_audio(xin), .out_valid(vout), .out_audio(yout));

    real PI = 3.14159265358979, FS = 64.8e6 / 4916.0;
    integer n, f, errors = 0, nout = 0, last_t = -1, cyc;
    always @(posedge clk) if (vout) begin
        cyc = $time / 2;                      // clk period 2 ns
        if (last_t >= 0 && cyc - last_t != 1229) begin
            errors = errors + 1;
            if (errors < 5) $display("spacing %0d at output %0d", cyc - last_t, nout);
        end
        last_t = cyc;
        nout = nout + 1;
        $fwrite(f, "%0d\n", yout);
    end
    initial begin
        f = $fopen("sim/up4_out.txt", "w");
        for (n = 0; n < 1200; n = n + 1) begin
            repeat (4915) @(posedge clk);
            #1 xin = $rtoi(16000.0 * $sin(2.0 * PI * 3000.0 * n / FS)); vin = 1;
            @(posedge clk); #1 vin = 0;
        end
        repeat (5000) @(posedge clk);
        $fclose(f);
        if (nout != 4 * 1200) begin errors = errors + 1; $display("outputs %0d, want %0d", nout, 4 * 1200); end
        $display("%s: %0d outputs, %0d errors", errors ? "FAIL" : "PASS", nout, errors);
        $finish;
    end
endmodule
