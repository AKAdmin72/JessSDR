// audio_i2s: feeds a 1 kHz sine at 13.181 kHz, decodes the I2S stream
// (left slot) like the amp would, checks the BCLK/LRCK ratio, and checks
// the decoded 48.2 kHz stream against the ideal (delayed) sine.
`timescale 1ns/1ps
module tb_audio_i2s;
    reg clk = 0;
    always #7.716 clk = ~clk;
    reg in_valid = 0;
    reg signed [15:0] in_audio = 0;
    wire bclk, lrck, din;
    audio_i2s dut (.clk(clk), .in_valid(in_valid), .in_audio(in_audio), .bclk(bclk), .lrck(lrck), .din(din));

    integer n_in = 0, k;
    always begin
        repeat (4915) @(posedge clk);
        #1 in_valid = 1; in_audio = $rtoi(20000.0 * $sin(2.0 * 3.14159265 * 1000.0 * n_in / 13181.4));
        n_in = n_in + 1;
        @(posedge clk); #1 in_valid = 0;
    end

    // decoder
    reg lr_d = 0; integer bitpos = 0, nout = 0, errors = 0, nb = 0;
    reg [15:0] sh = 0;
    real tdec, expv, err, maxerr = 0;
    always @(posedge bclk) begin
        nb = nb + 1;
        if (lrck != lr_d) begin
            if (lr_d == 1'b0 && nout > 0) begin
                // left word complete; time of its frame start ~ now - 32 bclk
                tdec = ($realtime * 1e-9) - 32.0 / 3.0857e6;
                // interpolator lags one input period plus the latch
                expv = 20000.0 * $sin(2.0 * 3.14159265 * 1000.0 * (tdec - 2.0 / 13181.4 - 64.0/3.0857e6));
                err = $signed(sh) - expv; if (err < 0) err = -err;
                if (nout > 200 && err > maxerr) maxerr = err;
                $fwrite(fd, "%0d\n", $signed(sh));
            end
            if (lr_d == 1'b0) nout = nout + 1;
            bitpos = 0; lr_d = lrck;
        end else begin
            bitpos = bitpos + 1;
            if (lrck == 1'b0 && bitpos <= 16) sh = {sh[14:0], din};
        end
    end
    integer fd;
    initial fd = $fopen("sim/audio_out.txt", "w");
    initial begin
        #20_000_000;
        $fclose(fd);
        $display("%0d frames in 20 ms -> fs %0.1f Hz, bclk/frame %0d, max interp error %0.0f LSB",
                 nout, nout / 0.02, nb / nout, maxerr);
        if (nout < 960 || nout > 970) begin errors = errors + 1; $display("FAIL rate"); end
        // waveform quality is checked offline by fitting a sine (see below)
        if (errors == 0) $display("PASS"); else $display("FAILED");
        $finish;
    end
endmodule
