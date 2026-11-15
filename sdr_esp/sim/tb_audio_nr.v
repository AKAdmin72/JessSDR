`timescale 1ns/1ns
// Self-checking testbench for audio_nr.v. Inputs every 1024 cycles (the real
// spacing is 2048; one sample takes ~780).
//   (1) mode 0: out = in;
//   (2) ANF, level 1: 1kHz tone (amp 8000) + white noise (rms ~1000): after
//       convergence the tone must be >= 20dB down in the output, the noise
//       roughly kept (within -6..+3dB);
//   (3) NR, level 0 (no residual mixed back): 700Hz tone (amp 4000) + white
//       noise (rms ~4000): the tone-to-noise ratio must improve by >= 6dB;
//   (3b) NR, level 3 (e/2 mixed back): improves by 1..6dB, the noise is
//       kept within -7..-2dB (milder than level 0);
//   (4) out_valid once per input.
module tb_audio_nr;
    reg clk = 0;
    always #18.5 clk = ~clk;

    reg  [1:0]         mode = 0, level = 1;
    reg                vin = 0;
    reg  signed [15:0] xin = 0;
    wire               vout;
    wire signed [15:0] yout;

    audio_nr dut (.clk(clk), .mode(mode), .level(level), .in_valid(vin), .in_audio(xin),
                  .out_valid(vout), .out_audio(yout));

    real PI = 3.14159265358979, FS = 13183.59375;
    integer errors = 0, seed = 11, nvout = 0, nin = 0;
    always @(posedge clk) if (vout) nvout = nvout + 1;

    real xs [0:3999], ys [0:3999];
    integer nc;

    // approx. Gaussian noise, rms ~ r
    function real gauss(input real r);
        integer j;
        real a;
        begin
            a = 0;
            for (j = 0; j < 12; j = j + 1) a = a + ($random(seed) & 16'hFFFF) / 65536.0;
            gauss = (a - 6.0) * r;
        end
    endfunction

    // feed n samples of tone(f, amp) + noise(r); keep the last 4000 in/out
    task feed(input real f, input real amp, input real r, input integer n);
        integer m;
        real v;
        begin
            for (m = 0; m < n; m = m + 1) begin
                v = amp * $sin(2.0*PI*f*nin/FS) + gauss(r);
                if (v > 32767.0) v = 32767.0;
                if (v < -32768.0) v = -32768.0;
                @(posedge clk); #1 xin = $rtoi(v); vin = 1;
                @(posedge clk); #1 vin = 0;
                @(posedge vout); #1;
                if (m >= n - 4000) begin xs[m - (n - 4000)] = xin; ys[m - (n - 4000)] = yout; end
                if (mode == 0 && yout !== xin) begin
                    if (errors < 5) $display("FAIL: bypass out %0d in %0d", yout, xin);
                    errors = errors + 1;
                end
                nin = nin + 1;
                repeat (1000) @(posedge clk);
            end
        end
    endtask

    // tone amplitude at f and residual rms of buffer b (0 = in, 1 = out)
    real amp, res;
    task measure(input real f, input integer b);
        integer m;
        real si, sc, v, fit;
        begin
            si = 0; sc = 0;
            for (m = 0; m < 4000; m = m + 1) begin
                v = b ? ys[m] : xs[m];
                si = si + v * $sin(2.0*PI*f*m/FS);
                sc = sc + v * $cos(2.0*PI*f*m/FS);
            end
            si = 2.0*si/4000; sc = 2.0*sc/4000;
            amp = $sqrt(si*si + sc*sc);
            res = 0;
            for (m = 0; m < 4000; m = m + 1) begin
                v = b ? ys[m] : xs[m];
                fit = si * $sin(2.0*PI*f*m/FS) + sc * $cos(2.0*PI*f*m/FS);
                res = res + (v - fit) * (v - fit);
            end
            res = $sqrt(res / 4000);
        end
    endtask

    real a_in, r_in, a_out, r_out, d_in, d_out;
    initial begin
        $dumpfile("sim/tb_audio_nr.vcd");
        $dumpvars(0, tb_audio_nr);
        repeat (3) @(posedge clk);

        // (1) bypass
        mode = 0;
        feed(1000.0, 8000.0, 1000.0, 500);

        // (2) ANF
        mode = 2; level = 1;
        feed(1000.0, 8000.0, 1000.0, 12000);
        measure(1000.0, 0); a_in = amp; r_in = res;
        measure(1000.0, 1); a_out = amp; r_out = res;
        $display("ANF: tone %0.0f -> %0.0f (%0.1f dB), noise %0.0f -> %0.0f (%0.1f dB)",
                 a_in, a_out, 20.0*$log10(a_out/a_in), r_in, r_out, 20.0*$log10(r_out/r_in));
        if (a_out > a_in * 0.1) begin $display("FAIL: ANF tone not >= 20dB down"); errors = errors + 1; end
        if (r_out < r_in * 0.5 || r_out > r_in * 1.41) begin $display("FAIL: ANF noise changed too much"); errors = errors + 1; end

        // (3) NR
        mode = 1; level = 0;
        feed(700.0, 4000.0, 4000.0, 12000);
        measure(700.0, 0); a_in = amp; r_in = res;
        measure(700.0, 1); a_out = amp; r_out = res;
        d_in = 20.0*$log10(a_in/r_in); d_out = 20.0*$log10(a_out/r_out);
        $display("NR: tone/noise %0.1f dB -> %0.1f dB (tone %0.0f -> %0.0f, noise %0.0f -> %0.0f)",
                 d_in, d_out, a_in, a_out, r_in, r_out);
        if (d_out < d_in + 6.0) begin $display("FAIL: NR improvement < 6dB"); errors = errors + 1; end

        // (3b) NR, mildest: e/2 mixed back (filter already converged)
        level = 3;
        feed(700.0, 4000.0, 4000.0, 4000);
        measure(700.0, 0); a_in = amp; r_in = res;
        measure(700.0, 1); a_out = amp; r_out = res;
        d_in = 20.0*$log10(a_in/r_in); d_out = 20.0*$log10(a_out/r_out);
        $display("NR e/2: tone/noise %0.1f dB -> %0.1f dB, noise %0.1f dB",
                 d_in, d_out, 20.0*$log10(r_out/r_in));
        if (d_out < d_in + 1.0 || d_out > d_in + 6.0) begin $display("FAIL: NR e/2 improvement not 1..6dB"); errors = errors + 1; end
        if (r_out < r_in * 0.447 || r_out > r_in * 0.794) begin $display("FAIL: NR e/2 noise not -7..-2dB"); errors = errors + 1; end

        if (nvout != nin) begin $display("FAIL: %0d outputs for %0d inputs", nvout, nin); errors = errors + 1; end
        if (errors == 0) $display("PASS: audio_nr - bypass, ANF notches a tone, NR improves tone/noise");
        else             $display("FAIL: audio_nr (%0d errors)", errors);
        $finish;
    end
endmodule
