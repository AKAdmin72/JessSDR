// Receive chain as wired in sdr_top: ddc -> ssb_demod (LSB) -> audio_agc.
// Tone at dial - 1 kHz must give 1 kHz audio; tone at dial + 1 kHz (other
// sideband) must be rejected (audio before AGC compared).
`timescale 1ns/1ps
module tb_chain;
    reg clk = 0;
    always #7.716 clk = ~clk;
    localparam real FS = 64.8e6, FLO = 7.1e6;
    reg  [11:0] code = 12'd2048;
    wire [31:0] inc = 32'd470590552;
    real df = -1000.0, amp = 300.0, t = 0.0;
    always @(posedge clk) begin
        code <= 2048 + $rtoi(amp * $cos(2.0 * 3.14159265358979 * (FLO + df) * t) + 4096.5) - 4096;
        t = t + 1.0 / FS;
    end
    wire ddc_v; wire signed [17:0] di, dq;
    ddc u_ddc (.clk(clk), .adc_code(code), .phase_inc(inc), .out_valid(ddc_v), .out_i(di), .out_q(dq));
    wire aud_v, dec_v; wire signed [17:0] aud, deci, decq;
    ssb_demod u_ssb (.clk(clk), .in_valid(ddc_v), .in_i(di), .in_q(dq), .out_valid(aud_v), .out_audio(aud),
                     .dec_valid(dec_v), .dec_i(deci), .dec_q(decq));

    real a [0:399];
    integer n;
    task grab;
        begin
            repeat (150) @(posedge aud_v);              // settle the FIRs
            for (n = 0; n < 400; n = n + 1) begin @(posedge aud_v); #1; a[n] = aud; end
        end
    endtask
    // single-bin DFT magnitude at f (fs = 13181.4)
    function real tone(input real f);
        real c, s; integer k;
        begin c = 0; s = 0;
            for (k = 0; k < 400; k = k + 1) begin
                c = c + a[k] * $cos(2.0 * 3.14159265 * f * k / 13181.4);
                s = s + a[k] * $sin(2.0 * 3.14159265 * f * k / 13181.4);
            end
            tone = 2.0 * $sqrt(c * c + s * s) / 400.0;
        end
    endfunction
    real lsb1k, lsb_rms, usb1k;
    integer errors = 0;
    initial begin
        df = -1000.0; grab; lsb1k = tone(1000.0);
        lsb_rms = 0; for (n = 0; n < 400; n = n + 1) lsb_rms = lsb_rms + a[n] * a[n];
        lsb_rms = $sqrt(lsb_rms / 400.0);
        df = 1000.0; grab; usb1k = tone(1000.0);
        $display("dial-1kHz: audio 1 kHz amplitude %0.1f (rms %0.1f, 1k fraction %0.3f)", lsb1k, lsb_rms, lsb1k / 1.41421 / lsb_rms);
        $display("dial+1kHz: audio 1 kHz amplitude %0.2f -> rejection %0.1f dB", usb1k, 20.0 * $log10(lsb1k / (usb1k + 1e-9)));
        if (lsb1k / 1.41421 / lsb_rms < 0.98) begin errors = errors + 1; $display("FAIL: LSB audio not a clean 1 kHz"); end
        if (20.0 * $log10(lsb1k / (usb1k + 1e-9)) < 50.0) begin errors = errors + 1; $display("FAIL: opposite sideband"); end
        if (errors == 0) $display("PASS"); else $display("FAILED");
        $finish;
    end
endmodule
