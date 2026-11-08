// ddc: tone at f_lo + DF must come out as a complex exponential at +DF
// (I/Q rotating counter-clockwise), with the expected amplitude; a tone
// 52 kHz away (aliases to -0.7 kHz, i.e. into the audio band) must be
// rejected by the CIC null at the output rate.
`timescale 1ns/1ps
module tb_ddc;
    reg clk = 0;
    always #7.716 clk = ~clk;                      // 64.8 MHz
    localparam real FS = 64.8e6, FLO = 7.1e6;
    reg  [11:0] code = 12'd2048;
    wire [31:0] inc = 32'd470590552;               // round(7.1e6/64.8e6*2^32)
    wire v; wire signed [17:0] oi, oq;
    ddc dut (.clk(clk), .adc_code(code), .phase_inc(inc), .out_valid(v), .out_i(oi), .out_q(oq));

    real df = 1000.0, amp = 200.0, t = 0.0;
    integer n = 0, errors = 0;
    always @(posedge clk) begin
        code <= 2048 + $rtoi(amp * $cos(2.0 * 3.14159265358979 * (FLO + df) * t) + 4096.5) - 4096;
        t = t + 1.0 / FS;
    end

    real ph, ph_prev, dph, mag, sum_dph, sum_mag, ri, rq;
    integer cnt;
    task measure(output real f_meas, output real a_meas);
        begin
            cnt = 0; sum_dph = 0; sum_mag = 0;
            // skip CIC settling
            repeat (8) @(posedge v);
            @(posedge v); #1; ph_prev = $atan2(oq, oi);
            repeat (200) begin
                @(posedge v); #1;
                ph = $atan2(oq, oi);
                dph = ph - ph_prev;
                if (dph > 3.14159265) dph = dph - 6.2831853;
                if (dph < -3.14159265) dph = dph + 6.2831853;
                sum_dph = sum_dph + dph;
                ri = oi; rq = oq;
                sum_mag = sum_mag + $sqrt(ri * ri + rq * rq);
                ph_prev = ph; cnt = cnt + 1;
            end
            f_meas = sum_dph / cnt / 6.2831853 * (FS / 1229.0);
            a_meas = sum_mag / cnt;
        end
    endtask

    real f1, a1, f2, a2, f3, a3, exp_a;
    initial begin
        df = 1000.0;  measure(f1, a1);
        df = -1500.0; measure(f2, a2);
        df = 52000.0; measure(f3, a3);     // would alias to -0.7 kHz: must sit in a CIC null
        // amplitude: amp * 32767/2 * R^4 / 2^47
        exp_a = 200.0 * 32767.0 / 2.0 * (1229.0 ** 4) / (2.0 ** 47);
        $display("+1000 Hz -> %0.1f Hz, |IQ| %0.1f (exp_a %0.1f)", f1, a1, exp_a);
        $display("-1500 Hz -> %0.1f Hz, |IQ| %0.1f", f2, a2);
        $display("+52 kHz  -> |IQ| %0.3f (%0.1f dB rel.)", a3, 20.0 * $log10(a3 / a1 + 1e-9));
        if (f1 < 990 || f1 > 1010)   begin errors = errors + 1; $display("FAIL f1"); end
        if (f2 > -1490 || f2 < -1510) begin errors = errors + 1; $display("FAIL f2"); end
        if (a1 < 0.95 * exp_a || a1 > 1.05 * exp_a) begin errors = errors + 1; $display("FAIL amplitude"); end
        if (a3 > a1 * 0.0001) begin errors = errors + 1; $display("FAIL rejection"); end
        if (errors == 0) $display("PASS"); else $display("FAILED %0d", errors);
        $finish;
    end
endmodule
