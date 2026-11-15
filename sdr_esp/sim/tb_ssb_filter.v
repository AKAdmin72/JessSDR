`timescale 1ns/1ns
// sdr_esp/ssb_demod.v (programmable 255-tap filter). Complex tones at the
// 52.7 kHz input, audio amplitude by correlation at the 13.18 kHz output.
//   (1) power-up taps (cf_coef.hex, LSB 300..2700): -1000 Hz passes,
//       +1000 Hz (USB side) >= 60 dB down;
//   (2) CW 500 taps written through cf_we (sim/cf_cw500.hex, -950..-450):
//       -700 Hz within 1 dB of the LSB -1000 Hz level, -1500 Hz >= 40 dB
//       down, -2200 Hz >= 60 dB down, +700 Hz >= 60 dB down.
module tb_ssb_filter;
    reg clk = 0;
    always #7.7 clk = ~clk;

    reg               vin = 0;
    reg signed [17:0] xi = 0, xq = 0;
    wire              vout, dv;
    wire signed [17:0] aud, di, dq;
    reg               cf_we = 0;
    reg  [7:0]        cf_waddr = 0;
    reg  [35:0]       cf_wdata = 0;

    ssb_demod dut (.clk(clk), .in_valid(vin), .in_i(xi), .in_q(xq),
                   .out_valid(vout), .out_audio(aud), .dec_valid(dv), .dec_i(di), .dec_q(dq),
                   .cf_we(cf_we), .cf_waddr(cf_waddr), .cf_wdata(cf_wdata));

    real PI = 3.14159265358979, FS_IN = 64.8e6 / 1229.0;
    real FS_OUT;
    integer errors = 0;
    reg [35:0] cw [0:255];

    real ys [0:2047];
    integer no;
    always @(posedge clk) if (vout) begin if (no >= 500 && no < 2548) ys[no - 500] = aud; no = no + 1; end

    // amplitude of the audio at f over the last 2048 outputs, after n inputs of tone f_in
    real amp;
    task tone(input real f_in, input real f_out);
        integer m;
        real si, sc;
        begin
            no = 0;
            for (m = 0; m < 4 * 2600; m = m + 1) begin
                @(posedge clk); #1;
                xi = $rtoi(60000.0 * $cos(2.0*PI*f_in*m/FS_IN));
                xq = $rtoi(60000.0 * $sin(2.0*PI*f_in*m/FS_IN));
                vin = 1;
                @(posedge clk); #1 vin = 0;
                repeat (398) @(posedge clk);
            end
            si = 0; sc = 0;
            for (m = 0; m < 2048; m = m + 1) begin
                si = si + ys[m] * $sin(2.0*PI*f_out*m/FS_OUT);
                sc = sc + ys[m] * $cos(2.0*PI*f_out*m/FS_OUT);
            end
            amp = 2.0 * $sqrt(si*si + sc*sc) / 2048.0 + 1e-3;
        end
    endtask

    real r0, a;
    integer k;
    initial begin
        FS_OUT = FS_IN / 4.0;
        $readmemh("sim/cf_cw500.hex", cw);
        repeat (3) @(posedge clk);

        tone(-1000.0, 1000.0); r0 = amp;
        tone(1000.0, 1000.0); a = amp;
        $display("LSB: -1000 Hz amp %0.0f, +1000 Hz %0.1f dB", r0, 20.0*$log10(a/r0));
        if (r0 < 10000.0) begin $display("FAIL: LSB passband level"); errors = errors + 1; end
        if (a > r0 * 0.001) begin $display("FAIL: LSB opposite sideband not >= 60 dB down"); errors = errors + 1; end

        for (k = 0; k < 256; k = k + 1) begin
            @(posedge clk); #1 cf_we = 1; cf_waddr = k; cf_wdata = cw[k];
        end
        @(posedge clk); #1 cf_we = 0;

        tone(-700.0, 700.0);
        $display("CW500: -700 Hz %0.1f dB re LSB", 20.0*$log10(amp/r0));
        if (amp < r0 * 0.891 || amp > r0 * 1.122) begin $display("FAIL: CW passband level"); errors = errors + 1; end
        a = amp;
        tone(-1500.0, 1500.0);
        $display("CW500: -1500 Hz %0.1f dB", 20.0*$log10(amp/a));
        if (amp > a * 0.01) begin $display("FAIL: CW -1500 not >= 40 dB down"); errors = errors + 1; end
        tone(-2200.0, 2200.0);
        $display("CW500: -2200 Hz %0.1f dB", 20.0*$log10(amp/a));
        if (amp > a * 0.001) begin $display("FAIL: CW -2200 not >= 60 dB down"); errors = errors + 1; end
        tone(700.0, 700.0);
        $display("CW500: +700 Hz %0.1f dB", 20.0*$log10(amp/a));
        if (amp > a * 0.001) begin $display("FAIL: CW +700 not >= 60 dB down"); errors = errors + 1; end

        if (errors == 0) $display("PASS: ssb_demod programmable filter - LSB default, CW 500 loaded");
        else             $display("FAIL: ssb_demod (%0d errors)", errors);
        $finish;
    end
endmodule
