`timescale 1ns/1ps
// Speaker path audio_up4 -> audio_i2s (IN_PERIOD 1229), as in sdr_esp_top:
// a 3 kHz tone (amp 16000) at 13.18 kHz in, the 48.21 kHz samples the I2S
// frame carries (audio_i2s.smp, latched per frame) to sim/audio_out48.txt.
// sim/check_audio_out.py: tone gain and the worst spur (images) in the band.
module tb_audio_out;
    reg clk = 0;
    always #7.716 clk = ~clk;
    reg vin = 0;
    reg signed [15:0] xin = 0;
    wire up_v;
    wire signed [15:0] up_aud;
    wire bclk, lrck, din;
    audio_up4 #(.PERIOD(1229)) u_up (.clk(clk), .in_valid(vin), .in_audio(xin), .out_valid(up_v), .out_audio(up_aud));
    audio_i2s #(.IN_PERIOD(1229)) u_i2s (.clk(clk), .in_valid(up_v), .in_audio(up_aud), .bclk(bclk), .lrck(lrck), .din(din));

    real PI = 3.14159265358979, FS = 64.8e6 / 4916.0;
    integer n, f, nout = 0;
    always @(posedge clk) if (u_i2s.fall && u_i2s.bnext == 6'd0) begin
        #1 $fwrite(f, "%0d\n", $signed(u_i2s.smp));
        nout = nout + 1;
    end
    initial begin
        f = $fopen("sim/audio_out48.txt", "w");
        for (n = 0; n < 2000; n = n + 1) begin
            repeat (4915) @(posedge clk);
            #1 xin = $rtoi(16000.0 * $sin(2.0 * PI * 3000.0 * n / FS)); vin = 1;
            @(posedge clk); #1 vin = 0;
        end
        $fclose(f);
        $display("%0d samples at 48.2 kHz", nout);
        $finish;
    end
endmodule
