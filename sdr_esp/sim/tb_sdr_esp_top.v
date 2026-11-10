// Whole sdr_esp_top at 64.8 MHz (sim/../../sdr/sim/rpll_sim.v pass-through):
// ADC = tone at 7.099 MHz (1 kHz below the 7.100 MHz dial) + noise.
// A bit-banged SPI master (the ESP's role, 2 MHz mode 0) and an I2S slave
// receiver on the link pins check:
//   ID / VERSION, FREQ write -> LO_FW = (f * 69500145) >> 20, MODE / NR /
//   CTRL read-back, PEAK of the known ADC amplitude, LEVEL > 0,
//   I2S test pattern, then the wide I/Q: the tone must come out at -1 kHz
//   (>= 40 dB over +1 kHz), and LSB audio must be a ~1 kHz tone.
`timescale 1ns/1ps
module tb_sdr_esp_top;
    reg clk = 0;
    always #7.716 clk = ~clk;
    reg [11:0] code = 12'd2048;
    real t = 0.0;
    integer seed = 1;
    localparam real F_TONE = 7.099e6, A_TONE = 300.0;
    always @(posedge clk) begin
        code <= 2048 + $rtoi(A_TONE * $cos(2.0 * 3.14159265358979 * F_TONE * t) + 4096.5) - 4096 + ($random(seed) % 3);
        t = t + 1.0 / 64.8e6;
    end
    reg sck = 0, cs_n = 1, mosi = 0;
    wire miso, bck, ws, data, adc_clk, txd, abclk, alrck, adin, pa;
    sdr_esp_top #(.PEAK_WIN(14)) dut (.clk(clk), .btn_s1(1'b0), .btn_s2(1'b0), .adc_clk(adc_clk), .adc_d(code), .adc_otr(1'b0),
        .uart_tx(txd), .uart_rx(1'b1), .i2s_bclk(abclk), .i2s_lrck(alrck), .i2s_din(adin), .pa_en(pa),
        .esp_sck(sck), .esp_cs_n(cs_n), .esp_mosi(mosi), .esp_miso(miso),
        .esp_bck(bck), .esp_ws(ws), .esp_data(data));

    integer errors = 0;
    localparam HALF = 250;
    task xfer(input [7:0] o, output [7:0] i);
        integer b;
        for (b = 7; b >= 0; b = b - 1) begin
            mosi = o[b]; #HALF sck = 1; i[b] = miso; #HALF sck = 0;
        end
    endtask
    reg [7:0] dmy;
    task rd(input [6:0] a, output [7:0] v);
        begin cs_n = 0; #HALF; xfer({1'b1, a}, dmy); xfer(8'h00, v); #HALF cs_n = 1; #(4*HALF); end
    endtask
    task wr(input [6:0] a, input [7:0] v);
        begin cs_n = 0; #HALF; xfer({1'b0, a}, dmy); xfer(v, dmy); #HALF cs_n = 1; #(4*HALF); end
    endtask
    task rd32(input [6:0] a, output [31:0] v);
        begin cs_n = 0; #HALF; xfer({1'b1, a}, dmy);
              xfer(8'h00, v[31:24]); xfer(8'h00, v[23:16]); xfer(8'h00, v[15:8]); xfer(8'h00, v[7:0]);
              #HALF cs_n = 1; #(4*HALF); end
    endtask
    task wr_freq(input [31:0] bcd);
        begin cs_n = 0; #HALF; xfer(8'h04, dmy);
              xfer(bcd[31:24], dmy); xfer(bcd[23:16], dmy); xfer(bcd[15:8], dmy); xfer(bcd[7:0], dmy);
              #HALF cs_n = 1; #(4*HALF); end
    endtask
    task expect8(input [7:0] got, input [7:0] want, input [8*16-1:0] what);
        if (got !== want) begin $display("FAIL: %0s = %02h, expected %02h", what, got, want); errors = errors + 1; end
    endtask

    // ---- I2S slave receiver on the link (samples on BCK rises) ----
    reg [31:0] acc = 0; integer nb = -1; reg ch = 0, ws_seen = 0;
    reg [31:0] lw = 0; reg have_l = 0;
    integer frames = 0, tbad = 0; reg [30:0] lastc = 0; reg havec = 0;
    reg collect = 0, test_chk = 0;
    integer ncol = 0;
    real sp_r = 0, sp_i = 0, sn_r = 0, sn_i = 0, ph;
    always @(posedge bck) begin
        if (ws != ws_seen) begin
            if (nb == 31) begin
                if (!ch) begin lw = {acc[30:0], data}; have_l = 1; end
                else if (have_l) frame({acc[30:0], data});
            end
            ws_seen = ws; ch = ws; nb = 0; acc = 0;
        end else if (nb >= 0 && nb < 31) begin
            acc = {acc[30:0], data}; nb = nb + 1;
        end
    end
    task frame(input [31:0] r);
        real xi, xq, w;
        begin
            frames = frames + 1;
            if (test_chk) begin
                if (r[31:1] !== ~lw[31:1] || (havec && lw[31:1] !== lastc + 1)) tbad = tbad + 1;
                lastc = lw[31:1]; havec = 1;
            end
            if (collect) begin
                // single-bin DFTs at +-1 kHz (fs = 64.8 MHz / 614)
                // Hann window: a rectangular one leaks -36 dB from -1 to +1 kHz
                w = 0.5 - 0.5 * $cos(2.0 * 3.14159265358979 * ncol / 1024.0);
                xi = w * $signed(lw) / 2147483648.0; xq = w * $signed(r) / 2147483648.0;
                ph = 2.0 * 3.14159265358979 * 1000.0 * ncol / (64.8e6 / 614.0);
                // X(+f) = sum x e^{-j ph}, x = xi + j xq
                sp_r = sp_r + xi * $cos(ph) + xq * $sin(ph);
                sp_i = sp_i + xq * $cos(ph) - xi * $sin(ph);
                sn_r = sn_r + xi * $cos(ph) - xq * $sin(ph);
                sn_i = sn_i + xq * $cos(ph) + xi * $sin(ph);
                ncol = ncol + 1;
            end
        end
    endtask

    // ---- audio after NR: zero crossings ----
    integer na = 0, zc = 0, amax = 0; reg signed [15:0] aprev = 0; reg acount = 0;
    always @(posedge clk) if (dut.nr_v && acount) begin
        na = na + 1;
        if ((dut.nr_out >= 0) != (aprev >= 0)) zc = zc + 1;
        if (dut.nr_out > amax) amax = dut.nr_out;
        aprev = dut.nr_out;
    end

    reg [7:0] v; reg [31:0] v32, want32;
    real pp, pn, fa;
    initial begin
        #2000;
        rd(7'h00, v); expect8(v, 8'h5A, "ID");
        rd(7'h01, v); expect8(v, 8'h05, "VERSION");
        wr(7'h02, 8'hA5); rd(7'h02, v); expect8(v, 8'hA5, "SCRATCH");
        // tune away and back: 14.074 MHz, then 7.100 MHz
        wr_freq(32'h14074000); #2000;
        rd32(7'h08, v32); want32 = (64'd14074000 * 64'd69500145) >> 20;
        if (v32 !== want32 || dut.freq !== 25'd14074000) begin
            $display("FAIL: LO_FW %08h (freq %0d), expected %08h", v32, dut.freq, want32); errors = errors + 1; end
        wr_freq(32'h07100000); #2000;
        rd32(7'h08, v32); want32 = (64'd7100000 * 64'd69500145) >> 20;
        if (v32 !== want32) begin $display("FAIL: LO_FW %08h, expected %08h", v32, want32); errors = errors + 1; end
        rd32(7'h04, v32); if (v32 !== 32'h07100000) begin $display("FAIL: FREQ read-back %08h", v32); errors = errors + 1; end
        rd(7'h0C, v); expect8(v, 8'h00, "LO_SHIFT_K");
        wr(7'h0D, 8'h02); rd(7'h0D, v); expect8(v, 8'h02, "MODE AM");
        if (!dut.mode_am) begin $display("FAIL: AM not selected"); errors = errors + 1; end
        wr(7'h0D, 8'h00); expect8({6'd0, dut.mode}, 8'h00, "MODE LSB");
        wr(7'h13, 8'h04); rd(7'h13, v); expect8(v, 8'h04, "NR");
        if (dut.esp_nr !== 4'h4) begin $display("FAIL: NR not at audio_nr"); errors = errors + 1; end
        wr(7'h13, 8'h00);

        // I2S test pattern
        wr(7'h03, 8'h01); #(30 * 9500); test_chk = 1; #(200 * 9476);
        test_chk = 0; wr(7'h03, 8'h00);
        if (tbad != 0 || !havec) begin $display("FAIL: I2S test pattern, %0d bad frames", tbad); errors = errors + 1; end
        else $display("I2S test pattern ok (frames so far %0d)", frames);

        // PEAK: amplitude 300 (+-2 noise) -> 300 >> 3 = 37; window 2^14 clk
        #(3 * 16384 * 15.432);
        rd(7'h14, v);
        if (v < 8'd36 || v > 8'd38) begin $display("FAIL: PEAK %0d, expected ~37", v); errors = errors + 1; end
        else $display("PEAK %0d", v);

        // wide I/Q: let the CIC settle, then 1024 frames
        #2000000; collect = 1;
        wait (ncol == 1024); collect = 0;
        pp = sp_r * sp_r + sp_i * sp_i; pn = sn_r * sn_r + sn_i * sn_i;
        $display("I2S I/Q: -1 kHz %.1f dB, +1 kHz %.1f dB (rel. full scale per bin)",
                 10.0 * $log10(pn / 1048576.0 + 1e-30), 10.0 * $log10(pp / 1048576.0 + 1e-30));
        if (pn < pp * 1e4 || pn < 1e-6 * 1048576.0) begin
            $display("FAIL: tone not at -1 kHz in the I2S I/Q"); errors = errors + 1; end

        // LSB audio: ~1 kHz at 13.18 kHz
        acount = 1; #20000000; acount = 0;
        fa = zc / 2.0 / (na / 13183.0);
        rd(7'h15, v);
        $display("audio: %0d samples, %0d zero crossings -> %.0f Hz, peak %0d; LEVEL %0d", na, zc, fa, amax, v);
        if (fa < 900 || fa > 1100 || amax < 1000) begin $display("FAIL: LSB audio"); errors = errors + 1; end
        if (v == 0) begin $display("FAIL: LEVEL 0"); errors = errors + 1; end

        if (errors == 0) $display("PASS: sdr_esp_top - SPI regs, BCD tuning, I2S pattern + I/Q, PEAK, LSB audio");
        else $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
