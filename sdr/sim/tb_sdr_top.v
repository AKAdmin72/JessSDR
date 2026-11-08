// Whole sdr_top for one display frame (FAST_SIM driver delays, rPLL
// pass-through, ADC = 1 kHz-offset tone + noise): decodes the SPI stream into
// sim/screen.ppm. Visual check of the UI layout.
`timescale 1ns/1ps
module tb_sdr_top;
    reg clk = 0;
    always #7.716 clk = ~clk;
    wire adc_clk, sck, mosi, cs, dc, rst, txd, bclk, lrck, din, pa;
    reg [11:0] code = 12'd2048;
    real t = 0.0;
    integer seed = 1;
    always @(posedge clk) begin
        code <= 2048 + $rtoi(300.0 * $cos(2.0 * 3.14159265358979 * 7.11e6 * t) + 4096.5) - 4096 + ($random(seed) % 3);
        t = t + 1.0 / 64.8e6;
    end
    sdr_top dut (.clk(clk), .btn_s1(1'b0), .btn_s2(1'b0), .adc_clk(adc_clk), .adc_d(code), .adc_otr(1'b0),
                 .tft_sck(sck), .tft_mosi(mosi), .tft_cs(cs), .tft_dc(dc), .tft_rst(rst),
                 .uart_tx(txd), .uart_rx(1'b1), .i2s_bclk(bclk), .i2s_lrck(lrck), .i2s_din(din), .pa_en(pa));
    localparam integer NF = 3;                 // frame written to screen.ppm
    defparam dut.u_spec.NAVG_LOG2 = 1;         // a spectrum line every ~20 ms
    // pre-fill the waterfall with a palette ramp (left->right) so the colours show
    integer wi;
    initial for (wi = 0; wi < 32768; wi = wi + 1) dut.wfm[wi] = ((wi % 320) * 255) / 319;
    initial for (wi = 0; wi < 512; wi = wi + 1) dut.spm[wi] = 9'd0;
    reg [7:0] sh = 0; integer nb = 0, frame = -1, npix = 0, fd;
    reg in_ram = 0, hi_phase = 0; reg [7:0] hi_byte; reg [15:0] p;
    always @(posedge sck) begin
        sh = {sh[6:0], mosi}; nb = nb + 1;
        if (nb == 8) begin
            nb = 0;
            if (!dc) begin
                if (in_ram) begin $display("frame %0d: %0d px, level %0d", frame, npix, dut.level); if (frame == NF) begin $fclose(fd); $finish; end end
                in_ram = (sh == 8'h2C); hi_phase = 0; npix = 0;
                if (in_ram) begin frame = frame + 1;
                    if (frame == NF) begin fd = $fopen("sim/screen.ppm", "w"); $fwrite(fd, "P3\n320 240\n255\n"); end end
            end else if (in_ram) begin
                if (!hi_phase) begin hi_byte = sh; hi_phase = 1; end
                else begin hi_phase = 0; p = {hi_byte, sh};
                    if (frame == NF) $fwrite(fd, "%0d %0d %0d\n", {p[15:11], 3'b0}, {p[10:5], 2'b0}, {p[4:0], 3'b0});
                    npix = npix + 1; end
            end
        end
    end
endmodule
