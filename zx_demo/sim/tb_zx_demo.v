// Full-system run of zx_demo_top (rPLL pass-through, FAST_SIM driver
// delays): decodes the SPI stream and writes frame NSHOT to sim/frame.ppm.
// The dancer drawn after frame 0 is displayed from frame 2 on. The beat
// position is forced into part A so the stage is in its normal state.
`timescale 1ns/1ps
module tb_zx_demo;
    parameter integer NSHOT = 2;
    reg clk = 0;
    always #7.4 clk = ~clk;
    wire sck, mosi, cs, dc, rst;
    zx_demo_top dut (.clk(clk), .tft_sck(sck), .tft_mosi(mosi), .tft_cs(cs), .tft_dc(dc), .tft_rst(rst));
    initial begin #1 dut.u_beat.ord = 4'd2; dut.u_beat.step = 6'd21; dut.u_beat.tick = 3'd2; end

    reg [7:0] sh = 0; integer nb = 0;
    reg in_ram = 0, hi_phase = 0; reg [7:0] hi_byte;
    integer frame = -1, npix = 0, fd;
    reg [15:0] p;
    always @(posedge sck) begin
        sh = {sh[6:0], mosi}; nb = nb + 1;
        if (nb == 8) begin
            nb = 0;
            if (!dc) begin
                if (in_ram) begin
                    $display("frame %0d: %0d pixels at %0t", frame, npix, $time);
                    if (frame == NSHOT) begin $fclose(fd); $finish; end
                end
                in_ram = (sh == 8'h2C); hi_phase = 0; npix = 0;
                if (in_ram) begin
                    frame = frame + 1;
                    if (frame == NSHOT) begin fd = $fopen("sim/frame.ppm", "w"); $fwrite(fd, "P3\n320 240\n255\n"); end
                end
            end else if (in_ram) begin
                if (!hi_phase) begin hi_byte = sh; hi_phase = 1; end
                else begin
                    hi_phase = 0; p = {hi_byte, sh};
                    if (frame == NSHOT)
                        $fwrite(fd, "%0d %0d %0d\n", {p[15:11], 3'b0}, {p[10:5], 2'b0}, {p[4:0], 3'b0});
                    npix = npix + 1;
                end
            end
        end
    end
endmodule
