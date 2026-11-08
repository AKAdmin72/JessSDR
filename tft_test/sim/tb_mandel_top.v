// System test of mandel_top: rPLL stand-in (pass-through), FAST_SIM driver
// delays, an SPI receiver that decodes the ILI9341 stream, and a behavioural
// reference of the escape-time algorithm. Frame 0 must match the reference
// pixel-for-pixel (checks raster order through the 4-engine round-robin) and
// contain exactly 76800 pixels; frame 0 is also written to sim/top_frame0.ppm.
`timescale 1ns/1ps
module tb_mandel_top;
    reg clk = 0;
    always #18.5 clk = ~clk;
    wire sck, mosi, cs, dc, rst;
    mandel_top #(.CLK_KHZ(27000)) dut (
        .clk_in(clk), .btn_s1(1'b0), .btn_s2(1'b0),
        .tft_sck(sck), .tft_mosi(mosi), .tft_cs(cs), .tft_dc(dc), .tft_rst(rst));

    // reference (same fixed point as mandel_engine)
    function [10:0] ref_iter(input signed [35:0] cr, input signed [35:0] ci, input [9:0] maxit);
        reg signed [35:0] zr, zi;
        reg signed [71:0] rr, ii, ri, d;
        reg [72:0] mag;
        integer it, fin;
        begin
            zr = cr; zi = ci; fin = 0; ref_iter = 0;
            for (it = 0; it <= maxit && !fin; it = it + 1) begin
                rr = zr * zr; ii = zi * zi; ri = zr * zi;
                mag = {1'b0, rr} + {1'b0, ii};
                if (mag > (73'd4 << 64)) begin ref_iter = {1'b0, it[9:0]}; fin = 1; end
                else if (it == maxit) begin ref_iter = {1'b1, it[9:0]}; fin = 1; end
                else begin d = rr - ii; zr = d[67:32] + cr; zi = ri[66:31] + ci; end
            end
        end
    endfunction
    function [7:0] tri8(input [7:0] v);
        tri8 = v[7] ? {~v[6:0], 1'b0} : {v[6:0], 1'b0};
    endfunction
    function [15:0] ref_color(input integer x, input integer y);
        reg signed [35:0] step, cr, ci;
        reg [10:0] r;
        reg [7:0] t, pr, pg, pb;
        begin
            step = 36'sd42949673;
            cr = -36'sd3193926175 - 160 * step + x * step;
            ci =  36'sd566187947  + 120 * step - y * step;
            r = ref_iter(cr, ci, 10'd64);
            t = {r[5:0], 2'b00};
            pr = tri8(t); pg = tri8(t + 8'd85); pb = tri8(t + 8'd170);
            ref_color = r[10] ? 16'h0000 : {pr[7:3], pg[7:2], pb[7:3]};
        end
    endfunction

    // SPI receiver
    reg [7:0] sh = 0; integer nb = 0;
    reg in_ram = 0, hi_phase = 0; reg [7:0] hi_byte;
    integer frame = 0, npix = 0, errors = 0, fd, x, y;
    reg [15:0] p, e;
    always @(posedge sck) begin
        sh = {sh[6:0], mosi}; nb = nb + 1;
        if (nb == 8) begin
            nb = 0;
            if (!dc) begin
                if (in_ram) begin                      // a command ends the pixel stream
                    if (frame == 0 && npix != 76800) begin errors = errors + 1; $display("FAIL frame0 npix=%0d", npix); end
                    if (frame == 0) begin
                        $fclose(fd);
                        $display("frame 0 done: %0d pixels, %0d errors", npix, errors);
                        if (errors == 0) $display("PASS"); else $display("FAILED");
                        $finish;
                    end
                    frame = frame + 1;
                end
                in_ram = (sh == 8'h2C); hi_phase = 0; npix = 0;
                if (sh == 8'h2C && frame == 0) begin
                    fd = $fopen("sim/top_frame0.ppm", "w"); $fwrite(fd, "P3\n320 240\n255\n");
                end
            end else if (in_ram) begin
                if (!hi_phase) begin hi_byte = sh; hi_phase = 1; end
                else begin
                    hi_phase = 0; p = {hi_byte, sh};
                    x = npix % 320; y = npix / 320;
                    if (frame == 0) begin
                        $fwrite(fd, "%0d %0d %0d\n", {p[15:11], 3'b0}, {p[10:5], 2'b0}, {p[4:0], 3'b0});
                        e = ref_color(x, y);
                        if (p !== e) begin
                            errors = errors + 1;
                            if (errors < 10) $display("FAIL px %0d,%0d got %h exp %h", x, y, p, e);
                        end
                    end
                    npix = npix + 1;
                end
            end
        end
    end
    // driver toggles cs only once; bytes are framed by the 8-bit counter
    initial begin #400_000_000; $display("TIMEOUT frame=%0d npix=%0d", frame, npix); $finish; end
endmodule
