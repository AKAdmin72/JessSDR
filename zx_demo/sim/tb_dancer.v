// Renders every key pose (and one mirrored, one mid-interpolation frame)
// with dancer.v into sim/poses.pgm (4x4 grid of 256x192 screens).
`timescale 1ns/1ps
module tb_dancer;
    reg clk = 0;
    always #5 clk = ~clk;
    reg start = 0, mirror = 0;
    reg [3:0] pa = 0, pb = 0;
    reg [7:0] ph = 0;
    wire done, we, din;
    wire [16:0] addr;
    dancer dut (.clk(clk), .start(start), .page(1'b0), .pose_a(pa), .pose_b(pb), .ph(ph),
                .mirror(mirror), .hip_x(10'sd128), .hip_y(10'sd100),
                .done(done), .fb_we(we), .fb_addr(addr), .fb_din(din));
    reg fb [0:65535];
    always @(posedge clk) if (we) fb[addr[15:0]] <= din;

    reg img [0:786431];
    integer p, x, y, gx, gy, fd, cycles;
    task render(input integer slot);
        begin
            @(negedge clk); start = 1; @(negedge clk); start = 0;
            cycles = 0;
            @(posedge clk); #1; cycles = 1; while (!done && cycles < 400000) begin @(posedge clk); #1; cycles = cycles + 1; end
            if (!done) $display("HANG in slot %0d, dut state %0d bone %0d", slot, dut.state, dut.bi);
            $display("slot %0d: pose %0d->%0d ph %0d mirror %0d, %0d cycles", slot, pa, pb, ph, mirror, cycles);
            gx = (slot % 4) * 256; gy = (slot / 4) * 192;
            for (y = 0; y < 192; y = y + 1)
                for (x = 0; x < 256; x = x + 1)
                    img[(gy + y) * 1024 + gx + x] = fb[y * 256 + x];
        end
    endtask
    initial begin
        for (p = 0; p < 12; p = p + 1) begin pa = p; pb = p; ph = 0; mirror = 0; render(p); end
        pa = 4; pb = 4; mirror = 1; render(12);                 // mirrored knee-up
        pa = 0; pb = 7; ph = 128; mirror = 0; render(13);        // halfway stand -> twirl
        pa = 10; pb = 11; ph = 128; render(14);
        pa = 6; pb = 1; ph = 200; render(15);
        fd = $fopen("sim/poses.pgm", "w");
        $fwrite(fd, "P2\n1024 768\n1\n");
        for (y = 0; y < 768; y = y + 1) begin
            for (x = 0; x < 1024; x = x + 1)
                $fwrite(fd, "%0d ", (x % 256 == 0 || y % 192 == 0) ? 0 : (img[y * 1024 + x] === 1'b1 ? 0 : 1));
            $fwrite(fd, "\n");
        end
        $fclose(fd);
        $finish;
    end
endmodule
