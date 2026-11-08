// 1 bit-per-pixel, full 320x240 framebuffer (no header/margin reserved -
// v1 has no text/labels, grid is a combinational overlay only, see
// osc_top.v). Linear address = y*320+x, same formula as line_draw.v's
// fb_addr output and the sibling project's trace_framebuffer.v (which
// this is directly pattern-derived from, just resized: 320*240=76800
// addrs here vs. that file's 320*206=65920). addr kept 17-bit to match
// line_draw.v's fb_addr width unmodified.
module waveform_framebuffer (
    input  wire        clk,
    input  wire [16:0] addr,
    input  wire        we,
    input  wire        din,
    output reg          dout
);

    reg mem [0:76799];

    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];
    end

endmodule
