// 1bpp framebuffer, 2 pages x 256x256 bits (rows 192-255 unused), simple
// dual port: one write port (dancer), one registered read port (display).
// Two access points in one always block -> infers BSRAM (see CLAUDE.md).
module fb_ram (
    input  wire        clk,
    input  wire        we,
    input  wire [16:0] waddr,
    input  wire        din,
    input  wire [16:0] raddr,
    output reg         dout = 1'b0
);
    reg mem [0:131071];
    always @(posedge clk) begin
        if (we)
            mem[waddr] <= din;
        dout <= mem[raddr];
    end
endmodule
