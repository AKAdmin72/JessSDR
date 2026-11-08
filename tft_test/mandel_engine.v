// Mandelbrot escape-time engine: one iteration per clock.
// Fixed point Q4.32 signed 36-bit (range +-8) - 36x36 is exactly one Gowin
// MULT36X36, so the extra 4 bits over 32 are free. z starts at c (iteration
// 0 done implicitly). Escape test |z|^2 > 4 uses the full 72-bit products.
// While |z| <= 2 the next z is at most 4+|c| < 8, so no overflow before the
// escape is detected.
// Outputs on `done` (1-cycle pulse): iter = iterations run, in_set = hit
// maxit. iter/in_set hold until the next done.
module mandel_engine (
    input  wire               clk,
    input  wire               start,
    input  wire signed [35:0] cr,
    input  wire signed [35:0] ci,
    input  wire [9:0]         maxit,
    output reg                busy   = 1'b0,
    output reg                done   = 1'b0,
    output reg  [9:0]         iter   = 10'd0,
    output reg                in_set = 1'b0
);
    reg signed [35:0] zr = 36'sd0, zi = 36'sd0, cr_q = 36'sd0, ci_q = 36'sd0;
    reg [9:0] it = 10'd0;

    wire signed [71:0] p_rr = zr * zr;
    wire signed [71:0] p_ii = zi * zi;
    wire signed [71:0] p_ri = zr * zi;
    wire        [72:0] mag  = {1'b0, p_rr} + {1'b0, p_ii};   // both >= 0
    wire escaped = (mag > (73'd4 << 64));
    wire last    = (it == maxit);
    wire finish  = busy && (escaped || last);
    wire signed [71:0] d_rr_ii = p_rr - p_ii;
    wire signed [35:0] zr_next = d_rr_ii[67:32] + cr_q;
    wire signed [35:0] zi_next = p_ri[66:31] + ci_q;          // 2*zr*zi

    always @(posedge clk)
        if (start) cr_q <= cr;
    always @(posedge clk)
        if (start) ci_q <= ci;
    always @(posedge clk)
        if (start)                 zr <= cr;
        else if (busy && !finish)  zr <= zr_next;
    always @(posedge clk)
        if (start)                 zi <= ci;
        else if (busy && !finish)  zi <= zi_next;
    always @(posedge clk)
        if (start)                 it <= 10'd0;
        else if (busy && !finish)  it <= it + 10'd1;
    always @(posedge clk)
        if (start)       busy <= 1'b1;
        else if (finish) busy <= 1'b0;
    always @(posedge clk)
        done <= finish && !start;
    always @(posedge clk)
        if (finish) iter <= it;
    always @(posedge clk)
        if (finish) in_set <= !escaped;
endmodule
