// Bresenham line rasterizer. One line per start pulse; writes '1' bits
// into the framebuffer along the line from (x0,y0) to (x1,y1) inclusive.
module line_draw (
    input  wire              clk,
    input  wire              start,
    input  wire signed [9:0] x0, y0, x1, y1,
    output reg               busy,
    output reg               done,
    output reg  [16:0]       fb_addr,
    output reg               fb_we
);

    reg signed [9:0] cx, cy, ex, ey;
    reg signed [11:0] dx, dy, err;
    reg signed [1:0]  sx, sy;

    reg [2:0] state;
    localparam S_IDLE = 3'd0, S_INIT = 3'd1, S_ERR = 3'd2,
               S_PLOT = 3'd3, S_STEP = 3'd4, S_DONE = 3'd5;

    // Bresenham step, computed from currently-registered err/dx/dy/sx/sy.
    // NOTE: {a,b} concatenation is always UNSIGNED in Verilog even when a/b
    // are signed - sign-extend via plain assignment to a wider signed wire
    // instead, or a signed >=/<= comparison silently becomes unsigned.
    wire signed [12:0] err_ext = err;
    wire signed [12:0] dx_ext  = dx;
    wire signed [12:0] dy_ext  = dy;
    wire signed [12:0] e2      = err_ext <<< 1;
    wire cond_x = (e2 >= dy_ext);
    wire cond_y = (e2 <= dx_ext);

    wire signed [11:0] err_next = err + (cond_x ? dy : 12'sd0) + (cond_y ? dx : 12'sd0);
    wire signed [9:0]  cx_next  = cx  + (cond_x ? sx : 10'sd0);
    wire signed [9:0]  cy_next  = cy  + (cond_y ? sy : 10'sd0);

    wire [8:0] ux = cx[8:0];
    wire [7:0] uy = cy[7:0];
    wire [16:0] uy_ext = {9'b0, uy};
    wire [16:0] ux_ext = {8'b0, ux};
    wire [16:0] plot_addr = (uy_ext <<< 8) + (uy_ext <<< 6) + ux_ext;

    always @(posedge clk) begin
        fb_we <= 1'b0;
        case (state)
            S_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    cx <= x0; cy <= y0; ex <= x1; ey <= y1;
                    busy <= 1'b1;
                    state <= S_INIT;
                end
            end
            S_INIT: begin
                dx <= (ex >= cx) ? (ex - cx) : (cx - ex);
                dy <= (ey >= cy) ? (cy - ey) : (ey - cy); // dy = -abs(y1-y0)
                sx <= (ex >= cx) ? 2'sd1 : -2'sd1;
                sy <= (ey >= cy) ? 2'sd1 : -2'sd1;
                state <= S_ERR;
            end
            S_ERR: begin
                err <= dx + dy;
                state <= S_PLOT;
            end
            S_PLOT: begin
                fb_addr <= plot_addr;
                fb_we   <= 1'b1;
                if (cx == ex && cy == ey)
                    state <= S_DONE;
                else
                    state <= S_STEP;
            end
            S_STEP: begin
                err <= err_next;
                cx  <= cx_next;
                cy  <= cy_next;
                state <= S_PLOT;
            end
            S_DONE: begin
                busy  <= 1'b0;
                done  <= 1'b1;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
        endcase
    end

endmodule
