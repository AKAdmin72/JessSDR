// Thick line into a 1bpp 256x192 framebuffer: Bresenham walk from (x0,y0)
// to (x1,y1), stamping a filled disc of radius r (dx^2+dy^2 <= r^2+r) at
// every step. r = 0 draws nothing (done immediately); x0==x1,y0==y1 draws a
// single disc. One framebuffer write per clock; pixels outside the screen
// are skipped. Coordinates are signed so bones may leave the screen.
module brush_line (
    input  wire              clk,
    input  wire              start,
    input  wire signed [9:0] x0, y0, x1, y1,
    input  wire [3:0]        r,
    output reg               done = 1'b0,
    output reg               we = 1'b0,
    output reg  [7:0]        wx = 8'd0,
    output reg  [7:0]        wy = 8'd0
);
    localparam S_IDLE = 2'd0, S_STAMP = 2'd1, S_NEXT = 2'd2;
    reg [1:0] state = S_IDLE;

    reg signed [9:0]  x = 0, y = 0, xe = 0, ye = 0;
    reg signed [10:0] ddx = 0, ddy = 0, err = 0;
    reg               sx = 1'b0, sy = 1'b0;       // 1 = step negative
    reg signed [4:0]  bx = 0, by = 0;
    reg signed [4:0]  rr = 0;

    wire signed [10:0] e2   = err <<< 1;
    wire signed [9:0]  px   = x + bx;
    wire signed [9:0]  py   = y + by;
    wire [9:0]         d2   = bx * bx + by * by;
    wire [9:0]         lim  = rr * rr + rr;
    wire               in_d = (d2 <= lim);
    wire               on_s = (px >= 0) && (px < 256) && (py >= 0) && (py < 192);
    wire               last_b = (bx == rr) && (by == rr);

    wire signed [10:0] adx = (x1 > x0) ? x1 - x0 : x0 - x1;
    wire signed [10:0] ady = (y1 > y0) ? y1 - y0 : y0 - y1;

    always @(posedge clk) begin
        done <= 1'b0;
        we   <= 1'b0;
        case (state)
            S_IDLE:
                if (start) begin
                    if (r == 4'd0) done <= 1'b1;
                    else begin
                        x <= x0; y <= y0; xe <= x1; ye <= y1;
                        ddx <= adx; ddy <= -ady; err <= adx - ady;
                        sx <= (x1 < x0); sy <= (y1 < y0);
                        rr <= {1'b0, r};
                        bx <= -$signed({1'b0, r});
                        by <= -$signed({1'b0, r});
                        state <= S_STAMP;
                    end
                end
            S_STAMP: begin
                if (in_d && on_s) begin
                    we <= 1'b1;
                    wx <= px[7:0];
                    wy <= py[7:0];
                end
                if (last_b) state <= S_NEXT;
                else if (bx == rr) begin bx <= -rr; by <= by + 5'sd1; end
                else bx <= bx + 5'sd1;
            end
            S_NEXT: begin
                if (x == xe && y == ye) begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end else begin
                    // standard all-octant Bresenham step (both moves may apply)
                    if (e2 >= ddy && e2 <= ddx) begin
                        err <= err + ddy + ddx;
                        x <= sx ? x - 10'sd1 : x + 10'sd1;
                        y <= sy ? y - 10'sd1 : y + 10'sd1;
                    end else if (e2 >= ddy) begin
                        err <= err + ddy;
                        x <= sx ? x - 10'sd1 : x + 10'sd1;
                    end else begin
                        err <= err + ddx;
                        y <= sy ? y - 10'sd1 : y + 10'sd1;
                    end
                    bx <= -rr;
                    by <= -rr;
                    state <= S_STAMP;
                end
            end
            default: state <= S_IDLE;
        endcase
    end
endmodule
