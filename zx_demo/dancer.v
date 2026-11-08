// Dancer renderer: clears one 256x192 framebuffer page, then walks the
// skeleton table (bone_rom) doing forward kinematics and draws every bone
// as a thick line (brush_line).
//
// Each bone: angle = lerp(pose_a[var], pose_b[var], ph) + offset + k*spread,
// mirrored (128 - angle) if `mirror`; end point = parent + len*(cos, sin).
// The root (hip) is at (hip_x, hip_y + lerp(dy)). spread (skirt fan) =
// 7 + lerp(flare)/2. All inputs are sampled on `start`.
// Framebuffer write: {page, y[7:0], x[7:0]}.
module dancer (
    input  wire              clk,
    input  wire              start,
    input  wire              page,
    input  wire [3:0]        pose_a,
    input  wire [3:0]        pose_b,
    input  wire [7:0]        ph,            // 0 = pose_a, 255 = ~pose_b
    input  wire              mirror,
    input  wire signed [9:0] hip_x,
    input  wire signed [9:0] hip_y,
    output reg               done = 1'b0,
    output reg               fb_we = 1'b0,
    output reg  [16:0]       fb_addr = 17'd0,
    output reg               fb_din = 1'b0
);
    localparam S_IDLE = 4'd0, S_CLEAR = 4'd1, S_DY = 4'd2, S_FLARE = 4'd3,
               S_B0 = 4'd4, S_B1 = 4'd5, S_B2 = 4'd6, S_B3 = 4'd7, S_B4 = 4'd8,
               S_LINE = 4'd9, S_DYW = 4'd10, S_FLW = 4'd11, S_B0W = 4'd12, S_B3W = 4'd13;
    // *W states: one-cycle waits for the pipeline registers lerp_q / ddx_q
    // (keeps ROM -> multiply -> add out of a single clock at 67.5 MHz)
    localparam [4:0] NBONES = 5'd31;
    reg [3:0] state = S_IDLE;

    reg        pg = 1'b0, mir = 1'b0;
    reg [3:0]  pa = 4'd0, pb = 4'd0;
    reg [7:0]  phr = 8'd0;
    reg [15:0] caddr = 16'd0;
    reg [4:0]  bi = 5'd0;
    reg [3:0]  field = 4'd0;

    // ---- pose interpolation (combinational on `field`) ----
    wire [7:0] fa, fb;
    pose_rom u_pa (.a({pa, field}), .d(fa));
    pose_rom u_pb (.a({pb, field}), .d(fb));
    // wrap-aware difference in 256-units: (fb - fa) as signed 8-bit
    wire signed [7:0]  wdiff = fb - fa;
    wire signed [16:0] wprod = wdiff * $signed({1'b0, phr});
    wire [7:0]         flerp = fa + wprod[15:8];
    reg  [7:0]         lerp_q = 8'd0;
    always @(posedge clk) lerp_q <= flerp;

    // ---- bone table ----
    wire [38:0] bone;
    bone_rom u_bone (.a(bi), .d(bone));
    wire [4:0]        b_par = bone[38:34];
    wire [4:0]        b_out = bone[33:29];
    wire [3:0]        b_var = bone[28:25];
    wire [7:0]        b_off = bone[24:17];
    wire signed [2:0] b_k   = bone[16:14];
    wire [5:0]        b_len = bone[13:8];
    wire [3:0]        b_r   = bone[3:0];

    // ---- trig ----
    reg  [7:0] ang = 8'd0;
    wire [7:0] s_raw, c_raw;
    sin_rom u_sin (.a(ang), .d(s_raw));
    sin_rom u_cos (.a(ang + 8'd64), .d(c_raw));
    wire signed [14:0] ddx = $signed(c_raw) * $signed({1'b0, b_len});
    wire signed [14:0] ddy = $signed(s_raw) * $signed({1'b0, b_len});
    reg  signed [14:0] ddx_q = 15'sd0, ddy_q = 15'sd0;
    always @(posedge clk) ddx_q <= ddx;
    always @(posedge clk) ddy_q <= ddy;

    // ---- points ----
    reg signed [9:0] px [0:18];
    reg signed [9:0] py [0:18];
    reg signed [9:0] hx = 0, hy = 0;
    reg signed [9:0] ox = 0, oy = 0;
    reg [7:0]        avar = 8'd0;
    reg [4:0]        spread = 5'd7;
    // k*spread as its own signed wire: mixed signed/unsigned in one
    // expression would zero-extend the negative k
    wire signed [7:0] kspr = $signed(b_k) * $signed({1'b0, spread});
    wire [7:0]        ang_raw = avar + b_off + kspr;
    integer i;
    initial for (i = 0; i < 19; i = i + 1) begin px[i] = 0; py[i] = 0; end

    // ---- line drawer ----
    reg  ln_start = 1'b0;
    wire ln_done, ln_we;
    wire [7:0] ln_x, ln_y;
    brush_line u_line (
        .clk(clk), .start(ln_start),
        .x0(px[b_par]), .y0(py[b_par]), .x1(ox), .y1(oy), .r(b_r),
        .done(ln_done), .we(ln_we), .wx(ln_x), .wy(ln_y)
    );

    always @(posedge clk) begin
        done     <= 1'b0;
        ln_start <= 1'b0;
        fb_we    <= 1'b0;
        case (state)
            S_IDLE:
                if (start) begin
                    pg <= page; mir <= mirror; pa <= pose_a; pb <= pose_b; phr <= ph;
                    hx <= hip_x; hy <= hip_y;
                    caddr <= 16'd0;
                    state <= S_CLEAR;
                end
            S_CLEAR: begin
                fb_we   <= 1'b1;
                fb_din  <= 1'b0;
                fb_addr <= {pg, caddr};
                caddr   <= caddr + 16'd1;
                if (caddr == 16'hFFFF) begin
                    field <= 4'd10;              // dy
                    state <= S_DYW;
                end
            end
            S_DYW:  state <= S_DY;
            S_DY: begin
                px[0] <= hx;
                py[0] <= hy + {{2{lerp_q[7]}}, lerp_q};
                field <= 4'd11;                  // flare
                state <= S_FLW;
            end
            S_FLW:  state <= S_FLARE;
            S_FLARE: begin
                spread <= 5'd7 + lerp_q[5:1];
                bi    <= 5'd0;
                state <= S_B0;
            end
            S_B0: begin                          // bone fields valid; select pose field
                field <= (b_var == 4'd0) ? 4'd15 : b_var - 4'd1;
                state <= S_B0W;
            end
            S_B0W:  state <= S_B1;
            S_B1: begin                          // interpolated variable
                avar  <= (b_var == 4'd0) ? 8'd0 : lerp_q;
                state <= S_B2;
            end
            S_B2: begin                          // final angle
                ang   <= mir ? 8'd128 - ang_raw : ang_raw;
                state <= S_B3W;
            end
            S_B3W:  state <= S_B3;               // ddx_q/ddy_q settle
            S_B3: begin                          // end point
                ox <= px[b_par] + (ddx_q >>> 7);
                oy <= py[b_par] + (ddy_q >>> 7);
                state <= S_B4;
            end
            S_B4: begin
                px[b_out] <= ox;
                py[b_out] <= oy;
                ln_start  <= 1'b1;               // r = 0 finishes immediately
                state     <= S_LINE;
            end
            S_LINE:
                if (ln_done) begin
                    if (bi == NBONES - 5'd1) begin
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        bi    <= bi + 5'd1;
                        state <= S_B0;
                    end
                end
            default: state <= S_IDLE;
        endcase
        if (ln_we) begin
            fb_we   <= 1'b1;
            fb_din  <= 1'b1;
            fb_addr <= {pg, ln_y, ln_x};
        end
    end
endmodule
