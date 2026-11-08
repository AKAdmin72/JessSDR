// TFT wiring test for Tang Nano 20K + ILI9341 (pins = adc_test.cst).
// No framebuffer: every pixel is computed from (x, y, pattern) while
// streaming the whole 320x240 screen (~50 ms per frame at 27 MHz SPI).
// Patterns change every PAT_FRAMES frames (~1.5 s), or immediately on S1:
//   0  8 colour bars (W Y C G M R B K) + 1px white border + red/green/blue
//      corner squares (top-left red, top-right green, bottom-left blue)
//   1  red   2  green   3  blue   4  white
//   5  horizontal gradients R / G / B / grey (4 horizontal bands)
//   6  checkerboard 8x8 (black/white)
// S2 held = freeze on the current pattern.
module tft_test_top (
    input  wire clk,
    input  wire btn_s1,      // active-high
    input  wire btn_s2,      // active-high
    output wire tft_sck,
    output wire tft_mosi,
    output wire tft_cs,
    output wire tft_dc,
    output wire tft_rst
);
    localparam [5:0] PAT_FRAMES = 6'd30;
    localparam [2:0] PAT_LAST   = 3'd6;

    localparam [15:0] C_BLACK = 16'h0000, C_WHITE = 16'hFFFF,
                      C_RED   = 16'hF800, C_GREEN = 16'h07E0, C_BLUE  = 16'h001F,
                      C_YEL   = 16'hFFE0, C_CYAN  = 16'h07FF, C_MAG   = 16'hF81F;

    // ---- S1 edge detect (sync + ~10 ms debounce) ----
    reg [1:0]  s1_sync = 2'b00;
    reg [18:0] s1_cnt = 19'd0;
    reg        s1_state = 1'b0;
    reg        s1_state_d = 1'b0;
    always @(posedge clk) s1_sync <= {s1_sync[0], btn_s1};
    always @(posedge clk)
        if (s1_sync[1] == s1_state) s1_cnt <= 19'd0;
        else                        s1_cnt <= s1_cnt + 19'd1;
    always @(posedge clk)
        if (s1_cnt == 19'd270000) s1_state <= s1_sync[1];
    always @(posedge clk) s1_state_d <= s1_state;
    wire s1_press = s1_state & ~s1_state_d;

    reg [1:0] s2_sync = 2'b00;
    always @(posedge clk) s2_sync <= {s2_sync[0], btn_s2};

    // ---- driver ----
    reg        ili_start_init  = 1'b0;
    reg        ili_set_window  = 1'b0;
    reg        ili_pixel_valid = 1'b0;
    reg [15:0] ili_pixel_data  = 16'd0;
    wire       ili_init_done, ili_window_done, ili_pixel_ready;
    ili9341_driver u_ili (
        .clk(clk),
        .start_init(ili_start_init), .init_done(ili_init_done),
        .set_window(ili_set_window),
        .win_x0(9'd0), .win_x1(9'd319), .win_y0(8'd0), .win_y1(8'd239),
        .window_done(ili_window_done),
        .pixel_data(ili_pixel_data), .pixel_valid(ili_pixel_valid), .pixel_ready(ili_pixel_ready),
        .tft_sck(tft_sck), .tft_mosi(tft_mosi), .tft_cs(tft_cs), .tft_dc(tft_dc), .tft_rst(tft_rst)
    );

    // ---- pixel generator (combinational from x, y, pattern) ----
    reg [8:0] x = 9'd0;
    reg [7:0] y = 8'd0;
    reg [2:0] pat = 3'd0;

    wire [2:0] bar = (x < 9'd40)  ? 3'd0 : (x < 9'd80)  ? 3'd1 : (x < 9'd120) ? 3'd2 :
                     (x < 9'd160) ? 3'd3 : (x < 9'd200) ? 3'd4 : (x < 9'd240) ? 3'd5 :
                     (x < 9'd280) ? 3'd6 : 3'd7;
    reg [15:0] bar_col;
    always @(*)
        case (bar)
            3'd0: bar_col = C_WHITE;  3'd1: bar_col = C_YEL;
            3'd2: bar_col = C_CYAN;   3'd3: bar_col = C_GREEN;
            3'd4: bar_col = C_MAG;    3'd5: bar_col = C_RED;
            3'd6: bar_col = C_BLUE;   default: bar_col = C_BLACK;
        endcase

    wire border = (x == 9'd0) || (x == 9'd319) || (y == 8'd0) || (y == 8'd239);
    wire sq_l = (x >= 9'd8)   && (x < 9'd32);
    wire sq_r = (x >= 9'd288) && (x < 9'd312);
    wire sq_t = (y >= 8'd8)   && (y < 8'd32);
    wire sq_b = (y >= 8'd208) && (y < 8'd232);

    // 320 px -> 5-bit / 6-bit ramps (x/10 approximated by x*51/512)
    wire [14:0] xr = x * 6'd51;           // 0..16269
    wire [4:0]  r5 = xr[13:9];            // 0..31
    wire [5:0]  g6 = xr[13:8];            // 0..63
    wire [1:0]  band = (y < 8'd60) ? 2'd0 : (y < 8'd120) ? 2'd1 : (y < 8'd180) ? 2'd2 : 2'd3;

    reg [15:0] pix;
    always @(*)
        case (pat)
            3'd0: pix = border ? C_WHITE :
                        (sq_l && sq_t) ? C_RED :
                        (sq_r && sq_t) ? C_GREEN :
                        (sq_l && sq_b) ? C_BLUE : bar_col;
            3'd1: pix = C_RED;
            3'd2: pix = C_GREEN;
            3'd3: pix = C_BLUE;
            3'd4: pix = C_WHITE;
            3'd5: case (band)
                      2'd0: pix = {r5, 6'd0, 5'd0};
                      2'd1: pix = {5'd0, g6, 5'd0};
                      2'd2: pix = {5'd0, 6'd0, r5};
                      default: pix = {r5, g6, r5};
                  endcase
            default: pix = (x[3] ^ y[3]) ? C_WHITE : C_BLACK;
        endcase

    // ---- sequencer ----
    localparam S_INIT = 3'd0, S_INIT_WAIT = 3'd1, S_WIN = 3'd2, S_WIN_WAIT = 3'd3,
               S_PRESENT = 3'd4, S_PULSE = 3'd5, S_FRAME_END = 3'd6;
    reg [2:0] state = S_INIT;
    reg [5:0] frame_cnt = 6'd0;
    reg       skip_req = 1'b0;   // S1 pressed, advance at end of frame

    always @(posedge clk) begin
        ili_start_init  <= 1'b0;
        ili_set_window  <= 1'b0;
        ili_pixel_valid <= 1'b0;
        if (s1_press) skip_req <= 1'b1;
        case (state)
            S_INIT: begin
                ili_start_init <= 1'b1;
                state <= S_INIT_WAIT;
            end
            S_INIT_WAIT:
                if (ili_init_done) state <= S_WIN;
            // set_window is dropped unless the driver is in S_READY.
            S_WIN:
                if (ili_pixel_ready) begin
                    ili_set_window <= 1'b1;
                    state <= S_WIN_WAIT;
                end
            S_WIN_WAIT:
                if (ili_window_done) begin
                    x <= 9'd0;
                    y <= 8'd0;
                    state <= S_PRESENT;
                end
            S_PRESENT:
                if (ili_pixel_ready) begin
                    ili_pixel_data  <= pix;
                    ili_pixel_valid <= 1'b1;
                    state <= S_PULSE;
                end
            // one cycle gap: the driver drops pixel_ready the cycle after valid
            S_PULSE: begin
                if (x == 9'd319) begin
                    x <= 9'd0;
                    if (y == 8'd239) state <= S_FRAME_END;
                    else begin y <= y + 8'd1; state <= S_PRESENT; end
                end else begin
                    x <= x + 9'd1;
                    state <= S_PRESENT;
                end
            end
            S_FRAME_END: begin
                if (skip_req || (frame_cnt == PAT_FRAMES - 6'd1 && !s2_sync[1])) begin
                    frame_cnt <= 6'd0;
                    skip_req  <= 1'b0;
                    pat <= (pat == PAT_LAST) ? 3'd0 : pat + 3'd1;
                end else if (frame_cnt != PAT_FRAMES - 6'd1)
                    frame_cnt <= frame_cnt + 6'd1;
                state <= S_WIN;
            end
            default: state <= S_INIT;
        endcase
    end
endmodule
