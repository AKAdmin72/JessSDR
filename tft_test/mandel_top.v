// Mandelbrot zoom on the ILI9341 (Tang Nano 20K, pins = tft_test.cst).
//
// Everything runs on one PLL clock (CLK_KHZ); the SPI SCK equals that clock
// (1 bit/clk, see ili9341_driver.v), so the PLL frequency sets both the
// pixel rate and the compute rate. The build script generates a small
// wrapper (build_out/mandel<MHz>_wrap.v) that picks the PLL settings.
//
// No framebuffer. NENG mandel_engine instances work in parallel: pixels are
// handed out round-robin (pixel n -> engine n mod NENG) and sent to the
// panel in the same round-robin order, so the output stays in raster order
// while slow (deep) pixels overlap with fast ones and with the SPI transfer.
//
// The view zooms in towards a boundary point (x1.032 per frame, ~2^20 deep,
// limited by the Q4.32 precision), then back out, forever. The palette
// rotates every frame.
//   S1 = next target point     S2 held = pause the zoom
module mandel_top #(
    parameter integer CLK_KHZ   = 27000,
    parameter integer PLL_IDIV  = 0,      // rPLL settings, CLKOUT = 27*(FBDIV+1)/(IDIV+1)
    parameter integer PLL_FBDIV = 0,
    parameter integer PLL_ODIV  = 32,
    parameter         PLL_BYPASS = "true"
) (
    input  wire clk_in,
    input  wire btn_s1,
    input  wire btn_s2,
    output wire tft_sck,
    output wire tft_mosi,
    output wire tft_cs,
    output wire tft_dc,
    output wire tft_rst
);
    localparam integer NENG = 4;
    localparam signed [35:0] STEP_MAX = 36'sd42949673;  // 0.01 per pixel = 3.2 wide (Q4.32)
    localparam signed [35:0] STEP_MIN = 36'sd24;        // ~5.6e-9 per pixel
    localparam [16:0] NPIX = 17'd76800;

    // ---- clock ----
    wire clk, pll_lock;
    rPLL u_pll (
        .CLKOUT(clk), .LOCK(pll_lock), .CLKOUTP(), .CLKOUTD(), .CLKOUTD3(),
        .RESET(1'b0), .RESET_P(1'b0), .CLKIN(clk_in), .CLKFB(1'b0),
        .FBDSEL(6'd0), .IDSEL(6'd0), .ODSEL(6'd0),
        .PSDA(4'd0), .DUTYDA(4'd0), .FDLY(4'd0)
    );
    defparam u_pll.FCLKIN        = "27";
    defparam u_pll.DYN_IDIV_SEL  = "false";
    defparam u_pll.IDIV_SEL      = PLL_IDIV;
    defparam u_pll.DYN_FBDIV_SEL = "false";
    defparam u_pll.FBDIV_SEL     = PLL_FBDIV;
    defparam u_pll.DYN_ODIV_SEL  = "false";
    defparam u_pll.ODIV_SEL      = PLL_ODIV;
    defparam u_pll.PSDA_SEL      = "0000";
    defparam u_pll.DYN_DA_EN     = "true";
    defparam u_pll.DUTYDA_SEL    = "1000";
    defparam u_pll.CLKOUT_FT_DIR = 1'b1;
    defparam u_pll.CLKOUTP_FT_DIR = 1'b1;
    defparam u_pll.CLKOUT_DLY_STEP = 0;
    defparam u_pll.CLKOUTP_DLY_STEP = 0;
    defparam u_pll.CLKFB_SEL     = "internal";
    defparam u_pll.CLKOUT_BYPASS = PLL_BYPASS;
    defparam u_pll.CLKOUTP_BYPASS = "false";
    defparam u_pll.CLKOUTD_BYPASS = "false";
    defparam u_pll.DYN_SDIV_SEL  = 2;
    defparam u_pll.CLKOUTD_SRC   = "CLKOUT";
    defparam u_pll.CLKOUTD3_SRC  = "CLKOUT";
    defparam u_pll.DEVICE        = "GW2A-18C";

    // ---- buttons ----
    reg [1:0]  s1_sync = 2'b00;
    reg [20:0] s1_cnt = 21'd0;
    reg        s1_state = 1'b0, s1_state_d = 1'b0;
    always @(posedge clk) s1_sync <= {s1_sync[0], btn_s1};
    always @(posedge clk)
        if (s1_sync[1] == s1_state) s1_cnt <= 21'd0;
        else                        s1_cnt <= s1_cnt + 21'd1;
    always @(posedge clk)
        if (s1_cnt == 10 * CLK_KHZ) s1_state <= s1_sync[1];    // 10 ms stable
    always @(posedge clk) s1_state_d <= s1_state;
    wire s1_press = s1_state & ~s1_state_d;
    reg [1:0] s2_sync = 2'b00;
    always @(posedge clk) s2_sync <= {s2_sync[0], btn_s2};

    // ---- targets (Q4.32) ----
    reg  [1:0] target = 2'd0;
    // (wires, not always @*: iverilog never evaluates an always @* whose
    // inputs don't change after time 0)
    wire signed [35:0] cx = (target == 2'd0) ? -36'sd3193926175 :  // seahorse valley
                            (target == 2'd1) ? -36'sd434205576  :  // Misiurewicz point -0.1011+0.9563i
                            (target == 2'd2) ? -36'sd3331536424 :  // spiral -0.7757+0.1365i
                                                36'sd7059732;      // 0.00164-0.82247i
    wire signed [35:0] cy = (target == 2'd0) ?  36'sd566187947  :
                            (target == 2'd1) ?  36'sd4107219290 :
                            (target == 2'd2) ?  36'sd586122891  :
                                               -36'sd3532471587;

    // ---- driver ----
    reg        ili_start_init  = 1'b0;
    reg        ili_set_window  = 1'b0;
    reg        ili_pixel_valid = 1'b0;
    reg [15:0] ili_pixel_data  = 16'd0;
    wire       ili_init_done, ili_window_done, ili_pixel_ready;
    ili9341_driver #(.CLK_KHZ(CLK_KHZ)) u_ili (
        .clk(clk),
        .start_init(ili_start_init), .init_done(ili_init_done),
        .set_window(ili_set_window),
        .win_x0(9'd0), .win_x1(9'd319), .win_y0(8'd0), .win_y1(8'd239),
        .window_done(ili_window_done),
        .pixel_data(ili_pixel_data), .pixel_valid(ili_pixel_valid), .pixel_ready(ili_pixel_ready),
        .tft_sck(tft_sck), .tft_mosi(tft_mosi), .tft_cs(tft_cs), .tft_dc(tft_dc), .tft_rst(tft_rst)
    );

    // ---- engines ----
    wire [NENG-1:0]   e_start;                          // = do_disp one-hot (see below)
    reg signed [35:0] d_cr = 36'sd0, d_ci = 36'sd0;     // coordinates of the pixel being dispatched
    reg  [9:0]        maxit = 10'd64;
    wire [NENG-1:0]   e_done, e_in;
    wire [10*NENG-1:0] e_iter;
    genvar g;
    generate
        for (g = 0; g < NENG; g = g + 1) begin : eng
            mandel_engine u_eng (
                .clk(clk), .start(e_start[g]), .cr(d_cr), .ci(d_ci), .maxit(maxit),
                .busy(), .done(e_done[g]), .iter(e_iter[10*g +: 10]), .in_set(e_in[g])
            );
        end
    endgenerate

    // ---- palette: rotating rainbow from triangle waves ----
    reg  [1:0] out_k = 2'd0;                 // engine whose result goes out next
    reg  [7:0] pal_off = 8'd0;
    function [7:0] tri8(input [7:0] v);
        tri8 = v[7] ? {~v[6:0], 1'b0} : {v[6:0], 1'b0};
    endfunction
    wire [9:0] o_iter = e_iter[10*out_k +: 10];
    wire [7:0] t  = {o_iter[5:0], 2'b00} + pal_off;
    wire [7:0] pr = tri8(t);
    wire [7:0] pg = tri8(t + 8'd85);
    wire [7:0] pb = tri8(t + 8'd170);
    wire [15:0] color = e_in[out_k] ? 16'h0000 : {pr[7:3], pg[7:2], pb[7:3]};

    // ---- view / scheduling state ----
    reg signed [35:0] step = STEP_MAX;
    reg signed [35:0] cr_left = 36'sd0;
    reg               zoom_in = 1'b1;
    reg  [8:0]        zlevel = 9'd0;
    reg               next_target = 1'b0;

    reg  [8:0]      d_x = 9'd0;              // dispatcher: column of d_cr
    reg  [16:0]     d_cnt = 17'd0;           // pixels handed out this frame
    reg  [1:0]      d_k = 2'd0;              // next engine to receive a pixel
    reg  [NENG-1:0] assigned = {NENG{1'b0}}; // engine holds a pixel not yet sent
    reg  [NENG-1:0] ready = {NENG{1'b0}};    // engine result available
    reg  [16:0]     o_cnt = 17'd0;           // pixels sent this frame
    reg  [1:0]      o_gap = 2'd0;            // holdoff after pixel_valid

    localparam S_RESET = 3'd0, S_INIT = 3'd1, S_INIT_WAIT = 3'd2, S_WIN = 3'd3,
               S_WIN_WAIT = 3'd4, S_RUN = 3'd5, S_FRAME_END = 3'd6;
    reg [2:0] state = S_RESET;

    wire signed [35:0] half_w = (step <<< 7) + (step <<< 5);   // 160*step
    wire signed [35:0] half_h = (step <<< 7) - (step <<< 3);   // 120*step

    wire dispatching = (state == S_WIN_WAIT || state == S_RUN) && (d_cnt != NPIX);
    wire do_disp = dispatching && !assigned[d_k];
    // combinational start: the engine latches d_cr/d_ci on the same edge
    // where the dispatcher advances them to the next pixel
    assign e_start = do_disp ? ({{(NENG-1){1'b0}}, 1'b1} << d_k) : {NENG{1'b0}};
    wire do_send = (state == S_RUN) && ready[out_k] && ili_pixel_ready && (o_gap == 2'd0);

    wire [9:0] maxit_next = (zlevel > 9'd468) ? 10'd1000 : 10'd64 + {zlevel, 1'b0};

    integer k;
    always @(posedge clk) begin
        ili_start_init  <= 1'b0;
        ili_set_window  <= 1'b0;
        ili_pixel_valid <= 1'b0;
        if (o_gap != 2'd0) o_gap <= o_gap - 2'd1;
        if (s1_press) next_target <= 1'b1;

        // engine bookkeeping (bitwise: set on done / dispatch, clear on send)
        for (k = 0; k < NENG; k = k + 1) begin
            if (e_done[k]) ready[k] <= 1'b1;
            if (do_send && out_k == k) begin
                ready[k]    <= 1'b0;
                assigned[k] <= 1'b0;
            end
            if (do_disp && d_k == k) assigned[k] <= 1'b1;
        end

        // dispatcher
        if (do_disp) begin
            d_k   <= d_k + 2'd1;
            d_cnt <= d_cnt + 17'd1;
        end

        // output
        if (do_send) begin
            ili_pixel_data  <= color;
            ili_pixel_valid <= 1'b1;
            o_gap <= 2'd2;
            out_k <= out_k + 2'd1;
            o_cnt <= o_cnt + 17'd1;
        end

        case (state)
            S_RESET:
                if (pll_lock) state <= S_INIT;
            S_INIT: begin
                ili_start_init <= 1'b1;
                state <= S_INIT_WAIT;
            end
            S_INIT_WAIT:
                if (ili_init_done) state <= S_WIN;
            S_WIN:
                if (ili_pixel_ready && o_gap == 2'd0) begin   // set_window only accepted in S_READY
                    ili_set_window <= 1'b1;
                    cr_left <= cx - half_w;
                    d_cr    <= cx - half_w;
                    d_ci    <= cy + half_h;
                    d_x     <= 9'd0;
                    d_cnt   <= 17'd0;
                    o_cnt   <= 17'd0;
                    state   <= S_WIN_WAIT;
                end
            S_WIN_WAIT:
                if (ili_window_done) state <= S_RUN;
            S_RUN:
                if (o_cnt == NPIX) state <= S_FRAME_END;
            S_FRAME_END: begin
                pal_off <= pal_off + 8'd2;
                if (next_target) begin
                    next_target <= 1'b0;
                    target  <= target + 2'd1;
                    step    <= STEP_MAX;
                    zoom_in <= 1'b1;
                    zlevel  <= 9'd0;
                    maxit   <= 10'd64;
                end else if (!s2_sync[1]) begin
                    maxit <= maxit_next;
                    if (zoom_in) begin
                        if (step <= STEP_MIN) zoom_in <= 1'b0;
                        else begin
                            step   <= step - (step >>> 5);
                            zlevel <= zlevel + 9'd1;
                        end
                    end else begin
                        if (step >= STEP_MAX) zoom_in <= 1'b1;
                        else begin
                            step   <= step + (step >>> 5) + 36'sd1;
                            zlevel <= (zlevel == 9'd0) ? 9'd0 : zlevel - 9'd1;
                        end
                    end
                end
                state <= S_WIN;
            end
            default: state <= S_RESET;
        endcase

        // coordinate advance for the dispatched pixel (after the case so the
        // S_WIN frame-start assignment above is not overridden: do_disp is
        // never true in S_WIN)
        if (do_disp) begin
            if (d_x == 9'd319) begin
                d_x  <= 9'd0;
                d_cr <= cr_left;
                d_ci <= d_ci - step;
            end else begin
                d_x  <= d_x + 9'd1;
                d_cr <= d_cr + step;
            end
        end
    end
endmodule
