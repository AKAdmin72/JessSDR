// ZX Spectrum style demo on the ILI9341 (Tang Nano 20K, TFT pins as in
// ../adc_test/adc_test.cst). 320x240 panel = 256x192 "Spectrum screen" with a
// 32/24 px border around it, Spectrum 8-colour palette (normal/bright).
//
//   rows 0-1    title, double-height font, rainbow ink
//   rows 2-18   attribute plasma (paper) + black-ink dancer from the
//               framebuffer; fake equalizer bars at columns 1-3 / 28-30
//   rows 19-21  checkerboard floor flipping on the beat
//   rows 22-23  smooth scroller, double-height font, rainbow ink
//   border      loading stripes during the intro, then flashes on the
//               virtual kick (chord colour) / snare (white)
//
// The dancer is drawn by dancer.v into the back page of a double-buffered
// 1bpp framebuffer (2 x 256x256 bits in BSRAM) at every frame end, then
// the pages swap. No CPU; everything is driven by beat_gen (125 BPM).
module zx_demo_top #(
    parameter integer CLK_KHZ   = 67500,
    parameter integer PLL_IDIV  = 1,      // 27*(4+1)/(1+1) = 67.5 MHz, VCO 540
    parameter integer PLL_FBDIV = 4,
    parameter integer PLL_ODIV  = 8
) (
    input  wire clk,
    output wire tft_sck,
    output wire tft_mosi,
    output wire tft_cs,
    output wire tft_dc,
    output wire tft_rst
);
    // ---------------- clock ----------------
    wire sclk, pll_lock;
    rPLL u_pll (
        .CLKOUT(sclk), .LOCK(pll_lock), .CLKOUTP(), .CLKOUTD(), .CLKOUTD3(),
        .RESET(1'b0), .RESET_P(1'b0), .CLKIN(clk), .CLKFB(1'b0),
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
    defparam u_pll.CLKOUT_BYPASS = "false";
    defparam u_pll.CLKOUTP_BYPASS = "false";
    defparam u_pll.CLKOUTD_BYPASS = "false";
    defparam u_pll.DYN_SDIV_SEL  = 2;
    defparam u_pll.CLKOUTD_SRC   = "CLKOUT";
    defparam u_pll.CLKOUTD3_SRC  = "CLKOUT";
    defparam u_pll.DEVICE        = "GW2A-18C";

    // ---------------- beat ----------------
    wire [5:0] step;
    wire [2:0] tick, ptype;
    wire       intro, kick_hit, snare_hit;
    wire [3:0] vu0, vu1, vu2;
    beat_gen #(.CLK_KHZ(CLK_KHZ)) u_beat (
        .clk(sclk), .step(step), .tick(tick), .ptype(ptype), .intro(intro),
        .vu0(vu0), .vu1(vu1), .vu2(vu2), .kick_hit(kick_hit), .snare_hit(snare_hit)
    );

    // ---------------- TFT driver ----------------
    reg        ili_start_init  = 1'b0;
    reg        ili_set_window  = 1'b0;
    reg        ili_pixel_valid = 1'b0;
    reg [15:0] ili_pixel_data  = 16'd0;
    wire       ili_init_done, ili_window_done, ili_pixel_ready;
    ili9341_driver #(.CLK_KHZ(CLK_KHZ)) u_ili (
        .clk(sclk),
        .start_init(ili_start_init), .init_done(ili_init_done),
        .set_window(ili_set_window),
        .win_x0(9'd0), .win_x1(9'd319), .win_y0(8'd0), .win_y1(8'd239),
        .window_done(ili_window_done),
        .pixel_data(ili_pixel_data), .pixel_valid(ili_pixel_valid), .pixel_ready(ili_pixel_ready),
        .tft_sck(tft_sck), .tft_mosi(tft_mosi), .tft_cs(tft_cs), .tft_dc(tft_dc), .tft_rst(tft_rst)
    );

    // ---------------- framebuffer + dancer ----------------
    reg         front = 1'b0;            // page being displayed
    reg  [16:0] fb_raddr = 17'd0;
    wire        fb_dout;
    wire        d_we, d_din, d_done;
    wire [16:0] d_addr;
    fb_ram u_fb (.clk(sclk), .we(d_we), .waddr(d_addr), .din(d_din), .raddr(fb_raddr), .dout(fb_dout));

    // per-frame dance parameters (from the beat state at frame end)
    wire [4:0] raw_ph = {2'b00, step[1:0], 1'b0} + {1'b0, step[1:0], 2'b00} + {2'b00, tick};   // step%4*6 + tick, 0..23
    wire [7:0] ph     = (raw_ph >= 5'd12) ? 8'd255 : raw_ph * 8'd21;
    wire [1:0] sec = (ptype == 3'd2) ? 2'd1 : (ptype == 3'd3 || ptype == 3'd5) ? 2'd2 :
                     (ptype == 3'd4) ? 2'd3 : 2'd0;       // dance section
    wire [3:0] pose_b, pose_a;
    dance_rom u_dr_b (.a({sec, step[4:2]}),        .d(pose_b));
    dance_rom u_dr_a (.a({sec, step[4:2] - 3'd1}), .d(pose_a));
    wire [7:0] sway_s;
    sin_rom u_sway (.a({step, 2'b00}), .d(sway_s));
    wire signed [9:0] hip_x  = 10'sd128 + ($signed({{2{sway_s[7]}}, sway_s}) >>> 3);
    wire signed [9:0] hip_y  = 10'sd100 + ((raw_ph < 5'd6) ? (10'sd6 - $signed({5'd0, raw_ph})) : 10'sd0);

    reg d_start = 1'b0, d_ready = 1'b1;
    dancer u_dancer (
        .clk(sclk), .start(d_start), .page(~front),   // front already toggled when start is seen
        .pose_a(pose_a), .pose_b(pose_b), .ph(ph), .mirror(sec == 2'd2 && step[5]),
        .hip_x(hip_x), .hip_y(hip_y),
        .done(d_done), .fb_we(d_we), .fb_addr(d_addr), .fb_din(d_din)
    );
    always @(posedge sclk)
        if (d_start)     d_ready <= 1'b0;
        else if (d_done) d_ready <= 1'b1;

    // ---------------- frame-level state ----------------
    reg [15:0] fr = 16'd0;               // frame counter
    reg [7:0]  rnd = 8'h5A;              // per-frame random (loading stripes)
    reg [2:0]  kick_fl = 3'd0, snare_fl = 3'd0;
    reg        frame_end = 1'b0;
    always @(posedge sclk)
        if (kick_hit)       kick_fl <= 3'd4;
        else if (frame_end && kick_fl != 3'd0) kick_fl <= kick_fl - 3'd1;
    always @(posedge sclk)
        if (snare_hit)      snare_fl <= 3'd2;
        else if (frame_end && snare_fl != 3'd0) snare_fl <= snare_fl - 3'd1;

    // ---------------- pixel colour ----------------
    function [15:0] zx_rgb(input [2:0] c, input br);   // c = {G, R, B}
        zx_rgb = { c[1] ? (br ? 5'd31 : 5'd26) : 5'd0,
                   c[2] ? (br ? 6'd63 : 6'd53) : 6'd0,
                   c[0] ? (br ? 5'd31 : 5'd26) : 5'd0 };
    endfunction
    function [2:0] rainbow(input [2:0] v);             // never black
        rainbow = (v == 3'd0) ? 3'd7 : v;
    endfunction
    function [7:0] tri8(input [7:0] v);
        tri8 = v[7] ? ~v : v;
    endfunction
    function [2:0] chord_col(input [1:0] b);
        case (b)
            2'd0: chord_col = 3'd3;  2'd1: chord_col = 3'd5;
            2'd2: chord_col = 3'd6;  default: chord_col = 3'd4;
        endcase
    endfunction
    function [2:0] plasma_col(input [2:0] i);
        case (i)
            3'd0: plasma_col = 3'd1; 3'd1: plasma_col = 3'd3; 3'd2: plasma_col = 3'd2;
            3'd3: plasma_col = 3'd6; 3'd4: plasma_col = 3'd4; 3'd5: plasma_col = 3'd5;
            3'd6: plasma_col = 3'd7; default: plasma_col = 3'd5;
        endcase
    endfunction

    // pipeline registers
    reg [8:0]  x = 9'd0;
    reg [7:0]  y = 8'd0;
    reg        p_in = 1'b0;
    reg [7:0]  p_sx = 8'd0, p_sy = 8'd0;
    reg [11:0] p_tpos = 12'd0;
    reg [5:0]  p_code = 6'd0;
    reg [2:0]  p_col = 3'd0;
    reg [2:0]  q_c = 3'd0;
    reg        q_br = 1'b0;

    wire [4:0] cx = p_sx[7:3];
    wire [4:0] cy = p_sy[7:3];
    wire [5:0] t_code, s_code;
    title_rom u_title (.a(p_sx[7:3]),    .d(t_code));
    text_rom  u_text  (.a(p_tpos[11:3]), .d(s_code));
    wire [7:0] font_bits;
    font_rom  u_font  (.a({p_code, p_sy[3:1]}), .d(font_bits));
    wire       fbit = font_bits[3'd7 - p_col];

    // plasma on the attribute grid
    wire [7:0]  t8 = fr[7:0];
    wire [9:0]  psum = tri8({cx, 3'b000} + t8) + tri8({cy, 3'b000} + {cy, 2'b00} + {t8[6:0], 1'b0})
                     + tri8({cx + cy, 2'b00} + {cx + cy, 1'b0} - t8);
    // equalizer bars
    wire        vu_col = (cx >= 5'd1 && cx <= 5'd3) || (cx >= 5'd28 && cx <= 5'd30);
    wire [3:0]  vu_v   = (cx == 5'd1 || cx == 5'd30) ? vu0 : (cx == 5'd2 || cx == 5'd29) ? vu1 : vu2;
    wire [3:0]  vu_lvl = 5'd17 - cy;
    wire        vu_row = (cy >= 5'd3) && (cy <= 5'd17);
    wire        vu_lit = vu_v > vu_lvl;
    wire [2:0]  vu_c   = (vu_lvl >= 4'd11) ? 3'd2 : (vu_lvl >= 4'd7) ? 3'd6 : 3'd4;
    // border
    wire [7:0]  by8    = y + rnd;
    wire        stripe = step[5] ? by8[2] : by8[4];
    wire [2:0]  border_c = intro ? (step[5] ? (stripe ? 3'd1 : 3'd6) : (stripe ? 3'd2 : 3'd5))
                         : (snare_fl != 3'd0) ? 3'd7 : (kick_fl != 3'd0) ? chord_col(step[5:4]) : 3'd0;

    reg  [2:0] c_now;
    reg        br_now;
    always @(*) begin
        if (!p_in) begin
            c_now = border_c; br_now = 1'b1;
        end else if (cy < 5'd2) begin                       // title
            c_now = fbit ? rainbow(cx[2:0] + fr[5:3]) : 3'd0; br_now = 1'b1;
        end else if (cy >= 5'd22) begin                     // scroller
            c_now = fbit ? rainbow(cx[2:0] - fr[4:2]) : 3'd0; br_now = 1'b1;
        end else if (cy >= 5'd19) begin                     // floor
            c_now = fb_dout ? 3'd0 : (((cx[4:1] + cy + step[2]) & 1) ? 3'd7 : chord_col(step[5:4]));
            br_now = 1'b1;
        end else if (vu_col && vu_row) begin                // equalizer
            c_now = fb_dout ? 3'd0 : (vu_lit ? vu_c : 3'd0); br_now = 1'b1;
        end else begin                                      // stage
            c_now = fb_dout ? 3'd0 : plasma_col(psum[8:6]);
            br_now = psum[5] | (kick_fl != 3'd0);
        end
    end

    // ---------------- display sequencer ----------------
    localparam S_RESET = 4'd0, S_INIT = 4'd1, S_INIT_WAIT = 4'd2, S_WIN = 4'd3, S_WIN_WAIT = 4'd4,
               S_P0 = 4'd5, S_P1 = 4'd6, S_P2 = 4'd7, S_P3 = 4'd8, S_FRAME_END = 4'd9;
    reg [3:0] state = S_RESET;

    wire [8:0] sx9 = x - 9'd32;
    wire [7:0] sy8 = y - 8'd24;

    always @(posedge sclk) begin
        ili_start_init  <= 1'b0;
        ili_set_window  <= 1'b0;
        ili_pixel_valid <= 1'b0;
        d_start         <= 1'b0;
        frame_end       <= 1'b0;
        case (state)
            S_RESET:     if (pll_lock) state <= S_INIT;
            S_INIT:      begin ili_start_init <= 1'b1; state <= S_INIT_WAIT; end
            S_INIT_WAIT: if (ili_init_done) state <= S_WIN;
            S_WIN:
                if (ili_pixel_ready) begin                  // set_window only accepted in S_READY
                    ili_set_window <= 1'b1;
                    x <= 9'd0;
                    y <= 8'd0;
                    state <= S_WIN_WAIT;
                end
            S_WIN_WAIT:  if (ili_window_done) state <= S_P0;
            S_P0: begin                                     // coordinates, framebuffer read
                p_in     <= (x >= 9'd32) && (x < 9'd288) && (y >= 8'd24) && (y < 8'd216);
                p_sx     <= sx9[7:0];
                p_sy     <= sy8;
                p_tpos   <= {fr[10:0], 1'b0} + {4'd0, sx9[7:0]};
                fb_raddr <= {front, sy8, sx9[7:0]};
                state    <= S_P1;
            end
            S_P1: begin                                     // text ROMs
                p_code <= (p_sy[7:3] < 5'd2) ? t_code : s_code;
                p_col  <= (p_sy[7:3] < 5'd2) ? p_sx[2:0] : p_tpos[2:0];
                state  <= S_P2;
            end
            S_P2: begin                                     // font ROM, fb data, colour
                q_c   <= c_now;
                q_br  <= br_now;
                state <= S_P3;
            end
            S_P3:
                if (ili_pixel_ready) begin
                    ili_pixel_data  <= zx_rgb(q_c, q_br);
                    ili_pixel_valid <= 1'b1;
                    if (x == 9'd319) begin
                        x <= 9'd0;
                        if (y == 8'd239) state <= S_FRAME_END;
                        else begin y <= y + 8'd1; state <= S_P0; end
                    end else begin
                        x <= x + 9'd1;
                        state <= S_P0;
                    end
                end
            S_FRAME_END: begin
                fr        <= fr + 16'd1;
                rnd       <= {rnd[6:0], rnd[7] ^ rnd[5] ^ rnd[4] ^ rnd[3]};
                frame_end <= 1'b1;
                if (d_ready) begin
                    front   <= ~front;                      // show the page just drawn
                    d_start <= 1'b1;                        // draw the next one into the old front
                end
                state <= S_WIN;
            end
            default: state <= S_RESET;
        endcase
    end
endmodule
