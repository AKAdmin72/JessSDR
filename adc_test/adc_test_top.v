// ADC bring-up scope: AD9226 module -> adc_capture (13.5MSPS, decimated
// timebase) -> ring_buffer + trigger -> waveform_draw -> 1bpp framebuffer ->
// ILI9341 TFT. Derived from ../../oscilloscope/osc_top.v (Tang Nano 9K, I2S
// input) - only the sample source, the timebase buttons and the OTR colour
// are new; trigger/ring_buffer/waveform_draw/line_draw/ili9341_driver are
// copied verbatim from that project.
//
// Controls (onboard buttons, active-high):
//   S1 short - slower timebase (decimation x2), S2 short - faster (/2).
//   S1 long (>0.5 s) - vertical zoom in (x1/x4/x16/x64), S2 long - out;
//   zoom level = number of yellow squares bottom-left. At x1 one pixel is
//   32 ADC codes (full scale = +-64 px), at x64 one pixel is 0.5 code.
//   Power-on timebase: decimation 2^DECIM_INIT.
// Trace colour: cyan normally, RED if the ADC's OTR (out-of-range) flag was
// set on any sample of the displayed capture - input is clipping.
module adc_test_top #(
    parameter BIT_REVERSE      = 0,
    parameter [3:0] DECIM_INIT = 4'd4     // 13.5MSPS/16 = 844kSPS -> 379us per screen
) (
    input  wire        clk,               // 27MHz onboard oscillator
    input  wire        btn_s1,
    input  wire        btn_s2,

    output wire        adc_clk,
    input  wire [11:0] adc_d,
    input  wire        adc_otr,

    output wire        tft_sck,
    output wire        tft_mosi,
    output wire        tft_cs,
    output wire        tft_dc,
    output wire        tft_rst
);

    // ============================================================
    // buttons -> timebase select
    // ============================================================
    reg [3:0] decim_sel = DECIM_INIT;
    reg [1:0] s1_sync = 2'b00, s2_sync = 2'b00;
    always @(posedge clk) s1_sync <= {s1_sync[0], btn_s1};
    always @(posedge clk) s2_sync <= {s2_sync[0], btn_s2};

    // ~10ms debounce tick: sample the synchronized buttons every 2^18 clk
    reg [17:0] deb_div = 18'd0;
    always @(posedge clk) deb_div <= deb_div + 18'd1;
    wire deb_tick = (deb_div == 18'd0);
    reg s1_st = 1'b0, s2_st = 1'b0;   // debounced levels
    always @(posedge clk) if (deb_tick) s1_st <= s1_sync[1];
    always @(posedge clk) if (deb_tick) s2_st <= s2_sync[1];
    reg s1_prev = 1'b0, s2_prev = 1'b0;
    always @(posedge clk) s1_prev <= s1_st;
    always @(posedge clk) s2_prev <= s2_st;
    // hold time in debounce ticks (~9.7 ms each); >= HOLD_LONG = long press
    localparam [6:0] HOLD_LONG = 7'd50;
    reg [6:0] s1_hold = 7'd0, s2_hold = 7'd0;
    always @(posedge clk)
        if (deb_tick) s1_hold <= !s1_st ? 7'd0 : (s1_hold == 7'd127) ? s1_hold : s1_hold + 7'd1;
    always @(posedge clk)
        if (deb_tick) s2_hold <= !s2_st ? 7'd0 : (s2_hold == 7'd127) ? s2_hold : s2_hold + 7'd1;
    wire s1_short = !s1_st && s1_prev && (s1_hold < HOLD_LONG);   // on release
    wire s2_short = !s2_st && s2_prev && (s2_hold < HOLD_LONG);
    wire s1_long  = deb_tick && s1_st && (s1_hold == HOLD_LONG - 7'd1);   // once, while held
    wire s2_long  = deb_tick && s2_st && (s2_hold == HOLD_LONG - 7'd1);
    // short press: timebase (S1 slower, S2 faster)
    always @(posedge clk) begin
        if (s1_short && decim_sel != 4'd15)
            decim_sel <= decim_sel + 4'd1;
        else if (s2_short && decim_sel != 4'd0)
            decim_sel <= decim_sel - 4'd1;
    end
    // long press: vertical zoom x1 / x4 / x16 / x64 (S1 in, S2 out)
    reg [1:0] zoom = 2'd0;
    always @(posedge clk)
        if (s1_long && zoom != 2'd3)      zoom <= zoom + 2'd1;
        else if (s2_long && zoom != 2'd0) zoom <= zoom - 2'd1;

    // ============================================================
    // ADC capture
    // ============================================================
    wire signed [17:0] adc_sample;
    wire        [11:0] adc_raw;
    wire               adc_valid;
    wire               adc_sample_otr;
    adc_capture #(.BIT_REVERSE(BIT_REVERSE)) u_adc (
        .clk(clk),
        .adc_clk(adc_clk), .adc_d(adc_d), .adc_otr(adc_otr),
        .decim_sel(decim_sel),
        .sample(adc_sample), .valid(adc_valid), .otr(adc_sample_otr),
        .raw(adc_raw)
    );

    // ============================================================
    // bit-activity monitor (bring-up aid): OR / AND of every raw ADC word
    // since the last screen refresh. Shown as 12 boxes at the top, MSB
    // (D11) left: green = toggling, white = stuck 1, dark blue = stuck 0.
    // A stuck or swapped wire shows immediately; with no input signal only
    // the low bits toggle (noise around mid-scale 2048, so D11 toggles too).
    // ============================================================
    reg [11:0] act_or = 12'h000, act_and = 12'hFFF;
    reg [11:0] show_or = 12'h000, show_and = 12'hFFF;
    wire       act_latch;                  // = start of a screen refresh (below)
    always @(posedge clk)
        if (act_latch) act_or <= 12'h000;
        else           act_or <= act_or | adc_raw;
    always @(posedge clk)
        if (act_latch) act_and <= 12'hFFF;
        else           act_and <= act_and & adc_raw;
    always @(posedge clk) if (act_latch) show_or  <= act_or;
    always @(posedge clk) if (act_latch) show_and <= act_and;

    // vertical zoom: sample << 2*zoom, saturated to the 18-bit range
    // (sample = code<<6, so the low bits are zero and the shift is exact)
    wire signed [24:0] zs = $signed({{7{adc_sample[17]}}, adc_sample}) <<< {zoom, 1'b0};
    wire signed [17:0] adc_scaled = (zs > 25'sd131071)  ?  18'sd131071 :
                                    (zs < -25'sd131072) ? -18'sd131072 : zs[17:0];

    // ============================================================
    // trigger (write side of the ring buffer)
    // ============================================================
    reg  trig_rearm = 1'b0;
    wire [8:0] trig_wptr;
    wire       trig_ring_we;
    wire [1:0] trig_state;
    wire       trig_capture_done;
    // THRESHOLD 0 = rising zero crossing (mid-scale); AUTO_TIMEOUT so a DC
    // or very slow input still redraws (free-runs) instead of freezing.
    trigger #(.THRESHOLD(18'sd0), .AUTO_TIMEOUT(16'd20000)) u_trigger (
        .clk(clk),
        .i_sample(adc_scaled), .i_valid(adc_valid),
        .rearm(trig_rearm),
        .wptr(trig_wptr), .ring_we(trig_ring_we),
        .trig_addr(), .capture_done(trig_capture_done), .state(trig_state)
    );

    // ============================================================
    // waveform draw stage (read side of the ring buffer)
    // ============================================================
    reg  wd_start = 1'b0;
    wire wd_busy, wd_done;
    reg  [8:0] wd_read_base;
    wire [8:0] wd_ring_raddr;
    wire signed [17:0] ring_dout;
    wire        ld_start_wd, ld_busy_wd, ld_done_wd;
    wire signed [9:0] ld_x0, ld_y0, ld_x1, ld_y1;

    waveform_draw u_waveform_draw (
        .clk(clk),
        .start(wd_start), .busy(wd_busy), .done(wd_done),
        .read_base(wd_read_base),
        .ring_raddr(wd_ring_raddr), .ring_rdata(ring_dout),
        .ld_start(ld_start_wd), .ld_x0(ld_x0), .ld_y0(ld_y0), .ld_x1(ld_x1), .ld_y1(ld_y1),
        .ld_busy(ld_busy_wd), .ld_done(ld_done_wd)
    );

    // ---- ring_buffer: time-muxed between trigger.v (write) and
    // waveform_draw.v (read) - never concurrent by construction: trigger
    // only sits in S_CAPTURED (ring_we forced low) while waveform_draw
    // reads, and rearm only fires (S_REARM below) after waveform_draw.done.
    wire trig_owns_ring = (trig_state != 2'd2); // 2'd2 == trigger.S_CAPTURED
    wire [8:0] ring_addr = trig_owns_ring ? trig_wptr    : wd_ring_raddr;
    wire        ring_we   = trig_owns_ring ? trig_ring_we : 1'b0;
    ring_buffer u_ring (
        .clk(clk), .addr(ring_addr), .we(ring_we), .din(adc_scaled), .dout(ring_dout)
    );

    // ============================================================
    // shared line_draw instance
    // ============================================================
    wire [16:0] ld_fb_addr;
    wire        ld_fb_we;
    line_draw u_ld (
        .clk(clk), .start(ld_start_wd),
        .x0(ld_x0), .y0(ld_y0), .x1(ld_x1), .y1(ld_y1),
        .busy(ld_busy_wd), .done(ld_done_wd),
        .fb_addr(ld_fb_addr), .fb_we(ld_fb_we)
    );

    // ---- waveform_framebuffer: muxed between line_draw's draw output
    // (while osc_top is in S_DRAW_WAIT) and the top-level clear/stream
    // readback logic otherwise - same idiom as stage1_top.v's trace_fb mux.
    reg  [16:0] fb_addr_top;
    reg          fb_we_top = 1'b0;
    reg          fb_din_top;
    wire [16:0] fb_addr = (state == S_DRAW_WAIT) ? ld_fb_addr : fb_addr_top;
    wire         fb_we   = (state == S_DRAW_WAIT) ? ld_fb_we   : fb_we_top;
    wire         fb_din  = (state == S_DRAW_WAIT) ? 1'b1       : fb_din_top;
    wire         fb_dout;
    waveform_framebuffer u_fb (
        .clk(clk), .addr(fb_addr), .we(fb_we), .din(fb_din), .dout(fb_dout)
    );

    // ============================================================
    // ILI9341 driver
    // ============================================================
    reg  ili_start_init  = 1'b0;
    reg  ili_set_window  = 1'b0;
    reg  ili_pixel_valid = 1'b0;
    wire ili_init_done, ili_window_done, ili_pixel_ready;
    reg  [15:0] ili_pixel_data;
    reg  [8:0]  win_x0, win_x1;
    reg  [7:0]  win_y0, win_y1;
    ili9341_driver u_ili (
        .clk(clk),
        .start_init(ili_start_init), .init_done(ili_init_done),
        .set_window(ili_set_window), .win_x0(win_x0), .win_x1(win_x1), .win_y0(win_y0), .win_y1(win_y1),
        .window_done(ili_window_done),
        .pixel_data(ili_pixel_data), .pixel_valid(ili_pixel_valid), .pixel_ready(ili_pixel_ready),
        .tft_sck(tft_sck), .tft_mosi(tft_mosi), .tft_cs(tft_cs), .tft_dc(tft_dc), .tft_rst(tft_rst)
    );

    // ============================================================
    // main FSM
    // ============================================================
    localparam [3:0]
        S_INIT_SETUP     = 4'd0,
        S_INIT_WAIT      = 4'd1,
        S_FBCLR          = 4'd2,
        S_DRAW_SETUP     = 4'd3,
        S_DRAW_WAIT      = 4'd4,
        S_REARM          = 4'd5,
        S_WINDOW_SETUP   = 4'd6,
        S_WINDOW_WAIT    = 4'd7,
        S_STREAM_FETCH   = 4'd8,
        S_STREAM_FETCH2  = 4'd9,
        S_STREAM_PRESENT = 4'd10,
        S_STREAM_PULSE   = 4'd11,
        S_WAIT_TRIGGER   = 4'd12;

    reg [3:0] state        = S_INIT_SETUP;
    reg       boot_pending = 1'b1; // first pass paints a blank+grid frame, no capture/draw yet

    localparam [15:0] COLOR_BLACK = 16'h0000;
    localparam [15:0] COLOR_TRACE = 16'h07FF; // cyan
    localparam [15:0] COLOR_CLIP  = 16'hF800; // red - OTR seen in this capture
    localparam [15:0] COLOR_GRID  = 16'h8410; // dim grey, recedes behind the trace

    // OTR seen while the ring buffer is being written (cleared on rearm);
    // frozen into otr_frame when a capture is handed to the drawing stage.
    reg otr_seen  = 1'b0;
    reg otr_frame = 1'b0;
    always @(posedge clk) begin
        if (trig_rearm)
            otr_seen <= 1'b0;
        else if (adc_valid && adc_sample_otr && trig_state != 2'd2)
            otr_seen <= 1'b1;
    end
    always @(posedge clk)
        if (state == S_DRAW_SETUP)
            otr_frame <= otr_seen;

    reg [16:0] fbclr_addr;
    reg [16:0] stream_idx;
    reg [8:0]  stream_x;
    reg [7:0]  stream_y;

    // Grid overlay: 1 horizontal center line (bipolar zero reference) + 4
    // more horizontal division lines + 5 vertical division lines, OR'd
    // into the streamed pixel alongside fb_dout - zero extra framebuffer
    // or line_draw calls, since these positions never change. x=80 is
    // always the trigger-instant column (see trigger.v's addressing
    // proof), so that vertical line doubles as a free visual trigger
    // marker at no extra cost.
    assign act_latch = (state == S_WINDOW_WAIT) && ili_window_done;

    // bit boxes: rows 3..14, box i (0 = D11) at x 6+26*i .. +21
    localparam [15:0] COLOR_BIT_TOG = 16'h07E0;   // green
    localparam [15:0] COLOR_BIT_ONE = 16'hFFFF;   // white
    localparam [15:0] COLOR_BIT_ZERO = 16'h000C;  // dark blue
    wire [8:0] bx_off = stream_x - 9'd6;
    wire [3:0] bx_i   = bx_off / 9'd26;
    wire [4:0] bx_in  = bx_off - bx_i * 9'd26;
    wire       box_on = (stream_y >= 8'd3) && (stream_y <= 8'd14) && (stream_x >= 9'd6) &&
                        (bx_i < 4'd12) && (bx_in < 5'd22);
    wire [3:0] bit_n  = 4'd11 - bx_i;
    wire [15:0] box_color = show_and[bit_n] ? COLOR_BIT_ONE :
                            show_or[bit_n]  ? COLOR_BIT_TOG : COLOR_BIT_ZERO;
    // zoom indicator: zoom+1 yellow squares at the bottom left (rows 228..234)
    wire       zoom_on = (stream_y >= 8'd228) && (stream_y <= 8'd234) && (stream_x >= 9'd6) &&
                         (stream_x < 9'd6 + 9'd10 * ({7'd0, zoom} + 9'd1)) && ((stream_x - 9'd6) % 9'd10 < 9'd7);

    wire grid_on = (stream_x==9'd0)||(stream_x==9'd80)||(stream_x==9'd160)||(stream_x==9'd240)||(stream_x==9'd319) ||
                   (stream_y==8'd0)||(stream_y==8'd60)||(stream_y==8'd120)||(stream_y==8'd180)||(stream_y==8'd239);

    always @(posedge clk) begin
        ili_start_init  <= 1'b0;
        ili_set_window  <= 1'b0;
        ili_pixel_valid <= 1'b0;
        fb_we_top       <= 1'b0;
        wd_start        <= 1'b0;
        trig_rearm      <= 1'b0;

        case (state)
            S_INIT_SETUP: begin
                ili_start_init <= 1'b1;
                state <= S_INIT_WAIT;
            end
            S_INIT_WAIT: begin
                if (ili_init_done) begin
                    fbclr_addr <= 17'd0;
                    state <= S_FBCLR;
                end
            end

            // Clears the framebuffer before every redraw (boot's initial
            // blank+grid frame, and every loop iteration before the next
            // polyline is drawn) - entered both from boot and from
            // S_WAIT_TRIGGER below.
            S_FBCLR: begin
                fb_addr_top <= fbclr_addr;
                fb_din_top  <= 1'b0;
                fb_we_top   <= 1'b1;
                if (fbclr_addr == 17'd76799) begin
                    if (boot_pending) begin
                        boot_pending <= 1'b0;
                        state <= S_WINDOW_SETUP; // boot: skip draw, stream the blank+grid frame
                    end else begin
                        state <= S_DRAW_SETUP;
                    end
                end else begin
                    fbclr_addr <= fbclr_addr + 17'd1;
                end
            end

            S_DRAW_SETUP: begin
                wd_read_base <= trig_wptr; // frozen (S_CAPTURED) - the oldest-sample/leftmost-pixel address
                wd_start     <= 1'b1;
                state        <= S_DRAW_WAIT;
            end
            S_DRAW_WAIT: begin
                if (wd_done)
                    state <= S_REARM;
            end
            S_REARM: begin
                // Rearm as soon as the framebuffer is filled, BEFORE the
                // slow ~1.3-1.5M-cycle (~50ms) SPI pixel stream below runs -
                // waiting until after the stream would leave the scope
                // blind to new triggers for that whole window (~2400
                // missed samples' worth at 48kHz) every redraw, for no
                // reason: the ring buffer's single port is never touched
                // by both trigger and waveform_draw at the same time by
                // construction, so there's no race in rearming early.
                trig_rearm <= 1'b1;
                state <= S_WINDOW_SETUP;
            end

            // Guarded on ili_pixel_ready: a set_window pulse asserted
            // while the driver isn't in its own S_READY state is silently
            // dropped, not queued - see ili9341_driver.v/stage1_top.v's
            // documented pitfall. Every WINDOW_SETUP state needs this.
            S_WINDOW_SETUP: begin
                if (ili_pixel_ready) begin
                    win_x0 <= 9'd0; win_x1 <= 9'd319;
                    win_y0 <= 8'd0; win_y1 <= 8'd239;
                    ili_set_window <= 1'b1;
                    state <= S_WINDOW_WAIT;
                end
            end
            S_WINDOW_WAIT: begin
                if (ili_window_done) begin
                    stream_idx <= 17'd0;
                    stream_x   <= 9'd0;
                    stream_y   <= 8'd0;
                    state <= S_STREAM_FETCH;
                end
            end
            S_STREAM_FETCH: begin
                fb_addr_top <= stream_idx;
                fb_we_top   <= 1'b0;
                state <= S_STREAM_FETCH2;
            end
            S_STREAM_FETCH2: begin
                state <= S_STREAM_PRESENT;
            end
            S_STREAM_PRESENT: begin
                if (ili_pixel_ready) begin
                    ili_pixel_data  <= box_on ? box_color : zoom_on ? 16'hFFE0 :
                                       fb_dout ? (otr_frame ? COLOR_CLIP : COLOR_TRACE) : (grid_on ? COLOR_GRID : COLOR_BLACK);
                    ili_pixel_valid <= 1'b1;
                    state <= S_STREAM_PULSE;
                end
            end
            S_STREAM_PULSE: begin
                if (stream_idx == 17'd76799) begin
                    state <= S_WAIT_TRIGGER;
                end else begin
                    stream_idx <= stream_idx + 17'd1;
                    if (stream_x == 9'd319) begin
                        stream_x <= 9'd0;
                        stream_y <= stream_y + 8'd1;
                    end else begin
                        stream_x <= stream_x + 9'd1;
                    end
                    state <= S_STREAM_FETCH;
                end
            end

            S_WAIT_TRIGGER: begin
                if (trig_capture_done) begin
                    fbclr_addr <= 17'd0;
                    state <= S_FBCLR;
                end
            end

            default: state <= S_INIT_SETUP;
        endcase
    end

endmodule
