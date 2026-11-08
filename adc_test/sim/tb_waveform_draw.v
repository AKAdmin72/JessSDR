`timescale 1ns/1ns
// Standalone check of waveform_draw.v: pre-loads ring_buffer with a known
// signed sample pattern (a smooth ramp through the middle, plus explicit
// full-scale extremes at both ends - at the current VSHIFT=11/MID_ROW=120
// these no longer clip, by design: VSHIFT=11 was specifically chosen so
// the WHOLE +-2048-ADC-code range fits within the panel with no clipping
// anywhere), then verifies every line_draw segment waveform_draw issues
// matches the expected (x,y) polyline computed from the SAME scaling
// formula (VSHIFT=11, MID_ROW=120) waveform_draw.v itself uses - see that
// file's header for the derivation. Uses the real line_draw.v (already
// proven) just to drive the busy/done handshake; its fb_addr/fb_we
// outputs are left unconnected since this test only checks the segment
// coordinates waveform_draw issues, not rasterized pixels.
module tb_waveform_draw;
    reg clk = 0;
    always #18.5 clk = ~clk;

    // ---- ring buffer, pre-loaded with a known pattern ----
    reg  [8:0] preload_addr = 9'd0;
    reg         rb_we        = 1'b0;
    reg  signed [17:0] rb_din = 18'sd0;
    reg         preloading   = 1'b1;
    wire signed [17:0] rb_dout;
    wire [8:0] wd_ring_raddr;
    wire [8:0] rb_addr = preloading ? preload_addr : wd_ring_raddr;

    ring_buffer u_ring (.clk(clk), .addr(rb_addr), .we(rb_we), .din(rb_din), .dout(rb_dout));

    reg signed [17:0] samp_pattern [0:319];
    integer x;

    initial begin
        for (x = 0; x < 320; x = x + 1) begin
            if (x == 0)
                samp_pattern[x] = 18'sd131071;   // max positive (full ADC scale) - must NOT clip at VSHIFT=11
            else if (x == 319)
                samp_pattern[x] = -18'sd131072;  // max negative (full ADC scale) - must NOT clip at VSHIFT=11
            else
                samp_pattern[x] = (x - 160) * 700; // smooth ramp, stays within [0,239] unclamped at VSHIFT=11
        end
    end

    // ---- DUT ----
    wire        wd_busy, wd_done;
    reg         wd_start = 1'b0;
    reg  [8:0] wd_read_base = 9'd0;
    wire        ld_start, ld_busy, ld_done;
    wire signed [9:0] ld_x0, ld_y0, ld_x1, ld_y1;

    waveform_draw dut (
        .clk(clk),
        .start(wd_start), .busy(wd_busy), .done(wd_done),
        .read_base(wd_read_base),
        .ring_raddr(wd_ring_raddr), .ring_rdata(rb_dout),
        .ld_start(ld_start), .ld_x0(ld_x0), .ld_y0(ld_y0), .ld_x1(ld_x1), .ld_y1(ld_y1),
        .ld_busy(ld_busy), .ld_done(ld_done)
    );

    wire [16:0] ld_fb_addr; // unused - only line_draw's busy/done handshake is needed here
    wire        ld_fb_we;
    line_draw u_ld (
        .clk(clk), .start(ld_start),
        .x0(ld_x0), .y0(ld_y0), .x1(ld_x1), .y1(ld_y1),
        .busy(ld_busy), .done(ld_done),
        .fb_addr(ld_fb_addr), .fb_we(ld_fb_we)
    );

    // ---- expected-y oracle: same formula as waveform_draw.v (VSHIFT=11, MID_ROW=120) ----
    function signed [9:0] expected_y(input signed [17:0] samp);
        reg signed [17:0] raw_shifted;
        reg signed [18:0] y_raw;
        begin
            raw_shifted = samp >>> 11;
            y_raw = 10'sd120 - raw_shifted;
            if (y_raw < 0) expected_y = 10'sd0;
            else if (y_raw > 239) expected_y = 10'sd239;
            else expected_y = y_raw[9:0];
        end
    endfunction

    // ---- capture every segment waveform_draw issues ----
    reg signed [9:0] seg_x0 [0:318];
    reg signed [9:0] seg_y0 [0:318];
    reg signed [9:0] seg_x1 [0:318];
    reg signed [9:0] seg_y1 [0:318];
    integer seg_count = 0;

    always @(posedge clk) begin
        if (ld_start) begin
            seg_x0[seg_count] <= ld_x0;
            seg_y0[seg_count] <= ld_y0;
            seg_x1[seg_count] <= ld_x1;
            seg_y1[seg_count] <= ld_y1;
            seg_count <= seg_count + 1;
        end
    end

    integer errors = 0;
    integer i;

    initial begin
        // ---- preload ring_buffer[0..319] = samp_pattern[0..319] ----
        // NOTE: #1 after every @(posedge clk), before the next blocking
        // assignment - setting new stimulus in the SAME time step as the
        // edge the DUT's always block also reacts to is a genuine
        // scheduling race (confirmed empirically against ring_buffer.v -
        // iverilog 12.0 can silently drop the DUT's same-edge update
        // otherwise) - see this codebase's documented "blocking assignment
        // right after @(posedge clk)" pitfall.
        rb_we = 1'b1;
        for (x = 0; x < 320; x = x + 1) begin
            preload_addr = x[8:0];
            rb_din        = samp_pattern[x];
            @(posedge clk);
            #1;
        end
        rb_we = 1'b0;
        preloading = 1'b0;
        @(posedge clk);
        #1;

        // ---- run waveform_draw over the full buffer ----
        wd_read_base = 9'd0;
        wd_start = 1'b1;
        @(posedge clk);
        #1;
        wd_start = 1'b0;

        wait (wd_done);
        @(posedge clk); // let seg_count's last increment settle

        if (seg_count !== 319) begin
            $display("FAIL: expected 319 segments, got %0d", seg_count);
            errors = errors + 1;
        end

        // ---- verify every segment against the expected polyline ----
        for (i = 0; i < 319; i = i + 1) begin
            if (seg_x0[i] !== i[9:0]) begin
                $display("FAIL: segment %0d x0=%0d expected=%0d", i, seg_x0[i], i);
                errors = errors + 1;
            end
            if (seg_x1[i] !== (i + 1)) begin
                $display("FAIL: segment %0d x1=%0d expected=%0d", i, seg_x1[i], i + 1);
                errors = errors + 1;
            end
            if (seg_y0[i] !== expected_y(samp_pattern[i])) begin
                $display("FAIL: segment %0d y0=%0d expected=%0d (sample=%0d)", i, seg_y0[i], expected_y(samp_pattern[i]), samp_pattern[i]);
                errors = errors + 1;
            end
            if (seg_y1[i] !== expected_y(samp_pattern[i+1])) begin
                $display("FAIL: segment %0d y1=%0d expected=%0d (sample=%0d)", i, seg_y1[i], expected_y(samp_pattern[i+1]), samp_pattern[i+1]);
                errors = errors + 1;
            end
        end

        // ---- full-range-fits-on-screen check: neither extreme should hit
        // the clamp boundary at VSHIFT=11 (that's the whole point of that
        // shift value - see waveform_draw.v's header) ----
        if (expected_y(samp_pattern[0]) <= 0 || expected_y(samp_pattern[0]) >= 239) begin
            $display("FAIL: max positive sample clamped (y=%0d) - full range no longer fits on screen", expected_y(samp_pattern[0]));
            errors = errors + 1;
        end
        if (expected_y(samp_pattern[319]) <= 0 || expected_y(samp_pattern[319]) >= 239) begin
            $display("FAIL: max negative sample clamped (y=%0d) - full range no longer fits on screen", expected_y(samp_pattern[319]));
            errors = errors + 1;
        end
        $display("INFO: expected_y(max positive)=%0d, expected_y(max negative)=%0d (neither should be 0 or 239)", expected_y(samp_pattern[0]), expected_y(samp_pattern[319]));

        if (errors == 0) $display("PASS: tb_waveform_draw");
        else $display("FAIL: tb_waveform_draw had %0d errors", errors);
        $finish;
    end
endmodule
