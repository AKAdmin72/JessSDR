// Draws the captured ring-buffer window as a connected polyline: for each
// of N_SAMPLES=320 samples, computes its screen row via a fixed center-
// referenced scale (y = MID_ROW - (sample >>> VSHIFT), clamped to
// [0,239]) and issues one line_draw segment per consecutive pair of
// points (index 0 has no previous point, so it only latches; indices
// 1..319 each draw one segment, 319 segments total). Mirrors
// spectrum_trace.v's FETCH/FETCH2 pattern for absorbing the ring
// buffer's 1-cycle registered-read latency, but draws point-to-point
// segments instead of spectrum_trace.v's per-column fill-to-baseline,
// since this is a bipolar waveform, not a magnitude spectrum. Does NOT
// instantiate line_draw.v itself - the caller (osc_top.v) owns the one
// shared instance, matching how stage1_top.v plumbs spectrum_trace.v's
// ld_* ports through.
//
// Scaling: i_sample's low 6 bits are always 0 (12-bit ADC data
// left-justified into 18 bits by the Pico bridge - see i2s_rx.v), i.e.
// i_sample == ADC_code<<6 for ADC_code in +-2048. MID_ROW=120 (screen
// center) so the +-2048-code full range gets equal headroom in both
// directions. VSHIFT=11 is the smallest shift (i.e. finest resolution)
// that still keeps the WHOLE +-2048-code range within the panel's
// [0,239] rows with no clipping anywhere (2048 >> 5 == 64 <= the 120-row
// headroom each way; VSHIFT=10 would give 2048>>4==128, clipping past
// row 120+-120) - only reaches ~53% of the panel's height (64 of 120 rows
// each direction) because a plain power-of-2 shift can't hit the exact
// 120/2048 ratio, the cost of the no-multiplier/no-divider design (see
// line_draw.v's/spectrum_trace.v's own shift-only address math for the
// same tradeoff elsewhere in this codebase). Was previously VSHIFT=6/
// MID_ROW=180 (max sensitivity, baseline low) - reverted to this
// full-range/centered mode on request, since a low baseline forces the
// scale down to whichever headroom is smaller, wasting most of the
// screen when the true signal is bipolar and spans the full range.
module waveform_draw #(
    parameter [8:0] N_SAMPLES      = 9'd320,
    parameter signed [9:0] MID_ROW = 10'sd120,
    parameter integer VSHIFT       = 11
) (
    input  wire        clk,
    input  wire        start,
    output reg          busy = 1'b0,
    output reg          done = 1'b0,

    input  wire [8:0]  read_base,      // ring address of the oldest (leftmost) sample - trigger.wptr once frozen
    output reg  [8:0]  ring_raddr = 9'd0,
    input  wire signed [17:0] ring_rdata,

    output reg           ld_start = 1'b0,
    output reg  signed [9:0] ld_x0, ld_y0, ld_x1, ld_y1,
    input  wire           ld_busy,        // unused (interface parity with line_draw.v, matches spectrum_trace.v's convention)
    input  wire           ld_done
);

    localparam [2:0]
        S_IDLE      = 3'd0,
        S_FETCH     = 3'd1,
        S_FETCH2    = 3'd2,
        S_YCALC     = 3'd3,
        S_DRAW_WAIT = 3'd4,
        S_ADVANCE   = 3'd5,
        S_DONE      = 3'd6;

    reg [2:0] state = S_IDLE;
    reg [8:0] index = 9'd0;
    reg signed [9:0] px = 10'sd0, py = 10'sd0;

    // ring_rdata is valid throughout S_FETCH2/S_YCALC/S_DRAW_WAIT (the
    // address only changes in S_IDLE/S_ADVANCE, and the BSRAM's registered
    // dout re-reads the same address every cycle it's unchanged) - safe to
    // read combinationally rather than latching into a separate register.
    //
    // raw_shifted/y_raw are kept as wide as ring_rdata itself (18-bit
    // signed, +-131072) rather than sized just for the current VSHIFT/
    // MID_ROW - at VSHIFT=6 raw_shifted alone can reach +-2048, which
    // silently overflowed an earlier 10-bit version of this wire (fine at
    // the original VSHIFT=10, where the range was only +-128 - a real bug
    // caught only by re-deriving the range for the new VSHIFT, not by
    // synthesis or simulation at the old settings). Full width here means
    // VSHIFT/MID_ROW can be retuned again later without re-deriving safe
    // bit widths every time.
    wire signed [17:0] raw_shifted = ring_rdata >>> VSHIFT;
    wire signed [18:0] y_raw       = MID_ROW - raw_shifted;
    wire signed [9:0]  y_clamped   = (y_raw < 19'sd0)   ? 10'sd0   :
                                      (y_raw > 19'sd239) ? 10'sd239 :
                                      y_raw[9:0];

    always @(posedge clk) begin
        ld_start <= 1'b0;
        case (state)
            S_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    busy       <= 1'b1;
                    index      <= 9'd0;
                    ring_raddr <= read_base;
                    state      <= S_FETCH;
                end
            end
            S_FETCH: begin
                state <= S_FETCH2;
            end
            S_FETCH2: begin
                state <= S_YCALC;
            end
            S_YCALC: begin
                if (index == 9'd0) begin
                    // first point: no previous point to connect from yet
                    px    <= 10'sd0;
                    py    <= y_clamped;
                    state <= S_ADVANCE;
                end else begin
                    ld_x0    <= px;
                    ld_y0    <= py;
                    ld_x1    <= {1'b0, index};
                    ld_y1    <= y_clamped;
                    ld_start <= 1'b1;
                    state    <= S_DRAW_WAIT;
                end
            end
            S_DRAW_WAIT: begin
                if (ld_done) begin
                    px    <= {1'b0, index};
                    py    <= y_clamped;
                    state <= S_ADVANCE;
                end
            end
            S_ADVANCE: begin
                if (index == N_SAMPLES - 9'd1) begin
                    state <= S_DONE;
                end else begin
                    index      <= index + 9'd1;
                    ring_raddr <= (ring_raddr == N_SAMPLES - 9'd1) ? 9'd0 : ring_raddr + 9'd1;
                    state      <= S_FETCH;
                end
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
