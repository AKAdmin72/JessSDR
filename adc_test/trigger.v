// Rising-edge trigger + ring-buffer write-pointer/write-enable generator.
// Watches i_sample/i_valid from i2s_rx.v; on prev_sample<THRESHOLD &&
// i_sample>=THRESHOLD (rising edge only - falling transitions never
// fire), latches the current ring address as the trigger point and
// counts POSTTRIG further i_valid pulses (the triggering one included)
// before freezing writes (ring_we drops, wptr stops moving).
//
// PRETRIG+POSTTRIG==RING_DEPTH exactly (80+240==320) - this is what makes
// the frozen write pointer land precisely on the oldest pre-trigger
// sample's address (T+POSTTRIG mod RING_DEPTH == T-PRETRIG mod
// RING_DEPTH), so it doubles as the read-base for waveform_draw.v with no
// separate readout-start computation and no off-by-one. See the
// implementation plan for the full addressing proof. Because
// RING_DEPTH=320 isn't a power of 2, wptr uses explicit compare-and-wrap
// increment, not truncation.
//
// THRESHOLD must be `parameter signed` - a plain (unsigned) parameter
// would silently make the >= comparison unsigned, per this codebase's
// documented sign-extension pitfalls. 18'sd3200 (~ADC code +50 of a
// +-2048 12-bit range, since i_sample = ADC_code*64) is a placeholder;
// retune once real signal amplitude from the Pico ADC is observed on
// hardware - this is intentionally the "fixed compile-time threshold"
// PLAN.md calls sufficient for v1.
//
// Auto-trigger timeout: PLAN.md's original v1 scope explicitly left out
// an "auto-trigger fallback (free-run if no trigger found)" as a real-DSO
// feature not needed to prove the core loop - but a genuinely flat/DC
// input (e.g. the ADC input grounded, a real bring-up sanity check) then
// never produces a rising edge at all, so the display would wait forever
// showing only the empty grid, never even the flat baseline trace. Real
// scopes solve exactly this with an "Auto" trigger mode: if no real edge
// is found within a timeout, trigger anyway on whatever's currently in
// the buffer, so the display always shows something and still locks onto
// genuine edges the moment one occurs.
//
// AUTO_TIMEOUT=1 (fires on the very first sample after every arm/rearm,
// i.e. effectively disables edge-based waiting entirely - continuous
// free-run) is a deliberate debugging setting, requested to rule out
// trigger timing as a cause while checking the basic capture/scale/draw
// path shows anything at all. A real value (e.g. 8192, ~171ms at 48kHz)
// is more useful once edge-triggering itself needs testing - edge_hit
// (see below) still fires immediately as well and would win the race
// with auto_fire on any real crossing, so raising AUTO_TIMEOUT back up
// doesn't require touching this logic again.
//
// warmed_up gate: no capture (real edge OR auto-fire) is allowed to
// complete until RING_DEPTH samples have been written at least once
// since power-on. Without this, an early trigger (trivially guaranteed
// with AUTO_TIMEOUT=1, but possible with any small timeout or a genuine
// edge crossing early after reset) freezes a capture whose PRETRIG
// region reaches back into ring buffer addresses that were never
// actually written - undefined content (X in simulation; on real
// hardware, whatever BSRAM happens to power up with) that broke
// line_draw.v's Bresenham loop outright in simulation (a segment with
// X-valued coordinates can never satisfy its cx==ex&&cy==ey termination
// check) - caught via a 3M-cycle stuck-state trace, not by inspection.
// Once true, warmed_up never needs to go false again - the ring buffer
// stays fully populated with real data for the rest of the session
// regardless of how state/rearm move afterward.
module trigger #(
    parameter signed [17:0] THRESHOLD    = 18'sd3200,
    parameter [8:0]  PRETRIG      = 9'd80,
    parameter [8:0]  POSTTRIG     = 9'd240,
    parameter [8:0]  RING_DEPTH   = 9'd320,
    parameter [15:0] AUTO_TIMEOUT = 16'd1
) (
    input  wire         clk,
    input  wire signed [17:0] i_sample,
    input  wire          i_valid,
    input  wire          rearm,          // 1-cycle pulse: re-arm after the previous capture is consumed

    output wire [8:0]   wptr,           // ring buffer write address; frozen value also serves as the read-base
    output wire          ring_we,        // gates ring_buffer.we
    output reg  [8:0]   trig_addr    = 9'd0,   // address that held the triggering sample (debug/tb)
    output reg           capture_done = 1'b0,  // level, held through S_CAPTURED until rearm
    output reg  [1:0]   state        = 2'd0   // S_ARMED
);

    localparam [1:0] S_ARMED = 2'd0, S_CAPTURING = 2'd1, S_CAPTURED = 2'd2;

    reg signed [17:0] prev_sample = 18'sd0;
    reg [8:0]  wptr_r      = 9'd0;
    reg [8:0]  post_count  = 9'd0;
    reg [15:0] armed_count = 16'd0; // counts i_valid pulses since last (re)arm, for the auto-trigger timeout

    // Independent of state/rearm - tracks whether RING_DEPTH samples have
    // ever been written since power-on. Deliberately its own always block
    // (a different reg, not touched by the state/wptr/etc block below) -
    // one always block per reg, per this codebase's convention.
    reg [8:0] warmup_count = 9'd0;
    reg       warmed_up    = 1'b0;
    always @(posedge clk) begin
        if (!warmed_up && i_valid) begin
            if (warmup_count == RING_DEPTH - 9'd1)
                warmed_up <= 1'b1;
            else
                warmup_count <= warmup_count + 9'd1;
        end
    end

    wire edge_hit  = i_valid && (prev_sample < THRESHOLD) && (i_sample >= THRESHOLD);
    // >= (not ==): armed_count may already have overshot AUTO_TIMEOUT-1 by
    // the time warmed_up first turns true (warmup takes RING_DEPTH cycles,
    // which can exceed a small AUTO_TIMEOUT) - an equality check would
    // then never fire again until a full 16-bit wraparound.
    wire auto_fire = (armed_count >= AUTO_TIMEOUT - 16'd1);
    assign wptr    = wptr_r;
    assign ring_we = i_valid && (state != S_CAPTURED);

    always @(posedge clk) begin
        if (rearm) begin
            state        <= S_ARMED;
            capture_done <= 1'b0;
            armed_count  <= 16'd0;
        end else begin
            case (state)
                S_ARMED: begin
                    if (i_valid) begin
                        prev_sample <= i_sample;
                        if ((edge_hit || auto_fire) && warmed_up) begin
                            trig_addr  <= wptr_r;
                            post_count <= 9'd1;
                            state      <= S_CAPTURING;
                        end else begin
                            armed_count <= armed_count + 16'd1;
                        end
                        wptr_r <= (wptr_r == RING_DEPTH - 9'd1) ? 9'd0 : wptr_r + 9'd1;
                    end
                end
                S_CAPTURING: begin
                    if (i_valid) begin
                        wptr_r <= (wptr_r == RING_DEPTH - 9'd1) ? 9'd0 : wptr_r + 9'd1;
                        if (post_count == POSTTRIG - 9'd1) begin
                            capture_done <= 1'b1;
                            state        <= S_CAPTURED;
                        end else begin
                            post_count <= post_count + 9'd1;
                        end
                    end
                end
                S_CAPTURED: ; // hold - wait for rearm (handled above)
                default: state <= S_ARMED;
            endcase
        end
    end

endmodule
