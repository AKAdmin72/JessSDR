`timescale 1ns/1ns
// Drives i_sample/i_valid directly (no I2S framing needed - this is pure
// digital-logic testing of trigger.v's rising-edge detection, post-trigger
// counting, write-freeze, and rearm behavior). Independently tracks an
// expected write-pointer alongside the DUT to verify the ring-addressing
// identity worked out in the implementation plan:
// wptr_after_capture == (trig_addr + POSTTRIG) mod RING_DEPTH.
module tb_trigger;
    reg clk = 0;
    always #18.5 clk = ~clk;

    localparam signed [17:0] THRESHOLD    = 18'sd3200;
    localparam [8:0]  POSTTRIG     = 9'd240;
    localparam [8:0]  RING_DEPTH   = 9'd320;
    // Overridden down from the real design's 8192 (~171ms @ 48kHz) purely
    // so the dedicated auto-trigger test below doesn't need to send 8192
    // samples - still comfortably larger than every other sequence in this
    // testbench (max ~31 samples per S_ARMED period elsewhere), so it can't
    // interfere with the earlier real-edge tests.
    localparam [15:0] AUTO_TIMEOUT = 16'd100;

    reg signed [17:0] i_sample = 18'sd0;
    reg          i_valid = 1'b0;
    reg          rearm   = 1'b0;
    wire [8:0]   wptr;
    wire          ring_we;
    wire [8:0]   trig_addr;
    wire          capture_done;
    wire [1:0]   state;

    trigger #(.AUTO_TIMEOUT(AUTO_TIMEOUT)) dut (
        .clk(clk), .i_sample(i_sample), .i_valid(i_valid), .rearm(rearm),
        .wptr(wptr), .ring_we(ring_we), .trig_addr(trig_addr),
        .capture_done(capture_done), .state(state)
    );

    integer errors = 0;
    integer k;
    integer wptr_model;
    integer expected_trig_addr;
    integer expected_wptr_after;

    // #1 after @(posedge clk), before clearing i_valid: clearing it in the
    // SAME time step as the edge the DUT's always block also reacts to is
    // a genuine scheduling race (confirmed empirically against ring_buffer.v
    // - iverilog 12.0 can silently drop the DUT's same-edge update otherwise)
    // - see this codebase's documented "blocking assignment right after
    // @(posedge clk)" pitfall.
    task automatic send_sample(input signed [17:0] val);
        begin
            i_sample = val;
            i_valid  = 1'b1;
            @(posedge clk);
            #1;
            i_valid  = 1'b0;
            wptr_model = (wptr_model == RING_DEPTH - 1) ? 0 : wptr_model + 1;
        end
    endtask

    initial begin
        wptr_model = 0;

        // ---- pre-warmup gate: a genuine rising-edge crossing occurring
        // before RING_DEPTH samples have ever been written must NOT
        // complete a capture (see trigger.v's header for why - an early
        // capture's PRETRIG region would reach into never-written ring
        // buffer addresses). Send a real edge at sample #10 (well under
        // RING_DEPTH=320) and confirm it's ignored. ----
        for (k = 0; k < 9; k = k + 1)
            send_sample(18'sd0);
        send_sample(18'sd5000); // would trigger post-warmup - must be ignored here
        if (state !== dut.S_ARMED) begin
            $display("FAIL: a pre-warmup edge crossing incorrectly triggered a capture");
            errors = errors + 1;
        end
        if (capture_done) begin
            $display("FAIL: capture_done set from a pre-warmup edge crossing");
            errors = errors + 1;
        end
        if (dut.warmed_up) begin
            $display("FAIL: warmed_up set after only 10 samples (expected RING_DEPTH=%0d)", RING_DEPTH);
            errors = errors + 1;
        end

        // ---- finish warming up: no capture (real edge OR auto-fire) is
        // allowed until RING_DEPTH samples have been written since
        // power-on. Send below-threshold samples the rest of the way so
        // every test after this point runs post-warmup, matching how it
        // behaves for the rest of a real session. ----
        for (k = 0; k < RING_DEPTH - 10; k = k + 1)
            send_sample(18'sd1000);
        if (!dut.warmed_up) begin
            $display("FAIL: warmed_up not set after RING_DEPTH samples");
            errors = errors + 1;
        end

        // Rearm to reset armed_count to 0 (warmed_up itself never resets,
        // it's a one-way latch) - without this, armed_count already sits
        // near RING_DEPTH from the warmup loop above, past this
        // testbench's overridden AUTO_TIMEOUT=100, so auto_fire would
        // fire on the very next sample instead of leaving room for the
        // deliberately-timed edge tests below.
        rearm = 1'b1;
        @(posedge clk);
        #1;
        rearm = 1'b0;
        @(posedge clk);
        #1;

        // ---- watch phase: below-threshold samples, no trigger expected ----
        for (k = 0; k < 40; k = k + 1)
            send_sample(18'sd1000);

        if (state !== dut.S_ARMED) begin
            $display("FAIL: state left S_ARMED during below-threshold watch phase");
            errors = errors + 1;
        end

        // ---- rising-edge trigger ----
        expected_trig_addr = wptr_model; // address the triggering sample lands at
        send_sample(18'sd5000); // crosses THRESHOLD (3200)

        if (state !== dut.S_CAPTURING) begin
            $display("FAIL: state did not move to S_CAPTURING right after the trigger sample");
            errors = errors + 1;
        end
        if (trig_addr !== expected_trig_addr[8:0]) begin
            $display("FAIL: trig_addr=%0d expected=%0d", trig_addr, expected_trig_addr);
            errors = errors + 1;
        end

        // ---- 239 more post-trigger samples (240 total incl. the trigger sample) ----
        for (k = 0; k < 239; k = k + 1) begin
            if (capture_done) begin
                $display("FAIL: capture_done asserted early, after only %0d post-trigger samples", k + 1);
                errors = errors + 1;
            end
            send_sample(18'sd0); // level irrelevant during S_CAPTURING
        end

        if (!capture_done) begin
            $display("FAIL: capture_done not asserted after 240 post-trigger samples");
            errors = errors + 1;
        end
        if (state !== dut.S_CAPTURED) begin
            $display("FAIL: state did not reach S_CAPTURED after capture completed");
            errors = errors + 1;
        end

        // ---- addressing identity: wptr == (trig_addr + POSTTRIG) mod RING_DEPTH ----
        expected_wptr_after = (expected_trig_addr + POSTTRIG) % RING_DEPTH;
        if (wptr !== expected_wptr_after[8:0]) begin
            $display("FAIL: frozen wptr=%0d expected=%0d ((trig_addr+POSTTRIG) mod RING_DEPTH)", wptr, expected_wptr_after);
            errors = errors + 1;
        end

        // ---- freeze verification: further i_valid pulses must not move wptr or assert ring_we ----
        for (k = 0; k < 5; k = k + 1) begin
            if (ring_we) begin
                $display("FAIL: ring_we asserted while state==S_CAPTURED");
                errors = errors + 1;
            end
            i_sample = 18'sd9999;
            i_valid  = 1'b1;
            @(posedge clk);
            #1;
            i_valid  = 1'b0;
        end
        if (wptr !== expected_wptr_after[8:0]) begin
            $display("FAIL: wptr moved while frozen in S_CAPTURED (now %0d, expected %0d)", wptr, expected_wptr_after);
            errors = errors + 1;
        end

        // ---- rearm ----
        rearm = 1'b1;
        @(posedge clk);
        #1;
        rearm = 1'b0;
        @(posedge clk); // let state settle
        #1;
        if (state !== dut.S_ARMED) begin
            $display("FAIL: state did not return to S_ARMED after rearm");
            errors = errors + 1;
        end
        wptr_model = expected_wptr_after; // wptr_r is untouched by rearm, continues from here

        // ---- falling-edge-only transition must NOT trigger ----
        send_sample(18'sd5000); // above threshold, establishes prev_sample high
        for (k = 0; k < 10; k = k + 1)
            send_sample(18'sd0); // drops below threshold repeatedly - falling edges only
        if (state !== dut.S_ARMED) begin
            $display("FAIL: a falling-edge transition incorrectly triggered capture");
            errors = errors + 1;
        end

        // ---- second rising-edge trigger, positioned to wrap past RING_DEPTH ----
        while (wptr_model < 310)
            send_sample(18'sd0);
        expected_trig_addr = wptr_model;
        send_sample(18'sd5000); // trigger #2

        for (k = 0; k < 239; k = k + 1)
            send_sample(18'sd0);

        if (!capture_done) begin
            $display("FAIL: capture_done not asserted on the second (wraparound) trigger");
            errors = errors + 1;
        end
        expected_wptr_after = (expected_trig_addr + POSTTRIG) % RING_DEPTH;
        if (wptr !== expected_wptr_after[8:0]) begin
            $display("FAIL: second trigger frozen wptr=%0d expected=%0d (wraparound case)", wptr, expected_wptr_after);
            errors = errors + 1;
        end

        // ---- auto-trigger: a flat/DC input (no edge ever crosses
        // THRESHOLD) must still trigger once AUTO_TIMEOUT samples have
        // elapsed since the last (re)arm, exactly like a real scope's
        // "Auto" mode - otherwise a grounded/quiet input would leave the
        // display waiting forever, never even showing the flat baseline. ----
        rearm = 1'b1;
        @(posedge clk);
        #1;
        rearm = 1'b0;
        @(posedge clk);
        #1;

        for (k = 0; k < AUTO_TIMEOUT - 1; k = k + 1) begin
            if (state !== dut.S_ARMED) begin
                $display("FAIL: auto-trigger fired early, after only %0d flat samples", k);
                errors = errors + 1;
            end
            send_sample(18'sd0); // flat, never crosses THRESHOLD
        end

        expected_trig_addr = wptr_model; // address the auto-triggering sample lands at
        send_sample(18'sd0); // the AUTO_TIMEOUT-th sample - auto_fire must fire here

        if (state !== dut.S_CAPTURING) begin
            $display("FAIL: auto-trigger did not fire after %0d flat samples", AUTO_TIMEOUT);
            errors = errors + 1;
        end
        if (trig_addr !== expected_trig_addr[8:0]) begin
            $display("FAIL: auto-trigger trig_addr=%0d expected=%0d", trig_addr, expected_trig_addr);
            errors = errors + 1;
        end

        for (k = 0; k < 239; k = k + 1)
            send_sample(18'sd0);

        if (!capture_done) begin
            $display("FAIL: capture_done not asserted after an auto-triggered capture");
            errors = errors + 1;
        end
        expected_wptr_after = (expected_trig_addr + POSTTRIG) % RING_DEPTH;
        if (wptr !== expected_wptr_after[8:0]) begin
            $display("FAIL: auto-triggered capture frozen wptr=%0d expected=%0d", wptr, expected_wptr_after);
            errors = errors + 1;
        end

        if (errors == 0) $display("PASS: tb_trigger");
        else $display("FAIL: tb_trigger had %0d errors", errors);
        $finish;
    end
endmodule
