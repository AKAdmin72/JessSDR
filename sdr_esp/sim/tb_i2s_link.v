`timescale 1ns/1ps
// Self-checking tb for i2s_link_clk.v + iq_i2s_tx.v at the real rates:
// 64.8 MHz clk, frame = 614 clk, I/Q pair every 614 clk (like ddc.v w_valid).
// A bit-level Philips I2S receiver (samples DATA on BCK rises) checks
//   - BCK: 64 periods per frame, half periods 4 or 5 clk, frame = 614 clk
//   - WS: 32 BCK low (left) / 32 high (right), changes only on BCK falls
//   - DATA changes only 2 clk after a BCK fall (never near a rise)
//   - test mode: left = {cnt,0}, right = {~cnt,0}, cnt +1 per frame
//   - normal mode: every pair fed in comes out once, in order ({I,0}/{Q,0})
module tb_i2s_link;
    reg clk = 0;
    always #7.716 clk = ~clk;
    wire bck, ws, data_raw;
    reg  test_mode = 1;
    reg  in_valid = 0;
    reg  signed [17:0] in_i = 0, in_q = 0;
    i2s_link_clk #(.DIV(614)) gen (.clk(clk), .bck(bck), .ws(ws));
    iq_i2s_tx dut (.clk(clk), .bck(bck), .lrck(ws), .test_mode(test_mode),
                   .in_valid(in_valid), .in_i(in_i), .in_q(in_q), .data(data_raw));
    reg data = 0;
    always @(posedge clk) data <= data_raw;      // = sdr_esp_top data_o

    integer errors = 0;
    // ---- clock-level checks ----
    integer cyc = 0, last_edge = 0, last_fall = -100, last_rise = -100;
    reg bck_p = 0, ws_p = 0, data_p = 0;
    integer nrise = 0, frame_start = -1;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (bck != bck_p) begin
            if (cyc > 2000 && (cyc - last_edge < 4 || cyc - last_edge > 5)) begin
                $display("FAIL: BCK half period %0d clk", cyc - last_edge); errors = errors + 1; end
            last_edge = cyc;
            if (!bck) last_fall = cyc; else begin last_rise = cyc; nrise = nrise + 1; end
        end
        if (ws != ws_p) begin
            if (!(bck_p && !bck)) begin $display("FAIL: WS changed without a BCK fall"); errors = errors + 1; end
            if (!ws) begin                        // new frame (left slot)
                if (frame_start >= 0 && (cyc - frame_start != 614 || nrise != 64)) begin
                    $display("FAIL: frame %0d clk, %0d BCK", cyc - frame_start, nrise); errors = errors + 1; end
                frame_start = cyc; nrise = 0;
            end
        end
        if (data != data_p && cyc - last_fall != 2) begin
            $display("FAIL: DATA changed %0d clk after the BCK fall", cyc - last_fall); errors = errors + 1; end
        bck_p = bck; ws_p = ws; data_p = data;
    end

    // ---- I/Q source: a pair every 614 clk, values from a counter ----
    integer pc = 0, pairs = 0;
    always @(posedge clk) begin
        #1 in_valid = 0;
        pc = pc + 1;
        if (pc == 614) begin
            pc = 0; pairs = pairs + 1;
            in_i = pairs * 37 - 70000; in_q = -(pairs * 53) + 1000;
            in_valid = 1;
        end
    end

    // ---- bit-level receiver ----
    reg [31:0] acc = 0; integer nb = -1; reg ch = 0, ws_seen = 0;
    reg [31:0] lw = 0; reg have_l = 0;
    integer frames = 0, tframes = 0, nframes = 0; reg [30:0] lastc = 0; reg havec = 0;
    integer exp_k = -1;  // normal mode: index of the expected pair
    task check_frame(input [31:0] l, input [31:0] r);
        integer k; reg signed [17:0] ei, eq;
        begin
            frames = frames + 1;
            if (test_mode) begin
                if (l[0] || r[0] || r[31:1] !== ~l[31:1]) begin $display("FAIL: test frame %h %h", l, r); errors = errors + 1; end
                if (havec && l[31:1] !== lastc + 1) begin $display("FAIL: counter jump %h -> %h", lastc, l[31:1]); errors = errors + 1; end
                lastc = l[31:1]; havec = 1; tframes = tframes + 1;
            end else if (frames > 3) begin
                // recover the pair index from I, then Q must match the same index
                k = ($signed(l[31:14]) + 70000) / 37;
                ei = k * 37 - 70000; eq = -(k * 53) + 1000;
                if (l[13:0] != 0 || r[13:0] != 0 || $signed(l[31:14]) !== ei || $signed(r[31:14]) !== eq) begin
                    $display("FAIL: pair %h %h", l, r); errors = errors + 1; end
                if (exp_k >= 0 && k != exp_k) begin $display("FAIL: pair %0d after %0d", k, exp_k - 1); errors = errors + 1; end
                exp_k = k + 1; nframes = nframes + 1;
            end
        end
    endtask
    always @(posedge bck) begin
        if (ws != ws_seen) begin
            if (nb == 31) begin
                if (!ch) begin lw = {acc[30:0], data}; have_l = 1; end
                else if (have_l) check_frame(lw, {acc[30:0], data});
            end
            ws_seen = ws; ch = ws; nb = 0; acc = 0;
        end else if (nb >= 0 && nb < 31) begin
            acc = {acc[30:0], data}; nb = nb + 1;
        end
    end

    initial begin
        #(614 * 15.432 * 300);
        if (tframes < 290) begin $display("FAIL: only %0d test frames", tframes); errors = errors + 1; end
        test_mode = 0; frames = 0;
        #(614 * 15.432 * 300);
        if (nframes < 290) begin $display("FAIL: only %0d I/Q frames", nframes); errors = errors + 1; end
        if (errors == 0) $display("PASS: i2s link - %0d test frames, %0d I/Q frames, BCK/WS/DATA timing", tframes, nframes);
        else $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
