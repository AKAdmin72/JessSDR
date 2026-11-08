`timescale 1ns/1ns
// Writes a known pattern to every ring_buffer address, then reads it back
// confirming 1-cycle synchronous read latency, read-first same-address
// read-during-write semantics, and that addresses 319/0 don't alias.
module tb_ring_buffer;
    reg clk = 0;
    always #18.5 clk = ~clk;

    reg  [8:0] addr = 9'd0;
    reg         we   = 1'b0;
    reg  signed [17:0] din = 18'sd0;
    wire signed [17:0] dout;

    ring_buffer dut (.clk(clk), .addr(addr), .we(we), .din(din), .dout(dout));

    integer errors = 0;
    integer i;
    reg signed [17:0] expected [0:319];

    initial begin
        // ---- write a distinct value to every address ----
        // NOTE: #1 after every @(posedge clk), before the next iteration's
        // blocking assignments - setting new stimulus in the SAME time step
        // as the edge the DUT's always block also reacts to is a genuine
        // scheduling race (confirmed empirically: without this, iverilog
        // 12.0 silently drops every other write in a back-to-back
        // @(posedge clk) sequence) - see this codebase's documented
        // "blocking assignment right after @(posedge clk)" pitfall.
        we = 1'b1;
        for (i = 0; i < 320; i = i + 1) begin
            expected[i] = (i * 97) - 15000; // arbitrary spread, some negative
            addr = i[8:0];
            din  = expected[i];
            @(posedge clk);
            #1;
        end
        we = 1'b0;
        @(posedge clk);
        #1;

        // ---- linear readback, confirming 1-cycle registered latency ----
        for (i = 0; i < 320; i = i + 1) begin
            addr = i[8:0];
            @(posedge clk);
            #1;
            if (dout !== expected[i]) begin
                $display("FAIL: addr %0d readback=%0d expected=%0d", i, dout, expected[i]);
                errors = errors + 1;
            end
        end

        // ---- read-first same-address read-during-write ----
        addr = 9'd50;
        din  = 18'sd12345;
        we   = 1'b1;
        @(posedge clk);
        #1;
        if (dout !== expected[50]) begin
            $display("FAIL: read-during-write returned %0d, expected OLD value %0d", dout, expected[50]);
            errors = errors + 1;
        end
        we = 1'b0;
        @(posedge clk);
        #1;
        if (dout !== 18'sd12345) begin
            $display("FAIL: post-write readback=%0d, expected 12345", dout);
            errors = errors + 1;
        end

        // ---- wrap-boundary addresses 319/0 don't alias ----
        addr = 9'd319; @(posedge clk); #1;
        if (dout !== expected[319]) begin
            $display("FAIL: addr 319 readback=%0d expected=%0d", dout, expected[319]);
            errors = errors + 1;
        end
        addr = 9'd0; @(posedge clk); #1;
        if (dout !== expected[0]) begin
            $display("FAIL: addr 0 readback=%0d expected=%0d", dout, expected[0]);
            errors = errors + 1;
        end

        if (errors == 0) $display("PASS: tb_ring_buffer");
        else $display("FAIL: tb_ring_buffer had %0d errors", errors);
        $finish;
    end
endmodule
