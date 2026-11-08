// adc65_top end-to-end: fake ADC = 12-bit counter advancing on every ADC
// clock (OTR = counter bit 5), UART driven/decoded by the tb at DIV=8.
// Sends '?', 'p' 5, 'c' and checks the replies and that the dump is a run of
// consecutive counter values (no dropped / duplicated samples).
`timescale 1ns/1ps
module tb_adc65;
    reg clk = 0;
    always #5 clk = ~clk;                       // rpll_sim passes it through
    wire adc_clk, txd;
    reg  rxd = 1;
    reg [11:0] cnt = 12'd100;
    always @(posedge adc_clk) cnt <= cnt + 12'd1;
    adc65_top #(.CLK_KHZ(100000), .BAUD(12500000), .NSAMP_LOG2(6)) dut (
        .clk(clk), .adc_clk(adc_clk), .adc_d(cnt), .adc_otr(cnt[5]), .uart_tx(txd), .uart_rx(rxd));

    localparam integer BIT = 80;                // 8 clocks of 10 ns
    task send(input [7:0] b);
        integer i;
        begin
            rxd = 0; #(BIT);
            for (i = 0; i < 8; i = i + 1) begin rxd = b[i]; #(BIT); end
            rxd = 1; #(BIT);
        end
    endtask
    reg [7:0] rb [0:255];
    integer nrx = 0;
    task recv(output [7:0] b);
        integer i;
        begin
            @(negedge txd); #(BIT + BIT / 2);
            for (i = 0; i < 8; i = i + 1) begin b[i] = txd; #(BIT); end
            if (txd !== 1'b1) $display("FAIL stop bit");
        end
    endtask

    integer i, errors = 0;
    reg [7:0] b0, b1, b2, b3, b4;
    reg [15:0] w, prev;
    initial begin
        #2000;
        send("?"); recv(b0); recv(b1); recv(b2); recv(b3);
        $display("info: %c ph=%0d clk=%0d ver=%0d", b0, b1, b2, b3);
        if (b0 != "I" || b1 != dut.PHASE_INIT || b2 != 100) errors = errors + 1;
        send("p"); send(8'd5); recv(b0); recv(b1);
        if (b0 != "P" || b1 != 5) begin errors = errors + 1; $display("FAIL phase ack %c %0d", b0, b1); end
        if (dut.phase != 5) errors = errors + 1;
        send("c");
        recv(b0); recv(b1); recv(b2); recv(b3); recv(b4);
        if (b0 != "A" || b1 != "D" || {b3, b2} != 64 || b4 != 5) begin errors = errors + 1; $display("FAIL header"); end
        for (i = 0; i < 64; i = i + 1) begin
            recv(b0); recv(b1);
            w = {b1, b0};
            if (i > 0 && w[11:0] != ((prev[11:0] + 1) & 12'hFFF)) begin
                errors = errors + 1; if (errors < 5) $display("FAIL sample %0d: %h after %h", i, w, prev);
            end
            if (w[15] != w[5]) begin errors = errors + 1; $display("FAIL otr bit at %0d", i); end
            prev = w;
        end
        if (errors == 0) $display("PASS"); else $display("FAILED %0d", errors);
        $finish;
    end
    initial begin #5_000_000; $display("TIMEOUT"); $finish; end
endmodule
