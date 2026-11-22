// I2S transmitter: sends the I/Q pair to the ESP32-S3 (stage 4, PLAN.md).
// The FPGA is the bus master; BCK and WS come from outside this module.
// (FpgaRX: the PCM1808 clocks, BCK = clk/8, LRCK = clk/512. JessSDR
// sdr_esp: i2s_link_clk.v, BCK 6.754 MHz, WS 105.54 kHz = the wide DDC
// rate.) This module only produces DATA.
//
// Format: Philips I2S, 32-bit slots, MSB first, MSB one BCK after the WS
// edge. WS low = left = I, WS high = right = Q. Each slot carries bits
// 31..1 of the word; bit 0 falls on the next WS edge and is always 0.
//   normal mode: word = {sample[17:0], 14'b0} (left-justified 18-bit)
//   test mode:   left = {cnt[30:0], 0}, right = {~cnt[30:0], 0}, cnt + 1
//                per frame - the ESP checks continuity and the complement.
//
// DATA changes on BCK falling edges (2 clk after the fall, ~74ns), and the
// receiver samples on rising edges (4 clk = 148ns after the fall).
module iq_i2s_tx (
    input  wire        clk,
    input  wire        bck,        // pcm1808_clkgen outputs (registers, sync to clk)
    input  wire        lrck,
    input  wire        test_mode,

    input  wire        in_valid,   // new I/Q pair
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,

    output reg         data = 1'b0
);

    reg bck_d = 1'b0;
    always @(posedge clk) bck_d <= bck;
    wire bck_fall = bck_d & ~bck;

    // latest pair, and the pair being sent (taken at the start of the I slot
    // so that I and Q always belong together)
    reg signed [17:0] hold_i = 18'sd0, hold_q = 18'sd0;
    always @(posedge clk) if (in_valid) hold_i <= in_i;
    always @(posedge clk) if (in_valid) hold_q <= in_q;

    reg [17:0] send_q = 18'd0;
    reg [30:0] cnt    = 31'd0;
    reg        ws_last = 1'b0;
    reg [31:0] sr     = 32'd0;

    wire ws_edge = bck_fall && (lrck != ws_last);

    always @(posedge clk) if (bck_fall) ws_last <= lrck;

    always @(posedge clk)
        if (ws_edge && !lrck) send_q <= hold_q;

    always @(posedge clk)
        if (ws_edge && lrck) cnt <= cnt + 31'd1;   // after the Q slot started

    always @(posedge clk)
        if (ws_edge) begin
            if (!lrck) sr <= test_mode ? {cnt, 1'b0}  : {hold_i, 14'd0};
            else       sr <= test_mode ? {~cnt, 1'b0} : {send_q, 14'd0};
        end else if (bck_fall)
            sr <= {sr[30:0], 1'b0};

    // On the WS edge itself the previous slot's last bit (always 0) goes out;
    // the new word's MSB follows one BCK later.
    always @(posedge clk)
        if (ws_edge)       data <= 1'b0;
        else if (bck_fall) data <= sr[31];

endmodule
