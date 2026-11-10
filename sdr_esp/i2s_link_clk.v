// BCK / WS for the I/Q link to the ESP32-S3 (iq_i2s_tx.v sends DATA).
// 32-bit stereo frames at fs = clk / DIV (64.8 MHz / 614 = 105.537 kHz, the
// ddc.v wide output rate), so BCK = 64 * fs = clk * 64 / DIV = 6.754 MHz.
// 64/614 is not an integer ratio: a fractional divider toggles BCK
// whenever acc (+128 per clk, mod DIV) wraps - half periods of 4 or 5 clk,
// exactly 128 toggles per DIV clocks, so a frame is exactly DIV clocks long
// and never slips against the DDC output (one I/Q pair per frame).
// WS changes together with the BCK falling edge (Philips I2S: the receiver
// sees the new WS on the next rise, the MSB one BCK later); WS low = left.
// Both outputs are registers (go straight to the pins).
module i2s_link_clk #(
    parameter integer DIV = 614            // clocks per frame, > 128
) (
    input  wire clk,
    output reg  bck = 1'b0,
    output reg  ws  = 1'b0
);
    reg  [10:0] acc = 11'd0;
    wire [11:0] nxt = {1'b0, acc} + 12'd128;
    wire        tog = (nxt >= DIV);
    reg  [5:0]  nb  = 6'd0;                 // BCK falls so far in this frame

    always @(posedge clk) acc <= tog ? nxt - DIV : nxt[10:0];
    always @(posedge clk) if (tog) bck <= ~bck;
    // falling edge = toggle while high
    always @(posedge clk) if (tog && bck) nb <= nb + 6'd1;
    always @(posedge clk) if (tog && bck) ws <= (nb + 6'd1) >= 6'd32;   // nb+1 = 32..63 -> right
endmodule
