// Tang Nano 20K buttons S1/S2 (active high) for the ESP while the encoder is
// not wired: debounced (8.1 ms ticks at 64.8 MHz, the ../sdr/sdr_top.v
// logic), short press = released before ~0.5 s, long = still held at 0.5 s
// (sent once, no short on release). Each event type has a free-running 2-bit
// counter; the ESP polls spi_regs BTN and acts on the counter differences,
// so nothing has to be cleared and no event is lost between polls (< 4 per
// type per poll).
//   btn = {s2_long, s2_short, s1_long, s1_short}, 2 bits each
module buttons #(
    parameter integer DEB_BITS = 19            // 2^19 / 64.8 MHz = 8.1 ms
) (
    input  wire       clk,
    input  wire       s1,
    input  wire       s2,
    output wire [7:0] btn
);
    reg [1:0] s1_sync = 2'b00, s2_sync = 2'b00;
    always @(posedge clk) s1_sync <= {s1_sync[0], s1};
    always @(posedge clk) s2_sync <= {s2_sync[0], s2};
    reg [DEB_BITS-1:0] deb_div = {DEB_BITS{1'b0}};
    always @(posedge clk) deb_div <= deb_div + 1'b1;
    wire deb_tick = (deb_div == {DEB_BITS{1'b0}});
    reg s1_st = 1'b0, s2_st = 1'b0, s1_prev = 1'b0, s2_prev = 1'b0;
    always @(posedge clk) if (deb_tick) s1_st <= s1_sync[1];
    always @(posedge clk) if (deb_tick) s2_st <= s2_sync[1];
    always @(posedge clk) s1_prev <= s1_st;
    always @(posedge clk) s2_prev <= s2_st;
    localparam [6:0] HOLD_LONG = 7'd62;        // ~0.5 s
    reg [6:0] s1_hold = 7'd0, s2_hold = 7'd0;
    always @(posedge clk)
        if (deb_tick) s1_hold <= !s1_st ? 7'd0 : (s1_hold == 7'd127) ? s1_hold : s1_hold + 7'd1;
    always @(posedge clk)
        if (deb_tick) s2_hold <= !s2_st ? 7'd0 : (s2_hold == 7'd127) ? s2_hold : s2_hold + 7'd1;
    wire s1_short = !s1_st && s1_prev && (s1_hold < HOLD_LONG);
    wire s2_short = !s2_st && s2_prev && (s2_hold < HOLD_LONG);
    wire s1_long  = deb_tick && s1_st && (s1_hold == HOLD_LONG - 7'd1);
    wire s2_long  = deb_tick && s2_st && (s2_hold == HOLD_LONG - 7'd1);

    reg [1:0] c1s = 2'd0, c1l = 2'd0, c2s = 2'd0, c2l = 2'd0;
    always @(posedge clk) if (s1_short) c1s <= c1s + 2'd1;
    always @(posedge clk) if (s1_long)  c1l <= c1l + 2'd1;
    always @(posedge clk) if (s2_short) c2s <= c2s + 2'd1;
    always @(posedge clk) if (s2_long)  c2l <= c2l + 2'd1;
    assign btn = {c2l, c2s, c1l, c1s};
endmodule
