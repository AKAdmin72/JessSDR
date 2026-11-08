// S-meter level: |I+jQ| ~ max(|I|,|Q|) + min/2 (within ~12 %), then
// log2 in 1/8 steps (L8 = 8*msb_position + 3 mantissa bits), peak-reading:
// rises instantly, falls ~26 L8 units/s (~20 dB/s at 13.18 kHz input rate).
// level = L8 (0..151); 6.02 dB per 8 units.
module level_meter (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    output wire [7:0]         level
);
    // 3-stage pipeline (64.8 MHz): |I|,|Q| -> magnitude -> log -> peak hold
    reg [17:0] ai = 18'd0, aq = 18'd0;
    reg [18:0] m = 19'd0;
    reg        v1 = 1'b0, v2 = 1'b0, v3 = 1'b0;
    always @(posedge clk) ai <= in_i[17] ? -in_i : in_i;
    always @(posedge clk) aq <= in_q[17] ? -in_q : in_q;
    always @(posedge clk) v1 <= in_valid;
    always @(posedge clk) m  <= (ai > aq) ? ai + (aq >> 1) : aq + (ai >> 1);
    always @(posedge clk) v2 <= v1;

    // msb position + 3 bits below it
    reg [4:0] p;
    reg [2:0] mant;
    integer k;
    always @(*) begin
        p = 5'd0;
        for (k = 0; k < 19; k = k + 1)
            if (m[k]) p = k;
        mant = (p >= 3) ? (m >> (p - 3)) : (m << (3 - p));
    end
    reg  [7:0] l8 = 8'd0;
    always @(posedge clk) l8 <= (m == 19'd0) ? 8'd0 : {p, mant};
    always @(posedge clk) v3 <= v2;

    reg [23:0] lv = 24'd0;                        // L8 in Q8.16
    always @(posedge clk)
        if (v3) begin
            if ({l8, 16'd0} > lv)   lv <= {l8, 16'd0};
            else if (lv > 24'd131)  lv <= lv - 24'd131;
            else                    lv <= 24'd0;
        end
    assign level = lv[23:16];
endmodule
