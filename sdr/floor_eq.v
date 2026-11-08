// Automatic noise floor: per-bin flattening of the spectrum line, adapted
// from ../FpgaRX-SDR/stage1/spectrum_eq.v (same algorithm; 9-bit L8 levels,
// 320 columns in display order, runs once per averaged line ~12/s).
//
// 1. Per-bin floor estimate est[x] (9.8 fixed point): L < est -> est = L
//    (snaps down at once), else est += RISE/256 per line (RISE = 512:
//    2 L8 units = 0.75 dB per line, ~9 dB/s - an LNA switched on (+20 dB
//    floor) is followed within ~2.5 s). First line loads est = L.
// 2. After every line, a lower envelope across frequency:
//      forward s[i] = min(est[i], s[i-1] + SLOPE), backward the same
//    = min_k est[i+k] + |k|*SLOPE, so narrow peaks (stations, even steady
//    carriers that pulled up their own est) don't lift it, while the
//    floor level, the CIC edge droop and front-end slopes are followed.
// 3. out = clamp(L - s[x] + BASE, 0..511), using the previous line's
//    envelope. Model (sim/... eqmodel): noise median lands ~9 units above
//    BASE, a 60-unit station keeps ~58-66.
// en = 0 passes L through unchanged (still tracking).
// Output lags input by 2 clocks; done_out = line_done delayed 3 clocks.
module floor_eq #(
    parameter [8:0]  BASE  = 9'd94,
    parameter [8:0]  SLOPE = 9'd1,
    parameter [9:0]  RISE  = 10'd512
) (
    input  wire       clk,
    input  wire       en,
    input  wire       in_we,
    input  wire [8:0] in_x,
    input  wire [8:0] in_l8,
    input  wire       line_done,
    output reg        out_we = 1'b0,
    output reg  [8:0] out_x  = 9'd0,
    output reg  [8:0] out_l8 = 9'd0,
    output wire       done_out
);
    localparam [8:0] LAST = 9'd319;
    localparam [1:0] P_IDLE = 2'd0, P_FWD = 2'd1, P_BWD = 2'd2;
    reg [1:0] pass     = P_IDLE;
    reg       pass_ph  = 1'b0;
    reg [8:0] pass_i   = 9'd0;
    reg [8:0] s_prev   = 9'd0;
    reg       primed   = 1'b0;
    reg       sm_valid = 1'b0;
    reg [2:0] done_sr  = 3'b000;
    assign done_out = done_sr[2];

    reg       pend_valid = 1'b0;
    reg [8:0] pend_x = 9'd0, pend_l = 9'd0;

    // ---- per-bin estimate, 9.8 fixed point ----
    reg  [16:0] fl_mem [0:511];
    reg  [16:0] fl_dout = 17'd0;
    wire [8:0]  fl_addr = (pass != P_IDLE) ? pass_i : (in_we ? in_x : pend_x);
    wire [16:0] lvl17   = {pend_l, 8'd0};
    wire [17:0] est_up  = {1'b0, fl_dout} + {8'd0, RISE};
    wire [16:0] est_new = (!primed || lvl17 < fl_dout) ? lvl17 :
                          (est_up[17] ? 17'h1FFFF : est_up[16:0]);
    always @(posedge clk) if (pend_valid) fl_mem[fl_addr] <= est_new;
    always @(posedge clk) fl_dout <= fl_mem[fl_addr];

    // ---- envelope ----
    reg  [8:0] sm_mem [0:511];
    reg  [8:0] sm_dout = 9'd0;
    wire [9:0] s_cand  = {1'b0, s_prev} + {1'b0, SLOPE};
    wire [8:0] s_lim   = s_cand[9] ? 9'd511 : s_cand[8:0];
    wire [8:0] fwd_in  = fl_dout[16:8];
    wire [8:0] fwd_s   = (pass_i == 9'd0) ? fwd_in : ((fwd_in < s_lim) ? fwd_in : s_lim);
    wire [8:0] bwd_s   = (pass_i == LAST) ? sm_dout : ((sm_dout < s_lim) ? sm_dout : s_lim);
    wire       sm_we   = pass_ph && (pass != P_IDLE);
    wire [8:0] sm_din  = (pass == P_FWD) ? fwd_s : bwd_s;
    wire [8:0] sm_addr = (pass != P_IDLE) ? pass_i : in_x;
    always @(posedge clk) if (sm_we) sm_mem[sm_addr] <= sm_din;
    always @(posedge clk) sm_dout <= sm_mem[sm_addr];

    // ---- output ----
    wire signed [10:0] eq_raw = $signed({2'b00, pend_l}) - $signed({2'b00, sm_dout})
                              + $signed({2'b00, BASE});
    wire [8:0] eq_out = (eq_raw < 11'sd0) ? 9'd0 : (eq_raw > 11'sd511) ? 9'd511 : eq_raw[8:0];

    always @(posedge clk) begin
        pend_valid <= in_we;
        if (in_we) begin pend_x <= in_x; pend_l <= in_l8; end
        out_we <= pend_valid;
        out_x  <= pend_x;
        out_l8 <= (sm_valid && en) ? eq_out : pend_l;
        done_sr <= {done_sr[1:0], line_done};
        case (pass)
            P_IDLE:
                if (done_sr[2]) begin
                    primed <= 1'b1; pass <= P_FWD; pass_i <= 9'd0; pass_ph <= 1'b0;
                end
            P_FWD: begin
                pass_ph <= ~pass_ph;
                if (pass_ph) begin
                    s_prev <= fwd_s;
                    if (pass_i == LAST) pass <= P_BWD;
                    else                pass_i <= pass_i + 9'd1;
                end
            end
            P_BWD: begin
                pass_ph <= ~pass_ph;
                if (pass_ph) begin
                    s_prev <= bwd_s;
                    if (pass_i == 9'd0) begin pass <= P_IDLE; sm_valid <= 1'b1; end
                    else pass_i <= pass_i - 9'd1;
                end
            end
            default: pass <= P_IDLE;
        endcase
    end
endmodule
