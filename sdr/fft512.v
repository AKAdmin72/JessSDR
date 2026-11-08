// 512-point complex FFT for the spectrum display (radix-2 DIT, in place).
//
//   start  -> loads the next 512 in_valid samples, windowed (Blackman-
//             Harris 4-term, fft_win.hex Q15) into bit-reversed addresses,
//             then runs 9 stages x 256 butterflies (~11 clocks each, one
//             shared multiplier), then pulses done. ~10 ms per frame at
//             52.7 kHz input, of which the FFT itself is ~0.4 ms.
//   after done: rd_addr -> rd_re/rd_im (1 clock latency), natural order,
//             bin k = k * fs/512 (k >= 256: negative frequencies).
//
// No scaling: 18-bit input, 9 stages of growth -> |X| < 2^26.5, held in
// 28-bit signed words. Twiddles Q14 (fft_tw.hex: {cos, -sin} 16+16 bits).
module fft512 (
    input  wire               clk,
    input  wire               start,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    output reg                busy = 1'b0,
    output reg                done = 1'b0,
    input  wire [8:0]         rd_addr,
    output wire signed [27:0] rd_re,
    output wire signed [27:0] rd_im
);
    // ---- working memory (simple dual port) ----
    reg signed [27:0] mre [0:511];
    reg signed [27:0] mim [0:511];
    reg  [8:0]  raddr = 9'd0, waddr = 9'd0;
    reg         we = 1'b0;
    reg  signed [27:0] wre = 28'sd0, wim = 28'sd0;
    reg  signed [27:0] dre = 28'sd0, dim = 28'sd0;
    wire [8:0]  ra = busy ? raddr : rd_addr;
    always @(posedge clk) if (we) mre[waddr] <= wre;
    always @(posedge clk) if (we) mim[waddr] <= wim;
    always @(posedge clk) dre <= mre[ra];
    always @(posedge clk) dim <= mim[ra];
    assign rd_re = dre;
    assign rd_im = dim;

    // ---- ROMs ----
    reg [15:0] win_rom [0:511];
    reg [31:0] tw_rom  [0:255];
    initial $readmemh("fft_win.hex", win_rom);
    initial $readmemh("fft_tw.hex", tw_rom);
    reg  [8:0]  n = 9'd0;                      // load index
    reg  [7:0]  twa = 8'd0;
    reg  [15:0] w_q = 16'd0;
    reg  [31:0] tw_q = 32'd0;
    always @(posedge clk) w_q  <= win_rom[n];
    always @(posedge clk) tw_q <= tw_rom[twa];
    wire signed [15:0] wr = tw_q[31:16];
    wire signed [15:0] wi = tw_q[15:0];

    // ---- shared multiplier: 28 x 17 signed, registered ----
    reg  signed [27:0] ma = 28'sd0;
    reg  signed [16:0] mb = 17'sd0;
    reg  signed [44:0] p = 45'sd0;
    always @(posedge clk) p <= ma * mb;

    // ---- control ----
    localparam [4:0] S_IDLE = 5'd0, S_LWAIT = 5'd1, S_LM1 = 5'd2, S_LM2 = 5'd3, S_LM3 = 5'd4,
                     S_LWR = 5'd5, S_B0 = 5'd6, S_B1 = 5'd7, S_B2 = 5'd8, S_B3 = 5'd9,
                     S_B4 = 5'd10, S_B5 = 5'd11, S_B6 = 5'd12, S_B7 = 5'd13, S_B8 = 5'd14,
                     S_W1 = 5'd15, S_W2 = 5'd16;
    reg [4:0] st = S_IDLE;
    reg [3:0] stage = 4'd0;                    // 0..8
    reg [7:0] b = 8'd0;                        // butterfly 0..255
    reg signed [17:0] xi = 18'sd0, xq = 18'sd0;
    reg signed [27:0] ar = 28'sd0, ai_ = 28'sd0, br = 28'sd0, bi = 28'sd0;
    reg signed [44:0] acc = 45'sd0;
    reg signed [27:0] tr = 28'sd0, ti = 28'sd0;
    reg signed [27:0] lre = 28'sd0;

    function [8:0] bitrev9(input [8:0] v);
        bitrev9 = {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]};
    endfunction
    wire [8:0] mask = (9'd1 << stage) - 9'd1;
    wire [8:0] i1 = ({1'b0, b} & ~mask) << 1 | ({1'b0, b} & mask);
    wire [8:0] i2 = i1 | (9'd1 << stage);
    wire [7:0] tw_idx = ({1'b0, b} & mask) << (4'd8 - stage);

    always @(posedge clk) begin
        done <= 1'b0;
        we   <= 1'b0;
        case (st)
            S_IDLE:
                if (start) begin busy <= 1'b1; n <= 9'd0; st <= S_LWAIT; end
            // ---- load: x * window -> bitrev(n) ----
            S_LWAIT:
                if (in_valid) begin xi <= in_i; xq <= in_q; st <= S_LM1; end
            S_LM1: begin ma <= xi; mb <= {1'b0, w_q}; st <= S_LM2; end
            S_LM2: begin ma <= xq; st <= S_LM3; end
            S_LM3: begin lre <= p >>> 15; st <= S_LWR; end           // p = xi*w
            S_LWR: begin
                we <= 1'b1; waddr <= bitrev9(n); wre <= lre; wim <= p >>> 15;
                n <= n + 9'd1;
                if (n == 9'd511) begin stage <= 4'd0; b <= 8'd0; st <= S_B0; end
                else st <= S_LWAIT;
            end
            // ---- butterflies ----
            S_B0: begin raddr <= i1; twa <= tw_idx; st <= S_B1; end
            S_B1: begin raddr <= i2; st <= S_B2; end
            S_B2: begin ar <= dre; ai_ <= dim; st <= S_B3; end       // M[i1]
            S_B3: begin br <= dre; bi <= dim;                        // M[i2]
                        ma <= dre; mb <= wr; st <= S_B4; end         // br*wr
            S_B4: begin ma <= bi; mb <= wi; st <= S_B5; end          // bi*wi
            S_B5: begin acc <= p; ma <= br; mb <= wi; st <= S_B6; end // p = br*wr
            S_B6: begin tr <= (acc - p) >>> 14; ma <= bi; mb <= wr; st <= S_B7; end
            S_B7: begin acc <= p; st <= S_B8; end                    // p = br*wi
            S_B8: begin ti <= (acc + p) >>> 14; st <= S_W1; end      // p = bi*wr
            S_W1: begin
                we <= 1'b1; waddr <= i1; wre <= ar + tr; wim <= ai_ + ti; st <= S_W2;
            end
            S_W2: begin
                we <= 1'b1; waddr <= i2; wre <= ar - tr; wim <= ai_ - ti;
                b <= b + 8'd1;
                if (b == 8'd255) begin
                    if (stage == 4'd8) begin busy <= 1'b0; done <= 1'b1; st <= S_IDLE; end
                    else begin stage <= stage + 4'd1; st <= S_B0; end
                end else st <= S_B0;
            end
            default: st <= S_IDLE;
        endcase
    end
endmodule
