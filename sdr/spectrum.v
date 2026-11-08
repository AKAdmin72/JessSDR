// Spectrum / waterfall line generator around fft512.
//
// Every FFT frame, for display columns x = 0..319 (bin k = x - 160, i.e.
// -160..+159 bins of fs/512 = 102.98 Hz, the dial at x = 160):
//   |X| ~ max + 3/8 min (alpha-max-beta-min, +-0.6 dB),
//   L8 = 16*log2|X| = 8*log2(power) (msb position + 4 mantissa bits;
//        one unit = 0.376 dB), summed over NAVG frames per bin.
// After NAVG frames (~82 ms for 8) every column's mean L8 is emitted on
// line_we/line_x/line_l8 together with wf_row (the waterfall row to write);
// wf_top then advances to that row (newest line).
module spectrum #(
    parameter integer NAVG_LOG2 = 3,
    parameter integer WF_ROWS   = 102
) (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    input  wire               navg_half,           // 1: average 2^(NAVG_LOG2-1) frames
    output reg                line_we = 1'b0,
    output reg  [8:0]         line_x = 9'd0,
    output reg  [8:0]         line_l8 = 9'd0,
    output reg  [6:0]         wf_row = 7'd1,
    output reg  [6:0]         wf_top = 7'd0,
    output reg                line_done = 1'b0      // pulse after the last column
);
    reg        f_start = 1'b0;
    wire       f_busy, f_done;
    reg  [8:0] f_ra = 9'd0;
    wire signed [27:0] f_re, f_im;
    fft512 u_fft (.clk(clk), .start(f_start), .in_valid(in_valid), .in_i(in_i), .in_q(in_q),
                  .busy(f_busy), .done(f_done), .rd_addr(f_ra), .rd_re(f_re), .rd_im(f_im));

    // per-column L8 accumulator
    localparam integer AW = 9 + NAVG_LOG2;
    reg [AW-1:0] accm [0:511];
    reg [8:0]    acc_ra = 9'd0, acc_wa = 9'd0;
    reg          acc_we = 1'b0;
    reg [AW-1:0] acc_wd = 0, acc_rd = 0;
    always @(posedge clk) if (acc_we) accm[acc_wa] <= acc_wd;
    always @(posedge clk) acc_rd <= accm[acc_ra];

    localparam [3:0] S_START = 4'd0, S_WAIT = 4'd1, S_O0 = 4'd2, S_O1 = 4'd3, S_O2 = 4'd4,
                     S_O3 = 4'd5, S_O4 = 4'd6, S_O5 = 4'd7, S_O6 = 4'd8, S_O7 = 4'd9,
                     S_END = 4'd10;
    reg [3:0]  st = S_START;
    reg [8:0]  x = 9'd0;
    reg [NAVG_LOG2-1:0] frame = 0;
    reg [27:0] are = 28'd0, aim = 28'd0;
    reg [27:0] mx = 28'd0, mn = 28'd0;
    reg [28:0] mag = 29'd0;
    reg [4:0]  msb = 5'd0;
    reg [8:0]  l8 = 9'd0;

    function [4:0] msb_pos(input [28:0] v);
        integer k;
        begin
            msb_pos = 5'd0;
            for (k = 0; k < 29; k = k + 1)
                if (v[k]) msb_pos = k;
        end
    endfunction
    wire [4:0] pe = msb_pos(mag);
    wire [28:0] mshift = (msb >= 5'd4) ? (mag >> (msb - 5'd4)) : (mag << (5'd4 - msb));
    wire [NAVG_LOG2-1:0] fmask = navg_half ? ({NAVG_LOG2{1'b1}} >> 1) : {NAVG_LOG2{1'b1}};
    wire f_first = ((frame & fmask) == 0);
    wire f_last  = ((frame & fmask) == fmask);
    wire [AW-1:0] sum = (f_first ? {AW{1'b0}} : acc_rd) + {{NAVG_LOG2{1'b0}}, l8};
    wire [8:0] mean = navg_half ? sum[NAVG_LOG2-1 +: 9] : sum[NAVG_LOG2 +: 9];

    always @(posedge clk) begin
        f_start   <= 1'b0;
        acc_we    <= 1'b0;
        line_we   <= 1'b0;
        line_done <= 1'b0;
        case (st)
            S_START: begin f_start <= 1'b1; st <= S_WAIT; end
            S_WAIT:  if (f_done) begin x <= 9'd0; st <= S_O0; end
            S_O0: begin f_ra <= x + 9'd352; acc_ra <= x; st <= S_O1; end   // bin x-160
            S_O1: st <= S_O2;
            S_O2: begin
                are <= f_re[27] ? -f_re : f_re;
                aim <= f_im[27] ? -f_im : f_im;
                st <= S_O3;
            end
            S_O3: begin
                mx <= (are > aim) ? are : aim;
                mn <= (are > aim) ? aim : are;
                st <= S_O4;
            end
            S_O4: begin mag <= mx + (mn >> 2) + (mn >> 3); st <= S_O5; end
            S_O5: begin msb <= pe; st <= S_O6; end
            S_O6: begin l8 <= (mag == 29'd0) ? 9'd0 : {msb, mshift[3:0]}; st <= S_O7; end
            S_O7: begin
                acc_we <= 1'b1; acc_wa <= x; acc_wd <= sum;
                if (f_last) begin
                    line_we <= 1'b1; line_x <= x; line_l8 <= mean;
                end
                if (x == 9'd319) st <= S_END;
                else begin x <= x + 9'd1; st <= S_O0; end
            end
            S_END: begin
                if (f_last) begin
                    wf_top    <= wf_row;
                    wf_row    <= (wf_row == WF_ROWS - 1) ? 7'd0 : wf_row + 7'd1;
                    line_done <= 1'b1;
                end
                frame <= frame + 1'b1;
                st <= S_START;
            end
            default: st <= S_START;
        endcase
    end
endmodule
