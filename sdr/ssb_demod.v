// LSB demodulator: zero-IF I/Q at fs = 52.734kHz in, real audio at
// fs/4 = 13.184kHz out. Fed from the same I/Q pair as the FFT (after
// hum_hpf and IQ_SWAP, tuned frequency at 0Hz, positive = above the dial).
//
//   1. Decimate by 4: 63-tap Blackman-windowed-sinc low-pass (fc 6kHz) on I
//      and Q, computed only for every 4th input. Everything that would
//      alias into +-3.3kHz is down >87dB.
//   2. Sideband selection + audio filter in one: a 127-tap COMPLEX FIR, the
//      low-pass prototype (fc 1350Hz) shifted to -1500Hz, so it passes
//      -2700..-300Hz (LSB, within 0.5dB) and rejects the upper sideband
//      +300..+2700Hz by 76-96dB. The real part of its output is the audio:
//        audio = sum(h_re[k]*I[n-k]) - sum(h_im[k]*Q[n-k])
//      A tone at dial-f comes out as audio at f.
//
// Coefficients are Q1.17 (18-bit signed), generated offline (Python,
// Blackman window) - the ROM tables below. Both FIRs run on the same two
// 18x18 multipliers (one per I/Q lane), one tap per clock, pipelined:
// issue (history address) -> BSRAM read + coefficient -> multiply ->
// accumulate. Histories are two circular buffers in BSRAM ({I,Q} per
// word): 64 deep at 52.7kHz, 128 deep at 13.2kHz.
//
// Timing: every 4th input triggers ~200 cycles of work (63 + 127 taps +
// pipeline drain); inputs arriving while busy are dropped, so inputs must
// be >= ~210 cycles apart (real pairs arrive every 512).
// out_valid: single-cycle pulse per audio sample.
// dec_valid/dec_i/dec_q: the decimated complex stream (13.184kHz, +-6kHz
// wide) for am_demod.v; dec_valid pulses once per decimated sample, the
// values stay until the next one.
module ssb_demod (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    output reg                out_valid = 1'b0,
    output reg  signed [17:0] out_audio = 18'sd0,
    output reg                dec_valid = 1'b0,
    output reg  signed [17:0] dec_i = 18'sd0,
    output reg  signed [17:0] dec_q = 18'sd0
);

    function signed [17:0] dec_coef(input [5:0] k);
        case (k)
            6'd0: dec_coef =  18'sd0;
            6'd1: dec_coef =  18'sd1;
            6'd2: dec_coef =  18'sd5;
            6'd3: dec_coef =  18'sd12;
            6'd4: dec_coef =  18'sd11;
            6'd5: dec_coef = -18'sd10;
            6'd6: dec_coef = -18'sd52;
            6'd7: dec_coef = -18'sd91;
            6'd8: dec_coef = -18'sd87;
            6'd9: dec_coef = -18'sd3;
            6'd10: dec_coef =  18'sd153;
            6'd11: dec_coef =  18'sd311;
            6'd12: dec_coef =  18'sd347;
            6'd13: dec_coef =  18'sd155;
            6'd14: dec_coef = -18'sd264;
            6'd15: dec_coef = -18'sd743;
            6'd16: dec_coef = -18'sd980;
            6'd17: dec_coef = -18'sd689;
            6'd18: dec_coef =  18'sd200;
            6'd19: dec_coef =  18'sd1387;
            6'd20: dec_coef =  18'sd2243;
            6'd21: dec_coef =  18'sd2063;
            6'd22: dec_coef =  18'sd492;
            6'd23: dec_coef = -18'sd2120;
            6'd24: dec_coef = -18'sd4633;
            6'd25: dec_coef = -18'sd5442;
            6'd26: dec_coef = -18'sd3149;
            6'd27: dec_coef =  18'sd2713;
            6'd28: dec_coef =  18'sd11245;
            6'd29: dec_coef =  18'sd20310;
            6'd30: dec_coef =  18'sd27237;
            6'd31: dec_coef =  18'sd29829;
            6'd32: dec_coef =  18'sd27237;
            6'd33: dec_coef =  18'sd20310;
            6'd34: dec_coef =  18'sd11245;
            6'd35: dec_coef =  18'sd2713;
            6'd36: dec_coef = -18'sd3149;
            6'd37: dec_coef = -18'sd5442;
            6'd38: dec_coef = -18'sd4633;
            6'd39: dec_coef = -18'sd2120;
            6'd40: dec_coef =  18'sd492;
            6'd41: dec_coef =  18'sd2063;
            6'd42: dec_coef =  18'sd2243;
            6'd43: dec_coef =  18'sd1387;
            6'd44: dec_coef =  18'sd200;
            6'd45: dec_coef = -18'sd689;
            6'd46: dec_coef = -18'sd980;
            6'd47: dec_coef = -18'sd743;
            6'd48: dec_coef = -18'sd264;
            6'd49: dec_coef =  18'sd155;
            6'd50: dec_coef =  18'sd347;
            6'd51: dec_coef =  18'sd311;
            6'd52: dec_coef =  18'sd153;
            6'd53: dec_coef = -18'sd3;
            6'd54: dec_coef = -18'sd87;
            6'd55: dec_coef = -18'sd91;
            6'd56: dec_coef = -18'sd52;
            6'd57: dec_coef = -18'sd10;
            6'd58: dec_coef =  18'sd11;
            6'd59: dec_coef =  18'sd12;
            6'd60: dec_coef =  18'sd5;
            6'd61: dec_coef =  18'sd1;
            6'd62: dec_coef =  18'sd0;
            default: dec_coef = 18'sd0;
        endcase
    endfunction

    function signed [17:0] lsb_coef_re(input [6:0] k);
        case (k)
            7'd0: lsb_coef_re =  18'sd0;
            7'd1: lsb_coef_re =  18'sd0;
            7'd2: lsb_coef_re =  18'sd1;
            7'd3: lsb_coef_re =  18'sd1;
            7'd4: lsb_coef_re =  18'sd0;
            7'd5: lsb_coef_re =  18'sd1;
            7'd6: lsb_coef_re =  18'sd5;
            7'd7: lsb_coef_re =  18'sd6;
            7'd8: lsb_coef_re =  18'sd0;
            7'd9: lsb_coef_re = -18'sd2;
            7'd10: lsb_coef_re =  18'sd8;
            7'd11: lsb_coef_re =  18'sd18;
            7'd12: lsb_coef_re =  18'sd9;
            7'd13: lsb_coef_re = -18'sd9;
            7'd14: lsb_coef_re = -18'sd4;
            7'd15: lsb_coef_re =  18'sd25;
            7'd16: lsb_coef_re =  18'sd32;
            7'd17: lsb_coef_re = -18'sd7;
            7'd18: lsb_coef_re = -18'sd38;
            7'd19: lsb_coef_re = -18'sd3;
            7'd20: lsb_coef_re =  18'sd50;
            7'd21: lsb_coef_re =  18'sd22;
            7'd22: lsb_coef_re = -18'sd71;
            7'd23: lsb_coef_re = -18'sd91;
            7'd24: lsb_coef_re =  18'sd7;
            7'd25: lsb_coef_re =  18'sd61;
            7'd26: lsb_coef_re = -18'sd59;
            7'd27: lsb_coef_re = -18'sd208;
            7'd28: lsb_coef_re = -18'sd154;
            7'd29: lsb_coef_re =  18'sd27;
            7'd30: lsb_coef_re =  18'sd8;
            7'd31: lsb_coef_re = -18'sd267;
            7'd32: lsb_coef_re = -18'sd416;
            7'd33: lsb_coef_re = -18'sd197;
            7'd34: lsb_coef_re =  18'sd34;
            7'd35: lsb_coef_re = -18'sd187;
            7'd36: lsb_coef_re = -18'sd635;
            7'd37: lsb_coef_re = -18'sd643;
            7'd38: lsb_coef_re = -18'sd177;
            7'd39: lsb_coef_re = -18'sd30;
            7'd40: lsb_coef_re = -18'sd609;
            7'd41: lsb_coef_re = -18'sd1140;
            7'd42: lsb_coef_re = -18'sd779;
            7'd43: lsb_coef_re = -18'sd65;
            7'd44: lsb_coef_re = -18'sd266;
            7'd45: lsb_coef_re = -18'sd1317;
            7'd46: lsb_coef_re = -18'sd1662;
            7'd47: lsb_coef_re = -18'sd655;
            7'd48: lsb_coef_re =  18'sd133;
            7'd49: lsb_coef_re = -18'sd823;
            7'd50: lsb_coef_re = -18'sd2333;
            7'd51: lsb_coef_re = -18'sd1970;
            7'd52: lsb_coef_re = -18'sd23;
            7'd53: lsb_coef_re =  18'sd367;
            7'd54: lsb_coef_re = -18'sd1995;
            7'd55: lsb_coef_re = -18'sd3744;
            7'd56: lsb_coef_re = -18'sd1596;
            7'd57: lsb_coef_re =  18'sd1812;
            7'd58: lsb_coef_re =  18'sd556;
            7'd59: lsb_coef_re = -18'sd5302;
            7'd60: lsb_coef_re = -18'sd7003;
            7'd61: lsb_coef_re =  18'sd2803;
            7'd62: lsb_coef_re =  18'sd18882;
            7'd63: lsb_coef_re =  18'sd26844;
            7'd64: lsb_coef_re =  18'sd18882;
            7'd65: lsb_coef_re =  18'sd2803;
            7'd66: lsb_coef_re = -18'sd7003;
            7'd67: lsb_coef_re = -18'sd5302;
            7'd68: lsb_coef_re =  18'sd556;
            7'd69: lsb_coef_re =  18'sd1812;
            7'd70: lsb_coef_re = -18'sd1596;
            7'd71: lsb_coef_re = -18'sd3744;
            7'd72: lsb_coef_re = -18'sd1995;
            7'd73: lsb_coef_re =  18'sd367;
            7'd74: lsb_coef_re = -18'sd23;
            7'd75: lsb_coef_re = -18'sd1970;
            7'd76: lsb_coef_re = -18'sd2333;
            7'd77: lsb_coef_re = -18'sd823;
            7'd78: lsb_coef_re =  18'sd133;
            7'd79: lsb_coef_re = -18'sd655;
            7'd80: lsb_coef_re = -18'sd1662;
            7'd81: lsb_coef_re = -18'sd1317;
            7'd82: lsb_coef_re = -18'sd266;
            7'd83: lsb_coef_re = -18'sd65;
            7'd84: lsb_coef_re = -18'sd779;
            7'd85: lsb_coef_re = -18'sd1140;
            7'd86: lsb_coef_re = -18'sd609;
            7'd87: lsb_coef_re = -18'sd30;
            7'd88: lsb_coef_re = -18'sd177;
            7'd89: lsb_coef_re = -18'sd643;
            7'd90: lsb_coef_re = -18'sd635;
            7'd91: lsb_coef_re = -18'sd187;
            7'd92: lsb_coef_re =  18'sd34;
            7'd93: lsb_coef_re = -18'sd197;
            7'd94: lsb_coef_re = -18'sd416;
            7'd95: lsb_coef_re = -18'sd267;
            7'd96: lsb_coef_re =  18'sd8;
            7'd97: lsb_coef_re =  18'sd27;
            7'd98: lsb_coef_re = -18'sd154;
            7'd99: lsb_coef_re = -18'sd208;
            7'd100: lsb_coef_re = -18'sd59;
            7'd101: lsb_coef_re =  18'sd61;
            7'd102: lsb_coef_re =  18'sd7;
            7'd103: lsb_coef_re = -18'sd91;
            7'd104: lsb_coef_re = -18'sd71;
            7'd105: lsb_coef_re =  18'sd22;
            7'd106: lsb_coef_re =  18'sd50;
            7'd107: lsb_coef_re = -18'sd3;
            7'd108: lsb_coef_re = -18'sd38;
            7'd109: lsb_coef_re = -18'sd7;
            7'd110: lsb_coef_re =  18'sd32;
            7'd111: lsb_coef_re =  18'sd25;
            7'd112: lsb_coef_re = -18'sd4;
            7'd113: lsb_coef_re = -18'sd9;
            7'd114: lsb_coef_re =  18'sd9;
            7'd115: lsb_coef_re =  18'sd18;
            7'd116: lsb_coef_re =  18'sd8;
            7'd117: lsb_coef_re = -18'sd2;
            7'd118: lsb_coef_re =  18'sd0;
            7'd119: lsb_coef_re =  18'sd6;
            7'd120: lsb_coef_re =  18'sd5;
            7'd121: lsb_coef_re =  18'sd1;
            7'd122: lsb_coef_re =  18'sd0;
            7'd123: lsb_coef_re =  18'sd1;
            7'd124: lsb_coef_re =  18'sd1;
            7'd125: lsb_coef_re =  18'sd0;
            7'd126: lsb_coef_re =  18'sd0;
            default: lsb_coef_re = 18'sd0;
        endcase
    endfunction

    function signed [17:0] lsb_coef_im(input [6:0] k);
        case (k)
            7'd0: lsb_coef_im =  18'sd0;
            7'd1: lsb_coef_im =  18'sd0;
            7'd2: lsb_coef_im =  18'sd0;
            7'd3: lsb_coef_im = -18'sd1;
            7'd4: lsb_coef_im = -18'sd1;
            7'd5: lsb_coef_im =  18'sd1;
            7'd6: lsb_coef_im =  18'sd0;
            7'd7: lsb_coef_im = -18'sd6;
            7'd8: lsb_coef_im = -18'sd8;
            7'd9: lsb_coef_im = -18'sd2;
            7'd10: lsb_coef_im =  18'sd2;
            7'd11: lsb_coef_im = -18'sd11;
            7'd12: lsb_coef_im = -18'sd28;
            7'd13: lsb_coef_im = -18'sd23;
            7'd14: lsb_coef_im = -18'sd2;
            7'd15: lsb_coef_im = -18'sd6;
            7'd16: lsb_coef_im = -18'sd46;
            7'd17: lsb_coef_im = -18'sd69;
            7'd18: lsb_coef_im = -18'sd36;
            7'd19: lsb_coef_im =  18'sd0;
            7'd20: lsb_coef_im = -18'sd40;
            7'd21: lsb_coef_im = -18'sd121;
            7'd22: lsb_coef_im = -18'sd121;
            7'd23: lsb_coef_im = -18'sd30;
            7'd24: lsb_coef_im = -18'sd3;
            7'd25: lsb_coef_im = -18'sd122;
            7'd26: lsb_coef_im = -18'sd229;
            7'd27: lsb_coef_im = -18'sd143;
            7'd28: lsb_coef_im =  18'sd17;
            7'd29: lsb_coef_im = -18'sd29;
            7'd30: lsb_coef_im = -18'sd264;
            7'd31: lsb_coef_im = -18'sd326;
            7'd32: lsb_coef_im = -18'sd72;
            7'd33: lsb_coef_im =  18'sd119;
            7'd34: lsb_coef_im = -18'sd105;
            7'd35: lsb_coef_im = -18'sd438;
            7'd36: lsb_coef_im = -18'sd309;
            7'd37: lsb_coef_im =  18'sd173;
            7'd38: lsb_coef_im =  18'sd263;
            7'd39: lsb_coef_im = -18'sd247;
            7'd40: lsb_coef_im = -18'sd550;
            7'd41: lsb_coef_im = -18'sd22;
            7'd42: lsb_coef_im =  18'sd650;
            7'd43: lsb_coef_im =  18'sd403;
            7'd44: lsb_coef_im = -18'sd430;
            7'd45: lsb_coef_im = -18'sd410;
            7'd46: lsb_coef_im =  18'sd729;
            7'd47: lsb_coef_im =  18'sd1381;
            7'd48: lsb_coef_im =  18'sd476;
            7'd49: lsb_coef_im = -18'sd544;
            7'd50: lsb_coef_im =  18'sd308;
            7'd51: lsb_coef_im =  18'sd2225;
            7'd52: lsb_coef_im =  18'sd2389;
            7'd53: lsb_coef_im =  18'sd431;
            7'd54: lsb_coef_im = -18'sd303;
            7'd55: lsb_coef_im =  18'sd2369;
            7'd56: lsb_coef_im =  18'sd5312;
            7'd57: lsb_coef_im =  18'sd4024;
            7'd58: lsb_coef_im =  18'sd257;
            7'd59: lsb_coef_im =  18'sd1536;
            7'd60: lsb_coef_im =  18'sd10833;
            7'd61: lsb_coef_im =  18'sd19746;
            7'd62: lsb_coef_im =  18'sd16391;
            7'd63: lsb_coef_im =  18'sd0;
            7'd64: lsb_coef_im = -18'sd16391;
            7'd65: lsb_coef_im = -18'sd19746;
            7'd66: lsb_coef_im = -18'sd10833;
            7'd67: lsb_coef_im = -18'sd1536;
            7'd68: lsb_coef_im = -18'sd257;
            7'd69: lsb_coef_im = -18'sd4024;
            7'd70: lsb_coef_im = -18'sd5312;
            7'd71: lsb_coef_im = -18'sd2369;
            7'd72: lsb_coef_im =  18'sd303;
            7'd73: lsb_coef_im = -18'sd431;
            7'd74: lsb_coef_im = -18'sd2389;
            7'd75: lsb_coef_im = -18'sd2225;
            7'd76: lsb_coef_im = -18'sd308;
            7'd77: lsb_coef_im =  18'sd544;
            7'd78: lsb_coef_im = -18'sd476;
            7'd79: lsb_coef_im = -18'sd1381;
            7'd80: lsb_coef_im = -18'sd729;
            7'd81: lsb_coef_im =  18'sd410;
            7'd82: lsb_coef_im =  18'sd430;
            7'd83: lsb_coef_im = -18'sd403;
            7'd84: lsb_coef_im = -18'sd650;
            7'd85: lsb_coef_im =  18'sd22;
            7'd86: lsb_coef_im =  18'sd550;
            7'd87: lsb_coef_im =  18'sd247;
            7'd88: lsb_coef_im = -18'sd263;
            7'd89: lsb_coef_im = -18'sd173;
            7'd90: lsb_coef_im =  18'sd309;
            7'd91: lsb_coef_im =  18'sd438;
            7'd92: lsb_coef_im =  18'sd105;
            7'd93: lsb_coef_im = -18'sd119;
            7'd94: lsb_coef_im =  18'sd72;
            7'd95: lsb_coef_im =  18'sd326;
            7'd96: lsb_coef_im =  18'sd264;
            7'd97: lsb_coef_im =  18'sd29;
            7'd98: lsb_coef_im = -18'sd17;
            7'd99: lsb_coef_im =  18'sd143;
            7'd100: lsb_coef_im =  18'sd229;
            7'd101: lsb_coef_im =  18'sd122;
            7'd102: lsb_coef_im =  18'sd3;
            7'd103: lsb_coef_im =  18'sd30;
            7'd104: lsb_coef_im =  18'sd121;
            7'd105: lsb_coef_im =  18'sd121;
            7'd106: lsb_coef_im =  18'sd40;
            7'd107: lsb_coef_im =  18'sd0;
            7'd108: lsb_coef_im =  18'sd36;
            7'd109: lsb_coef_im =  18'sd69;
            7'd110: lsb_coef_im =  18'sd46;
            7'd111: lsb_coef_im =  18'sd6;
            7'd112: lsb_coef_im =  18'sd2;
            7'd113: lsb_coef_im =  18'sd23;
            7'd114: lsb_coef_im =  18'sd28;
            7'd115: lsb_coef_im =  18'sd11;
            7'd116: lsb_coef_im = -18'sd2;
            7'd117: lsb_coef_im =  18'sd2;
            7'd118: lsb_coef_im =  18'sd8;
            7'd119: lsb_coef_im =  18'sd6;
            7'd120: lsb_coef_im =  18'sd0;
            7'd121: lsb_coef_im = -18'sd1;
            7'd122: lsb_coef_im =  18'sd1;
            7'd123: lsb_coef_im =  18'sd1;
            7'd124: lsb_coef_im =  18'sd0;
            7'd125: lsb_coef_im =  18'sd0;
            7'd126: lsb_coef_im =  18'sd0;
            default: lsb_coef_im = 18'sd0;
        endcase
    endfunction

    // ---- history buffers (BSRAM single-port sync-read pattern) ----
    reg        [5:0]  d_addr = 6'd0;
    reg               d_we   = 1'b0;
    reg        [35:0] d_din  = 36'd0;
    reg        [35:0] d_dout = 36'd0;
    reg        [35:0] d_mem [0:63];
    always @(posedge clk) begin
        if (d_we) d_mem[d_addr] <= d_din;
        d_dout <= d_mem[d_addr];
    end

    reg        [6:0]  c_addr = 7'd0;
    reg               c_we   = 1'b0;
    reg        [35:0] c_din  = 36'd0;
    reg        [35:0] c_dout = 36'd0;
    reg        [35:0] c_mem [0:127];
    always @(posedge clk) begin
        if (c_we) c_mem[c_addr] <= c_din;
        c_dout <= c_mem[c_addr];
    end

    // ---- control ----
    localparam [2:0] S_IDLE = 3'd0, S_DEC = 3'd1, S_DEC_DRAIN = 3'd2,
                     S_CF = 3'd3, S_CF_DRAIN = 3'd4;
    reg [2:0] state = S_IDLE;
    reg [1:0] phase = 2'd0;        // input count mod 4
    reg [5:0] d_wp  = 6'd0;        // next d_mem write slot
    reg [6:0] c_wp  = 7'd0;        // next c_mem write slot
    reg [5:0] d_new = 6'd0;        // slot of the newest d_mem sample
    reg [6:0] c_new = 7'd0;        // slot of the newest c_mem sample
    reg [6:0] k     = 7'd0;        // tap being issued
    reg       mode_cf = 1'b0;      // 0 = decimator taps, 1 = LSB taps

    // pipeline: v0 issue -> v1 data+coef ready -> v2 products ready
    reg        v0 = 1'b0, v1 = 1'b0, v2 = 1'b0;
    reg  [6:0] k0 = 7'd0;
    reg signed [17:0] coef_a = 18'sd0, coef_b = 18'sd0;
    reg signed [35:0] p_a = 36'sd0, p_b = 36'sd0;
    reg signed [43:0] acc_a = 44'sd0, acc_b = 44'sd0;

    wire [35:0] hist = mode_cf ? c_dout : d_dout;
    wire signed [17:0] hist_i = hist[35:18];
    wire signed [17:0] hist_q = hist[17:0];

    // round(acc >> 17), saturated to 18 bits
    function signed [17:0] scale_sat(input signed [43:0] a);
        reg signed [43:0] r;
        begin
            r = (a + 44'sd65536) >>> 17;
            if (r > 44'sd131071)       scale_sat =  18'sd131071;
            else if (r < -44'sd131072) scale_sat = -18'sd131072;
            else                       scale_sat = r[17:0];
        end
    endfunction

    wire signed [17:0] dec_i_out = scale_sat(acc_a);
    wire signed [17:0] dec_q_out = scale_sat(acc_b);

    always @(posedge clk) begin
        d_we      <= 1'b0;
        c_we      <= 1'b0;
        out_valid <= 1'b0;
        dec_valid <= 1'b0;
        v0        <= 1'b0;

        case (state)
            S_IDLE: begin
                if (in_valid) begin
                    d_addr <= d_wp;
                    d_din  <= {in_i, in_q};
                    d_we   <= 1'b1;
                    d_new  <= d_wp;
                    d_wp   <= d_wp + 6'd1;
                    phase  <= phase + 2'd1;
                    if (phase == 2'd3) begin
                        mode_cf <= 1'b0;
                        k       <= 7'd0;
                        acc_a   <= 44'sd0;
                        acc_b   <= 44'sd0;
                        state   <= S_DEC;
                    end
                end
            end
            S_DEC: begin
                d_addr <= d_new - k[5:0];
                k0     <= k;
                v0     <= 1'b1;
                if (k == 7'd62) state <= S_DEC_DRAIN;
                else            k     <= k + 7'd1;
            end
            S_DEC_DRAIN: begin
                if (!v0 && !v1 && !v2) begin
                    // decimated sample -> LSB filter history
                    c_addr  <= c_wp;
                    c_din   <= {dec_i_out, dec_q_out};
                    c_we    <= 1'b1;
                    dec_i     <= dec_i_out;
                    dec_q     <= dec_q_out;
                    dec_valid <= 1'b1;
                    c_new   <= c_wp;
                    c_wp    <= c_wp + 7'd1;
                    mode_cf <= 1'b1;
                    k       <= 7'd0;
                    acc_a   <= 44'sd0;
                    state   <= S_CF;
                end
            end
            S_CF: begin
                c_addr <= c_new - k;
                k0     <= k;
                v0     <= 1'b1;
                if (k == 7'd126) state <= S_CF_DRAIN;
                else             k     <= k + 7'd1;
            end
            S_CF_DRAIN: begin
                if (!v0 && !v1 && !v2) begin
                    out_audio <= dec_i_out;   // same round/saturate of acc_a
                    out_valid <= 1'b1;
                    state     <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
        endcase

        // ---- MAC pipeline (runs alongside the FSM) ----
        v1 <= v0;
        if (v0) begin
            coef_a <= mode_cf ? lsb_coef_re(k0) : dec_coef(k0[5:0]);
            coef_b <= mode_cf ? lsb_coef_im(k0) : dec_coef(k0[5:0]);
        end
        v2 <= v1;
        if (v1) begin
            p_a <= hist_i * coef_a;
            p_b <= hist_q * coef_b;
        end
        if (v2) begin
            if (mode_cf) acc_a <= acc_a + p_a - p_b;
            else begin
                acc_a <= acc_a + p_a;
                acc_b <= acc_b + p_b;
            end
        end
    end

endmodule
