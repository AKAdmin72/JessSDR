// AM envelope detector: decimated complex I/Q (ssb_demod.v dec_* outputs,
// 13.184kHz, carrier shifted to 0Hz by audio_freq_shift) in, audio out.
//
//   1. mag = sqrt(I^2 + Q^2), EXACT (floor of the integer square root).
//      An alpha-max-beta-min approximation (mag_approx.v) would not do here:
//      its error depends on the carrier phase, and a carrier tuned a little
//      off 0Hz rotates, so the error would come out as a tone at 8x the
//      mistuning (a few % of the carrier - clearly audible).
//      I^2 and Q^2 on ONE 18x18 multiplier, then the classic bit-by-bit
//      (two radicand bits per step) square root, 18 steps, no multiplier.
//   2. Carrier removal: the envelope's DC is the carrier amplitude.
//      avg follows it with a one-pole low-pass (time constant 2^AVG_SHIFT
//      samples: 9 -> 39ms, corner ~4Hz), audio = mag - avg, saturated to
//      18 bits signed. Fading slower than that is removed too; audio_agc
//      handles the rest.
//
// Timing: out_valid pulses 23 cycles after in_valid; inputs must be >= 24
// cycles apart (they come every 2048).
module am_demod #(
    parameter AVG_SHIFT = 9
) (
    input  wire               clk,
    input  wire               in_valid,
    input  wire signed [17:0] in_i,
    input  wire signed [17:0] in_q,
    output reg                out_valid = 1'b0,
    output reg  signed [17:0] out_audio = 18'sd0
);

    // ---- I^2 + Q^2 ----
    reg signed [17:0] xq  = 18'sd0;
    reg signed [17:0] m_a = 18'sd0;
    reg signed [35:0] p   = 36'sd0;
    always @(posedge clk) p <= m_a * m_a;

    reg [4:0]  step = 5'd0;           // 0 = idle
    reg [35:0] sum  = 36'd0;          // I^2 + Q^2 < 2^35, then the radicand being shifted out
    reg [19:0] rem  = 20'd0;          // remainder <= 2*root < 2^19
    reg [17:0] root = 18'd0;

    wire [21:0] rem_sh = {rem, sum[35:34]};
    wire [21:0] trial  = {2'b00, root, 2'b01};
    wire        fits   = rem_sh >= trial;

    // ---- carrier average, AVG_SHIFT fractional bits ----
    localparam AW = 18 + AVG_SHIFT;
    reg  [AW-1:0] avg_acc = {AW{1'b0}};
    wire [17:0]   avg     = avg_acc[AW-1:AVG_SHIFT];
    wire signed [18:0] diff = $signed({1'b0, root}) - $signed({1'b0, avg});

    always @(posedge clk) begin
        out_valid <= 1'b0;
        case (step)
            5'd0: if (in_valid) begin
                      m_a  <= in_i;
                      xq   <= in_q;
                      step <= 5'd1;
                  end
            5'd1: begin m_a <= xq; step <= 5'd2; end           // p <= I*I
            5'd2: begin sum <= p; step <= 5'd3; end            // p <= Q*Q
            5'd3: begin
                      sum  <= sum + p;
                      rem  <= 20'd0;
                      root <= 18'd0;
                      step <= 5'd4;
                  end
            5'd22: begin                                       // root = sqrt, done
                      avg_acc   <= avg_acc + root - avg;
                      out_audio <= (diff > 19'sd131071)  ?  18'sd131071 :
                                   (diff < -19'sd131072) ? -18'sd131072 : diff[17:0];
                      out_valid <= 1'b1;
                      step      <= 5'd0;
                   end
            default: begin                                     // steps 4..21: one root bit each
                      rem  <= fits ? rem_sh[19:0] - trial[19:0] : rem_sh[19:0];
                      root <= {root[16:0], fits};
                      sum  <= sum << 2;
                      step <= step + 5'd1;
                   end
        endcase
    end

endmodule
