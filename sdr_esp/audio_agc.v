// JessSDR sdr_esp fork of ../sdr/audio_agc.v (2026-11-15): runtime speed
// (spi_regs 0x18 AGC) instead of the fixed HANG / release:
//   speed 0 FAST:   hang 0.1s, release g/1024 per sample (~110 dB/s)
//   speed 1 MEDIUM: hang 0.5s, release g/4096 (~28 dB/s)  = the original
//   speed 2 SLOW:   hang 1.0s, release g/8192 (~14 dB/s)
//   (3 acts as SLOW)
//
// Audio AGC: keeps the demodulated audio near a fixed loudness regardless
// of signal strength (a weak and a strong SSB station differ by 60dB+).
//
//   out  = in * g            g: unsigned Q12.12 (1.0 = 4096)
//   |out| > TARGET  -> attack:  g -= g/16  (~0.56dB per sample, a few ms)
//                      and restart the hang timer
//   hang timer runs -> hold g (no pumping between syllables)
//   hang expired    -> release: g += g/4096 + 1 (~28dB/s at 13.2kHz)
//   g clamped to [G_MIN, G_MAX] - G_MAX bounds how loud the band noise gets
//   when there is no station.
// out is saturated to 16 bits signed. Latency 2 cycles (out_valid is a
// single-cycle pulse); inputs must be >= 2 cycles apart.
module audio_agc #(
    parameter [23:0] G_MAX  = 24'd4194303,  // ~1024x (60dB)
    parameter [23:0] G_MIN  = 24'd256,      // 1/16
    parameter [15:0] TARGET = 16'd16384     // half of 16-bit full scale
) (
    input  wire               clk,
    input  wire [1:0]         speed,
    input  wire               in_valid,
    input  wire signed [17:0] in_audio,
    output reg                out_valid = 1'b0,
    output reg  signed [15:0] out_audio = 16'sd0
);

    reg        [23:0] g    = 24'd4096;
    reg        [13:0] hang = 14'd0;
    wire       [13:0] hang_len = (speed == 2'd0) ? 14'd1318 : (speed == 2'd1) ? 14'd6592 : 14'd13184;

    // stage 1: multiply
    wire signed [24:0] g_s = {1'b0, g};
    reg  signed [42:0] prod = 43'sd0;
    reg                v1   = 1'b0;
    always @(posedge clk) if (in_valid) prod <= in_audio * g_s;
    always @(posedge clk) v1 <= in_valid;

    // stage 2: scale, saturate, update gain
    wire signed [30:0] scaled = prod >>> 12;
    wire        [30:0] mag    = scaled[30] ? -scaled : scaled;
    wire               loud   = (mag > {15'd0, TARGET});

    wire [23:0] g_att = g - (g >> 4);
    wire [23:0] g_rel = g + ((speed == 2'd0) ? (g >> 10) : (speed == 2'd1) ? (g >> 12) : (g >> 13)) + 24'd1;

    always @(posedge clk) begin
        if (v1) begin
            if (loud) begin
                g    <= (g_att < G_MIN) ? G_MIN : g_att;
                hang <= hang_len;
            end else if (hang != 14'd0) begin
                hang <= hang - 14'd1;
            end else begin
                g <= (g_rel > G_MAX) ? G_MAX : g_rel;
            end
        end
    end

    always @(posedge clk) begin
        if (v1)
            out_audio <= (scaled >  31'sd32767) ?  16'sd32767 :
                         (scaled < -31'sd32768) ? -16'sd32768 : scaled[15:0];
    end
    always @(posedge clk) out_valid <= v1;

endmodule
