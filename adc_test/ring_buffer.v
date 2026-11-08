// 320-deep x signed-18-bit single-port synchronous BSRAM - holds exactly
// one capture window (matches the 320-pixel display width 1:1). Same
// single-port/1-cycle-registered-read shape as every other memory in the
// sibling sdr_spectrum_analyzer project (see e.g. column_level_buffer.v):
// if(we) mem[addr]<=din; dout<=mem[addr]; unconditionally every cycle.
// Read-first on same-address read-during-write (returns the OLD value
// that cycle).
//
// One shared addr/we/din/dout port, time-muxed at osc_top.v between
// trigger.v (writing, whenever it isn't frozen in S_CAPTURED) and
// waveform_draw.v (reading, only after capture_done) - the two never run
// concurrently by construction, so no ping-pong bank is needed here
// (unlike bin_level_buffer.v in the sibling project).
module ring_buffer (
    input  wire        clk,
    input  wire [8:0]  addr,
    input  wire        we,
    input  wire signed [17:0] din,
    output reg  signed [17:0] dout
);

    reg signed [17:0] mem [0:319];

    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];
    end

endmodule
