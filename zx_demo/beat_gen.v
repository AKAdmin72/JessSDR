// Silent "music" clock for the visuals: 50 Hz tick (like a Spectrum
// frame interrupt), 6 ticks per sixteenth step -> 125 BPM, 64 steps per
// pattern (4 bars). The pattern order gives the dance sections:
//   ptype 0 intro, 1 intro2, 2 part A, 3 part B, 4 breakdown, 5 build-up
// kick/snare are virtual drum hits (border flashes); vu_* are fake
// equalizer levels driven by those hits.
module beat_gen #(
    parameter integer CLK_KHZ = 67500
) (
    input  wire       clk,
    output reg  [5:0] step = 6'd63,
    output reg  [2:0] tick = 3'd5,
    output wire [2:0] ptype,
    output wire       intro,
    output reg  [3:0] vu0 = 4'd0,
    output reg  [3:0] vu1 = 4'd0,
    output reg  [3:0] vu2 = 4'd0,
    output reg        kick_hit = 1'b0,    // 1-cycle pulses
    output reg        snare_hit = 1'b0
);
    localparam integer TICK_DIV = CLK_KHZ * 20;   // clocks per 20 ms

    reg [21:0] tcnt = 22'd0;
    reg        tick_ev = 1'b0, step_ev = 1'b0;
    always @(posedge clk) tcnt <= (tcnt == TICK_DIV - 1) ? 22'd0 : tcnt + 22'd1;
    always @(posedge clk) tick_ev <= (tcnt == TICK_DIV - 1);
    always @(posedge clk) step_ev <= tick_ev && (tick == 3'd5);
    always @(posedge clk)
        if (tick_ev) tick <= (tick == 3'd5) ? 3'd0 : tick + 3'd1;
    always @(posedge clk)
        if (tick_ev && tick == 3'd5) step <= step + 6'd1;

    localparam [3:0] ORDER_LEN = 4'd10, LOOP_TO = 4'd2;
    reg [3:0] ord = 4'd15;               // 15 = before the start
    always @(posedge clk)
        if (tick_ev && tick == 3'd5 && step == 6'd63)
            ord <= (ord == 4'd15) ? 4'd0 : (ord == ORDER_LEN - 4'd1) ? LOOP_TO : ord + 4'd1;
    function [2:0] order_rom(input [3:0] o);
        case (o)
            4'd0: order_rom = 3'd0; 4'd1: order_rom = 3'd1;
            4'd2: order_rom = 3'd2; 4'd3: order_rom = 3'd3;
            4'd4: order_rom = 3'd2; 4'd5: order_rom = 3'd3;
            4'd6: order_rom = 3'd4; 4'd7: order_rom = 3'd5;
            4'd8: order_rom = 3'd2; 4'd9: order_rom = 3'd3;
            default: order_rom = 3'd0;
        endcase
    endfunction
    assign ptype = order_rom(ord);
    assign intro = (ord == 4'd0) || (ord == 4'd15);

    wire [1:0] bar = step[5:4];
    wire kick_tr  = (ptype != 3'd4) && (ptype != 3'd5) && (step[1:0] == 2'd0);
    wire snare_tr = ((ptype == 3'd1) || (ptype == 3'd2) || (ptype == 3'd3)) ?
                        ((step[3:0] == 4'd4) || (step[3:0] == 4'd12) ||
                         ((ptype != 3'd2) && bar == 2'd3 && step[3] && !step[0])) :    // fill
                    (ptype == 3'd5) ?                                                   // snare roll
                        ((bar < 2'd2) ? (step[1:0] == 2'd0) : (bar == 2'd2) ? !step[0] : 1'b1) :
                        1'b0;
    wire hat_tr   = (ptype != 3'd4) && (step[1:0] == 2'd2);

    reg [15:0] lfsr = 16'hACE1;
    always @(posedge clk) if (step_ev) lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

    function [3:0] dec_sat(input [3:0] v, input [3:0] d);
        dec_sat = (v > d) ? v - d : 4'd0;
    endfunction

    always @(posedge clk) kick_hit  <= step_ev && kick_tr;
    always @(posedge clk) snare_hit <= step_ev && snare_tr;
    always @(posedge clk)
        if (step_ev && kick_tr) vu0 <= 4'd15;
        else if (tick_ev) vu0 <= dec_sat(vu0, 4'd2);
    always @(posedge clk)
        if (step_ev && snare_tr) vu1 <= 4'd14;
        else if (step_ev && hat_tr) vu1 <= 4'd8;
        else if (tick_ev) vu1 <= dec_sat(vu1, 4'd1);
    always @(posedge clk)                                 // "melody": random level each step
        if (step_ev && ptype != 3'd0) vu2 <= 4'd7 + {1'b0, lfsr[2:0]};
        else if (tick_ev) vu2 <= dec_sat(vu2, 4'd1);
endmodule
