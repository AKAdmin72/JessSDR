// AD9226 at full speed (64.8 MSPS) + sample dump over UART.
//
// Clocking: rPLL 27 MHz -> CLKOUT 64.8 MHz (system + capture clock) and
// CLKOUTP (same frequency, dynamic phase PSDA in 1/16-period steps, 50%
// duty kept via DUTYDA = PSDA + 8) -> ADC CLK pin. The capture registers
// sample the bus on CLKOUT's rising edge; the right phase depends on the
// AD9226 output delay + wiring and is found by tools/adc65.py (phase sweep).
//
// UART (pins 69 TX / 70 RX via the BL616, 8N1, BAUD) commands:
//   'c'          capture NSAMP consecutive samples, then send
//                'A' 'D' N_lo N_hi phase, then N x 2 bytes little-endian:
//                bits 11:0 = ADC code (offset binary), bit 15 = OTR
//   'p' <byte>   set ADC clock phase (low 4 bits), reply 'P' phase
//   '?'          reply 'I' phase CLK_MHZ VERSION
module adc65_top #(
    parameter integer CLK_KHZ    = 64800,
    parameter integer BAUD       = 1000000,
    parameter [3:0]   PHASE_INIT = 4'd13,     // sweep 2026-11-03: errors only at 5-6
    parameter integer NSAMP_LOG2 = 14
) (
    input  wire        clk,               // 27 MHz
    output wire        adc_clk,
    input  wire [11:0] adc_d,
    input  wire        adc_otr,
    output wire        uart_tx,
    input  wire        uart_rx
);
    localparam integer DIV   = (CLK_KHZ * 1000 + BAUD / 2) / BAUD;
    localparam integer NSAMP = 1 << NSAMP_LOG2;
    localparam [7:0]   VERSION = 8'd1;

    // ---------------- PLL ----------------
    reg  [3:0] phase = PHASE_INIT;
    wire       sclk, pll_lock;
    rPLL u_pll (
        .CLKOUT(sclk), .LOCK(pll_lock), .CLKOUTP(adc_clk), .CLKOUTD(), .CLKOUTD3(),
        .RESET(1'b0), .RESET_P(1'b0), .CLKIN(clk), .CLKFB(1'b0),
        .FBDSEL(6'd0), .IDSEL(6'd0), .ODSEL(6'd0),
        .PSDA(phase), .DUTYDA(phase + 4'd8), .FDLY(4'b1111)
    );
    defparam u_pll.FCLKIN        = "27";
    defparam u_pll.DYN_IDIV_SEL  = "false";
    defparam u_pll.IDIV_SEL      = 4;          // 27/5 = 5.4 MHz PFD
    defparam u_pll.DYN_FBDIV_SEL = "false";
    defparam u_pll.FBDIV_SEL     = 11;         // x12 = 64.8 MHz
    defparam u_pll.DYN_ODIV_SEL  = "false";
    defparam u_pll.ODIV_SEL      = 16;         // VCO 1036.8 MHz
    defparam u_pll.PSDA_SEL      = "0000";
    defparam u_pll.DYN_DA_EN     = "true";
    defparam u_pll.DUTYDA_SEL    = "1000";
    defparam u_pll.CLKOUT_FT_DIR = 1'b1;
    defparam u_pll.CLKOUTP_FT_DIR = 1'b1;
    defparam u_pll.CLKOUT_DLY_STEP = 0;
    defparam u_pll.CLKOUTP_DLY_STEP = 0;
    defparam u_pll.CLKFB_SEL     = "internal";
    defparam u_pll.CLKOUT_BYPASS = "false";
    defparam u_pll.CLKOUTP_BYPASS = "false";
    defparam u_pll.CLKOUTD_BYPASS = "false";
    defparam u_pll.DYN_SDIV_SEL  = 2;
    defparam u_pll.CLKOUTD_SRC   = "CLKOUT";
    defparam u_pll.CLKOUTD3_SRC  = "CLKOUT";
    defparam u_pll.DEVICE        = "GW2A-18C";

    // ---------------- ADC input registers ----------------
    reg [11:0] d_q = 12'd0;
    reg        otr_q = 1'b0;
    always @(posedge sclk) d_q   <= adc_d;
    always @(posedge sclk) otr_q <= adc_otr;

    // ---------------- sample buffer (BSRAM, simple dual port) ----------------
    reg  [NSAMP_LOG2-1:0] wa = 0, ra = 0;
    reg                   we = 1'b0;
    reg  [12:0]           mem [0:NSAMP-1];
    reg  [12:0]           rd = 13'd0;
    always @(posedge sclk) begin
        if (we)
            mem[wa] <= {otr_q, d_q};
        rd <= mem[ra];
    end

    // ---------------- UART ----------------
    wire [7:0] rx_data;
    wire       rx_valid;
    uart_rx #(.DIV(DIV)) u_rx (.clk(sclk), .rx(uart_rx), .data(rx_data), .valid(rx_valid));
    reg        tx_start = 1'b0;
    reg  [7:0] tx_data = 8'd0;
    wire       tx_busy;
    uart_tx #(.DIV(DIV)) u_tx (.clk(sclk), .start(tx_start), .data(tx_data), .tx(uart_tx), .busy(tx_busy));

    // reply/header bytes: kind 0 = capture header, 1 = phase ack, 2 = info
    function [7:0] hdr_byte(input [1:0] kind, input [2:0] i);
        case ({kind, i})
            {2'd0, 3'd0}: hdr_byte = "A";
            {2'd0, 3'd1}: hdr_byte = "D";
            {2'd0, 3'd2}: hdr_byte = NSAMP[7:0];
            {2'd0, 3'd3}: hdr_byte = NSAMP[15:8];
            {2'd0, 3'd4}: hdr_byte = {4'd0, phase};
            {2'd1, 3'd0}: hdr_byte = "P";
            {2'd1, 3'd1}: hdr_byte = {4'd0, phase};
            {2'd2, 3'd0}: hdr_byte = "I";
            {2'd2, 3'd1}: hdr_byte = {4'd0, phase};
            {2'd2, 3'd2}: hdr_byte = CLK_KHZ / 1000;
            {2'd2, 3'd3}: hdr_byte = VERSION;
            default:      hdr_byte = 8'd0;
        endcase
    endfunction
    function [2:0] hdr_len(input [1:0] kind);
        hdr_len = (kind == 2'd0) ? 3'd5 : (kind == 2'd1) ? 3'd2 : 3'd4;
    endfunction

    // ---------------- command FSM ----------------
    localparam S_IDLE = 3'd0, S_PHASE = 3'd1, S_CAP = 3'd2, S_HDR = 3'd3,
               S_RD = 3'd4, S_LO = 3'd5, S_HI = 3'd6;
    reg [2:0] state = S_IDLE;
    reg [1:0] kind = 2'd0;
    reg [2:0] hi = 3'd0;
    reg       sent = 1'b0;      // byte handed to uart_tx, waiting for it to finish
    reg [1:0] rd_wait = 2'd0;

    // send helper: issue once, advance when the transmitter is idle again
    wire can_issue = !tx_busy && !sent;
    wire byte_done = sent && !tx_busy && !tx_start;

    always @(posedge sclk) begin
        tx_start <= 1'b0;
        we       <= 1'b0;
        case (state)
            S_IDLE:
                if (rx_valid) begin
                    if (rx_data == "c") begin
                        wa    <= 0;
                        we    <= 1'b1;
                        state <= S_CAP;
                    end else if (rx_data == "p") begin
                        state <= S_PHASE;
                    end else if (rx_data == "?") begin
                        kind <= 2'd2; hi <= 3'd0; state <= S_HDR;
                    end
                end
            S_PHASE:
                if (rx_valid) begin
                    phase <= rx_data[3:0];
                    kind  <= 2'd1; hi <= 3'd0; state <= S_HDR;
                end
            S_CAP: begin                               // one sample per clock
                we <= 1'b1;
                wa <= wa + 1'b1;
                if (wa == NSAMP - 1) begin
                    we    <= 1'b0;
                    kind  <= 2'd0; hi <= 3'd0; state <= S_HDR;
                end
            end
            S_HDR:
                if (can_issue) begin
                    tx_data <= hdr_byte(kind, hi); tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0;
                    if (hi == hdr_len(kind) - 3'd1) begin
                        ra      <= 0;
                        rd_wait <= 2'd2;
                        state   <= (kind == 2'd0) ? S_RD : S_IDLE;
                    end else
                        hi <= hi + 3'd1;
                end
            S_RD:                                      // BSRAM read latency
                if (rd_wait != 2'd0) rd_wait <= rd_wait - 2'd1;
                else state <= S_LO;
            S_LO:
                if (can_issue) begin
                    tx_data <= rd[7:0]; tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0; state <= S_HI;
                end
            S_HI:
                if (can_issue) begin
                    tx_data <= {rd[12], 3'd0, rd[11:8]}; tx_start <= 1'b1; sent <= 1'b1;
                end else if (byte_done) begin
                    sent <= 1'b0;
                    if (ra == NSAMP - 1) state <= S_IDLE;
                    else begin
                        ra      <= ra + 1'b1;
                        rd_wait <= 2'd2;
                        state   <= S_RD;
                    end
                end
            default: state <= S_IDLE;
        endcase
    end
endmodule
