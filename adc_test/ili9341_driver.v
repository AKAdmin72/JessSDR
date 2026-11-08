// ILI9341 SPI driver: reset + init command table, CASET/PASET/RAMWR window
// setup, and a 16bpp pixel-streaming interface. SPI clock runs at the raw
// 27MHz oscillator rate (1 system-clock cycle per bit - see the shifter
// below for how tft_sck is gated safely). CS is asserted once at the
// start of init and held low permanently afterward (no shared bus).
//
// Windowed CASET/PASET: win_x0/win_x1/win_y0/win_y1 are latched on the
// set_window pulse (same latch-on-pulse pattern already used for
// pixel_data) and used to build the CASET/PASET byte sequence, replacing
// tft_demo's hardcoded full-screen literals - everything else (init
// sequence, race-free SPI shifter) is unchanged from the proven original.
module ili9341_driver (
    input  wire        clk,
    input  wire        start_init,
    output reg          init_done,
    input  wire        set_window,
    input  wire [8:0]  win_x0, win_x1,
    input  wire [7:0]  win_y0, win_y1,
    output reg          window_done,
    input  wire [15:0] pixel_data,
    input  wire         pixel_valid,
    output reg          pixel_ready,
    output wire         tft_sck,
    output reg          tft_mosi,
    output reg          tft_cs   = 1'b1,
    output reg          tft_dc   = 1'b0,
    output reg          tft_rst  = 1'b1
);

    reg [8:0] win_x0_r, win_x1_r;
    reg [7:0] win_y0_r, win_y1_r;

`ifdef FAST_SIM
    // Shortened delays for simulation only - real synthesis never defines FAST_SIM.
    localparam [21:0] D_10MS  = 22'd10;
    localparam [21:0] D_120MS = 22'd40;
    localparam [21:0] D_20MS  = 22'd20;
`else
    localparam [21:0] D_10MS  = 22'd270000;
    localparam [21:0] D_120MS = 22'd3240000;
    localparam [21:0] D_20MS  = 22'd540000;
`endif

    localparam
        S_IDLE        = 6'd0,
        S_RST_LOW     = 6'd1,
        S_RST_HIGH    = 6'd2,
        S_SWRESET     = 6'd3,
        S_SWRESET_DLY = 6'd4,
        S_PWCTR1      = 6'd5,
        S_PWCTR1_D    = 6'd6,
        S_PWCTR2      = 6'd7,
        S_PWCTR2_D    = 6'd8,
        S_VMCTR1      = 6'd9,
        S_VMCTR1_D0   = 6'd10,
        S_VMCTR1_D1   = 6'd11,
        S_VMCTR2      = 6'd12,
        S_VMCTR2_D    = 6'd13,
        S_MADCTL      = 6'd14,
        S_MADCTL_D    = 6'd15,
        S_COLMOD      = 6'd16,
        S_COLMOD_D    = 6'd17,
        S_SLPOUT      = 6'd18,
        S_SLPOUT_DLY  = 6'd19,
        S_DISPON      = 6'd20,
        S_DISPON_DLY  = 6'd21,
        S_INIT_DONE   = 6'd22,
        S_READY       = 6'd23,
        S_CASET       = 6'd24,
        S_CASET_D0    = 6'd25,
        S_CASET_D1    = 6'd26,
        S_CASET_D2    = 6'd27,
        S_CASET_D3    = 6'd28,
        S_PASET       = 6'd29,
        S_PASET_D0    = 6'd30,
        S_PASET_D1    = 6'd31,
        S_PASET_D2    = 6'd32,
        S_PASET_D3    = 6'd33,
        S_RAMWR       = 6'd34,
        S_WINDOW_DONE = 6'd35,
        S_STREAM_LOW  = 6'd36,
        S_BYTE_WAIT   = 6'd37,
        S_DELAY_WAIT  = 6'd38;

    reg [5:0]  state = S_IDLE;
    reg [5:0]  ret_state;
    reg [21:0] delay_cnt;
    reg [7:0]  spi_byte;
    reg        byte_start;
    reg [15:0] pixel_latch;

    // ---- SPI byte shifter: raw 27MHz (1 sysclk cycle per bit) ----
    // tft_sck is `sck_en & clk` - a combinational gate, not a divided/
    // registered clock, so SCK genuinely runs at the full 27MHz oscillator
    // rate (matching what worked in the reference GitHub projects) instead
    // of being capped at 27/2=13.5MHz like a same-edge-toggle FSM would give.
    //
    // Timing: with SCK effectively being a gated copy of clk, a posedge-only
    // design would update tft_mosi on the *same* edge that also serves as
    // some bit's SCK sampling edge - simulation showed this is a genuine
    // race (which value a receiver sees at that instant is not guaranteed),
    // not just a simulator artifact. Fixed by preparing each bit's mosi
    // value on the *negedge* instead: that gives a full half sysclk period
    // of guaranteed setup time before the *next* posedge (=SCK's rise)
    // samples it, with zero ambiguity. Bit rate is unchanged (still exactly
    // 1 bit per clk cycle) - only *when within* each cycle mosi updates.
    reg        sck_en = 1'b0;
    reg [7:0]  shreg;
    reg [3:0]  bitcnt;
    reg        byte_done;

    assign tft_sck = sck_en & clk;

    // Single negedge-only driver for all of sck_en/shreg/bitcnt/tft_mosi/
    // byte_done (a reg must have exactly one driving always block - a
    // separate posedge block to catch byte_start earlier was rejected by
    // Gowin's synthesizer as a multi-driver conflict on the same net, even
    // though iverilog tolerated it). byte_start itself comes from the
    // outer (posedge-clk) FSM below and is held stable for a full cycle
    // (set at one posedge issue-state, cleared at the next), so it is
    // already stable and safe to sample directly here at the negedge.
    always @(negedge clk) begin
        byte_done <= 1'b0;
        if (!sck_en) begin
            if (byte_start) begin
                shreg    <= spi_byte;
                tft_mosi <= spi_byte[7];
                bitcnt   <= 4'd8;
                sck_en   <= 1'b1;
            end
        end else begin
            if (bitcnt == 4'd1) begin
                sck_en    <= 1'b0;
                byte_done <= 1'b1;
            end else begin
                shreg    <= {shreg[6:0], 1'b0};
                tft_mosi <= shreg[6];
                bitcnt   <= bitcnt - 4'd1;
            end
        end
    end

    // ---- Outer sequencer ----
    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                init_done <= 1'b0;
                if (start_init) begin
                    tft_cs <= 1'b0;
                    state  <= S_RST_LOW;
                end
            end
            S_RST_LOW: begin
                tft_rst   <= 1'b0;
                delay_cnt <= D_10MS;
                ret_state <= S_RST_HIGH;
                state     <= S_DELAY_WAIT;
            end
            S_RST_HIGH: begin
                tft_rst   <= 1'b1;
                delay_cnt <= D_120MS;
                ret_state <= S_SWRESET;
                state     <= S_DELAY_WAIT;
            end

            S_SWRESET:   begin tft_dc<=1'b0; spi_byte<=8'h01; byte_start<=1'b1; ret_state<=S_SWRESET_DLY; state<=S_BYTE_WAIT; end
            S_SWRESET_DLY: begin delay_cnt<=D_120MS; ret_state<=S_PWCTR1; state<=S_DELAY_WAIT; end

            S_PWCTR1:    begin tft_dc<=1'b0; spi_byte<=8'hC0; byte_start<=1'b1; ret_state<=S_PWCTR1_D; state<=S_BYTE_WAIT; end
            S_PWCTR1_D:  begin tft_dc<=1'b1; spi_byte<=8'h23; byte_start<=1'b1; ret_state<=S_PWCTR2;   state<=S_BYTE_WAIT; end
            S_PWCTR2:    begin tft_dc<=1'b0; spi_byte<=8'hC1; byte_start<=1'b1; ret_state<=S_PWCTR2_D; state<=S_BYTE_WAIT; end
            S_PWCTR2_D:  begin tft_dc<=1'b1; spi_byte<=8'h10; byte_start<=1'b1; ret_state<=S_VMCTR1;   state<=S_BYTE_WAIT; end
            S_VMCTR1:    begin tft_dc<=1'b0; spi_byte<=8'hC5; byte_start<=1'b1; ret_state<=S_VMCTR1_D0;state<=S_BYTE_WAIT; end
            S_VMCTR1_D0: begin tft_dc<=1'b1; spi_byte<=8'h3E; byte_start<=1'b1; ret_state<=S_VMCTR1_D1;state<=S_BYTE_WAIT; end
            S_VMCTR1_D1: begin tft_dc<=1'b1; spi_byte<=8'h28; byte_start<=1'b1; ret_state<=S_VMCTR2;   state<=S_BYTE_WAIT; end
            S_VMCTR2:    begin tft_dc<=1'b0; spi_byte<=8'hC7; byte_start<=1'b1; ret_state<=S_VMCTR2_D; state<=S_BYTE_WAIT; end
            S_VMCTR2_D:  begin tft_dc<=1'b1; spi_byte<=8'h86; byte_start<=1'b1; ret_state<=S_MADCTL;   state<=S_BYTE_WAIT; end
            S_MADCTL:    begin tft_dc<=1'b0; spi_byte<=8'h36; byte_start<=1'b1; ret_state<=S_MADCTL_D; state<=S_BYTE_WAIT; end
            // MV(bit5)=1 swaps row/col addressing so the 320-pixel side
            // becomes the CASET (column) axis - required for landscape
            // 320x240 on a physically-portrait 240x320 ILI9341 panel.
            // BGR(bit3)=1 for correct color order on most modules.
            S_MADCTL_D:  begin tft_dc<=1'b1; spi_byte<=8'h28; byte_start<=1'b1; ret_state<=S_COLMOD;   state<=S_BYTE_WAIT; end
            S_COLMOD:    begin tft_dc<=1'b0; spi_byte<=8'h3A; byte_start<=1'b1; ret_state<=S_COLMOD_D; state<=S_BYTE_WAIT; end
            S_COLMOD_D:  begin tft_dc<=1'b1; spi_byte<=8'h55; byte_start<=1'b1; ret_state<=S_SLPOUT;   state<=S_BYTE_WAIT; end
            S_SLPOUT:    begin tft_dc<=1'b0; spi_byte<=8'h11; byte_start<=1'b1; ret_state<=S_SLPOUT_DLY; state<=S_BYTE_WAIT; end
            S_SLPOUT_DLY: begin delay_cnt<=D_120MS; ret_state<=S_DISPON; state<=S_DELAY_WAIT; end
            S_DISPON:    begin tft_dc<=1'b0; spi_byte<=8'h29; byte_start<=1'b1; ret_state<=S_DISPON_DLY; state<=S_BYTE_WAIT; end
            S_DISPON_DLY: begin delay_cnt<=D_20MS; ret_state<=S_INIT_DONE; state<=S_DELAY_WAIT; end

            S_INIT_DONE: begin
                init_done <= 1'b1;
                state     <= S_READY;
            end

            S_READY: begin
                init_done   <= 1'b0;
                window_done <= 1'b0;
                pixel_ready <= 1'b1;
                if (set_window) begin
                    pixel_ready <= 1'b0;
                    win_x0_r    <= win_x0;
                    win_x1_r    <= win_x1;
                    win_y0_r    <= win_y0;
                    win_y1_r    <= win_y1;
                    state       <= S_CASET;
                end else if (pixel_valid) begin
                    pixel_ready <= 1'b0;
                    pixel_latch <= pixel_data;
                    tft_dc      <= 1'b1;
                    spi_byte    <= pixel_data[15:8];
                    byte_start  <= 1'b1;
                    ret_state   <= S_STREAM_LOW;
                    state       <= S_BYTE_WAIT;
                end
            end

            S_CASET:    begin tft_dc<=1'b0; spi_byte<=8'h2A; byte_start<=1'b1; ret_state<=S_CASET_D0; state<=S_BYTE_WAIT; end
            S_CASET_D0: begin tft_dc<=1'b1; spi_byte<={7'd0, win_x0_r[8]}; byte_start<=1'b1; ret_state<=S_CASET_D1; state<=S_BYTE_WAIT; end
            S_CASET_D1: begin tft_dc<=1'b1; spi_byte<=win_x0_r[7:0];      byte_start<=1'b1; ret_state<=S_CASET_D2; state<=S_BYTE_WAIT; end
            S_CASET_D2: begin tft_dc<=1'b1; spi_byte<={7'd0, win_x1_r[8]}; byte_start<=1'b1; ret_state<=S_CASET_D3; state<=S_BYTE_WAIT; end
            S_CASET_D3: begin tft_dc<=1'b1; spi_byte<=win_x1_r[7:0];      byte_start<=1'b1; ret_state<=S_PASET;    state<=S_BYTE_WAIT; end
            S_PASET:    begin tft_dc<=1'b0; spi_byte<=8'h2B; byte_start<=1'b1; ret_state<=S_PASET_D0; state<=S_BYTE_WAIT; end
            S_PASET_D0: begin tft_dc<=1'b1; spi_byte<=8'h00;             byte_start<=1'b1; ret_state<=S_PASET_D1; state<=S_BYTE_WAIT; end
            S_PASET_D1: begin tft_dc<=1'b1; spi_byte<=win_y0_r;          byte_start<=1'b1; ret_state<=S_PASET_D2; state<=S_BYTE_WAIT; end
            S_PASET_D2: begin tft_dc<=1'b1; spi_byte<=8'h00;             byte_start<=1'b1; ret_state<=S_PASET_D3; state<=S_BYTE_WAIT; end
            S_PASET_D3: begin tft_dc<=1'b1; spi_byte<=win_y1_r;          byte_start<=1'b1; ret_state<=S_RAMWR;    state<=S_BYTE_WAIT; end
            S_RAMWR:    begin tft_dc<=1'b0; spi_byte<=8'h2C; byte_start<=1'b1; ret_state<=S_WINDOW_DONE; state<=S_BYTE_WAIT; end

            S_WINDOW_DONE: begin
                window_done <= 1'b1;
                state       <= S_READY;
            end

            S_STREAM_LOW: begin
                tft_dc     <= 1'b1;
                spi_byte   <= pixel_latch[7:0];
                byte_start <= 1'b1;
                ret_state  <= S_READY;
                state      <= S_BYTE_WAIT;
            end

            S_BYTE_WAIT: begin
                byte_start <= 1'b0;
                if (byte_done)
                    state <= ret_state;
            end

            S_DELAY_WAIT: begin
                if (delay_cnt == 22'd0)
                    state <= ret_state;
                else
                    delay_cnt <= delay_cnt - 22'd1;
            end

            default: state <= S_IDLE;
        endcase
    end

endmodule
