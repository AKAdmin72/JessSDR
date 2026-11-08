// 8N1 UART receiver, DIV = clk / baud. Samples mid-bit; `valid` is a
// 1-cycle pulse with the received byte in `data`.
module uart_rx #(
    parameter integer DIV = 65
) (
    input  wire       clk,
    input  wire       rx,
    output reg  [7:0] data = 8'd0,
    output reg        valid = 1'b0
);
    reg [2:0]  sync = 3'b111;
    reg [15:0] cnt = 16'd0;
    reg [3:0]  bitn = 4'd0;          // 0 idle, 1 start, 2..9 data, 10 stop
    wire       r = sync[2];
    wire       half = (cnt == DIV / 2 - 1);
    wire       full = (cnt == DIV - 1);
    always @(posedge clk) sync <= {sync[1:0], rx};
    always @(posedge clk)
        if (bitn == 4'd0) cnt <= 16'd0;
        else if ((bitn == 4'd1) ? half : full) cnt <= 16'd0;
        else cnt <= cnt + 16'd1;
    always @(posedge clk)
        if (bitn == 4'd0) begin if (!r) bitn <= 4'd1; end
        else if (bitn == 4'd1) begin if (half) bitn <= r ? 4'd0 : 4'd2; end   // false start -> idle
        else if (full) bitn <= (bitn == 4'd10) ? 4'd0 : bitn + 4'd1;
    always @(posedge clk)
        if (full && bitn >= 4'd2 && bitn <= 4'd9) data <= {r, data[7:1]};
    always @(posedge clk)
        valid <= full && (bitn == 4'd10) && r;
endmodule
