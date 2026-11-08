// 8N1 UART transmitter. DIV = clk / baud (rounded).
// The whole frame {stop, data, start} sits in a shift register that idles
// at all-ones, so tx is a plain register bit (glitch-free).
module uart_tx #(
    parameter integer DIV = 65
) (
    input  wire       clk,
    input  wire       start,        // 1-cycle pulse, ignored while busy
    input  wire [7:0] data,
    output wire       tx,
    output wire       busy
);
    reg [9:0]  fr = 10'h3FF;
    reg [3:0]  left = 4'd0;
    reg [15:0] cnt = 16'd0;
    assign busy = (left != 4'd0);
    assign tx   = fr[0];
    wire tick = (cnt == DIV - 1);

    always @(posedge clk)
        if (!busy || tick) cnt <= 16'd0;
        else               cnt <= cnt + 16'd1;
    always @(posedge clk)
        if (!busy) begin if (start) fr <= {1'b1, data, 1'b0}; end
        else if (tick) fr <= {1'b1, fr[9:1]};
    always @(posedge clk)
        if (!busy) begin if (start) left <= 4'd10; end
        else if (tick) left <= left - 4'd1;
endmodule
