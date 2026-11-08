// 25-bit binary -> 8 BCD digits, sequential double dabble (26 cycles).
// Restarts whenever `bin` changes; `bcd` holds the last complete result.
module bin2bcd (
    input  wire        clk,
    input  wire [24:0] bin,
    output reg  [31:0] bcd = 32'd0
);
    reg [24:0] src = 25'h1FFFFFF;
    reg [24:0] sh = 25'd0;
    reg [31:0] acc = 32'd0;
    reg [4:0]  n = 5'd0;
    reg        busy = 1'b0;
    function [31:0] add3(input [31:0] v);
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1)
                add3[i*4 +: 4] = (v[i*4 +: 4] >= 4'd5) ? v[i*4 +: 4] + 4'd3 : v[i*4 +: 4];
        end
    endfunction
    wire [31:0] adj = add3(acc);
    always @(posedge clk) begin
        if (!busy) begin
            if (bin != src) begin
                src <= bin; sh <= bin; acc <= 32'd0; n <= 5'd0; busy <= 1'b1;
            end
        end else begin
            acc <= {adj[30:0], sh[24]};
            sh  <= {sh[23:0], 1'b0};
            n   <= n + 5'd1;
            if (n == 5'd24) begin
                busy <= 1'b0;
                bcd  <= {adj[30:0], sh[24]};
            end
        end
    end
endmodule
