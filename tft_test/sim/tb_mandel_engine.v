// Renders a 160x120 overview with mandel_engine into sim/mandel.ppm
// (same palette as mandel_top) and self-checks a few known points.
`timescale 1ns/1ps
module tb_mandel_engine;
    reg clk = 0;
    always #5 clk = ~clk;
    reg start = 0;
    reg signed [35:0] cr = 0, ci = 0;
    reg [9:0] maxit = 10'd64;
    wire done, in_set;
    wire [9:0] iter;
    mandel_engine dut (.clk(clk), .start(start), .cr(cr), .ci(ci), .maxit(maxit),
                       .busy(), .done(done), .iter(iter), .in_set(in_set));

    integer errors = 0, x, y, fd;
    real fx, fy;

    task run(input real re, input real im);
        begin
            @(negedge clk);
            cr = $rtoi(re * 1048576.0); ci = $rtoi(im * 1048576.0); cr = cr <<< 12; ci = ci <<< 12;
            start = 1; @(negedge clk); start = 0;
            @(posedge done); #1;
        end
    endtask

    function [7:0] tri8(input [7:0] v);
        tri8 = v[7] ? {~v[6:0], 1'b0} : {v[6:0], 1'b0};
    endfunction

    reg [7:0] t;
    initial begin
        run(0.0, 0.0);   if (!in_set) begin errors = errors + 1; $display("FAIL c=0"); end
        run(-1.0, 0.0);  if (!in_set) begin errors = errors + 1; $display("FAIL c=-1"); end
        run(1.0, 1.0);   if (in_set || iter > 2) begin errors = errors + 1; $display("FAIL c=1+i iter=%0d", iter); end
        run(0.5, 0.0);   if (in_set) begin errors = errors + 1; $display("FAIL c=0.5"); end
        run(-2.5, 0.0);  if (in_set || iter != 0) begin errors = errors + 1; $display("FAIL c=-2.5 iter=%0d", iter); end

        fd = $fopen("sim/mandel.ppm", "w");
        $fwrite(fd, "P3\n160 120\n255\n");
        for (y = 0; y < 120; y = y + 1)
            for (x = 0; x < 160; x = x + 1) begin
                fx = -2.2 + x * 0.02; fy = 1.2 - y * 0.02;
                run(fx, fy);
                t = iter[4:0] * 8;
                if (in_set) $fwrite(fd, "0 0 0\n");
                else $fwrite(fd, "%0d %0d %0d\n", tri8(t), tri8(t + 8'd85), tri8(t + 8'd170));
            end
        $fclose(fd);
        if (errors == 0) $display("PASS"); else $display("FAILED: %0d", errors);
        $finish;
    end
endmodule
