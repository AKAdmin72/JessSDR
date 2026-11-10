`timescale 1ns/1ns
// Self-checking testbench for am_demod.v.
//   (1) the magnitude is the exact integer square root: random I/Q,
//       compared with floor(sqrt(I^2+Q^2)) computed here (via the output
//       with the averager reached - checked through a DUT-internal probe);
//   (2) AM signal: carrier A=60000 mistuned by 37Hz (its phase rotates),
//       m=0.5, 1kHz tone at fs=13184Hz. After the carrier average settles,
//       the output must be the 1kHz tone with amplitude A*m (+-1%), no DC
//       (|mean| < 1% of A*m), and a residual after removing the fitted tone
//       below 0.5% of A*m (an approximate magnitude would leave a ripple at
//       8x37Hz of a few %);
//   (3) an unmodulated carrier gives ~0 out;
//   (4) out_valid exactly 23 cycles after in_valid.
module tb_am_demod;
    reg clk = 0;
    always #18.5 clk = ~clk;

    reg               vin = 0;
    reg signed [17:0] xi = 0, xq = 0;
    wire              vout;
    wire signed [17:0] y;

    am_demod dut (.clk(clk), .in_valid(vin), .in_i(xi), .in_q(xq), .out_valid(vout), .out_audio(y));

    integer errors = 0;
    integer n, lat, k;
    real PI = 3.14159265358979;
    real FS = 13184.0;
    reg  lat_bad = 0;

    task push(input integer i, input integer q);
        begin
            @(posedge clk); #1 xi = i; xq = q; vin = 1;
            @(posedge clk); #1 vin = 0;
            lat = 1;
            while (vout !== 1'b1 && lat < 40) begin @(posedge clk); #1 lat = lat + 1; end
            if (lat != 23) begin if (!lat_bad) $display("latency %0d", lat); lat_bad = 1; end
            repeat (2) @(posedge clk); #1;
        end
    endtask

    // exact floor(sqrt(v)) for v < 2^35
    function integer isqrt(input reg [35:0] v);
        reg [35:0] r;
        begin
            r = $rtoi($sqrt(v * 1.0));
            while (r * r > v) r = r - 1;
            while ((r + 1) * (r + 1) <= v) r = r + 1;
            isqrt = r;
        end
    endfunction

    real A = 60000.0, M = 0.5, FT = 1000.0, FC = 37.0;
    real si, sc, sdc, sres, amp, ph, e, fit;
    integer NS = 6000, SKIP = 3000;
    real ys [0:5999];
    reg [35:0] v;
    integer ri, rq;

    initial begin
        $dumpfile("sim/tb_am_demod.vcd");
        $dumpvars(0, tb_am_demod);
        repeat (3) @(posedge clk);

        // (1) exact root: random values incl. full scale corners
        for (k = 0; k < 300; k = k + 1) begin
            ri = (k == 0) ? -131072 : (k == 1) ? 131071 : ($random % 131072);
            rq = (k == 0) ? -131072 : (k == 1) ? -131072 : ($random % 131072);
            push(ri, rq);
            v = ri * ri + rq * rq;
            if (dut.root !== isqrt(v)) begin
                if (errors < 5) $display("FAIL: sqrt(%0d^2+%0d^2) = %0d, expected %0d", ri, rq, dut.root, isqrt(v));
                errors = errors + 1;
            end
        end

        // (2) AM, mistuned carrier
        for (n = 0; n < NS; n = n + 1) begin
            e  = A * (1.0 + M * $cos(2.0*PI*FT*n/FS));
            ph = 2.0*PI*FC*n/FS;
            push($rtoi(e * $cos(ph)), $rtoi(e * $sin(ph)));
            ys[n] = y;
        end
        si = 0; sc = 0; sdc = 0;
        for (n = SKIP; n < NS; n = n + 1) begin
            si  = si  + ys[n] * $sin(2.0*PI*FT*n/FS);
            sc  = sc  + ys[n] * $cos(2.0*PI*FT*n/FS);
            sdc = sdc + ys[n];
        end
        si = 2.0 * si / (NS - SKIP); sc = 2.0 * sc / (NS - SKIP); sdc = sdc / (NS - SKIP);
        amp = $sqrt(si*si + sc*sc);
        sres = 0;
        for (n = SKIP; n < NS; n = n + 1) begin
            fit  = si * $sin(2.0*PI*FT*n/FS) + sc * $cos(2.0*PI*FT*n/FS) + sdc;
            sres = sres + (ys[n] - fit) * (ys[n] - fit);
        end
        sres = $sqrt(sres / (NS - SKIP));
        $display("AM: tone %0.1f (expect %0.1f), DC %0.1f, residual rms %0.1f", amp, A*M, sdc, sres);
        if (amp < A*M*0.99 || amp > A*M*1.01) begin $display("FAIL: tone amplitude"); errors = errors + 1; end
        if (sdc > A*M*0.01 || sdc < -A*M*0.01) begin $display("FAIL: DC left"); errors = errors + 1; end
        if (sres > A*M*0.005) begin $display("FAIL: residual too large"); errors = errors + 1; end

        // (3) unmodulated carrier
        sres = 0;
        for (n = 0; n < 3000; n = n + 1) begin
            ph = 2.0*PI*FC*n/FS;
            push($rtoi(A * $cos(ph)), $rtoi(A * $sin(ph)));
            if (n >= 2000) sres = sres + y * y;
        end
        sres = $sqrt(sres / 1000);
        $display("carrier only: out rms %0.1f", sres);
        if (sres > A*0.002) begin $display("FAIL: carrier not removed"); errors = errors + 1; end

        if (lat_bad) begin $display("FAIL: latency != 23"); errors = errors + 1; end
        if (errors == 0) $display("PASS: am_demod - exact sqrt, AM tone, no DC, no phase ripple, carrier removed");
        else             $display("FAIL: am_demod (%0d errors)", errors);
        $finish;
    end
endmodule
