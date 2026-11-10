`timescale 1ns/1ns
// Self-checking testbench for spi_regs.v: a bit-banged SPI master (mode 0,
// 1MHz, MSB first) reads ID/VERSION, writes and reads back SCRATCH and
// CTRL, does a 2-byte auto-increment read, and checks that a write to a
// read-only register changes nothing.
module tb_spi_regs;
    reg clk = 0;
    always #7.716 clk = ~clk;             // 64.8MHz

    reg sck = 0, cs_n = 1, mosi = 0;
    wire miso;
    wire [7:0] ctrl;
    wire [31:0] freq_bcd;
    wire freq_set;
    reg  [31:0] lo_fw = 32'h12345678;
    spi_regs dut (.clk(clk), .spi_sck(sck), .spi_cs_n(cs_n), .spi_mosi(mosi),
                  .spi_miso(miso), .ctrl(ctrl), .freq_bcd(freq_bcd), .freq_set(freq_set),
                  .mode(mode), .mode_set(mode_set), .nr(nr),
                  .lo_fw(lo_fw), .lo_shift_k(lo_k),
                  .iqc_c1(18'h2A5C7), .iqc_c2(18'h10004), .iqc_stat(2'b11), .adc_peak(8'hF3), .level(8'h9C), .btn(8'hB4));
    wire [1:0] mode;
    wire       mode_set;
    wire [3:0] nr;
    reg  signed [3:0] lo_k = 4'sd1;
    integer mode_pulses = 0;
    always @(posedge clk) if (mode_set) mode_pulses = mode_pulses + 1;

    integer set_pulses = 0;
    always @(posedge clk) if (freq_set) set_pulses = set_pulses + 1;

    integer errors = 0;
    localparam HALF = 500;                // 1MHz SCK

    // one byte out on MOSI, one byte in from MISO (sampled on the rise)
    task xfer(input [7:0] out, output [7:0] in);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = out[b];
                #HALF sck = 1; in[b] = miso;
                #HALF sck = 0;
            end
        end
    endtask

    task rd(input [6:0] a, output [7:0] v);
        reg [7:0] dummy;
        begin
            cs_n = 0; #HALF;
            xfer({1'b1, a}, dummy);
            xfer(8'h00, v);
            #HALF cs_n = 1; #(4*HALF);
        end
    endtask

    task wr(input [6:0] a, input [7:0] v);
        reg [7:0] dummy;
        begin
            cs_n = 0; #HALF;
            xfer({1'b0, a}, dummy);
            xfer(v, dummy);
            #HALF cs_n = 1; #(4*HALF);
        end
    endtask

    task expect8(input [7:0] got, input [7:0] want, input [8*24-1:0] what);
        if (got !== want) begin
            $display("FAIL: %0s = %02h, expected %02h", what, got, want);
            errors = errors + 1;
        end
    endtask

    reg [7:0] v, v2, dummy;
    integer k;
    initial begin
        $dumpfile("sim/tb_spi_regs.vcd");
        $dumpvars(0, tb_spi_regs);
        #(10*HALF);

        rd(7'h00, v); expect8(v, 8'h5A, "ID");
        rd(7'h01, v); expect8(v, 8'h06, "VERSION");
        rd(7'h7F, v); expect8(v, 8'h00, "unused reg");

        for (k = 0; k < 8; k = k + 1) begin
            wr(7'h02, 8'h3C ^ (k * 37));
            rd(7'h02, v); expect8(v, 8'h3C ^ (k * 37), "SCRATCH");
        end
        wr(7'h02, 8'hFF); rd(7'h02, v); expect8(v, 8'hFF, "SCRATCH ff");
        wr(7'h02, 8'h00); rd(7'h02, v); expect8(v, 8'h00, "SCRATCH 00");

        wr(7'h03, 8'h01); expect8(ctrl, 8'h01, "CTRL out");
        rd(7'h03, v); expect8(v, 8'h01, "CTRL");
        wr(7'h03, 8'h00); expect8(ctrl, 8'h00, "CTRL out 0");

        wr(7'h00, 8'h11); rd(7'h00, v); expect8(v, 8'h5A, "ID after write");

        // auto-increment: ID then VERSION in one transaction
        cs_n = 0; #HALF;
        xfer(8'h80, dummy); xfer(8'h00, v); xfer(8'h00, v2);
        #HALF cs_n = 1; #(4*HALF);
        expect8(v, 8'h5A, "burst byte 0"); expect8(v2, 8'h06, "burst byte 1");

        // FREQ: power-on value, then a 4-byte burst write; exactly one
        // freq_set pulse, only after the last byte
        if (freq_bcd !== 32'h07100000) begin $display("FAIL: freq init %08h", freq_bcd); errors = errors + 1; end
        cs_n = 0; #HALF;
        xfer(8'h04, dummy); xfer(8'h14, dummy); xfer(8'h07, dummy); xfer(8'h40, dummy);
        if (set_pulses != 0 || freq_bcd !== 32'h07100000) begin
            $display("FAIL: freq changed before the last byte (%08h, %0d pulses)", freq_bcd, set_pulses); errors = errors + 1;
        end
        xfer(8'h00, dummy);
        #HALF cs_n = 1; #(4*HALF);
        if (freq_bcd !== 32'h14074000) begin $display("FAIL: freq %08h, expected 14074000", freq_bcd); errors = errors + 1; end
        if (set_pulses != 1) begin $display("FAIL: %0d freq_set pulses, expected 1", set_pulses); errors = errors + 1; end
        rd(7'h05, v); expect8(v, 8'h07, "FREQ byte 1");
        rd(7'h0A, v); expect8(v, 8'h56, "LO_FW byte 2");
        rd(7'h0C, v); expect8(v, 8'h01, "LO_SHIFT_K");
        lo_k = -4'sd1; rd(7'h0C, v); expect8(v, 8'hFF, "LO_SHIFT_K -1 (sign-extended)");
        lo_k = 4'sd4;  rd(7'h0C, v); expect8(v, 8'h04, "LO_SHIFT_K 4");

        // MODE: power-on LSB, write USB then AM, one mode_set pulse per write
        expect8({6'd0, mode}, 8'h00, "MODE init");
        wr(7'h0D, 8'h01); expect8({6'd0, mode}, 8'h01, "MODE out USB");
        rd(7'h0D, v); expect8(v, 8'h01, "MODE USB");
        wr(7'h0D, 8'h02); rd(7'h0D, v); expect8(v, 8'h02, "MODE AM");
        if (mode_pulses != 2) begin $display("FAIL: %0d mode_set pulses, expected 2", mode_pulses); errors = errors + 1; end

        // IQC_C1/C2 (bits 17..2, MSB first), IQC_STAT
        rd(7'h0E, v); expect8(v, 8'hA9, "IQC_C1 hi");
        rd(7'h0F, v); expect8(v, 8'h71, "IQC_C1 lo");
        rd(7'h10, v); expect8(v, 8'h40, "IQC_C2 hi");
        rd(7'h11, v); expect8(v, 8'h01, "IQC_C2 lo");
        rd(7'h12, v); expect8(v, 8'h03, "IQC_STAT");

        // NR: 4 bits RW
        expect8({4'd0, nr}, 8'h00, "NR init");
        wr(7'h13, 8'hF6); expect8({4'd0, nr}, 8'h06, "NR out"); rd(7'h13, v); expect8(v, 8'h06, "NR");
        // PEAK: RO, straight from adc_peak
        rd(7'h14, v); expect8(v, 8'hF3, "PEAK");
        rd(7'h15, v); expect8(v, 8'h9C, "LEVEL");
        rd(7'h16, v); expect8(v, 8'hB4, "BTN");

        // MISO must be 0 during the command byte
        cs_n = 0; #HALF; xfer(8'h80, dummy); xfer(8'h00, v); #HALF cs_n = 1; #(4*HALF);
        expect8(dummy, 8'h00, "MISO in cmd byte");

        if (errors == 0) $display("PASS: spi_regs - ID, VERSION, SCRATCH, CTRL, burst read, FREQ, LO_FW, MODE, IQC, NR, PEAK");
        $finish;
    end
endmodule
