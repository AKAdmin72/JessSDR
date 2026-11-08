// Behavioral stand-in for Gowin's rPLL primitive, iverilog only (never in
// build.tcl): passes CLKIN straight through and reports lock immediately.
// Enough for tb_stage1_top.v, which checks the display/DSP side - the LO
// itself is verified at its real rate by tb_lo_nco.v.
module rPLL #(
    parameter FCLKIN = "100.0", parameter DYN_IDIV_SEL = "false", parameter IDIV_SEL = 0,
    parameter DYN_FBDIV_SEL = "false", parameter FBDIV_SEL = 0,
    parameter DYN_ODIV_SEL = "false", parameter ODIV_SEL = 8,
    parameter PSDA_SEL = "0000", parameter DYN_DA_EN = "false", parameter DUTYDA_SEL = "1000",
    parameter CLKOUT_FT_DIR = 1'b1, parameter CLKOUTP_FT_DIR = 1'b1,
    parameter CLKOUT_DLY_STEP = 0, parameter CLKOUTP_DLY_STEP = 0,
    parameter CLKFB_SEL = "internal",
    parameter CLKOUT_BYPASS = "false", parameter CLKOUTP_BYPASS = "false", parameter CLKOUTD_BYPASS = "false",
    parameter DYN_SDIV_SEL = 2, parameter CLKOUTD_SRC = "CLKOUT", parameter CLKOUTD3_SRC = "CLKOUT",
    parameter DEVICE = "GW1NR-9C"
) (
    output wire CLKOUT, output wire CLKOUTP, output wire CLKOUTD, output wire CLKOUTD3,
    output wire LOCK,
    input  wire CLKIN, input wire CLKFB, input wire RESET, input wire RESET_P,
    input  wire [5:0] FBDSEL, input wire [5:0] IDSEL, input wire [5:0] ODSEL,
    input  wire [3:0] PSDA, input wire [3:0] DUTYDA, input wire [3:0] FDLY
);
    assign CLKOUT = CLKIN;
    assign CLKOUTP = CLKIN;
    assign CLKOUTD = CLKIN;
    assign CLKOUTD3 = CLKIN;
    assign LOCK = 1'b1;
endmodule
