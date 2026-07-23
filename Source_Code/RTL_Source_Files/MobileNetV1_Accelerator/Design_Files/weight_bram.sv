`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// weight_bram.sv
//
// Generic single-port weight memory: synchronous write, synchronous
// (registered) read -- the canonical pattern Vivado infers as Block RAM
// rather than distributed RAM/registers. Used to hold every conv/FC
// layer's weight tensor instead of one giant flattened port.
//
// Read latency is exactly 1 cycle: rd_data reflects rd_addr from the
// PREVIOUS cycle. mac_engine.sv's S_ADDR wait state exists specifically to
// give this one cycle of settle time before the read data is consumed.
//////////////////////////////////////////////////////////////////////////////

module weight_bram #(
    parameter DW    = 8,
    parameter DEPTH = 1024,
    parameter AW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    input  wire             clk,

    input  wire              wr_en,
    input  wire [AW-1:0]     wr_addr,
    input  wire [DW-1:0]     wr_data,

    input  wire [AW-1:0]     rd_addr,
    output reg  [DW-1:0]     rd_data
);

reg [DW-1:0] mem [0:DEPTH-1];

always @(posedge clk) begin
    if (wr_en)
        mem[wr_addr] <= wr_data;
    rd_data <= mem[rd_addr];
end

endmodule
