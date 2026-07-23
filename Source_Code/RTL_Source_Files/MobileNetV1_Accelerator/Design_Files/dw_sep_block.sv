`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// dw_sep_block.sv
//
// One MobileNetV1 "depthwise separable" block:
//   depthwise 3x3 (per-channel, optional stride-2) -> pointwise 1x1 (channel
//   mix, CIN->COUT)
//
// STRIDE=2 is used at exactly the points in the real MobileNetV1 topology
// where spatial resolution halves (4 of the 13 blocks); STRIDE=1 everywhere
// else. Both stages are built from the same mac_engine-driven layer
// modules, so the same MAC_SEL (baseline / dsp / proposed_AOR) propagates
// through every arithmetic op in the block.
//
// Weight loading: depthwise_layer and pointwise_layer each hold their
// weights in an internal weight_bram. Rather than exposing two separate
// loading ports (which would multiply into 26 ports across the full
// 13-block network), this module exposes ONE external weight-loading port
// with a flat address space: addresses [0, CIN*9) target the depthwise
// weights, addresses [CIN*9, CIN*9 + COUT*CIN) target the pointwise
// weights (offset by CIN*9). A simple combinational address decoder routes
// each write to the correct internal BRAM.
//////////////////////////////////////////////////////////////////////////////

module dw_sep_block #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H       = 4,     // input spatial height
    parameter FW      = 4,     // input spatial width
    parameter CIN     = 8,
    parameter COUT    = 16,
    parameter STRIDE  = 1,     // 1 or 2
    parameter MAC_SEL = 2,
    parameter integer OH = (H  + STRIDE - 1) / STRIDE,  // output spatial height (post depthwise stride)
    parameter integer OW = (FW + STRIDE - 1) / STRIDE,  // output spatial width

    parameter integer DW_DEPTH = CIN*9,        // depthwise weight count
    parameter integer PW_DEPTH = COUT*CIN,     // pointwise weight count
    parameter integer TOTAL_DEPTH = DW_DEPTH + PW_DEPTH,
    parameter WADDR_W = (TOTAL_DEPTH > 1) ? $clog2(TOTAL_DEPTH) : 1,
    parameter DW_AW    = (DW_DEPTH > 1) ? $clog2(DW_DEPTH) : 1,
    parameter PW_AW    = (PW_DEPTH > 1) ? $clog2(PW_DEPTH) : 1
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN*H*FW*DW-1:0]        act_in_flat,

    // unified weight-loading port (see address map above)
    input  wire                          w_wr_en,
    input  wire [WADDR_W-1:0]            w_wr_addr,
    input  wire [DW-1:0]                 w_wr_data,

    output wire [COUT*OH*OW*DW-1:0]      act_out_flat,
    output reg                           done,

    output reg [31:0]                    tot_toggle_count,
    output reg [31:0]                    tot_adder_invocations,
    output reg [31:0]                    tot_addition_operations
);

wire [CIN*OH*OW*DW-1:0] dw_out_flat;
wire dw_done, pw_done;
wire [31:0] dw_tog, dw_adi, dw_ado;
wire [31:0] pw_tog, pw_adi, pw_ado;

reg dw_start, pw_start;

localparam S_IDLE=2'd0, S_DW=2'd1, S_PW=2'd2, S_DONE=2'd3;
reg [1:0] state;

// ---- weight-loading address decode ----
wire        target_pw    = (w_wr_addr >= DW_DEPTH);
wire        dw_w_wr_en   = w_wr_en && !target_pw;
wire        pw_w_wr_en   = w_wr_en &&  target_pw;
wire [DW_AW-1:0] dw_w_wr_addr = w_wr_addr[DW_AW-1:0];
wire [PW_AW-1:0] pw_w_wr_addr = w_wr_addr - DW_DEPTH;

depthwise_layer #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .C(CIN), .STRIDE(STRIDE), .MAC_SEL(MAC_SEL)) u_dw (
    .clk(clk), .rst(rst), .start(dw_start),
    .act_in_flat(act_in_flat),
    .w_wr_en(dw_w_wr_en), .w_wr_addr(dw_w_wr_addr), .w_wr_data(w_wr_data),
    .act_out_flat(dw_out_flat), .done(dw_done),
    .tot_toggle_count(dw_tog), .tot_adder_invocations(dw_adi), .tot_addition_operations(dw_ado)
);

pointwise_layer #(.DW(DW), .ACC_W(ACC_W), .H(OH), .FW(OW), .CIN(CIN), .COUT(COUT), .MAC_SEL(MAC_SEL)) u_pw (
    .clk(clk), .rst(rst), .start(pw_start),
    .act_in_flat(dw_out_flat),
    .w_wr_en(pw_w_wr_en), .w_wr_addr(pw_w_wr_addr), .w_wr_data(w_wr_data),
    .act_out_flat(act_out_flat), .done(pw_done),
    .tot_toggle_count(pw_tog), .tot_adder_invocations(pw_adi), .tot_addition_operations(pw_ado)
);

always @(posedge clk) begin
    if (rst) begin
        state <= S_IDLE;
        dw_start <= 0; pw_start <= 0;
        done <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        dw_start <= 0;
        pw_start <= 0;
        case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    dw_start <= 1;
                    state <= S_DW;
                end
            end
            S_DW: begin
                if (dw_done) begin
                    pw_start <= 1;
                    tot_toggle_count        <= dw_tog;
                    tot_adder_invocations   <= dw_adi;
                    tot_addition_operations <= dw_ado;
                    state <= S_PW;
                end
            end
            S_PW: begin
                if (pw_done) begin
                    tot_toggle_count        <= tot_toggle_count        + pw_tog;
                    tot_adder_invocations   <= tot_adder_invocations   + pw_adi;
                    tot_addition_operations <= tot_addition_operations + pw_ado;
                    state <= S_DONE;
                end
            end
            S_DONE: begin
                done <= 1;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
        endcase
    end
end

endmodule
