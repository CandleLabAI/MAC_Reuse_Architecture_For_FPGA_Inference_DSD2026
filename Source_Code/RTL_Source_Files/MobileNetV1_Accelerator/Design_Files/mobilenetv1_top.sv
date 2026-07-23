`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// mobilenetv1_top.sv
//
// Scaled-down but structurally faithful MobileNetV1 accelerator:
//
//   input (CIN0 ch, HxFW)
//     -> std_conv3x3 (stem)          CIN0  -> C0
//     -> dw_sep_block 1 (DW+PW)      C0    -> C1
//     -> dw_sep_block 2 (DW+PW)      C1    -> C2
//     -> output (C2 ch, HxFW)
//
// This mirrors MobileNetV1's real topology (standard conv stem, then a
// stack of depthwise-separable blocks); the block/channel counts are kept
// small (see defaults) purely so full-network RTL simulation completes in a
// reasonable amount of time. Every parameter is exposed so the depth/width
// can be scaled up for real synthesis runs.
//
// MAC_SEL (0=baseline, 1=dsp, 2=proposed_AOR) is threaded through every
// layer, so instantiating this module three times with the three MAC_SEL
// values (see tb_mobilenetv1_compare.sv) gives a fair, whole-network
// comparison of correctness and of the toggle_count / adder_invocations /
// addition_operations instrumentation.
//
// Weight loading: one unified address-decoded port across stem + block1 +
// block2's internal weight_bram instances. Address map:
//   [0, STEM_DEPTH)                            -> stem weights
//   [STEM_DEPTH, STEM_DEPTH+B1_DEPTH)           -> block1 weights (dw+pw)
//   [STEM_DEPTH+B1_DEPTH, .. +B2_DEPTH)         -> block2 weights (dw+pw)
//////////////////////////////////////////////////////////////////////////////

module mobilenetv1_top #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H       = 4,
    parameter FW      = 4,
    parameter CIN0    = 3,   // e.g. RGB
    parameter C0      = 4,   // stem output channels
    parameter C1      = 8,   // after block 1
    parameter C2      = 16,  // after block 2
    parameter MAC_SEL = 2,

    parameter integer STEM_DEPTH = C0*CIN0*9,
    parameter integer B1_DEPTH   = C0*9 + C1*C0,
    parameter integer B2_DEPTH   = C1*9 + C2*C1,
    parameter integer TOTAL_DEPTH = STEM_DEPTH + B1_DEPTH + B2_DEPTH,
    parameter WADDR_W  = (TOTAL_DEPTH > 1) ? $clog2(TOTAL_DEPTH) : 1,
    parameter STEM_AW  = (STEM_DEPTH  > 1) ? $clog2(STEM_DEPTH)  : 1,
    parameter B1_AW    = (B1_DEPTH    > 1) ? $clog2(B1_DEPTH)    : 1,
    parameter B2_AW    = (B2_DEPTH    > 1) ? $clog2(B2_DEPTH)    : 1
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN0*H*FW*DW-1:0]       img_in_flat,

    // unified weight-loading port (see address map above)
    input  wire                          w_wr_en,
    input  wire [WADDR_W-1:0]            w_wr_addr,
    input  wire [DW-1:0]                 w_wr_data,

    output wire [C2*H*FW*DW-1:0]         out_flat,
    output reg                           done,

    output reg [31:0]                    tot_toggle_count,
    output reg [31:0]                    tot_adder_invocations,
    output reg [31:0]                    tot_addition_operations
);

wire [C0*H*FW*DW-1:0] stem_out;
wire [C1*H*FW*DW-1:0] b1_out;
wire stem_done, b1_done, b2_done;
wire [31:0] stem_tog, stem_adi, stem_ado;
wire [31:0] b1_tog, b1_adi, b1_ado;
wire [31:0] b2_tog, b2_adi, b2_ado;

reg stem_start, b1_start, b2_start;

localparam S_IDLE=3'd0, S_STEM=3'd1, S_B1=3'd2, S_B2=3'd3, S_DONE=3'd4;
reg [2:0] state;

// ---- weight-loading address decode ----
localparam integer STEM_BASE_END = STEM_DEPTH;
localparam integer B1_BASE = STEM_DEPTH;
localparam integer B2_BASE = STEM_DEPTH + B1_DEPTH;

wire in_stem = (w_wr_addr < STEM_BASE_END);
wire in_b1   = (w_wr_addr >= B1_BASE) && (w_wr_addr < B2_BASE);
wire in_b2   = (w_wr_addr >= B2_BASE);

wire stem_w_wr_en = w_wr_en && in_stem;
wire b1_w_wr_en   = w_wr_en && in_b1;
wire b2_w_wr_en   = w_wr_en && in_b2;

wire [STEM_AW-1:0] stem_w_wr_addr = w_wr_addr[STEM_AW-1:0];
wire [B1_AW-1:0]   b1_w_wr_addr   = w_wr_addr - B1_BASE;
wire [B2_AW-1:0]   b2_w_wr_addr   = w_wr_addr - B2_BASE;

std_conv3x3_layer #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(CIN0), .COUT(C0), .STRIDE(1), .MAC_SEL(MAC_SEL)) u_stem (
    .clk(clk), .rst(rst), .start(stem_start),
    .act_in_flat(img_in_flat),
    .w_wr_en(stem_w_wr_en), .w_wr_addr(stem_w_wr_addr), .w_wr_data(w_wr_data),
    .act_out_flat(stem_out), .done(stem_done),
    .tot_toggle_count(stem_tog), .tot_adder_invocations(stem_adi), .tot_addition_operations(stem_ado)
);

dw_sep_block #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(C0), .COUT(C1), .MAC_SEL(MAC_SEL)) u_block1 (
    .clk(clk), .rst(rst), .start(b1_start),
    .act_in_flat(stem_out),
    .w_wr_en(b1_w_wr_en), .w_wr_addr(b1_w_wr_addr), .w_wr_data(w_wr_data),
    .act_out_flat(b1_out), .done(b1_done),
    .tot_toggle_count(b1_tog), .tot_adder_invocations(b1_adi), .tot_addition_operations(b1_ado)
);

dw_sep_block #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(C1), .COUT(C2), .MAC_SEL(MAC_SEL)) u_block2 (
    .clk(clk), .rst(rst), .start(b2_start),
    .act_in_flat(b1_out),
    .w_wr_en(b2_w_wr_en), .w_wr_addr(b2_w_wr_addr), .w_wr_data(w_wr_data),
    .act_out_flat(out_flat), .done(b2_done),
    .tot_toggle_count(b2_tog), .tot_adder_invocations(b2_adi), .tot_addition_operations(b2_ado)
);

always @(posedge clk) begin
    if (rst) begin
        state <= S_IDLE;
        stem_start <= 0; b1_start <= 0; b2_start <= 0;
        done <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        stem_start <= 0; b1_start <= 0; b2_start <= 0;
        case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    stem_start <= 1;
                    state <= S_STEM;
                end
            end
            S_STEM: begin
                if (stem_done) begin
                    b1_start <= 1;
                    tot_toggle_count        <= stem_tog;
                    tot_adder_invocations   <= stem_adi;
                    tot_addition_operations <= stem_ado;
                    state <= S_B1;
                end
            end
            S_B1: begin
                if (b1_done) begin
                    b2_start <= 1;
                    tot_toggle_count        <= tot_toggle_count        + b1_tog;
                    tot_adder_invocations   <= tot_adder_invocations   + b1_adi;
                    tot_addition_operations <= tot_addition_operations + b1_ado;
                    state <= S_B2;
                end
            end
            S_B2: begin
                if (b2_done) begin
                    tot_toggle_count        <= tot_toggle_count        + b2_tog;
                    tot_adder_invocations   <= tot_adder_invocations   + b2_adi;
                    tot_addition_operations <= tot_addition_operations + b2_ado;
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
