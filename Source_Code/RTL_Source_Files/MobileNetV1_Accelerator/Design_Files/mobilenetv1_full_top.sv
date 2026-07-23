`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// mobilenetv1_full_top.sv
//
// Full MobileNetV1 (width multiplier 1.0) topology, real channel counts:
//
//   input (CIN0, H0 x FW0)                                    [e.g. 3x224x224]
//     -> std_conv3x3 stem, stride 2       CIN0  -> 32
//     -> block 1  (dw s1, pw)             32    -> 64
//     -> block 2  (dw s2, pw)             64    -> 128
//     -> block 3  (dw s1, pw)             128   -> 128
//     -> block 4  (dw s2, pw)             128   -> 256
//     -> block 5  (dw s1, pw)             256   -> 256
//     -> block 6  (dw s2, pw)             256   -> 512
//     -> block 7  (dw s1, pw)             512   -> 512   \
//     -> block 8  (dw s1, pw)             512   -> 512    | five identical
//     -> block 9  (dw s1, pw)             512   -> 512    | 512->512 blocks
//     -> block 10 (dw s1, pw)             512   -> 512    | (paper Table 1)
//     -> block 11 (dw s1, pw)             512   -> 512   /
//     -> block 12 (dw s2, pw)             512   -> 1024
//     -> block 13 (dw s1, pw)             1024  -> 1024
//     -> global average pool + FC         1024  -> NUM_CLASSES
//
// This is a straight, non-generate-loop instantiation of the 13 blocks
// (each dw_sep_block call is ~4 lines) so the topology is easy to read and
// to hand-modify (e.g. width multiplier, different stride placement).
//
// WEIGHT LOADING: every layer (stem, each block's dw+pw, FC) holds its
// weights in an internal weight_bram rather than a flattened port. Instead
// of exposing 15+ separate loading ports, this module presents ONE
// external address-decoded weight-loading port spanning all of them:
//
//   [0, STEM_DEPTH)                       -> stem
//   [.., +B1_DEPTH)                       -> block 1 (dw+pw, decoded again inside dw_sep_block)
//   [.., +B2_DEPTH)                       -> block 2
//   ...
//   [.., +B13_DEPTH)                      -> block 13
//   [.., +FC_DEPTH)                       -> FC classifier
//
// A weight loader (e.g. driven from an AXI stream, SPI flash reader, or a
// testbench loop) writes the whole network's weights through this single
// port before pulsing start.
//
// IMPORTANT -- simulation vs synthesis scale:
//   Every parameter here defaults to MobileNetV1's REAL channel counts, as
//   requested, so this is the correct RTL to hand to Vivado synthesis (set
//   H0/FW0 to 224 for the real input size). But this design is a bit-serial,
//   one-(activation,weight)-pair-at-a-time accelerator: at real scale the
//   network totals on the order of tens of millions of MAC taps, which is
//   simply too slow to run to completion in an interactive RTL simulator
//   (would take on the order of hours). For functional / regression
//   verification, instantiate this module with H0/FW0 and the channel
//   parameters overridden to small values (see
//   tb_mobilenetv1_full_reduced_compare.sv) -- the RTL and control logic
//   are identical, only the loop trip counts shrink.
//////////////////////////////////////////////////////////////////////////////

module mobilenetv1_full_top #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H0      = 224,   // input spatial height  (real MobileNetV1: 224)
    parameter FW0     = 224,   // input spatial width
    parameter CIN0    = 3,     // RGB

    // real MobileNetV1 channel progression (override for reduced-scale sim)
    parameter C_STEM  = 32,
    parameter C1      = 64,
    parameter C2      = 128,
    parameter C3      = 128,
    parameter C4      = 256,
    parameter C5      = 256,
    parameter C6      = 512,
    parameter C7      = 512,
    parameter C8      = 512,
    parameter C9      = 512,
    parameter C10     = 512,
    parameter C11     = 512,
    parameter C12     = 1024,
    parameter C13     = 1024,

    parameter NUM_CLASSES = 1000,
    parameter MAC_SEL     = 2,

    // ---- weight address map (computed from the channel params above) ----
    parameter integer STEM_DEPTH = C_STEM*CIN0*9,
    parameter integer B1_DEPTH   = C_STEM*9 + C1*C_STEM,
    parameter integer B2_DEPTH   = C1*9     + C2*C1,
    parameter integer B3_DEPTH   = C2*9     + C3*C2,
    parameter integer B4_DEPTH   = C3*9     + C4*C3,
    parameter integer B5_DEPTH   = C4*9     + C5*C4,
    parameter integer B6_DEPTH   = C5*9     + C6*C5,
    parameter integer B7_DEPTH   = C6*9     + C7*C6,
    parameter integer B8_DEPTH   = C7*9     + C8*C7,
    parameter integer B9_DEPTH   = C8*9     + C9*C8,
    parameter integer B10_DEPTH  = C9*9     + C10*C9,
    parameter integer B11_DEPTH  = C10*9    + C11*C10,
    parameter integer B12_DEPTH  = C11*9    + C12*C11,
    parameter integer B13_DEPTH  = C12*9    + C13*C12,
    parameter integer FC_DEPTH   = NUM_CLASSES*C13,

    parameter integer BASE_STEM = 0,
    parameter integer BASE_B1   = BASE_STEM + STEM_DEPTH,
    parameter integer BASE_B2   = BASE_B1   + B1_DEPTH,
    parameter integer BASE_B3   = BASE_B2   + B2_DEPTH,
    parameter integer BASE_B4   = BASE_B3   + B3_DEPTH,
    parameter integer BASE_B5   = BASE_B4   + B4_DEPTH,
    parameter integer BASE_B6   = BASE_B5   + B5_DEPTH,
    parameter integer BASE_B7   = BASE_B6   + B6_DEPTH,
    parameter integer BASE_B8   = BASE_B7   + B7_DEPTH,
    parameter integer BASE_B9   = BASE_B8   + B8_DEPTH,
    parameter integer BASE_B10  = BASE_B9   + B9_DEPTH,
    parameter integer BASE_B11  = BASE_B10  + B10_DEPTH,
    parameter integer BASE_B12  = BASE_B11  + B11_DEPTH,
    parameter integer BASE_B13  = BASE_B12  + B12_DEPTH,
    parameter integer BASE_FC   = BASE_B13  + B13_DEPTH,
    parameter integer TOTAL_DEPTH = BASE_FC + FC_DEPTH,

    parameter WADDR_W   = (TOTAL_DEPTH > 1) ? $clog2(TOTAL_DEPTH) : 1,
    parameter STEM_AW   = (STEM_DEPTH  > 1) ? $clog2(STEM_DEPTH)  : 1,
    parameter B1_AW     = (B1_DEPTH    > 1) ? $clog2(B1_DEPTH)    : 1,
    parameter B2_AW     = (B2_DEPTH    > 1) ? $clog2(B2_DEPTH)    : 1,
    parameter B3_AW     = (B3_DEPTH    > 1) ? $clog2(B3_DEPTH)    : 1,
    parameter B4_AW     = (B4_DEPTH    > 1) ? $clog2(B4_DEPTH)    : 1,
    parameter B5_AW     = (B5_DEPTH    > 1) ? $clog2(B5_DEPTH)    : 1,
    parameter B6_AW      = (B6_DEPTH   > 1) ? $clog2(B6_DEPTH)    : 1,
    parameter B7_AW      = (B7_DEPTH   > 1) ? $clog2(B7_DEPTH)    : 1,
    parameter B8_AW      = (B8_DEPTH   > 1) ? $clog2(B8_DEPTH)    : 1,
    parameter B9_AW      = (B9_DEPTH   > 1) ? $clog2(B9_DEPTH)    : 1,
    parameter B10_AW     = (B10_DEPTH  > 1) ? $clog2(B10_DEPTH)   : 1,
    parameter B11_AW     = (B11_DEPTH  > 1) ? $clog2(B11_DEPTH)   : 1,
    parameter B12_AW     = (B12_DEPTH  > 1) ? $clog2(B12_DEPTH)   : 1,
    parameter B13_AW     = (B13_DEPTH  > 1) ? $clog2(B13_DEPTH)   : 1,
    parameter FC_AW       = (FC_DEPTH  > 1) ? $clog2(FC_DEPTH)    : 1
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN0*H0*FW0*DW-1:0]     img_in_flat,

    // unified weight-loading port (see address map above)
    input  wire                          w_wr_en,
    input  wire [WADDR_W-1:0]            w_wr_addr,
    input  wire [DW-1:0]                 w_wr_data,

    output wire [NUM_CLASSES*ACC_W-1:0]  class_scores_flat,
    output reg                           done,

    output reg [31:0]                    tot_toggle_count,
    output reg [31:0]                    tot_adder_invocations,
    output reg [31:0]                    tot_addition_operations
);

// ---- spatial size tracking through the 5 stride-2 points ----
localparam integer H_STEM = (H0     + 1) / 2;  localparam integer W_STEM = (FW0    + 1) / 2;  // stem stride2
localparam integer H_B2   = (H_STEM + 1) / 2;  localparam integer W_B2   = (W_STEM + 1) / 2;  // block2 stride2
localparam integer H_B4   = (H_B2   + 1) / 2;  localparam integer W_B4   = (W_B2   + 1) / 2;  // block4 stride2
localparam integer H_B6   = (H_B4   + 1) / 2;  localparam integer W_B6   = (W_B4   + 1) / 2;  // block6 stride2
localparam integer H_B12  = (H_B6   + 1) / 2;  localparam integer W_B12  = (W_B6   + 1) / 2;  // block12 stride2
// blocks 1,3,5,7-11,13 are stride 1 (spatial size unchanged)

wire [C_STEM*H_STEM*W_STEM*DW-1:0] stem_out;
wire [C1*H_STEM*W_STEM*DW-1:0]     b1_out;
wire [C2*H_B2*W_B2*DW-1:0]         b2_out;
wire [C3*H_B2*W_B2*DW-1:0]         b3_out;
wire [C4*H_B4*W_B4*DW-1:0]         b4_out;
wire [C5*H_B4*W_B4*DW-1:0]         b5_out;
wire [C6*H_B6*W_B6*DW-1:0]         b6_out;
wire [C7*H_B6*W_B6*DW-1:0]         b7_out;
wire [C8*H_B6*W_B6*DW-1:0]         b8_out;
wire [C9*H_B6*W_B6*DW-1:0]         b9_out;
wire [C10*H_B6*W_B6*DW-1:0]        b10_out;
wire [C11*H_B6*W_B6*DW-1:0]        b11_out;
wire [C12*H_B12*W_B12*DW-1:0]      b12_out;
wire [C13*H_B12*W_B12*DW-1:0]      b13_out;

wire stem_done, d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13, fc_done;
wire [31:0] stem_tog, stem_adi, stem_ado;
wire [31:0] b1_tog,b1_adi,b1_ado, b2_tog,b2_adi,b2_ado, b3_tog,b3_adi,b3_ado,
            b4_tog,b4_adi,b4_ado, b5_tog,b5_adi,b5_ado, b6_tog,b6_adi,b6_ado,
            b7_tog,b7_adi,b7_ado, b8_tog,b8_adi,b8_ado, b9_tog,b9_adi,b9_ado,
            b10_tog,b10_adi,b10_ado, b11_tog,b11_adi,b11_ado,
            b12_tog,b12_adi,b12_ado, b13_tog,b13_adi,b13_ado,
            fc_tog,fc_adi,fc_ado;

reg stem_start,b1_start,b2_start,b3_start,b4_start,b5_start,b6_start,
    b7_start,b8_start,b9_start,b10_start,b11_start,b12_start,b13_start,fc_start;

// ---- unified weight-loading address decode ----
wire in_stem = (w_wr_addr >= BASE_STEM) && (w_wr_addr < BASE_B1);
wire in_b1   = (w_wr_addr >= BASE_B1)   && (w_wr_addr < BASE_B2);
wire in_b2   = (w_wr_addr >= BASE_B2)   && (w_wr_addr < BASE_B3);
wire in_b3   = (w_wr_addr >= BASE_B3)   && (w_wr_addr < BASE_B4);
wire in_b4   = (w_wr_addr >= BASE_B4)   && (w_wr_addr < BASE_B5);
wire in_b5   = (w_wr_addr >= BASE_B5)   && (w_wr_addr < BASE_B6);
wire in_b6   = (w_wr_addr >= BASE_B6)   && (w_wr_addr < BASE_B7);
wire in_b7   = (w_wr_addr >= BASE_B7)   && (w_wr_addr < BASE_B8);
wire in_b8   = (w_wr_addr >= BASE_B8)   && (w_wr_addr < BASE_B9);
wire in_b9   = (w_wr_addr >= BASE_B9)   && (w_wr_addr < BASE_B10);
wire in_b10  = (w_wr_addr >= BASE_B10)  && (w_wr_addr < BASE_B11);
wire in_b11  = (w_wr_addr >= BASE_B11)  && (w_wr_addr < BASE_B12);
wire in_b12  = (w_wr_addr >= BASE_B12)  && (w_wr_addr < BASE_B13);
wire in_b13  = (w_wr_addr >= BASE_B13)  && (w_wr_addr < BASE_FC);
wire in_fc   = (w_wr_addr >= BASE_FC);

wire stem_we = w_wr_en && in_stem;
wire b1_we   = w_wr_en && in_b1;
wire b2_we   = w_wr_en && in_b2;
wire b3_we   = w_wr_en && in_b3;
wire b4_we   = w_wr_en && in_b4;
wire b5_we   = w_wr_en && in_b5;
wire b6_we   = w_wr_en && in_b6;
wire b7_we   = w_wr_en && in_b7;
wire b8_we   = w_wr_en && in_b8;
wire b9_we   = w_wr_en && in_b9;
wire b10_we  = w_wr_en && in_b10;
wire b11_we  = w_wr_en && in_b11;
wire b12_we  = w_wr_en && in_b12;
wire b13_we  = w_wr_en && in_b13;
wire fc_we   = w_wr_en && in_fc;

wire [STEM_AW-1:0] stem_addr = w_wr_addr[STEM_AW-1:0];
wire [B1_AW-1:0]   b1_addr   = w_wr_addr - BASE_B1;
wire [B2_AW-1:0]   b2_addr   = w_wr_addr - BASE_B2;
wire [B3_AW-1:0]   b3_addr   = w_wr_addr - BASE_B3;
wire [B4_AW-1:0]   b4_addr   = w_wr_addr - BASE_B4;
wire [B5_AW-1:0]   b5_addr   = w_wr_addr - BASE_B5;
wire [B6_AW-1:0]   b6_addr   = w_wr_addr - BASE_B6;
wire [B7_AW-1:0]   b7_addr   = w_wr_addr - BASE_B7;
wire [B8_AW-1:0]   b8_addr   = w_wr_addr - BASE_B8;
wire [B9_AW-1:0]   b9_addr   = w_wr_addr - BASE_B9;
wire [B10_AW-1:0]  b10_addr  = w_wr_addr - BASE_B10;
wire [B11_AW-1:0]  b11_addr  = w_wr_addr - BASE_B11;
wire [B12_AW-1:0]  b12_addr  = w_wr_addr - BASE_B12;
wire [B13_AW-1:0]  b13_addr  = w_wr_addr - BASE_B13;
wire [FC_AW-1:0]   fc_addr   = w_wr_addr - BASE_FC;

std_conv3x3_layer #(.DW(DW),.ACC_W(ACC_W),.H(H0),.FW(FW0),.CIN(CIN0),.COUT(C_STEM),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_stem (
    .clk(clk),.rst(rst),.start(stem_start),.act_in_flat(img_in_flat),
    .w_wr_en(stem_we),.w_wr_addr(stem_addr),.w_wr_data(w_wr_data),
    .act_out_flat(stem_out),.done(stem_done),
    .tot_toggle_count(stem_tog),.tot_adder_invocations(stem_adi),.tot_addition_operations(stem_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_STEM),.FW(W_STEM),.CIN(C_STEM),.COUT(C1),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b1 (
    .clk(clk),.rst(rst),.start(b1_start),.act_in_flat(stem_out),
    .w_wr_en(b1_we),.w_wr_addr(b1_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b1_out),.done(d1),.tot_toggle_count(b1_tog),.tot_adder_invocations(b1_adi),.tot_addition_operations(b1_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_STEM),.FW(W_STEM),.CIN(C1),.COUT(C2),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b2 (
    .clk(clk),.rst(rst),.start(b2_start),.act_in_flat(b1_out),
    .w_wr_en(b2_we),.w_wr_addr(b2_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b2_out),.done(d2),.tot_toggle_count(b2_tog),.tot_adder_invocations(b2_adi),.tot_addition_operations(b2_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B2),.FW(W_B2),.CIN(C2),.COUT(C3),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b3 (
    .clk(clk),.rst(rst),.start(b3_start),.act_in_flat(b2_out),
    .w_wr_en(b3_we),.w_wr_addr(b3_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b3_out),.done(d3),.tot_toggle_count(b3_tog),.tot_adder_invocations(b3_adi),.tot_addition_operations(b3_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B2),.FW(W_B2),.CIN(C3),.COUT(C4),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b4 (
    .clk(clk),.rst(rst),.start(b4_start),.act_in_flat(b3_out),
    .w_wr_en(b4_we),.w_wr_addr(b4_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b4_out),.done(d4),.tot_toggle_count(b4_tog),.tot_adder_invocations(b4_adi),.tot_addition_operations(b4_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B4),.FW(W_B4),.CIN(C4),.COUT(C5),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b5 (
    .clk(clk),.rst(rst),.start(b5_start),.act_in_flat(b4_out),
    .w_wr_en(b5_we),.w_wr_addr(b5_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b5_out),.done(d5),.tot_toggle_count(b5_tog),.tot_adder_invocations(b5_adi),.tot_addition_operations(b5_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B4),.FW(W_B4),.CIN(C5),.COUT(C6),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b6 (
    .clk(clk),.rst(rst),.start(b6_start),.act_in_flat(b5_out),
    .w_wr_en(b6_we),.w_wr_addr(b6_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b6_out),.done(d6),.tot_toggle_count(b6_tog),.tot_adder_invocations(b6_adi),.tot_addition_operations(b6_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C6),.COUT(C7),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b7 (
    .clk(clk),.rst(rst),.start(b7_start),.act_in_flat(b6_out),
    .w_wr_en(b7_we),.w_wr_addr(b7_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b7_out),.done(d7),.tot_toggle_count(b7_tog),.tot_adder_invocations(b7_adi),.tot_addition_operations(b7_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C7),.COUT(C8),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b8 (
    .clk(clk),.rst(rst),.start(b8_start),.act_in_flat(b7_out),
    .w_wr_en(b8_we),.w_wr_addr(b8_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b8_out),.done(d8),.tot_toggle_count(b8_tog),.tot_adder_invocations(b8_adi),.tot_addition_operations(b8_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C8),.COUT(C9),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b9 (
    .clk(clk),.rst(rst),.start(b9_start),.act_in_flat(b8_out),
    .w_wr_en(b9_we),.w_wr_addr(b9_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b9_out),.done(d9),.tot_toggle_count(b9_tog),.tot_adder_invocations(b9_adi),.tot_addition_operations(b9_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C9),.COUT(C10),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b10 (
    .clk(clk),.rst(rst),.start(b10_start),.act_in_flat(b9_out),
    .w_wr_en(b10_we),.w_wr_addr(b10_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b10_out),.done(d10),.tot_toggle_count(b10_tog),.tot_adder_invocations(b10_adi),.tot_addition_operations(b10_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C10),.COUT(C11),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b11 (
    .clk(clk),.rst(rst),.start(b11_start),.act_in_flat(b10_out),
    .w_wr_en(b11_we),.w_wr_addr(b11_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b11_out),.done(d11),.tot_toggle_count(b11_tog),.tot_adder_invocations(b11_adi),.tot_addition_operations(b11_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C11),.COUT(C12),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b12 (
    .clk(clk),.rst(rst),.start(b12_start),.act_in_flat(b11_out),
    .w_wr_en(b12_we),.w_wr_addr(b12_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b12_out),.done(d12),.tot_toggle_count(b12_tog),.tot_adder_invocations(b12_adi),.tot_addition_operations(b12_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B12),.FW(W_B12),.CIN(C12),.COUT(C13),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b13 (
    .clk(clk),.rst(rst),.start(b13_start),.act_in_flat(b12_out),
    .w_wr_en(b13_we),.w_wr_addr(b13_addr),.w_wr_data(w_wr_data),
    .act_out_flat(b13_out),.done(d13),.tot_toggle_count(b13_tog),.tot_adder_invocations(b13_adi),.tot_addition_operations(b13_ado));

global_avgpool_fc #(.DW(DW),.ACC_W(ACC_W),.H(H_B12),.FW(W_B12),.C(C13),.NUM_CLASSES(NUM_CLASSES),.MAC_SEL(MAC_SEL)) u_fc (
    .clk(clk),.rst(rst),.start(fc_start),.act_in_flat(b13_out),
    .w_wr_en(fc_we),.w_wr_addr(fc_addr),.w_wr_data(w_wr_data),
    .class_scores_flat(class_scores_flat),.done(fc_done),
    .tot_toggle_count(fc_tog),.tot_adder_invocations(fc_adi),.tot_addition_operations(fc_ado));

// ---- sequencer: stem -> b1 -> b2 -> ... -> b13 -> fc ----
localparam S_IDLE=5'd0, S_STEM=5'd1, S_B1=5'd2, S_B2=5'd3, S_B3=5'd4, S_B4=5'd5,
           S_B5=5'd6, S_B6=5'd7, S_B7=5'd8, S_B8=5'd9, S_B9=5'd10, S_B10=5'd11,
           S_B11=5'd12, S_B12=5'd13, S_B13=5'd14, S_FC=5'd15, S_DONE=5'd16;
reg [4:0] state;

reg [31:0] acc_tog, acc_adi, acc_ado;

always @(posedge clk) begin
    if (rst) begin
        state <= S_IDLE;
        {stem_start,b1_start,b2_start,b3_start,b4_start,b5_start,b6_start,
         b7_start,b8_start,b9_start,b10_start,b11_start,b12_start,b13_start,fc_start} <= 0;
        done <= 0;
        acc_tog <= 0; acc_adi <= 0; acc_ado <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        {stem_start,b1_start,b2_start,b3_start,b4_start,b5_start,b6_start,
         b7_start,b8_start,b9_start,b10_start,b11_start,b12_start,b13_start,fc_start} <= 0;

        case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    acc_tog<=0; acc_adi<=0; acc_ado<=0;
                    stem_start <= 1; state <= S_STEM;
                end
            end
            S_STEM: if (stem_done) begin acc_tog<=acc_tog+stem_tog; acc_adi<=acc_adi+stem_adi; acc_ado<=acc_ado+stem_ado; b1_start<=1; state<=S_B1; end
            S_B1:   if (d1)  begin acc_tog<=acc_tog+b1_tog;  acc_adi<=acc_adi+b1_adi;  acc_ado<=acc_ado+b1_ado;  b2_start<=1;  state<=S_B2;  end
            S_B2:   if (d2)  begin acc_tog<=acc_tog+b2_tog;  acc_adi<=acc_adi+b2_adi;  acc_ado<=acc_ado+b2_ado;  b3_start<=1;  state<=S_B3;  end
            S_B3:   if (d3)  begin acc_tog<=acc_tog+b3_tog;  acc_adi<=acc_adi+b3_adi;  acc_ado<=acc_ado+b3_ado;  b4_start<=1;  state<=S_B4;  end
            S_B4:   if (d4)  begin acc_tog<=acc_tog+b4_tog;  acc_adi<=acc_adi+b4_adi;  acc_ado<=acc_ado+b4_ado;  b5_start<=1;  state<=S_B5;  end
            S_B5:   if (d5)  begin acc_tog<=acc_tog+b5_tog;  acc_adi<=acc_adi+b5_adi;  acc_ado<=acc_ado+b5_ado;  b6_start<=1;  state<=S_B6;  end
            S_B6:   if (d6)  begin acc_tog<=acc_tog+b6_tog;  acc_adi<=acc_adi+b6_adi;  acc_ado<=acc_ado+b6_ado;  b7_start<=1;  state<=S_B7;  end
            S_B7:   if (d7)  begin acc_tog<=acc_tog+b7_tog;  acc_adi<=acc_adi+b7_adi;  acc_ado<=acc_ado+b7_ado;  b8_start<=1;  state<=S_B8;  end
            S_B8:   if (d8)  begin acc_tog<=acc_tog+b8_tog;  acc_adi<=acc_adi+b8_adi;  acc_ado<=acc_ado+b8_ado;  b9_start<=1;  state<=S_B9;  end
            S_B9:   if (d9)  begin acc_tog<=acc_tog+b9_tog;  acc_adi<=acc_adi+b9_adi;  acc_ado<=acc_ado+b9_ado;  b10_start<=1; state<=S_B10; end
            S_B10:  if (d10) begin acc_tog<=acc_tog+b10_tog; acc_adi<=acc_adi+b10_adi; acc_ado<=acc_ado+b10_ado; b11_start<=1; state<=S_B11; end
            S_B11:  if (d11) begin acc_tog<=acc_tog+b11_tog; acc_adi<=acc_adi+b11_adi; acc_ado<=acc_ado+b11_ado; b12_start<=1; state<=S_B12; end
            S_B12:  if (d12) begin acc_tog<=acc_tog+b12_tog; acc_adi<=acc_adi+b12_adi; acc_ado<=acc_ado+b12_ado; b13_start<=1; state<=S_B13; end
            S_B13:  if (d13) begin acc_tog<=acc_tog+b13_tog; acc_adi<=acc_adi+b13_adi; acc_ado<=acc_ado+b13_ado; fc_start<=1;  state<=S_FC;  end
            S_FC:   if (fc_done) begin
                        tot_toggle_count        <= acc_tog + fc_tog;
                        tot_adder_invocations   <= acc_adi + fc_adi;
                        tot_addition_operations <= acc_ado + fc_ado;
                        state <= S_DONE;
                    end
            S_DONE: begin done <= 1; state <= S_IDLE; end
            default: state <= S_IDLE;
        endcase
    end
end

endmodule
