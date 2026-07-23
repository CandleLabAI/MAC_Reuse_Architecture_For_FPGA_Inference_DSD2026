`timescale 1ns / 1ps

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
    parameter MAC_SEL     = 2
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN0*H0*FW0*DW-1:0]     img_in_flat,

    // stem weights: COUT*CIN*9 taps
    input  wire [C_STEM*CIN0*9*DW-1:0]   stem_w_flat,

    // per-block depthwise (9 taps/channel) and pointwise (COUT*CIN) weights
    input  wire [C_STEM*9*DW-1:0]        b1_dw_w,  input wire [C1*C_STEM*DW-1:0] b1_pw_w,
    input  wire [C1*9*DW-1:0]            b2_dw_w,  input wire [C2*C1*DW-1:0]     b2_pw_w,
    input  wire [C2*9*DW-1:0]            b3_dw_w,  input wire [C3*C2*DW-1:0]     b3_pw_w,
    input  wire [C3*9*DW-1:0]            b4_dw_w,  input wire [C4*C3*DW-1:0]     b4_pw_w,
    input  wire [C4*9*DW-1:0]            b5_dw_w,  input wire [C5*C4*DW-1:0]     b5_pw_w,
    input  wire [C5*9*DW-1:0]            b6_dw_w,  input wire [C6*C5*DW-1:0]     b6_pw_w,
    input  wire [C6*9*DW-1:0]            b7_dw_w,  input wire [C7*C6*DW-1:0]     b7_pw_w,
    input  wire [C7*9*DW-1:0]            b8_dw_w,  input wire [C8*C7*DW-1:0]     b8_pw_w,
    input  wire [C8*9*DW-1:0]            b9_dw_w,  input wire [C9*C8*DW-1:0]     b9_pw_w,
    input  wire [C9*9*DW-1:0]            b10_dw_w, input wire [C10*C9*DW-1:0]    b10_pw_w,
    input  wire [C10*9*DW-1:0]           b11_dw_w, input wire [C11*C10*DW-1:0]   b11_pw_w,
    input  wire [C11*9*DW-1:0]           b12_dw_w, input wire [C12*C11*DW-1:0]   b12_pw_w,
    input  wire [C12*9*DW-1:0]           b13_dw_w, input wire [C13*C12*DW-1:0]   b13_pw_w,

    input  wire [NUM_CLASSES*C13*DW-1:0] fc_w_flat,

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

std_conv3x3_layer #(.DW(DW),.ACC_W(ACC_W),.H(H0),.FW(FW0),.CIN(CIN0),.COUT(C_STEM),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_stem (
    .clk(clk),.rst(rst),.start(stem_start),.act_in_flat(img_in_flat),.weight_flat(stem_w_flat),
    .act_out_flat(stem_out),.done(stem_done),
    .tot_toggle_count(stem_tog),.tot_adder_invocations(stem_adi),.tot_addition_operations(stem_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_STEM),.FW(W_STEM),.CIN(C_STEM),.COUT(C1),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b1 (
    .clk(clk),.rst(rst),.start(b1_start),.act_in_flat(stem_out),.dw_weight_flat(b1_dw_w),.pw_weight_flat(b1_pw_w),
    .act_out_flat(b1_out),.done(d1),.tot_toggle_count(b1_tog),.tot_adder_invocations(b1_adi),.tot_addition_operations(b1_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_STEM),.FW(W_STEM),.CIN(C1),.COUT(C2),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b2 (
    .clk(clk),.rst(rst),.start(b2_start),.act_in_flat(b1_out),.dw_weight_flat(b2_dw_w),.pw_weight_flat(b2_pw_w),
    .act_out_flat(b2_out),.done(d2),.tot_toggle_count(b2_tog),.tot_adder_invocations(b2_adi),.tot_addition_operations(b2_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B2),.FW(W_B2),.CIN(C2),.COUT(C3),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b3 (
    .clk(clk),.rst(rst),.start(b3_start),.act_in_flat(b2_out),.dw_weight_flat(b3_dw_w),.pw_weight_flat(b3_pw_w),
    .act_out_flat(b3_out),.done(d3),.tot_toggle_count(b3_tog),.tot_adder_invocations(b3_adi),.tot_addition_operations(b3_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B2),.FW(W_B2),.CIN(C3),.COUT(C4),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b4 (
    .clk(clk),.rst(rst),.start(b4_start),.act_in_flat(b3_out),.dw_weight_flat(b4_dw_w),.pw_weight_flat(b4_pw_w),
    .act_out_flat(b4_out),.done(d4),.tot_toggle_count(b4_tog),.tot_adder_invocations(b4_adi),.tot_addition_operations(b4_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B4),.FW(W_B4),.CIN(C4),.COUT(C5),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b5 (
    .clk(clk),.rst(rst),.start(b5_start),.act_in_flat(b4_out),.dw_weight_flat(b5_dw_w),.pw_weight_flat(b5_pw_w),
    .act_out_flat(b5_out),.done(d5),.tot_toggle_count(b5_tog),.tot_adder_invocations(b5_adi),.tot_addition_operations(b5_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B4),.FW(W_B4),.CIN(C5),.COUT(C6),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b6 (
    .clk(clk),.rst(rst),.start(b6_start),.act_in_flat(b5_out),.dw_weight_flat(b6_dw_w),.pw_weight_flat(b6_pw_w),
    .act_out_flat(b6_out),.done(d6),.tot_toggle_count(b6_tog),.tot_adder_invocations(b6_adi),.tot_addition_operations(b6_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C6),.COUT(C7),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b7 (
    .clk(clk),.rst(rst),.start(b7_start),.act_in_flat(b6_out),.dw_weight_flat(b7_dw_w),.pw_weight_flat(b7_pw_w),
    .act_out_flat(b7_out),.done(d7),.tot_toggle_count(b7_tog),.tot_adder_invocations(b7_adi),.tot_addition_operations(b7_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C7),.COUT(C8),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b8 (
    .clk(clk),.rst(rst),.start(b8_start),.act_in_flat(b7_out),.dw_weight_flat(b8_dw_w),.pw_weight_flat(b8_pw_w),
    .act_out_flat(b8_out),.done(d8),.tot_toggle_count(b8_tog),.tot_adder_invocations(b8_adi),.tot_addition_operations(b8_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C8),.COUT(C9),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b9 (
    .clk(clk),.rst(rst),.start(b9_start),.act_in_flat(b8_out),.dw_weight_flat(b9_dw_w),.pw_weight_flat(b9_pw_w),
    .act_out_flat(b9_out),.done(d9),.tot_toggle_count(b9_tog),.tot_adder_invocations(b9_adi),.tot_addition_operations(b9_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C9),.COUT(C10),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b10 (
    .clk(clk),.rst(rst),.start(b10_start),.act_in_flat(b9_out),.dw_weight_flat(b10_dw_w),.pw_weight_flat(b10_pw_w),
    .act_out_flat(b10_out),.done(d10),.tot_toggle_count(b10_tog),.tot_adder_invocations(b10_adi),.tot_addition_operations(b10_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C10),.COUT(C11),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b11 (
    .clk(clk),.rst(rst),.start(b11_start),.act_in_flat(b10_out),.dw_weight_flat(b11_dw_w),.pw_weight_flat(b11_pw_w),
    .act_out_flat(b11_out),.done(d11),.tot_toggle_count(b11_tog),.tot_adder_invocations(b11_adi),.tot_addition_operations(b11_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B6),.FW(W_B6),.CIN(C11),.COUT(C12),.STRIDE(2),.MAC_SEL(MAC_SEL)) u_b12 (
    .clk(clk),.rst(rst),.start(b12_start),.act_in_flat(b11_out),.dw_weight_flat(b12_dw_w),.pw_weight_flat(b12_pw_w),
    .act_out_flat(b12_out),.done(d12),.tot_toggle_count(b12_tog),.tot_adder_invocations(b12_adi),.tot_addition_operations(b12_ado));

dw_sep_block #(.DW(DW),.ACC_W(ACC_W),.H(H_B12),.FW(W_B12),.CIN(C12),.COUT(C13),.STRIDE(1),.MAC_SEL(MAC_SEL)) u_b13 (
    .clk(clk),.rst(rst),.start(b13_start),.act_in_flat(b12_out),.dw_weight_flat(b13_dw_w),.pw_weight_flat(b13_pw_w),
    .act_out_flat(b13_out),.done(d13),.tot_toggle_count(b13_tog),.tot_adder_invocations(b13_adi),.tot_addition_operations(b13_ado));

global_avgpool_fc #(.DW(DW),.ACC_W(ACC_W),.H(H_B12),.FW(W_B12),.C(C13),.NUM_CLASSES(NUM_CLASSES),.MAC_SEL(MAC_SEL)) u_fc (
    .clk(clk),.rst(rst),.start(fc_start),.act_in_flat(b13_out),.fc_weight_flat(fc_w_flat),
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
