`timescale 1ns / 1ps

module mobilenetv1_top #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H       = 4,
    parameter FW      = 4,
    parameter CIN0    = 3,   // e.g. RGB
    parameter C0      = 4,   // stem output channels
    parameter C1      = 8,   // after block 1
    parameter C2      = 16,  // after block 2
    parameter MAC_SEL = 2
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN0*H*FW*DW-1:0]       img_in_flat,

    input  wire [C0*CIN0*9*DW-1:0]       stem_w_flat,
    input  wire [C0*9*DW-1:0]            b1_dw_w_flat,
    input  wire [C1*C0*DW-1:0]           b1_pw_w_flat,
    input  wire [C1*9*DW-1:0]            b2_dw_w_flat,
    input  wire [C2*C1*DW-1:0]           b2_pw_w_flat,

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

std_conv3x3_layer #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(CIN0), .COUT(C0), .MAC_SEL(MAC_SEL)) u_stem (
    .clk(clk), .rst(rst), .start(stem_start),
    .act_in_flat(img_in_flat), .weight_flat(stem_w_flat),
    .act_out_flat(stem_out), .done(stem_done),
    .tot_toggle_count(stem_tog), .tot_adder_invocations(stem_adi), .tot_addition_operations(stem_ado)
);

dw_sep_block #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(C0), .COUT(C1), .MAC_SEL(MAC_SEL)) u_block1 (
    .clk(clk), .rst(rst), .start(b1_start),
    .act_in_flat(stem_out), .dw_weight_flat(b1_dw_w_flat), .pw_weight_flat(b1_pw_w_flat),
    .act_out_flat(b1_out), .done(b1_done),
    .tot_toggle_count(b1_tog), .tot_adder_invocations(b1_adi), .tot_addition_operations(b1_ado)
);

dw_sep_block #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .CIN(C1), .COUT(C2), .MAC_SEL(MAC_SEL)) u_block2 (
    .clk(clk), .rst(rst), .start(b2_start),
    .act_in_flat(b1_out), .dw_weight_flat(b2_dw_w_flat), .pw_weight_flat(b2_pw_w_flat),
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
