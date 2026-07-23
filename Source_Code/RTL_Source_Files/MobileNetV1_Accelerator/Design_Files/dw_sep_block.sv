`timescale 1ns / 1ps

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
    parameter integer OW = (FW + STRIDE - 1) / STRIDE   // output spatial width
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,

    input  wire [CIN*H*FW*DW-1:0]        act_in_flat,
    input  wire [CIN*9*DW-1:0]           dw_weight_flat,
    input  wire [COUT*CIN*DW-1:0]        pw_weight_flat,

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

depthwise_layer #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW), .C(CIN), .STRIDE(STRIDE), .MAC_SEL(MAC_SEL)) u_dw (
    .clk(clk), .rst(rst), .start(dw_start),
    .act_in_flat(act_in_flat), .weight_flat(dw_weight_flat),
    .act_out_flat(dw_out_flat), .done(dw_done),
    .tot_toggle_count(dw_tog), .tot_adder_invocations(dw_adi), .tot_addition_operations(dw_ado)
);

pointwise_layer #(.DW(DW), .ACC_W(ACC_W), .H(OH), .FW(OW), .CIN(CIN), .COUT(COUT), .MAC_SEL(MAC_SEL)) u_pw (
    .clk(clk), .rst(rst), .start(pw_start),
    .act_in_flat(dw_out_flat), .weight_flat(pw_weight_flat),
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
