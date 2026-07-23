`timescale 1ns / 1ps

module depthwise_layer #(
    parameter DW      = 8,     // activation/weight bit width
    parameter ACC_W   = 32,
    parameter H       = 4,     // input feature map height
    parameter FW      = 4,     // input feature map width
    parameter C       = 8,     // channel count
    parameter STRIDE  = 1,     // 1 or 2
    parameter MAC_SEL = 2
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,        // pulse: begin whole layer

    input  wire [C*H*FW*DW-1:0]          act_in_flat,  // input feature map
    input  wire [C*9*DW-1:0]             weight_flat,  // 9 taps per channel

    output reg  [C*((H+STRIDE-1)/STRIDE)*((FW+STRIDE-1)/STRIDE)*DW-1:0] act_out_flat, // ReLU6-clamped output
    output reg                           done,

    output reg [31:0]                    tot_toggle_count,
    output reg [31:0]                    tot_adder_invocations,
    output reg [31:0]                    tot_addition_operations
);

localparam RELU6 = 8'd63; // clamp ceiling in this quantized activation domain
localparam IDX_W = 4;     // enough for 0..8
localparam integer OH = (H  + STRIDE - 1) / STRIDE;
localparam integer OW = (FW + STRIDE - 1) / STRIDE;

// ---- iteration state: channel c, output pixel (oy,ox) in OUTPUT coords ----
integer c, oy, ox;

localparam S_IDLE=2'd0, S_RUN=2'd1, S_STORE=2'd2, S_NEXT=2'd3;
reg [1:0] state;

function [DW-1:0] get_act;
    input integer cc, yy, xx;
    begin
        if (yy < 0 || yy >= H || xx < 0 || xx >= FW)
            get_act = {DW{1'b0}};
        else
            get_act = act_in_flat[((cc*H+yy)*FW+xx)*DW +: DW];
    end
endfunction

function [DW-1:0] get_w;
    input integer cc;
    input integer kk;
    begin
        get_w = weight_flat[(cc*9+kk)*DW +: DW];
    end
endfunction

// ---- mac_engine instance ----
reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

// combinational: activation/weight for the CURRENT tap index (idx = ky*3+kx)
// output pixel (oy,ox) is centered on input position (oy*STRIDE, ox*STRIDE)
wire signed [7:0] cur_iy = (oy*STRIDE) + (eng_idx/3) - 1;
wire signed [7:0] cur_ix = (ox*STRIDE) + (eng_idx%3) - 1;
assign eng_x = get_act(c, cur_iy, cur_ix);
assign eng_w = get_w(c, eng_idx);

mac_engine #(.W(DW), .ACC_W(ACC_W), .IDX_W(IDX_W), .MAC_SEL(MAC_SEL)) u_eng (
    .clk(clk), .rst(rst),
    .start(eng_start), .k_count(4'd9),
    .idx(eng_idx), .x_in(eng_x), .w_in(eng_w),
    .result(eng_result), .result_valid(eng_result_valid), .busy(),
    .tot_toggle_count(eng_tog),
    .tot_adder_invocations(eng_adi),
    .tot_addition_operations(eng_ado)
);

always @(posedge clk) begin
    if (rst) begin
        state <= S_IDLE;
        done  <= 0;
        c <= 0; oy <= 0; ox <= 0;
        eng_start <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        eng_start <= 0;

        case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    c <= 0; oy <= 0; ox <= 0;
                    tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
                    eng_start <= 1;
                    state <= S_RUN;
                end
            end

            S_RUN: begin
                if (eng_result_valid) begin
                    state <= S_STORE;
                end
            end

            S_STORE: begin
                act_out_flat[((c*OH+oy)*OW+ox)*DW +: DW] <=
                    (eng_result > RELU6) ? RELU6 : eng_result[DW-1:0];

                tot_toggle_count        <= tot_toggle_count        + eng_tog;
                tot_adder_invocations   <= tot_adder_invocations   + eng_adi;
                tot_addition_operations <= tot_addition_operations + eng_ado;

                state <= S_NEXT;
            end

            S_NEXT: begin
                if (ox < OW-1) begin
                    ox <= ox + 1;
                    eng_start <= 1;
                    state <= S_RUN;
                end else if (oy < OH-1) begin
                    ox <= 0; oy <= oy + 1;
                    eng_start <= 1;
                    state <= S_RUN;
                end else if (c < C-1) begin
                    ox <= 0; oy <= 0; c <= c + 1;
                    eng_start <= 1;
                    state <= S_RUN;
                end else begin
                    done <= 1;
                    state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
