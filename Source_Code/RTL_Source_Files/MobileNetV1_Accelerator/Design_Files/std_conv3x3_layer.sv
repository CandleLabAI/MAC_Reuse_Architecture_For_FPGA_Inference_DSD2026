`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// std_conv3x3_layer.sv
//
// Standard (non-separable) 3x3 zero-padded convolution, used as the
// MobileNetV1 "stem" layer that ingests the raw CIN-channel image (CIN=3
// for RGB) and produces the first COUT-channel feature map. Real MobileNetV1
// uses STRIDE=2 here. Weights (COUT*CIN*9 taps) are held in a weight_bram
// (synchronous read/write, Vivado infers Block RAM) instead of a flattened
// port -- load them via w_wr_en/w_wr_addr/w_wr_data before asserting start
// (address = cout*(CIN*9) + cin*9 + tap, tap = ky*3+kx). Computed by
// mac_engine (index-driven, race-free, BRAM-latency-aware).
//////////////////////////////////////////////////////////////////////////////

module std_conv3x3_layer #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H       = 4,
    parameter FW      = 4,
    parameter CIN     = 3,
    parameter COUT    = 8,
    parameter STRIDE  = 2,     // MobileNetV1 stem uses stride 2
    parameter MAC_SEL = 2,
    parameter WADDR_W = (COUT*CIN*9 > 1) ? $clog2(COUT*CIN*9) : 1
)(
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              start,

    input  wire [CIN*H*FW*DW-1:0]            act_in_flat,

    // weight loading interface (address = cout*(CIN*9) + cin*9 + ky*3+kx)
    input  wire                              w_wr_en,
    input  wire [WADDR_W-1:0]                w_wr_addr,
    input  wire [DW-1:0]                     w_wr_data,

    output reg  [COUT*((H+STRIDE-1)/STRIDE)*((FW+STRIDE-1)/STRIDE)*DW-1:0] act_out_flat,
    output reg                               done,

    output reg [31:0]                        tot_toggle_count,
    output reg [31:0]                        tot_adder_invocations,
    output reg [31:0]                        tot_addition_operations
);

localparam RELU6 = 8'd63;
localparam integer TAPS = CIN*9;
localparam IDX_W = 16;
localparam integer OH = (H  + STRIDE - 1) / STRIDE;
localparam integer OW = (FW + STRIDE - 1) / STRIDE;
localparam integer COUT_W = (COUT > 1) ? $clog2(COUT) : 1;
localparam integer OH_W   = (OH   > 1) ? $clog2(OH)   : 1;
localparam integer OW_W   = (OW   > 1) ? $clog2(OW)   : 1;

reg [COUT_W-1:0] cout;
reg [OH_W-1:0]   oy;
reg [OW_W-1:0]   ox;

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

reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

// combinational: decode tap index t (0..TAPS-1) -> (cin, ky, kx)
wire [15:0] cur_cin = eng_idx / 9;
wire [3:0]  cur_k   = eng_idx % 9;
wire [3:0] cur_k_div3 = cur_k / 3;
wire [3:0] cur_k_mod3 = cur_k % 3;
wire signed [7:0] cur_iy = $signed({1'b0, oy}) * STRIDE + $signed({4'b0, cur_k_div3}) - 8'sd1;
wire signed [7:0] cur_ix = $signed({1'b0, ox}) * STRIDE + $signed({4'b0, cur_k_mod3}) - 8'sd1;

assign eng_x = get_act(cur_cin, cur_iy, cur_ix);

// weight BRAM: address = cout*TAPS + eng_idx (eng_idx already spans cin*9+k
// since it counts 0..TAPS-1), read latency 1 cycle (matches mac_engine's
// S_ADDR settle state)
wire [WADDR_W-1:0] w_rd_addr = cout*TAPS + eng_idx;

weight_bram #(.DW(DW), .DEPTH(COUT*TAPS), .AW(WADDR_W)) u_wmem (
    .clk(clk),
    .wr_en(w_wr_en), .wr_addr(w_wr_addr), .wr_data(w_wr_data),
    .rd_addr(w_rd_addr), .rd_data(eng_w)
);

mac_engine #(.W(DW), .ACC_W(ACC_W), .IDX_W(IDX_W), .MAC_SEL(MAC_SEL)) u_eng (
    .clk(clk), .rst(rst),
    .start(eng_start), .k_count(TAPS[15:0]),
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
        cout <= 0; oy <= 0; ox <= 0;
        eng_start <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        eng_start <= 0;

        case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    cout <= 0; oy <= 0; ox <= 0;
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
                act_out_flat[((cout*OH+oy)*OW+ox)*DW +: DW] <=
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
                end else if (cout < COUT-1) begin
                    ox <= 0; oy <= 0; cout <= cout + 1;
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
