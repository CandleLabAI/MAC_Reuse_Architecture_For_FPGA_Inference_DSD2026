`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// depthwise_layer.sv
//
// Depthwise 3x3, zero-padded ("same") convolution over a C-channel, HxFW
// feature map, with configurable STRIDE (1 or 2). Weights (9 taps/channel)
// are held in a weight_bram (synchronous read/write, Vivado infers Block
// RAM) instead of a flattened port -- load them via w_wr_en/w_wr_addr/
// w_wr_data before asserting start (address = channel*9 + tap_index, tap
// index = ky*3+kx). Activations stay a flattened combinational bus (the
// feature-map tensor size is much smaller and simpler to keep as-is).
//
// Each output pixel is a real 9-tap dot product, computed by mac_engine
// (index-driven, race-free, and now BRAM-latency-aware -- see
// mac_engine.sv's S_ADDR state).
//////////////////////////////////////////////////////////////////////////////

module depthwise_layer #(
    parameter DW      = 8,     // activation/weight bit width
    parameter ACC_W   = 32,
    parameter H       = 4,     // input feature map height
    parameter FW      = 4,     // input feature map width
    parameter C       = 8,     // channel count
    parameter STRIDE  = 1,     // 1 or 2
    parameter MAC_SEL = 2,
    parameter WADDR_W = (C*9 > 1) ? $clog2(C*9) : 1  // weight BRAM address width
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,        // pulse: begin whole layer

    input  wire [C*H*FW*DW-1:0]          act_in_flat,  // input feature map

    // weight loading interface (load before start; address = c*9 + tap)
    input  wire                          w_wr_en,
    input  wire [WADDR_W-1:0]            w_wr_addr,
    input  wire [DW-1:0]                 w_wr_data,

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
localparam integer C_W  = (C  > 1) ? $clog2(C)  : 1;
localparam integer OH_W = (OH > 1) ? $clog2(OH) : 1;
localparam integer OW_W = (OW > 1) ? $clog2(OW) : 1;

// ---- iteration state: channel c, output pixel (oy,ox) in OUTPUT coords ----
reg [C_W-1:0]  c;
reg [OH_W-1:0] oy;
reg [OW_W-1:0] ox;

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

// ---- mac_engine instance ----
reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

// combinational: activation for the CURRENT tap index (idx = ky*3+kx)
// output pixel (oy,ox) is centered on input position (oy*STRIDE, ox*STRIDE)
wire [3:0] idx_div3 = eng_idx / 3;
wire [3:0] idx_mod3 = eng_idx % 3;
wire signed [7:0] cur_iy = $signed({1'b0, oy}) * STRIDE + $signed({4'b0, idx_div3}) - 8'sd1;
wire signed [7:0] cur_ix = $signed({1'b0, ox}) * STRIDE + $signed({4'b0, idx_mod3}) - 8'sd1;
assign eng_x = get_act(c, cur_iy, cur_ix);

// weight BRAM: address = c*9 + eng_idx, read latency 1 cycle (matches
// mac_engine's S_ADDR settle state)
wire [WADDR_W-1:0] w_rd_addr = c*9 + eng_idx;

weight_bram #(.DW(DW), .DEPTH(C*9), .AW(WADDR_W)) u_wmem (
    .clk(clk),
    .wr_en(w_wr_en), .wr_addr(w_wr_addr), .wr_data(w_wr_data),
    .rd_addr(w_rd_addr), .rd_data(eng_w)
);

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
