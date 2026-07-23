`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// pointwise_layer.sv
//
// Pointwise (1x1) convolution: real Cin-length dot product per output
// channel per pixel, i.e. out[cout][y][x] = sum_cin( in[cin][y][x] *
// w[cout][cin] ). Weights are held in a weight_bram (synchronous read/
// write, Vivado infers Block RAM) instead of a flattened port -- load them
// via w_wr_en/w_wr_addr/w_wr_data before asserting start (address =
// cout*CIN + cin). Computed by mac_engine (index-driven, race-free, and
// BRAM-latency-aware -- see mac_engine.sv's S_ADDR state).
//////////////////////////////////////////////////////////////////////////////

module pointwise_layer #(
    parameter DW      = 8,
    parameter ACC_W   = 32,
    parameter H       = 4,
    parameter FW      = 4,
    parameter CIN     = 8,
    parameter COUT    = 16,
    parameter MAC_SEL = 2,
    parameter WADDR_W = (COUT*CIN > 1) ? $clog2(COUT*CIN) : 1
)(
    input  wire                            clk,
    input  wire                            rst,
    input  wire                            start,

    input  wire [CIN*H*FW*DW-1:0]          act_in_flat,

    // weight loading interface (load before start; address = cout*CIN + cin)
    input  wire                            w_wr_en,
    input  wire [WADDR_W-1:0]              w_wr_addr,
    input  wire [DW-1:0]                   w_wr_data,

    output reg  [COUT*H*FW*DW-1:0]         act_out_flat,
    output reg                             done,

    output reg [31:0]                      tot_toggle_count,
    output reg [31:0]                      tot_adder_invocations,
    output reg [31:0]                      tot_addition_operations
);

localparam RELU6 = 8'd63;
localparam IDX_W = 16;
localparam integer COUT_W = (COUT > 1) ? $clog2(COUT) : 1;
localparam integer H_W    = (H    > 1) ? $clog2(H)    : 1;
localparam integer FW_W   = (FW   > 1) ? $clog2(FW)   : 1;

reg [COUT_W-1:0] cout;
reg [H_W-1:0]    oy;
reg [FW_W-1:0]   ox;

localparam S_IDLE=2'd0, S_RUN=2'd1, S_STORE=2'd2, S_NEXT=2'd3;
reg [1:0] state;

function [DW-1:0] get_act;
    input integer cc, yy, xx;
    begin
        get_act = act_in_flat[((cc*H+yy)*FW+xx)*DW +: DW];
    end
endfunction

reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

// combinational: activation for the CURRENT reduction index (= cin)
assign eng_x = get_act(eng_idx, oy, ox);

// weight BRAM: address = cout*CIN + eng_idx, read latency 1 cycle (matches
// mac_engine's S_ADDR settle state)
wire [WADDR_W-1:0] w_rd_addr = cout*CIN + eng_idx;

weight_bram #(.DW(DW), .DEPTH(COUT*CIN), .AW(WADDR_W)) u_wmem (
    .clk(clk),
    .wr_en(w_wr_en), .wr_addr(w_wr_addr), .wr_data(w_wr_data),
    .rd_addr(w_rd_addr), .rd_data(eng_w)
);

mac_engine #(.W(DW), .ACC_W(ACC_W), .IDX_W(IDX_W), .MAC_SEL(MAC_SEL)) u_eng (
    .clk(clk), .rst(rst),
    .start(eng_start), .k_count(CIN[15:0]),
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
                act_out_flat[((cout*H+oy)*FW+ox)*DW +: DW] <=
                    (eng_result > RELU6) ? RELU6 : eng_result[DW-1:0];

                tot_toggle_count        <= tot_toggle_count        + eng_tog;
                tot_adder_invocations   <= tot_adder_invocations   + eng_adi;
                tot_addition_operations <= tot_addition_operations + eng_ado;

                state <= S_NEXT;
            end

            S_NEXT: begin
                if (ox < FW-1) begin
                    ox <= ox + 1;
                    eng_start <= 1;
                    state <= S_RUN;
                end else if (oy < H-1) begin
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
