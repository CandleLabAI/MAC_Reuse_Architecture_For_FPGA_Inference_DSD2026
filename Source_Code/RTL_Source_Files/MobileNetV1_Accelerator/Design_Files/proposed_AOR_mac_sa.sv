`timescale 1ns / 1ps

module proposed_mac_sa #(
    parameter N = 64,
    parameter W = 8
)(
    input clk,
    input rst,
    input start,
    input [W-1:0] x,
    input [N*W-1:0] weights_flat,

    output reg [2*W+8:0] y,
    output reg done,

    output reg [31:0] toggle_count,
    output reg [31:0] adder_invocations,
    output reg [31:0] addition_operations
);

integer i;

reg [W-1:0] weights [0:N-1];

reg [W+8:0] weight_sum;
reg [W+8:0] weight_sum_prev;

reg [2*W+8:0] acc;
reg [2*W+8:0] acc_prev;

reg [W-1:0] x_reg;

reg [4:0] bit_idx;

reg sum_done;

reg [W+8:0] temp_sum;

always @(*) begin
    for(i=0;i<N;i=i+1)
        weights[i] = weights_flat[i*W +: W];
end

function [31:0] count_ones;
input [2*W+8:0] val;
integer k;
begin
    count_ones = 0;
    for(k=0;k<2*W+9;k=k+1)
        count_ones = count_ones + val[k];
end
endfunction

always @(posedge clk) begin

    if(rst) begin

        acc <= 0;
        acc_prev <= 0;

        weight_sum <= 0;
        weight_sum_prev <= 0;

        toggle_count <= 0;

        adder_invocations <= 0;
        addition_operations <= 0;

        bit_idx <= 0;
        done <= 0;

        sum_done <= 0;

    end

    else if(start) begin

        acc <= 0;
        acc_prev <= 0;

        weight_sum <= 0;
        weight_sum_prev <= 0;

        toggle_count <= 0;

        adder_invocations <= 0;
        addition_operations <= 0;

        bit_idx <= 0;
        done <= 0;

        sum_done <= 0;

        x_reg <= x;

    end

    else if(!sum_done) begin

        temp_sum = 0;

        for(i=0;i<N;i=i+1)
            temp_sum = temp_sum + weights[i];

        weight_sum_prev <= weight_sum;
        weight_sum <= temp_sum;

        adder_invocations <= 1;

        addition_operations <= (N-1);

        toggle_count <=
            toggle_count +
            count_ones(weight_sum ^ weight_sum_prev);

        sum_done <= 1;

    end

    else if(bit_idx < W) begin

        acc_prev <= acc;

        if(x_reg[bit_idx]) begin

            acc <= acc + (weight_sum << bit_idx);

            addition_operations <=
                addition_operations + 1;

        end

        toggle_count <=
            toggle_count +
            count_ones(acc ^ acc_prev);

        bit_idx <= bit_idx + 1;

    end

    else begin

        y <= acc;
        done <= 1;

    end
end

endmodule