`timescale 1ns / 1ps

module dsp_mac_sa #(
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

// Synthesis attributes added to force hardware DSP block inference
(* use_dsp = "yes" *) reg [2*W+8:0] acc;
reg [2*W+8:0] acc_prev;

(* use_dsp = "yes" *) reg [2*W+8:0] cycle_sum;
reg [2*W+8:0] cycle_sum_prev;

reg [W-1:0] x_reg;
reg [4:0] bit_idx;

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
        cycle_sum <= 0;
        cycle_sum_prev <= 0;
        toggle_count <= 0;
        adder_invocations <= 0;
        addition_operations <= 0;
        bit_idx <= 0;
        done <= 0;
    end
    else if(start) begin
        acc <= 0;
        acc_prev <= 0;
        cycle_sum <= 0;
        cycle_sum_prev <= 0;
        toggle_count <= 0;
        adder_invocations <= 0;
        addition_operations <= 0;
        bit_idx <= 0;
        done <= 0;
        x_reg <= x;
    end
    else if(bit_idx < W) begin
        acc_prev <= acc;
        cycle_sum_prev <= cycle_sum;
        cycle_sum = 0;

        if(x_reg[bit_idx]) begin
            adder_invocations <= adder_invocations + 1;
            addition_operations <= addition_operations + (N-1);
            for(i=0;i<N;i=i+1)
                cycle_sum = cycle_sum + (weights[i] << bit_idx);
        end

        acc <= acc + cycle_sum;

        toggle_count <= toggle_count +
                        count_ones(acc ^ acc_prev) +
                        N*count_ones(cycle_sum ^ cycle_sum_prev);
        bit_idx <= bit_idx + 1;
    end
    else begin
        y <= acc;
        done <= 1;
    end
end
endmodule