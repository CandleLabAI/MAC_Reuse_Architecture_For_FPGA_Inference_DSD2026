`timescale 1ns / 1ps
module rst_ctrl (
	input 	rst_n_i,	// external reset
	input 	clk_i,		// 27 MHz
	output 	i2c_m_rst_n_o,
	output  sensor_rst_n_o,
	output	nx_rst_n_o
);

reg [23:0] reset_delay;
wire reset_n;

reg rst_meta_n, rst_sync_n;
reg nx_rst_n;

assign nx_rst_n_o = nx_rst_n;

always @(posedge clk_i or negedge rst_n_i) begin
	if (~rst_n_i) begin
		rst_meta_n <= 0;
		rst_sync_n <= 0;
	end
	else begin
		rst_meta_n <= rst_n_i;
		rst_sync_n <= rst_meta_n;
	end
end

///// reset release order : sensor, NX, i2c_m /////

always @ (posedge clk_i or negedge rst_sync_n) begin
	if (~rst_sync_n) begin
		reset_delay <= 0;
	end
`ifndef SIM
	else if (~reset_delay[23]) begin
`else
	else if (~reset_delay[10]) begin
`endif
		reset_delay <= reset_delay + 1;
	end
end

`ifndef SIM
	assign reset_n = reset_delay[23];
`else
	assign reset_n = reset_delay[10];
`endif
assign sensor_rst_n_o = reset_n;

reg [19:0] i2c_delay;	// MT

always @ (posedge clk_i or negedge reset_n) begin
	if (~reset_n) begin
		i2c_delay <= 0;
	end
`ifndef SIM
	else if (!i2c_delay[19]) begin
`else
	else if (!i2c_delay[10]) begin
`endif
		i2c_delay <= i2c_delay + 1;
	end
end

`ifndef SIM
	assign i2c_m_rst_n_o = i2c_delay[19];
`else
	assign i2c_m_rst_n_o = i2c_delay[10];
`endif

always @ (posedge clk_i or negedge reset_n) begin
	if (~reset_n) begin
		nx_rst_n <= 0;
	end
`ifndef SIM
	else if (i2c_delay[16]) begin
`else
	else if (i2c_delay[8]) begin
`endif
		nx_rst_n <= 1;
	end
end


endmodule
