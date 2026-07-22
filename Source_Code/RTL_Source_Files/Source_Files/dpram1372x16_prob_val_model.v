module dpram1372x16_prob_val_model (
	input [11:0]wr_addr_i,
	input [11:0]rd_addr_i , //anchor_cnt
	input [15:0]wr_data_i ,
	input wr_en_i ,
	input rd_en_i ,
	input rd_clk_i ,
	input rd_clk_en_i,//
	input rst_i ,
	input wr_clk_i,
	input wr_clk_en_i,
	output reg [15:0] rd_data_o
	);
	
	reg [15:0] mem_arr [0:1371];
	
	always @ (posedge wr_clk_i)
		if(wr_en_i && wr_clk_en_i) mem_arr[wr_addr_i] <= wr_data_i;
			
	always @ (posedge rd_clk_i)
		if(rd_en_i && rd_clk_en_i) rd_data_o <= mem_arr[rd_addr_i];
	
endmodule