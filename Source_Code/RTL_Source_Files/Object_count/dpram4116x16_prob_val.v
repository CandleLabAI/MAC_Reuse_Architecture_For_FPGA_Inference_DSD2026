module dpram4116x16_prob_val (wr_clk_i, rd_clk_i, rst_i, wr_clk_en_i, 
            rd_en_i, rd_clk_en_i, wr_en_i, wr_data_i, wr_addr_i, rd_addr_i, 
            rd_data_o);
    input wr_clk_i;
    input rd_clk_i;
    input rst_i;
    input wr_clk_en_i;
    input rd_en_i;
    input rd_clk_en_i;
    input wr_en_i;
    input [15:0]wr_data_i;
    input [12:0]wr_addr_i;
    input [12:0]rd_addr_i;
    output [15:0]rd_data_o;
    
    
    
endmodule
