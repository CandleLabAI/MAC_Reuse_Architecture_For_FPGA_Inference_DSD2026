module tdpram_512x36 (clk_a_i, clk_b_i, rst_a_i, rst_b_i, clk_en_a_i, 
            clk_en_b_i, wr_en_a_i, wr_en_b_i, wr_data_a_i, addr_a_i, 
            rd_data_a_o, wr_data_b_i, addr_b_i, rd_data_b_o);
    input clk_a_i;
    input clk_b_i;
    input rst_a_i;
    input rst_b_i;
    input clk_en_a_i;
    input clk_en_b_i;
    input wr_en_a_i;
    input wr_en_b_i;
    input [35:0]wr_data_a_i;
    input [8:0]addr_a_i;
    output [35:0]rd_data_a_o;
    input [35:0]wr_data_b_i;
    input [8:0]addr_b_i;
    output [35:0]rd_data_b_o;
    
    
    
endmodule
