module fifo_16in_16out (wr_clk_i, rd_clk_i, rst_i, rp_rst_i, wr_en_i, 
            rd_en_i, wr_data_i, full_o, empty_o, rd_data_o);
    input wr_clk_i;
    input rd_clk_i;
    input rst_i;
    input rp_rst_i;
    input wr_en_i;
    input rd_en_i;
    input [15:0]wr_data_i;
    output full_o;
    output empty_o;
    output [15:0]rd_data_o;
    
    
    
endmodule
