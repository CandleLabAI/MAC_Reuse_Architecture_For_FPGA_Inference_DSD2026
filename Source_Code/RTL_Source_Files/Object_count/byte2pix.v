module byte2pix (reset_byte_n_i, clk_byte_i, sp_en_i, dt_i, lp_av_en_i, 
            payload_en_i, payload_i, wc_i, reset_pixel_n_i, clk_pixel_i, 
            fv_o, lv_o, pd_o, p_odd_o, write_cycle_o, mem_we_o, 
            mem_re_o, read_cycle_o, fifo_empty_o, fifo_full_o);
    input reset_byte_n_i;
    input clk_byte_i;
    input sp_en_i;
    input [5:0]dt_i;
    input lp_av_en_i;
    input payload_en_i;
    input [7:0]payload_i;
    input [15:0]wc_i;
    input reset_pixel_n_i;
    input clk_pixel_i;
    output fv_o;
    output lv_o;
    output [7:0]pd_o;
    output [1:0]p_odd_o;
    output [3:0]write_cycle_o;
    output mem_we_o;
    output mem_re_o;
    output [1:0]read_cycle_o;
    output fifo_empty_o;
    output fifo_full_o;
    
    
    
endmodule
