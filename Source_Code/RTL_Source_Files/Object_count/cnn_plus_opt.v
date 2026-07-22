module cnn_plus_opt (clk_i, aclk_i, reset_n_i, code_base_addr_i, rd_rdy_o, 
            start_i, lmmi_request_i, lmmi_wr_rdn_i, lmmi_offset_i, lmmi_wdata_i, 
            lmmi_ready_o, lmmi_rdata_valid_o, lmmi_rdata_o, we_o, dout_o, 
            debug_rdy_i, debug_vld_o, gpo_o, status_o, axi4_awid_o, 
            axi4_awaddr_o, axi4_awregion_o, axi4_awlen_o, axi4_awsize_o, 
            axi4_awburst_o, axi4_awlock_o, axi4_awcache_o, axi4_awprot_o, 
            axi4_awqos_o, axi4_awvalid_o, axi4_awready_i, axi4_wid_o, 
            axi4_wdata_o, axi4_wstrb_o, axi4_wlast_o, axi4_wvalid_o, 
            axi4_wready_i, axi4_bid_i, axi4_bresp_i, axi4_bvalid_i, 
            axi4_bready_o, axi4_arid_o, axi4_araddr_o, axi4_arregion_o, 
            axi4_arlen_o, axi4_arsize_o, axi4_arburst_o, axi4_arlock_o, 
            axi4_arcache_o, axi4_arprot_o, axi4_arqos_o, axi4_arvalid_o, 
            axi4_arready_i, axi4_rid_i, axi4_rdata_i, axi4_rresp_i, 
            axi4_rlast_i, axi4_rvalid_i, axi4_rready_o);
    input clk_i;
    input aclk_i;
    input reset_n_i;
    input [31:0]code_base_addr_i;
    output rd_rdy_o;
    input start_i;
    input lmmi_request_i;
    input lmmi_wr_rdn_i;
    input [17:0]lmmi_offset_i;
    input [15:0]lmmi_wdata_i;
    output lmmi_ready_o;
    output lmmi_rdata_valid_o;
    output [15:0]lmmi_rdata_o;
    output we_o;
    output [15:0]dout_o;
    input debug_rdy_i;
    output debug_vld_o;
    output [31:0]gpo_o;
    output [7:0]status_o;
    output [7:0]axi4_awid_o;
    output [31:0]axi4_awaddr_o;
    output [3:0]axi4_awregion_o;
    output [7:0]axi4_awlen_o;
    output [2:0]axi4_awsize_o;
    output [1:0]axi4_awburst_o;
    output axi4_awlock_o;
    output [3:0]axi4_awcache_o;
    output [2:0]axi4_awprot_o;
    output [3:0]axi4_awqos_o;
    output axi4_awvalid_o;
    input axi4_awready_i;
    output [7:0]axi4_wid_o;
    output [63:0]axi4_wdata_o;
    output [7:0]axi4_wstrb_o;
    output axi4_wlast_o;
    output axi4_wvalid_o;
    input axi4_wready_i;
    input [7:0]axi4_bid_i;
    input [1:0]axi4_bresp_i;
    input axi4_bvalid_i;
    output axi4_bready_o;
    output [7:0]axi4_arid_o;
    output [31:0]axi4_araddr_o;
    output [3:0]axi4_arregion_o;
    output [7:0]axi4_arlen_o;
    output [2:0]axi4_arsize_o;
    output [1:0]axi4_arburst_o;
    output axi4_arlock_o;
    output [3:0]axi4_arcache_o;
    output [2:0]axi4_arprot_o;
    output [3:0]axi4_arqos_o;
    output axi4_arvalid_o;
    input axi4_arready_i;
    input [7:0]axi4_rid_i;
    input [63:0]axi4_rdata_i;
    input [1:0]axi4_rresp_i;
    input axi4_rlast_i;
    input axi4_rvalid_i;
    output axi4_rready_o;
    
    
    
endmodule
