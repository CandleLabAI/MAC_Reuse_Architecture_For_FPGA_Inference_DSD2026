
`timescale 1 ns / 100 ps

module jedi_object_count_top 
(
  input           clk_27_in       ,
  input           clk_24_in       ,
  input 	      gsr_n_i         ,	// push button reset for NX & ECP5
  //-------------- Camera module -------------------------------------------------
  output	      rx2_mclk_o      ,	// 24 MHz
  output          rx2_clk_sel_o   , 	// 1'b1 for MCLK from VNV
  output          rx2_clk_rtc_o   , 	// 1'b0 connect to DGND
  output          rx2_xsdn_o      , 	// reset to HM camera XSHUTDOWN pin
  output          rx2_xsleepn_o   , 	// 1'b1 used for Low Power Sleep Mode
  inout	          rx2_scl         ,
  inout	          rx2_sda         ,
  inout 	      rx2_clk_p_i     ,
  inout 	      rx2_clk_n_i     ,
  inout 	      rx2_d_p_io      ,
  inout 	      rx2_d_n_io      ,
  //------------------FX3 Boot---------------------------------------------------
  output          pmod0_o         ,	// FX3 Boot PMOD0 value 
  output          pmod1_o         ,	// FX3 Boot PMOD1 value 
  output          pmod2_o         ,	// FX3 Boot PMOD2 value 
  output          reset_fx3_o     ,	// FX3 reset
  output          fx3_scl         ,
  input           fx3_sda         ,
  input           pmod0_ctrl_sw0_i,	// Control for PMOD0 of FX3
  output          led0_o          , 
  output          led1_o          , 
  output          led2_o          , 
  output          led3_o          , 
  output 		  led4_o          , 
  output 		  led5_o          , 
  output 		  led6_o          , 
  output 		  led7_o          , 
  output 		  led8_o          , 
  output      	  led9_o          , 
  input           push_button0_i  ,	// Resets the FX3
  //------------- HyperRAM --------------------------------------------------------
  output          hr_ck         	,
  output          hr_ckn        	,
  output          hr_csb        	,
  output          hr_rstb       	,
  inout           hr_rwds       	,
  inout   [7:0]   hr_dq         	,
  //-------------- SD CARD SPI-----------------------------------------------------
  output    	  spi_css       	,
  output    	  spi_clk       	,
  input     	  spi_miso      	,
  output    	  spi_mosi      	,
  //------------- ON BOARD SPI from J9 Connector-----------------------------------
  output    	  spi2_css        ,
  output    	  spi2_clk        ,
  input     	  spi2_miso       ,
  output    	  spi2_mosi       ,
  //------------- OSD out----------------------------------------------------------
  output 		  o_txd		,	//o_tx_serdata
  //---------------------- Video out-----------------------------------------------
  output          clk_24_out      ,
  output 	      fv_o            ,
  output 		  lv_o            ,
  output [15:0]   data_o            
);

`define JEDI_ON
`define RX_CLK_MODE_HS_LP
`define CT_ALGO
`define DEBUG
`define SENSOR_SLAVE_ADDR 7'h24
`define I2C_HIGH_CYCLE 8'd35
`define I2C_LOW_CYCLE 8'd35
`define I2C_GAP_CYCLE 8'd200
`define NUM_OF_TRANS_I2C 509

//  Parameters

`ifdef SENSOR_SLAVE_ADDR
  parameter SENSOR_SLAVE_ADDR        = `SENSOR_SLAVE_ADDR;
`endif

`ifdef I2C_HIGH_CYCLE
  parameter I2C_HIGH_CYCLE           = `I2C_HIGH_CYCLE;
`endif

`ifdef I2C_LOW_CYCLE
  parameter I2C_LOW_CYCLE 	          = `I2C_LOW_CYCLE;
`endif

`ifdef I2C_GAP_CYCLE
  parameter I2C_GAP_CYCLE 	          = `I2C_GAP_CYCLE;
`endif

  parameter        NUM_OF_TRANS      = `NUM_OF_TRANS_I2C;
  parameter [6:0]  SLAVE_ADDR        = SENSOR_SLAVE_ADDR; // I2C Slave Address
  parameter [7:0]  HIGH_CYCLE        = I2C_HIGH_CYCLE;    // 400 kHz operation
  parameter [7:0]  LOW_CYCLE         = I2C_LOW_CYCLE;     // 400 kHz operation
  parameter [7:0]  GAP_COUNT         = I2C_GAP_CYCLE;     // GAP time between two transactions in sys_clk_i cycles  
  parameter 	   idw 	             = 7;
  parameter        BYTE_MODE         = "UNSIGNED";
  parameter [5:0 ] LEFT_TRIM         = 6'd0;
  parameter [11:0] H_TX_PEL          = 12'd640;
  parameter [5:0 ] TOP_TRIM          = 6'd0;
  parameter [11:0] V_TX_LINE         = 12'd480;
  parameter [1:0 ] BAYER_PATTERN     = 2'b00;    	       // 00: normal, 01: H-mirror       
  parameter        VFP               = 6'd2;
  parameter        VS_LENGTH         = 6'd2;
  parameter        HFP               = 6'd2;
  parameter        HS_LENGTH         = 8'd2;
  parameter        NUM_ANCHOR        = 12'd1372,          // number of anchors
       		       NUM_GRID          = 9'd196,	       // number of grids
       		       NUM_X_GRID        = 4'd14,	       // number of X grids
       		       NUM_Y_GRID        = 4'd14,	       // number of Y grids
       		       NUM_FRAC          = 10,	               // bit width of fractional part
       		       OVLP_TH_2X        = 10,		       // overlap threshold 2X (inverse ratio: union / overlap)
       		       PIC_WIDTH         = 224,	       // picture width
       		       PIC_HEIGHT        = 224,	       // picture height
       		       NUM_CLASS         = 5,		       // number of classes
       		       WIDTH	         = 16,
       		       TOP_N_DET         = 5;
  parameter [31:0] HYPERRAM_BASEADDR = 32'h400000;
  parameter [23:0] FLASH_START_ADDR  = 24'h300000;        //Read base addr for Firmware File
  parameter [23:0] FLASH_END_ADDR    = 24'h400000;        //Read End address for Firmware File (Should be in multiple of 512Bytes)
  parameter        INPUT_WIDTH       = 5'd8;
  parameter        MULT_FACTOR_WIDTH = 5'd8;
  parameter        FP_WIDTH          = 5'd16;
  parameter        ID_WIDTH          = 4'd10;
  parameter        OP_WIDTH          = 6'd32;
  parameter        IMG_WIDTH         = 8'd224;
  parameter        IMG_HEIGHT        = 8'd224;
  parameter        DEL_MAX           = 5'd25;
//======================================= INFERENCE TIME ========================================

       parameter   INF_MULT_FAC      = 32'd15907;         // Inference Time multiplying Factor (Calc acc. to CNN Freq and Q-Format)
                                   		               // CNN Freq=135MHz, CNN_Clock_Duration = 1/135us = 0.007407407 us 
                                         	               // Q-Format = Q1.31 => 0.000007407 * 2^31 = 15906.41138 ~ 15906
       parameter 	EN_INF_TIME      = 1'b1;               // Enable/Disable Inference Time Logic Display 

//----------------------- Platform signals------------------------------------------------------ 
wire				            clk;        
wire				            aclk;
wire				            sync_clk;   
wire				            clk_27_pll; 
wire    		                clk_24;
wire    		                clk_72;
wire				            clki;       
wire				            clkq;
wire				            resetn;
wire 				            o_update;
wire [4:0]                      o_box_vld;
wire [25:0]			            o_cls_bmap;
wire [(ID_WIDTH*NUM_CLASS)-1:0] count_list;
wire [OP_WIDTH-1:0]		        final_tracklist1;
wire [OP_WIDTH-1:0]		        final_tracklist2;
wire [OP_WIDTH-1:0]	 	        final_tracklist3;
wire [OP_WIDTH-1:0]	 	        final_tracklist4;
wire [OP_WIDTH-1:0]	 	        final_tracklist5;


// AXI2Memory

// Write Address Channel
wire [idw:0]			A2M_AWID   ;
wire  [31:0]			A2M_AWADDR ;
wire   [3:0]			A2M_AWREGION;
wire   [7:0]			A2M_AWLEN  ;
wire   [2:0]			A2M_AWSIZE ;
wire   [1:0]			A2M_AWBURST;
wire        			A2M_AWLOCK ;
wire   [3:0]			A2M_AWCACHE;
wire   [2:0]			A2M_AWPROT ;
wire   [3:0]			A2M_AWQOS  ;
wire        			A2M_AWVALID;
wire          			A2M_AWREADY;

// Write Channel
wire [idw:0]   			A2M_WID    ;
wire [ 63:0]   			A2M_WDATA  ;
wire  [ 7:0]   			A2M_WSTRB  ;
wire           			A2M_WLAST  ;
wire           			A2M_WVALID ;
wire           			A2M_WREADY ;

// Write Response Channel
wire [idw:0]   			A2M_BID    ;
wire   [1:0]   			A2M_BRESP  ;
wire           			A2M_BVALID ;
wire           			A2M_BREADY ;

// Read Address Channel
wire [idw:0]  			A2M_ARID   ;
wire  [31:0]  			A2M_ARADDR ;
wire   [3:0]  			A2M_ARREGION;
wire   [7:0]  			A2M_ARLEN  ;
wire   [2:0]  			A2M_ARSIZE ;
wire   [1:0]  			A2M_ARBURST;
wire          			A2M_ARLOCK ;
wire   [3:0]  			A2M_ARCACHE;
wire   [2:0]  			A2M_ARPROT ;
wire   [3:0]  			A2M_ARQOS  ;
wire          			A2M_ARVALID;
wire          			A2M_ARREADY;

// Read Channel
wire [idw:0] 			A2M_RID    ;
wire [ 63:0] 			A2M_RDATA  ;
wire   [1:0] 			A2M_RRESP  ;
wire         			A2M_RLAST  ;
wire         			A2M_RVALID ;
wire         			A2M_RREADY ;

// AXI DMA port 0
wire [idw:0]		    A2P0_AWID;   // Address ID
wire [31:0]			    A2P0_AWADDR ;    // Address bus
wire [3:0]			    A2P0_AWREGION;   
wire [7:0]			    A2P0_AWLEN  ;    // Transfer length
wire [2:0]			    A2P0_AWSIZE ;    // Transfer width
wire [1:0]			    A2P0_AWBURST;    // Burst type
wire      			    A2P0_AWLOCK ;    // Atomic access information
wire [3:0]			    A2P0_AWCACHE;    // Cacheable/bufferable information
wire [2:0]			    A2P0_AWPROT ;    // Protection information
wire [3:0]			    A2P0_AWQOS  ;    //
wire				    A2P0_AWVALID;    // Address/Control valid handshake
wire				    A2P0_AWREADY;    // Address/Control ready handshake
                        
wire [idw:0]		    A2P0_WID    ;    // Write data ID
wire [ 63:0]		    A2P0_WDATA  ;    // Write data bus
wire [ 7:0]			    A2P0_WSTRB  ;    // Write data byte lane strobes
wire				    A2P0_WLAST  ;    // Indicats last transfer in write burst
wire				    A2P0_WVALID ;    // Write data valid handshake
wire				    A2P0_WREADY ;    // Write data ready handshake
                        
wire [idw:0]		    A2P0_BID    ;    // Buffered response ID
wire [1:0]			    A2P0_BRESP  ;    // Buffered write response
wire				    A2P0_BVALID ;    // Buffered response valid handshake
wire				    A2P0_BREADY ;    // Buffered response ready handshake
                        
// AXI DMA port 1       
wire [idw:0]   		    A2P1_AWID   ;    // Address ID
wire [31:0]			    A2P1_AWADDR ;    // Address bus
wire [3:0]			    A2P1_AWREGION;   // 
wire [7:0]			    A2P1_AWLEN  ;    // Transfer length
wire [2:0]			    A2P1_AWSIZE ;    // Transfer width
wire [1:0]			    A2P1_AWBURST;    // Burst type
wire      			    A2P1_AWLOCK ;    // Atomic access information
wire [3:0]			    A2P1_AWCACHE;    // Cacheable/bufferable information
wire [2:0]			    A2P1_AWPROT ;    // Protection information
wire [3:0]			    A2P1_AWQOS  ;    //
wire				    A2P1_AWVALID;    // Address/Control valid handshake
wire				    A2P1_AWREADY;    // Address/Control ready handshake
                        
wire [idw:0]		    A2P1_WID    ;    // Write data ID
wire [ 63:0]		    A2P1_WDATA  ;    // Write data bus
wire [ 7:0]			    A2P1_WSTRB  ;    // Write data byte lane strobes
wire				    A2P1_WLAST  ;    // Indicats last transfer in write burst
wire				    A2P1_WVALID ;    // Write data valid handshake
wire				    A2P1_WREADY ;    // Write data ready handshake
                        
wire [idw:0]    	    A2P1_BID    ;    // Buffered response ID
wire [1:0]			    A2P1_BRESP  ;    // Buffered write response
wire				    A2P1_BVALID ;    // Buffered response valid handshake
wire				    A2P1_BREADY ;    // Buffered response ready handshake

// AXI Dma Port 2
wire [idw:0]    		A2P2_AWID    ;
wire  [31:0]    		A2P2_AWADDR  ;
wire   [3:0]    		A2P2_AWREGION;
wire   [7:0]    		A2P2_AWLEN   ;
wire   [2:0]    		A2P2_AWSIZE  ;
wire   [1:0]    		A2P2_AWBURST ;
wire            		A2P2_AWLOCK  ;
wire   [3:0]    		A2P2_AWCACHE ;
wire   [2:0]    		A2P2_AWPROT  ;
wire   [3:0]    		A2P2_AWQOS   ;
wire            		A2P2_AWVALID ;
wire            		A2P2_AWREADY ;
wire [idw:0]    		A2P2_WID     ;
wire  [63:0]    		A2P2_WDATA   ;
wire   [7:0]    		A2P2_WSTRB   ;
wire            		A2P2_WLAST   ;
wire            		A2P2_WVALID  ;
wire            		A2P2_WREADY  ;
wire [idw:0]    		A2P2_BID     ;
wire   [1:0]    		A2P2_BRESP   ;
wire            		A2P2_BVALID  ;
wire            		A2P2_BREADY  ;

wire				            crc_err;
wire				            load_done;
wire   [7:0]		            spi_progress;
                                
wire	        	            w_running;
wire   [7:0]		            w_ml_status;
                                
wire				            w_result_en;
wire  [15:0]		            w_result;
wire				            w_debug_vld;
wire				            w_debug_rdy;
wire				            w_ml_start  ;
reg    [1:0]		            r_ml_start_d;
                                
wire				            w_rd_done;
wire				            w_ml_we;
wire   [16:0]		            w_ml_waddr;
wire   [15:0]		            w_ml_din;
                                
wire				            w_rd_rdy;
reg	        		            r_rd_rdy;
                                
wire				            w_hr_csb   ;
wire				            w_hr_rwds_z;
wire	[1:0]		            w_hr_rwds_o;
wire				            w_hr_dq_z  ;
wire	[15:0]		            w_hr_dq_o  ;
                                
wire	[1:0]		            w_hr_rwds_i;
wire	[15:0]		            w_hr_dq_i  ;
                                
wire 				            rx2_clk_byte_hs;
wire 				            pclk; 
                                
wire				            sensor_config_done;
wire 	[23:0]		            rx2_rgb_pd;
wire 				            rx2_rgb_vs;
wire		 		            rx2_rgb_hs;
wire 				            rx2_rgb_de;
wire 	[23:0]		            rx2_rgb_vid_path_pd;
wire 				            rx2_rgb_vid_path_vs;
wire 				            rx2_rgb_vid_path_hs;
wire				            rx2_rgb_vid_path_de;
wire		 		            nx_reset_n;
wire		 		            int_rst_n;
wire				            master_reset_n;
wire							clk_27;                                
wire				            w_out_en   ;
//wire	[9:0]					w_cls_bmap ;
wire [NUM_CLASS*TOP_N_DET-1:0]	w_cls_bmap ;
wire [NUM_CLASS*TOP_N_DET-1:0]	w_cls_bmap1;
wire [9:0]			            w_bbox_bmap;
wire [31:0]			            w_bbox_00;
wire [31:0]			            w_bbox_01;
wire [31:0]			            w_bbox_02;
wire [31:0]			            w_bbox_03;
wire [31:0]			            w_bbox_04;
wire [31:0]			            w_bbox_05;
wire [31:0]			            w_bbox_06;
wire [31:0]			            w_bbox_07;
wire [31:0]			            w_bbox_08;
wire [31:0]			            w_bbox_09;
                                
wire [4:0]			            w_box_vld;
wire [31:0]			            w_box0;
wire [31:0]			            w_box1;
wire [31:0]			            w_box2;
wire [31:0]			            w_box3;
wire [31:0]			            w_box4;
wire				            pll_lock_24  ;
                                
wire [31:0]			            ind_inf_time_hex;
wire [31:0]			            avg_inf_time_hex;
wire [15:0]			            inf_time_ms;
wire [15:0]			            inf_time_ind_ms;
wire [15:0]			            fps_val;
                                
reg  [15:0]			            bcd_frame_count;
wire [11:0] 			        final_frame_count;
wire                            clk_lp_ctrl;
wire                            reset_lp_n;
reg                             reset_lp_n_meta;
reg                             reset_lp_n_sync;

wire                            i2c_m_rst_n;
wire                            rst_n_i   ;
wire                            sys_clk_i ;	// assuming 27 MHz for 37ns glitch filtering
wire                            done_o    ;	// ch0 I2C transaction done
wire                            ack_err_o ;	// ch0 I2C ACK Error (NACK) flag
wire                            csi_scl, csi_sda;
reg                             w_out_en_d1;
wire [7:0]                      w_pix_r_d2;
wire [7:0]                      w_pix_g_d2;
wire [7:0]                      w_pix_b_d2;
wire 	                        w_hs_d2;
wire 	                        w_vs_d2;
wire 	                        w_de_d2;

wire 	                        o_hs   ;
wire 	                        o_vs   ;
wire 	                        o_de   ;
wire [7:0]                      o_pix_r;
wire [7:0]                      o_pix_g;
wire [7:0]                      o_pix_b;
reg                             fv;
reg                             lv;

wire [15:0]                     data_w;
wire                            fv_w;
wire                            lv_w;
reg  [23:0]                     pix_ycbcr;
wire                            W_fv  ; 
wire                            P_fv;
wire                            W_lv  ;
wire                            P_lv;
wire [15:0]                     W_data; 
wire [15:0]                     P_data;
reg [15:0]                      counter_val;
reg                             led_val;

//----------------------------------------------------------------------------------------------------

`ifdef JEDI_ON
//result fifo
reg			result_fifo_rd;
reg			debug_rdy_clk;
reg			result_fifo_vld;

wire		result_fifo_empty;
wire		result_fifo_full;
wire [15:0]	result_fifo_data;
`endif

`ifdef DEBUG
reg [11:0]                      bcd_obj_0;
reg [11:0]                      bcd_obj_1;
reg [11:0]                      bcd_obj_2;
reg [11:0]                      bcd_obj_3;
reg [11:0]                      bcd_obj_4;

reg [11:0]                      bcd_x1_0;
reg [11:0]                      bcd_x2_0;
reg [11:0]                      bcd_y1_0;
reg [11:0]                      bcd_y2_0;

reg [11:0]                      bcd_x1_1;
reg [11:0]                      bcd_x2_1;
reg [11:0]                      bcd_y1_1;
reg [11:0]                      bcd_y2_1;

reg [11:0]                      bcd_x1_2;
reg [11:0]                      bcd_x2_2;
reg [11:0]                      bcd_y1_2;
reg [11:0]                      bcd_y2_2;

reg [11:0]                      bcd_x1_3;
reg [11:0]                      bcd_x2_3;
reg [11:0]                      bcd_y1_3;
reg [11:0]                      bcd_y2_3;

reg [11:0]                      bcd_x1_4;
reg [11:0]                      bcd_x2_4;
reg [11:0]                      bcd_y1_4;
reg [11:0]                      bcd_y2_4;

reg [11:0]                      bcd_class_obj_0;
reg [11:0]                      bcd_class_obj_1;
reg [11:0]                      bcd_class_obj_2;
reg [11:0]                      bcd_class_obj_3;
reg [11:0]                      bcd_class_obj_4;

reg [11:0]                      bcd_i_box_vld;

wire                            object1_bmap;
wire                            object2_bmap;
wire                            object3_bmap;
wire                            object4_bmap;
wire                            object5_bmap;

wire                            object1_obmap;
wire                            object2_obmap;
wire                            object3_obmap;
wire                            object4_obmap;
wire                            object5_obmap;

wire [4:0] 			             object1_cls;
wire [4:0] 			             object2_cls;
wire [4:0] 			             object3_cls;
wire [4:0] 			             object4_cls;
wire [4:0] 			             object5_cls;
                                 
wire [7:0] 			             object1_x;
wire [7:0] 			             object1_y;
wire [7:0] 			             object1_w;
wire [7:0] 			             object1_h;
                                 
wire [7:0] 			             object2_x;
wire [7:0] 			             object2_y;
wire [7:0] 			             object2_w;
wire [7:0] 			             object2_h;
                                 
wire [7:0] 			             object3_x;
wire [7:0] 			             object3_y;
wire [7:0] 			             object3_w;
wire [7:0] 			             object3_h;
                                 
wire [7:0] 			             object4_x;
wire [7:0] 			             object4_y;
wire [7:0] 			             object4_w;
wire [7:0] 			             object4_h;
                                 
wire [7:0] 			             object5_x;
wire [7:0] 			             object5_y;
wire [7:0] 			             object5_w;
wire [7:0] 			             object5_h;
`endif

//---------------------------------Platform blocks--------------------------------------------------------
// Platform blocks {{{

assign master_reset_n  = gsr_n_i ;
assign int_rst_n       = master_reset_n & nx_reset_n;

assign rx2_mclk_o = clk_24_in;	
assign clk_27     = clk_27_in;
assign aclk       = clki;

assign clk_byte_fr = clk_24_in;
assign pclk        = clk_24_in;

assign clk_lp_ctrl = sync_clk;	// > 20 MHz
assign reset_lp_n  = reset_lp_n_sync;

// PLL, clock and reset generation
lsc_resetn 
u_lsc_resetn 
(
    .clk      ( clk           ),
    .i_resetn ( int_rst_n     ),
    .o_resetn ( resetn        )
);

pll_2xq_5x_dynport
u_pll_2xq_5x 
(
    .clki_i     ( clk_27_in  ),
    .rstn_i	    ( int_rst_n  ),
    .clkop_o    ( clki       ), // x2
    .clkos_o    ( clkq       ), // x2
    .clkos2_o   ( clk        ), // x5: 135MHz
    .clkos3_o   (            ), // x3: 81MHz
    .clkos4_o   ( clk_27_pll ), // x1: 27MHz
    .lock_o           (),
    .done_pll_init_o  ()
);

always @(posedge clk_lp_ctrl or negedge int_rst_n) begin
	if (~int_rst_n) begin
		reset_lp_n_meta <= 0;
		reset_lp_n_sync <= 0;
	end
	else begin
		reset_lp_n_meta <= int_rst_n;
		reset_lp_n_sync <= reset_lp_n_meta;
	end
end

int_osc int_osc_int
(
.hf_out_en_i    ( 1'b1      ), 
.hf_clk_out_o   ( sync_clk  )  
);

//===========================================
//               HyperRAM Blocks
//===========================================

`ifdef JEDI_ON

//========================================
//               Hyber bus
//========================================
lsc_hyperbus_io 
u_lsc_hyperbus_io 
(
    .clki          ( clki          ),
    .clkq          ( clkq          ),
    .resetn        ( resetn        ),
    .i_hr_csb      ( w_hr_csb      ),
    .i_hr_rwds_z   ( w_hr_rwds_z   ),
    .i_hr_rwds_o   ( w_hr_rwds_o   ),
    .i_hr_dq_z     ( w_hr_dq_z     ),
    .i_hr_dq_o     ( w_hr_dq_o     ),
    .o_hr_rwds_i   ( w_hr_rwds_i   ),
    .o_hr_dq_i     ( w_hr_dq_i     ),
    .hr_ck         ( hr_ck         ),
    .hr_ckn        ( hr_ckn        ),
    .hr_csb        ( hr_csb        ),
    .hr_rstb       ( hr_rstb       ),
    .hr_rwds       ( hr_rwds       ),
    .hr_dq         ( hr_dq         )
);

//========================================
//               AXI Interface 
//========================================
axi2hyperbus #(
    .idw	   ( idw 	   )) 
u_axi2hyperbus 
(
    .clk               ( aclk           ),
    .resetn            ( resetn         ),
    .o_hr_csb          ( w_hr_csb       ),
    .o_hr_rwds_z       ( w_hr_rwds_z    ),
    .o_hr_rwds_o       ( w_hr_rwds_o    ),
    .o_hr_dq_z         ( w_hr_dq_z      ),
    .o_hr_dq_o         ( w_hr_dq_o      ),
    .i_hr_rwds_i       ( w_hr_rwds_i    ),
    .i_hr_dq_i         ( w_hr_dq_i      ),
    .ARID              ( A2M_ARID       ),
    .ARADDR            ( A2M_ARADDR     ),
    .ARLEN             ( A2M_ARLEN      ),
    .ARSIZE            ( A2M_ARSIZE     ), 
    .ARBURST           ( A2M_ARBURST    ), 
    .ARLOCK            ( A2M_ARLOCK     ), 
    .ARCACHE           ( A2M_ARCACHE    ), 
    .ARPROT            ( A2M_ARPROT     ), 
    .ARVALID           ( A2M_ARVALID    ),
    .ARREADY           ( A2M_ARREADY    ),
    .RID               ( A2M_RID        ),
    .RDATA             ( A2M_RDATA      ),
    .RRESP             ( A2M_RRESP      ), 
    .RLAST             ( A2M_RLAST      ),
    .RVALID            ( A2M_RVALID     ),
    .RREADY            ( A2M_RREADY     ),
    .AWID              ( A2M_AWID       ),
    .AWADDR            ( A2M_AWADDR     ),
    .AWLEN             ( A2M_AWLEN      ),
    .AWSIZE            ( A2M_AWSIZE     ), 
    .AWBURST           ( A2M_AWBURST    ), 
    .AWLOCK            ( A2M_AWLOCK     ), 
    .AWCACHE           ( A2M_AWCACHE    ), 
    .AWPROT            ( A2M_AWPROT     ), 
    .AWVALID           ( A2M_AWVALID    ),
    .AWREADY           ( A2M_AWREADY    ),
    .WID               ( A2M_WID        ),
    .WDATA             ( A2M_WDATA      ),
    .WSTRB             ( A2M_WSTRB      ), 
    .WLAST             ( A2M_WLAST      ),
    .WVALID            ( A2M_WVALID     ),
    .WREADY            ( A2M_WREADY     ),
    .BID               ( A2M_BID        ),
    .BRESP             ( A2M_BRESP      ), 
    .BVALID            ( A2M_BVALID     ),
    .BREADY            ( A2M_BREADY     )
);

axi_ws2m1 #(.dw        (63 		),
            .stw       (7  		),
            .idw       (idw		))
u_axi_ws2m1 
(
    // Global inputs
    .ACLK      	       ( aclk        ),
    .ARESETn           ( resetn      ),
    // SlaveInterface 0 (connects to Master 0)
    .IDMASKS0          ( 8'hf0       ),
    // Write Address Channel
    .AWIDS0            ( A2P0_AWID   ),
    .AWADDRS0          ( A2P0_AWADDR ),
    .AWREGIONS0        ( A2P0_AWREGION),
    .AWLENS0           ( A2P0_AWLEN  ),
    .AWSIZES0          ( A2P0_AWSIZE ),
    .AWBURSTS0         ( A2P0_AWBURST),
    .AWLOCKS0          ( A2P0_AWLOCK ),
    .AWCACHES0         ( A2P0_AWCACHE),
    .AWPROTS0          ( A2P0_AWPROT ),
    .AWQOSS0           ( A2P0_AWQOS  ),
    .AWVALIDS0         ( A2P0_AWVALID),
    .AWREADYS0         ( A2P0_AWREADY),
    // Write Channel
    .WIDS0   	       ( A2P0_WID   ),
    .WDATAS0 	       ( A2P0_WDATA ),
    .WSTRBS0 	       ( A2P0_WSTRB ),
    .WLASTS0 	       ( A2P0_WLAST ),
    .WVALIDS0	       ( A2P0_WVALID),
    .WREADYS0	       ( A2P0_WREADY),
    // Write Response Channel
    .BIDS0    	       ( A2P0_BID   ),
    .BRESPS0  	       ( A2P0_BRESP ),
    .BVALIDS0 	       ( A2P0_BVALID),
    .BREADYS0 	       ( A2P0_BREADY),
    // SlaveInterface 1 (connects to Master 1 (I/O interface))
    .IDMASKS1  	       ( 8'hf0       ),
    // Write Address Channel
    .AWIDS1            ( A2P1_AWID   ),
    .AWADDRS1          ( A2P1_AWADDR ),
    .AWREGIONS1        ( A2P1_AWREGION),
    .AWLENS1           ( A2P1_AWLEN  ),
    .AWSIZES1          ( A2P1_AWSIZE ),
    .AWBURSTS1         ( A2P1_AWBURST),
    .AWLOCKS1          ( A2P1_AWLOCK ),
    .AWCACHES1         ( A2P1_AWCACHE),
    .AWPROTS1          ( A2P1_AWPROT ),
    .AWQOSS1           ( A2P1_AWQOS  ),
    .AWVALIDS1         ( A2P1_AWVALID),
    .AWREADYS1         ( A2P1_AWREADY),
    // Write Channel
    .WIDS1             ( A2P1_WID   ),
    .WDATAS1           ( A2P1_WDATA ),
    .WSTRBS1           ( A2P1_WSTRB ),
    .WLASTS1           ( A2P1_WLAST ),
    .WVALIDS1          ( A2P1_WVALID),
    .WREADYS1          ( A2P1_WREADY),
    // Write Response Channel
    .BIDS1             ( A2P1_BID   ),
    .BRESPS1           ( A2P1_BRESP ),
    .BVALIDS1          ( A2P1_BVALID),
    .BREADYS1          ( A2P1_BREADY),
    // SlaveInterface 2 (connects to Master 1 (I/O interface))
    .IDMASKS2          ( 8'hf0       ),
    // Write Address Channel
    .AWIDS2            ( A2P2_AWID   ),
    .AWADDRS2          ( A2P2_AWADDR ),
    .AWREGIONS2        ( A2P2_AWREGION),
    .AWLENS2           ( A2P2_AWLEN  ),
    .AWSIZES2          ( A2P2_AWSIZE ),
    .AWBURSTS2         ( A2P2_AWBURST),
    .AWLOCKS2          ( A2P2_AWLOCK ),
    .AWCACHES2         ( A2P2_AWCACHE),
    .AWPROTS2          ( A2P2_AWPROT ),
    .AWQOSS2           ( A2P2_AWQOS  ),
    .AWVALIDS2         ( A2P2_AWVALID),
    .AWREADYS2         ( A2P2_AWREADY),
    // Write Channel
    .WIDS2             ( A2P2_WID   ),
    .WDATAS2           ( A2P2_WDATA ),
    .WSTRBS2           ( A2P2_WSTRB ),
    .WLASTS2           ( A2P2_WLAST ),
    .WVALIDS2          ( A2P2_WVALID),
    .WREADYS2          ( A2P2_WREADY),
    // Write Response Channel
    .BIDS2             ( A2P2_BID   ),
    .BRESPS2           ( A2P2_BRESP ),
    .BVALIDS2          ( A2P2_BVALID),
    .BREADYS2          ( A2P2_BREADY),
    // MasterInterface 0 (connects to Slave 0)
    // Write Address Channel
    .AWIDM0            ( A2M_AWID   ),
    .AWADDRM0          ( A2M_AWADDR ),
    .AWREGIONM0        ( A2M_AWREGION),
    .AWLENM0           ( A2M_AWLEN  ),
    .AWSIZEM0          ( A2M_AWSIZE ),
    .AWBURSTM0         ( A2M_AWBURST),
    .AWLOCKM0          ( A2M_AWLOCK ),
    .AWCACHEM0         ( A2M_AWCACHE),
    .AWPROTM0          ( A2M_AWPROT ),
    .AWQOSM0           ( A2M_AWQOS  ),
    .AWVALIDM0         ( A2M_AWVALID),
    .AWREADYM0         ( A2M_AWREADY),
    // Write Channel
    .WIDM0             ( A2M_WID    ),
    .WDATAM0           ( A2M_WDATA  ),
    .WSTRBM0           ( A2M_WSTRB  ),
    .WLASTM0           ( A2M_WLAST  ),
    .WVALIDM0          ( A2M_WVALID ),
    .WREADYM0          ( A2M_WREADY ),
    // Write Response Channel
    .BIDM0             ( A2M_BID    ),
    .BRESPM0           ( A2M_BRESP  ),
    .BVALIDM0          ( A2M_BVALID ),
    .BREADYM0          ( A2M_BREADY )
);

//========================================
//               SPI Flash
//========================================              

spi_loader_spram #(.idw(idw),
                   .FLASH_START_ADDR (FLASH_START_ADDR),
                   .FLASH_END_ADDR   (FLASH_END_ADDR)) 
u_spi_flash 
(
    .clk               ( aclk           ),
    .resetn            ( resetn         ),
    .i_init            ( 1'b1           ),
    .o_load_done       ( load_done      ),
    .i_burst_lmt       ( 8'h0f          ),
    .SPI_CLK           ( spi2_clk       ),
    .SPI_CSS           ( spi2_css       ),
    .SPI_MISO          ( spi2_miso      ),
    .SPI_MOSI          ( spi2_mosi      ),
    .ACLK              ( aclk           ),
    .ARESETn           ( resetn         ),
    .AWID              ( A2P0_AWID[3:0] ),
    .AWADDR            ( A2P0_AWADDR    ),
    .AWLEN             ( A2P0_AWLEN     ),
    .AWSIZE            ( A2P0_AWSIZE    ), 
    .AWBURST           ( A2P0_AWBURST   ), 
    .AWLOCK            ( A2P0_AWLOCK    ), 
    .AWCACHE           ( A2P0_AWCACHE   ), 
    .AWPROT            ( A2P0_AWPROT    ), 
    .AWVALID           ( A2P0_AWVALID   ),
    .AWREADY           ( A2P0_AWREADY   ),
    .WID               ( A2P0_WID[3:0]  ),
    .WDATA             ( A2P0_WDATA     ),
    .WSTRB             ( A2P0_WSTRB     ), 
    .WLAST             ( A2P0_WLAST     ),
    .WVALID            ( A2P0_WVALID    ),
    .WREADY            ( A2P0_WREADY    ),
    .BID               ( A2P0_BID       ),
    .BRESP             ( A2P0_BRESP     ), 
    .BVALID            ( A2P0_BVALID    ),
    .BREADY            ( A2P0_BREADY    )
);
assign A2P0_AWID [idw:4] = 1;
assign A2P0_WID [idw:4]  = 1;

`endif

// Platform blocks }}}

//----------------------------------------------------------------------------------------------------

//---------------------------------------Input Side of the Video Path---------------------------------

// Input Side of the Video Path {{{

//========================================
//          Camera Configuration
//========================================  

assign rx2_clk_sel_o = 1'b1; 
assign rx2_clk_rtc_o = 1'b0;
assign rx2_xsleepn_o = 1'b1;

rst_ctrl 
rst_ctrl 
(
    .rst_n_i		(master_reset_n  ),
    .clk_i		    (clk_24_in       ),	// 27 MHz
    .i2c_m_rst_n_o	(i2c_m_rst_n     ),
    .sensor_rst_n_o	(rx2_xsdn_o      ),
    .nx_rst_n_o		(nx_reset_n      )
);

assign rst_n_i       = i2c_m_rst_n;
assign sys_clk_i     = clk_24_in; 
assign rx2_scl       = csi_scl;  
assign rx2_sda       = csi_sda ? 1'bz : 1'b0;

i2c_single  #(
	.SLAVE_ADDR		( SLAVE_ADDR ),
	.NUM_OF_TRANS   ( NUM_OF_TRANS ),
	.HIGH_CYCLE		( HIGH_CYCLE ),
	.LOW_CYCLE		( LOW_CYCLE ),
	.GAP_COUNT		( GAP_COUNT ) 
) i2c_single (
	.sys_clk_i		( sys_clk_i ),  // 24 MHz
	.rst_n_i		( rst_n_i ),
	.scl_i        	( rx2_scl ),    //(scl_in),
	.scl_o			( csi_scl ),
	.sda_i			( rx2_sda ),    //(sda_in),
	.sda_o			( csi_sda ),
	.done_o	        ( done_o ),
	.ack_err_o      ( ack_err_o )
);

lsc_i2c_auto_bus_reset u_lsc_l2c_auto_bus_reset (
    .clk      ( clk_24_in  ),
    .resetn   ( resetn     ),
    .i_sda    ( fx3_sda    ), 
    .o_scl    ( fx3_scl    ),
    .o_resetn ( reset_fx3_o)  //(fx3_resetn)
);

//========================================
//       CH #0 CSI-2 to parallel
//========================================

csi2_to_parallel #(
	.NUM_RX_LANE		    (1),
	.RX_GEAR		        (8),
	.RX_PD_BUS_WIDTH	    (8),
	.LB_DEPTH		       	(4096),
	.TX_PD_BUS_WIDTH	    (24),
	.V_TOTAL_LINE		    (480)
) csi2_to_p0 (
	.hs_sync_o				(hs_sync0_o),
	.sp_en_o				(sp_en0),
	.lp_av_en_o				(lp_av_en0),
	.payload_en_o			(payload_en0),
	.ready_o	    		(ready_0),
	.reset_n_i			    (resetn),
	.clk_p_i			    (rx2_clk_p_i),
	.clk_n_i			    (rx2_clk_n_i),
	.d_p_io				    (rx2_d_p_io),
	.d_n_io				    (rx2_d_n_io),
	.pd_dphy_i			    (~resetn),
    .sync_clk_i				(sync_clk),
    .sync_rst_i				(~resetn),
	.tx_rdy_i				(1'b1),
	.clk_lp_ctrl_i			(sync_clk),
	.reset_lp_n_i			(resetn),
	.clk_byte_fr_i			(clk_byte_fr),
	.reset_byte_fr_n_sync_i	(resetn),
	.reset_byte_n_sync_i	(resetn),
	.clk_byte_o				(),
	.clk_byte_hs_o			(rx2_clk_byte_hs),
	.ref_dt_i				(6'h2a),
	.clk_pixel_i			(pclk),
	.reset_pixel_n_sync_i	(resetn),
	.vfp_i					(VFP),
	.vs_length_i			(VS_LENGTH),
	.hfp_i					(HFP),
	.hs_length_i			(HS_LENGTH),
	.bayer_pattern_i		(BAYER_PATTERN),
	.top_trim_i				(TOP_TRIM),
	.v_tx_line_i			(V_TX_LINE),
	.left_trim_i			(LEFT_TRIM),
	.h_tx_pel_i			    (H_TX_PEL),
	.rgb_vs_o			    (rx2_rgb_vid_path_vs),
	.rgb_hs_o			    (rx2_rgb_vid_path_hs),
	.rgb_de_o			    (rx2_rgb_vid_path_de),
	.rgb_pd_o			    (rx2_rgb_vid_path_pd)
);

assign rx2_rgb_pd = rx2_rgb_vid_path_pd;
assign rx2_rgb_hs = rx2_rgb_vid_path_hs;  
assign rx2_rgb_vs = rx2_rgb_vid_path_vs;
assign rx2_rgb_de = rx2_rgb_vid_path_de;

//=====================================
//          Crop & Downscale
//=====================================

crop_downscale_front_224x224 #(.BYTE_MODE         (BYTE_MODE        ),
                               .idw               (idw              ),
                               .HYPERRAM_BASEADDR (HYPERRAM_BASEADDR),
                               .PIC_HEIGHT        (PIC_HEIGHT       ),
                               .PIC_WIDTH         (PIC_WIDTH        ),
                               .INF_MULT_FAC      (INF_MULT_FAC     ),
                               .EN_INF_TIME       (EN_INF_TIME      ))
u_crop_downscale_front_224x224 
(
    .pclk             (pclk             ),
    .clk              (clk              ),
    .resetn           (resetn           ),
    .i_rd_rdy         (r_rd_rdy         ),
    .o_rd_done        (w_rd_done        ),
    .i_hs             (rx2_rgb_hs       ),
    .i_vs             (rx2_rgb_vs       ),
    .i_de             (rx2_rgb_de       ),
    .i_r              (rx2_rgb_pd[23:16]),
    .i_g              (rx2_rgb_pd[15: 8]),
    .i_b              (rx2_rgb_pd[ 7: 0]),
    .o_hs_d2          (w_hs_d2          ),
    .o_vs_d2          (w_vs_d2          ),
    .o_de_d2          (w_de_d2          ),
    .o_r_d2           (w_pix_r_d2       ),
    .o_g_d2           (w_pix_g_d2       ),
    .o_b_d2           (w_pix_b_d2       ),
    .o_mask           (   	          ),
    .ind_inf_time_hex (ind_inf_time_hex	),
    .avg_inf_time_hex (avg_inf_time_hex	),
    .inf_time_ms      (inf_time_ms     	),
    .inf_time_ind_ms  (inf_time_ind_ms	),
    .fps_val 	      (fps_val		),

    .ACLK             (aclk             ), 
    .ARESETn          (resetn           ), 
    .AWID             (A2P2_AWID        ), 
    .AWADDR           (A2P2_AWADDR      ),  
    .AWREGION         (A2P2_AWREGION    ), 
    .AWLEN            (A2P2_AWLEN       ),
    .AWSIZE           (A2P2_AWSIZE      ),
    .AWBURST          (A2P2_AWBURST     ),
    .AWLOCK           (A2P2_AWLOCK      ),
    .AWCACHE          (A2P2_AWCACHE     ),
    .AWPROT           (A2P2_AWPROT      ),
    .AWQOS            (A2P2_AWQOS       ),
    .AWVALID          (A2P2_AWVALID     ),
    .AWREADY          (A2P2_AWREADY     ),
                      
    .WID              (A2P2_WID[3:0]    ),
    .WDATA            (A2P2_WDATA       ),
    .WSTRB            (A2P2_WSTRB       ),
    .WLAST            (A2P2_WLAST       ),
    .WVALID           (A2P2_WVALID      ),
    .WREADY           (A2P2_WREADY      ),
                      
    .BID              (A2P2_BID[7:0]    ),
    .BRESP            (A2P2_BRESP       ),
    .BVALID           (A2P2_BVALID      ),
    .BREADY           (A2P2_BREADY      ) 
);

// Input Side of the Video Path }}}

//----------------------------------------------------------------------------------------------------

//---------------------------------------ML ENGINE----------------------------------------------------

// ML engine {{{ 

`ifdef JEDI_ON 

assign w_ml_start        = w_rd_done & load_done;

reg			r_ml_start;
reg [3:0]	start_cnt;
reg			result_fifo_rst;

always @(posedge clk)
begin
    if(w_ml_start)
	start_cnt <= 4'hf;
    else if(start_cnt != 4'h0)
	start_cnt <= start_cnt - 4'h1;
end

always @(posedge clk)
begin
    r_ml_start <= (start_cnt != 4'h0);
end

always @(posedge clk_24_in)
begin
    r_ml_start_d    <= {r_ml_start_d[0], r_ml_start};
    result_fifo_rst <= (r_ml_start_d == 2'b01);
end

//========================================
//               ML Engine 
//======================================== 
 
cnn_plus_opt lsc_ml(
    .gpo_o             (             ),
    .clk_i             ( clk          ),
    .aclk_i            ( aclk         ),
    .reset_n_i         ( resetn       ),
    .code_base_addr_i  ( 32'b0        ),
    .rd_rdy_o          ( w_rd_rdy     ),
    .start_i           ( w_ml_start   ),
    .lmmi_request_i    ( w_ml_we     ),
    .lmmi_wr_rdn_i     ( 1'b1        ),
    .lmmi_offset_i     ( w_ml_waddr  ),
    .lmmi_wdata_i      ( w_ml_din    ),
    .lmmi_ready_o      (             ),
    .lmmi_rdata_valid_o(             ),
    .lmmi_rdata_o      (             ),
    .we_o              ( w_result_en  ),
    .dout_o            ( w_result     ),
    .debug_rdy_i       ( w_debug_rdy  ),
    .debug_vld_o       ( w_debug_vld  ),
    .status_o          (  w_ml_status ),
    .axi4_awid_o       ( A2P1_AWID[3:0] ),
    .axi4_awaddr_o     ( A2P1_AWADDR    ),
    .axi4_awregion_o   ( A2P1_AWREGION  ),
    .axi4_awlen_o      ( A2P1_AWLEN     ),
    .axi4_awsize_o     ( A2P1_AWSIZE    ),
    .axi4_awburst_o    ( A2P1_AWBURST   ),
    .axi4_awlock_o     ( A2P1_AWLOCK    ),
    .axi4_awcache_o    ( A2P1_AWCACHE   ),
    .axi4_awprot_o     ( A2P1_AWPROT    ),
    .axi4_awqos_o      ( A2P1_AWQOS     ),
    .axi4_awvalid_o    ( A2P1_AWVALID   ),
    .axi4_awready_i    ( A2P1_AWREADY   ),
    .axi4_wid_o        ( A2P1_WID[3:0]  ),
    .axi4_wdata_o      ( A2P1_WDATA     ),
    .axi4_wstrb_o      ( A2P1_WSTRB     ),
    .axi4_wlast_o      ( A2P1_WLAST     ),
    .axi4_wvalid_o     ( A2P1_WVALID    ),
    .axi4_wready_i     ( A2P1_WREADY    ),
    .axi4_bid_i        ( A2P1_BID[7:0]  ),
    .axi4_bresp_i      ( A2P1_BRESP     ),
    .axi4_bvalid_i     ( A2P1_BVALID    ),
    .axi4_bready_o     ( A2P1_BREADY    ),
    .axi4_arid_o       ( A2M_ARID      ),
    .axi4_araddr_o     ( A2M_ARADDR    ),
    .axi4_arregion_o   ( A2M_ARREGION  ),
    .axi4_arlen_o      ( A2M_ARLEN     ),
    .axi4_arsize_o     ( A2M_ARSIZE    ),
    .axi4_arburst_o    ( A2M_ARBURST   ),
    .axi4_arlock_o     ( A2M_ARLOCK    ),
    .axi4_arcache_o    ( A2M_ARCACHE   ),
    .axi4_arprot_o     ( A2M_ARPROT    ),
    .axi4_arqos_o      ( A2M_ARQOS     ),
    .axi4_arvalid_o    ( A2M_ARVALID   ),
    .axi4_arready_i    ( A2M_ARREADY   ),
    .axi4_rid_i        ( A2M_RID       ),
    .axi4_rdata_i      ( A2M_RDATA     ),
    .axi4_rresp_i      ( A2M_RRESP     ),
    .axi4_rlast_i      ( A2M_RLAST     ),
    .axi4_rvalid_i     ( A2M_RVALID    ),
    .axi4_rready_o     ( A2M_RREADY    )
);

assign A2P1_AWID[idw:4]  = 2;
assign A2P1_WID[idw:4]   = 2;

always @(posedge clk or negedge resetn)
begin
    if(resetn == 1'b0)
	r_rd_rdy <= 1'b0;
    else 
	r_rd_rdy <= load_done & w_rd_rdy;
end

`endif

// ML engine }}}

//----------------------------------------------------------------------------------------------------

//---------------------------------------Post Processing----------------------------------------------

// Post Processing {{{

`ifdef JEDI_ON

//result fifo
reg			result_fifo_rd;
reg			debug_rdy_clk;
reg			result_fifo_vld;

wire		result_fifo_empty;
wire		result_fifo_full;
wire [15:0]	result_fifo_data;

always @(posedge clk)
begin
    debug_rdy_clk <= result_fifo_empty;
end

assign w_debug_rdy = debug_rdy_clk;

always @(posedge clk_24_in)
begin
    result_fifo_rd  <= !result_fifo_empty;
    result_fifo_vld <= (!result_fifo_empty) & result_fifo_rd;
end

//========================================
//     CDC - Clock Domain Crossing
//========================================
fifo_16in_16out u_fifo_16in_16out (
    .rst_i         ( result_fifo_rst  ),
    .rp_rst_i      ( result_fifo_rst  ),
    .wr_clk_i      ( clk              ),
    .rd_clk_i      ( clk_24_in        ),
    .wr_data_i     ( w_result         ),
    .wr_en_i       ( w_debug_vld      ),
    .rd_en_i       ( result_fifo_rd   ),
    .rd_data_o     ( result_fifo_data ),
    .full_o        ( result_fifo_full ),
    .empty_o       ( result_fifo_empty)
);

//====================================
//         Detection processing
//====================================

det_out_filter #(
                 .NUM_ANCHOR (NUM_ANCHOR), 	
                 .NUM_GRID   (NUM_GRID	), 
                 .NUM_X_GRID (NUM_X_GRID), 	
                 .NUM_Y_GRID (NUM_Y_GRID), 	
                 .NUM_FRAC   (NUM_FRAC	), 
                 .OVLP_TH_2X (OVLP_TH_2X), 	
                 .PIC_WIDTH  (PIC_WIDTH	),
                 .PIC_HEIGHT (PIC_HEIGHT), 
				 .NUM_CLASS  (NUM_CLASS),	 
                 .TOP_N_DET  (TOP_N_DET	) 
) u_det_out_filter (
    .clk          ( clk_24_in       ),
    .resetn       ( resetn          ),
    .i_conf_th    ( 16'd500         ),
    .i_out_start  ( result_fifo_rst ),
    .i_data_en    ( result_fifo_vld ),
    .i_data       ( result_fifo_data),
    .o_out_en     ( w_out_en        ),
    .o_cls_bmap   ( w_cls_bmap      ),
    .o_bbox_bmap  ( w_bbox_bmap     ),
    .o_bbox_00    ( w_bbox_00       ),
    .o_bbox_01    ( w_bbox_01       ),
    .o_bbox_02    ( w_bbox_02       ),
    .o_bbox_03    ( w_bbox_03       ),
    .o_bbox_04    ( w_bbox_04       ),
    .o_bbox_05    ( w_bbox_05       ),
    .o_bbox_06    ( w_bbox_06       ),
    .o_bbox_07    ( w_bbox_07       ),
    .o_bbox_08    ( w_bbox_08       ),
    .o_bbox_09    ( w_bbox_09       )
);

always @(posedge clk_24_in, negedge resetn) begin
	if (!resetn) begin
		w_out_en_d1 <= 1'b0;
	end
	else begin
		w_out_en_d1 <= w_out_en;
    end
end

//====================================
//         Bounding box processing
//====================================

bbox2box  #(
    .TOP_N_DET       ( TOP_N_DET ),
    .NUM_CLASS       ( NUM_CLASS ),
    .X1_MIDDLE       ( 60 		 ),
    .X2_MIDDLE       ( 224 		 ),
    .MINX	     	 ( 5 		 ),
    .MINY	     	 ( 10 		 )
) u_bbox2box	(
    .clk             ( clk_24_in    ),
    .resetn          ( resetn       ),
    .i_update        ( w_out_en & !w_out_en_d1),
    .i_cls_bmap      ( w_cls_bmap	   ),
    .i_bbox_bmap     ( w_bbox_bmap  ),
    .i_bbox_00       ( w_bbox_00    ), // h, w, y, x
    .i_bbox_01       ( w_bbox_01    ),
    .i_bbox_02       ( w_bbox_02    ),
    .i_bbox_03       ( w_bbox_03    ),
    .i_bbox_04       ( w_bbox_04    ),
    .i_bbox_05       ( w_bbox_05    ),
    .i_bbox_06       ( w_bbox_06    ),
    .i_bbox_07       ( w_bbox_07    ),
    .i_bbox_08       ( w_bbox_08    ),
    .i_bbox_09       ( w_bbox_09    ),
    .o_cls_bmap      ( w_cls_bmap1  ),
    .o_box_vld       ( w_box_vld   ),
    .o_box0          ( w_box0      ), // y2, y1, x2, x1
    .o_box1          ( w_box1      ),
    .o_box2          ( w_box2      ),
    .o_box3          ( w_box3      ),
    .o_box4          ( w_box4      ),
    .o_update        ( o_update	  )
);

//====================================
//         Centroid Algorithm
//====================================

`ifdef CT_ALGO

centroid_tracker #(
	.INPUT_WIDTH   		( INPUT_WIDTH	),
	.NUM_CLASS	   		( NUM_CLASS		),
	.MULT_FACTOR_WIDTH  ( MULT_FACTOR_WIDTH	),
	.FP_WIDTH	        ( FP_WIDTH		),
	.ID_WIDTH	        ( ID_WIDTH		),
	.OP_WIDTH	        ( OP_WIDTH		),
	.IMG_WIDTH	        ( IMG_WIDTH    	),
	.IMG_HEIGHT	        ( IMG_HEIGHT	),
	.DEL_MAX	        ( DEL_MAX		)
) centroid_tracker_i 	  (
	.clk		      ( clk_24_in	),
	.resetn		      ( resetn		),
	.i_vs		      ( o_update	),	// Output from bbox2box: y2, y1, x2, x1
	.i_bbox_00	      ( {w_box0[7:0],w_box0[23:16],w_box0[15:8],w_box0[31:24]} ), // Input required: x1, y1, x2, y2
	.i_bbox_01	      ( {w_box1[7:0],w_box1[23:16],w_box1[15:8],w_box1[31:24]} ),
	.i_bbox_02	      ( {w_box2[7:0],w_box2[23:16],w_box2[15:8],w_box2[31:24]} ),
	.i_bbox_03	      ( {w_box3[7:0],w_box3[23:16],w_box3[15:8],w_box3[31:24]} ),
	.i_bbox_04	      ( {w_box4[7:0],w_box4[23:16],w_box4[15:8],w_box4[31:24]} ),
	.i_bbox_bmap      ( w_box_vld		),
	.i_cls_bmap	      ( w_cls_bmap1		),
	.o_box_vld	      ( o_box_vld		),
	.o_cls_bmap	      ( o_cls_bmap		),
	.count_list	      ( count_list          ),
	.final_tracklist1 ( final_tracklist1	),
	.final_tracklist2 ( final_tracklist2	),
	.final_tracklist3 ( final_tracklist3	),
	.final_tracklist4 ( final_tracklist4	),
	.final_tracklist5 ( final_tracklist5	),
	.frame_count      ( final_frame_count   )
);
`endif

`endif

// Post Processing }}}

//----------------------------------------------------------------------------------------------------

//-------------------------------Output Side of the Video Path----------------------------------------

// Output Side of the Video Path {{{

//====================================
//         OSD Display module
//====================================

osd_back_224x224_object_count #(.EN_INF_TIME  ( EN_INF_TIME),
				                .TOP_N_DET    ( TOP_N_DET),
				                .NUM_CLASS    ( NUM_CLASS),
				                .ID_WIDTH     ( ID_WIDTH)
) u_osd_back_224x224_object_count 
(
    .pclk	          ( pclk	),
    .clk	          ( clk		),
    .resetn	          ( resetn	),
    .o_objdet         ( o_objdet ),
    `ifdef CT_ALGO
    .i_cls_bmap       ( {25'd0,o_cls_bmap} ),  //w_cls_bmap1
    .i_box_vld        ( {5'b0, o_box_vld}  ),  //w_box_vld
    .count_list       ( count_list         ),										
    .i_box_00         ( {final_tracklist1[7:0],final_tracklist1[23:16],final_tracklist1[15:8],final_tracklist1[31:24]}), // Input to OSD : y2, y1, x2, x1//w_box0
    .i_box_01         ( {final_tracklist2[7:0],final_tracklist2[23:16],final_tracklist2[15:8],final_tracklist2[31:24]} ),//w_box1
    .i_box_02         ( {final_tracklist3[7:0],final_tracklist3[23:16],final_tracklist3[15:8],final_tracklist3[31:24]} ),//w_box2
    .i_box_03         ( {final_tracklist4[7:0],final_tracklist4[23:16],final_tracklist4[15:8],final_tracklist4[31:24]} ),//w_box3
    .i_box_04         ( {final_tracklist5[7:0],final_tracklist5[23:16],final_tracklist5[15:8],final_tracklist5[31:24]} ),//w_box4
     `else
    .i_cls_bmap       ( w_cls_bmap1		 ),
    .i_box_vld        ( {5'b0, w_box_vld}),
    .count_list	      ( 50'b0			 ),
    .i_box_00         ( w_box0           ), // y2, y1, x2, x1
    .i_box_01         ( w_box1           ),
    .i_box_02         ( w_box2           ),
    .i_box_03         ( w_box3           ),
    .i_box_04         ( w_box4           ),
    `endif
    .i_box_05         ( 32'b0            ),
    .i_box_06         ( 32'b0            ),
    .i_box_07         ( 32'b0            ),
    .i_box_08         ( 32'b0            ),
    .i_box_09         ( 32'b0            ),
    .bcd_frame_count  ( bcd_frame_count  ),
    .bcd_class_obj_0  ( bcd_class_obj_0  ),
    .bcd_x1_0         ( bcd_x1_0         ),
    .bcd_y1_0         ( bcd_y1_0         ),
    .bcd_class_obj_1  ( bcd_class_obj_1  ),
    .bcd_x1_1         ( bcd_x1_1         ),
    .bcd_y1_1         ( bcd_y1_1         ),
    .ind_inf_time_hex ( ind_inf_time_hex ),
    .avg_inf_time_hex ( avg_inf_time_hex ),
    .inf_time_ms      ( inf_time_ms      ),
    .inf_time_ind_ms  ( inf_time_ind_ms  ),
    .fps_val 	      ( fps_val          ),
    .i_hs	          ( rx2_rgb_hs       ),
    .i_vs	          ( rx2_rgb_vs       ),
    .i_de	          ( rx2_rgb_de       ),
    .i_hs_d2	      ( w_hs_d2	         ),
    .i_vs_d2	      ( w_vs_d2	         ),
    .i_de_d2	      ( w_de_d2	         ),
    .i_r_d2	          ( w_pix_r_d2	     ),
    .i_g_d2	          ( w_pix_g_d2	     ),
    .i_b_d2	          ( w_pix_b_d2	     ),
    .o_tx_serdata     ( o_txd),          
    .o_hs	          ( o_hs		     ),
    .o_vs	          ( o_vs		     ),
    .o_de	          ( o_de		     ),
    .o_r	          ( o_pix_r	         ),
    .o_g	          ( o_pix_g	         ),
    .o_b	          ( o_pix_b	         )
);

//====================================
//             Video Data Output
//====================================

always @ (posedge clk_24_in or negedge resetn) 
begin
	if(~resetn) begin
		pix_ycbcr <= 24'h108080;
		fv        <= 0;
		lv        <= 0;
	end
	else begin
		`ifdef JEDI_ON
		pix_ycbcr <= {o_pix_r,o_pix_g,o_pix_b};
		fv        <= o_vs;
		lv        <= o_de;
		`else
		pix_ycbcr <= {8'h7f,8'h7f,rx2_rgb_pd[7:0]};
		fv        <= rx2_rgb_vs;
		lv        <= rx2_rgb_de;
		`endif
	end
end


//====================================
//        USB WRAPPER
//====================================

WrapperUSB3
uWrapperUSB3
(
  .rst_ni     ( resetn    ),
  .ClkUSB_i   ( clk_24_in ),
  .Fv_i       ( fv        ),
  .Lv_i       ( lv        ),
  .DatYCbCr_i ( pix_ycbcr ),
  .Fv_o       ( fv_w      ),
  .Lv_o       ( lv_w      ),
  .Data_o     ( data_w    )
);

//====================================
//       VIDEO OUTPUT
//====================================

assign pmod1_o       = 1'b1;
//assign reset_fx3_o = push_button0_i;

OBZ
OBZ_pmod2_o (
  .I (pmod1_o),   
  .T (1'b1),  
  .O (pmod2_o)   
);

OBZ
OBZ_pmod0_o (
  .I (pmod0_ctrl_sw0_i),   
  .T (~pmod0_ctrl_sw0_i),  
  .O (pmod0_o)   
);

OFD1P3DX u_oddrx1f_fv_o  (
    .D   (fv_w ),
    .SP  (1'b1 ),
    .CK  (clk_24_in ),
    .CD  (~resetn ),
    .Q   (fv_o)    
);

OFD1P3DX u_oddrx1f_lv_o  (
    .D   (lv_w ),
    .SP  (1'b1 ),
    .CK  (clk_24_in ),
    .CD  (~resetn ),
    .Q   (lv_o)    
);

OFD1P3DX u_oddrx1f_data_o[15:0]  (
    .D   (data_w ),
    .SP  (1'b1  ),
    .CK  (clk_24_in ),
    .CD  (~resetn ),
    .Q   (data_o)    
);

ODDRX1 
uPClk 
( .D0   (1'b0),
  .D1   (1'b1),
  .SCLK (clk_24_in),
  .RST  (~resetn),
  .Q    (clk_24_out)
);

// Output Side of the Video Path }}}

//----------------------------------------------------------------------------------------------------

//LED Toggling }}}
always @ (posedge clk_24_in or negedge resetn)
begin
	if(!resetn)
	begin
		counter_val <= 'd0;
		led_val <= 1'b0;
	end
	else
	begin
		counter_val <= counter_val + 1'b1;
		if (counter_val==16'h1000)
			led_val <= 1'b1;
		else if (counter_val==16'hffff)
			led_val <= 1'b0;
		else
			led_val <= led_val;
	end
end
//LED Toggling }}}

// LED Display 
assign led0_o      = pmod0_ctrl_sw0_i; 
assign led1_o      = ~load_done; 
assign led2_o      = led_val; 
assign led3_o	   = done_o;
assign led4_o	   = w_rd_rdy; 
assign led5_o	   = load_done;  
assign led6_o	   = ~o_objdet; 
assign led7_o	   = rx2_xsdn_o; 
assign led8_o	   = int_rst_n; 
assign led9_o	   = 1'b1;     


//To add signals to reveal debugger 
`ifdef DEBUG

assign object1_cls = w_cls_bmap1[4:0];
assign object2_cls = w_cls_bmap1[9:5];
assign object3_cls = w_cls_bmap1[14:10];
assign object4_cls = w_cls_bmap1[19:15];
assign object5_cls = w_cls_bmap1[24:20];

assign object1_bmap = w_box_vld[0];
assign object2_bmap = w_box_vld[1];
assign object3_bmap = w_box_vld[2];
assign object4_bmap = w_box_vld[3];
assign object5_bmap = w_box_vld[4];

assign object1_obmap = w_bbox_bmap[0];
assign object2_obmap = w_bbox_bmap[1];
assign object3_obmap = w_bbox_bmap[2];
assign object4_obmap = w_bbox_bmap[3];
assign object5_obmap = w_bbox_bmap[5];

assign object1_w = w_box0[31:24];
assign object1_h = w_box0[23:16];
assign object1_x = w_box0[15:8];
assign object1_y = w_box0[7:0];

assign object2_w = w_box1[31:24];
assign object2_h = w_box1[23:16];
assign object2_x = w_box1[15:8];
assign object2_y = w_box1[7:0];

assign object3_w = w_box2[31:24];
assign object3_h = w_box2[23:16];
assign object3_x = w_box2[15:8];
assign object3_y = w_box2[7:0];

assign object4_w = w_box3[31:24];
assign object4_h = w_box3[23:16];
assign object4_x = w_box3[15:8];
assign object4_y = w_box3[7:0];

assign object5_w = w_box4[31:24];
assign object5_h = w_box4[23:16];
assign object5_x = w_box4[15:8];
assign object5_y = w_box4[7:0];

//To send the box_coordinates,box_class,box_valid data to uart taken from jedi_object_count_top module to UART_ENABLER section in the osd_back_224x224_object_count module. 
always@(posedge clk or negedge resetn)
begin
	if(!resetn) begin
	   bcd_x1_0 <= 12'd0;
	   bcd_x2_0 <= 12'd0;
	   bcd_y1_0 <= 12'd0;
	   bcd_y2_0 <= 12'd0;
	   bcd_x1_1 <= 12'd0;
	   bcd_x2_1 <= 12'd0;
	   bcd_y1_1 <= 12'd0;
	   bcd_y2_1 <= 12'd0;
	   bcd_x1_2 <= 12'd0;
	   bcd_x2_2 <= 12'd0;
	   bcd_y1_2 <= 12'd0;
	   bcd_y2_2 <= 12'd0;
	   bcd_x1_3 <= 12'd0;
	   bcd_x2_3 <= 12'd0;
	   bcd_y1_3 <= 12'd0;
	   bcd_y2_3 <= 12'd0;
	   bcd_x1_4 <= 12'd0;
	   bcd_x2_4 <= 12'd0;
	   bcd_y1_4 <= 12'd0;
	   bcd_y2_4 <= 12'd0;
	   bcd_class_obj_0 <= 12'd0;
	   bcd_class_obj_1 <= 12'd0;
	   bcd_class_obj_2 <= 12'd0;
	   bcd_class_obj_3 <= 12'd0;
	   bcd_class_obj_4 <= 12'd0;
	   bcd_i_box_vld   <= 12'd0;
	   bcd_frame_count <= 16'd0;
	end
	else begin
	  bcd_frame_count <= bcd_12bits(final_frame_count);
	  bcd_x1_0 <= bcd((w_box_vld[0]) ? w_box0[7:0] : 8'd0);
	  bcd_x2_0 <= bcd((w_box_vld[0]) ? w_box0[15:8]  : 8'd0); 
	  bcd_y1_0 <= bcd((w_box_vld[0]) ? w_box0[23:16] : 8'd0);
	  bcd_y2_0 <= bcd((w_box_vld[0]) ? w_box0[31:24] : 8'd0);	
	  bcd_x1_1 <= bcd((w_box_vld[1]) ? w_box1[7:0]   : 8'd0);
	  bcd_x2_1 <= bcd((w_box_vld[1]) ? w_box1[15:8]  : 8'd0); 
	  bcd_y1_1 <= bcd((w_box_vld[1]) ? w_box1[23:16] : 8'd0);
	  bcd_y2_1 <= bcd((w_box_vld[1]) ? w_box1[31:24] : 8'd0);	
	  bcd_x1_2 <= bcd((w_box_vld[2]) ? w_box2[7:0]   : 8'd0);
	  bcd_x2_2 <= bcd((w_box_vld[2]) ? w_box2[15:8]  : 8'd0); 
	  bcd_y1_2 <= bcd((w_box_vld[2]) ? w_box2[23:16] : 8'd0);
	  bcd_y2_2 <= bcd((w_box_vld[2]) ? w_box2[31:24] : 8'd0);	
	  bcd_x1_3 <= bcd((w_box_vld[3]) ? w_box3[7:0]   : 8'd0);
	  bcd_x2_3 <= bcd((w_box_vld[3]) ? w_box3[15:8]  : 8'd0); 
	  bcd_y1_3 <= bcd((w_box_vld[3]) ? w_box3[23:16] : 8'd0);
	  bcd_y2_3 <= bcd((w_box_vld[3]) ? w_box3[31:24] : 8'd0);	
	  bcd_x1_4 <= bcd((w_box_vld[4]) ? w_box4[7:0]   : 8'd0);
	  bcd_x2_4 <= bcd((w_box_vld[4]) ? w_box4[15:8]  : 8'd0); 
	  bcd_y1_4 <= bcd((w_box_vld[4]) ? w_box4[23:16] : 8'd0);
	  bcd_y2_4 <= bcd((w_box_vld[4]) ? w_box4[31:24] : 8'd0);
	  bcd_class_obj_0 <= bcd((w_box_vld[0])?{3'd0,w_cls_bmap1[4:0]}:8'd0);
	  bcd_class_obj_1 <= bcd((w_box_vld[1])?{3'd0,w_cls_bmap1[9:5]}:8'd0);
	  bcd_class_obj_2 <= bcd((w_box_vld[2])?{3'd0,w_cls_bmap1[14:10]}:8'd0);
	  bcd_class_obj_3 <= bcd((w_box_vld[3])?{3'd0,w_cls_bmap1[19:15]}:8'd0);
	  bcd_class_obj_4 <= bcd((w_box_vld[4])?{3'd0,w_cls_bmap1[24:20]}:8'd0);
	  bcd_i_box_vld   <= bcd({3'b0,w_box_vld});
	end
end

function [11:0]bcd;

  input [7:0] decimal;
  reg [3:0] h;
  reg [3:0] t;
  reg [3:0] o;
      
  integer i;    
     
     begin
     h = 4'd0;
     t = 4'd0;
     o = 4'd0;
     
     for(i=7;i>=0;i=i-1)
     begin  
     
     if(h>=5)
      h = h+3;
      if(t>=5)
      t = t +3;
      if(o>=5)
      o = o+3;
      
      h = h<< 1;
      h[0] = t[3];
      t = t << 1;
      t[0] = o[3];
      o = o << 1;
      o[0] = decimal[i];
     end
    
      bcd = {h,t,o};
  end
endfunction

function [15:0]bcd_12bits;

  input [11:0] decimal;
  reg [3:0] th;
  reg [3:0] h;
  reg [3:0] t;
  reg [3:0] o;
      
  integer i;    
     
     begin
     th = 4'd0;
     h = 4'd0;
     t = 4'd0;
     o = 4'd0;
     
     for(i=11;i>=0;i=i-1)
     begin  
     
     if(th>=5)
  	  th = th+3;
     if(h>=5)
      h = h+3;
      if(t>=5)
      t = t +3;
      if(o>=5)
      o = o+3;
      
  	th = th << 1;
      th[0] = h[3];
      h = h << 1;
      h[0] = t[3];
      t = t << 1;
      t[0] = o[3];
      o = o << 1;
      o[0] = decimal[i];
     end
    
      bcd_12bits = {th,h,t,o};
  end
endfunction
`endif

endmodule

