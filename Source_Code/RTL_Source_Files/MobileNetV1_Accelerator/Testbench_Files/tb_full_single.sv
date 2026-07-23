`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////
// tb_full_single.sv
//
// Single-DUT smoke test of the FULL 13-block MobileNetV1 topology
// (mobilenetv1_full_top), with channel/spatial parameters reduced purely
// for tractable simulation time. Weights are loaded through the unified
// w_wr_en/w_wr_addr/w_wr_data BRAM-loading port instead of flat buses.
//////////////////////////////////////////////////////////////////////////////

module tb_full_single;
localparam DW=8, ACC_W=32;
localparam H0=8, FW0=8, CIN0=3;
localparam C_STEM=4, C1=4,C2=8,C3=8,C4=8,C5=8,C6=8,C7=8,C8=8,C9=8,C10=8,C11=8,C12=8,C13=8;
localparam NUM_CLASSES=4;

// mirrors mobilenetv1_full_top's internal weight address-map computation
localparam integer STEM_DEPTH = C_STEM*CIN0*9;
localparam integer B1_DEPTH   = C_STEM*9 + C1*C_STEM;
localparam integer B2_DEPTH   = C1*9     + C2*C1;
localparam integer B3_DEPTH   = C2*9     + C3*C2;
localparam integer B4_DEPTH   = C3*9     + C4*C3;
localparam integer B5_DEPTH   = C4*9     + C5*C4;
localparam integer B6_DEPTH   = C5*9     + C6*C5;
localparam integer B7_DEPTH   = C6*9     + C7*C6;
localparam integer B8_DEPTH   = C7*9     + C8*C7;
localparam integer B9_DEPTH   = C8*9     + C9*C8;
localparam integer B10_DEPTH  = C9*9     + C10*C9;
localparam integer B11_DEPTH  = C10*9    + C11*C10;
localparam integer B12_DEPTH  = C11*9    + C12*C11;
localparam integer B13_DEPTH  = C12*9    + C13*C12;
localparam integer FC_DEPTH   = NUM_CLASSES*C13;
localparam integer TOTAL_DEPTH = STEM_DEPTH+B1_DEPTH+B2_DEPTH+B3_DEPTH+B4_DEPTH+B5_DEPTH+
                                  B6_DEPTH+B7_DEPTH+B8_DEPTH+B9_DEPTH+B10_DEPTH+B11_DEPTH+
                                  B12_DEPTH+B13_DEPTH+FC_DEPTH;
localparam WADDR_W = (TOTAL_DEPTH > 1) ? $clog2(TOTAL_DEPTH) : 1;

reg clk=0, rst=1, start=0;

localparam IMG_BITS = CIN0*H0*FW0*DW;
reg [IMG_BITS-1:0] img_in=0;

reg               w_wr_en=0;
reg [WADDR_W-1:0] w_wr_addr=0;
reg [DW-1:0]      w_wr_data=0;

wire [NUM_CLASSES*ACC_W-1:0] scores;
wire done;
wire [31:0] tog,adi,ado;

mobilenetv1_full_top #(
  .DW(DW), .ACC_W(ACC_W), .H0(H0), .FW0(FW0), .CIN0(CIN0),
  .C_STEM(C_STEM), .C1(C1), .C2(C2), .C3(C3), .C4(C4), .C5(C5), .C6(C6),
  .C7(C7), .C8(C8), .C9(C9), .C10(C10), .C11(C11), .C12(C12), .C13(C13),
  .NUM_CLASSES(NUM_CLASSES), .MAC_SEL(2)
) dut (
  .clk(clk), .rst(rst), .start(start),
  .img_in_flat(img_in),
  .w_wr_en(w_wr_en), .w_wr_addr(w_wr_addr), .w_wr_data(w_wr_data),
  .class_scores_flat(scores), .done(done),
  .tot_toggle_count(tog), .tot_adder_invocations(adi), .tot_addition_operations(ado)
);

always #5 clk=~clk;
integer i;

initial begin
  repeat(4) @(posedge clk); #1; rst=0;

  for(i=0;i<IMG_BITS/DW;i=i+1) img_in[i*DW+:DW]=$urandom_range(0,15);

  w_wr_en = 1;
  for (i = 0; i < TOTAL_DEPTH; i = i + 1) begin
    w_wr_addr = i[WADDR_W-1:0];
    w_wr_data = $urandom_range(0, 3);
    @(posedge clk);
  end
  w_wr_en = 0;

  @(posedge clk); #1; start=1;
  @(posedge clk); #1; start=0;

  wait(done);
  $display("DONE at t=%0t tog=%0d adi=%0d ado=%0d", $time, tog, adi, ado);
  $display("class_scores: %h", scores);
  $finish;
end
endmodule
