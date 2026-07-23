`timescale 1ns/1ps
module tb_full_single;
localparam DW=8, ACC_W=32;
localparam H0=8, FW0=8, CIN0=3;
localparam C_STEM=4, C1=4,C2=8,C3=8,C4=8,C5=8,C6=8,C7=8,C8=8,C9=8,C10=8,C11=8,C12=8,C13=8;
localparam NUM_CLASSES=4;

reg clk=0, rst=1, start=0;

localparam IMG_BITS   = CIN0*H0*FW0*DW;
localparam STEMW_BITS = C_STEM*CIN0*9*DW;

reg [IMG_BITS-1:0] img_in=0;
reg [STEMW_BITS-1:0] stem_w=0;
reg [C_STEM*9*DW-1:0] b1_dw_w=0;  reg [C1*C_STEM*DW-1:0] b1_pw_w=0;
reg [C1*9*DW-1:0]     b2_dw_w=0;  reg [C2*C1*DW-1:0]     b2_pw_w=0;
reg [C2*9*DW-1:0]     b3_dw_w=0;  reg [C3*C2*DW-1:0]     b3_pw_w=0;
reg [C3*9*DW-1:0]     b4_dw_w=0;  reg [C4*C3*DW-1:0]     b4_pw_w=0;
reg [C4*9*DW-1:0]     b5_dw_w=0;  reg [C5*C4*DW-1:0]     b5_pw_w=0;
reg [C5*9*DW-1:0]     b6_dw_w=0;  reg [C6*C5*DW-1:0]     b6_pw_w=0;
reg [C6*9*DW-1:0]     b7_dw_w=0;  reg [C7*C6*DW-1:0]     b7_pw_w=0;
reg [C7*9*DW-1:0]     b8_dw_w=0;  reg [C8*C7*DW-1:0]     b8_pw_w=0;
reg [C8*9*DW-1:0]     b9_dw_w=0;  reg [C9*C8*DW-1:0]     b9_pw_w=0;
reg [C9*9*DW-1:0]     b10_dw_w=0; reg [C10*C9*DW-1:0]    b10_pw_w=0;
reg [C10*9*DW-1:0]    b11_dw_w=0; reg [C11*C10*DW-1:0]   b11_pw_w=0;
reg [C11*9*DW-1:0]    b12_dw_w=0; reg [C12*C11*DW-1:0]   b12_pw_w=0;
reg [C12*9*DW-1:0]    b13_dw_w=0; reg [C13*C12*DW-1:0]   b13_pw_w=0;
reg [NUM_CLASSES*C13*DW-1:0] fc_w=0;

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
  .img_in_flat(img_in), .stem_w_flat(stem_w),
  .b1_dw_w(b1_dw_w), .b1_pw_w(b1_pw_w), .b2_dw_w(b2_dw_w), .b2_pw_w(b2_pw_w),
  .b3_dw_w(b3_dw_w), .b3_pw_w(b3_pw_w), .b4_dw_w(b4_dw_w), .b4_pw_w(b4_pw_w),
  .b5_dw_w(b5_dw_w), .b5_pw_w(b5_pw_w), .b6_dw_w(b6_dw_w), .b6_pw_w(b6_pw_w),
  .b7_dw_w(b7_dw_w), .b7_pw_w(b7_pw_w), .b8_dw_w(b8_dw_w), .b8_pw_w(b8_pw_w),
  .b9_dw_w(b9_dw_w), .b9_pw_w(b9_pw_w), .b10_dw_w(b10_dw_w), .b10_pw_w(b10_pw_w),
  .b11_dw_w(b11_dw_w), .b11_pw_w(b11_pw_w), .b12_dw_w(b12_dw_w), .b12_pw_w(b12_pw_w),
  .b13_dw_w(b13_dw_w), .b13_pw_w(b13_pw_w),
  .fc_w_flat(fc_w),
  .class_scores_flat(scores), .done(done),
  .tot_toggle_count(tog), .tot_adder_invocations(adi), .tot_addition_operations(ado)
);

always #5 clk=~clk;
integer i;

initial begin
  repeat(4) @(posedge clk); #1; rst=0;
  for(i=0;i<IMG_BITS/DW;i=i+1) img_in[i*DW+:DW]=$urandom_range(0,15);
  for(i=0;i<STEMW_BITS/DW;i=i+1) stem_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C_STEM*9);i=i+1) b1_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C1*C_STEM);i=i+1) b1_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C1*9);i=i+1) b2_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C2*C1);i=i+1) b2_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C2*9);i=i+1) b3_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C3*C2);i=i+1) b3_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C3*9);i=i+1) b4_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C4*C3);i=i+1) b4_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C4*9);i=i+1) b5_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C5*C4);i=i+1) b5_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C5*9);i=i+1) b6_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C6*C5);i=i+1) b6_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C6*9);i=i+1) b7_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C7*C6);i=i+1) b7_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C7*9);i=i+1) b8_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C8*C7);i=i+1) b8_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C8*9);i=i+1) b9_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C9*C8);i=i+1) b9_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C9*9);i=i+1) b10_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C10*C9);i=i+1) b10_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C10*9);i=i+1) b11_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C11*C10);i=i+1) b11_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C11*9);i=i+1) b12_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C12*C11);i=i+1) b12_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C12*9);i=i+1) b13_dw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(C13*C12);i=i+1) b13_pw_w[i*DW+:DW]=$urandom_range(0,3);
  for(i=0;i<(NUM_CLASSES*C13);i=i+1) fc_w[i*DW+:DW]=$urandom_range(0,3);

  @(posedge clk); #1; start=1;
  @(posedge clk); #1; start=0;

  wait(done);
  $display("DONE at t=%0t tog=%0d adi=%0d ado=%0d", $time, tog, adi, ado);
  $display("class_scores: %h", scores);
  $finish;
end
endmodule
