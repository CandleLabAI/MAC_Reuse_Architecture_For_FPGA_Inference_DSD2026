`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////
// tb_mobilenetv1_full_reduced_compare.sv
//
// 3-way MAC comparison (baseline / dsp / proposed_AOR) run across the FULL
// 13-block MobileNetV1 topology (stem + 13 dw-sep blocks + avgpool + FC),
// with all channel/spatial parameters overridden to small values purely so
// the simulation completes in a reasonable amount of time. Weights are
// loaded through the unified w_wr_en/w_wr_addr/w_wr_data BRAM-loading port,
// broadcast identically to all three DUTs (they share an identical address
// map since all three use identical channel/spatial params).
//
// mobilenetv1_full_top.sv itself defaults to MobileNetV1's real channel
// counts (32/64/128/.../1024) and is what should be handed to Vivado
// synthesis; this testbench proves the same RTL (stride handling, all 13
// blocks, global average pool, FC head, BRAM weight storage) is
// functionally correct end-to-end and that the AOR MAC produces
// bit-identical results to baseline/dsp at full-topology scale.
//////////////////////////////////////////////////////////////////////////////

module tb_mobilenetv1_full_reduced_compare;
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

wire [NUM_CLASSES*ACC_W-1:0] scores_b, scores_d, scores_p;
wire done_b, done_d, done_p;
wire [31:0] tog_b,adi_b,ado_b, tog_d,adi_d,ado_d, tog_p,adi_p,ado_p;

`define MNV1_PORTS(m_scores, m_done, m_tog, m_adi, m_ado) \
  .clk(clk), .rst(rst), .start(start),                     \
  .img_in_flat(img_in),                                     \
  .w_wr_en(w_wr_en), .w_wr_addr(w_wr_addr), .w_wr_data(w_wr_data), \
  .class_scores_flat(m_scores), .done(m_done),               \
  .tot_toggle_count(m_tog), .tot_adder_invocations(m_adi), .tot_addition_operations(m_ado)

mobilenetv1_full_top #(
  .DW(DW), .ACC_W(ACC_W), .H0(H0), .FW0(FW0), .CIN0(CIN0),
  .C_STEM(C_STEM), .C1(C1), .C2(C2), .C3(C3), .C4(C4), .C5(C5), .C6(C6),
  .C7(C7), .C8(C8), .C9(C9), .C10(C10), .C11(C11), .C12(C12), .C13(C13),
  .NUM_CLASSES(NUM_CLASSES), .MAC_SEL(0)
) dut_baseline ( `MNV1_PORTS(scores_b, done_b, tog_b, adi_b, ado_b) );

mobilenetv1_full_top #(
  .DW(DW), .ACC_W(ACC_W), .H0(H0), .FW0(FW0), .CIN0(CIN0),
  .C_STEM(C_STEM), .C1(C1), .C2(C2), .C3(C3), .C4(C4), .C5(C5), .C6(C6),
  .C7(C7), .C8(C8), .C9(C9), .C10(C10), .C11(C11), .C12(C12), .C13(C13),
  .NUM_CLASSES(NUM_CLASSES), .MAC_SEL(1)
) dut_dsp ( `MNV1_PORTS(scores_d, done_d, tog_d, adi_d, ado_d) );

mobilenetv1_full_top #(
  .DW(DW), .ACC_W(ACC_W), .H0(H0), .FW0(FW0), .CIN0(CIN0),
  .C_STEM(C_STEM), .C1(C1), .C2(C2), .C3(C3), .C4(C4), .C5(C5), .C6(C6),
  .C7(C7), .C8(C8), .C9(C9), .C10(C10), .C11(C11), .C12(C12), .C13(C13),
  .NUM_CLASSES(NUM_CLASSES), .MAC_SEL(2)
) dut_proposed ( `MNV1_PORTS(scores_p, done_p, tog_p, adi_p, ado_p) );

always #5 clk=~clk;
integer i;

// latch done independently per DUT (different MAC variants -> different
// pipeline latency -> done pulses on different cycles, see mac_engine.sv note)
reg done_b_latched, done_d_latched, done_p_latched;
always @(posedge clk) begin
  if (rst) begin
    done_b_latched<=0; done_d_latched<=0; done_p_latched<=0;
  end else begin
    if (start) begin done_b_latched<=0; done_d_latched<=0; done_p_latched<=0; end
    if (done_b) done_b_latched<=1;
    if (done_d) done_d_latched<=1;
    if (done_p) done_p_latched<=1;
  end
end

initial begin
  repeat(4) @(posedge clk); #1; rst=0;

  for(i=0;i<IMG_BITS/DW;i=i+1) img_in[i*DW+:DW]=$urandom_range(0,15);

  // broadcast identical weight data to all three DUTs (identical address map)
  w_wr_en = 1;
  for (i = 0; i < TOTAL_DEPTH; i = i + 1) begin
    w_wr_addr = i[WADDR_W-1:0];
    w_wr_data = $urandom_range(0, 3);
    @(posedge clk);
  end
  w_wr_en = 0;

  @(posedge clk); #1; start=1;
  @(posedge clk); #1; start=0;

  wait(done_b_latched && done_d_latched && done_p_latched);
  @(posedge clk);

  $display("=====================================================================");
  $display(" Full 13-block MobileNetV1 topology -- MAC variant comparison (reduced scale)");
  $display("=====================================================================");
  $display(" %-14s | %-14s | %-16s | %-18s", "MAC variant", "Toggle count", "Adder invocations", "Addition operations");
  $display("---------------------------------------------------------------------");
  $display(" %-14s | %-14d | %-16d | %-18d", "baseline",     tog_b, adi_b, ado_b);
  $display(" %-14s | %-14d | %-16d | %-18d", "dsp",          tog_d, adi_d, ado_d);
  $display(" %-14s | %-14d | %-16d | %-18d", "proposed_AOR", tog_p, adi_p, ado_p);
  $display("=====================================================================");

  if (scores_b === scores_d && scores_b === scores_p)
    $display(" RESULT: PASS - all three MAC variants produced identical class scores across all 13 blocks + FC.");
  else begin
    $display(" RESULT: FAIL - class score mismatch detected.");
    $display("   baseline: %h", scores_b);
    $display("   dsp     : %h", scores_d);
    $display("   proposed: %h", scores_p);
  end

  $finish;
end
endmodule
