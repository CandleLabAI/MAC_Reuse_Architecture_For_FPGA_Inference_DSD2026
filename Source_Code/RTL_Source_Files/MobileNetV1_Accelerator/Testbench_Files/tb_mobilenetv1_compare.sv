`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_mobilenetv1_compare.sv
//
// Instantiates the SAME MobileNetV1-style accelerator three times, once per
// MAC variant (MAC_SEL = 0 baseline, 1 dsp, 2 proposed_AOR), loads all
// three's weight_bram-backed weight memories through the unified
// w_wr_en/w_wr_addr/w_wr_data port with identical randomized data, drives
// identical randomized image data, and:
//   1) checks that all three produce bit-identical outputs (correctness
//      check -- the AOR optimization must not change the numerical result)
//   2) reports total cycles-to-done, toggle_count, adder_invocations, and
//      addition_operations for each variant side by side.
//////////////////////////////////////////////////////////////////////////////

module tb_mobilenetv1_compare;

localparam DW    = 8;
localparam ACC_W = 32;
localparam H     = 4;
localparam FW    = 4;
localparam CIN0  = 3;
localparam C0    = 4;
localparam C1    = 8;
localparam C2    = 16;

// mirrors mobilenetv1_top's internal weight address-map computation
localparam integer STEM_DEPTH  = C0*CIN0*9;
localparam integer B1_DEPTH    = C0*9 + C1*C0;
localparam integer B2_DEPTH    = C1*9 + C2*C1;
localparam integer TOTAL_DEPTH = STEM_DEPTH + B1_DEPTH + B2_DEPTH;
localparam WADDR_W = (TOTAL_DEPTH > 1) ? $clog2(TOTAL_DEPTH) : 1;

localparam IMG_BITS = CIN0*H*FW*DW;
localparam OUT_BITS = C2*H*FW*DW;

reg clk, rst, start;

reg [IMG_BITS-1:0] img_in;

reg                  w_wr_en;
reg [WADDR_W-1:0]    w_wr_addr;
reg [DW-1:0]         w_wr_data;

wire [OUT_BITS-1:0] out_baseline, out_dsp, out_proposed;
wire done_baseline, done_dsp, done_proposed;
wire [31:0] tog_b, adi_b, ado_b;
wire [31:0] tog_d, adi_d, ado_d;
wire [31:0] tog_p, adi_p, ado_p;

integer cyc_baseline, cyc_dsp, cyc_proposed;
reg run_b, run_d, run_p;

// same w_wr_en/w_wr_addr/w_wr_data broadcast to all three -- they share an
// identical address map since all three use identical channel/spatial params
mobilenetv1_top #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW),
                   .CIN0(CIN0), .C0(C0), .C1(C1), .C2(C2), .MAC_SEL(0)) dut_baseline (
    .clk(clk), .rst(rst), .start(start),
    .img_in_flat(img_in),
    .w_wr_en(w_wr_en), .w_wr_addr(w_wr_addr), .w_wr_data(w_wr_data),
    .out_flat(out_baseline), .done(done_baseline),
    .tot_toggle_count(tog_b), .tot_adder_invocations(adi_b), .tot_addition_operations(ado_b)
);

mobilenetv1_top #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW),
                   .CIN0(CIN0), .C0(C0), .C1(C1), .C2(C2), .MAC_SEL(1)) dut_dsp (
    .clk(clk), .rst(rst), .start(start),
    .img_in_flat(img_in),
    .w_wr_en(w_wr_en), .w_wr_addr(w_wr_addr), .w_wr_data(w_wr_data),
    .out_flat(out_dsp), .done(done_dsp),
    .tot_toggle_count(tog_d), .tot_adder_invocations(adi_d), .tot_addition_operations(ado_d)
);

mobilenetv1_top #(.DW(DW), .ACC_W(ACC_W), .H(H), .FW(FW),
                   .CIN0(CIN0), .C0(C0), .C1(C1), .C2(C2), .MAC_SEL(2)) dut_proposed (
    .clk(clk), .rst(rst), .start(start),
    .img_in_flat(img_in),
    .w_wr_en(w_wr_en), .w_wr_addr(w_wr_addr), .w_wr_data(w_wr_data),
    .out_flat(out_proposed), .done(done_proposed),
    .tot_toggle_count(tog_p), .tot_adder_invocations(adi_p), .tot_addition_operations(ado_p)
);

// clock
always #5 clk = ~clk;

// per-DUT cycle counters (active from start until done)
always @(posedge clk) begin
    if (rst) begin
        run_b <= 0; run_d <= 0; run_p <= 0;
        cyc_baseline <= 0; cyc_dsp <= 0; cyc_proposed <= 0;
    end else begin
        if (start) begin run_b <= 1; run_d <= 1; run_p <= 1; cyc_baseline <= 0; cyc_dsp <= 0; cyc_proposed <= 0; end
        if (run_b && !done_baseline) cyc_baseline <= cyc_baseline + 1;
        if (run_d && !done_dsp)      cyc_dsp      <= cyc_dsp      + 1;
        if (run_p && !done_proposed) cyc_proposed <= cyc_proposed + 1;
        if (done_baseline) run_b <= 0;
        if (done_dsp)      run_d <= 0;
        if (done_proposed) run_p <= 0;
    end
end

integer i;

// latch each DUT's done independently -- baseline/dsp/proposed MAC variants
// can have different internal pipeline latency, so their single-cycle done
// pulses are not guaranteed to land on the same cycle. A combinational
// wait(doneA && doneB && doneC) would race against that; latching each one
// makes the check level-stable and race-free.
reg done_baseline_latched, done_dsp_latched, done_proposed_latched;
always @(posedge clk) begin
    if (rst) begin
        done_baseline_latched <= 0;
        done_dsp_latched      <= 0;
        done_proposed_latched <= 0;
    end else begin
        if (start) begin
            done_baseline_latched <= 0;
            done_dsp_latched      <= 0;
            done_proposed_latched <= 0;
        end
        if (done_baseline) done_baseline_latched <= 1;
        if (done_dsp)      done_dsp_latched      <= 1;
        if (done_proposed) done_proposed_latched <= 1;
    end
end

task automatic randomize_image;
    begin
        for (i = 0; i < IMG_BITS/DW; i = i + 1)
            img_in[i*DW +: DW] = $urandom_range(0, 63);
    end
endtask

// load every weight_bram in all three DUTs (same address/data broadcast to
// all three since they share an identical address map)
task automatic load_all_weights;
    begin
        w_wr_en = 1;
        for (i = 0; i < TOTAL_DEPTH; i = i + 1) begin
            w_wr_addr = i[WADDR_W-1:0];
            w_wr_data = $urandom_range(0, 15);
            @(posedge clk);
        end
        w_wr_en = 0;
        w_wr_addr = 0;
        w_wr_data = 0;
    end
endtask

integer mismatches;

initial begin
    clk = 0; rst = 1; start = 0;
    img_in = 0; w_wr_en = 0; w_wr_addr = 0; w_wr_data = 0;
    mismatches = 0;

    repeat (4) @(posedge clk);
    rst = 0;
    repeat (2) @(posedge clk);

    randomize_image();
    load_all_weights();

    @(posedge clk);
    start = 1;
    @(posedge clk);
    start = 0;

    wait (done_baseline_latched && done_dsp_latched && done_proposed_latched);
    @(posedge clk);

    if (out_baseline !== out_dsp) begin
        mismatches = mismatches + 1;
        $display("MISMATCH: baseline vs dsp outputs differ!");
    end
    if (out_baseline !== out_proposed) begin
        mismatches = mismatches + 1;
        $display("MISMATCH: baseline vs proposed_AOR outputs differ!");
    end

    $display("=====================================================================");
    $display(" MobileNetV1 accelerator MAC comparison  (H=%0d FW=%0d CIN0=%0d C0=%0d C1=%0d C2=%0d)", H, FW, CIN0, C0, C1, C2);
    $display("=====================================================================");
    $display(" %-14s | %-10s | %-14s | %-16s | %-18s", "MAC variant", "Cycles", "Toggle count", "Adder invocations", "Addition operations");
    $display("---------------------------------------------------------------------");
    $display(" %-14s | %-10d | %-14d | %-16d | %-18d", "baseline",     cyc_baseline, tog_b, adi_b, ado_b);
    $display(" %-14s | %-10d | %-14d | %-16d | %-18d", "dsp",          cyc_dsp,      tog_d, adi_d, ado_d);
    $display(" %-14s | %-10d | %-14d | %-16d | %-18d", "proposed_AOR", cyc_proposed, tog_p, adi_p, ado_p);
    $display("=====================================================================");

    if (mismatches == 0)
        $display(" RESULT: PASS - all three MAC variants produced identical outputs.");
    else
        $display(" RESULT: FAIL - %0d output mismatch(es) detected.", mismatches);

    $display(" Toggle-count reduction  (proposed vs baseline): %0.2f%%",
              100.0 * (tog_b - tog_p) / tog_b);
    $display(" Toggle-count reduction  (proposed vs dsp)     : %0.2f%%",
              100.0 * (tog_d - tog_p) / tog_d);

    $finish;
end

endmodule
