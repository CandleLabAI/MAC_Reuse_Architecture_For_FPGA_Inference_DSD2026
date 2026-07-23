`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// global_avgpool_fc.sv
//
// Final two layers of MobileNetV1:
//   1) Global average pool: collapse the HxFW spatial map to one value per
//      channel (mean over all spatial positions).
//   2) Fully-connected classifier: NUM_CLASSES-way dot product over the
//      pooled C-channel vector, driven through mac_engine (same race-free,
//      index-driven MAC core as every conv layer, MAC_SEL selects the same
//      baseline/dsp/proposed_AOR variant).
//
// Average pooling itself is a plain accumulate-and-divide (no MAC needed --
// there's no weight, just an unweighted mean), done with a simple adder;
// only the FC layer's real dot products go through mac_engine.
//////////////////////////////////////////////////////////////////////////////

module global_avgpool_fc #(
    parameter DW         = 8,
    parameter ACC_W      = 32,
    parameter H          = 7,
    parameter FW         = 7,
    parameter C          = 1024,
    parameter NUM_CLASSES = 1000,
    parameter MAC_SEL    = 2,
    parameter WADDR_W    = (NUM_CLASSES*C > 1) ? $clog2(NUM_CLASSES*C) : 1
)(
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              start,

    input  wire [C*H*FW*DW-1:0]              act_in_flat,

    // FC weight loading interface (address = class*C + channel)
    input  wire                              w_wr_en,
    input  wire [WADDR_W-1:0]                w_wr_addr,
    input  wire [DW-1:0]                     w_wr_data,

    output reg  [NUM_CLASSES*ACC_W-1:0]      class_scores_flat, // raw (unclamped) logits
    output reg                               done,

    output reg [31:0]                        tot_toggle_count,
    output reg [31:0]                        tot_adder_invocations,
    output reg [31:0]                        tot_addition_operations
);

localparam integer NPIX = H*FW;
localparam IDX_W = 16;
localparam integer C_W    = (C    > 1) ? $clog2(C)    : 1;
localparam integer NPIX_W = (NPIX > 1) ? $clog2(NPIX) : 1;

// ---------------------------------------------------------------------
// Stage 1: global average pool -> pooled_flat [C*DW-1:0]
// ---------------------------------------------------------------------
reg  [C*DW-1:0] pooled_flat;
reg             pool_done;
reg [C_W-1:0]    pc;
reg [NPIX_W-1:0] pp;
reg [ACC_W-1:0] pool_acc;

localparam PS_IDLE=2'd0, PS_ACCUM=2'd1, PS_STORE=2'd2, PS_NEXT=2'd3;
reg [1:0] pool_state;

function [DW-1:0] get_pix;
    input integer cc, pidx;
    integer yy, xx;
    begin
        yy = pidx / FW;
        xx = pidx % FW;
        get_pix = act_in_flat[((cc*H+yy)*FW+xx)*DW +: DW];
    end
endfunction

always @(posedge clk) begin
    if (rst) begin
        pool_state <= PS_IDLE;
        pool_done  <= 0;
        pc <= 0; pp <= 0; pool_acc <= 0;
    end else begin
        case (pool_state)
            PS_IDLE: begin
                pool_done <= 0;
                if (start) begin
                    pc <= 0; pp <= 0; pool_acc <= 0;
                    pool_state <= PS_ACCUM;
                end
            end
            PS_ACCUM: begin
                pool_acc <= pool_acc + get_pix(pc, pp);
                if (pp < NPIX-1) begin
                    pp <= pp + 1;
                end else begin
                    pool_state <= PS_STORE;
                end
            end
            PS_STORE: begin
                // integer mean (divide by NPIX)
                pooled_flat[pc*DW +: DW] <= (pool_acc / NPIX);
                pool_state <= PS_NEXT;
            end
            PS_NEXT: begin
                if (pc < C-1) begin
                    pc <= pc + 1; pp <= 0; pool_acc <= 0;
                    pool_state <= PS_ACCUM;
                end else begin
                    pool_done <= 1;
                    pool_state <= PS_IDLE;
                end
            end
            default: pool_state <= PS_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------
// Stage 2: FC layer -- NUM_CLASSES separate C-length dot products, driven
// through mac_engine (real MAC arithmetic, MAC_SEL-selected variant)
// ---------------------------------------------------------------------
localparam integer CLS_W = (NUM_CLASSES > 1) ? $clog2(NUM_CLASSES) : 1;
reg [CLS_W-1:0] cls;
reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

// weight BRAM: address = cls*C + eng_idx, read latency 1 cycle (matches
// mac_engine's S_ADDR settle state)
wire [WADDR_W-1:0] w_rd_addr = cls*C + eng_idx;

weight_bram #(.DW(DW), .DEPTH(NUM_CLASSES*C), .AW(WADDR_W)) u_wmem (
    .clk(clk),
    .wr_en(w_wr_en), .wr_addr(w_wr_addr), .wr_data(w_wr_data),
    .rd_addr(w_rd_addr), .rd_data(eng_w)
);

assign eng_x = pooled_flat[eng_idx*DW +: DW];

mac_engine #(.W(DW), .ACC_W(ACC_W), .IDX_W(IDX_W), .MAC_SEL(MAC_SEL)) u_eng (
    .clk(clk), .rst(rst),
    .start(eng_start), .k_count(C[15:0]),
    .idx(eng_idx), .x_in(eng_x), .w_in(eng_w),
    .result(eng_result), .result_valid(eng_result_valid), .busy(),
    .tot_toggle_count(eng_tog),
    .tot_adder_invocations(eng_adi),
    .tot_addition_operations(eng_ado)
);

localparam FS_IDLE=2'd0, FS_RUN=2'd1, FS_STORE=2'd2, FS_NEXT=2'd3;
reg [1:0] fc_state;

always @(posedge clk) begin
    if (rst) begin
        fc_state <= FS_IDLE;
        done <= 0;
        cls <= 0;
        eng_start <= 0;
        tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
    end else begin
        eng_start <= 0;
        case (fc_state)
            FS_IDLE: begin
                done <= 0;
                if (pool_done) begin
                    cls <= 0;
                    tot_toggle_count <= 0; tot_adder_invocations <= 0; tot_addition_operations <= 0;
                    eng_start <= 1;
                    fc_state <= FS_RUN;
                end
            end
            FS_RUN: begin
                if (eng_result_valid) fc_state <= FS_STORE;
            end
            FS_STORE: begin
                class_scores_flat[cls*ACC_W +: ACC_W] <= eng_result;
                tot_toggle_count        <= tot_toggle_count        + eng_tog;
                tot_adder_invocations   <= tot_adder_invocations   + eng_adi;
                tot_addition_operations <= tot_addition_operations + eng_ado;
                fc_state <= FS_NEXT;
            end
            FS_NEXT: begin
                if (cls < NUM_CLASSES-1) begin
                    cls <= cls + 1;
                    eng_start <= 1;
                    fc_state <= FS_RUN;
                end else begin
                    done <= 1;
                    fc_state <= FS_IDLE;
                end
            end
            default: fc_state <= FS_IDLE;
        endcase
    end
end

endmodule
