`timescale 1ns / 1ps

module global_avgpool_fc #(
    parameter DW         = 8,
    parameter ACC_W      = 32,
    parameter H          = 7,
    parameter FW         = 7,
    parameter C          = 1024,
    parameter NUM_CLASSES = 1000,
    parameter MAC_SEL    = 2
)(
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              start,

    input  wire [C*H*FW*DW-1:0]              act_in_flat,
    input  wire [NUM_CLASSES*C*DW-1:0]       fc_weight_flat,   // w[class][channel]

    output reg  [NUM_CLASSES*ACC_W-1:0]      class_scores_flat, // raw (unclamped) logits
    output reg                               done,

    output reg [31:0]                        tot_toggle_count,
    output reg [31:0]                        tot_adder_invocations,
    output reg [31:0]                        tot_addition_operations
);

localparam integer NPIX = H*FW;
localparam IDX_W = 16;

// ---------------------------------------------------------------------
// Stage 1: global average pool -> pooled_flat [C*DW-1:0]
// ---------------------------------------------------------------------
reg  [C*DW-1:0] pooled_flat;
reg             pool_done;
integer pc, pp;
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
integer cls;
reg                    eng_start;
wire [IDX_W-1:0]       eng_idx;
wire [DW-1:0]          eng_x, eng_w;
wire [ACC_W-1:0]       eng_result;
wire                   eng_result_valid;
wire [31:0]            eng_tog, eng_adi, eng_ado;

function [DW-1:0] get_w;
    input integer classc, cc;
    begin
        get_w = fc_weight_flat[(classc*C+cc)*DW +: DW];
    end
endfunction

assign eng_x = pooled_flat[eng_idx*DW +: DW];
assign eng_w = get_w(cls, eng_idx);

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
