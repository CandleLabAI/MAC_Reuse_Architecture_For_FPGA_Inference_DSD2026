`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// mac_engine.sv  (index-driven, race-free version)
//
// Generic sequential dot-product engine used as the PE building block for
// every conv layer in the MobileNetV1 accelerator.
//
// Rationale / assumption (per user's chosen option):
//   The provided *_mac_sa modules compute  y = x * SUM(weights)  which is
//   only mathematically equal to a real dot-product SUM(x_i * w_i) when the
//   underlying MAC is invoked with N = 1 (a single weight, so SUM(weights)
//   degenerates to that one weight). This engine drives the chosen MAC
//   variant with N=1, one (x_i, w_i) pair at a time, and accumulates the
//   partial products itself. This guarantees bit-exact, synthesizable,
//   mathematically correct convolution while still letting every MAC
//   invocation exercise (and be instrumented by) the real baseline / dsp /
//   proposed_AOR arithmetic core.
//
// Interface style: the engine owns the loop counter (idx) and drives it out;
// the caller (a conv layer) combinationally supplies x_in/w_in = f(idx) from
// its activation/weight memories. This avoids any cross-module "last
// element" handshake race -- completion is decided purely from the engine's
// own idx == K-1 comparison.
//
// MAC_SEL: 0 = baseline_mac_sa, 1 = dsp_mac_sa, 2 = proposed_mac_sa (AOR)
//////////////////////////////////////////////////////////////////////////////

module mac_engine #(
    parameter W       = 8,   // activation / weight bit width
    parameter ACC_W   = 32,  // running accumulator width
    parameter IDX_W   = 16,  // width of the element-count / index bus
    parameter MAC_SEL = 2    // 0=baseline, 1=dsp, 2=proposed(AOR)
)(
    input  wire                  clk,
    input  wire                  rst,

    input  wire                  start,       // pulse: begin a new K-element dot-product
    input  wire [IDX_W-1:0]      k_count,     // number of elements in this dot-product (>=1)

    output reg  [IDX_W-1:0]      idx,         // current element index (caller addresses memory with this)
    input  wire [W-1:0]          x_in,        // = activation(idx), combinational from caller
    input  wire [W-1:0]          w_in,        // = weight(idx),     combinational from caller

    output reg  [ACC_W-1:0]      result,
    output reg                   result_valid, // pulses one cycle when dot-product done
    output reg                   busy,

    output reg  [31:0]           tot_toggle_count,
    output reg  [31:0]           tot_adder_invocations,
    output reg  [31:0]           tot_addition_operations
);

localparam ATOMIC_Y_W = 2*W+9;

// ---- atomic MAC (N = 1) interface ----
reg                    atomic_start;
reg  [W-1:0]           atomic_x;
reg  [W-1:0]           atomic_wflat;

wire [ATOMIC_Y_W-1:0]  atomic_y;
wire                   atomic_done;
wire [31:0]            atomic_toggle;
wire [31:0]            atomic_adder_inv;
wire [31:0]            atomic_add_ops;

generate
    if (MAC_SEL == 0) begin : g_baseline
        baseline_mac_sa #(.N(1), .W(W)) u_mac (
            .clk(clk), .rst(rst), .start(atomic_start),
            .x(atomic_x), .weights_flat(atomic_wflat),
            .y(atomic_y), .done(atomic_done),
            .toggle_count(atomic_toggle),
            .adder_invocations(atomic_adder_inv),
            .addition_operations(atomic_add_ops)
        );
    end else if (MAC_SEL == 1) begin : g_dsp
        dsp_mac_sa #(.N(1), .W(W)) u_mac (
            .clk(clk), .rst(rst), .start(atomic_start),
            .x(atomic_x), .weights_flat(atomic_wflat),
            .y(atomic_y), .done(atomic_done),
            .toggle_count(atomic_toggle),
            .adder_invocations(atomic_adder_inv),
            .addition_operations(atomic_add_ops)
        );
    end else begin : g_proposed
        proposed_mac_sa #(.N(1), .W(W)) u_mac (
            .clk(clk), .rst(rst), .start(atomic_start),
            .x(atomic_x), .weights_flat(atomic_wflat),
            .y(atomic_y), .done(atomic_done),
            .toggle_count(atomic_toggle),
            .adder_invocations(atomic_adder_inv),
            .addition_operations(atomic_add_ops)
        );
    end
endgenerate

// ---- controller FSM ----
// S_ADDR exists purely to give memories with registered (1-cycle-latency)
// reads -- i.e. weight_bram -- one cycle to settle after idx changes,
// before S_ISSUE consumes x_in/w_in. Harmless extra cycle for callers whose
// x_in/w_in are still fully combinational.
localparam S_IDLE   = 2'd0,
           S_ADDR   = 2'd1,
           S_ISSUE  = 2'd2,
           S_WAIT   = 2'd3;

reg [1:0] state;
reg [IDX_W-1:0] k_reg;
reg just_issued; // guards against sampling stale atomic_done/atomic_y the cycle right after atomic_start

always @(posedge clk) begin
    if (rst) begin
        state                    <= S_IDLE;
        result                   <= 0;
        result_valid             <= 0;
        busy                     <= 0;
        idx                      <= 0;
        k_reg                    <= 0;
        tot_toggle_count         <= 0;
        tot_adder_invocations    <= 0;
        tot_addition_operations  <= 0;
        atomic_start             <= 0;
        atomic_x                 <= 0;
        atomic_wflat             <= 0;
        just_issued              <= 0;
    end else begin
        atomic_start  <= 0;
        result_valid  <= 0;

        case (state)
            S_IDLE: begin
                busy <= 0;
                if (start) begin
                    result                  <= 0;
                    idx                     <= 0;
                    k_reg                   <= k_count;
                    tot_toggle_count        <= 0;
                    tot_adder_invocations   <= 0;
                    tot_addition_operations <= 0;
                    busy                    <= 1;
                    state                   <= S_ADDR;
                end
            end

            S_ADDR: begin
                // one cycle for a registered (BRAM) read of x_in/w_in at the
                // current idx to settle; no action needed otherwise.
                state <= S_ISSUE;
            end

            S_ISSUE: begin
                // x_in/w_in now reflect the CURRENT idx (combinational
                // callers were already valid; registered/BRAM callers had
                // their one settle cycle in S_ADDR) -- latch into the
                // atomic MAC.
                atomic_x     <= x_in;
                atomic_wflat <= w_in;
                atomic_start <= 1;
                just_issued  <= 1;
                state        <= S_WAIT;
            end

            S_WAIT: begin
                if (just_issued) begin
                    just_issued <= 0; // atomic_done/atomic_y may still reflect the PRIOR call this cycle
                end else if (atomic_done) begin
                    result                  <= result + atomic_y;
                    tot_toggle_count        <= tot_toggle_count        + atomic_toggle;
                    tot_adder_invocations   <= tot_adder_invocations   + atomic_adder_inv;
                    tot_addition_operations <= tot_addition_operations + atomic_add_ops;

                    if (idx == k_reg - 1) begin
                        result_valid <= 1;
                        busy         <= 0;
                        state        <= S_IDLE;
                    end else begin
                        idx   <= idx + 1;
                        state <= S_ADDR;
                    end
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
