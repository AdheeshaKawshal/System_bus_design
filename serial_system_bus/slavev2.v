// ============================================================================
// slave.v  (BB_Slave_and_Master copy)
// ----------------------------------------------------------------------------
// Plain register-file bus slave. addr_i's port width stays fully
// parameterized (ADDR_W, unchanged interface), but mem[] itself only has 16
// slots - only addr_i[3:0] is ever used to index it, so addr_i[ADDR_W-1:4]
// alias. This keeps mem small (16*DATA_W bits) regardless of how wide
// addr_i is instantiated, matching slave.v's own 16-slot mem[] size. Callers
// are expected to only ever address the low 16 slots.
//
// This distinction matters for synthesis: at ADDR_W=11 the earlier version
// of this file declared mem[0:2047] (2 KB, indexed by the FULL addr_i) -
// left to inference heuristics that consistently landed on one flip-flop
// per bit instead of Block RAM (not helped by the MARK_DEBUG probes on this
// memory's own ports below), which is where DRC UTLZ-1's FDCE/FDRE overflow
// came from: two instances of this module (one per bus in
// serial_2bus_top.v) each wanting 2048*8 = 16384 FFs for what should cost a
// couple of BRAM tiles. Just not having 2048 slots in the first place sidesteps
// the inference question entirely.
//
// Other differences from the top-level slave.v this was copied from:
//   ready_o removed. The bus takes only rdata + rvalid back from a slave;
//   there is no ready/backpressure handshake anywhere in this design. A
//   write simply happens on the cycle it is presented, and a read answers
//   with rvalid one cycle later.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module bb_local_regfile #(
    parameter ADDR_W = 11,
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,                 // active-low

    input wire                  cs_i,     // chip select from addr_decoder
    input wire                  valid_i,  // master has a valid transaction
    input wire                  we_i,     // 1 = write, 0 = read
    input wire [ADDR_W-1:0] addr_i,
    input wire [DATA_W-1:0] wdata_i,

    output reg [DATA_W-1:0] rdata_o,  // data back to master on a read
    output reg                  rvalid_o  // pulses when rdata_o holds valid read data
);

    reg [DATA_W-1:0] mem [0:15];

    wire sel = cs_i && valid_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_o  <= {DATA_W{1'b0}};
            rvalid_o <= 1'b0;
        end else begin
            rvalid_o <= 1'b0;
            // Write only happens once the master has driven a valid,
            // selected transaction (sel = cs_i && valid_i) on this edge.
            if (sel) begin
                if (we_i) begin
                    mem[addr_i[3:0]] <= wdata_i;
                end else begin
                    rdata_o  <= mem[addr_i[3:0]];
                    rvalid_o <= 1'b1;
                end
            end
        end
    end

endmodule
