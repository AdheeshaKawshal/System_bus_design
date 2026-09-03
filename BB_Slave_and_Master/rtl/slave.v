// ============================================================================
// slave.v  (BB_Slave_and_Master copy)
// ----------------------------------------------------------------------------
// Plain register-file bus slave, 2 KB deep.
//
// Differences from the top-level slave.v this was copied from:
//   1. 2 KB of storage (mem[0:2047], ADDR_W = 11) instead of 16 bytes. The
//      original declared mem[0:15] but wrote via addr_i[3:0] and read via the
//      full addr_i -- with an 11-bit address that read indexes past the end of
//      the array and returns X. Both paths now use the same full addr_i.
//   2. ready_o removed. The bus takes only rdata + rvalid back from a slave;
//      there is no ready/backpressure handshake anywhere in this design. A
//      write simply happens on the cycle it is presented, and a read answers
//      with rvalid one cycle later.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module slave #(
    parameter ADDR_W = 11,          // 11 bits -> 2 KB
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,                 // active-low

    input wire                  cs_i,     // chip select from addr_decoder
    input wire                  valid_i,  // master has a valid transaction
    input wire                  we_i,     // 1 = write, 0 = read
    (* MARK_DEBUG = "TRUE" *) input wire [ADDR_W-1:0] addr_i,
    (* MARK_DEBUG = "TRUE" *) input wire [DATA_W-1:0] wdata_i,

    (* MARK_DEBUG = "TRUE" *) output reg [DATA_W-1:0] rdata_o,  // data back to master on a read
    output reg                  rvalid_o  // pulses when rdata_o holds valid read data
);

    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

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
                    mem[addr_i] <= wdata_i;
                end else begin
                    rdata_o  <= mem[addr_i];
                    rvalid_o <= 1'b1;
                end
            end
        end
    end

endmodule
