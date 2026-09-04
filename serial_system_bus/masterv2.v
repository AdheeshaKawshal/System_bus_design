// ============================================================================
// master.v  (BB_Slave_and_Master copy)
// ----------------------------------------------------------------------------
// Bus master FSM driven ENTIRELY by an external transaction port. Used by
// bb_master_core to put UART-delivered transactions onto the local bus.
//
// Differences from the top-level master.v this was copied from:
//   1. The built-in addr_mem/wdata_mem/we_mem demo table is GONE. In the
//      original, IDLE fell through to "else if (tx_ptr < NUM_TXN)", so the
//      instant reset released this master started firing eight hardcoded
//      transactions onto the bus with no external request at all -- and since
//      tx_ptr is $clog2(NUM_TXN) bits wide it wrapped 7->0, making that
//      condition permanently true and the table loop forever. A UART-driven
//      bridge master must issue nothing it was not asked to issue.
//   2. ready_i is GONE. Slaves return only rdata + rvalid; there is no
//      ready/backpressure handshake in this design. A write is complete the
//      cycle after it is driven; a read completes when rvalid_i arrives.
//   3. ext_valid_o is GONE (it was always tied low here anyway), and with the
//      table went NUM_TXN, START_TXN, tx_ptr and rdata_mem.
//   4. BACKOFF is GONE. It existed to retry a transaction that never got its
//      ready_i; with no ready_i the only thing that can stall is a read whose
//      rvalid never comes, and ACTIVE_TIMEOUT below covers that directly.
//
// Handshake: wait for txn_ready_o high, pulse txn_valid_i for one cycle with
// txn_addr_i/txn_we_i/txn_wdata_i stable, then wait for txn_done_o. On a read,
// txn_rdata_o + txn_rdata_valid_o pulse together with txn_done_o.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module bb_master_txn_core #(
    parameter ADDR_W         = 15,
    parameter DATA_W         = 8,
    parameter RW             = 1,
    parameter ACTIVE_TIMEOUT = 64   // cycles to wait for rvalid_i on a read before giving up
)(
    input wire clk,
    input wire rst,                 // active-low

    // Arbiter interface
    output reg req_o,
    input wire grant_i,

    // Bus interface (driven only while granted).
    // addr_o carries {address, we} packed together (we in the LSB), ready
    // to connect straight to the bus's addr_Mx port with no wrapping needed.
    (* MARK_DEBUG = "TRUE" *) output reg [ADDR_W+RW-1:0] addr_o,
    (* MARK_DEBUG = "TRUE" *) output reg [DATA_W-1:0]    wdata_o,
    (* MARK_DEBUG = "TRUE" *) output reg                 valid_o,

    (* MARK_DEBUG = "TRUE" *) input wire [DATA_W-1:0] rdata_i,
    (* MARK_DEBUG = "TRUE" *) input wire              rvalid_i,  // pulses alongside rdata_i on a read

    // External transaction-injection port -- the only source of transactions.
    input  wire              txn_valid_i,
    input  wire [ADDR_W-1:0] txn_addr_i,
    input  wire              txn_we_i,
    input  wire [DATA_W-1:0] txn_wdata_i,
    output wire              txn_ready_o,
    output reg  [DATA_W-1:0] txn_rdata_o,
    output reg               txn_rdata_valid_o,
    output reg               txn_done_o
);
    localparam IDLE    = 2'd0,
               REQUEST = 2'd1,
               ACTIVE  = 2'd2;

    reg [1:0]  state;
    reg [31:0] timeout_cnt;

    reg [ADDR_W-1:0] cur_addr;
    reg              cur_we;
    reg [DATA_W-1:0] cur_wdata;

    assign txn_ready_o = (state == IDLE);

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state             <= IDLE;
            timeout_cnt       <= 32'd0;
            req_o             <= 1'b0;
            valid_o           <= 1'b0;
            addr_o            <= {(ADDR_W+RW){1'b0}};
            wdata_o           <= {DATA_W{1'b0}};
            cur_addr          <= {ADDR_W{1'b0}};
            cur_we            <= 1'b0;
            cur_wdata         <= {DATA_W{1'b0}};
            txn_rdata_o       <= {DATA_W{1'b0}};
            txn_rdata_valid_o <= 1'b0;
            txn_done_o        <= 1'b0;
        end else begin
            txn_rdata_valid_o <= 1'b0;   // default: 1-cycle pulse
            txn_done_o        <= 1'b0;   // default: 1-cycle pulse

            case (state)
                IDLE: begin
                    req_o   <= 1'b0;
                    valid_o <= 1'b0;
                    if (txn_valid_i) begin
                        cur_addr  <= txn_addr_i;
                        cur_we    <= txn_we_i;
                        cur_wdata <= txn_wdata_i;
                        req_o     <= 1'b1;
                        state     <= REQUEST;
                    end
                end

                REQUEST: begin
                    // hold the request up until the arbiter grants the bus
                    if (grant_i) begin
                        addr_o      <= {cur_addr, cur_we};
                        wdata_o     <= cur_wdata;
                        valid_o     <= 1'b1;
                        timeout_cnt <= 32'd0;
                        state       <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    if (cur_we) begin
                        // Write: the slave latches it on this very cycle (no
                        // ready to wait for), so the transaction is done.
                        req_o      <= 1'b0;
                        valid_o    <= 1'b0;
                        txn_done_o <= 1'b1;
                        state      <= IDLE;
                    end else if (rvalid_i) begin
                        txn_rdata_o       <= rdata_i;
                        txn_rdata_valid_o <= 1'b1;
                        txn_done_o        <= 1'b1;
                        req_o             <= 1'b0;
                        valid_o           <= 1'b0;
                        state             <= IDLE;
                    end else if (timeout_cnt >= ACTIVE_TIMEOUT) begin
                        // Read whose rvalid never came (unmapped address, dead
                        // slave). Release the bus and report the transaction
                        // done with no read data rather than hanging here --
                        // the far side is blocked on this reply.
                        req_o      <= 1'b0;
                        valid_o    <= 1'b0;
                        txn_done_o <= 1'b1;
                        state      <= IDLE;
                    end else begin
                        timeout_cnt <= timeout_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
