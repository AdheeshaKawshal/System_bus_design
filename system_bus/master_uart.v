module master_uart #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8,
    parameter RW      = 1,
    parameter REQ_DELAY    = 0,  // clock cycles to hold off after reset before req_o is ever asserted
    parameter ACTIVE_TIMEOUT = 16,  // give up waiting for ready_i after this many granted cycles
    parameter BACKOFF_DELAY  = 5   // cycles to hold off before retrying a timed-out transaction
)(
    input wire clk,
    input wire rst,

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
    (* MARK_DEBUG = "TRUE" *) input wire ready_i,
    input wire ext_valid_o,
    input wire rvalid_i,   // pulses high alongside rdata_i when the slave's read data is valid

    // ------------------------------------------------------------------
    // UART-side transaction interface. Replaces the old hardcoded
    // addr_mem/wdata_mem/we_mem/tx_ptr table: UART_TOP now supplies one
    // transaction at a time and watches txn_ready_o to know when this
    // master is free to accept the next one.
    // ------------------------------------------------------------------
    input  wire              txn_valid_i,   // pulse: latch txn_*_i and start it
    input  wire [ADDR_W-1:0] txn_addr_i,
    input  wire              txn_we_i,
    input  wire [DATA_W-1:0] txn_wdata_i,
    output wire              txn_ready_o,   // high only while idle and free to accept a new txn

    output reg  [DATA_W-1:0] rdata_o,       // valid for one cycle when rdata_valid_o pulses
    output reg               rdata_valid_o, // pulses once per completed read
    output reg               txn_done_o     // pulses once per completed transaction (read or write)
);
    // FSM states
    localparam WAIT    = 3'd0,
               IDLE    = 3'd1,
               REQUEST = 3'd2,
               ACTIVE  = 3'd3,
               BACKOFF = 3'd4;   // granted but ready_i never came: dropped req, waiting to retry

    reg [2:0] state;
    reg [31:0] delay_cnt;
    reg [31:0] timeout_cnt;   // cycles spent granted in ACTIVE without seeing ready_i

    // Latched copy of the transaction currently in flight (replaces the
    // old addr_mem/wdata_mem/we_mem[tx_ptr] lookups). txn_active_r stays
    // set across a BACKOFF retry so the same transaction is re-issued
    // without needing a fresh txn_valid_i pulse from UART_TOP.
    reg [ADDR_W-1:0] cur_addr;
    reg              cur_we;
    reg [DATA_W-1:0] cur_wdata;
    reg              txn_active_r;

    // Only accept a new transaction while sitting idle with nothing
    // in flight (i.e. not mid-retry after a BACKOFF).
    assign txn_ready_o = (state == IDLE) && !txn_active_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state       <= WAIT;
            delay_cnt   <= 0;
            timeout_cnt <= 0;
            req_o   <= 1'b0;
            valid_o <= 1'b0;
            addr_o  <= {(ADDR_W+RW){1'b0}};
            wdata_o <= {DATA_W{1'b0}};

            cur_addr     <= {ADDR_W{1'b0}};
            cur_we       <= 1'b0;
            cur_wdata    <= {DATA_W{1'b0}};
            txn_active_r <= 1'b0;

            rdata_o       <= {DATA_W{1'b0}};
            rdata_valid_o <= 1'b0;
            txn_done_o    <= 1'b0;

        end else begin
            // rdata_valid_o / txn_done_o are single-cycle pulses; default
            // them low each cycle and let the ACTIVE branch below assert
            // them explicitly on completion. req_o/valid_o/state keep the
            // original level-signal behavior (no default assignment).
            rdata_valid_o <= 1'b0;
            txn_done_o    <= 1'b0;

            case (state)
                WAIT: begin
                    // Hold off req_o until REQ_DELAY cycles have elapsed
                    // since reset released.
                    if (delay_cnt >= REQ_DELAY) begin
                        state <= IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end

                IDLE: begin
                    if (txn_active_r) begin
                        // Retrying the same pending transaction after a
                        // BACKOFF timeout: cur_addr/cur_we/cur_wdata are
                        // already latched from before, just re-request.
                        req_o <= 1'b1;
                        state <= REQUEST;
                    end else if (txn_valid_i) begin
                        // Accept a new transaction from UART_TOP.
                        cur_addr     <= txn_addr_i;
                        cur_we       <= txn_we_i;
                        cur_wdata    <= txn_wdata_i;
                        txn_active_r <= 1'b1;
                        req_o        <= 1'b1;
                        state        <= REQUEST;
                    end else begin
                        // nothing to do: stay idle and keep outputs low
                        req_o   <= 1'b0;
                        valid_o <= 1'b0;
                    end
                end

                REQUEST: begin
                    // drive request until grant is received
                    if (grant_i) begin
                        // drive transaction signals from the latched txn (we packed into addr_o's LSB)
                        addr_o  <= {cur_addr, cur_we};
                        wdata_o <= cur_wdata;
                        valid_o <= 1'b1;
                        state   <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    // Keep valid asserted while waiting for slave ready
                    if (ready_i) begin
                        // capture read data only once the slave flags it valid
                        if (!cur_we && rvalid_i) begin
                            rdata_o       <= rdata_i;
                            rdata_valid_o <= 1'b1;
                        end

                        // transaction complete: release bus and go idle
                        txn_active_r <= 1'b0;
                        txn_done_o   <= 1'b1;
                        req_o        <= 1'b0;
                        valid_o      <= 1'b0;
                        timeout_cnt  <= 0;
                        state        <= IDLE;
                    end
                    else if (timeout_cnt >= ACTIVE_TIMEOUT) begin
                        // Gave up waiting for ready_i (e.g. stuck behind a
                        // stalled cross-bus grant): drop off the bus and
                        // retry the same transaction after a backoff delay
                        // instead of holding the arbiter hostage forever.
                        // txn_active_r stays set: same transaction retried.
                        req_o       <= 1'b0;
                        valid_o     <= 1'b0;
                        timeout_cnt <= 0;
                        delay_cnt   <= 0;
                        state       <= BACKOFF;
                    end
                    else begin
                        timeout_cnt <= timeout_cnt + 1;

                        if (!ext_valid_o) begin
                            // keep driving valid and data until slave signals ready
                            addr_o  <= {cur_addr, cur_we};
                            wdata_o <= cur_wdata;
                            valid_o <= 1'b1;
                        end
                    end
                end

                BACKOFF: begin
                    // Hold off re-requesting for BACKOFF_DELAY cycles, then
                    // retry the same (still-pending) transaction.
                    if (delay_cnt >= BACKOFF_DELAY) begin
                        delay_cnt <= 0;
                        state     <= IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
