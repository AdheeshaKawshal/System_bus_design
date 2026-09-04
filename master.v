module master #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8,
    parameter RW      = 1,
    parameter NUM_TXN  = 8,
    parameter START_TXN     = 0,  // tx_ptr's starting index - which transaction in the table to begin from
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
    input wire rvalid_i   // pulses high alongside rdata_i when the slave's read data is valid
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

    // Transaction memory: type (we), addr, wdata and space to store read results
    reg [DATA_W-1:0] wdata_mem [0:NUM_TXN-1];
    reg [ADDR_W-1:0] addr_mem  [0:NUM_TXN-1];
    reg              we_mem    [0:NUM_TXN-1];
    reg [DATA_W-1:0] rdata_mem [0:NUM_TXN-1];

    // current transaction pointer and count
    reg [$clog2(NUM_TXN)-1:0] tx_ptr;
    integer i;

    // On reset populate a simple transaction table (can be customized for simulation)
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state       <= WAIT;
            delay_cnt   <= 0;
            timeout_cnt <= 0;
            req_o   <= 1'b0;
            valid_o <= 1'b0;
            addr_o  <= {(ADDR_W+RW){1'b0}};
            wdata_o <= {DATA_W{1'b0}};
            tx_ptr  <= START_TXN;
            for (i = 0; i < NUM_TXN; i = i + 1) begin
                addr_mem[i]  <= {ADDR_W{1'b0}};
                wdata_mem[i] <= {DATA_W{1'b0}};
                we_mem[i]    <= 1'b0;
                rdata_mem[i] <= {DATA_W{1'b0}};
            end

            // Example transaction sequence (modify as needed).
            // addr_o encoding (matches addr_decoder): [14]=0 internal, [13:12]=slave sel, [11:0]=slave addr
            //   slave1 -> sel 00 -> addr_mem[13:12]=00 (e.g. 15'h0xxx)
            //   slave2 -> sel 01 -> addr_mem[13:12]=01 (e.g. 15'h1xxx)
            //
            // 0: write slave1 addr 0x001 <- 0x11
            // 1: read  slave1 addr 0x001
            // 2: write slave1 addr 0x005 <- 0x22
            // 3: read  slave1 addr 0x005
            // 4: write slave2 addr 0x001 <- 0x33
            // 5: read  slave2 addr 0x001
            // 6: write slave2 addr 0x008 <- 0x44
            // 7: read  slave2 addr 0x008
            addr_mem[0]  <= 15'h4001; wdata_mem[0] <= 8'h11; we_mem[0] <= 1'b1;
            addr_mem[1]  <= 15'h4001; wdata_mem[1] <= {DATA_W{1'b0}}; we_mem[1] <= 1'b0;
            addr_mem[2]  <= 15'h5005; wdata_mem[2] <= 8'h22; we_mem[2] <= 1'b1;
            addr_mem[3]  <= 15'h5005; wdata_mem[3] <= {DATA_W{1'b0}}; we_mem[3] <= 1'b0;
            addr_mem[4]  <= 15'h4001; wdata_mem[4] <= 8'h33; we_mem[4] <= 1'b1;
            addr_mem[5]  <= 15'h5001; wdata_mem[5] <= {DATA_W{1'b0}}; we_mem[5] <= 1'b0;
            addr_mem[6]  <= 15'h1008; wdata_mem[6] <= 8'h44; we_mem[6] <= 1'b1;
            addr_mem[7]  <= 15'h1008; wdata_mem[7] <= {DATA_W{1'b0}}; we_mem[7] <= 1'b0;

        end else begin
            case (state)
                WAIT: begin
                    // Hold off req_o until REQ_DELAY cycles have elapsed
                    // since reset released.
                    if (delay_cnt >= REQ_DELAY) begin
                        state <= IDLE;
                        delay_cnt <= 0;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end

                IDLE: begin
                    // If there are remaining transactions, request the bus
                    if (tx_ptr < NUM_TXN) begin
                        req_o <= 1'b1;
                        state <= REQUEST;
                    end else begin
                        // no more transactions: stay idle and keep outputs low
                        req_o   <= 1'b0;
                        valid_o <= 1'b0;
                    end
                end

                REQUEST: begin
                    // drive request until grant is received
                    if (grant_i) begin
                        // drive transaction signals from memory (we packed into addr_o's LSB)
                        addr_o  <= {addr_mem[tx_ptr], we_mem[tx_ptr]};
                        wdata_o <= wdata_mem[tx_ptr];
                        valid_o <= 1'b1;
                        state   <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    // Keep valid asserted while waiting for slave ready
                    if (ready_i) begin
                        // capture read data only once the slave flags it valid
                        if (!we_mem[tx_ptr] && rvalid_i) begin
                            rdata_mem[tx_ptr] <= rdata_i;
                        end

                        // transaction complete: advance pointer and release bus
                        tx_ptr      <= tx_ptr + 1;
                        req_o       <= 1'b0;
                        valid_o     <= 1'b0;
                        timeout_cnt <= 0;
                        state       <= WAIT;
                    end
                    else if (timeout_cnt >= ACTIVE_TIMEOUT) begin
                        // Gave up waiting for ready_i (e.g. stuck behind a
                        // stalled cross-bus grant): drop off the bus and
                        // retry the same transaction after a backoff delay
                        // instead of holding the arbiter hostage forever.
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
                            addr_o  <= {addr_mem[tx_ptr], we_mem[tx_ptr]};
                            wdata_o <= wdata_mem[tx_ptr];
                            valid_o <= 1'b1;
                        end
                    end
                end

                BACKOFF: begin
                    // Hold off re-requesting for BACKOFF_DELAY cycles, then
                    // retry the same (still-pending) transaction.
                    if (delay_cnt >= BACKOFF_DELAY) begin
                        delay_cnt <= 0;
                        state     <= WAIT;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
