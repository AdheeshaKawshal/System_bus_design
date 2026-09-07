module master #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8,
    parameter RW      = 1,
    parameter NUM_TXN  = 8,
    parameter START_TXN     = 0,  // tx_ptr's starting index - which transaction in the table to begin from
    parameter REQ_DELAY    = 0,  // clock cycles to hold off after reset before req_o is ever asserted
    parameter WRITE_DELAY    = 26
)(
    input wire clk,
    input wire rst,

    // External packet select: chooses which entry in the transaction
    // table (tx_ptr) is sent on the next request. Sampled when leaving
    // IDLE to start a new request.
    input wire [3:0] pkt_sel_i,

    // Arbiter interface
    output reg req_o,
    input wire grant_i,

    // Serial bus interface (driven only while granted).
    output wire addr_data_o,    // serial {addr,we,wdata} request frame, MSB first
    output wire frame_valid_o,  // request frame-start strobe
    output wire mready_o,       // this master is always ready for its response

    input wire rdata_ser_i,     // serial rdata response frame, MSB first
    input wire rvalid_i         // response frame-valid (held-high style)
);
    // FSM states
    localparam WAIT    = 3'd0,
               IDLE    = 3'd1,
               REQUEST = 3'd2,
               ACTIVE  = 3'd3;

    reg [2:0] state;
    reg [31:0] delay_cnt;
    reg [31:0] timeout_cnt;   // cycles spent granted in ACTIVE (write hold only - a read just waits for rvalid)

    // Transaction memory: type (we), addr, wdata and space to store read
    // results. Indexed 1..NUM_TXN (not 0-based) so index 0 stays free as
    // pkt_sel_i's "no selection" sentinel.
    reg [DATA_W-1:0] wdata_mem [1:NUM_TXN];
    reg [ADDR_W-1:0] addr_mem  [1:NUM_TXN];
    reg              we_mem    [1:NUM_TXN];
    reg [DATA_W-1:0] rdata_mem [1:NUM_TXN];

    // current transaction pointer and count - kept 4 bits wide to match
    // pkt_sel_i, the widest index external logic can actually supply.
    reg [3:0] tx_ptr;
    integer i;

    // This simple master model has no reason to ever refuse a response.
    assign mready_o = 1'b1;

    // ---------------------------------------------------------
    // Outgoing request: one addr_serializer builds the {addr,we,wdata}
    // frame and shifts it out on addr_data_o, self-timed once triggered.
    // tx_start pulses for exactly one cycle on the REQUEST -> ACTIVE
    // transition (i.e. the cycle grant_i is first seen).
    // ---------------------------------------------------------
    reg tx_start;

    addr_serializer #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_addr_serializer (
        .clk         (clk),
        .rst_n       (rst),
        .addr_i      ({addr_mem[tx_ptr], we_mem[tx_ptr]}),
        .wdata_i     (wdata_mem[tx_ptr]),
        .valid_i     (tx_start),
        .serial_out      (addr_data_o),
        .frame_valid_out (frame_valid_o)
    );

    // ---------------------------------------------------------
    // Incoming response: one deserializer recovers the read data byte.
    // Its data_valid output stays high once bit 7 is captured (until the
    // next reception starts), so an edge-detect below turns it into a
    // one-cycle capture pulse.
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_par;
    wire               rvalid_par;
    reg                rvalid_par_d;
    wire               rvalid_par_pulse = rvalid_par && !rvalid_par_d;

    deserializer u_deserializer (
        .clk_in       (clk),
        .rst_n        (rst),
        .serial_in    (rdata_ser_i),
        .data_valid_in(rvalid_i),
        .data_out     (rdata_par),
        .data_valid   (rvalid_par)
    );

    // On reset populate a simple transaction table (can be customized for simulation)
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state        <= WAIT;
            delay_cnt    <= 0;
            timeout_cnt  <= 0;
            req_o        <= 1'b0;
            tx_start     <= 1'b0;
            rvalid_par_d <= 1'b0;
            tx_ptr       <= START_TXN;
            for (i = 1; i <= NUM_TXN; i = i + 1) begin
                addr_mem[i]  <= {ADDR_W{1'b0}};
                wdata_mem[i] <= {DATA_W{1'b0}};
                we_mem[i]    <= 1'b0;
                rdata_mem[i] <= {DATA_W{1'b0}};
            end

            // 1: write slave1 addr 0x001 <- 0x11
            // 2: read  slave1 addr 0x001
            // 3: write slave1 addr 0x005 <- 0x22
            // 4: read  slave1 addr 0x005
            // 5: write slave1 addr 0x001 <- 0x33
            // 6: read  slave2 addr 0x001
            // 7: write slave2 addr 0x008 <- 0x44
            // 8: read  slave2 addr 0x008
            // Address layout is {external_flag(1), slave_sel(2), slave_addr(12)}
            // (see addr_redirect.v/addr_decoder.v) - external_flag must be 0
            // for a genuine on-bus slave1/slave2 access, with slave_sel
            // picking 00=slave1, 01=slave2. slave_addr below is the plain
            // 0x001/0x005/0x008 named in the comments above.
            addr_mem[1]  <= 15'h4001; wdata_mem[1] <= 8'h03; we_mem[1] <= 1'b1;
            addr_mem[2]  <= 15'h4001; wdata_mem[2] <= {DATA_W{1'b0}}; we_mem[2] <= 1'b0;
            addr_mem[3]  <= 15'h2005; wdata_mem[3] <= 8'h02; we_mem[3] <= 1'b1;
            addr_mem[4]  <= 15'h2005; wdata_mem[4] <= {DATA_W{1'b0}}; we_mem[4] <= 1'b0;
            addr_mem[5]  <= 15'h0001; wdata_mem[5] <= 8'h04; we_mem[5] <= 1'b1;
            addr_mem[6]  <= 15'h0001; wdata_mem[6] <= {DATA_W{1'b0}}; we_mem[6] <= 1'b0;
            addr_mem[7]  <= 15'h1008; wdata_mem[7] <= 8'h04; we_mem[7] <= 1'b1;
            addr_mem[8]  <= 15'h1008; wdata_mem[8] <= {DATA_W{1'b0}}; we_mem[8] <= 1'b0;

        end else begin
            tx_start     <= 1'b0;
            rvalid_par_d <= rvalid_par;

            case (state)
                WAIT: begin
                    // Hold off req_o until REQ_DELAY cycles have elapsed
                    // since reset released.
                    tx_start     <= 1'b0;
                    if (delay_cnt >= REQ_DELAY) begin
                        state <= IDLE;
                        delay_cnt <= 0;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end

                IDLE: begin
                    // pkt_sel_i == 0 (no bit set) is the reserved "no
                    // selection" sentinel: keep this master parked here,
                    // never asserting req_o, so the arbiter/bus stays free
                    // for the other master to use without collision.
                    // A nonzero selection picks table entry pkt_sel_i
                    // directly (table is indexed 1..NUM_TXN).
                    if (|pkt_sel_i && pkt_sel_i <= NUM_TXN) begin
                        tx_ptr <= pkt_sel_i;
                        req_o  <= 1'b1;
                        state  <= REQUEST;
                    end else begin
                        // no selection, or out-of-range: stay idle and
                        // keep outputs low
                        req_o <= 1'b0;
                    end
                end

                REQUEST: begin
                    // drive request until grant is received, then kick off
                    // the outgoing frame for exactly one cycle
                    if (grant_i) begin
                        tx_start    <= 1'b1;
                        timeout_cnt <= 0;
                        state       <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    if (we_mem[tx_ptr]) begin
                        // Write: no acknowledgement on this bus - just hold
                        // the grant long enough for the frame to clear the
                        // request line, then release.
                        if (timeout_cnt >= WRITE_DELAY) begin
                            req_o       <= 1'b0;
                            timeout_cnt <= 0;
                            state       <= WAIT;
                        end else begin
                            timeout_cnt <= timeout_cnt + 1;
                        end
                    end else begin
                        // Read: hold here indefinitely until the
                        // deserializer's capture pulse arrives - no
                        // timeout, this master waits as long as it takes
                        // (needed for the external/bridged slave path,
                        // whose UART round trip can far exceed WRITE_DELAY).
                        if (rvalid_par_pulse) begin
                            rdata_mem[tx_ptr] <= rdata_par;
                            req_o       <= 1'b0;
                            state       <= WAIT;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
