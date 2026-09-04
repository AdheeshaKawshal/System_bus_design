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

    // Transaction memory: type (we), addr, wdata and space to store read results
    reg [DATA_W-1:0] wdata_mem [0:NUM_TXN-1];
    reg [ADDR_W-1:0] addr_mem  [0:NUM_TXN-1];
    reg              we_mem    [0:NUM_TXN-1];
    reg [DATA_W-1:0] rdata_mem [0:NUM_TXN-1];

    // current transaction pointer and count
    reg [$clog2(NUM_TXN)-1:0] tx_ptr;
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
            for (i = 0; i < NUM_TXN; i = i + 1) begin
                addr_mem[i]  <= {ADDR_W{1'b0}};
                wdata_mem[i] <= {DATA_W{1'b0}};
                we_mem[i]    <= 1'b0;
                rdata_mem[i] <= {DATA_W{1'b0}};
            end

            // 0: write slave1 addr 0x001 <- 0x11
            // 1: read  slave1 addr 0x001
            // 2: write slave1 addr 0x005 <- 0x22
            // 3: read  slave1 addr 0x005
            // 4: write slave1 addr 0x001 <- 0x33
            // 5: read  slave2 addr 0x001
            // 6: write slave2 addr 0x008 <- 0x44
            // 7: read  slave2 addr 0x008
            // Address layout is {external_flag(1), slave_sel(2), slave_addr(12)}
            // (see addr_redirect.v/addr_decoder.v) - external_flag must be 0
            // for a genuine on-bus slave1/slave2 access, with slave_sel
            // picking 00=slave1, 01=slave2. slave_addr below is the plain
            // 0x001/0x005/0x008 named in the comments above.
            addr_mem[0]  <= 15'h4001; wdata_mem[0] <= 8'h11; we_mem[0] <= 1'b1;
            addr_mem[1]  <= 15'h4001; wdata_mem[1] <= {DATA_W{1'b0}}; we_mem[1] <= 1'b0;
            addr_mem[2]  <= 15'h2005; wdata_mem[2] <= 8'h22; we_mem[2] <= 1'b1;
            addr_mem[3]  <= 15'h2005; wdata_mem[3] <= {DATA_W{1'b0}}; we_mem[3] <= 1'b0;
            addr_mem[4]  <= 15'h0001; wdata_mem[4] <= 8'h33; we_mem[4] <= 1'b1;
            addr_mem[5]  <= 15'h1001; wdata_mem[5] <= {DATA_W{1'b0}}; we_mem[5] <= 1'b0;
            addr_mem[6]  <= 15'h1008; wdata_mem[6] <= 8'h44; we_mem[6] <= 1'b1;
            addr_mem[7]  <= 15'h1008; wdata_mem[7] <= {DATA_W{1'b0}}; we_mem[7] <= 1'b0;

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
                    // If there are remaining transactions, request the bus
                    if (tx_ptr < NUM_TXN) begin
                        req_o <= 1'b1;
                        state <= REQUEST;
                    end else begin
                        // no more transactions: stay idle and keep outputs low
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
                            tx_ptr      <= tx_ptr + 1;
                            req_o       <= 1'b0;
                            timeout_cnt <= 0;
                            state       <= WAIT;
                        end else begin
                            timeout_cnt <= timeout_cnt + 1;
                        end
                    end else begin
                        // Read: hold here indefinitely until the
                        // deserializer's capture pulse arrives - no
                        // timeout, this master waits as long as it takes.
                        if (rvalid_par_pulse) begin
                            rdata_mem[tx_ptr] <= rdata_par;
                            tx_ptr      <= tx_ptr + 1;
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
