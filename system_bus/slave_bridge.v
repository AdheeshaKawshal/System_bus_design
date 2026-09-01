// slave_bridge: mirror of master_bridge. Reserves the local bus on behalf
// of a remote master_bridge (req/grant handshake over the link, ahead of
// any data), then receives the transaction over the two serial lines a
// master_bridge sends once granted (addr_rx_i <- addr_tx_o, wdata_rx_i <-
// wdata_tx_o), reassembles it, and drives it onto the local bus as a
// master - req_o/grant_i/addr_o/wdata_o/valid_o/rdata_i/ready_i/rvalid_i
// deliberately match master.v's port shape so this drops straight into a
// system_bus master port (req_M0/grant_M0/...).
//
// Reserving the bus before any data crosses the link means the wait for
// local-bus availability never leaves data sitting buffered here (see the
// stale-retry hazard this replaces): the remote master_bridge is the one
// waiting, in its own domain, using its own timeout logic - nothing here
// is holding a half-received transaction hostage to local bus contention.
//
// rdata+rvalid are packed into a single uart_rx word master_bridge decodes
// (only meaningful for reads); req/grant/ready are latency-sensitive
// single-bit handshakes and each get their own dedicated, stretched line
// instead - see STRETCH_CYCLES below.
module slave_bridge #(
    parameter ADDR_W         = 15,
    parameter DATA_W         = 8,
    parameter RW              = 1,
    parameter CLOCKS_PER_PULSE = 2,
    parameter STRETCH_CYCLES  = 4  // extra clk cycles req_rx_i/grant_tx_o/ready_tx_o... are held, so the other clock domain has plenty of time to sample directly
)(
    input  wire clk,
    input  wire rst,

    // ============================================================
    // Internal side - local bus master port (same shape as master.v,
    // plugs straight into a system_bus master port: req_M0/grant_M0/...)
    // ============================================================
    output reg                  req_o,
    input  wire                 grant_i,
    output reg  [ADDR_W+RW-1:0] addr_o,
    output reg  [DATA_W-1:0]    wdata_o,
    output reg                  valid_o,

    input  wire [DATA_W-1:0]    rdata_i,
    input  wire                 ready_i,
    input  wire                 rvalid_i,

    // ============================================================
    // External side - serial link to the remote master_bridge
    // ============================================================
    input  wire                 req_rx_i,         // raw stretched: remote wants the local bus
    output wire                 grant_tx_o,        // stretched: local bus is reserved, remote may send addr/wdata now
    input  wire                 addr_rx_i,
    input  wire                 wdata_rx_i,
    output wire                 link_rx_busy_o,    // high from the request onward, until the transaction completes
    output wire                 rdata_tx_o,
    output wire                 link_tx_busy_o,    // high while the rvalid+rdata word is shifting out
    output wire                 ready_tx_o         // stretched: transaction complete
);

    localparam S_IDLE       = 2'd0,
               S_WAIT_GRANT = 2'd1,  // requested the local bus, waiting for its own arbiter
               S_WAIT_DATA  = 2'd2,  // local bus reserved, waiting for addr/wdata to arrive over the link
               S_ACTIVE     = 2'd3;

    (* MARK_DEBUG = "TRUE" *) reg [1:0] state;
    assign link_rx_busy_o = (state != S_IDLE);

    // ---------------------------------------------------------
    // req_rx_i: raw stretched pulse from the remote master_bridge's own
    // clock domain. Synchronize onto this module's clk (2 flops resolve
    // metastability - the stretch guarantees an edge lands inside the
    // high window - then edge-detect regenerates a clean one-cycle pulse).
    // ---------------------------------------------------------
    reg req_sync0, req_sync1, req_sync2;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            req_sync0 <= 1'b0;
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
        end else begin
            req_sync0 <= req_rx_i;
            req_sync1 <= req_sync0;
            req_sync2 <= req_sync1;
        end
    end
    (* MARK_DEBUG = "TRUE" *) wire req_pulse = req_sync1 && !req_sync2;

    (* MARK_DEBUG = "TRUE" *) wire       addr_rx_ready;
    wire [ADDR_W+RW-1:0]      addr_rx_word;
    (* MARK_DEBUG = "TRUE" *) wire       wdata_rx_ready;
    wire [DATA_W-1:0]         wdata_rx_word;

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (ADDR_W + RW)
    ) addr_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (addr_rx_i),
        .ready    (addr_rx_ready),
        .data_out (addr_rx_word)
    );

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W)
    ) wdata_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (wdata_rx_i),
        .ready    (wdata_rx_ready),
        .data_out (wdata_rx_word)
    );

    // addr and wdata arrive on separate lines and, since they're different
    // widths, complete at different times - latch each independently and
    // only drive the transaction once both halves have arrived. Only
    // meaningful in S_WAIT_DATA: the remote side never sends these before
    // grant_tx_o has gone out, so nothing is missed by not watching them
    // in the other states.
    (* MARK_DEBUG = "TRUE" *) reg addr_got, wdata_got;
    reg [ADDR_W+RW-1:0] addr_word_r;
    reg [DATA_W-1:0]    wdata_word_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state        <= S_IDLE;
            addr_got     <= 1'b0;
            wdata_got    <= 1'b0;
            addr_word_r  <= {(ADDR_W+RW){1'b0}};
            wdata_word_r <= {DATA_W{1'b0}};
            req_o        <= 1'b0;
            valid_o      <= 1'b0;
            addr_o       <= {(ADDR_W+RW){1'b0}};
            wdata_o      <= {DATA_W{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (req_pulse) begin
                        req_o <= 1'b1;
                        state <= S_WAIT_GRANT;
                    end
                end

                S_WAIT_GRANT: begin
                    if (grant_i) begin
                        state <= S_WAIT_DATA;
                    end
                end

                S_WAIT_DATA: begin
                    if (addr_rx_ready)  begin addr_word_r  <= addr_rx_word;  addr_got  <= 1'b1; end
                    if (wdata_rx_ready) begin wdata_word_r <= wdata_rx_word; wdata_got <= 1'b1; end

                    if ((addr_got || addr_rx_ready) && (wdata_got || wdata_rx_ready)) begin
                        addr_o    <= addr_rx_ready  ? addr_rx_word  : addr_word_r;
                        wdata_o   <= wdata_rx_ready ? wdata_rx_word : wdata_word_r;
                        valid_o   <= 1'b1;  // bus already reserved - no extra grant wait needed here
                        addr_got  <= 1'b0;
                        wdata_got <= 1'b0;
                        state     <= S_ACTIVE;
                    end
                end

                S_ACTIVE: begin
                    if (ready_i) begin
                        valid_o <= 1'b0;
                        req_o   <= 1'b0;
                        state   <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------
    // grant_tx_o: fires the cycle the local arbiter grants the bus,
    // stretched over STRETCH_CYCLES so the remote master_bridge (a
    // different, unrelated clock domain) can sample it directly.
    // ---------------------------------------------------------
    wire grant_fire = (state == S_WAIT_GRANT) && grant_i;

    reg [$clog2(STRETCH_CYCLES+1)-1:0] grant_stretch_cnt;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            grant_stretch_cnt <= 0;
        end else if (grant_fire) begin
            grant_stretch_cnt <= STRETCH_CYCLES;
        end else if (grant_stretch_cnt != 0) begin
            grant_stretch_cnt <= grant_stretch_cnt - 1'b1;
        end
    end
    assign grant_tx_o = grant_fire || (grant_stretch_cnt != 0);

    // ---------------------------------------------------------
    // ready_tx_o: same stretch technique, fired on transaction completion.
    // rdata/rvalid - only meaningful for reads - still ride the serialized
    // link instead, since they're multi-bit and need the fixed framing.
    // ---------------------------------------------------------
    wire tx_data_en = (state == S_ACTIVE) && ready_i;
    wire [DATA_W:0] status_word = {rvalid_i, rdata_i};

    reg [$clog2(STRETCH_CYCLES+1)-1:0] ready_stretch_cnt;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            ready_stretch_cnt <= 0;
        end else if (tx_data_en) begin
            ready_stretch_cnt <= STRETCH_CYCLES;
        end else if (ready_stretch_cnt != 0) begin
            ready_stretch_cnt <= ready_stretch_cnt - 1'b1;
        end
    end
    assign ready_tx_o = tx_data_en || (ready_stretch_cnt != 0);

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W + 1)
    ) status_tx (
        .data_in  (status_word),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (rdata_tx_o),
        .tx_busy  (link_tx_busy_o)
    );

endmodule
