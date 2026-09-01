// master_bridge: forwards system_bus external-port transactions across a
// UART serial link. Reserves the remote bus first (req/grant handshake
// over the link) and only starts sending addr/wdata once the remote side
// confirms the grant - so the remote bus's own arbitration delay is
// absorbed here, before any data is committed to the wire, instead of
// leaving a fully-received transaction sitting on the far side waiting on
// local grant (see slave_bridge.v's header for the hazard this avoids).
//
// Address (+we) and wdata are each sent whole in one uart_tx word
// (DATA_WIDTH configured to ADDR_W+RW / DATA_W - no manual byte-splitting
// needed) over their own serial line, in parallel, once granted.
//
// The return link carries rdata plus rvalid packed into a single uart_rx
// word (only meaningful for reads). req/grant/ready are latency-sensitive
// single-bit handshakes and each get their own dedicated, stretched line
// instead - synchronized onto this module's own clk below.
//
// link_busy_o / link_rx_valid_o are exposed so anything wiring this bridge
// to an external bus/monitor can see TX/RX activity without reaching
// inside the module.
module master_bridge #(
    parameter ADDR_W          = 15,
    parameter DATA_W          = 8,
    parameter RW               = 1,
    parameter CLOCKS_PER_PULSE = 2,
    parameter STRETCH_CYCLES   = 4  // extra clk cycles req_tx_o is held, so the remote clock domain has plenty of time to sample it directly
)(
    input  wire clk,
    input  wire rst,

    // ============================================================
    // Internal side - system_bus's ext_* port (this module looks like a
    // slave to bus's own arbiter here, matching addr_M.../grant_ext shape)
    // ============================================================
    input  wire [ADDR_W+RW-1:0] addr_ext_o,
    input  wire [DATA_W-1:0]    wdata_ext_o,
    input  wire                 ext_valid_o,

    output wire [DATA_W-1:0]    rdata_ext_i,
    output wire                 ready_ext_i,
    output wire                 rvalid_ext_i,

    input  wire                 req_ext,
    output wire                 grant_ext,

    // external arbiter interface: this bridge contends for the shared
    // serial link the same way a master contends for the local bus (local
    // arbitration only - separate from the remote req/grant below)
    output wire                 ext_arb_req_o,
    input  wire                 ext_arb_grant_i,

    // ============================================================
    // External side - serial link to the remote slave_bridge
    // ============================================================
    output wire                 req_tx_o,          // stretched: reserve the remote bus
    input  wire                 grant_rx_i,        // raw stretched: remote bus reserved, safe to send now
    output wire                 addr_tx_o,
    output wire                 wdata_tx_o,
    output wire                 link_busy_o,       // high while addr/wdata still shifting out
    input  wire                 uart_rx_i,
    output wire                 link_rx_valid_o,   // pulses when an rvalid+rdata word has arrived
    input  wire                 ready_rx_i         // raw stretched: transaction complete
);

    localparam RX_WIDTH = DATA_W + 1;   // {rvalid, rdata} - req/grant/ready travel on their own dedicated lines

    // pulse tx_start_en for one cycle on ext_valid_o's rising edge so a
    // held-high valid doesn't retrigger the reservation once idle again
    reg ext_valid_o_d;
    always @(posedge clk or negedge rst)
        if (!rst) ext_valid_o_d <= 1'b0;
        else      ext_valid_o_d <= ext_valid_o;

    wire tx_start_en = ext_valid_o && !ext_valid_o_d;

    // Forward each local request to the shared serial link's own local
    // arbiter (unrelated to reserving the remote bus - this is purely
    // about who on *this* side gets to use the link).
    assign ext_arb_req_o = req_ext;
    assign grant_ext     = req_ext && ext_arb_grant_i;

    // ---------------------------------------------------------
    // Reservation FSM: latch the transaction on ext_valid_o's rising
    // edge, request the remote bus, and only fire addr_tx/wdata_tx once
    // the remote grant has been confirmed.
    // ---------------------------------------------------------
    localparam M_IDLE       = 1'b0,
               M_WAIT_GRANT = 1'b1;

    (* MARK_DEBUG = "TRUE" *) reg                  m_state;
    reg [ADDR_W+RW-1:0]  addr_latch;
    reg [DATA_W-1:0]     wdata_latch;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            m_state     <= M_IDLE;
            addr_latch  <= {(ADDR_W+RW){1'b0}};
            wdata_latch <= {DATA_W{1'b0}};
        end else begin
            case (m_state)
                M_IDLE: begin
                    if (tx_start_en) begin
                        addr_latch  <= addr_ext_o;
                        wdata_latch <= wdata_ext_o;
                        m_state     <= M_WAIT_GRANT;
                    end
                end
                M_WAIT_GRANT: begin
                    if (grant_pulse) begin
                        m_state <= M_IDLE;
                    end
                end
                default: m_state <= M_IDLE;
            endcase
        end
    end

    // req_tx_o: fires on tx_start_en, stretched so the remote slave_bridge
    // (a different clock domain) has plenty of time to sample it directly.
    reg [$clog2(STRETCH_CYCLES+1)-1:0] req_stretch_cnt;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            req_stretch_cnt <= 0;
        end else if (tx_start_en) begin
            req_stretch_cnt <= STRETCH_CYCLES;
        end else if (req_stretch_cnt != 0) begin
            req_stretch_cnt <= req_stretch_cnt - 1'b1;
        end
    end
    (* MARK_DEBUG = "TRUE" *) wire req_tx_o_dbg;
    assign req_tx_o     = tx_start_en || (req_stretch_cnt != 0);
    assign req_tx_o_dbg = req_tx_o;

    // grant_rx_i: raw stretched level from the remote domain - synchronize
    // (2 flops resolve metastability, edge-detect regenerates a clean
    // one-cycle pulse) before using it to gate the actual data send.
    reg grant_sync0, grant_sync1, grant_sync2;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            grant_sync0 <= 1'b0;
            grant_sync1 <= 1'b0;
            grant_sync2 <= 1'b0;
        end else begin
            grant_sync0 <= grant_rx_i;
            grant_sync1 <= grant_sync0;
            grant_sync2 <= grant_sync1;
        end
    end
    (* MARK_DEBUG = "TRUE" *) wire grant_pulse = grant_sync1 && !grant_sync2;

    // The actual addr/wdata transmission only fires once the remote bus
    // is confirmed reserved.
    (* MARK_DEBUG = "TRUE" *) wire tx_data_en = (m_state == M_WAIT_GRANT) && grant_pulse;

    wire addr_tx_busy, wdata_tx_busy;
    assign link_busy_o = addr_tx_busy | wdata_tx_busy;

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (ADDR_W + RW)
    ) addr_tx (
        .data_in  (addr_latch),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (addr_tx_o),
        .tx_busy  (addr_tx_busy)
    );

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W)
    ) wdata_tx (
        .data_in  (wdata_latch),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (wdata_tx_o),
        .tx_busy  (wdata_tx_busy)
    );

    wire [RX_WIDTH-1:0] rx_word;

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (RX_WIDTH)
    ) status_rdata_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (uart_rx_i),
        .ready    (link_rx_valid_o),
        .data_out (rx_word)
    );

    // rx_word (uart_rx's data_out) is really its internal shift register -
    // it changes bit by bit while a word is still coming in, so it must
    // only be sampled once link_rx_valid_o (the completed serial-to-
    // parallel conversion) actually pulses. Latch it there and hold
    // rvalid high for exactly that one cycle, matching the pulse
    // semantics master.v's ACTIVE state expects.
    reg [DATA_W-1:0] rdata_ext_i_r;
    reg              rvalid_ext_i_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_ext_i_r  <= {DATA_W{1'b0}};
            rvalid_ext_i_r <= 1'b0;
        end else if (link_rx_valid_o) begin
            rdata_ext_i_r  <= rx_word[DATA_W-1:0];
            rvalid_ext_i_r <= rx_word[DATA_W];
        end else begin
            rvalid_ext_i_r <= 1'b0;
        end
    end

    // ready_rx_i: same raw-level-plus-synchronizer treatment as grant_rx_i
    // above, held high for several cycles by slave_bridge.v specifically
    // so this domain's own clock is guaranteed at least one edge inside
    // that window.
    reg ready_sync0, ready_sync1, ready_sync2;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            ready_sync0 <= 1'b0;
            ready_sync1 <= 1'b0;
            ready_sync2 <= 1'b0;
        end else begin
            ready_sync0 <= ready_rx_i;
            ready_sync1 <= ready_sync0;
            ready_sync2 <= ready_sync1;
        end
    end

    reg ready_ext_i_r;
    always @(posedge clk or negedge rst) begin
        if (!rst) ready_ext_i_r <= 1'b0;
        else      ready_ext_i_r <= ready_sync1 && !ready_sync2;
    end

    assign rdata_ext_i  = rdata_ext_i_r;
    assign ready_ext_i  = ready_ext_i_r;
    assign rvalid_ext_i = rvalid_ext_i_r;

endmodule
