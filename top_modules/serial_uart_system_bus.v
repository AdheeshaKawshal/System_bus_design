module serial_uart_system_bus (
    input wire clk,
    input wire rst_n,

    // ---------------------------------------------------------
    // Outbound: this board's system_bus acting as master onto the other
    // board, via master_bridge. Wire straight to the other board's
    // rx_req_i/tx_grant_o/rx_addr_i/rx_wdata_i/tx_status_o/tx_ready_o.
    // ---------------------------------------------------------
    output wire tx_req_o,
    input  wire rx_grant_i,
    output wire tx_addr_o,
    output wire tx_wdata_o,
    input  wire rx_status_i,
    input  wire rx_ready_i,

    // ---------------------------------------------------------
    // Inbound: the other board's master reaching onto this board's Master
    // 1 port, via slave_bridge. Wire straight to the other board's
    // tx_req_o/rx_grant_i/tx_addr_o/tx_wdata_o/rx_status_i/rx_ready_i.
    // ---------------------------------------------------------
    input  wire rx_req_i,
    output wire tx_grant_o,
    input  wire rx_addr_i,
    input  wire rx_wdata_i,
    output wire tx_status_o,
    output wire tx_ready_o,

    // Physical UART to a PC (via USB-TTL) - drives both local masters
    // (u_master0/u_master1 below, now master_uart) with transactions
    // framed over this link instead of hardcoded tables.
    input  wire uart_rx_serial,
    output wire uart_tx_serial,

    output wire [3:0] led
);
    wire rst = !rst_n;

    localparam ADDR_W = 15;
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Clock divider: clk -> a slow tick, via a free-running counter.
    // ---------------------------------------------------------
    localparam integer CLK_FREQ_HZ  = 125000000;
    localparam integer TICK_FREQ_HZ = 4;
    localparam integer DIV_MAX      = CLK_FREQ_HZ / TICK_FREQ_HZ - 1;

    reg [$clog2(DIV_MAX+1)-1:0] div_cnt;
    reg                         tick_4hz;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            div_cnt  <= 0;
            tick_4hz <= 1'b0;
        end else if (div_cnt == DIV_MAX) begin
            div_cnt  <= 0;
            tick_4hz <= 1'b1;
        end else begin
            div_cnt  <= div_cnt + 1'b1;
            tick_4hz <= 1'b0;
        end
    end

    // ---------------------------------------------------------
    // Master 0 - local, internal to this bus like system_busv1's own
    // Master 0.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 req_M0, grant_M0;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] addr_M0;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    wdata_M0;
    (* MARK_DEBUG = "TRUE" *) wire                 valid_M0;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    rdata_M0;
    (* MARK_DEBUG = "TRUE" *) wire                 ready_M0;
    (* MARK_DEBUG = "TRUE" *) wire                 rvalid_M0;

    wire                 m0_txn_valid, m0_txn_we, m0_txn_ready;
    wire [ADDR_W-1:0]    m0_txn_addr;
    wire [DATA_W-1:0]    m0_txn_wdata;
    wire [DATA_W-1:0]    m0u_rdata;
    wire                 m0u_rdata_valid, m0_txn_done;

    master_uart #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RW        (RW),
        .REQ_DELAY (4)
    ) u_master0 (
        .clk         (tick_4hz),
        .rst         (rst),
        .req_o       (req_M0),
        .grant_i     (grant_M0),
        .addr_o      (addr_M0),
        .wdata_o     (wdata_M0),
        .valid_o     (valid_M0),
        .rdata_i     (rdata_M0),
        .ready_i     (ready_M0),
        .rvalid_i    (rvalid_M0),
        .ext_valid_o (ext_valid),

        .txn_valid_i   (m0_txn_valid),
        .txn_addr_i    (m0_txn_addr),
        .txn_we_i      (m0_txn_we),
        .txn_wdata_i   (m0_txn_wdata),
        .txn_ready_o   (m0_txn_ready),
        .rdata_o       (m0u_rdata),
        .rdata_valid_o (m0u_rdata_valid),
        .txn_done_o    (m0_txn_done)
    );

    // ---------------------------------------------------------
    // Master 1's socket - shared, via m1_select_mux below, between a
    // second local master and slave_bridge relaying the remote board's
    // master.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 req_M1, grant_M1;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] addr_M1;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    wdata_M1;
    (* MARK_DEBUG = "TRUE" *) wire                 valid_M1;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    rdata_M1;
    (* MARK_DEBUG = "TRUE" *) wire                 ready_M1;
    (* MARK_DEBUG = "TRUE" *) wire                 rvalid_M1;

    // ---------------------------------------------------------
    // External bus port (master-role): drives master_bridge below.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 ext_req, ext_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] ext_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    ext_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    ext_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_rvalid;

    // ---------------------------------------------------------
    // Slaves
    // ---------------------------------------------------------
    wire slave_sel1, slave_sel2, slave_sel3;
    wire addr_invalid;
    wire [11:0]       addr_bus;
    wire [DATA_W-1:0] wdata_bus;
    wire              we_bus;
    wire              valid_bus;

    wire split, resume;

    system_bus #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .RW         (RW),
        .NUM_SLAVES (3)
    ) u_system_bus (
        .clk          (tick_4hz),
        .rst          (rst),

        .req_M0       (req_M0),
        .grant_M0     (grant_M0),
        .addr_M0      (addr_M0),
        .wdata_M0     (wdata_M0),
        .valid_M0     (valid_M0),
        .rdata_M0     (rdata_M0),
        .ready_M0     (ready_M0),
        .rvalid_M0    (rvalid_M0),

        .req_M1       (req_M1),
        .grant_M1     (grant_M1),
        .addr_M1      (addr_M1),
        .wdata_M1     (wdata_M1),
        .valid_M1     (valid_M1),
        .rdata_M1     (rdata_M1),
        .ready_M1     (ready_M1),
        .rvalid_M1    (rvalid_M1),

        .slave_sel1   (slave_sel1),
        .slave_sel2   (slave_sel2),
        .slave_sel3   (slave_sel3),
        .addr_invalid (addr_invalid),

        .addr_bus     (addr_bus),
        .wdata_bus    (wdata_bus),
        .we_bus       (we_bus),
        .valid_bus    (valid_bus),

        .rdata_S0     (rdata_S0),
        .ready_S0     (ready_S0),
        .rvalid_S0    (rvalid_S0),

        .rdata_S1     (rdata_S1),
        .ready_S1     (ready_S1),
        .rvalid_S1    (rvalid_S1),

        .rdata_S2     (rdata_S2),
        .ready_S2     (ready_S2),
        .rvalid_S2    (rvalid_S2),
        .split        (split),
        .resume       (resume),

        .addr_ext_o   (ext_addr),
        .wdata_ext_o  (ext_wdata),
        .ext_valid_o  (ext_valid),

        .rdata_ext_i  (ext_rdata),
        .ready_ext_i  (ext_ready),
        .rvalid_ext_i (ext_rvalid),

        .req_ext      (ext_req),
        .grant_ext    (ext_grant)
    );

    // ---------------------------------------------------------
    // Slave 0 (selected by slave_sel1)
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_S0;
    wire              ready_S0;
    wire              rvalid_S0;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave0 (
        .clk      (tick_4hz),
        .rst      (rst),
        .cs_i     (slave_sel1),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_S0),
        .ready_o  (ready_S0),
        .rvalid_o (rvalid_S0)
    );

    // ---------------------------------------------------------
    // Slave 1 (selected by slave_sel2)
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_S1;
    wire              ready_S1;
    wire              rvalid_S1;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave1 (
        .clk      (tick_4hz),
        .rst      (rst),
        .cs_i     (slave_sel2),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_S1),
        .ready_o  (ready_S1),
        .rvalid_o (rvalid_S1)
    );

    // ---------------------------------------------------------
    // Slave 2 (selected by slave_sel3) - split-transaction slave
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_S2;
    wire              ready_S2;
    wire              rvalid_S2;
    wire              split_S2;
    wire              resume_S2;

    slave_split #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave2 (
        .clk      (tick_4hz),
        .rst      (rst),
        .cs_i     (slave_sel3),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_S2),
        .ready_o  (ready_S2),
        .rvalid_o (rvalid_S2),
        .split_o  (split_S2),
        .resume_o (resume_S2)
    );

    assign split  = split_S2;
    assign resume = resume_S2;

    // ---------------------------------------------------------
    // For led_display below: each slave's own ready_o only pulses once,
    // when that slave (and no other) just completed a transaction (see
    // slave.v: ready_o <= 1'b1 only under sel = cs_i && valid_i), so the
    // three are naturally mutually exclusive - OR'ing them together, and
    // picking the matching rdata, needs no extra slave_sel/valid_bus
    // qualification.
    // ---------------------------------------------------------
    wire              sready = ready_S0 | ready_S1 | ready_S2;
    wire [DATA_W-1:0] rdata_any = ready_S0 ? rdata_S0 :
                                   ready_S1 ? rdata_S1 :
                                   ready_S2 ? rdata_S2 :
                                   {DATA_W{1'b0}};

    // Only one local master (system_bus's ext_* port) ever wants the
    // outbound serial link, so it can just grant itself (same trick
    // bus_interconnect_serial.v uses for its own per-bridge local arbiter).
    wire link_arb_req, link_arb_grant;
    assign link_arb_grant = link_arb_req;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge (
        .clk          (tick_4hz),
        .rst          (rst),

        .addr_ext_o   (ext_addr),
        .wdata_ext_o  (ext_wdata),
        .ext_valid_o  (ext_valid),
        .rdata_ext_i  (ext_rdata),
        .ready_ext_i  (ext_ready),
        .rvalid_ext_i (ext_rvalid),
        .req_ext      (ext_req),
        .grant_ext    (ext_grant),

        .ext_arb_req_o   (link_arb_req),
        .ext_arb_grant_i (link_arb_grant),

        // Physical serial link out to the other board
        .req_tx_o        (tx_req_o),
        .grant_rx_i      (rx_grant_i),
        .addr_tx_o       (tx_addr_o),
        .wdata_tx_o      (tx_wdata_o),
        .link_busy_o     (),
        .uart_rx_i       (rx_status_i),
        .link_rx_valid_o (),
        .ready_rx_i      (rx_ready_i)
    );

    // slave_bridge's own side of the shared Master 1 port (source B into
    // m1_select_mux below)
    (* MARK_DEBUG = "TRUE" *) wire                 sb_req, sb_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] sb_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    sb_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    sb_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge (
        .clk  (tick_4hz),
        .rst  (rst),

        .req_o    (sb_req),
        .grant_i  (sb_grant),
        .addr_o   (sb_addr),
        .wdata_o  (sb_wdata),
        .valid_o  (sb_valid),
        .rdata_i  (sb_rdata),
        .ready_i  (sb_ready),
        .rvalid_i (sb_rvalid),

        // Physical serial link in from the other board
        .req_rx_i       (rx_req_i),
        .grant_tx_o     (tx_grant_o),
        .addr_rx_i      (rx_addr_i),
        .wdata_rx_i     (rx_wdata_i),
        .link_rx_busy_o (),
        .rdata_tx_o     (tx_status_o),
        .link_tx_busy_o (),
        .ready_tx_o     (tx_ready_o)
    );

    // ---------------------------------------------------------
    // Second local master (source A into m1_select_mux below) - drives
    // this board's own local bus directly through the shared Master 1
    // port, same as the remote board's master does via slave_bridge.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_req, m1L_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] m1L_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1L_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1L_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_rvalid;

    wire                 m1_txn_valid, m1_txn_we, m1_txn_ready;
    wire [ADDR_W-1:0]    m1_txn_addr;
    wire [DATA_W-1:0]    m1_txn_wdata;
    wire [DATA_W-1:0]    m1u_rdata;
    wire                 m1u_rdata_valid, m1_txn_done;

    master_uart #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RW        (RW),
        .REQ_DELAY (4)
    ) u_master1 (
        .clk         (tick_4hz),
        .rst         (rst),
        .req_o       (m1L_req),
        .grant_i     (m1L_grant),
        .addr_o      (m1L_addr),
        .wdata_o     (m1L_wdata),
        .valid_o     (m1L_valid),
        .rdata_i     (m1L_rdata),
        .ready_i     (m1L_ready),
        .rvalid_i    (m1L_rvalid),
        .ext_valid_o (m1L_valid),

        .txn_valid_i   (m1_txn_valid),
        .txn_addr_i    (m1_txn_addr),
        .txn_we_i      (m1_txn_we),
        .txn_wdata_i   (m1_txn_wdata),
        .txn_ready_o   (m1_txn_ready),
        .rdata_o       (m1u_rdata),
        .rdata_valid_o (m1u_rdata_valid),
        .txn_done_o    (m1_txn_done)
    );

    // Both local masters' UART framing/transport in one place. See
    // uart_top.v's header for the request-frame layout (master_sel field
    // picks m0/m1/broadcast).
    UART_TOP #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W)
    ) u_uart_top (
        .clk         (tick_4hz),
        .rst         (rst),

        .i_Rx_Serial (uart_rx_serial),
        .o_Tx_Serial (uart_tx_serial),

        .m0_txn_valid_o   (m0_txn_valid),
        .m0_txn_addr_o    (m0_txn_addr),
        .m0_txn_we_o      (m0_txn_we),
        .m0_txn_wdata_o   (m0_txn_wdata),
        .m0_txn_ready_i   (m0_txn_ready),
        .m0_rdata_i       (m0u_rdata),
        .m0_rdata_valid_i (m0u_rdata_valid),
        .m0_txn_done_i    (m0_txn_done),

        .m1_txn_valid_o   (m1_txn_valid),
        .m1_txn_addr_o    (m1_txn_addr),
        .m1_txn_we_o      (m1_txn_we),
        .m1_txn_wdata_o   (m1_txn_wdata),
        .m1_txn_ready_i   (m1_txn_ready),
        .m1_rdata_i       (m1u_rdata),
        .m1_rdata_valid_i (m1u_rdata_valid),
        .m1_txn_done_i    (m1_txn_done)
    );

    // sel picks which of the two (local master1, or slave_bridge relaying
    // the remote board's master) drives system_bus's Master 1 socket -
    // 0 = local master1, 1 = slave_bridge (see m1_select_mux.v).
    // Hardwired here; swap to a real priority/arbiter signal if both need
    // to actually contend for the port.
    m1_select_mux #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_m1_select_mux (
        .sel (1'b0),

        .reqA    (m1L_req),
        .grantA  (m1L_grant),
        .addrA   (m1L_addr),
        .wdataA  (m1L_wdata),
        .validA  (m1L_valid),
        .rdataA  (m1L_rdata),
        .readyA  (m1L_ready),
        .rvalidA (m1L_rvalid),

        .reqB    (sb_req),
        .grantB  (sb_grant),
        .addrB   (sb_addr),
        .wdataB  (sb_wdata),
        .validB  (sb_valid),
        .rdataB  (sb_rdata),
        .readyB  (sb_ready),
        .rvalidB (sb_rvalid),

        .req_m1    (req_M1),
        .grant_m1  (grant_M1),
        .addr_m1   (addr_M1),
        .wdata_m1  (wdata_M1),
        .valid_m1  (valid_M1),
        .rdata_m1  (rdata_M1),
        .ready_m1  (ready_M1),
        .rvalid_m1 (rvalid_M1)
    );

    // Shows the lower 4 bits of whatever data last moved on this board's
    // own local bus - see led_display.v's header.
    led_display #(
        .DATA_W (DATA_W)
    ) u_led_display (
        .clk      (tick_4hz),
        .rst      (rst),
        .we_i     (we_bus),
        .valid_i  (valid_bus),
        .ready_i  (sready),
        .wdata_i  (wdata_bus),
        .rdata_i  (rdata_any),
        .led      (led)
    );

endmodule
