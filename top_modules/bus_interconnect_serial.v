module bus_interconnect_serial (
    input wire clk,
    input wire rst
);

    localparam ADDR_W = 15;   // {internal_flag(1), slave_sel(2), slave_addr(12)}
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Bus 1 -> Bus 2 link: bus 1's ext_* port (master-role) drives a
    // master_bridge, which serializes addr/wdata out over link12_* and
    // gets a status+rdata word back. A slave_bridge on the far end
    // reassembles that into bus 2's Master 1 port (slave-role), so bus 1
    // becomes a second master on bus 2, over the serial link instead of
    // direct wires.
    // ---------------------------------------------------------
    wire link12_addr_tx, link12_wdata_tx, link12_status_tx;

    wire                 b1m_req, b1m_grant;
    wire [ADDR_W+RW-1:0] b1m_addr;
    wire [DATA_W-1:0]    b1m_wdata;
    wire                 b1m_valid;
    wire [DATA_W-1:0]    b1m_rdata;
    wire                 b1m_ready;
    wire                 b1m_rvalid;
    wire                 b1_arb_req, b1_arb_grant;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge_b1 (
        .clk          (clk),
        .rst          (rst),

        // internal bus side (system_bus's ext_* port)
        .req_ext      (b1m_req),
        .grant_ext    (b1m_grant),
        .addr_ext_o   (b1m_addr),
        .wdata_ext_o  (b1m_wdata),
        .ext_valid_o  (b1m_valid),
        .rdata_ext_i  (b1m_rdata),
        .ready_ext_i  (b1m_ready),
        .rvalid_ext_i (b1m_rvalid),

        // external side (serial link + link arbiter)
        .addr_tx_o       (link12_addr_tx),
        .wdata_tx_o      (link12_wdata_tx),
        .link_busy_o     (),
        .uart_rx_i       (link12_status_tx),
        .link_rx_valid_o (),

        // only bus 1 uses this link in this direction: auto-grant
        .ext_arb_req_o   (b1_arb_req),
        .ext_arb_grant_i (b1_arb_grant)
    );
    assign b1_arb_grant = b1_arb_req;

    wire                 b2s_req, b2s_grant;
    wire [ADDR_W+RW-1:0] b2s_addr;
    wire [DATA_W-1:0]    b2s_wdata;
    wire                 b2s_valid;
    wire [DATA_W-1:0]    b2s_rdata;
    wire                 b2s_ready;
    wire                 b2s_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge_b2 (
        .clk  (clk),
        .rst  (rst),

        // internal bus side (bus 2's Master 1 port)
        .req_o    (b2s_req),
        .grant_i  (b2s_grant),
        .addr_o   (b2s_addr),
        .wdata_o  (b2s_wdata),
        .valid_o  (b2s_valid),
        .rdata_i  (b2s_rdata),
        .ready_i  (b2s_ready),
        .rvalid_i (b2s_rvalid),

        // external side (serial link)
        .addr_rx_i      (link12_addr_tx),
        .wdata_rx_i     (link12_wdata_tx),
        .link_rx_busy_o (),
        .status_tx_o    (link12_status_tx),
        .link_tx_busy_o ()
    );

    // ---------------------------------------------------------
    // Bus 2 -> Bus 1 link: same shape, mirrored. Bus 2's ext_* port drives
    // a master_bridge over link21_*, reassembled by a slave_bridge into
    // bus 1's Master 1 port.
    // ---------------------------------------------------------
    wire link21_addr_tx, link21_wdata_tx, link21_status_tx;

    wire                 b2m_req, b2m_grant;
    wire [ADDR_W+RW-1:0] b2m_addr;
    wire [DATA_W-1:0]    b2m_wdata;
    wire                 b2m_valid;
    wire [DATA_W-1:0]    b2m_rdata;
    wire                 b2m_ready;
    wire                 b2m_rvalid;
    wire                 b2_arb_req, b2_arb_grant;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge_b2 (
        .clk          (clk),
        .rst          (rst),

        // internal bus side (system_bus's ext_* port)
        .req_ext      (b2m_req),
        .grant_ext    (b2m_grant),
        .addr_ext_o   (b2m_addr),
        .wdata_ext_o  (b2m_wdata),
        .ext_valid_o  (b2m_valid),
        .rdata_ext_i  (b2m_rdata),
        .ready_ext_i  (b2m_ready),
        .rvalid_ext_i (b2m_rvalid),

        // external side (serial link + link arbiter)
        .addr_tx_o       (link21_addr_tx),
        .wdata_tx_o      (link21_wdata_tx),
        .link_busy_o     (),
        .uart_rx_i       (link21_status_tx),
        .link_rx_valid_o (),

        .ext_arb_req_o   (b2_arb_req),
        .ext_arb_grant_i (b2_arb_grant)
    );
    assign b2_arb_grant = b2_arb_req;

    wire                 b1s_req, b1s_grant;
    wire [ADDR_W+RW-1:0] b1s_addr;
    wire [DATA_W-1:0]    b1s_wdata;
    wire                 b1s_valid;
    wire [DATA_W-1:0]    b1s_rdata;
    wire                 b1s_ready;
    wire                 b1s_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge_b1 (
        .clk  (clk),
        .rst  (rst),

        // internal bus side (bus 1's Master 1 port)
        .req_o    (b1s_req),
        .grant_i  (b1s_grant),
        .addr_o   (b1s_addr),
        .wdata_o  (b1s_wdata),
        .valid_o  (b1s_valid),
        .rdata_i  (b1s_rdata),
        .ready_i  (b1s_ready),
        .rvalid_i (b1s_rvalid),

        // external side (serial link)
        .addr_rx_i      (link21_addr_tx),
        .wdata_rx_i     (link21_wdata_tx),
        .link_rx_busy_o (),
        .status_tx_o    (link21_status_tx),
        .link_tx_busy_o ()
    );

    // ---------------------------------------------------------
    // The two buses themselves - unchanged except their Master 1 /
    // external ports now terminate on bridges instead of each other's
    // wires directly.
    // ---------------------------------------------------------
    system_busv1 u_bus1 (
        .clk          (clk),
        .rst          (rst),

        // Master 1 port (slave-role): accepts bus 2, arriving over the
        // serial link via u_slave_bridge_b1
        .req_M1       (b1s_req),
        .grant_M1     (b1s_grant),
        .addr_M1      (b1s_addr),
        .wdata_M1     (b1s_wdata),
        .valid_M1     (b1s_valid),
        .rdata_M1     (b1s_rdata),
        .ready_M1     (b1s_ready),
        .rvalid_M1    (b1s_rvalid),

        // External bus port (master-role): drives u_master_bridge_b1
        .req_ext      (b1m_req),
        .grant_ext    (b1m_grant),
        .addr_ext_o   (b1m_addr),
        .wdata_ext_o  (b1m_wdata),
        .ext_valid_o  (b1m_valid),
        .rdata_ext_i  (b1m_rdata),
        .ready_ext_i  (b1m_ready),
        .rvalid_ext_i (b1m_rvalid)
    );

    system_busv2 u_bus2 (
        .clk          (clk),
        .rst          (rst),

        // Master 1 port (slave-role): accepts bus 1, arriving over the
        // serial link via u_slave_bridge_b2
        .req_M1       (b2s_req),
        .grant_M1     (b2s_grant),
        .addr_M1      (b2s_addr),
        .wdata_M1     (b2s_wdata),
        .valid_M1     (b2s_valid),
        .rdata_M1     (b2s_rdata),
        .ready_M1     (b2s_ready),
        .rvalid_M1    (b2s_rvalid),

        // External bus port (master-role): drives u_master_bridge_b2
        .req_ext      (b2m_req),
        .grant_ext    (b2m_grant),
        .addr_ext_o   (b2m_addr),
        .wdata_ext_o  (b2m_wdata),
        .ext_valid_o  (b2m_valid),
        .rdata_ext_i  (b2m_rdata),
        .ready_ext_i  (b2m_ready),
        .rvalid_ext_i (b2m_rvalid)
    );

endmodule
