module bus_interconnect_serial (
    input wire clk_a,   // bus 1's FPGA   // bus 2's FPGA - independent oscillator, same nominal frequency
    input wire rst_n
);
    assign rst = !rst_n;
    localparam ADDR_W = 15;   // {internal_flag(1), slave_sel(2), slave_addr(12)}
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Bus 1 -> Bus 2 link: bus 1's ext_* port (master-role) drives a
    // master_bridge (clk_a domain). It first reserves bus 2 (req_tx/
    // grant_rx handshake with the remote slave_bridge), then, once
    // granted, serializes addr/wdata out over link12_* and gets
    // rdata+rvalid back the same way. The slave_bridge on the far end
    // (clk_b domain) reassembles that into bus 2's Master 1 port
    // (slave-role), so bus 1 becomes a second master on bus 2, over the
    // serial link instead of direct wires.
    //
    // req/grant/ready are all latency-sensitive single-bit handshakes, so
    // none of them ride the serialized addr/wdata/status words - each
    // gets its own dedicated line, held high for several cycles by its
    // sender (see STRETCH_CYCLES in both bridges) so the receiving side
    // can synchronize it directly onto its own clock with no separate
    // synchronizer module or second clock port needed here.
    // ---------------------------------------------------------
    wire link12_req_tx, link12_grant_tx;
    wire link12_addr_tx, link12_wdata_tx, link12_status_tx, link12_ready_tx;

    (* MARK_DEBUG = "TRUE" *) wire                 b1m_req, b1m_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] b1m_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    b1m_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 b1m_valid;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b1m_rdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b1m_ready;
    (* MARK_DEBUG = "TRUE" *)wire                 b1m_rvalid;
    (* MARK_DEBUG = "TRUE" *)wire                 b1_arb_req, b1_arb_grant;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge_b1 (
        .clk          (clk_a),
        .rst          (rst),

        // internal side (system_bus's ext_* port)
        .addr_ext_o   (b1m_addr),
        .wdata_ext_o  (b1m_wdata),
        .ext_valid_o  (b1m_valid),
        .rdata_ext_i  (b1m_rdata),
        .ready_ext_i  (b1m_ready),
        .rvalid_ext_i (b1m_rvalid),
        .req_ext      (b1m_req),
        .grant_ext    (b1m_grant),

        .ext_arb_req_o   (b1_arb_req),
        .ext_arb_grant_i (b1_arb_grant),

        // external side (serial link to u_slave_bridge_b2)
        .req_tx_o        (link12_req_tx),
        .grant_rx_i      (link12_grant_tx),
        .addr_tx_o       (link12_addr_tx),
        .wdata_tx_o      (link12_wdata_tx),
        .link_busy_o     (),
        .uart_rx_i       (link12_status_tx),
        .link_rx_valid_o (),
        .ready_rx_i      (link12_ready_tx)
    );
    assign b1_arb_grant = b1_arb_req;

    (* MARK_DEBUG = "TRUE" *) wire                 b2s_req, b2s_grant;
    (* MARK_DEBUG = "TRUE" *)wire [ADDR_W+RW-1:0] b2s_addr;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b2s_wdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b2s_valid;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b2s_rdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b2s_ready;
    (* MARK_DEBUG = "TRUE" *)wire                 b2s_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge_b2 (
        .clk  (clk_a),
        .rst  (rst),

        // internal side (bus 2's Master 1 port)
        .req_o    (b2s_req),
        .grant_i  (b2s_grant),
        .addr_o   (b2s_addr),
        .wdata_o  (b2s_wdata),
        .valid_o  (b2s_valid),
        .rdata_i  (b2s_rdata),
        .ready_i  (b2s_ready),
        .rvalid_i (b2s_rvalid),

        // external side (serial link to u_master_bridge_b1)
        .req_rx_i       (link12_req_tx),
        .grant_tx_o     (link12_grant_tx),
        .addr_rx_i      (link12_addr_tx),
        .wdata_rx_i     (link12_wdata_tx),
        .link_rx_busy_o (),
        .status_tx_o    (link12_status_tx),
        .link_tx_busy_o (),
        .ready_tx_o     (link12_ready_tx)
    );

    // ---------------------------------------------------------
    // Bus 2 -> Bus 1 link: same shape, mirrored. Bus 2's ext_* port drives
    // a master_bridge (clk_b domain) over link21_*, reassembled by a
    // slave_bridge (clk_a domain) into bus 1's Master 1 port.
    // ---------------------------------------------------------
    wire link21_req_tx, link21_grant_tx;
    wire link21_addr_tx, link21_wdata_tx, link21_status_tx, link21_ready_tx;

    (* MARK_DEBUG = "TRUE" *) wire                 b2m_req, b2m_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] b2m_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    b2m_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 b2m_valid;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b2m_rdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b2m_ready;
    (* MARK_DEBUG = "TRUE" *)wire                 b2m_rvalid;
    wire                 b2_arb_req, b2_arb_grant;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge_b2 (
        .clk          (clk_a),
        .rst          (rst),

        // internal side (system_bus's ext_* port)
        .addr_ext_o   (b2m_addr),
        .wdata_ext_o  (b2m_wdata),
        .ext_valid_o  (b2m_valid),
        .rdata_ext_i  (b2m_rdata),
        .ready_ext_i  (b2m_ready),
        .rvalid_ext_i (b2m_rvalid),
        .req_ext      (b2m_req),
        .grant_ext    (b2m_grant),
        .ext_arb_req_o   (b2_arb_req),
        .ext_arb_grant_i (b2_arb_grant),

        // external side (serial link to u_slave_bridge_b1)
        .req_tx_o        (link21_req_tx),
        .grant_rx_i      (link21_grant_tx),
        .addr_tx_o       (link21_addr_tx),
        .wdata_tx_o      (link21_wdata_tx),
        .link_busy_o     (),
        .uart_rx_i       (link21_status_tx),
        .link_rx_valid_o (),
        .ready_rx_i      (link21_ready_tx)
    );
    assign b2_arb_grant = b2_arb_req;

    (* MARK_DEBUG = "TRUE" *) wire                 b1s_req, b1s_grant;
    (* MARK_DEBUG = "TRUE" *)wire [ADDR_W+RW-1:0] b1s_addr;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b1s_wdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b1s_valid;
    (* MARK_DEBUG = "TRUE" *)wire [DATA_W-1:0]    b1s_rdata;
    (* MARK_DEBUG = "TRUE" *)wire                 b1s_ready;
    (* MARK_DEBUG = "TRUE" *)wire                 b1s_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge_b1 (
        .clk  (clk_a),
        .rst  (rst),

        // internal side (bus 1's Master 1 port)
        .req_o    (b1s_req),
        .grant_i  (b1s_grant),
        .addr_o   (b1s_addr),
        .wdata_o  (b1s_wdata),
        .valid_o  (b1s_valid),
        .rdata_i  (b1s_rdata),
        .ready_i  (b1s_ready),
        .rvalid_i (b1s_rvalid),

        // external side (serial link to u_master_bridge_b2)
        .req_rx_i       (link21_req_tx),
        .grant_tx_o     (link21_grant_tx),
        .addr_rx_i      (link21_addr_tx),
        .wdata_rx_i     (link21_wdata_tx),
        .link_rx_busy_o (),
        .status_tx_o    (link21_status_tx),
        .link_tx_busy_o (),
        .ready_tx_o     (link21_ready_tx)
    );

    // ---------------------------------------------------------
    // The two buses themselves - unchanged except their Master 1 /
    // external ports now terminate on bridges instead of each other's
    // wires directly. Each bus and its local pair of bridges live
    // entirely within one clock domain (bus 1 + its bridges on clk_a,
    // bus 2 + its bridges on clk_b) - only the serial link wires and the
    // req/grant/ready handshake lines cross between the two.
    // ---------------------------------------------------------
    system_busv1 u_bus1 (
        .clk          (clk_a),
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
        .clk          (clk_a),
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
