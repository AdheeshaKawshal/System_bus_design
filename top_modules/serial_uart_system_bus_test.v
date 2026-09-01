// serial_uart_system_bus_test: two serial_uart_system_bus boards, only
// partially cross-wired internally.
//
// - board_b's own master reaching outward (its outbound link) stays
//   wired straight into board_a's inbound (slave_bridge) port -
//   fully internal, no external pins needed for that direction.
// - board_a's outbound (master_bridge) port and board_b's inbound
//   (slave_bridge) port are instead brought straight out to real
//   top-level pins, so an external slave_bridge/master or board can be
//   wired in via GPIO for those two.
//
// Only board_a's PC-facing UART is exposed; board_b's is tied off
// internally and simply hangs (not reachable from outside at all).
module serial_uart_system_bus_test (
    input wire clk,
    input wire rst_n,

    // board_a's outbound serial link (its master_bridge), exposed
    // externally - an external slave_bridge/board plugs in here via
    // GPIO
    output wire tx_req_o_a,
    input  wire rx_grant_i_a,
    output wire tx_addr_o_a,
    output wire tx_wdata_o_a,
    input  wire rx_status_i_a,
    input  wire rx_ready_i_a,

    // board_b's inbound serial link (its slave_bridge), exposed
    // externally - an external master_bridge/board plugs in here via
    // GPIO
    input  wire rx_req_i_b,
    output wire tx_grant_o_b,
    input  wire rx_addr_i_b,
    input  wire rx_wdata_i_b,
    output wire tx_status_o_b,
    output wire tx_ready_o_b,

    // Board A - physical UART to a PC + status LED, via GPIO
    input  wire uart_rx_serial_a,
    output wire uart_tx_serial_a
);

    // ---------------------------------------------------------
    // Internal link: B's outbound -> A's inbound
    // ---------------------------------------------------------
    wire link_ba_req, link_ba_grant;
    wire link_ba_addr, link_ba_wdata, link_ba_status, link_ba_ready;

    serial_uart_system_bus board_a (
        .clk   (clk),
        .rst_n (rst_n),

        // Outbound (its master_bridge) - now real external pins
        .tx_req_o    (tx_req_o_a),
        .rx_grant_i  (rx_grant_i_a),
        .tx_addr_o   (tx_addr_o_a),
        .tx_wdata_o  (tx_wdata_o_a),
        .rx_status_i (rx_status_i_a),
        .rx_ready_i  (rx_ready_i_a),

        // Inbound (board_b's master reaching onto board_a) - internal
        .rx_req_i    (link_ba_req),
        .tx_grant_o  (link_ba_grant),
        .rx_addr_i   (link_ba_addr),
        .rx_wdata_i  (link_ba_wdata),
        .tx_status_o (link_ba_status),
        .tx_ready_o  (link_ba_ready),

        .uart_rx_serial (uart_rx_serial_a),
        .uart_tx_serial (uart_tx_serial_a)
    );

    serial_uart_system_bus board_b (
        .clk   (clk),
        .rst_n (rst_n),

        // Outbound (B's master reaching onto A) - internal
        .tx_req_o    (link_ba_req),
        .rx_grant_i  (link_ba_grant),
        .tx_addr_o   (link_ba_addr),
        .tx_wdata_o  (link_ba_wdata),
        .rx_status_i (link_ba_status),
        .rx_ready_i  (link_ba_ready),

        // Inbound (its slave_bridge) - now real external pins
        .rx_req_i    (rx_req_i_b),
        .tx_grant_o  (tx_grant_o_b),
        .rx_addr_i   (rx_addr_i_b),
        .rx_wdata_i  (rx_wdata_i_b),
        .tx_status_o (tx_status_o_b),
        .tx_ready_o  (tx_ready_o_b),

        // board_b's PC-facing UART just hangs - not reachable from
        // outside this module at all.
        .uart_rx_serial (1'b1),
        .uart_tx_serial ()
    );

endmodule
