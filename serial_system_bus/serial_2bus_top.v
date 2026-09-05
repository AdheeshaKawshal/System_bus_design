// serial_2bus_top: two independent serial_bus_top boards, meant to chain to
// each other back and forth over UART instead of each board's
// bb_master_core/bb_slave_core self-looping to itself.
//
// Bus 0's bb_master_core (its M1 slot) is meant to reach Bus 1's
// bb_slave_core (behind Bus 1's slave_sel3), and symmetrically Bus 1's
// bb_master_core reaches Bus 0's bb_slave_core - a genuine two-way bridge:
// a REMOTE (external-flagged / addr[14]=1) access on either board's
// slave_sel3 is serviced by the OTHER board's bus, not looped back onto
// its own.
//
// Each board's 4 UART pins (its bb_master_core's TX/RX and its
// bb_slave_core's TX/RX) are real top-level GPIOs here, not wired to each
// other internally - the actual bus0<->bus1 crossing happens externally
// (physical wires between the two boards' pins), matching two genuinely
// separate boards. Whoever connects these pins from outside is responsible
// for wiring bus0_mc_uart_tx_o -> bus1_sc_uart_rx_i, bus1_sc_uart_tx_o ->
// bus0_mc_uart_rx_i, bus1_mc_uart_tx_o -> bus0_sc_uart_rx_i, and
// bus0_sc_uart_tx_o -> bus1_mc_uart_rx_i (the same crossing this module
// used to wire internally).
//
// Each bus keeps its own Master 0 with its own independently configurable
// timing (M0_*_BUS0/M0_*_BUS1 below) - the two boards are not required to
// run identical arbitration/timeout parameters.
module serial_2bus_top #(
    parameter ADDR_W     = 15,
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3,

    // Bus 0's Master 0 timing (plus its starting transaction index). No
    // ACTIVE_TIMEOUT/BACKOFF_DELAY - master.v's read path now just waits
    // indefinitely for rvalid, no retry timeout.
    parameter M0_START_TXN_BUS0      = 0,
    parameter M0_REQ_DELAY_BUS0      = 100,
    parameter M0_WRITE_DELAY_BUS0    = 26,

    // Bus 1's Master 0 timing - independent of Bus 0's.
    parameter M0_START_TXN_BUS1      = 1,
    parameter M0_REQ_DELAY_BUS1      = 300,
    parameter M0_WRITE_DELAY_BUS1    = 26
)(
    input wire clk,
    input wire rst,     // active-high external reset - inverted below for the active-low internal modules

    // ---- Bus 0's UART pins - real GPIOs, connect from outside ----------
    output wire bus0_mc_uart_tx_o,
    input  wire bus0_mc_uart_rx_i,
    output wire bus0_sc_uart_tx_o,
    input  wire bus0_sc_uart_rx_i,

    // ---- Bus 1's UART pins - real GPIOs, connect from outside ----------
    output wire bus1_mc_uart_tx_o,
    input  wire bus1_mc_uart_rx_i,
    output wire bus1_sc_uart_tx_o,
    input  wire bus1_sc_uart_rx_i,

    // ---- link-activity LEDs: bus0's slave-tx/master-rx pair, and the
    // mirrored master-tx/slave-rx pair on bus1 -----------------------------
    output wire bus0_sc_tx_led_o,
    output wire bus0_mc_rx_led_o,
    output wire bus1_mc_tx_led_o,
    output wire bus1_sc_rx_led_o
);

    wire rst_n = ~rst;

    assign bus0_sc_tx_led_o = bus0_sc_uart_tx_o;
    assign bus0_mc_rx_led_o = bus0_mc_uart_rx_i;
    assign bus1_mc_tx_led_o = bus1_mc_uart_tx_o;
    assign bus1_sc_rx_led_o = bus1_sc_uart_rx_i;

    serial_bus_top #(
        .ADDR_W              (ADDR_W),
        .DATA_W              (DATA_W),
        .RW                  (RW),
        .NUM_SLAVES          (NUM_SLAVES),
        .M0_START_TXN        (M0_START_TXN_BUS0),
        .M0_REQ_DELAY        (M0_REQ_DELAY_BUS0),
        .M0_WRITE_DELAY      (M0_WRITE_DELAY_BUS0)
    ) u_bus0 (
        .clk         (clk),
        .rst         (rst_n),

        .mc_uart_tx_o (bus0_mc_uart_tx_o),
        .mc_uart_rx_i (bus0_mc_uart_rx_i),
        .sc_uart_tx_o (bus0_sc_uart_tx_o),
        .sc_uart_rx_i (bus0_sc_uart_rx_i)
    );

    serial_bus_top #(
        .ADDR_W              (ADDR_W),
        .DATA_W              (DATA_W),
        .RW                  (RW),
        .NUM_SLAVES          (NUM_SLAVES),
        .M0_START_TXN        (M0_START_TXN_BUS1),
        .M0_REQ_DELAY        (M0_REQ_DELAY_BUS1),
        .M0_WRITE_DELAY      (M0_WRITE_DELAY_BUS1)
    ) u_bus1 (
        .clk         (clk),
        .rst         (rst_n),

        .mc_uart_tx_o (bus1_mc_uart_tx_o),
        .mc_uart_rx_i (bus1_mc_uart_rx_i),
        .sc_uart_tx_o (bus1_sc_uart_tx_o),
        .sc_uart_rx_i (bus1_sc_uart_rx_i)
    );

endmodule
