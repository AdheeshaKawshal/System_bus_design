
// masterv2.v/slavev2.v's inner modules (used by bb_master_core.v/
// bb_slave_core.v) are named bb_master_txn_core/bb_local_regfile - renamed
// from their original master/slave to avoid colliding with this file's own
// master.v/slave.v (used below for u_master0/u_slave0/u_slave1).
module serial_bus_top #(
    parameter ADDR_W     = 15,
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3,

    // Master 0's own timing (plus its starting transaction index), exposed
    // so each bus instance (e.g. in serial_2bus_top.v) can configure its
    // Master 0 independently of the other bus's. No ACTIVE_TIMEOUT/
    // BACKOFF_DELAY here - master.v's read path now just waits
    // indefinitely for rvalid, no retry timeout.
    parameter M0_START_TXN      = 0,
    parameter M0_REQ_DELAY      = 0,
    parameter M0_WRITE_DELAY    = 26
)(
    input wire clk,
    input wire rst,

    // ---- UART links out to another board's serial_bus_top, for chaining
    // two buses together back and forth (see serial_2bus_top.v) - this
    // board's bb_master_core and bb_slave_core each get their own external
    // TX/RX pair instead of self-looping to each other internally.
    output wire mc_uart_tx_o,  // this board's bb_master_core TX -> the other board's bb_slave_core RX
    input  wire mc_uart_rx_i,  // this board's bb_master_core RX <- the other board's bb_slave_core TX
    output wire sc_uart_tx_o,  // this board's bb_slave_core TX -> the other board's bb_master_core RX
    input  wire sc_uart_rx_i   // this board's bb_slave_core RX <- the other board's bb_master_core TX
);

    // ---------------------------------------------------------
    // Master 0: plain, directly connected.
    // ---------------------------------------------------------
    wire req_M0, grant_M0, frame_valid_M0, mready_M0, rvalid_M0;
    wire addr_data_M0, rdata_M0_ser;

    master #(
        .ADDR_W         (ADDR_W),
        .DATA_W         (DATA_W),
        .RW             (RW),
        .START_TXN      (M0_START_TXN),
        .REQ_DELAY      (M0_REQ_DELAY),
        .WRITE_DELAY    (M0_WRITE_DELAY)
    ) u_master0 (
        .clk           (clk),
        .rst           (rst),
        .req_o         (req_M0),
        .grant_i       (grant_M0),
        .addr_data_o   (addr_data_M0),
        .frame_valid_o (frame_valid_M0),
        .mready_o      (mready_M0),
        .rdata_ser_i   (rdata_M0_ser),
        .rvalid_i      (rvalid_M0)
    );

    // ---------------------------------------------------------
    // Master 1's slot is bb_master_core.v - not a local master with its
    // own transaction table. It plugs into serial_system_bus's M1 port
    // with the same shape master.v uses; its UART side now goes out to
    // mc_uart_tx_o/mc_uart_rx_i instead of looping back to this board's own
    // bb_slave_core, so a request it issues onto M1 originates from
    // whatever the OTHER connected board's bb_slave_core relayed in over
    // that link (see serial_2bus_top.v for the actual cross-connection).
    // ---------------------------------------------------------
    wire req_M1, grant_M1, frame_valid_M1, mready_M1, rvalid_M1;
    wire addr_data_M1, rdata_M1_ser;

    bb_master_core #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_bb_master_core (
        .clk           (clk),
        .rst           (rst),

        .uart_rx_i     (mc_uart_rx_i),
        .uart_tx_o     (mc_uart_tx_o),

        .req_o         (req_M1),
        .grant_i       (grant_M1),
        .addr_data_o   (addr_data_M1),
        .frame_valid_o (frame_valid_M1),
        .mready_o      (mready_M1),
        .rdata_ser_i   (rdata_M1_ser),
        .rvalid_i      (rvalid_M1),

        .overflow_o    (),
        .frame_err_o   ()
    );

    // ---------------------------------------------------------
    // The bus itself.
    // ---------------------------------------------------------
    wire slave_sel1, slave_sel2, slave_sel3, ext_redirect, addr_invalid;
    wire addr_data_bus, valid_bus, mready_bus;
    wire rdata_S0_ser, rvalid_S0;
    wire rdata_S1_ser, rvalid_S1;
    wire rdata_S2_ser, rvalid_S2;
    wire split, resume;

    serial_system_bus #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .RW         (RW),
        .NUM_SLAVES (NUM_SLAVES)
    ) u_serial_system_bus (
        .clk (clk),
        .rst (rst),

        .req_M0         (req_M0),
        .grant_M0       (grant_M0),
        .addr_data_M0   (addr_data_M0),
        .frame_valid_M0 (frame_valid_M0),
        .rdata_M0_ser   (rdata_M0_ser),
        .mready_M0      (mready_M0),
        .rvalid_M0      (rvalid_M0),

        .req_M1         (req_M1),
        .grant_M1       (grant_M1),
        .addr_data_M1   (addr_data_M1),
        .frame_valid_M1 (frame_valid_M1),
        .rdata_M1_ser   (rdata_M1_ser),
        .mready_M1      (mready_M1),
        .rvalid_M1      (rvalid_M1),

        .slave_sel1   (slave_sel1),
        .slave_sel2   (slave_sel2),
        .slave_sel3   (slave_sel3),
        .ext_redirect (ext_redirect),
        .addr_invalid (addr_invalid),

        .addr_data_bus (addr_data_bus),
        .valid_bus     (valid_bus),
        .mready_bus    (mready_bus),

        .rdata_S0_ser (rdata_S0_ser),
        .rvalid_S0    (rvalid_S0),

        .rdata_S1_ser (rdata_S1_ser),
        .rvalid_S1    (rvalid_S1),

        .rdata_S2_ser (rdata_S2_ser),
        .rvalid_S2    (rvalid_S2),
        .split        (split),
        .resume       (resume)
    );

    // ---------------------------------------------------------
    // Slave 0 / Slave 1: plain.
    // ---------------------------------------------------------
    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave0 (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (slave_sel1),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S0_ser),
        .rvalid_o    (rvalid_S0)
    );

    // ---------------------------------------------------------
    // Slave 1 (slave_sel2): split-capable. mready_i comes from mready_bus
    // (the granted master's readiness, forwarded by the bus) so RESUME
    // knows when it's safe to send the parked master its response.
    // ---------------------------------------------------------
    slave_split #(
        .ADDR_W      (12),
        .DATA_W      (DATA_W),
        .RW          (RW),
        .WAIT_CYCLES (10)
    ) u_slave1 (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (slave_sel2),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .mready_i    (mready_bus),
        .rdata_o_ser (rdata_S1_ser),
        .rvalid_o    (rvalid_S1),
        .split_o     (split),
        .resume_o    (resume)
    );

    // ---------------------------------------------------------
    // Slave 2 (slave_sel3): dedicated to bb_slave_core - nothing else
    // shares this select anymore. ext_redirect is no longer consumed here;
    // bb_slave_core derives its own LOCAL/REMOTE split from bit 14 of the
    // forwarded address instead (the same bit addr_decoder used to route
    // here in the first place). Its UART side goes out to sc_uart_tx_o/
    // sc_uart_rx_i - the OTHER connected board's bb_master_core, not this
    // board's own (see serial_2bus_top.v).
    // ---------------------------------------------------------
    bb_slave_core u_bb_slave_core (
        .clk         (clk),
        .rst         (rst),

        .cs_i        (slave_sel3),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S2_ser),
        .rvalid_o    (rvalid_S2),

        .uart_tx_o   (sc_uart_tx_o),
        .uart_rx_i   (sc_uart_rx_i),

        .timeout_o   ()
    );

endmodule
