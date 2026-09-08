// serial_2bus_top: despite the name (kept so nothing referencing this file
// needs renaming), this is now a SINGLE bus with TWO local masters -
// master.v on both M0 and M1 - contending for the same bus/slaves. No
// bb_master_core/bb_slave_core UART bridging between two boards anymore;
// M1 is a genuine second local master with its own transaction table, not
// a relay for a remote board's requests. bb_slave_core is still present as
// the S3/ext_redirect slave (a plain 4th slave target for either master to
// address, unrelated to the old cross-board M1 bridging), still going out
// over its own sc_uart pins if you want to exercise that path.
module serial_2bus_top #(
    parameter ADDR_W     = 15,
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3,

    // Master 0's own timing (plus its starting transaction index).
    parameter M0_START_TXN   = 0,
    parameter M0_REQ_DELAY   = 100,
    parameter M0_WRITE_DELAY = 26,

    // Master 1's own timing - independent of Master 0's, so the two can be
    // driven with different starting points/pacing while contending for
    // the same bus.
    parameter M1_START_TXN   = 1,
    parameter M1_REQ_DELAY   = 100,
    parameter M1_WRITE_DELAY = 26
)(
    input wire clk,
    input wire rst,     // active-high external reset - inverted below for the active-low internal modules

    // ---- bb_slave_core's UART pins (S3/ext_redirect slave) - real GPIOs,
    // connect from outside if you want to exercise the external-bridge
    // path. Not required for plain two-local-master operation.
    output wire sc_uart_tx_o,
    input  wire sc_uart_rx_i,

    // External selection of which entry in each master's transaction table
    // to send next (see master.v's pkt_sel_i/pkt_valid_i).
    input wire [3:0] m0_pkt_sel_i,
    input wire       m0_pkt_valid_i,
    input wire [3:0] m1_pkt_sel_i,
    input wire       m1_pkt_valid_i,

    // led_display's latched write/read + address[2:0] for the last frame
    // seen on the shared slave-side bus (see led_display.v).
    output wire [3:0] led_o
);
    wire rst_n = ~rst;

    // ---------------------------------------------------------
    // Master 0 / Master 1: both plain master.v, each with its own
    // transaction table, contending for the same bus via the arbiter.
    // ---------------------------------------------------------
    wire req_M0, grant_M0, frame_valid_M0, mready_M0, rvalid_M0;
    wire addr_data_M0, rdata_M0_ser;

    master #(
        .ADDR_W      (ADDR_W),
        .DATA_W      (DATA_W),
        .RW          (RW),
        .START_TXN   (M0_START_TXN),
        .REQ_DELAY   (M0_REQ_DELAY),
        .WRITE_DELAY (M0_WRITE_DELAY)
    ) u_master0 (
        .clk           (clk),
        .rst           (rst_n),
        .pkt_sel_i     (m0_pkt_sel_i),
        .pkt_valid_i   (m0_pkt_valid_i),
        .req_o         (req_M0),
        .grant_i       (grant_M0),
        .addr_data_o   (addr_data_M0),
        .frame_valid_o (frame_valid_M0),
        .mready_o      (mready_M0),
        .rdata_ser_i   (rdata_M0_ser),
        .rvalid_i      (rvalid_M0)
    );

    wire req_M1, grant_M1, frame_valid_M1, mready_M1, rvalid_M1;
    wire addr_data_M1, rdata_M1_ser;

    master #(
        .ADDR_W      (ADDR_W),
        .DATA_W      (DATA_W),
        .RW          (RW),
        .START_TXN   (M1_START_TXN),
        .REQ_DELAY   (M1_REQ_DELAY),
        .WRITE_DELAY (M1_WRITE_DELAY)
    ) u_master1 (
        .clk           (clk),
        .rst           (rst_n),
        .pkt_sel_i     (m1_pkt_sel_i),
        .pkt_valid_i   (m1_pkt_valid_i),
        .req_o         (req_M1),
        .grant_i       (grant_M1),
        .addr_data_o   (addr_data_M1),
        .frame_valid_o (frame_valid_M1),
        .mready_o      (mready_M1),
        .rdata_ser_i   (rdata_M1_ser),
        .rvalid_i      (rvalid_M1)
    );

    // ---------------------------------------------------------
    // The bus itself.
    // ---------------------------------------------------------
    wire slave_sel1, slave_sel2, slave_sel3, ext_redirect, addr_invalid;
    wire addr_data_bus, valid_bus, mready_bus;
    wire rdata_S0_ser, rvalid_S0;
    wire rdata_S1_ser, rvalid_S1;
    wire rdata_S2_ser, rvalid_S2;
    wire rdata_S3_ser, rvalid_S3;
    wire split, resume;

    serial_system_bus #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .RW         (RW),
        .NUM_SLAVES (NUM_SLAVES)
    ) u_serial_system_bus (
        .clk (clk),
        .rst (rst_n),

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
        .resume       (resume),

        .rdata_S3_ser (rdata_S3_ser),
        .rvalid_S3    (rvalid_S3)
    );

    // ---------------------------------------------------------
    // LED display: watches the shared slave-side bus directly, so it
    // catches every frame either master sends.
    // ---------------------------------------------------------
    led_display #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_led_display (
        .clk           (clk),
        .rst           (rst_n),
        .addr_data_bus (addr_data_bus),
        .valid_bus     (valid_bus),
        .rdata_bus_ser (rdata_M0_ser),
        .rvalid_bus    (rvalid_M0 | rvalid_M1),
        .led_o         (led_o)
    );

    // ---------------------------------------------------------
    // Slave 0 (slave_sel1): split-capable.
    // ---------------------------------------------------------
    slave_split #(
        .ADDR_W      (12),
        .DATA_W      (DATA_W),
        .RW          (RW),
        .WAIT_CYCLES (10)
    ) u_slave0 (
        .clk         (clk),
        .rst         (rst_n),
        .cs_i        (slave_sel1),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .mready_i    (mready_bus),
        .rdata_o_ser (rdata_S0_ser),
        .rvalid_o    (rvalid_S0),
        .split_o     (split),
        .resume_o    (resume)
    );

    // ---------------------------------------------------------
    // Slave 1 (slave_sel2) / Slave 2 (slave_sel3): plain.
    // ---------------------------------------------------------
    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave1 (
        .clk         (clk),
        .rst         (rst_n),
        .cs_i        (slave_sel2),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S1_ser),
        .rvalid_o    (rvalid_S1)
    );

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave2 (
        .clk         (clk),
        .rst         (rst_n),
        .cs_i        (slave_sel3),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S2_ser),
        .rvalid_o    (rvalid_S2)
    );

    // ---------------------------------------------------------
    // External/bridge slave (ext_redirect, S3): unrelated to the old M1
    // bridging - just a plain 4th slave target either master can address.
    // ---------------------------------------------------------
    bb_slave_core u_bb_slave_core (
        .clk         (clk),
        .rst         (rst_n),

        .cs_i        (ext_redirect),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S3_ser),
        .rvalid_o    (rvalid_S3),

        .uart_tx_o   (sc_uart_tx_o),
        .uart_rx_i   (sc_uart_rx_i),

        .timeout_o   ()
    );

endmodule
