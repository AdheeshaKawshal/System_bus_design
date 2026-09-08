module serial_bus_top #(
    parameter ADDR_W     = 15,
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3,
    parameter M0_START_TXN      = 1,
    parameter M0_REQ_DELAY      = 1000,
    parameter M0_WRITE_DELAY    = 26
)(
    input wire clk,
    input wire rst,

    output wire mc_uart_tx_o,  // this board's bb_master_core TX -> the other board's bb_slave_core RX
    input  wire mc_uart_rx_i,  // this board's bb_master_core RX <- the other board's bb_slave_core TX
    output wire sc_uart_tx_o,  // this board's bb_slave_core TX -> the other board's bb_master_core RX
    input  wire sc_uart_rx_i,  // this board's bb_slave_core RX <- the other board's bb_master_core TX

    input wire [3:0] m0_pkt_sel_i,
    input wire       pkt_valid_i,  // high when m0_pkt_sel_i is valid (see master.v's pkt_valid_i)
    // led_display's latched write/read + address[2:0] for the last frame
    // seen on the shared slave-side bus (see led_display.v).
    output wire [3:0] led_o

);
    wire rst_n = ~rst;
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
        .rst           (rst_n),
        .pkt_sel_i     (m0_pkt_sel_i),
        .pkt_valid_i   (pkt_valid_i),  // always valid, no "no selection" sentinel for this bus's Master 0
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

    bb_master_core #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_bb_master_core (
        .clk           (clk),
        .rst           (rst_n),

        .uart_rx_i     (mc_uart_rx_i),
        .uart_tx_o     (mc_uart_tx_o),

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
    // catches every frame a slave receives, S2's cross-board bridge
    // included.
    // ---------------------------------------------------------

    led_display #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_led_display (
        .clk          (clk),
        .rst          (rst_n),
        .addr_data_bus (addr_data_bus),
        .valid_bus     (valid_bus),
        .rdata_bus_ser (rdata_M0_ser),
        .rvalid_bus    (rvalid_M0 | rvalid_M1),
        .led_o         (led_o)
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
        .rst         (rst_n),
        .cs_i        (slave_sel1),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .rdata_o_ser (rdata_S0_ser),
        .rvalid_o    (rvalid_S0)
    );

    // ---------------------------------------------------------
    // Slave 1 (slave_sel2): split-capable. mready_i comes from mready_bus
    // ---------------------------------------------------------
    slave_split #(
        .ADDR_W      (12),
        .DATA_W      (DATA_W),
        .RW          (RW),
        .WAIT_CYCLES (10)
    ) u_slave1 (
        .clk         (clk),
        .rst         (rst_n),
        .cs_i        (slave_sel2),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .mready_i    (mready_bus),
        .rdata_o_ser (rdata_S1_ser),
        .rvalid_o    (rvalid_S1),
        .split_o     (split),
        .resume_o    (resume)
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
