// serial_bus_top: top-level integration for this folder - wires the 2
// masters, the bus (serial_system_bus.v), and the 3 slaves together for
// simulation. serial_system_bus.v itself is untouched by this file; it's
// just instantiated like any other submodule.
//
// Each physical slave select is dedicated to exactly one device: slave_sel1
// -> a plain slave (slave.v), slave_sel2 -> a split-capable slave
// (slave_split.v), slave_sel3 -> slave_bridge.v. slave_bridge still
// receives ext_redirect as a dedicated informational line (see
// addr_decoder.v/slave_bridge.v) even though it no longer shares its select
// with anything. Master 1's physical port is shared, via m1_select_mux,
// between a local master instance (source A) and slave_bridge acting as a
// master when it has a relay in flight (source B) - so a bridged request
// loops back onto this very bus through Master 1, letting the whole bridge
// path be exercised in simulation without a second physical board. Fixed
// priority (m1_sel = reqB) hands the shared port to the bridge whenever it
// has work pending, otherwise to the local master.
module serial_bus_top #(
    parameter ADDR_W     = 15,
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3
)(
    input wire clk,
    input wire rst
);

    // ---------------------------------------------------------
    // Master 0: plain, directly connected.
    // ---------------------------------------------------------
    wire req_M0, grant_M0, frame_valid_M0, mready_M0, rvalid_M0;
    wire addr_data_M0, rdata_M0_ser;

    master #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
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
    // Master 1: shared between a local master (source A) and slave_bridge
    // acting as a master when relaying a bridged request (source B).
    // ---------------------------------------------------------
    wire req_M1, grant_M1, frame_valid_M1, mready_M1, rvalid_M1;
    wire addr_data_M1, rdata_M1_ser;

    wire reqA, grantA, frame_valid_A, mreadyA, rvalidA, addr_data_A, rdata_A_ser;
    wire reqB, grantB, frame_valid_B, mreadyB, rvalidB, addr_data_B, rdata_B_ser;

    wire m1_sel = reqB;

    master #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master1 (
        .clk           (clk),
        .rst           (rst),
        .req_o         (reqA),
        .grant_i       (grantA),
        .addr_data_o   (addr_data_A),
        .frame_valid_o (frame_valid_A),
        .mready_o      (mreadyA),
        .rdata_ser_i   (rdata_A_ser),
        .rvalid_i      (rvalidA)
    );

    m1_select_mux u_m1_select_mux (
        .sel (m1_sel),

        .reqA          (reqA),
        .grantA        (grantA),
        .addr_data_A   (addr_data_A),
        .frame_valid_A (frame_valid_A),
        .rdata_A_ser   (rdata_A_ser),
        .mreadyA       (mreadyA),
        .rvalidA       (rvalidA),

        .reqB          (reqB),
        .grantB        (grantB),
        .addr_data_B   (addr_data_B),
        .frame_valid_B (frame_valid_B),
        .rdata_B_ser   (rdata_B_ser),
        .mreadyB       (mreadyB),
        .rvalidB       (rvalidB),

        .req_m1         (req_M1),
        .grant_m1       (grant_M1),
        .addr_data_m1   (addr_data_M1),
        .frame_valid_m1 (frame_valid_M1),
        .rdata_m1_ser   (rdata_M1_ser),
        .mready_m1      (mready_M1),
        .rvalid_m1      (rvalid_M1)
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
    // Slave 2 (slave_sel3): dedicated to slave_bridge - nothing else
    // shares this select anymore.
    // ---------------------------------------------------------
    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave2 (
        .clk            (clk),
        .rst            (rst),
        .cs_i           (slave_sel3),
        .ext_redirect_i (ext_redirect),
        .addr_data_i    (addr_data_bus),
        .valid_i        (valid_bus),
        .rdata_o_ser    (rdata_S2_ser),
        .rvalid_o       (rvalid_S2),

        .reqB          (reqB),
        .grantB        (grantB),
        .addr_data_B   (addr_data_B),
        .frame_valid_B (frame_valid_B),
        .rdata_B_ser   (rdata_B_ser),
        .mreadyB       (mreadyB),
        .rvalidB       (rvalidB)
    );

endmodule
