module system_bus #(
    parameter ADDR_W     = 15,  // {internal_flag(1), slave_sel(2), slave_addr(12)}
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3
)(
    input wire clk,
    input wire rst,

    // ---------------------------------------------------------
    // Arbiter interface
    // ---------------------------------------------------------
    input wire req_M0,
    input wire req_M1,
    input wire split,
    input wire resume,

    output wire grant_M0,
    output wire grant_M1,

    // ---------------------------------------------------------
    // Master 0 bus-side signals (addr_M0 LSB carries we; ADDR_W+RW wide)
    // ---------------------------------------------------------
    input  wire [ADDR_W+RW-1:0] addr_M0,
    input  wire [DATA_W-1:0]    wdata_M0,
    input  wire                 valid_M0,
    output wire [DATA_W-1:0]    rdata_M0,
    output wire                 ready_M0,

    // ---------------------------------------------------------
    // Master 1 bus-side signals (addr_M1 LSB carries we; ADDR_W+RW wide)
    // ---------------------------------------------------------
    input  wire [ADDR_W+RW-1:0] addr_M1,
    input  wire [DATA_W-1:0]    wdata_M1,
    input  wire                 valid_M1,
    output wire [DATA_W-1:0]    rdata_M1,
    output wire                 ready_M1,

    // ---------------------------------------------------------
    // Decoded/muxed bus toward the slaves
    // ---------------------------------------------------------
    output wire slave_sel1,
    output wire slave_sel2,
    output wire slave_sel3,
    output wire addr_invalid,

    output wire [ADDR_W-2-2:0] addr_bus,   // trimmed slave address (drops internal-flag + sel bits)
    output wire [DATA_W-1:0]   wdata_bus,
    output wire                we_bus,
    output wire                valid_bus,  // ctr line: master -> slave

    // ---------------------------------------------------------
    // Slave -> master return path (read data + ready), broadcast to
    // both masters; each one only acts on it while it holds the grant.
    // ---------------------------------------------------------
    input  wire [DATA_W-1:0] rdata_slave,
    input  wire              ready_slave,

    // ---------------------------------------------------------
    // Off-bus (external) forwarding
    // ---------------------------------------------------------
    output wire [ADDR_W+RW-1:0] addr_ext_o,
    output wire                 ext_valid_o
);

    wire addr_sel, data_sel, ctr_sel;

    arbiter u_arbiter (
        .clk      (clk),
        .rst      (rst),
        .req_M0   (req_M0),
        .req_M1   (req_M1),
        .split    (split),
        .resume   (resume),
        .grant_M0 (grant_M0),
        .grant_M1 (grant_M1),
        .addr_sel (addr_sel),
        .data_sel (data_sel),
        .ctr_sel  (ctr_sel)
    );

    // ---------------------------------------------------------
    // Addr / data / control muxes: pick the granted master's signals
    // ---------------------------------------------------------
    wire [ADDR_W+RW-1:0] addr_mux  = addr_sel ? addr_M1  : addr_M0;   // {addr, we}
    wire [DATA_W-1:0]    wdata_mux = data_sel ? wdata_M1 : wdata_M0;
    wire                 valid_mux = ctr_sel  ? valid_M1 : valid_M0;

    assign wdata_bus = wdata_mux;
    assign we_bus     = addr_mux[0];
    assign addr_bus   = addr_mux[ADDR_W-2-2+RW:RW];

    // ctr bus: valid (master->slave) muxed above; ready (slave->master)
    // is broadcast back to both masters below.
    assign valid_bus = slave_sel1 | slave_sel2 | slave_sel3;

    // Read data / ready return path: broadcast to both masters. Only the
    // one currently holding the grant will actually be waiting on it.
    assign rdata_M0 = rdata_slave;
    assign rdata_M1 = rdata_slave;
    assign ready_M0 = ready_slave;
    assign ready_M1 = ready_slave;

    addr_decoder #(
        .ADDR_W     (ADDR_W),
        .NUM_SLAVES (NUM_SLAVES),
        .RW         (RW)
    ) u_addr_decoder (
        .addr_i      (addr_mux),
        .valid_i     (valid_mux),
        .slave_sel1  (slave_sel1),
        .slave_sel2  (slave_sel2),
        .slave_sel3  (slave_sel3),
        .addr_o      (addr_bus),
        .we          (we_bus),
        
        .addr_ext_o  (addr_ext_o),
        .ext_valid_o (ext_valid_o),
        .addr_invalid(addr_invalid)
    );

endmodule
