module bus_interconnect (
    input wire clk,
    input wire rst
);

    localparam ADDR_W = 15;   // {internal_flag(1), slave_sel(2), slave_addr(12)}
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Bus 1 -> Bus 2 link: bus 1's ext_* port (MASTER-role signals -
    // this is the actual master driving req/addr/wdata/valid) is
    // cross-connected into bus 2's Master 1 port (SLAVE-role signals -
    // the socket that accepts an external master's req/addr/wdata/valid
    // and returns grant/rdata/ready/rvalid). So bus 1 becomes a second
    // master directly on bus 2.
    // ---------------------------------------------------------
    wire                 b1m_req;      // master(bus1) -> slave(bus2): request
    wire                 b1m_grant;    // slave(bus2)  -> master(bus1): grant
    wire [ADDR_W+RW-1:0] b1m_addr;     // master(bus1) -> slave(bus2): {addr, we}
    wire [DATA_W-1:0]    b1m_wdata;    // master(bus1) -> slave(bus2): write data
    wire                 b1m_valid;    // master(bus1) -> slave(bus2): valid
    wire [DATA_W-1:0]    b1m_rdata;    // slave(bus2)  -> master(bus1): read data
    wire                 b1m_ready;    // slave(bus2)  -> master(bus1): ready
    wire                 b1m_rvalid;   // slave(bus2)  -> master(bus1): rvalid

    // ---------------------------------------------------------
    // Bus 2 -> Bus 1 link: bus 2's ext_* port (MASTER-role signals) is
    // cross-connected into bus 1's Master 1 port (SLAVE-role signals),
    // so bus 2 becomes a second master directly on bus 1 as well. The
    // link is bidirectional overall: each bus can drive the other's
    // slaves through its Master 1 socket.
    // ---------------------------------------------------------
    wire                 b2m_req;      // master(bus2) -> slave(bus1): request
    wire                 b2m_grant;    // slave(bus1)  -> master(bus2): grant
    wire [ADDR_W+RW-1:0] b2m_addr;     // master(bus2) -> slave(bus1): {addr, we}
    wire [DATA_W-1:0]    b2m_wdata;    // master(bus2) -> slave(bus1): write data
    wire                 b2m_valid;    // master(bus2) -> slave(bus1): valid
    wire [DATA_W-1:0]    b2m_rdata;    // slave(bus1)  -> master(bus2): read data
    wire                 b2m_ready;    // slave(bus1)  -> master(bus2): ready
    wire                 b2m_rvalid;   // slave(bus1)  -> master(bus2): rvalid

    system_busv1 u_bus1 (
        .clk          (clk),
        .rst          (rst),

        // Master 1 port (slave-role): accepts bus 2 as an external master
        .req_M1       (b2m_req),
        .grant_M1     (b2m_grant),
        .addr_M1      (b2m_addr),
        .wdata_M1     (b2m_wdata),
        .valid_M1     (b2m_valid),
        .rdata_M1     (b2m_rdata),
        .ready_M1     (b2m_ready),
        .rvalid_M1    (b2m_rvalid),

        // External bus port (master-role): drives bus 2's Master 1 port
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

        // Master 1 port (slave-role): accepts bus 1 as an external master
        .req_M1       (b1m_req),
        .grant_M1     (b1m_grant),
        .addr_M1      (b1m_addr),
        .wdata_M1     (b1m_wdata),
        .valid_M1     (b1m_valid),
        .rdata_M1     (b1m_rdata),
        .ready_M1     (b1m_ready),
        .rvalid_M1    (b1m_rvalid),

        // External bus port (master-role): drives bus 1's Master 1 port
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
