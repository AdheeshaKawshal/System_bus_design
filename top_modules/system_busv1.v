module system_busv1 (
    input wire clk,
    input wire rst,

    // ---------------------------------------------------------
    // Master 1 port (SLAVE-role connection point): the socket where an
    // *external* master plugs into this bus in place of an internally
    // instantiated master. Directions mirror system_bus's own Master 1
    // port, i.e. from the outside this looks like a slave input: the
    // external master drives req/addr/wdata/valid in, this bus drives
    // grant/rdata/ready/rvalid back out.
    // ---------------------------------------------------------
    input  wire                req_M1,
    output wire                grant_M1,
    input  wire [15:0]         addr_M1,    // {addr, we}, ADDR_W+RW wide
    input  wire [7:0]          wdata_M1,
    input  wire                valid_M1,
    output wire [7:0]          rdata_M1,
    output wire                ready_M1,
    output wire                rvalid_M1,

    // ---------------------------------------------------------
    // External bus port (MASTER-role connection point): lets this
    // design act as a master on another bus. This bus drives
    // req/addr/wdata/valid out and consumes grant/rdata/ready/rvalid
    // back in, so it plugs directly into another bus's Master 1 port.
    // ---------------------------------------------------------
    output wire req_ext,
    input  wire grant_ext,

    output wire [15:0] addr_ext_o,     // {addr, we}, ADDR_W+RW wide
    output wire [7:0]  wdata_ext_o,
    output wire        ext_valid_o,

    input  wire [7:0]  rdata_ext_i,
    input  wire        ready_ext_i,
    input  wire        rvalid_ext_i,
    output wire           sready,
    output wire [11:0]    addr_m,
    output wire           we_m,
    output wire           valid_m,
    output wire [7:0]     wdata_m,
    output wire [7:0]     rdata_m
);

    localparam ADDR_W = 15;   // {internal_flag(1), slave_sel(2), slave_addr(12)}
    localparam DATA_W = 8;
    localparam RW     = 1;

    // Bus control: driven by slave3 (slave_split), the only slave that
    // ever needs to park the granted master mid-transaction.
    wire split, resume;

    // ---------------------------------------------------------
    // Master 0
    // ---------------------------------------------------------
    wire                  req_M0, grant_M0;
    wire [ADDR_W+RW-1:0]  addr_o_M0;   // {addr, we} packed by the master itself
    wire [DATA_W-1:0]     wdata_o_M0;
    wire                  valid_o_M0;
    wire [DATA_W-1:0]     rdata_i_M0;
    wire                  ready_i_M0;
    wire                  rvalid_i_M0;

    master #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RW        (RW),
        .REQ_DELAY       (40),
        .ACTIVE_TIMEOUT  (16),
        .BACKOFF_DELAY   (5)
    ) u_master0 (
        .clk     (clk),
        .rst     (rst),
        .req_o   (req_M0),
        .grant_i (grant_M0),
        .addr_o  (addr_o_M0),
        .wdata_o (wdata_o_M0),
        .valid_o (valid_o_M0),
        .rdata_i (rdata_i_M0),
        .ready_i (ready_i_M0),
        .rvalid_i(rvalid_i_M0),
        .ext_valid_o(ext_valid_o)
    );

    // ---------------------------------------------------------
    // System bus (arbiter + addr_decoder + muxes)
    // ---------------------------------------------------------
    wire slave_sel1, slave_sel2, slave_sel3;
    wire addr_invalid;
    wire [11:0]       addr_bus;
    wire [DATA_W-1:0] wdata_bus;
    wire              we_bus;
    wire              valid_bus;
    system_bus #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .RW         (RW),
        .NUM_SLAVES (3)
    ) u_system_bus (
        .clk          (clk),
        .rst          (rst),

        .req_M0       (req_M0),
        .grant_M0     (grant_M0),
        .addr_M0      (addr_o_M0),
        .wdata_M0     (wdata_o_M0),
        .valid_M0     (valid_o_M0),
        .rdata_M0     (rdata_i_M0),
        .ready_M0     (ready_i_M0),
        .rvalid_M0    (rvalid_i_M0),

        .req_M1       (req_M1),
        .grant_M1     (grant_M1),
        .addr_M1      (addr_M1),
        .wdata_M1     (wdata_M1),
        .valid_M1     (valid_M1),
        .rdata_M1     (rdata_M1),
        .ready_M1     (ready_M1),
        .rvalid_M1    (rvalid_M1),

        .slave_sel1   (slave_sel1),
        .slave_sel2   (slave_sel2),
        .slave_sel3   (slave_sel3),
        .addr_invalid (addr_invalid),

        .addr_bus     (addr_bus),
        .wdata_bus    (wdata_bus),
        .we_bus       (we_bus),
        .valid_bus    (valid_bus),

        .rdata_S0     (rdata_o_S0),
        .ready_S0     (ready_o_S0),
        .rvalid_S0    (rvalid_o_S0),

        .rdata_S1     (rdata_o_S1),
        .ready_S1     (ready_o_S1),
        .rvalid_S1    (rvalid_o_S1),

        .rdata_S2     (rdata_o_S2),
        .ready_S2     (ready_o_S2),
        .rvalid_S2    (rvalid_o_S2),
        .split        (split),
        .resume       (resume),

        .addr_ext_o   (addr_ext_o),
        .wdata_ext_o  (wdata_ext_o),
        .ext_valid_o  (ext_valid_o),

        .rdata_ext_i  (rdata_ext_i),
        .ready_ext_i  (ready_ext_i),
        .rvalid_ext_i (rvalid_ext_i),

        .req_ext      (req_ext),
        .grant_ext    (grant_ext)
    );

    // ---------------------------------------------------------
    // Slave 0 (selected by slave_sel1)
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_o_S0;
    wire              ready_o_S0;
    wire              rvalid_o_S0;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave0 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel1),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_o_S0),
        .ready_o  (ready_o_S0),
        .rvalid_o (rvalid_o_S0)
    );

    // ---------------------------------------------------------
    // Slave 1 (selected by slave_sel2)
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_o_S1;
    wire              ready_o_S1;
    wire              rvalid_o_S1;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave1 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel2),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_o_S1),
        .ready_o  (ready_o_S1),
        .rvalid_o (rvalid_o_S1)
    );

    // ---------------------------------------------------------
    // Slave 2 (selected by slave_sel3) - split-transaction slave
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_o_S2;
    wire              ready_o_S2;
    wire              rvalid_o_S2;
    wire              split_o_S2;
    wire              resume_o_S2;

    slave_split #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave2 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel3),
        .valid_i  (valid_bus),
        .we_i     (we_bus),
        .addr_i   (addr_bus),
        .wdata_i  (wdata_bus),
        .rdata_o  (rdata_o_S2),
        .ready_o  (ready_o_S2),
        .rvalid_o (rvalid_o_S2),
        .split_o  (split_o_S2),
        .resume_o (resume_o_S2)
    );

    assign split  = split_o_S2;
    assign resume = resume_o_S2;
    assign sready = ready_o_S2 || ready_o_S1 || ready_o_S0;  // OR of the local slaves' own ready pulses
    assign addr_m  = addr_bus;
    assign we_m    = we_bus;
    assign valid_m = valid_bus;
    assign wdata_m = wdata_bus;
    // Whichever local slave's ready just pulsed (see slave.v: ready_o only
    // pulses under that slave's own cs_i && valid_i) is the one whose read
    // data is actually meaningful right now - same mutual-exclusion sready
    // already relies on.
    assign rdata_m = ready_o_S0 ? rdata_o_S0 :
                      ready_o_S1 ? rdata_o_S1 :
                      ready_o_S2 ? rdata_o_S2 :
                      {DATA_W{1'b0}};

endmodule
