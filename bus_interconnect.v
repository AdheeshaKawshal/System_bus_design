module bus_interconnect (
    input wire clk,
    input wire rst
);

    localparam ADDR_W = 15;   // {internal_flag(1), slave_sel(2), slave_addr(12)}
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Bus 1: the existing top_module design. Its ext_* port no longer
    // goes straight outward — it plugs in as Master 1 on bus 2 below.
    // ---------------------------------------------------------
    wire                 req_ext_bus1, grant_ext_bus1;
    wire [ADDR_W+RW-1:0] addr_ext_bus1;
    wire [DATA_W-1:0]    wdata_ext_bus1;
    wire                 valid_ext_bus1;
    wire [DATA_W-1:0]    rdata_ext_bus1;
    wire                 ready_ext_bus1;
    wire                 rvalid_ext_bus1;

    top_module u_bus1 (
        .clk          (clk),
        .rst          (rst),

        .req_ext      (req_ext_bus1),
        .grant_ext    (grant_ext_bus1),

        .addr_ext_o   (addr_ext_bus1),
        .wdata_ext_o  (wdata_ext_bus1),
        .ext_valid_o  (valid_ext_bus1),

        .rdata_ext_i  (rdata_ext_bus1),
        .ready_ext_i  (ready_ext_bus1),
        .rvalid_ext_i (rvalid_ext_bus1)
    );

    // ---------------------------------------------------------
    // Bus 2: a fresh system_bus with its own Master 0, and bus 1's
    // ext_* port wired in as Master 1. Split/resume unused here since
    // none of bus 2's slaves are split-capable.
    // ---------------------------------------------------------
    wire req_M0_b2, grant_M0_b2;
    wire [ADDR_W+RW-1:0] addr_o_M0_b2;
    wire [DATA_W-1:0]    wdata_o_M0_b2;
    wire                 valid_o_M0_b2;
    wire [DATA_W-1:0]    rdata_i_M0_b2;
    wire                 ready_i_M0_b2;
    wire                 rvalid_i_M0_b2;

    master #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RW        (RW),
        .REQ_DELAY (26)
    ) u_masterA_b2 (
        .clk        (clk),
        .rst        (rst),
        .req_o      (req_M0_b2),
        .grant_i    (grant_M0_b2),
        .addr_o     (addr_o_M0_b2),
        .wdata_o    (wdata_o_M0_b2),
        .valid_o    (valid_o_M0_b2),
        .rdata_i    (rdata_i_M0_b2),
        .ready_i    (ready_i_M0_b2),
        .rvalid_i   (rvalid_i_M0_b2),
        .ext_valid_o(ext_valid_o_b2)
    );

    wire slave_sel1_b2, slave_sel2_b2, slave_sel3_b2;
    wire addr_invalid_b2;
    wire [11:0]       addr_bus_b2;
    wire [DATA_W-1:0] wdata_bus_b2;
    wire              we_bus_b2;
    wire              valid_bus_b2;
    wire [DATA_W-1:0] rdata_slave_b2;
    wire              ready_slave_b2;
    wire              rvalid_slave_b2;
    wire              ext_valid_o_b2;

    // Bus 2 has nothing further to chain to: its own ext_* port is left
    // dangling on the output side, and permanently un-granted on input.
    wire                 req_ext_b2;
    wire [ADDR_W+RW-1:0] addr_ext_o_b2;
    wire [DATA_W-1:0]    wdata_ext_o_b2;

    system_bus #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .RW         (RW),
        .NUM_SLAVES (3)
    ) u_system_bus2 (
        .clk          (clk),
        .rst          (rst),

        // Master 0: a plain local master
        .req_M0       (req_M0_b2),
        .grant_M0     (grant_M0_b2),
        .addr_M0      (addr_o_M0_b2),
        .wdata_M0     (wdata_o_M0_b2),
        .valid_M0     (valid_o_M0_b2),
        .rdata_M0     (rdata_i_M0_b2),
        .ready_M0     (ready_i_M0_b2),
        .rvalid_M0    (rvalid_i_M0_b2),

        // Master 1: bus 1, connected through its ext_* port
        .req_M1       (req_ext_bus1),
        .grant_M1     (grant_ext_bus1),
        .addr_M1      (addr_ext_bus1),
        .wdata_M1     (wdata_ext_bus1),
        .valid_M1     (valid_ext_bus1),
        .rdata_M1     (rdata_ext_bus1),
        .ready_M1     (ready_ext_bus1),
        .rvalid_M1    (rvalid_ext_bus1),

        .split        (1'b0),
        .resume       (1'b0),

        .slave_sel1   (slave_sel1_b2),
        .slave_sel2   (slave_sel2_b2),
        .slave_sel3   (slave_sel3_b2),
        .addr_invalid (addr_invalid_b2),

        .addr_bus     (addr_bus_b2),
        .wdata_bus    (wdata_bus_b2),
        .we_bus       (we_bus_b2),
        .valid_bus    (valid_bus_b2),

        .rdata_slave  (rdata_slave_b2),
        .ready_slave  (ready_slave_b2),
        .rvalid_slave (rvalid_slave_b2),

        .addr_ext_o   (addr_ext_o_b2),
        .wdata_ext_o  (wdata_ext_o_b2),
        .ext_valid_o  (ext_valid_o_b2),

        .rdata_ext_i  ({DATA_W{1'b0}}),
        .ready_ext_i  (1'b0),
        .rvalid_ext_i (1'b0),

        .req_ext      (req_ext_b2),
        .grant_ext    (1'b0)
    );

    // ---------------------------------------------------------
    // Bus 2's 3 slaves — a separate address space from bus 1's slaves.
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_o_S0_b2;
    wire              ready_o_S0_b2;
    wire              rvalid_o_S0_b2;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave0_b2 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel1_b2),
        .valid_i  (valid_bus_b2),
        .we_i     (we_bus_b2),
        .addr_i   (addr_bus_b2),
        .wdata_i  (wdata_bus_b2),
        .rdata_o  (rdata_o_S0_b2),
        .ready_o  (ready_o_S0_b2),
        .rvalid_o (rvalid_o_S0_b2)
    );

    wire [DATA_W-1:0] rdata_o_S1_b2;
    wire              ready_o_S1_b2;
    wire              rvalid_o_S1_b2;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave1_b2 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel2_b2),
        .valid_i  (valid_bus_b2),
        .we_i     (we_bus_b2),
        .addr_i   (addr_bus_b2),
        .wdata_i  (wdata_bus_b2),
        .rdata_o  (rdata_o_S1_b2),
        .ready_o  (ready_o_S1_b2),
        .rvalid_o (rvalid_o_S1_b2)
    );

    wire [DATA_W-1:0] rdata_o_S2_b2;
    wire              ready_o_S2_b2;
    wire              rvalid_o_S2_b2;

    slave #(
        .ADDR_W (12),
        .DATA_W (DATA_W)
    ) u_slave2_b2 (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (slave_sel3_b2),
        .valid_i  (valid_bus_b2),
        .we_i     (we_bus_b2),
        .addr_i   (addr_bus_b2),
        .wdata_i  (wdata_bus_b2),
        .rdata_o  (rdata_o_S2_b2),
        .ready_o  (ready_o_S2_b2),
        .rvalid_o (rvalid_o_S2_b2)
    );

    assign rdata_slave_b2  = slave_sel1_b2 ? rdata_o_S0_b2 :
                              slave_sel2_b2 ? rdata_o_S1_b2 :
                              slave_sel3_b2 ? rdata_o_S2_b2 :
                              {DATA_W{1'b0}};

    assign ready_slave_b2  = slave_sel1_b2 ? ready_o_S0_b2 :
                              slave_sel2_b2 ? ready_o_S1_b2 :
                              slave_sel3_b2 ? ready_o_S2_b2 :
                              1'b0;

    assign rvalid_slave_b2 = slave_sel1_b2 ? rvalid_o_S0_b2 :
                              slave_sel2_b2 ? rvalid_o_S1_b2 :
                              slave_sel3_b2 ? rvalid_o_S2_b2 :
                              1'b0;

endmodule
