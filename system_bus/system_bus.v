module system_bus #(
    parameter ADDR_W     = 15,  // {internal_flag(1), slave_sel(2), slave_addr(12)}
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3
)(
    input wire clk,
    input wire rst,

    // ===========================================================
    // Master 0
    // ===========================================================
    input  wire                 req_M0,
    output wire                 grant_M0,
    input  wire [ADDR_W+RW-1:0] addr_M0,   // LSB carries we
    input  wire [DATA_W-1:0]    wdata_M0,
    input  wire                 valid_M0,
    output wire [DATA_W-1:0]    rdata_M0,
    output wire                 ready_M0,
    output wire                 rvalid_M0,

    // ===========================================================
    // Master 1
    // ===========================================================
    input  wire                 req_M1,
    output wire                 grant_M1,
    input  wire [ADDR_W+RW-1:0] addr_M1,   // LSB carries we
    input  wire [DATA_W-1:0]    wdata_M1,
    input  wire                 valid_M1,
    output wire [DATA_W-1:0]    rdata_M1,
    output wire                 ready_M1,
    output wire                 rvalid_M1,

    // ===========================================================
    // Slaves (decoded/muxed bus + direct per-slave connections)
    // ===========================================================
    output wire slave_sel1,
    output wire slave_sel2,
    output wire slave_sel3,
    output wire addr_invalid,

    output wire [ADDR_W-2-2:0] addr_bus,   // trimmed slave address (drops internal-flag + sel bits)
    output wire [DATA_W-1:0]   wdata_bus,
    output wire                we_bus,
    output wire                valid_bus,  // ctr line: master -> slave

    input  wire [DATA_W-1:0]  rdata_S0,
    input  wire               ready_S0,
    input  wire               rvalid_S0,

    input  wire [DATA_W-1:0]  rdata_S1,
    input  wire               ready_S1,
    input  wire               rvalid_S1,

    input  wire [DATA_W-1:0]  rdata_S2,     // split-transaction slave
    input  wire               ready_S2,
    input  wire               rvalid_S2,
    input  wire               split,       // from slave S2
    input  wire               resume,      // from slave S2

    // ===========================================================
    // External bus (off-bus forwarding + external arbiter interface)
    // ===========================================================
    output reg  [ADDR_W+RW-1:0] addr_ext_o,
    output reg  [DATA_W-1:0]    wdata_ext_o,
    output reg                  ext_valid_o,

    input  wire [DATA_W-1:0]    rdata_ext_i,
    input  wire                 ready_ext_i,
    input  wire                 rvalid_ext_i,

    output wire req_ext,
    input  wire grant_ext
);

    wire addr_sel, data_sel, ctr_sel;
    wire parked_id;
    wire ext_sel_M0, ext_sel_M1;
    wire grant_M0_int, grant_M1_int;
    wire xfer_done = valid_bus && ready_slave;

    control_mux u_control_mux (
        .clk           (clk),
        .rst           (rst),
        .ext_sel_M0    (ext_sel_M0),
        .ext_sel_M1    (ext_sel_M1),

        .req_M0        (req_M0),
        .req_M1        (req_M1),
        .grant_M0      (grant_M0),
        .grant_M1      (grant_M1),

        .grant_M0_int  (grant_M0_int),
        .grant_M1_int  (grant_M1_int),

        .req_ext       (req_ext),
        .grant_ext     (grant_ext)
    );

    arbiter u_arbiter (
        .clk         (clk),
        .rst         (rst),
        .req_M0      (req_M0),
        .req_M1      (req_M1),
        .split       (split),
        .resume      (resume),
        .xfer_done   (xfer_done),
        .ext_valid_i (ext_valid_o_comb),
        .grant_M0    (grant_M0_int),
        .grant_M1    (grant_M1_int),
        .addr_sel    (addr_sel),
        .data_sel    (data_sel),
        .ctr_sel     (ctr_sel),
        .parked_id   (parked_id),
        .ext_sel_M0  (ext_sel_M0),
        .ext_sel_M1  (ext_sel_M1)
    );

    // ---------------------------------------------------------
    // Addr / data / control muxes: pick the granted master's signals
    // ---------------------------------------------------------
    wire [ADDR_W+RW-1:0] addr_mux;
    wire                 valid_mux;
    wire [DATA_W-1:0]    wdata_ext_o_comb;   // same wdata_bus data, forwarded if the address decodes external

    master_request_mux #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_request_mux (
        .addr_sel (addr_sel),
        .data_sel (data_sel),
        .ctr_sel  (ctr_sel),

        .addr_M0  (addr_M0),
        .addr_M1  (addr_M1),
        .wdata_M0 (wdata_M0),
        .wdata_M1 (wdata_M1),
        .valid_M0 (valid_M0),
        .valid_M1 (valid_M1),

        .slave_sel1 (slave_sel1),
        .slave_sel2 (slave_sel2),
        .slave_sel3 (slave_sel3),

        .addr_mux         (addr_mux),
        .valid_mux        (valid_mux),
        .wdata_bus        (wdata_bus),
        .wdata_ext_o_comb (wdata_ext_o_comb),
        .valid_bus        (valid_bus)
    );

    // Mux the selected slave's response back onto the shared return path
    wire [DATA_W-1:0] rdata_slave;
    wire              ready_slave;
    wire              rvalid_slave;

    slave_response_mux #(
        .DATA_W (DATA_W)
    ) u_slave_response_mux (
        .slave_sel1 (slave_sel1),
        .slave_sel2 (slave_sel2),
        .slave_sel3 (slave_sel3),

        .rdata_S0  (rdata_S0),
        .ready_S0  (ready_S0),
        .rvalid_S0 (rvalid_S0),

        .rdata_S1  (rdata_S1),
        .ready_S1  (ready_S1),
        .rvalid_S1 (rvalid_S1),

        .rdata_S2  (rdata_S2),
        .ready_S2  (ready_S2),
        .rvalid_S2 (rvalid_S2),

        .rdata_slave  (rdata_slave),
        .ready_slave  (ready_slave),
        .rvalid_slave (rvalid_slave)
    );

    reg [DATA_W-1:0] rdata_ext_i_sync;
    reg              ready_ext_i_sync;
    reg              rvalid_ext_i_sync;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_ext_i_sync  <= {DATA_W{1'b0}};
            ready_ext_i_sync  <= 1'b0;
            rvalid_ext_i_sync <= 1'b0;
        end else begin
            rdata_ext_i_sync  <= rdata_ext_i;
            ready_ext_i_sync  <= ready_ext_i;
            rvalid_ext_i_sync <= rvalid_ext_i;
        end
    end

    // Each master's return path is sourced from the internal slave bus or
    // the external bus depending on which one its (latched) request was
    // routed to.
    master_response_mux #(
        .DATA_W (DATA_W)
    ) u_master_response_mux (
        .ext_sel_M0 (ext_sel_M0),
        .ext_sel_M1 (ext_sel_M1),

        .rdata_slave  (rdata_slave),
        .ready_slave  (ready_slave),
        .rvalid_slave (rvalid_slave),

        .rdata_ext_i_sync  (rdata_ext_i_sync),
        .ready_ext_i_sync  (ready_ext_i_sync),
        .rvalid_ext_i_sync (rvalid_ext_i_sync),

        .grant_M0  (grant_M0),
        .grant_M1  (grant_M1),
        .resume    (resume),
        .parked_id (parked_id),

        .rdata_M0  (rdata_M0),
        .rdata_M1  (rdata_M1),
        .ready_M0  (ready_M0),
        .ready_M1  (ready_M1),
        .rvalid_M0 (rvalid_M0),
        .rvalid_M1 (rvalid_M1)
    );

    wire [ADDR_W+RW-1:0] addr_ext_o_comb;
    wire                 ext_valid_o_comb;

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

        .addr_ext_o  (addr_ext_o_comb),
        .ext_valid_o (ext_valid_o_comb),
        .addr_invalid(addr_invalid)
    );

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            addr_ext_o  <= {(ADDR_W+RW){1'b0}};
            wdata_ext_o <= {DATA_W{1'b0}};
            ext_valid_o <= 1'b0;
        end else begin
            addr_ext_o  <= addr_ext_o_comb;
            wdata_ext_o <= wdata_ext_o_comb;
            ext_valid_o <= ext_valid_o_comb;
        end
    end

endmodule
