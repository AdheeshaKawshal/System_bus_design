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

    // Pulses when the transfer currently on the bus completes, so the
    // arbiter can hand the bus back to a parked master at a clean
    // boundary instead of yanking it mid-transfer.
    wire xfer_done = valid_bus && ready_slave;

    // Routes each master's single grant pair to either this bus's local
    // arbiter or the external one, per the arbiter's ext_sel_Mx flags.
    // Occupancy (who owns the bus) is decided by the arbiter alone, from
    // the raw req_Mx below; this mux only steers where an already-granted
    // transaction's request/grant actually go.
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
    wire [ADDR_W+RW-1:0] addr_mux  = addr_sel ? addr_M1  : addr_M0;   // {addr, we}
    wire [DATA_W-1:0]    wdata_mux = data_sel ? wdata_M1 : wdata_M0;
    wire                 valid_mux = ctr_sel  ? valid_M1 : valid_M0;

    assign wdata_bus = wdata_mux;

    // Same data the granted master is driving is also what goes out to
    // the external bus when its address decodes external (ext_valid_o).
    wire [DATA_W-1:0] wdata_ext_o_comb = wdata_mux;

    // ctr bus: valid (master->slave) muxed above; ready (slave->master)
    // is broadcast back to both masters below.
    assign valid_bus = slave_sel1 | slave_sel2 | slave_sel3;

    // Mux the selected slave's response back onto the shared return path
    wire [DATA_W-1:0] rdata_slave  = slave_sel1 ? rdata_S0  :
                                      slave_sel2 ? rdata_S1  :
                                      slave_sel3 ? rdata_S2  :
                                      {DATA_W{1'b0}};

    wire               ready_slave = slave_sel1 ? ready_S0  :
                                      slave_sel2 ? ready_S1  :
                                      slave_sel3 ? ready_S2  :
                                      1'b0;

    wire               rvalid_slave = slave_sel1 ? rvalid_S0 :
                                       slave_sel2 ? rvalid_S1 :
                                       slave_sel3 ? rvalid_S2 :
                                       1'b0;

    // Loop-breaking register: rdata_ext_i/ready_ext_i/rvalid_ext_i are the
    // response coming back from whatever sits on this bus's external port.
    // In a cross-connected bus_interconnect that's the other system_bus's
    // Master 1 (slave-role) response - ready_M1/rdata_M1/rvalid_M1 - which
    // that bus forwards straight through combinationally the same way we
    // do below whenever ITS ext_sel_M1 is set. Wiring the two directly
    // together (as bus_interconnect.v does) closes a zero-delay
    // combinational loop across the two buses on this return path, same
    // mechanism as the grant_ext and addr_ext_o/wdata_ext_o/ext_valid_o
    // loops fixed elsewhere in this file / in control_mux.v. Registering
    // the inputs here breaks it with one cycle of extra return latency.
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
    wire [DATA_W-1:0] rdata_M0_src  = ext_sel_M0 ? rdata_ext_i_sync  : rdata_slave;
    wire [DATA_W-1:0] rdata_M1_src  = ext_sel_M1 ? rdata_ext_i_sync  : rdata_slave;
    wire              ready_M0_src  = ext_sel_M0 ? ready_ext_i_sync  : ready_slave;
    wire              ready_M1_src  = ext_sel_M1 ? ready_ext_i_sync  : ready_slave;
    wire              rvalid_M0_src = ext_sel_M0 ? rvalid_ext_i_sync : rvalid_slave;
    wire              rvalid_M1_src = ext_sel_M1 ? rvalid_ext_i_sync : rvalid_slave;

    // Read data is broadcast, but ready/rvalid are gated by each master's
    // own grant: once the arbiter can park one master and lend the bus to
    // the other, a parked master's FSM is still sitting there waiting for
    // ready_i, and must not see the *other* master's transfers complete.
    //
    // A split slave's resume pulse is the exception: it can land before
    // the arbiter has formally handed the bus back to the parked master
    // (it's waiting for the borrower's xfer_done boundary), so it's
    // routed straight to whichever master parked_id names, regardless of
    // current grant. (Split/resume is an internal-slave feature only.)
    assign rdata_M0  = rdata_M0_src;
    assign rdata_M1  = rdata_M1_src;
    assign ready_M0  = (ready_M0_src  && grant_M0) || (resume && !parked_id);
    assign ready_M1  = (ready_M1_src  && grant_M1) || (resume &&  parked_id);
    assign rvalid_M0 = (rvalid_M0_src && grant_M0) || (resume && !parked_id);
    assign rvalid_M1 = (rvalid_M1_src && grant_M1) || (resume &&  parked_id);

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

    // Loop-breaking register: addr_ext_o/wdata_ext_o/ext_valid_o are this
    // bus acting as a MASTER onto whatever sits on its external port. In
    // a cross-connected bus_interconnect that "whatever" is the other
    // system_bus's Master 1 (slave-role) port - addr_M1/wdata_M1/valid_M1
    // - which that bus's own addr_mux/wdata_mux/valid_mux can route right
    // back out through ITS addr_ext_o/wdata_ext_o/ext_valid_o, closing a
    // zero-delay combinational loop across the two buses (flagged as
    // LUTLP-1 during synthesis, same mechanism as the grant_ext loop
    // control_mux.v registers). Registering the exported values here
    // breaks that loop with one cycle of extra forwarding latency.
    // ext_valid_i above is deliberately fed from the pre-register *_comb
    // value instead, so the arbiter's own ext_sel_M0/M1 latch timing -
    // which depends on ext_valid_i lining up with the same cycle as the
    // granted master's address - is unaffected.
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
