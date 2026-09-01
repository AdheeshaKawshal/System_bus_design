// m1_select_mux: system_busv1 exposes only one Master 1 (slave-role)
// socket, but here two different requesters want it - a local master
// instantiated alongside it, and slave_bridge relaying the remote board's
// master. sel picks which one drives the shared port this cycle (0 = A,
// the local master; 1 = B, slave_bridge) - purely combinational here, the
// caller owns deciding (and holding) sel, see serial_system_bus.v.
module m1_select_mux #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8,
    parameter RW     = 1
)(
    input wire sel,

    // Source A: local master
    input  wire                 reqA,
    output wire                 grantA,
    input  wire [ADDR_W+RW-1:0] addrA,
    input  wire [DATA_W-1:0]    wdataA,
    input  wire                 validA,
    output wire [DATA_W-1:0]    rdataA,
    output wire                 readyA,
    output wire                 rvalidA,

    // Source B: slave_bridge (remote board's master, over the serial link)
    input  wire                 reqB,
    output wire                 grantB,
    input  wire [ADDR_W+RW-1:0] addrB,
    input  wire [DATA_W-1:0]    wdataB,
    input  wire                 validB,
    output wire [DATA_W-1:0]    rdataB,
    output wire                 readyB,
    output wire                 rvalidB,

    // Shared Master 1 port
    output wire                 req_m1,
    input  wire                 grant_m1,
    output wire [ADDR_W+RW-1:0] addr_m1,
    output wire [DATA_W-1:0]    wdata_m1,
    output wire                 valid_m1,
    input  wire [DATA_W-1:0]    rdata_m1,
    input  wire                 ready_m1,
    input  wire                 rvalid_m1
);

    assign req_m1   = sel ? reqB   : reqA;
    assign addr_m1  = sel ? addrB  : addrA;
    assign wdata_m1 = sel ? wdataB : wdataA;
    assign valid_m1 = sel ? validB : validA;

    // Response broadcasts to both, but ready/rvalid/grant are gated so
    // only the currently-selected source ever sees them.
    assign grantA  = !sel ? grant_m1  : 1'b0;
    
    assign grantB  = sel  ? grant_m1  : 1'b0;
    assign rdataA  = rdata_m1;
    assign rdataB  = rdata_m1;
    assign readyA  = !sel ? ready_m1  : 1'b0;
    assign readyB  = sel  ? ready_m1  : 1'b0;
    assign rvalidA = !sel ? rvalid_m1 : 1'b0;
    assign rvalidB = sel  ? rvalid_m1 : 1'b0;

endmodule
