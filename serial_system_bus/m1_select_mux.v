// m1_select_mux: system_busv1 exposes only one Master 1 (slave-role)
// socket, but here two different requesters want it - a local master
// instantiated alongside it, and slave_bridge relaying the remote board's
// master. sel picks which one drives the shared port this cycle (0 = A,
// the local master; 1 = B, slave_bridge) - purely combinational here, the
// caller owns deciding (and holding) sel, see serial_system_bus.v.
//
// Request/response payloads are single-wire serial lines (addr+we+data
// packed into one frame going out, one serial rdata line coming back) -
// req/grant stay single-bit control signals.
//
// mready flows the OPPOSITE direction from ready did: it's driven BY each
// master and consumed BY the bus (system_bus's mready_M1 is an input),
// so mreadyA/mreadyB are INPUTS here (one from each source) and mready_m1
// is the OUTPUT toward the bus, muxed by the same sel as everything else.
module m1_select_mux (
    input wire sel,

    // Source A: local master
    input  wire reqA,
    output wire grantA,
    input  wire addr_data_A,     // serial request frame line
    input  wire frame_valid_A,   // frame-start strobe
    output wire rdata_A_ser,     // serial response line
    input  wire mreadyA,
    output wire rvalidA,

    // Source B: slave_bridge (remote board's master, over the serial link)
    input  wire reqB,
    output wire grantB,
    input  wire addr_data_B,
    input  wire frame_valid_B,
    output wire rdata_B_ser,
    input  wire mreadyB,
    output wire rvalidB,

    // Shared Master 1 port
    output wire req_m1,
    input  wire grant_m1,
    output wire addr_data_m1,
    output wire frame_valid_m1,
    input  wire rdata_m1_ser,
    output wire mready_m1,
    input  wire rvalid_m1
);

    assign req_m1         = sel ? reqB          : reqA;
    assign addr_data_m1   = sel ? addr_data_B   : addr_data_A;
    assign frame_valid_m1 = sel ? frame_valid_B : frame_valid_A;
    assign mready_m1      = sel ? mreadyB       : mreadyA;

    // Response broadcasts to both, but rvalid/grant are gated so only the
    // currently-selected source ever sees them.
    assign grantA = !sel ? grant_m1 : 1'b0;
    assign grantB = sel  ? grant_m1 : 1'b0;

    assign rdata_A_ser = rdata_m1_ser;
    assign rdata_B_ser = rdata_m1_ser;

    assign rvalidA = !sel ? rvalid_m1 : 1'b0;
    assign rvalidB = sel  ? rvalid_m1 : 1'b0;

endmodule
