// slave_response_mux: picks whichever slave is currently selected
// (slave_sel1/2/3, mutually exclusive per addr_decoder) and muxes its
// rdata/rvalid back onto the shared return path. No per-slave ready here -
// the bus is fixed-latency and mready (the master's grant) is what a
// master watches instead; a slave that can't keep up uses split/resume to
// get parked, not a per-transfer ready stall. Split out of system_bus.v's
// own body, mirroring data_mux.v on the request side.
module slave_response_mux #(
    parameter DATA_W = 8
)(
    input wire slave_sel1,
    input wire slave_sel2,
    input wire slave_sel3,

    input wire [DATA_W-1:0] rdata_S0,
    input wire               rvalid_S0,

    input wire [DATA_W-1:0] rdata_S1,
    input wire               rvalid_S1,

    input wire [DATA_W-1:0] rdata_S2,
    input wire               rvalid_S2,

    output wire [DATA_W-1:0] rdata_slave,
    output wire               rvalid_slave
);

    assign rdata_slave  = slave_sel1 ? rdata_S0  :
                           slave_sel2 ? rdata_S1  :
                           slave_sel3 ? rdata_S2  :
                           {DATA_W{1'b0}};

    assign rvalid_slave = slave_sel1 ? rvalid_S0 :
                           slave_sel2 ? rvalid_S1 :
                           slave_sel3 ? rvalid_S2 :
                           1'b0;

endmodule
