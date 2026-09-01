// slave_response_mux: picks whichever slave is currently selected
// (slave_sel1/2/3, mutually exclusive per addr_decoder) and muxes its
// rdata/ready/rvalid back onto the shared return path. Split out of
// system_bus.v's own body, mirroring data_mux.v on the request side.
module slave_response_mux #(
    parameter DATA_W = 8
)(
    input wire slave_sel1,
    input wire slave_sel2,
    input wire slave_sel3,

    input wire [DATA_W-1:0] rdata_S0,
    input wire               ready_S0,
    input wire               rvalid_S0,

    input wire [DATA_W-1:0] rdata_S1,
    input wire               ready_S1,
    input wire               rvalid_S1,

    input wire [DATA_W-1:0] rdata_S2,
    input wire               ready_S2,
    input wire               rvalid_S2,

    output wire [DATA_W-1:0] rdata_slave,
    output wire               ready_slave,
    output wire               rvalid_slave
);

    assign rdata_slave  = slave_sel1 ? rdata_S0  :
                           slave_sel2 ? rdata_S1  :
                           slave_sel3 ? rdata_S2  :
                           {DATA_W{1'b0}};

    assign ready_slave  = slave_sel1 ? ready_S0  :
                           slave_sel2 ? ready_S1  :
                           slave_sel3 ? ready_S2  :
                           1'b0;

    assign rvalid_slave = slave_sel1 ? rvalid_S0 :
                           slave_sel2 ? rvalid_S1 :
                           slave_sel3 ? rvalid_S2 :
                           1'b0;

endmodule
