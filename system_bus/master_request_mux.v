// master_request_mux: picks the granted master's addr/wdata/valid (per
// the arbiter's addr_sel/data_sel/ctr_sel), and derives the bus-wide
// wdata_bus/valid_bus lines from it - the request-side counterpart to
// master_response_mux.v. Split out of system_bus.v's own body, mirroring
// how control_mux.v already owns the req/grant routing side.
module master_request_mux #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8,
    parameter RW     = 1
)(
    input wire addr_sel,
    input wire data_sel,
    input wire ctr_sel,

    input wire [ADDR_W+RW-1:0] addr_M0,
    input wire [ADDR_W+RW-1:0] addr_M1,
    input wire [DATA_W-1:0]    wdata_M0,
    input wire [DATA_W-1:0]    wdata_M1,
    input wire                 valid_M0,
    input wire                 valid_M1,

    input wire slave_sel1,
    input wire slave_sel2,
    input wire slave_sel3,

    output wire [ADDR_W+RW-1:0] addr_mux,    // {addr, we} of the granted master
    output wire                 valid_mux,   // granted master's valid, for addr_decoder

    output wire [DATA_W-1:0]    wdata_bus,        // granted master's write data, to the slaves
    output wire [DATA_W-1:0]    wdata_ext_o_comb, // same data, forwarded if the address decodes external
    output wire                 valid_bus         // 1 = some slave is selected this cycle
);

    assign addr_mux  = addr_sel ? addr_M1  : addr_M0;   // {addr, we}
    assign valid_mux = ctr_sel  ? valid_M1 : valid_M0;

    wire [DATA_W-1:0] wdata_mux = data_sel ? wdata_M1 : wdata_M0;

    assign wdata_bus        = wdata_mux;
    assign wdata_ext_o_comb = wdata_mux;

    assign valid_bus = slave_sel1 | slave_sel2 | slave_sel3;

endmodule
