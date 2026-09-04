// master_request_mux: picks the granted master's already-serialized request
// frame (per the arbiter's ctr_sel) and forwards it as the single shared
// addr_data_bus/frame_valid_bus pair. Now that address, we and write-data
// all travel together as one serial frame (see addr_serializer.v), one
// select line is enough - addr_sel/data_sel are no longer needed here, only
// ctr_sel. Split out of system_bus.v's own body, mirroring how
// control_mux.v already owns the req/grant routing side.
module master_request_mux (
    input wire ctr_sel,

    input wire serial_out_M0,
    input wire frame_valid_M0,

    input wire serial_out_M1,
    input wire frame_valid_M1,

    output wire addr_data_bus,    // muxed serial request line, to addr_redirect
    output wire frame_valid_bus   // muxed frame-start strobe, to addr_redirect
);

    assign addr_data_bus   = ctr_sel ? serial_out_M1   : serial_out_M0;
    assign frame_valid_bus = ctr_sel ? frame_valid_M1  : frame_valid_M0;

endmodule
