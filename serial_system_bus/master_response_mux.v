// master_response_mux: routes each master's return path from either the
// internal slave bus or the external bus (per its latched ext_sel_Mx),
// then gates ready/rvalid by that master's own grant so a parked master
// never sees the other master's transfers complete - except a split
// slave's resume pulse, which bypasses the grant gate for whichever
// master parked_id names (see arbiter.v's PARKED_* states). Split out of
// system_bus.v's own body, mirroring data_mux.v / slave_response_mux.v.
module master_response_mux #(
    parameter DATA_W = 8
)(
    input wire ext_sel_M0,
    input wire ext_sel_M1,

    input wire [DATA_W-1:0] rdata_slave,
    input wire               ready_slave,
    input wire               rvalid_slave,

    input wire [DATA_W-1:0] rdata_ext_i_sync,
    input wire               ready_ext_i_sync,
    input wire               rvalid_ext_i_sync,

    input wire grant_M0,
    input wire grant_M1,
    input wire resume,
    input wire parked_id,

    output wire [DATA_W-1:0] rdata_M0,
    output wire [DATA_W-1:0] rdata_M1,
    output wire               ready_M0,
    output wire               ready_M1,
    output wire               rvalid_M0,
    output wire               rvalid_M1
);

    wire [DATA_W-1:0] rdata_M0_src  = ext_sel_M0 ? rdata_ext_i_sync  : rdata_slave;
    wire [DATA_W-1:0] rdata_M1_src  = ext_sel_M1 ? rdata_ext_i_sync  : rdata_slave;
    wire              ready_M0_src  = ext_sel_M0 ? ready_ext_i_sync  : ready_slave;
    wire              ready_M1_src  = ext_sel_M1 ? ready_ext_i_sync  : ready_slave;
    wire              rvalid_M0_src = ext_sel_M0 ? rvalid_ext_i_sync : rvalid_slave;
    wire              rvalid_M1_src = ext_sel_M1 ? rvalid_ext_i_sync : rvalid_slave;

    assign rdata_M0  = rdata_M0_src;
    assign rdata_M1  = rdata_M1_src;
    assign ready_M0  = (ready_M0_src  && grant_M0) || (resume && !parked_id);
    assign ready_M1  = (ready_M1_src  && grant_M1) || (resume &&  parked_id);
    assign rvalid_M0 = (rvalid_M0_src && grant_M0) || (resume && !parked_id);
    assign rvalid_M1 = (rvalid_M1_src && grant_M1) || (resume &&  parked_id);

endmodule
