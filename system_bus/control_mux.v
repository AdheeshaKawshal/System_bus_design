module control_mux (
    // ---------------------------------------------------------
    // Per-master routing flag: asserted alongside req_Mx when that
    // master's pending transaction targets the external bus instead of
    // the local slaves. Driven by the master's own address-decode logic,
    // so it must be valid whenever req_Mx is asserted.
    // ---------------------------------------------------------
    input wire ext_sel_M0,
    input wire ext_sel_M1,

    // ---------------------------------------------------------
    // Master-side interface (single req/grant pair per master, as seen
    // by the master module itself)
    // ---------------------------------------------------------
    input  wire req_M0,
    input  wire req_M1,
    output wire grant_M0,
    output wire grant_M1,

    // ---------------------------------------------------------
    // Local (internal) arbiter side: the arbiter decides occupancy from
    // the raw req_Mx above, so only its grant needs to be muxed back out.
    // ---------------------------------------------------------
    input  wire grant_M0_int,
    input  wire grant_M1_int,

    // ---------------------------------------------------------
    // External bus arbiter side: ext_sel_M0/ext_sel_M1 are mutually
    // exclusive (the arbiter only ever flags its current internal-bus
    // owner), so exactly one master's request can be routed out here at
    // a time. A single req/grant pair is exposed outward instead of one
    // per master.
    // ---------------------------------------------------------
    output wire req_ext,
    input  wire grant_ext
);

    // Forward each master's request onward to the external bus only when
    // its own ext_sel flag says its granted transaction decoded external.
    // (Occupancy of the local arbiter no longer depends on this split --
    // see system_bus.v, where the arbiter is fed the raw req_Mx instead.)
    wire req_M0_ext = req_M0 & ext_sel_M0;
    wire req_M1_ext = req_M1 & ext_sel_M1;

    assign req_ext = req_M0_ext | req_M1_ext;

    // Mux: return the grant from whichever arbiter the request was
    // routed to. On the external side, ext_sel_M0/ext_sel_M1 select
    // which master grant_ext belongs to.
    assign grant_M0 = ext_sel_M0 ? grant_ext : grant_M0_int;
    assign grant_M1 = ext_sel_M1 ? grant_ext : grant_M1_int;

endmodule
