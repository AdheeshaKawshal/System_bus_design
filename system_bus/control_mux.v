module control_mux (

    input wire ext_sel_M0,
    input wire ext_sel_M1,

    // ---------------------------------------------------------
    // Master-side interface
    // ---------------------------------------------------------
    input  wire req_M0,
    input  wire req_M1,
    output wire grant_M0,
    output wire grant_M1,

    // ---------------------------------------------------------
    // Local (internal) arbiter side:
    // ---------------------------------------------------------
    input  wire grant_M0_int,
    input  wire grant_M1_int,

    // ---------------------------------------------------------
    // External bus arbiter side: ext_sel_M0/ext_sel_M1 are mutually exclusive, so only one of these req_Mx_ext signals can be high at a time.
    // ---------------------------------------------------------
    output wire req_ext,
    input  wire grant_ext
);

    wire req_M0_ext = req_M0 & ext_sel_M0;
    wire req_M1_ext = req_M1 & ext_sel_M1;

    assign req_ext = req_M0_ext | req_M1_ext;

    assign grant_M0 = ext_sel_M0 ? grant_ext : grant_M0_int;
    assign grant_M1 = ext_sel_M1 ? grant_ext : grant_M1_int;

endmodule
