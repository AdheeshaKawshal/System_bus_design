(* keep_hierarchy = "yes" *)
module control_mux (

    input wire clk,
    input wire rst,

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
    output reg  req_ext,
    input  wire grant_ext
);

    wire req_M0_ext = req_M0 & ext_sel_M0;
    wire req_M1_ext = req_M1 & ext_sel_M1;

    wire req_ext_comb = req_M0_ext | req_M1_ext;

    // req_ext, on a cross-connected bus_interconnect, feeds straight into
    // the other bus's req_M1, which that bus's own control_mux can route
    // right back out as ITS req_ext the same way we do here, closing a
    // zero-delay combinational loop across the two buses (bus1.req_ext ->
    // bus2.req_M1 -> bus2.req_ext -> bus1.req_M1 -> bus1.req_ext -> ...),
    // flagged as LUTLP-1 during synthesis - the same mechanism as the
    // grant_ext loop below, just on the request line instead of grant.
    // Registering req_ext here breaks that loop with one cycle of extra
    // request latency, which the arbiter/master handshakes already
    // tolerate.
    always @(posedge clk or negedge rst) begin
        if (!rst) req_ext <= 1'b0;
        else      req_ext <= req_ext_comb;
    end

    // grant_ext, on a cross-connected bus_interconnect, is another bus's
    // grant_M1 - which its own control_mux forwards straight through
    // combinationally the same way we do below. Wiring the two straight
    // together closes a zero-delay combinational loop across the two
    // buses (bus1.grant_M1 -> bus2.grant_ext -> bus2.grant_M1 ->
    // bus1.grant_ext -> bus1.grant_M1 -> ...), flagged as LUTLP-1 during
    // synthesis. Registering grant_ext here breaks that loop with one
    // cycle of extra grant latency, which the arbiter/master handshakes
    // already tolerate.
    reg grant_ext_sync;
    always @(posedge clk or negedge rst) begin
        if (!rst) grant_ext_sync <= 1'b0;
        else      grant_ext_sync <= grant_ext;
    end

    assign grant_M0 = ext_sel_M0 ? grant_ext_sync : grant_M0_int;
    assign grant_M1 = ext_sel_M1 ? grant_ext_sync : grant_M1_int;

endmodule
