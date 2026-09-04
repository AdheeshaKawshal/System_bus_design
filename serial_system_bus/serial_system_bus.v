module serial_system_bus #(
    parameter ADDR_W     = 15,  // {external_flag(1), slave_sel(2), slave_addr(12)}
    parameter DATA_W     = 8,
    parameter RW         = 1,
    parameter NUM_SLAVES = 3
)(
    input wire clk,
    input wire rst,

    // ===========================================================
    // Master 0 (fully serial: one addr+we+wdata request frame line in,
    // one rdata frame line back out)
    // ===========================================================
    input  wire req_M0,
    output wire grant_M0,
    input  wire addr_data_M0,    // serial {addr,we,wdata} request frame, MSB first
    input  wire frame_valid_M0,  // request frame-start strobe
    output wire rdata_M0_ser,    // serial rdata response frame, MSB first
    input  wire mready_M0,       // from the master: it's ready to accept the response now
    output wire rvalid_M0,       // also the response frame's start/valid strobe

    // ===========================================================
    // Master 1 (fully serial, same shape as Master 0)
    // ===========================================================
    input  wire req_M1,
    output wire grant_M1,
    input  wire addr_data_M1,
    input  wire frame_valid_M1,
    output wire rdata_M1_ser,
    input  wire mready_M1,
    output wire rvalid_M1,

    // ===========================================================
    // Slaves (decoded/muxed bus + direct per-slave connections)
    // ===========================================================
    output wire slave_sel1,
    output wire slave_sel2,
    output wire slave_sel3,
    output wire ext_redirect,  // alongside slave_sel3 only for an external address - tells S2 to bridge, not service
    output wire addr_invalid,

    output wire addr_data_bus,  // serial {addr,we,wdata} frame to the selected slave, MSB first
    output wire valid_bus,      // frame-start strobe for addr_data_bus
    output wire mready_bus,     // granted master's mready, forwarded so a slave (e.g. S2) knows it's safe to send data

    input  wire  rdata_S0_ser,   // slave's rdata byte, serialized MSB-first
    input  wire  rvalid_S0,      // also the deserializer's frame-start strobe

    input  wire  rdata_S1_ser,
    input  wire  rvalid_S1,

    input  wire  rdata_S2_ser,   // split-transaction slave
    input  wire  rvalid_S2,
    input  wire  split,       // from slave S2
    input  wire  resume       // from slave S2
);

    wire addr_sel, data_sel, ctr_sel;
    wire parked_id;

    // xfer_done must mark the END of the frame currently on addr_data_bus,
    // not its start - valid_bus only pulses once, the cycle the (delayed)
    // frame first appears on addr_data_bus, and the frame then keeps
    // shifting out for FRAME_W-1 more cycles. If the arbiter used valid_bus
    // directly as xfer_done, a resume arriving during that window could
    // switch ctr_sel/grant back to the parked master mid-frame, splicing
    // the parked master's bits onto addr_data_bus in the middle of the
    // borrowing master's still-in-flight frame (both at the request mux and
    // inside addr_redirect's delay pipeline, which would still be draining
    // the interrupted frame). So count out the full frame length before
    // declaring it done.
    localparam FRAME_W = ADDR_W + RW + DATA_W;
    localparam XFER_CNT_W = $clog2(FRAME_W);

    reg [XFER_CNT_W-1:0] xfer_cnt;
    reg                  xfer_active;
    reg                  xfer_done_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            xfer_cnt    <= {XFER_CNT_W{1'b0}};
            xfer_active <= 1'b0;
            xfer_done_r <= 1'b0;
        end else begin
            xfer_done_r <= 1'b0;

            if (valid_bus && !xfer_active) begin
                xfer_active <= 1'b1;
                xfer_cnt    <= {XFER_CNT_W{1'b0}};
            end else if (xfer_active) begin
                if (xfer_cnt < FRAME_W-2) begin
                    xfer_cnt <= xfer_cnt + 1'b1;
                end else begin
                    xfer_active <= 1'b0;
                    xfer_done_r <= 1'b1; // last bit of the frame just cleared addr_data_bus
                end
            end
        end
    end

    wire xfer_done = xfer_done_r;

    // No external bus: the arbiter's own grant_M0/grant_M1 drive the
    // top-level grants directly (no control_mux needed - that module only
    // existed to route grants between an internal arbiter and an external
    // one).
    arbiter u_arbiter (
        .clk         (clk),
        .rst         (rst),
        .req_M0      (req_M0),
        .req_M1      (req_M1),
        .split       (split),
        .resume      (resume),
        .xfer_done   (xfer_done),
        .grant_M0    (grant_M0),
        .grant_M1    (grant_M1),
        .addr_sel    (addr_sel),
        .data_sel    (data_sel),
        .ctr_sel     (ctr_sel),
        .parked_id   (parked_id)
    );

    // ---------------------------------------------------------
    // Request mux: pick the granted master's serial frame. Each master
    // already presents its {addr,we,wdata} as one serial frame directly on
    // its port (addr_data_Mx/frame_valid_Mx) - no serializer needed here,
    // the whole master<->bus hop is a single wire.
    // ---------------------------------------------------------
    wire req_line, req_frame_valid;

    master_request_mux u_master_request_mux (
        .ctr_sel (ctr_sel),

        .serial_out_M0   (addr_data_M0),
        .frame_valid_M0  (frame_valid_M0),
        .serial_out_M1   (addr_data_M1),
        .frame_valid_M1  (frame_valid_M1),

        .addr_data_bus   (req_line),
        .frame_valid_bus (req_frame_valid)
    );

    // ---------------------------------------------------------
    // addr_redirect: decides the slave from the request frame's early tap
    // bits, then forwards the whole raw frame (addr_data_bus/valid_bus) to
    // the slave side unchanged, just delayed until slave_sel* is stable.
    // An address whose external-flag bit is set is routed to slave 3 by
    // addr_decoder - there's no off-bus port anymore.
    // ---------------------------------------------------------
    addr_redirect #(
        .NUM_SLAVES (NUM_SLAVES)
    ) u_addr_redirect (
        .clk            (clk),
        .rst_n          (rst),

        .serial_in      (req_line),
        .frame_valid_in (req_frame_valid),

        .slave_sel1   (slave_sel1),
        .slave_sel2   (slave_sel2),
        .slave_sel3   (slave_sel3),
        .ext_redirect (ext_redirect),
        .addr_invalid (addr_invalid),

        .addr_data_o   (addr_data_bus),
        .frame_valid_o (valid_bus)
    );

    // Forward the granted master's own mready to the slave side, same
    // ctr_sel selection master_request_mux already uses for the request
    // line - a slave (S2's split path in particular) watches this to know
    // the owning master is actually ready to receive its response.
    assign mready_bus = ctr_sel ? mready_M1 : mready_M0;

    // ---------------------------------------------------------
    // Mux the three slaves' serial rdata bits directly, the same way
    // master_request_mux muxes serial request bits - it's a live wire
    // selection, so no deserialize/reserialize round trip is needed here.
    // slave_response_mux is DATA_W-parametrized already, so instantiating
    // it with DATA_W=1 turns its parallel mux into a 1-bit serial mux with
    // no source changes. No per-slave ready anymore - see mready below.
    // ---------------------------------------------------------
    wire rdata_slave_ser;
    wire rvalid_slave;

    slave_response_mux #(
        .DATA_W (1)
    ) u_slave_response_mux (
        .slave_sel1 (slave_sel1),
        .slave_sel2 (slave_sel2),
        .slave_sel3 (slave_sel3),

        .rdata_S0  (rdata_S0_ser),
        .rvalid_S0 (rvalid_S0),

        .rdata_S1  (rdata_S1_ser),
        .rvalid_S1 (rvalid_S1),

        .rdata_S2  (rdata_S2_ser),
        .rvalid_S2 (rvalid_S2),

        .rdata_slave  (rdata_slave_ser),
        .rvalid_slave (rvalid_slave)
    );

    // ---------------------------------------------------------
    // Each master's return path is just the (single) slave bus, gated by
    // that master's own grant so a parked master never sees the other
    // master's transfers complete - except a split slave's resume pulse,
    // which bypasses the grant gate for whichever master parked_id names
    // (see arbiter.v's PARKED_* states). With no external bus there's only
    // ever one source, so this is a plain gate, not a mux.
    //
    // mready (from each master, forwarded to slaves as mready_bus above)
    // replaces the old per-slave ready entirely: a slave that needs more
    // time uses split/resume (which already drives grant_M0/grant_M1
    // through the park/resume cycle in arbiter.v) instead of holding a
    // per-transfer ready low.
    // ---------------------------------------------------------
    assign rdata_M0_ser = rdata_slave_ser;
    assign rdata_M1_ser = rdata_slave_ser;

    assign rvalid_M0 = (rvalid_slave && grant_M0) || (resume && !parked_id);
    assign rvalid_M1 = (rvalid_slave && grant_M1) || (resume &&  parked_id);

endmodule
