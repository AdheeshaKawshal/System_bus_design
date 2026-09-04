// master_bridge: takes over Master 1's slot on this bus entirely (same
// port shape as master.v: req_o/grant_i/addr_data_o/frame_valid_o/
// mready_o/rdata_ser_i/rvalid_i, wired straight into serial_system_bus's M1
// port) - but instead of running its own local transaction table like
// master.v, every request it issues comes from slave_bridge.v's B-side
// port instead. It is the master-side gateway for a relayed/
// external-flagged request: slave_bridge captures a frame behind
// slave_sel3, hands it to this module over the dedicated point-to-point
// B-side link (reqB_i/grantB_o/...), this module arbitrates for and issues
// that SAME frame onto the bus as a normal master request, waits for the
// bus's response, and relays that response back to slave_bridge - the same
// "always relay something back, read or write" contract slave_bridge's own
// header describes.
//
// WARNING - re-entrant addressing hazard: the frame is forwarded onto M1
// with its address bits (including the external_flag bit) UNCHANGED. Since
// addr_redirect decodes that same external_flag bit again on this second
// pass, a genuinely external-flagged frame will route right back to
// slave_sel3/slave_bridge again, which will try to relay it out again,
// forever - there is no real second bus/board here for it to actually
// reach, only a loop back through the same addr_redirect. This mirrors the
// sel==10 self-reference bug slave_bridge.v itself already had to route
// around by servicing internal accesses locally; there is no equivalent
// fix available here without master_bridge deciding to clear/reinterpret
// the external_flag bit before re-issuing (not done here, since only the
// caller can say what the corrected address should become).
//
// FSM:
//   IDLE      - waiting for slave_bridge to request the link (reqB_i).
//   CAPTURE   - grantB_o is up; capturing the relayed frame off the B-side
//               port with the shared addr_data_deserializer.
//   REQUEST   - drive req_o until grant_i, then kick off the outgoing
//               frame for exactly one cycle (mirrors master.v's REQUEST).
//   ACTIVE    - frame is (self-timed) transmitting on the bus; for a write
//               just hold the grant WRITE_DELAY cycles, for a read wait
//               for the bus's response with ACTIVE_TIMEOUT/BACKOFF guarding
//               against one that never arrives (mirrors master.v's ACTIVE/
//               BACKOFF exactly).
//   BACKOFF   - short cool-off before retrying the same request.
//   RELAY     - shift the (possibly zero, for a write) response byte back
//               to slave_bridge over the B-side port.
//   WAIT_LOW  - wait for reqB_i to drop before returning to IDLE.
module master_bridge #(
    parameter ADDR_W        = 15,
    parameter DATA_W        = 8,
    parameter RW            = 1,
    parameter ACTIVE_TIMEOUT = 40,  // see master.v for the same margin reasoning
    parameter BACKOFF_DELAY  = 5,
    parameter WRITE_DELAY    = 26
)(
    input wire clk,
    input wire rst,   // active-low, matches every other module's convention

    // ---------------- bus master-side port (plugs into serial_system_bus's M1) ----------------
    output reg  req_o,
    input  wire grant_i,
    output wire addr_data_o,    // serial {addr,we,wdata} request frame, MSB first
    output wire frame_valid_o,  // request frame-start strobe
    output wire mready_o,       // always ready for the bus's response
    input  wire rdata_ser_i,    // serial rdata response frame, MSB first
    input  wire rvalid_i,       // response frame-valid (held-high style)

    // ---------------- external link to slave_bridge's B-side port ----------------
    input  wire reqB_i,
    output reg  grantB_o,
    input  wire addr_data_B_i,
    input  wire frame_valid_B_i,
    output wire rdata_B_ser_o,
    input  wire mreadyB_i,
    output wire rvalidB_o
);

    assign mready_o = 1'b1;

    // ---------------------------------------------------------
    // B-side frame capture: shared addr_data_deserializer, gated on
    // grantB_o exactly like cs_i in slave.v.
    // ---------------------------------------------------------
    wire              we_c;
    wire [ADDR_W-1:0] addr_c;
    wire [DATA_W-1:0] wdata_c;
    wire              frame_done;

    addr_data_deserializer #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_deserializer (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (grantB_o),
        .addr_data_i (addr_data_B_i),
        .valid_i     (frame_valid_B_i),
        .we_o        (we_c),
        .addr_o      (addr_c),
        .wdata_o     (wdata_c),
        .frame_done  (frame_done)
    );

    // ---------------------------------------------------------
    // Bus-side outgoing request: one addr_serializer, triggered for exactly
    // one cycle by tx_start on the REQUEST -> ACTIVE transition, same
    // pattern as master.v.
    // ---------------------------------------------------------
    reg tx_start;
    reg [ADDR_W-1:0] addr_latch;
    reg              we_latch;
    reg [DATA_W-1:0] wdata_latch;

    addr_serializer #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_addr_serializer (
        .clk         (clk),
        .rst_n       (rst),
        .addr_i      ({addr_latch, we_latch}),
        .wdata_i     (wdata_latch),
        .valid_i     (tx_start),
        .serial_out      (addr_data_o),
        .frame_valid_out (frame_valid_o)
    );

    // ---------------------------------------------------------
    // Bus-side incoming response: one deserializer, edge-detected exactly
    // like master.v's rvalid_par_pulse.
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdata_par;
    wire               rvalid_par;
    reg                rvalid_par_d;
    wire               rvalid_par_pulse = rvalid_par && !rvalid_par_d;

    deserializer u_deserializer_bus (
        .clk_in       (clk),
        .rst_n        (rst),
        .serial_in    (rdata_ser_i),
        .data_valid_in(rvalid_i),
        .data_out     (rdata_par),
        .data_valid   (rvalid_par)
    );

    // ---------------------------------------------------------
    // B-side outgoing response: one Serializer, fired in RELAY - always
    // sends something back, read or write, same as slave_bridge's own
    // contract toward its slave-side caller.
    // ---------------------------------------------------------
    reg [DATA_W-1:0] relay_data;
    reg              relay_trigger;

    Serializer u_serializer (
        .clk_in         (clk),
        .rst_n          (rst),
        .data_in        (relay_data),
        .data_valid     (relay_trigger),
        .serial_out     (rdata_B_ser_o),
        .data_valid_out (rvalidB_o),
        .done           ()
    );

    // ---------------------------------------------------------
    // Combined FSM
    // ---------------------------------------------------------
    localparam IDLE     = 3'd0,
               CAPTURE   = 3'd1,
               REQUEST   = 3'd2,
               ACTIVE    = 3'd3,
               BACKOFF   = 3'd4,
               RELAY     = 3'd5,
               WAIT_LOW  = 3'd6;

    reg [2:0] state;
    reg [31:0] timeout_cnt;
    reg [31:0] delay_cnt;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state         <= IDLE;
            req_o         <= 1'b0;
            grantB_o      <= 1'b0;
            tx_start      <= 1'b0;
            rvalid_par_d  <= 1'b0;
            relay_trigger <= 1'b0;
            relay_data    <= {DATA_W{1'b0}};
            addr_latch    <= {ADDR_W{1'b0}};
            we_latch      <= 1'b0;
            wdata_latch   <= {DATA_W{1'b0}};
            timeout_cnt   <= 0;
            delay_cnt     <= 0;
        end else begin
            tx_start      <= 1'b0;
            relay_trigger <= 1'b0;
            rvalid_par_d  <= rvalid_par;

            case (state)
                IDLE: begin
                    if (reqB_i) begin
                        grantB_o <= 1'b1;
                        state    <= CAPTURE;
                    end
                end

                CAPTURE: begin
                    if (frame_done) begin
                        addr_latch  <= addr_c;
                        we_latch    <= we_c;
                        wdata_latch <= wdata_c;
                        req_o       <= 1'b1;
                        timeout_cnt <= 0;
                        state       <= REQUEST;
                    end
                end

                REQUEST: begin
                    // drive request until grant is received, then kick off
                    // the outgoing frame for exactly one cycle
                    if (grant_i) begin
                        tx_start    <= 1'b1;
                        timeout_cnt <= 0;
                        state       <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    if (we_latch) begin
                        // Write: no acknowledgement on this bus - just hold
                        // the grant long enough for the frame to clear the
                        // request line, then relay a zero byte back.
                        if (timeout_cnt >= WRITE_DELAY) begin
                            req_o       <= 1'b0;
                            timeout_cnt <= 0;
                            relay_data  <= {DATA_W{1'b0}};
                            state       <= RELAY;
                        end else begin
                            timeout_cnt <= timeout_cnt + 1'b1;
                        end
                    end else begin
                        // Read: wait for the deserializer's capture pulse.
                        if (rvalid_par_pulse) begin
                            relay_data  <= rdata_par;
                            req_o       <= 1'b0;
                            timeout_cnt <= 0;
                            state       <= RELAY;
                        end else if (timeout_cnt >= ACTIVE_TIMEOUT) begin
                            // Gave up waiting for a response: drop off the
                            // bus and retry after a backoff instead of
                            // holding the arbiter hostage forever.
                            req_o       <= 1'b0;
                            timeout_cnt <= 0;
                            delay_cnt   <= 0;
                            state       <= BACKOFF;
                        end else begin
                            timeout_cnt <= timeout_cnt + 1'b1;
                        end
                    end
                end

                BACKOFF: begin
                    if (delay_cnt >= BACKOFF_DELAY) begin
                        delay_cnt <= 0;
                        req_o     <= 1'b1;
                        state     <= REQUEST;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                RELAY: begin
                    relay_trigger <= 1'b1;
                    state         <= WAIT_LOW;
                end

                WAIT_LOW: begin
                    // Same reasoning as slave.v/slave_bridge.v: the
                    // requester only drops reqB_i a cycle after seeing the
                    // response start, so sit tight until it actually
                    // clears before accepting a new request.
                    if (!reqB_i) begin
                        grantB_o <= 1'b0;
                        state    <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
