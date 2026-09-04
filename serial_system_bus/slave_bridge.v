`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: slave_bridge
// Description:
//   Sits behind slave_sel3, which is now dedicated to this module alone
//   (slave 0 = plain slave.v, slave 1 = slave_split.v, slave 2/slave_sel3 =
//   this module - nothing else shares slave_sel3 anymore).
//
//   slave_sel3 is reached two different ways (see addr_decoder.v): a
//   genuine internal address with sel==10, or any address whose
//   external_flag bit is set (routed here regardless of its sel bits).
//   ext_redirect_i tells the two apart:
//     - ext_redirect_i == 0 (internal, sel==10): serviced directly out of
//       local storage (mem[]), exactly like slave.v - no B-side relay, and
//       (like slave.v) a write gets no response at all.
//     - ext_redirect_i == 1 (external): this module acts as a small
//       one-shot master instead - it re-issues the captured frame as a
//       request on its own master-side "B" port (the shape
//       m1_select_mux.v's Source B expects - a port meant to lead off this
//       bus entirely), waits for that response, and relays the byte back
//       out its own slave-side port with a Serializer, the same way
//       slave.v answers a read. A bridged transaction ALWAYS gets relayed
//       back this way, whether the original request was a read or a
//       write: the B-side round trip has to complete before this bridge is
//       free to be reused for the next request anyway, and the bus (via
//       mready_B) always expects it to be a well-behaved master, so we
//       just always shift a response byte back (0 for a write, the real
//       read data for a read).
//
// FSM:
//   IDLE     - waiting for a selected frame to finish capturing; branches
//              on ext_redirect_i as described above.
//   ISSUE    - (external only) request the bus on the B side (reqB) and
//              hold until grantB.
//   WAIT_B   - (external only) the B-side request frame is being
//              (self-timed) transmitted and we're waiting for its
//              response; ACTIVE_TIMEOUT/BACKOFF guards against a response
//              that never arrives, same pattern as master.v.
//   BACKOFF  - (external only) short cool-off before retrying the same
//              relay.
//   RELAY    - (external only) shift the (possibly zero, for a write)
//              response byte back out on the slave side.
//   WAIT_LOW - wait for cs_i to drop before returning to IDLE, so a stale
//              still-asserted cs_i isn't mistaken for a fresh request.
//////////////////////////////////////////////////////////////////////////////////

module slave_bridge #(
    parameter ADDR_W        = 15,
    parameter DATA_W        = 8,
    parameter RW            = 1,
    parameter FRAME_W       = ADDR_W + RW + DATA_W,  // 24
    parameter ACTIVE_TIMEOUT = 40,  // see master.v for the same margin reasoning
    parameter BACKOFF_DELAY  = 5
)(
    input wire clk,
    input wire rst,   // active-low, matches slave.v/slave_split.v's convention

    // ---------------- slave-side port (behind slave_sel3) ----------------
    input wire cs_i,            // slave_sel3 - dedicated, this module is the only thing behind it
    input wire ext_redirect_i,  // dedicated external-flag passthrough from addr_decoder.v - informational only, doesn't gate selection
    input wire addr_data_i,     // shared serial {addr,we,wdata} request frame, MSB first
    input wire valid_i,         // shared frame-start strobe

    output wire rdata_o_ser,    // serial response back to the original master, MSB first
    output wire rvalid_o,       // held-high-during-frame response valid

    // ---------------- master-side "B" port (relayed request) ----------------
    // Shape matches m1_select_mux.v's Source B ports, from this module's own
    // (the requester's) point of view.
    output reg  reqB,
    input  wire grantB,
    output wire addr_data_B,
    output wire frame_valid_B,
    input  wire rdata_B_ser,
    output wire mreadyB,
    input  wire rvalidB
);

    // Always ready to accept the B-side response as soon as it arrives.
    assign mreadyB = 1'b1;

    // ---------------------------------------------------------
    // Slave-side frame capture: shared addr_data_deserializer, gated
    // directly on cs_i (slave_sel3) - dedicated to this module, so no
    // combining with ext_redirect_i is needed to know it's selected.
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
        .cs_i        (cs_i),
        .addr_data_i (addr_data_i),
        .valid_i     (valid_i),
        .we_o        (we_c),
        .addr_o      (addr_c),
        .wdata_o     (wdata_c),
        .frame_done  (frame_done)
    );

    // ---------------------------------------------------------
    // B-side outgoing request: one addr_serializer, triggered for exactly
    // one cycle by tx_start on the ISSUE -> WAIT_B transition (i.e. the
    // cycle grantB is first seen) - same pattern as master.v.
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
        .serial_out      (addr_data_B),
        .frame_valid_out (frame_valid_B)
    );

    // ---------------------------------------------------------
    // B-side incoming response: one deserializer, edge-detected exactly
    // like master.v's rvalid_par_pulse.
    // ---------------------------------------------------------
    wire [DATA_W-1:0] rdataB_par;
    wire               rvalidB_par;
    reg                rvalidB_par_d;
    wire               rvalidB_pulse = rvalidB_par && !rvalidB_par_d;

    deserializer u_deserializer_b (
        .clk_in       (clk),
        .rst_n        (rst),
        .serial_in    (rdata_B_ser),
        .data_valid_in(rvalidB),
        .data_out     (rdataB_par),
        .data_valid   (rvalidB_par)
    );

    // ---------------------------------------------------------
    // Local storage for genuine internal (sel==10) accesses - same shape as
    // slave.v's own mem[]. Only used when ext_redirect_i is NOT set; an
    // external-flagged access never touches this and goes out the B-side
    // relay below instead.
    // ---------------------------------------------------------
    reg [DATA_W-1:0] mem [0:15];

    // ---------------------------------------------------------
    // Slave-side outgoing response: one Serializer, fired either directly
    // for an internal read (IDLE) or after the B-side relay completes
    // (RELAY).
    // ---------------------------------------------------------
    reg [DATA_W-1:0] relay_data;
    reg              relay_trigger;

    Serializer u_serializer (
        .clk_in         (clk),
        .rst_n          (rst),
        .data_in        (relay_data),
        .data_valid     (relay_trigger),
        .serial_out     (rdata_o_ser),
        .data_valid_out (rvalid_o),
        .done           ()
    );

    // ---------------------------------------------------------
    // Small master FSM
    // ---------------------------------------------------------
    localparam IDLE     = 3'd0,
               ISSUE     = 3'd1,
               WAIT_B    = 3'd2,
               BACKOFF   = 3'd3,
               RELAY     = 3'd4,
               WAIT_LOW  = 3'd5;

    reg [2:0] state;
    reg [31:0] timeout_cnt;
    reg [31:0] delay_cnt;
    integer    mem_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state         <= IDLE;
            reqB          <= 1'b0;
            tx_start      <= 1'b0;
            rvalidB_par_d <= 1'b0;
            relay_trigger <= 1'b0;
            relay_data    <= {DATA_W{1'b0}};
            addr_latch    <= {ADDR_W{1'b0}};
            we_latch      <= 1'b0;
            wdata_latch   <= {DATA_W{1'b0}};
            timeout_cnt   <= 0;
            delay_cnt     <= 0;
            for (mem_i = 0; mem_i < 16; mem_i = mem_i + 1) begin
                mem[mem_i] <= {DATA_W{1'b0}};
            end
        end else begin
            tx_start      <= 1'b0;
            relay_trigger <= 1'b0;
            rvalidB_par_d <= rvalidB_par;

            case (state)
                IDLE: begin
                    if (frame_done) begin
                        if (ext_redirect_i) begin
                            // Genuine off-bus address: latch the transaction
                            // and go relay it out the B-side port.
                            addr_latch  <= addr_c;
                            we_latch    <= we_c;
                            wdata_latch <= wdata_c;
                            reqB        <= 1'b1;
                            timeout_cnt <= 0;
                            state       <= ISSUE;
                        end else begin
                            // Genuine internal (sel==10) access: service it
                            // directly with local storage, same as slave.v -
                            // no B-side relay involved. Writes get no
                            // response, same as slave.v.
                            if (we_c) begin
                                mem[addr_c[3:0]] <= wdata_c;
                            end else begin
                                relay_data    <= mem[addr_c[3:0]];
                                relay_trigger <= 1'b1;
                            end
                            state <= WAIT_LOW;
                        end
                    end
                end

                ISSUE: begin
                    // Hold reqB until granted, then kick off the outgoing
                    // frame for exactly one cycle.
                    if (grantB) begin
                        tx_start    <= 1'b1;
                        timeout_cnt <= 0;
                        state       <= WAIT_B;
                    end
                end

                WAIT_B: begin
                    if (rvalidB_pulse) begin
                        // Always relay something back - the real read byte
                        // for a read, 0 for a write (the original master
                        // never looks at it, but the frame still has to be
                        // well-formed).
                        relay_data  <= we_latch ? {DATA_W{1'b0}} : rdataB_par;
                        reqB        <= 1'b0;
                        timeout_cnt <= 0;
                        state       <= RELAY;
                    end else if (timeout_cnt >= ACTIVE_TIMEOUT) begin
                        // No response ever came back: drop off the B-side
                        // bus and retry the same relay after a backoff.
                        reqB        <= 1'b0;
                        timeout_cnt <= 0;
                        delay_cnt   <= 0;
                        state       <= BACKOFF;
                    end else begin
                        timeout_cnt <= timeout_cnt + 1'b1;
                    end
                end

                BACKOFF: begin
                    if (delay_cnt >= BACKOFF_DELAY) begin
                        delay_cnt <= 0;
                        reqB      <= 1'b1;
                        state     <= ISSUE;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                RELAY: begin
                    relay_trigger <= 1'b1;
                    state         <= WAIT_LOW;
                end

                WAIT_LOW: begin
                    // Same reasoning as slave.v/slave_split.v: the original
                    // master only drops cs_i a cycle after seeing the
                    // response start, so sit tight until it actually clears
                    // before accepting a new request.
                    if (!cs_i) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
