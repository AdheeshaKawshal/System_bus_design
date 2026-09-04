// ============================================================================
// bb_slave_core.v
// ----------------------------------------------------------------------------
// Slave-side bridge core. Upstream port is now SERIAL, matching this
// folder's slave.v shape exactly (cs_i, addr_data_i, valid_i in;
// rdata_o_ser, rvalid_o out) so this core can plug straight into
// serial_system_bus.v behind any slave_selN. An addr_data_deserializer
// captures the incoming {addr,we,wdata} frame into parallel we_c/addr_c/
// wdata_c (frame_done pulses once per frame, replacing the old
// cs_i&valid_i level + rising-edge-detect scheme entirely), and a
// Serializer at the far end turns the parallel local/remote read result
// back into a serial response frame. There is no ready/backpressure
// handshake anywhere in this design -- the bus takes only rdata + rvalid
// back from a slave.
//
// addr_i[14] selects functionality:
//
//   0 = LOCAL  : serviced by the local 16-byte register file (bb_local_regfile
//                in slavev2.v). Its addr_i port stays 11 bits wide (unchanged
//                interface), but only 16 slots actually exist - only
//                addr_i[3:0] is used to index, so addr_i[10:4] alias
//                (the caller is expected to only ever address the low 16
//                slots). A read answers with rvalid_o one cycle later; a
//                write just happens.
//
//   1 = REMOTE : the transaction is packed into a single 24-bit UART frame and
//                shipped out to a bb_master_core on the far side of the link. A
//                remote WRITE is fire-and-forget -- nothing comes back and
//                nothing is reported. A remote READ waits for the single
//                reply byte and then pulses rvalid_o with it.
//
// Packet layout sent on a REMOTE access -- one 24-bit frame, one start bit,
// one stop bit (26 bit-times total, vs 30 for the same payload framed as
// three separate bytes):
//   pkt[23:16] = { rw, 1'b0, addr[13:8] }
//   pkt[15:8]  = addr[7:0]
//   pkt[7:0]   = wdata[7:0]
// addr[14] is forced to 0 in the outgoing packet -- the receiving board sees
// a plain local address in its own space, so two boards chain symmetrically.
//
// RX_TIMEOUT guards the wait-for-reply state on reads: on expiry the FSM
// returns to idle with no rvalid_o pulse and raises the sticky timeout_o
// flag. Without it a dead link would leave this core wedged in
// R_WAIT_REPLY forever, silently swallowing every later remote access.
//
// A remote request is started by frame_done, the addr_data_deserializer's
// own one-cycle-per-frame capture-complete pulse - inherently a single
// pulse per transaction, so no separate edge-detect is needed here the way
// the old cs_i/valid_i-level scheme required.
//
// There is no backpressure toward the bus, so a remote request that arrives
// while the UART transmitter is still busy with the previous packet cannot be
// stalled -- it is silently dropped. Sequencing remote accesses far enough
// apart is the bus master's responsibility.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module bb_slave_core #(
    parameter CLK_FREQ_HZ = 125000000,     // passed through to the UART primitives
    parameter BAUD_RATE   = 100000,
    // Cycles to wait for the remote reply byte before giving up. At 125 MHz /
    // 100 kbaud one bit is 1250 cycles and one 8N1 byte is 12500, so the reply
    // alone cannot arrive in under 12500 cycles. 50000 leaves roughly three
    // byte-times of headroom for the far side's bus access and turnaround.
    // Rescale this whenever CLK_FREQ_HZ/BAUD_RATE change.
    parameter RX_TIMEOUT  = 50000
)(
    input  wire        clk,
    input  wire        rst,           // active-low

    // ---- upstream bus-slave-shaped port: serial, same shape as slave.v ----
    input  wire        cs_i,
    input  wire        addr_data_i,   // serial {addr,we,wdata} request frame, MSB first
    input  wire        valid_i,       // shared frame-start strobe

    output wire        rdata_o_ser,   // serial response frame, MSB first
    output wire        rvalid_o,      // held-high-during-frame response valid

    // ---- UART pins to the far-side bb_master_core ---------------------
    output wire        uart_tx_o,
    input  wire        uart_rx_i,

    // ---- observation port (sticky, cleared only by rst) -----------------
    output reg         timeout_o      // a remote read got no reply in time
);

    // ------------------------------------------------------------------
    // Request capture: shared addr_data_deserializer does the shift-
    // register work, gated on cs_i. frame_done is already the clean
    // one-cycle "a new transaction just fully arrived" pulse - it replaces
    // the old cs_i&valid_i level + rising-edge-detect scheme entirely, no
    // sel/sel_d/sel_rise needed.
    // ------------------------------------------------------------------
    wire        we_c;
    wire [14:0] addr_c;
    wire [7:0]  wdata_c;
    wire        frame_done;

    addr_data_deserializer #(
        .ADDR_W (15),
        .RW     (1),
        .DATA_W (8)
    ) u_req_deserializer (
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

    wire is_remote = addr_c[14];

    // ------------------------------------------------------------------
    // LOCAL path: the 2 KB register file. cs_i is tied high; selection is
    // done entirely through frame_done, gated so it only pulses when this
    // transaction is actually LOCAL.
    // ------------------------------------------------------------------
    wire [7:0] local_rdata;
    wire       local_rvalid;

    bb_local_regfile #(
        .ADDR_W (11),      // port width unchanged; only 16 slots actually exist - see slavev2.v
        .DATA_W (8)
    ) u_local_slave (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (1'b1),
        .valid_i  (frame_done & ~is_remote),
        .we_i     (we_c),
        .addr_i   (addr_c[10:0]),
        .wdata_i  (wdata_c),
        .rdata_o  (local_rdata),
        .rvalid_o (local_rvalid)
    );

    // ------------------------------------------------------------------
    // REMOTE path: request/reply FSM around uart_frame_tx (WIDTH=24) and
    // uart_frame_rx (WIDTH=8, the reply byte on reads).
    // ------------------------------------------------------------------
    localparam R_IDLE       = 2'd0,
               R_SEND       = 2'd1,   // uart_frame_tx shifting the 24-bit packet out
               R_WAIT_REPLY = 2'd2;   // reads only: wait for uart_frame_rx or RX_TIMEOUT

    reg [1:0]  rstate;
    reg        r_we_latched;
    reg [7:0]  r_rdata;
    reg        r_rvalid;
    reg [31:0] rx_timeout_cnt;

    // addr[14] is forced to 0 in the outgoing header -- see file header.
    wire [23:0] pkt = { {we_c, 1'b0, addr_c[13:8]},   // byte 0: header
                        addr_c[7:0],                  // byte 1
                        wdata_c };                    // byte 2

    reg         tx_send;
    reg  [23:0] tx_data;
    wire        tx_busy, tx_done;

    uart_frame_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .WIDTH       (24)         // the whole request goes out as one frame
    ) u_req_tx (
        .clk    (clk),
        .rst    (rst),
        .data_i (tx_data),
        .send_i (tx_send),
        .tx_o   (uart_tx_o),
        .busy_o (tx_busy),
        .done_o (tx_done)
    );

    wire [7:0] rx_data;
    wire       rx_valid;

    uart_frame_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .WIDTH       (8)          // the reply is a single rdata byte
    ) u_reply_rx (
        .clk         (clk),
        .rst         (rst),
        .rx_i        (uart_rx_i),
        .data_o      (rx_data),
        .valid_o     (rx_valid),
        .frame_err_o ()
    );

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rstate         <= R_IDLE;
            tx_send        <= 1'b0;
            tx_data        <= 24'h0;
            r_we_latched   <= 1'b0;
            r_rdata        <= 8'h00;
            r_rvalid       <= 1'b0;
            rx_timeout_cnt <= 32'd0;
            timeout_o      <= 1'b0;
        end else begin
            tx_send  <= 1'b0;
            r_rvalid <= 1'b0;

            case (rstate)
                R_IDLE: begin
                    // No backpressure toward the bus: if the UART TX is
                    // still busy with a previous packet, this request is
                    // simply dropped (nothing to stall with).
                    if (frame_done && is_remote && !tx_busy) begin
                        tx_data      <= pkt;
                        r_we_latched <= we_c;
                        tx_send      <= 1'b1;
                        rstate       <= R_SEND;
                    end
                end

                R_SEND: begin
                    // tx_send was a 1-cycle pulse; wait for the whole
                    // 24-bit frame to finish shifting out.
                    if (tx_done) begin
                        if (r_we_latched) begin
                            rstate <= R_IDLE;     // write: fire-and-forget
                        end else begin
                            rx_timeout_cnt <= 32'd0;
                            rstate         <= R_WAIT_REPLY;
                        end
                    end
                end

                R_WAIT_REPLY: begin
                    if (rx_valid) begin
                        r_rdata  <= rx_data;
                        r_rvalid <= 1'b1;
                        rstate   <= R_IDLE;
                    end else if (rx_timeout_cnt >= RX_TIMEOUT) begin
                        timeout_o <= 1'b1;        // sticky, only cleared by rst
                        rstate    <= R_IDLE;
                    end else begin
                        rx_timeout_cnt <= rx_timeout_cnt + 1'b1;
                    end
                end

                default: rstate <= R_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Output mux + serializer. Only one path can be pulsing valid on a
    // given cycle: a remote reply arrives hundreds of cycles after the
    // access that triggered it, long after any local access has answered.
    // The muxed byte/trigger feeds a Serializer, same as slave.v's own
    // response path, turning it into the serial rdata_o_ser/rvalid_o pair.
    // ------------------------------------------------------------------
    wire       ser_trigger = local_rvalid | r_rvalid;
    wire [7:0] ser_data    = r_rvalid ? r_rdata : local_rdata;

    Serializer u_serializer (
        .clk_in         (clk),
        .rst_n          (rst),
        .data_in        (ser_data),
        .data_valid     (ser_trigger),
        .serial_out     (rdata_o_ser),
        .data_valid_out (rvalid_o),
        .done           ()
    );

endmodule
