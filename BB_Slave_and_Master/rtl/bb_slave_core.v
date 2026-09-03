// ============================================================================
// bb_slave_core.v
// ----------------------------------------------------------------------------
// Slave-side bridge core. Sits between the bus bridge's deserializer and the
// local register file, and presents the same port shape as a plain bus slave:
// cs_i, valid_i, we_i, addr_i, wdata_i in; rdata_o, rvalid_o out. There is no
// ready/backpressure handshake anywhere in this design -- the bus takes only
// rdata + rvalid back from a slave.
//
// addr_i[14] selects functionality:
//
//   0 = LOCAL  : serviced by the local 2 KB register file (slave.v in this
//                folder), addressed by addr_i[10:0]. addr_i[13:11] are
//                ignored, so those bits alias. A read answers with rvalid_o
//                one cycle later; a write just happens.
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
// A remote request is started on the RISING EDGE of (cs_i & valid_i), never
// on its level. The upstream master holds valid_i up across many cycles, so
// a level trigger would relaunch the same packet on every one of them.
//
// There is no backpressure toward the bus, so a remote request that arrives
// while the UART transmitter is still busy with the previous packet cannot be
// stalled -- it is dropped and overflow_o is raised (sticky). Sequencing
// remote accesses far enough apart is the bus master's responsibility.
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

    // ---- upstream bus-slave-shaped port -------------------------------
    input  wire        cs_i,
    input  wire        valid_i,
    input  wire        we_i,          // 1 = write, 0 = read
    input  wire [14:0] addr_i,
    input  wire [7:0]  wdata_i,

    output reg  [7:0]  rdata_o,
    output reg         rvalid_o,

    // ---- UART pins to the far-side bb_master_core ---------------------
    output wire        uart_tx_o,
    input  wire        uart_rx_i,

    // ---- observation ports (sticky, cleared only by rst) ---------------
    output reg         timeout_o,     // a remote read got no reply in time
    output reg         overflow_o,    // a remote request arrived while the TX was busy
    output wire        frame_err_o    // a reply frame was dropped on a bad stop bit
);

    wire sel       = cs_i & valid_i;
    wire is_remote = addr_i[14];

    // Rising-edge detect on sel -- see the header note on level vs edge.
    reg  sel_d;
    always @(posedge clk or negedge rst) begin
        if (!rst) sel_d <= 1'b0;
        else      sel_d <= sel;
    end
    wire sel_rise = sel & ~sel_d;

    // ------------------------------------------------------------------
    // LOCAL path: the 2 KB register file. cs_i is tied high; selection is
    // done entirely through valid_i, gated so it only pulses when this
    // transaction is actually LOCAL.
    // ------------------------------------------------------------------
    wire [7:0] local_rdata;
    wire       local_rvalid;

    slave #(
        .ADDR_W (11),      // 2 KB
        .DATA_W (8)
    ) u_local_slave (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (1'b1),
        .valid_i  (sel & ~is_remote),
        .we_i     (we_i),
        .addr_i   (addr_i[10:0]),
        .wdata_i  (wdata_i),
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
    wire [23:0] pkt = { {we_i, 1'b0, addr_i[13:8]},   // byte 0: header
                        addr_i[7:0],                  // byte 1
                        wdata_i };                    // byte 2

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
        .frame_err_o (frame_err_o)
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
            overflow_o     <= 1'b0;
        end else begin
            tx_send  <= 1'b0;
            r_rvalid <= 1'b0;

            case (rstate)
                R_IDLE: begin
                    if (sel_rise && is_remote) begin
                        if (tx_busy) begin
                            overflow_o <= 1'b1;   // nothing to stall with; drop it
                        end else begin
                            tx_data      <= pkt;
                            r_we_latched <= we_i;
                            tx_send      <= 1'b1;
                            rstate       <= R_SEND;
                        end
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
    // Output mux. Only one path can be pulsing rvalid on a given cycle:
    // a remote reply arrives hundreds of cycles after the access that
    // triggered it, long after any local access has answered.
    // ------------------------------------------------------------------
    always @(*) begin
        rvalid_o = local_rvalid | r_rvalid;
        rdata_o  = r_rvalid ? r_rdata : local_rdata;
    end

endmodule
