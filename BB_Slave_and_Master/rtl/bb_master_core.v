// ============================================================================
// bb_master_core.v
// ----------------------------------------------------------------------------
// Master-side bridge core. Instantiates master.v (this folder's copy, which is
// driven purely by its external transaction port) as the actual
// req/grant/addr/wdata/valid <-> rdata/rvalid FSM. This core's own job is only
// the UART side:
//   1. Slice the incoming 24-bit request frame into transaction fields.
//      uart_frame_rx (WIDTH = 24) delivers the whole frame at once, so there
//      is no reassembly state left in this module.
//   2. Hold one (and only one) transaction if master.v is busy,
//      flagging overflow if a second one lands on top of it.
//   3. Feed the held/fresh transaction into master.v's external port.
//   4. When master.v reports a completed read, ship the single reply byte
//      back out over UART. Writes are fire-and-forget -- no reply byte.
//
// Bus-facing ports (req_o/grant_i/addr_o/wdata_o/valid_o/rdata_i/rvalid_i) are
// simple pass-throughs of master.v's own arbiter + bus ports. There is no
// ready_i: slaves return only rdata + rvalid.
//
// UART packet -> transaction unpacking, matching bb_slave_core's REMOTE
// packet layout exactly (one 24-bit frame):
//   pkt[23:16] = { rw, 1'b0, addr[13:8] }
//   pkt[15:8]  = addr[7:0]
//   pkt[7:0]   = wdata[7:0]
// addr[14] arrives implicitly as 0 (tied); addr[13:0] is passed through
// untouched so the receiving bus's address decoder still sees the right
// slave-select bits.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module bb_master_core #(
    parameter CLK_FREQ_HZ    = 125000000,
    parameter BAUD_RATE      = 100000,
    parameter ADDR_W         = 15,
    parameter DATA_W         = 8,
    parameter RW             = 1,
    parameter ACTIVE_TIMEOUT = 64     // passed through to master.v (read rvalid watchdog)
)(
    input  wire                 clk,
    input  wire                 rst,        // active-low

    // ---- UART pins from/to the far-side bb_slave_core -------------------
    input  wire                 uart_rx_i,  // request packets in
    output wire                 uart_tx_o,  // read-reply bytes out

    // ---- bus-facing master port (pass-through of master.v) ---------------
    output wire                  req_o,
    input  wire                  grant_i,

    output wire [ADDR_W+RW-1:0]  addr_o,     // {addr, we}, we in the LSB
    output wire [DATA_W-1:0]     wdata_o,
    output wire                  valid_o,

    input  wire [DATA_W-1:0]     rdata_i,
    input  wire                  rvalid_i,

    // ---- observation ports ------------------------------------------------
    output reg                   overflow_o,  // sticky: a 2nd packet landed on an already-held one
    output wire                  frame_err_o  // sticky: a request frame was dropped on a bad stop bit
);

    // ------------------------------------------------------------------
    // Block 1: UART RX -> unpacked transaction fields
    //
    // uart_frame_rx (WIDTH = 24) hands over the whole frame at once, so all
    // that is left here is slicing the fields out of it -- pure wiring, no
    // state.
    // ------------------------------------------------------------------
    wire [23:0] rx_frame;
    wire        pkt_ready;   // 1-cycle pulse: a whole request frame has arrived

    uart_frame_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .WIDTH       (24)        // the whole request arrives as one frame
    ) u_req_rx (
        .clk         (clk),
        .rst         (rst),
        .rx_i        (uart_rx_i),
        .data_o      (rx_frame),
        .valid_o     (pkt_ready),
        .frame_err_o (frame_err_o)
    );

    // rx_frame[23:16] = header {rw, 1'b0, addr[13:8]}
    // rx_frame[15:8]  = addr[7:0]
    // rx_frame[7:0]   = wdata
    wire        pkt_we    = rx_frame[23];
    wire [13:0] pkt_addr  = {rx_frame[21:16], rx_frame[15:8]};
    wire [7:0]  pkt_wdata = rx_frame[7:0];

    // ------------------------------------------------------------------
    // Block 2: 1-deep hold register + sticky overflow flag
    // ------------------------------------------------------------------
    wire m_txn_ready;   // master.v's txn_ready_o -- high whenever it can accept a transaction

    reg        hold_valid;
    reg        hold_we;
    reg [13:0] hold_addr;
    reg [7:0]  hold_wdata;

    // A fresh packet goes straight through only when master.v is free AND
    // nothing is already queued behind it.
    wire pass_through   = pkt_ready && m_txn_ready && !hold_valid;
    // The held transaction is consumed whenever master.v is free (it takes
    // priority over a fresh packet arriving the same cycle).
    wire hold_consumed  = hold_valid && m_txn_ready;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            hold_valid <= 1'b0;
            hold_we    <= 1'b0;
            hold_addr  <= 14'h0;
            hold_wdata <= 8'h00;
            overflow_o <= 1'b0;
        end else begin
            if (hold_consumed)
                hold_valid <= 1'b0;

            if (pkt_ready && !pass_through) begin
                // Free the slot in the same cycle it is being consumed, so a
                // packet arriving exactly then still gets held rather than
                // being counted as an overflow.
                if (!hold_valid || hold_consumed) begin
                    hold_valid <= 1'b1;
                    hold_we    <= pkt_we;
                    hold_addr  <= pkt_addr;
                    hold_wdata <= pkt_wdata;
                end else begin
                    // a transaction is already waiting and a second one just
                    // landed on top of it: flag it, don't overwrite
                    overflow_o <= 1'b1;   // sticky, only cleared by rst
                end
            end
        end
    end

    // Transaction actually presented to master.v this cycle: the held one if
    // there is one, otherwise a fresh packet.
    wire        txn_valid = m_txn_ready && (hold_valid || pkt_ready);
    wire        txn_we    = hold_valid ? hold_we    : pkt_we;
    wire [13:0] txn_addr  = hold_valid ? hold_addr  : pkt_addr;
    wire [7:0]  txn_wdata = hold_valid ? hold_wdata : pkt_wdata;

    // ------------------------------------------------------------------
    // Block 3: master.v, driven through its external transaction port.
    // addr[14] is forced to 0 -- see the packet layout note above.
    // ------------------------------------------------------------------
    wire [7:0] m_txn_rdata;
    wire       m_txn_rdata_valid;
    wire       m_txn_done;

    master #(
        .ADDR_W         (ADDR_W),
        .DATA_W         (DATA_W),
        .RW             (RW),
        .ACTIVE_TIMEOUT (ACTIVE_TIMEOUT)
    ) u_master (
        .clk               (clk),
        .rst               (rst),
        .req_o             (req_o),
        .grant_i           (grant_i),
        .addr_o            (addr_o),
        .wdata_o           (wdata_o),
        .valid_o           (valid_o),
        .rdata_i           (rdata_i),
        .rvalid_i          (rvalid_i),
        .txn_valid_i       (txn_valid),
        .txn_addr_i        ({1'b0, txn_addr}),   // addr[14]=0, addr[13:0]=txn_addr
        .txn_we_i          (txn_we),
        .txn_wdata_i       (txn_wdata),
        .txn_ready_o       (m_txn_ready),
        .txn_rdata_o       (m_txn_rdata),
        .txn_rdata_valid_o (m_txn_rdata_valid),
        .txn_done_o        (m_txn_done)
    );

    // ------------------------------------------------------------------
    // Block 4: read-reply byte -> UART TX (fire-and-forget on writes)
    // ------------------------------------------------------------------
    reg [7:0] rd_byte;
    reg       rd_byte_valid;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rd_byte       <= 8'h00;
            rd_byte_valid <= 1'b0;
        end else begin
            rd_byte_valid <= 1'b0;
            if (m_txn_rdata_valid) begin
                rd_byte       <= m_txn_rdata;
                rd_byte_valid <= 1'b1;
            end
        end
    end

    uart_frame_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .WIDTH       (8)         // the reply is a single rdata byte
    ) u_reply_tx (
        .clk    (clk),
        .rst    (rst),
        .data_i (rd_byte),
        .send_i (rd_byte_valid),
        .tx_o   (uart_tx_o),
        .busy_o (),
        .done_o ()
    );

endmodule
