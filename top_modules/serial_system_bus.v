// serial_system_bus: one side of a two-FPGA link. Wraps system_busv1 (its
// own local bus + Master 0 + 3 slaves) with a master_bridge/slave_bridge
// pair and brings the serial link out to real IO pins instead of wiring
// straight to a second bus instance in the same file.
//
// The SAME bitstream/module is meant to be flashed onto both FPGAs: this
// board's tx_* pins wire directly to the other board's rx_* pins and vice
// versa (straight point-to-point crossover, no glue logic needed between
// boards) - so "connecting an external FPGA" just means running this
// exact module on it and swapping tx/rx on the physical link.
//
// system_busv1's ext_* port (master-role: this board reaching onto the
// other board) drives master_bridge; its Master 1 port (slave-role: the
// other board's master reaching onto this board) is driven by
// slave_bridge. Both bridges + the local bus all share this board's own
// clk/rst - only the serial link wires and the req/grant/ready handshake
// lines actually cross to the other board's independent clock domain.
module serial_system_bus (
    input wire clk,
    input wire rst_n,

    // ---------------------------------------------------------
    // Outbound: this board's system_busv1 acting as master onto the other
    // board, via master_bridge. Wire straight to the other board's
    // rx_req_i/tx_grant_o/rx_addr_i/rx_wdata_i/tx_status_o/tx_ready_o.
    // ---------------------------------------------------------
    output wire tx_req_o,
    input  wire rx_grant_i,
    output wire tx_addr_o,
    output wire tx_wdata_o,
    input  wire rx_status_i,
    input  wire rx_ready_i,

    // ---------------------------------------------------------
    // Inbound: the other board's master reaching onto this board's Master
    // 1 port, via slave_bridge. Wire straight to the other board's
    // tx_req_o/rx_grant_i/tx_addr_o/tx_wdata_o/rx_status_i/rx_ready_i.
    // ---------------------------------------------------------
    input  wire rx_req_i,
    output wire tx_grant_o,
    input  wire rx_addr_i,
    input  wire rx_wdata_i,
    output wire tx_status_o,
    output wire tx_ready_o,

    // Physical UART to a PC (via USB-TTL) - drives local Master 1
    // (u_master1 below, now a master_uart) with transactions framed over
    // this link instead of a hardcoded table. system_busv1's own internal
    // Master 0 is unaffected - it's still the canned master.v table -
    // UART_TOP's m0_* interface is tied off below since Master 0 isn't
    // reachable from this file.
    input  wire uart_rx_serial,
    output wire uart_tx_serial,

    output wire [3:0] led
);
    wire rst = !rst_n;

    localparam ADDR_W = 15;
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Clock divider: 125 MHz clk -> a 4 Hz tick, via a free-running
    // counter. tick_4hz pulses for one clk cycle every 1/4 s; not
    // consumed by anything yet.
    // ---------------------------------------------------------
    localparam integer CLK_FREQ_HZ   = 125;//125000000;
    localparam integer TICK_FREQ_HZ  = 40;
    localparam integer DIV_MAX       = CLK_FREQ_HZ / TICK_FREQ_HZ - 1;

    reg [$clog2(DIV_MAX+1)-1:0] div_cnt;
    reg                         tick_4hz;
    wire sready;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            div_cnt  <= 0;
            tick_4hz <= 1'b0;
        end else if (div_cnt == DIV_MAX) begin
            div_cnt  <= 0;
            tick_4hz <= 1'b1;
        end else begin
            div_cnt  <= div_cnt + 1'b1;
            tick_4hz <= 1'b0;
        end
    end

    // ---------------------------------------------------------
    // Local bus (system_busv1's own Master 0 + 3 slaves), exposing its
    // Master 1 (slave-role) socket - shared, via m1_select_mux below,
    // between a second local master and slave_bridge - and its ext_*
    // (master-role) port to master_bridge.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 m1_req, m1_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] m1_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 m1_rvalid;

    (* MARK_DEBUG = "TRUE" *) wire                 ext_req, ext_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] ext_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    ext_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    ext_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_rvalid;

    (* MARK_DEBUG = "TRUE" *) wire [11:0] addr_m;
    (* MARK_DEBUG = "TRUE" *) wire        we_m;
    (* MARK_DEBUG = "TRUE" *) wire        valid_m;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0] wdata_m;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0] rdata_m;

    system_busv1 u_bus (
        .clk          (clk),
        .rst          (rst),

        // Master 1 port (slave-role): the remote board's master, arriving
        // over the serial link via slave_bridge below
        .req_M1       (m1_req),
        .grant_M1     (m1_grant),
        .addr_M1      (m1_addr),
        .wdata_M1     (m1_wdata),
        .valid_M1     (m1_valid),
        .rdata_M1     (m1_rdata),
        .ready_M1     (m1_ready),
        .rvalid_M1    (m1_rvalid),

        // External bus port (master-role): drives master_bridge below
        .req_ext      (ext_req),
        .grant_ext    (ext_grant),
        .addr_ext_o   (ext_addr),
        .wdata_ext_o  (ext_wdata),
        .ext_valid_o  (ext_valid),
        .rdata_ext_i  (ext_rdata),
        .ready_ext_i  (ext_ready),
        .rvalid_ext_i (ext_rvalid),
        .sready       (sready),
        .addr_m       (addr_m),
        .we_m         (we_m),
        .valid_m      (valid_m),
        .wdata_m      (wdata_m),
        .rdata_m      (rdata_m)
    );

    // Only one local master (system_busv1's ext_* port) ever wants the
    // outbound serial link, so it can just grant itself (same trick
    // bus_interconnect_serial.v uses for its own per-bridge local arbiter).
    wire link_arb_req, link_arb_grant;
    assign link_arb_grant = link_arb_req;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge (
        .clk          (clk),
        .rst          (rst),

        .addr_ext_o   (ext_addr),
        .wdata_ext_o  (ext_wdata),
        .ext_valid_o  (ext_valid),
        .rdata_ext_i  (ext_rdata),
        .ready_ext_i  (ext_ready),
        .rvalid_ext_i (ext_rvalid),
        .req_ext      (ext_req),
        .grant_ext    (ext_grant),

        .ext_arb_req_o   (link_arb_req),
        .ext_arb_grant_i (link_arb_grant),

        // Physical serial link out to the other board
        .req_tx_o        (tx_req_o),
        .grant_rx_i      (rx_grant_i),
        .addr_tx_o       (tx_addr_o),
        .wdata_tx_o      (tx_wdata_o),
        .link_busy_o     (),
        .uart_rx_i       (rx_status_i),
        .link_rx_valid_o (),
        .ready_rx_i      (rx_ready_i)
    );

    // slave_bridge's own side of the shared Master 1 port (source B into
    // m1_select_mux below)
    (* MARK_DEBUG = "TRUE" *) wire                 sb_req, sb_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] sb_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    sb_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    sb_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 sb_rvalid;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge (
        .clk  (clk),
        .rst  (rst),

        .req_o    (sb_req),
        .grant_i  (sb_grant),
        .addr_o   (sb_addr),
        .wdata_o  (sb_wdata),
        .valid_o  (sb_valid),
        .rdata_i  (sb_rdata),
        .ready_i  (sb_ready),
        .rvalid_i (sb_rvalid),

        // Physical serial link in from the other board
        .req_rx_i       (rx_req_i),
        .grant_tx_o     (tx_grant_o),
        .addr_rx_i      (rx_addr_i),
        .wdata_rx_i     (rx_wdata_i),
        .link_rx_busy_o (),
        .rdata_tx_o     (tx_status_o),
        .link_tx_busy_o (),
        .ready_tx_o     (tx_ready_o)
    );

    // ---------------------------------------------------------
    // Second local master (source A into m1_select_mux below) - drives
    // this board's own local bus directly through the shared Master 1
    // port, same as the remote board's master does via slave_bridge.
    // Now a master_uart: transactions come from UART_TOP (over the real
    // uart_rx_serial/uart_tx_serial pins) instead of a hardcoded table.
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_req, m1L_grant;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] m1L_addr;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1L_wdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_valid;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    m1L_rdata;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_ready;
    (* MARK_DEBUG = "TRUE" *) wire                 m1L_rvalid;

    wire                 m1_txn_valid, m1_txn_we, m1_txn_ready;
    wire [ADDR_W-1:0]    m1_txn_addr;
    wire [DATA_W-1:0]    m1_txn_wdata;
    wire [DATA_W-1:0]    m1u_rdata;
    wire                 m1u_rdata_valid, m1_txn_done;

    master_uart #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RW        (RW),
        .REQ_DELAY (4)
    ) u_master1 (
        .clk         (clk),
        .rst         (rst),
        .req_o       (m1L_req),
        .grant_i     (m1L_grant),
        .addr_o      (m1L_addr),
        .wdata_o     (m1L_wdata),
        .valid_o     (m1L_valid),
        .rdata_i     (m1L_rdata),
        .ready_i     (m1L_ready),
        .rvalid_i    (m1L_rvalid),
        .ext_valid_o (m1L_valid),

        .txn_valid_i   (m1_txn_valid),
        .txn_addr_i    (m1_txn_addr),
        .txn_we_i      (m1_txn_we),
        .txn_wdata_i   (m1_txn_wdata),
        .txn_ready_o   (m1_txn_ready),
        .rdata_o       (m1u_rdata),
        .rdata_valid_o (m1u_rdata_valid),
        .txn_done_o    (m1_txn_done)
    );

    // UART_TOP's m0_* interface has nothing to connect to here -
    // system_busv1's own internal Master 0 still runs off master.v's
    // hardcoded table and isn't reachable from this file - so it's tied
    // permanently "ready" (never blocks dispatch of an m1-only or
    // broadcast frame) with no read response ever coming back.
    UART_TOP #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W)
    ) u_uart_top (
        .clk         (clk),
        .rst         (rst),

        .i_Rx_Serial (uart_rx_serial),
        .o_Tx_Serial (uart_tx_serial),

        .m0_txn_valid_o   (),
        .m0_txn_addr_o    (),
        .m0_txn_we_o      (),
        .m0_txn_wdata_o   (),
        .m0_txn_ready_i   (1'b1),
        .m0_rdata_i       ({DATA_W{1'b0}}),
        .m0_rdata_valid_i (1'b0),
        .m0_txn_done_i    (1'b0),

        .m1_txn_valid_o   (m1_txn_valid),
        .m1_txn_addr_o    (m1_txn_addr),
        .m1_txn_we_o      (m1_txn_we),
        .m1_txn_wdata_o   (m1_txn_wdata),
        .m1_txn_ready_i   (m1_txn_ready),
        .m1_rdata_i       (m1u_rdata),
        .m1_rdata_valid_i (m1u_rdata_valid),
        .m1_txn_done_i    (m1_txn_done)
    );

    // sel picks which of the two (local master1, or slave_bridge relaying
    // the remote board's master) drives system_busv1's one physical
    // Master 1 socket. Local master gets priority whenever it wants the
    // bus; the choice only changes while the shared port is idle
    // (m1_valid low) so an in-flight transaction is never yanked
    // mid-transfer.

    m1_select_mux #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_m1_select_mux (
        .sel (1'b1),

        .reqA    (m1L_req),
        .grantA  (m1L_grant),
        .addrA   (m1L_addr),
        .wdataA  (m1L_wdata),
        .validA  (m1L_valid),
        .rdataA  (m1L_rdata),
        .readyA  (m1L_ready),
        .rvalidA (m1L_rvalid),

        .reqB    (sb_req),
        .grantB  (sb_grant),
        .addrB   (sb_addr),
        .wdataB  (sb_wdata),
        .validB  (sb_valid),
        .rdataB  (sb_rdata),
        .readyB  (sb_ready),
        .rvalidB (sb_rvalid),

        .req_m1    (m1_req),
        .grant_m1  (m1_grant),
        .addr_m1   (m1_addr),
        .wdata_m1  (m1_wdata),
        .valid_m1  (m1_valid),
        .rdata_m1  (m1_rdata),
        .ready_m1  (m1_ready),
        .rvalid_m1 (m1_rvalid)
    );

    // Shows the lower 4 bits of whatever data last moved on this board's
    // own local bus - we_m/valid_m/wdata_m/rdata_m/sready are all local-bus
    // signals now, instead of mixing sready (a local-slave-completion
    // signal) with ext_valid/ext_addr/ext_wdata/ext_rdata, which belong to
    // the outbound external port instead - see led_display.v's header.
    led_display #(
        .DATA_W (DATA_W)
    ) u_led_display (
        .clk      (clk),
        .rst      (rst),
        .we_i     (we_m),
        .valid_i  (valid_m),
        .ready_i  (sready),
        .wdata_i  (wdata_m),
        .rdata_i  (rdata_m),
        .led      (led)
    );

endmodule
