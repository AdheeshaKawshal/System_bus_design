// master_bridge_top: hardware test for master_bridge's req/grant
// handshake - the master-side counterpart to slave_bridge_top.v.
//
// req_tx_o/grant_rx_i are the real serial-link pins - wire them straight
// to a slave_bridge_top board's req_rx_i/grant_tx_o for an actual
// two-board test. req_btn additionally drives ext_valid_o so the
// reservation can also be triggered manually with no remote board
// attached at all (master_bridge latches a transaction on ext_valid_o's
// rising edge and fires req_tx_o once granted access to the shared
// link), and grant_led taps the same grant_rx_i pin for a visible
// indication alongside whatever's wired to the real link.
//
// master_bridge's own internal side (system_bus's ext_* port) has no
// real bus/master behind it here, so the local link arbiter is
// self-granted (ext_arb_grant_i tied straight to ext_arb_req_o, the same
// trick used elsewhere in this repo for a single, uncontested requester)
// and req_ext is simply tied to ext_valid_o, so the button is effectively
// standing in for that local master too.
//
// grant_rx_i itself needs to be a raw stretched level (master_bridge
// synchronizes and edge-detects it internally) - the real link pin
// already satisfies that from a genuine slave_bridge_top on the other
// end; nothing extra is needed here for that case.
module master_bridge_top (
    input  wire clk,
    input  wire rst_n,

    // Real serial link, wired straight to a slave_bridge_top board
    output wire req_tx_o,
    input  wire grant_rx_i,

    // Manual/local test aids
    input  wire req_btn,     // drives ext_valid_o - lets you trigger a reservation with no remote board
    output wire grant_led    // latched: on once granted, off when req_btn releases
);
    wire rst = !rst_n;

    localparam ADDR_W = 15;
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Internal side: no real system_bus/master here, so the button
    // itself plays both roles - the local master wanting the link
    // (req_ext) and the transaction trigger (ext_valid_o).
    // ---------------------------------------------------------
    wire [ADDR_W+RW-1:0] addr_ext_o  = {(ADDR_W+RW){1'b0}};
    wire [DATA_W-1:0]    wdata_ext_o = {DATA_W{1'b0}};
    wire                 ext_valid_o = req_btn;

    wire [DATA_W-1:0] rdata_ext_i;
    wire              ready_ext_i;
    wire              rvalid_ext_i;

    wire req_ext   = req_btn;
    wire grant_ext;

    wire ext_arb_req_o, ext_arb_grant_i;
    assign ext_arb_grant_i = ext_arb_req_o;

    (* MARK_DEBUG = "TRUE" *) wire req_tx_o_w;
    assign req_tx_o = req_tx_o_w;

    // No real remote status word ever arrives in this demo - tie the
    // return-link UART line idle so its receiver never sees a spurious
    // start bit.
    wire uart_rx_i = 1'b1;

    master_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_master_bridge (
        .clk          (clk),
        .rst          (rst),

        .addr_ext_o   (addr_ext_o),
        .wdata_ext_o  (wdata_ext_o),
        .ext_valid_o  (ext_valid_o),
        .rdata_ext_i  (rdata_ext_i),
        .ready_ext_i  (ready_ext_i),
        .rvalid_ext_i (rvalid_ext_i),
        .req_ext      (req_ext),
        .grant_ext    (grant_ext),

        .ext_arb_req_o   (ext_arb_req_o),
        .ext_arb_grant_i (ext_arb_grant_i),

        .req_tx_o        (req_tx_o_w),
        .grant_rx_i      (grant_rx_i),
        .addr_tx_o       (),
        .wdata_tx_o      (),
        .link_busy_o     (),
        .uart_rx_i       (uart_rx_i),
        .link_rx_valid_o (),
        .ready_rx_i      (1'b0)
    );

    reg grant_led_r;
    always @(posedge clk or negedge rst) begin
        if (!rst) grant_led_r <= 1'b0;
        else if (grant_rx_i) grant_led_r <= 1'b1;
        else if (!req_btn)   grant_led_r <= 1'b0;
    end

    assign grant_led = grant_led_r;

endmodule
