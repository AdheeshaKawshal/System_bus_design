module slave_bridge_top (
    input  wire clk,
    input  wire rst_n,

    // Real serial link, wired straight to a master_bridge_top board
    input  wire req_rx_i,
    output wire grant_tx_o,

    // Manual/local test aids
    input  wire req_btn,     // OR'd into req_rx_i - lets you trigger the handshake with no remote board
    output wire grant_led    // latched: on once granted, off when req_btn releases
);
    wire rst = !rst_n;

    localparam ADDR_W = 15;
    localparam DATA_W = 8;
    localparam RW     = 1;

    wire req_rx_effective = req_rx_i | req_btn;

    // ---------------------------------------------------------
    // Local-bus side: no real bus here, so self-grant the only requester
    // (mirrors the ext_arb_grant_i = ext_arb_req_o trick used for
    // master_bridge elsewhere in this repo).
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 req_o, grant_i;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] addr_o;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    wdata_o;
    (* MARK_DEBUG = "TRUE" *) wire                 valid_o;

    assign grant_i = req_o;

    // No real slave to answer a completed transaction with in this
    // demo - only the req/grant handshake is under test here.
    wire [DATA_W-1:0] rdata_i  = {DATA_W{1'b0}};
    wire              ready_i  = 1'b0;
    wire              rvalid_i = 1'b0;

    (* MARK_DEBUG = "TRUE" *) wire grant_tx_o_w;
    assign grant_tx_o = grant_tx_o_w;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge (
        .clk  (clk),
        .rst  (rst),

        .req_o    (req_o),
        .grant_i  (grant_i),
        .addr_o   (addr_o),
        .wdata_o  (wdata_o),
        .valid_o  (valid_o),
        .rdata_i  (rdata_i),
        .ready_i  (ready_i),
        .rvalid_i (rvalid_i),

        .req_rx_i       (req_rx_effective),
        .grant_tx_o     (grant_tx_o_w),
        .addr_rx_i      (1'b1),   // uart_rx idle level - no address word is ever sent in this demo
        .wdata_rx_i     (1'b1),   // uart_rx idle level - no data word is ever sent in this demo
        .link_rx_busy_o (),
        .rdata_tx_o     (),
        .link_tx_busy_o (),
        .ready_tx_o     ()
    );

    reg grant_led_r;
    always @(posedge clk or negedge rst) begin
        if (!rst) grant_led_r <= 1'b0;
        else if (grant_tx_o_w) grant_led_r <= 1'b1;
        else if (!req_btn)     grant_led_r <= 1'b0;
    end

    assign grant_led = grant_led_r;

endmodule
