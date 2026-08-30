// master_bridge: forwards system_bus external-port transactions across a
// UART serial link. Address (+we) and wdata are each sent whole in one
// uart_tx word (DATA_WIDTH configured to ADDR_W+RW / DATA_W - no manual
// byte-splitting needed) over their own serial line, in parallel.
//
// The return link carries rdata plus the remote slave's ready/rvalid
// status packed into a single uart_rx word, so ready and rvalid stay
// independently visible instead of being tied together.
//
// link_busy_o / link_rx_valid_o are exposed so anything wiring this bridge
// to an external bus/monitor can see TX/RX activity without reaching
// inside the module.
module master_bridge #(
    parameter ADDR_W           = 15,
    parameter DATA_W           = 8,
    parameter RW                = 1,
    parameter CLOCKS_PER_PULSE = 2
)(
    input  wire clk,
    input  wire rst,

    // system_bus external-port interface
    input  wire [ADDR_W+RW-1:0] addr_ext_o,
    input  wire [DATA_W-1:0]    wdata_ext_o,
    input  wire                 ext_valid_o,

    output wire [DATA_W-1:0]    rdata_ext_i,
    output wire                 ready_ext_i,
    output wire                 rvalid_ext_i,

    input  wire                 req_ext,
    output wire                 grant_ext,

    // serial link (visible externally for connection to the remote bridge
    // / for monitoring)
    output wire                 addr_tx_o,
    output wire                 wdata_tx_o,
    output wire                 link_busy_o,      // high while addr/wdata still shifting out
    input  wire                 uart_rx_i,
    output wire                 link_rx_valid_o,  // pulses when a status+rdata word has arrived

    // external arbiter interface: this bridge contends for the shared
    // serial link the same way a master contends for the local bus
    output wire                 ext_arb_req_o,
    input  wire                 ext_arb_grant_i
);

    localparam RX_WIDTH = DATA_W + 2;   // {rvalid, ready, rdata}

    // pulse data_en for one cycle on ext_valid_o's rising edge so a
    // held-high valid doesn't retrigger the uart_tx once it goes idle again
    reg ext_valid_o_d;
    always @(posedge clk or negedge rst)
        if (!rst) ext_valid_o_d <= 1'b0;
        else      ext_valid_o_d <= ext_valid_o;

    wire tx_data_en = ext_valid_o && !ext_valid_o_d;

    // Forward the local request out to the external arbiter, and only hand
    // the local bus a grant once the shared link itself has been granted -
    // this is a real arbitration point, not an auto-grant, since other
    // bridges/masters may be contending for the same serial link.
    assign ext_arb_req_o = req_ext;
    assign grant_ext     = req_ext && ext_arb_grant_i;

    wire addr_tx_busy, wdata_tx_busy;
    assign link_busy_o = addr_tx_busy | wdata_tx_busy;

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (ADDR_W + RW)
    ) addr_tx (
        .data_in  (addr_ext_o),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (addr_tx_o),
        .tx_busy  (addr_tx_busy)
    );

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W)
    ) wdata_tx (
        .data_in  (wdata_ext_o),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (wdata_tx_o),
        .tx_busy  (wdata_tx_busy)
    );

    wire [RX_WIDTH-1:0] rx_word;

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (RX_WIDTH)
    ) status_rdata_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (uart_rx_i),
        .ready    (link_rx_valid_o),
        .data_out (rx_word)
    );

    // rx_word (uart_rx's data_out) is really its internal shift register -
    // it changes bit by bit while a word is still coming in, so it must
    // only be sampled once link_rx_valid_o (the completed serial-to-
    // parallel conversion) actually pulses. Latch it there and hold
    // ready/rvalid high for exactly that one cycle, matching the pulse
    // semantics master.v's ACTIVE state expects on ready_i.
    reg [DATA_W-1:0] rdata_ext_i_r;
    reg              ready_ext_i_r, rvalid_ext_i_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_ext_i_r  <= {DATA_W{1'b0}};
            ready_ext_i_r  <= 1'b0;
            rvalid_ext_i_r <= 1'b0;
        end else if (link_rx_valid_o) begin
            rdata_ext_i_r  <= rx_word[DATA_W-1:0];
            ready_ext_i_r  <= rx_word[DATA_W];
            rvalid_ext_i_r <= rx_word[DATA_W+1];
        end else begin
            ready_ext_i_r  <= 1'b0;
            rvalid_ext_i_r <= 1'b0;
        end
    end

    assign rdata_ext_i  = rdata_ext_i_r;
    assign ready_ext_i  = ready_ext_i_r;
    assign rvalid_ext_i = rvalid_ext_i_r;

endmodule
