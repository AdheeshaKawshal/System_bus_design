// slave_bridge: mirror of master_bridge. Receives a transaction over the
// two serial lines a master_bridge sends (addr_rx_i <- addr_tx_o,
// wdata_rx_i <- wdata_tx_o), reassembles it, and drives it onto the local
// bus as a master - req_o/grant_i/addr_o/wdata_o/valid_o/rdata_i/ready_i/
// rvalid_i deliberately match master.v's port shape so this drops straight
// into a system_bus master port (req_M0/grant_M0/...).
//
// The response is packed back into the same single {rvalid, ready, rdata}
// word master_bridge decodes on its uart_rx_i, and sent out status_tx_o.
module slave_bridge #(
    parameter ADDR_W           = 15,
    parameter DATA_W           = 8,
    parameter RW                = 1,
    parameter CLOCKS_PER_PULSE = 2
)(
    input  wire clk,
    input  wire rst,

    // serial link (connect to the remote master_bridge)
    input  wire                 addr_rx_i,
    input  wire                 wdata_rx_i,
    output wire                 link_rx_busy_o,   // high while a partial command is being assembled
    output wire                 status_tx_o,
    output wire                 link_tx_busy_o,   // high while the status+rdata word is shifting out

    // local bus interface - same shape as master.v, plugs into a
    // system_bus master port (req_M0/grant_M0/addr_M0/... )
    output reg                  req_o,
    input  wire                 grant_i,
    output reg  [ADDR_W+RW-1:0] addr_o,
    output reg  [DATA_W-1:0]    wdata_o,
    output reg                  valid_o,

    input  wire [DATA_W-1:0]    rdata_i,
    input  wire                 ready_i,
    input  wire                 rvalid_i
);

    localparam S_IDLE       = 2'd0,
               S_WAIT_GRANT = 2'd1,
               S_ACTIVE     = 2'd2;

    reg [1:0] state;
    assign link_rx_busy_o = (state != S_IDLE);

    wire                      addr_rx_ready;
    wire [ADDR_W+RW-1:0]      addr_rx_word;
    wire                      wdata_rx_ready;
    wire [DATA_W-1:0]         wdata_rx_word;

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (ADDR_W + RW)
    ) addr_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (addr_rx_i),
        .ready    (addr_rx_ready),
        .data_out (addr_rx_word)
    );

    uart_rx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W)
    ) wdata_rx (
        .clk      (clk),
        .rstn     (rst),
        .rx       (wdata_rx_i),
        .ready    (wdata_rx_ready),
        .data_out (wdata_rx_word)
    );

    // addr and wdata arrive on separate lines and, since they're different
    // widths, complete at different times - latch each independently and
    // only drive the transaction once both halves have arrived.
    reg addr_got, wdata_got;
    reg [ADDR_W+RW-1:0] addr_word_r;
    reg [DATA_W-1:0]    wdata_word_r;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state        <= S_IDLE;
            addr_got     <= 1'b0;
            wdata_got    <= 1'b0;
            addr_word_r  <= {(ADDR_W+RW){1'b0}};
            wdata_word_r <= {DATA_W{1'b0}};
            req_o        <= 1'b0;
            valid_o      <= 1'b0;
            addr_o       <= {(ADDR_W+RW){1'b0}};
            wdata_o      <= {DATA_W{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (addr_rx_ready)  begin addr_word_r  <= addr_rx_word;  addr_got  <= 1'b1; end
                    if (wdata_rx_ready) begin wdata_word_r <= wdata_rx_word; wdata_got <= 1'b1; end

                    if ((addr_got || addr_rx_ready) && (wdata_got || wdata_rx_ready)) begin
                        addr_o    <= addr_rx_ready  ? addr_rx_word  : addr_word_r;
                        wdata_o   <= wdata_rx_ready ? wdata_rx_word : wdata_word_r;
                        req_o     <= 1'b1;
                        addr_got  <= 1'b0;
                        wdata_got <= 1'b0;
                        state     <= S_WAIT_GRANT;
                    end
                end

                S_WAIT_GRANT: begin
                    if (grant_i) begin
                        valid_o <= 1'b1;
                        state   <= S_ACTIVE;
                    end
                end

                S_ACTIVE: begin
                    if (ready_i) begin
                        valid_o <= 1'b0;
                        req_o   <= 1'b0;
                        state   <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Fire the status word the same cycle ready_i lands, packed identically
    // to what master_bridge expects on its uart_rx_i: {rvalid, ready, rdata}.
    wire tx_data_en = (state == S_ACTIVE) && ready_i;
    wire [DATA_W+1:0] status_word = {rvalid_i, ready_i, rdata_i};

    uart_tx #(
        .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
        .DATA_WIDTH       (DATA_W + 2)
    ) status_tx (
        .data_in  (status_word),
        .data_en  (tx_data_en),
        .clk      (clk),
        .rstn     (rst),
        .tx       (status_tx_o),
        .tx_busy  (link_tx_busy_o)
    );

endmodule
