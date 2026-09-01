// top_bridge_test: minimal single-FPGA loopback to exercise
// master_bridge <-> slave_bridge over the serial link without a second
// full system_bus on the receiving end.
//
// master.v drives master_bridge directly (its port shape already matches
// a system_bus ext_* port one-for-one - see top_module.v for the same
// wiring pattern). master_bridge's serial lines loop straight into a
// slave_bridge in the same clock domain (no cross-clock link needed for
// this test).
//
// slave_bridge's internal side normally plugs into a system_bus master
// port; here it's answered by a tiny always-grant, one-cycle-latency
// memory responder instead of instantiating a real bus + slave, since the
// goal is just to prove the two bridges hand a transaction back and forth
// correctly.
module top_bridge_test (
    input wire clk,
    input wire rst_n
);
    wire rst = !rst_n;

    localparam ADDR_W = 15;
    localparam DATA_W = 8;
    localparam RW     = 1;

    // ---------------------------------------------------------
    // Master 0 -> master_bridge
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 req_ext, grant_ext;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] addr_ext_o;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    wdata_ext_o;
    (* MARK_DEBUG = "TRUE" *) wire                 ext_valid_o;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    rdata_ext_i;
    (* MARK_DEBUG = "TRUE" *) wire                 ready_ext_i;
    (* MARK_DEBUG = "TRUE" *) wire                 rvalid_ext_i;

    // ACTIVE_TIMEOUT bumped well past master.v's default 16: a round trip
    // through both bridges (reservation handshake + two serialized UART
    // words each way) takes well over 100 cycles even at CLOCKS_PER_PULSE
    // =2, so the default would just make the master time out and retry
    // forever instead of ever seeing a real completion.
    master #(
        .ADDR_W        (ADDR_W),
        .DATA_W        (DATA_W),
        .RW            (RW),
        .REQ_DELAY     (4),
        .ACTIVE_TIMEOUT(2000)
    ) u_master0 (
        .clk         (clk),
        .rst         (rst),
        .req_o       (req_ext),
        .grant_i     (grant_ext),
        .addr_o      (addr_ext_o),
        .wdata_o     (wdata_ext_o),
        .valid_o     (ext_valid_o),
        .rdata_i     (rdata_ext_i),
        .ready_i     (ready_ext_i),
        .rvalid_i    (rvalid_ext_i),
        .ext_valid_o (ext_valid_o)
    );

    // Only one master ever wants the serial link, so it can just grant
    // itself (same trick bus_interconnect_serial.v uses for its own
    // per-bridge local arbiter).
    wire link_arb_req, link_arb_grant;
    assign link_arb_grant = link_arb_req;

    // Serial link wires between the two bridges (single clock domain here)
    (* MARK_DEBUG = "TRUE" *) wire link_req_tx, link_grant_tx;
    wire link_addr_tx, link_wdata_tx, link_status_tx, link_ready_tx;

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

        .ext_arb_req_o   (link_arb_req),
        .ext_arb_grant_i (link_arb_grant),

        .req_tx_o        (link_req_tx),
        .grant_rx_i      (link_grant_tx),
        .addr_tx_o       (link_addr_tx),
        .wdata_tx_o      (link_wdata_tx),
        .link_busy_o     (),
        .uart_rx_i       (link_status_tx),
        .link_rx_valid_o (),
        .ready_rx_i      (link_ready_tx)
    );

    // ---------------------------------------------------------
    // slave_bridge - internal side answered by simple logic below instead
    // of a real system_bus master port
    // ---------------------------------------------------------
    (* MARK_DEBUG = "TRUE" *) wire                 req_s, grant_s;
    (* MARK_DEBUG = "TRUE" *) wire [ADDR_W+RW-1:0] addr_s;
    (* MARK_DEBUG = "TRUE" *) wire [DATA_W-1:0]    wdata_s;
    (* MARK_DEBUG = "TRUE" *) wire                 valid_s;
    (* MARK_DEBUG = "TRUE" *) reg  [DATA_W-1:0]    rdata_s;
    (* MARK_DEBUG = "TRUE" *) reg                  ready_s;
    (* MARK_DEBUG = "TRUE" *) reg                  rvalid_s;

    slave_bridge #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RW     (RW)
    ) u_slave_bridge (
        .clk  (clk),
        .rst  (rst),

        .req_o    (req_s),
        .grant_i  (grant_s),
        .addr_o   (addr_s),
        .wdata_o  (wdata_s),
        .valid_o  (valid_s),
        .rdata_i  (rdata_s),
        .ready_i  (ready_s),
        .rvalid_i (rvalid_s),

        .req_rx_i       (link_req_tx),
        .grant_tx_o     (link_grant_tx),
        .addr_rx_i      (link_addr_tx),
        .wdata_rx_i     (link_wdata_tx),
        .link_rx_busy_o (),
        .rdata_tx_o     (link_status_tx),
        .link_tx_busy_o (),
        .ready_tx_o     (link_ready_tx)
    );

    // Always grant - this is the only master on the "bus" the slave_bridge
    // sees.
    assign grant_s = req_s;

    // ---------------------------------------------------------
    // Simple slave logic: a tiny 16-entry memory (addr_s[4:1] as index,
    // addr_s[0] as we) that answers one cycle after valid_o rises, same
    // start-pulse pattern master_bridge uses for tx_start_en.
    // ---------------------------------------------------------
    reg [DATA_W-1:0] smem [0:15];

    reg valid_s_d;
    always @(posedge clk or negedge rst)
        if (!rst) valid_s_d <= 1'b0;
        else      valid_s_d <= valid_s;

    (* MARK_DEBUG = "TRUE" *) wire start_en = valid_s && !valid_s_d;
    (* MARK_DEBUG = "TRUE" *) wire we_s     = addr_s[0];
    (* MARK_DEBUG = "TRUE" *) wire [3:0] idx = addr_s[4:1];

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_s  <= {DATA_W{1'b0}};
            ready_s  <= 1'b0;
            rvalid_s <= 1'b0;
        end else begin
            ready_s  <= 1'b0;
            rvalid_s <= 1'b0;
            if (start_en) begin
                ready_s <= 1'b1;
                if (we_s) begin
                    smem[idx] <= wdata_s;
                end else begin
                    rdata_s  <= smem[idx];
                    rvalid_s <= 1'b1;
                end
            end
        end
    end

endmodule
