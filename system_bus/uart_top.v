module UART_TOP #(
    parameter CLKS_PER_BIT = 13021,   // must match receiver/transmitter baud setting
    parameter ADDR_W       = 15,
    parameter DATA_W       = 8
)(
    input  wire clk,
    input  wire rst,

    // Physical UART lines to the PC (via USB-TTL)
    input  wire i_Rx_Serial,
    output wire o_Tx_Serial,

    // ---------------- Master 0 transaction interface ----------------
    output reg                m0_txn_valid_o,
    output reg  [ADDR_W-1:0]  m0_txn_addr_o,
    output reg                m0_txn_we_o,
    output reg  [DATA_W-1:0]  m0_txn_wdata_o,
    input  wire               m0_txn_ready_i,
    input  wire [DATA_W-1:0]  m0_rdata_i,
    input  wire                m0_rdata_valid_i,
    input  wire                m0_txn_done_i,   // unused here (writes need no response) but exposed for debug/monitoring

    // ---------------- Master 1 transaction interface ----------------
    output reg                m1_txn_valid_o,
    output reg  [ADDR_W-1:0]  m1_txn_addr_o,
    output reg                m1_txn_we_o,
    output reg  [DATA_W-1:0]  m1_txn_wdata_o,
    input  wire               m1_txn_ready_i,
    input  wire [DATA_W-1:0]  m1_rdata_i,
    input  wire                m1_rdata_valid_i,
    input  wire                m1_txn_done_i
);

    // ------------------------------------------------------------------
    // UART physical layer (unmodified receiver/transmitter)
    // ------------------------------------------------------------------
    wire        w_Rx_DV;
    wire [47:0] w_Rx_Word;

    receiver #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_receiver (
        .i_Clock     (clk),
        .i_Rx_Serial (i_Rx_Serial),
        .o_Rx_DV     (w_Rx_DV),
        .o_Rx_Word   (w_Rx_Word)
    );

    reg        r_Tx_DV;
    reg [39:0] r_Tx_Word;
    wire       w_Tx_Active;
    wire       w_Tx_Done;

    transmitter #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_transmitter (
        .i_Clock     (clk),
        .i_Tx_DV     (r_Tx_DV),
        .i_Tx_Word   (r_Tx_Word),
        .o_Tx_Active (w_Tx_Active),
        .o_Tx_Serial (o_Tx_Serial),
        .o_Tx_Done   (w_Tx_Done)
    );

    // ------------------------------------------------------------------
    // Request-frame layout, 6 bytes = o_Rx_Word[47:0], byte0 = bits[7:0]:
    //   byte0 [7:0]   = 8'hAA  (start marker)
    //   byte1 [15:8]  = { master_sel[1:0], we, addr[14:10] }
    //   byte2 [23:16] = addr[9:2]
    //   byte3 [31:24] = { addr[1:0], 6'b0 }
    //   byte4 [39:32] = wdata   (ignored/dummy on a read)
    //   byte5 [47:40] = 8'h55  (end marker)
    // master_sel: 00 = master0, 01 = master1, 11 = both (broadcast)
    // ------------------------------------------------------------------
    wire        frame_ok = (w_Rx_Word[7:0] == 8'hAA) && (w_Rx_Word[47:40] == 8'h55);
    wire [1:0]  rx_sel   = w_Rx_Word[15:14];
    wire        rx_we    = w_Rx_Word[13];
    wire [14:0] rx_addr  = {w_Rx_Word[12:8], w_Rx_Word[23:16], w_Rx_Word[31:30]};
    wire [7:0]  rx_wdata = w_Rx_Word[39:32];

    // Latched pending request, waiting for its target master(s) to be
    // ready before being dispatched.
    reg        pend_valid;
    reg [1:0]  pend_sel;
    reg        pend_we;
    reg [14:0] pend_addr;
    reg [7:0]  pend_wdata;

    wire need_m0 = pend_valid && ((pend_sel == 2'b00) || (pend_sel == 2'b11));
    wire need_m1 = pend_valid && ((pend_sel == 2'b01) || (pend_sel == 2'b11));

    // One response slot per master. A master can have at most one
    // outstanding transaction (gated by its own txn_ready_o), so a
    // single valid/data pair per master is enough.
    reg        resp0_valid;
    reg [7:0]  resp0_data;
    reg        resp1_valid;
    reg [7:0]  resp1_data;

    // Dispatch only once every master the request targets reports ready,
    // AND only once any previously captured (but not yet transmitted)
    // response has been sent -- this guarantees resp0_data/resp1_data can
    // never be overwritten by a second read before the first is sent out.
    wire dispatch_ok = pend_valid &&
                       (!need_m0 || m0_txn_ready_i) &&
                       (!need_m1 || m1_txn_ready_i) &&
                       !resp0_valid && !resp1_valid;

    // ------------------------------------------------------------------
    // TX sequencer: drains resp0 then resp1 into 5-byte response frames.
    //   byte0 [7:0]   = 8'hAA
    //   byte1 [15:8]  = { master_id[1:0], 6'b0 }
    //   byte2 [23:16] = rdata[7:0]
    //   byte3 [31:24] = 8'h00  (reserved)
    //   byte4 [39:32] = 8'h55
    // ------------------------------------------------------------------
    localparam TX_IDLE = 1'b0, TX_BUSY = 1'b1;
    reg tx_state;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            pend_valid <= 1'b0;
            pend_sel   <= 2'b00;
            pend_we    <= 1'b0;
            pend_addr  <= {ADDR_W{1'b0}};
            pend_wdata <= {DATA_W{1'b0}};

            m0_txn_valid_o <= 1'b0;
            m0_txn_addr_o  <= {ADDR_W{1'b0}};
            m0_txn_we_o    <= 1'b0;
            m0_txn_wdata_o <= {DATA_W{1'b0}};

            m1_txn_valid_o <= 1'b0;
            m1_txn_addr_o  <= {ADDR_W{1'b0}};
            m1_txn_we_o    <= 1'b0;
            m1_txn_wdata_o <= {DATA_W{1'b0}};

            resp0_valid <= 1'b0;
            resp0_data  <= {DATA_W{1'b0}};
            resp1_valid <= 1'b0;
            resp1_data  <= {DATA_W{1'b0}};

            r_Tx_DV   <= 1'b0;
            r_Tx_Word <= 40'b0;
            tx_state  <= TX_IDLE;

        end else begin
            // pulses: default low each cycle
            m0_txn_valid_o <= 1'b0;
            m1_txn_valid_o <= 1'b0;
            r_Tx_DV        <= 1'b0;

            // ---- latch a new request frame ----
            // Only accept a new frame once the previous one has been
            // dispatched. Given the long gaps between host packets you
            // described, this never stalls real traffic; it just refuses
            // to overwrite a request that hasn't gone out to the bus yet.
            if (w_Rx_DV && frame_ok && !pend_valid) begin
                pend_valid <= 1'b1;
                pend_sel   <= rx_sel;
                pend_we    <= rx_we;
                pend_addr  <= rx_addr;
                pend_wdata <= rx_wdata;
            end

            // ---- dispatch to target master(s), simultaneously ----
            if (dispatch_ok) begin
                if (need_m0) begin
                    m0_txn_valid_o <= 1'b1;
                    m0_txn_addr_o  <= pend_addr;
                    m0_txn_we_o    <= pend_we;
                    m0_txn_wdata_o <= pend_wdata;
                end
                if (need_m1) begin
                    m1_txn_valid_o <= 1'b1;
                    m1_txn_addr_o  <= pend_addr;
                    m1_txn_we_o    <= pend_we;
                    m1_txn_wdata_o <= pend_wdata;
                end
                pend_valid <= 1'b0;
            end

            // ---- capture read completions ----
            if (m0_rdata_valid_i) begin
                resp0_valid <= 1'b1;
                resp0_data  <= m0_rdata_i;
            end
            if (m1_rdata_valid_i) begin
                resp1_valid <= 1'b1;
                resp1_data  <= m1_rdata_i;
            end

            // ---- TX sequencer: send resp0 before resp1 if both pending ----
            case (tx_state)
                TX_IDLE: begin
                    if (resp0_valid) begin
                        r_Tx_Word   <= {8'h55, 8'h00, resp0_data, {2'b00, 6'b0}, 8'hAA};
                        r_Tx_DV     <= 1'b1;
                        resp0_valid <= 1'b0;
                        tx_state    <= TX_BUSY;
                    end else if (resp1_valid) begin
                        r_Tx_Word   <= {8'h55, 8'h00, resp1_data, {2'b01, 6'b0}, 8'hAA};
                        r_Tx_DV     <= 1'b1;
                        resp1_valid <= 1'b0;
                        tx_state    <= TX_BUSY;
                    end
                end

                TX_BUSY: begin
                    if (w_Tx_Done) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
