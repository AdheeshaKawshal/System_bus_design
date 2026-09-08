module bb_slave_core #(
    parameter CLK_FREQ_HZ = 125000000,     // passed through to the UART primitives
    parameter BAUD_RATE   = 2000000,
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

    localparam R_IDLE       = 2'd0,
               R_SEND       = 2'd1,   // uart_frame_tx shifting the 24-bit packet out
               R_WAIT_REPLY = 2'd2;   // reads only: wait for uart_frame_rx or RX_TIMEOUT

    reg [1:0]  rstate;
    reg        r_we_latched;
    reg [7:0]  r_rdata;
    reg        r_rvalid;
    reg [31:0] rx_timeout_cnt;

    // addr[14] is forced to 0 in the outgoing header -- see file header.
    (* MARK_DEBUG = "TRUE" *) wire [23:0] pkt = { {we_c, 1'b0, addr_c[13:8]},   // byte 0: header
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

    (* MARK_DEBUG = "TRUE" *) wire [7:0] rx_data;
    (* MARK_DEBUG = "TRUE" *) wire       rx_valid;

    uart_frame_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .WIDTH       (8)          // the reply is a single rdata byte
    ) u_reply_rx (
        .clk         (clk),
        .rst         (rst),
        .rx_i        (uart_rx_i),
        .data_o      (rx_data),
        .valid_o     (rx_valid)
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
                    if (frame_done && !tx_busy) begin
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
                    end
                    // end else if (rx_timeout_cnt >= RX_TIMEOUT) begin
                    //     timeout_o <= 1'b1;        // sticky, only cleared by rst
                    //     rstate    <= R_IDLE;
                    // end else begin
                    //     rx_timeout_cnt <= rx_timeout_cnt + 1'b1;
                    // end
                end

                default: rstate <= R_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Serializer: turns the remote reply byte into the serial
    // rdata_o_ser/rvalid_o pair, same as slave.v's own response path.
    // ------------------------------------------------------------------
    Serializer u_serializer (
        .clk_in         (clk),
        .rst_n          (rst),
        .data_in        (r_rdata),
        .data_valid     (r_rvalid),
        .serial_out     (rdata_o_ser),
        .data_valid_out (rvalid_o),
        .done           ()
    );

endmodule
