`timescale 1ns / 1ps
module top_loopback #(
    parameter CLKS_PER_BIT = 13021   // 125 MHz / 9600 baud - set to match CoolTerm
)(
    input  wire clk,
    input  wire rst,        // active-HIGH: matches a Zybo push-button (idle=0, pressed=1)
    input  wire rx_serial,
    output wire tx_serial,
    output wire [3:0] led   // optional: shows low nibble of last received byte
);
    // UART RX
    wire        rx_data_valid;
    wire [47:0] rx_word;

    receiver #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_receiver (
        .i_Clock    (clk),
        .i_Rx_Serial(rx_serial),
        .o_Rx_DV    (rx_data_valid),
        .o_Rx_Word  (rx_word)
    );

    // receiver has no led port of its own - latch the low nibble of the
    // last received byte here instead, held until the next word arrives.
    reg [3:0] led_r;
    always @(posedge clk) begin
        if (rst) led_r <= 4'b0;
        else if (rx_data_valid) led_r <= rx_word[3:0];
    end
    assign led = led_r;

    // UART TX
    reg        tx_data_valid;
    reg [39:0] tx_word;
    wire tx_active;
    wire tx_done;

    transmitter #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_transmitter (
        .i_Clock     (clk),
        .i_Tx_DV     (tx_data_valid),
        .i_Tx_Word   (tx_word),
        .o_Tx_Active (tx_active),
        .o_Tx_Serial (tx_serial),
        .o_Tx_Done   (tx_done)
    );

    // Loopback FSM
    localparam [1:0]
        S_WAIT_RX  = 2'd0,
        S_START_TX = 2'd1,
        S_WAIT_TX  = 2'd2;
    reg [1:0] state;

    // Synchronous, active-HIGH reset - matches a physical button directly
    always @(posedge clk) begin
        if (rst) begin
            state         <= S_WAIT_RX;
            tx_data_valid <= 1'b0;
            tx_word       <= 40'b0;
        end else begin
            tx_data_valid <= 1'b0; // one-clock pulse, default low
            case (state)
                // Wait until receiver has collected all 6 bytes
                S_WAIT_RX: begin
                    if (rx_data_valid) begin
                        // Drop byte0 (first byte received), forward
                        // bytes 1-5 (the last 5 received) in order.
                        tx_word <= {
                            rx_word[47:40],
                            rx_word[39:32],
                            rx_word[31:24],
                            rx_word[23:16],
                            rx_word[15:8]
                        };
                        state <= S_START_TX;
                    end
                end

                // Wait until transmitter is idle, then kick it off
                S_START_TX: begin
                    if (!tx_active) begin
                        tx_data_valid <= 1'b1;
                        state         <= S_WAIT_TX;
                    end
                end

                // Wait for the 5-byte transmission to finish
                S_WAIT_TX: begin
                    if (tx_done)
                        state <= S_WAIT_RX;
                end

                default: state <= S_WAIT_RX;
            endcase
        end
    end
endmodule