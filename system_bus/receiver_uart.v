`timescale 1ns / 1ps
module receiver #(
    parameter CLKS_PER_BIT = 13021   // 125 MHz / 9600 baud
)(
    input  wire        i_Clock,
    input  wire        i_Rx_Serial,
    output reg         o_Rx_DV,
    output reg  [47:0] o_Rx_Word   // <-- now a real port
);
    localparam [2:0]
        s_IDLE         = 3'b000,
        s_RX_START_BIT = 3'b001,
        s_RX_DATA_BITS = 3'b010,
        s_RX_STOP_BIT  = 3'b011;

    reg [2:0]  r_SM_Main     = s_IDLE;
    reg        r_Rx_Data_R   = 1'b1;
    reg        r_Rx_Data     = 1'b1;
    reg [20:0] r_Clock_Count = 0;
    reg [2:0]  r_Bit_Index   = 0;
    reg [7:0]  r_Rx_Byte     = 0;
    reg [2:0]  byte_count    = 0;

    // Double register RX input to remove metastability
    always @(posedge i_Clock) begin
        r_Rx_Data_R <= i_Rx_Serial;
        r_Rx_Data   <= r_Rx_Data_R;
    end

    // UART RX FSM + byte/word assembly, all in ONE always block
    // so the store always uses byte_count's value from the SAME
    // edge as the stop-bit completion (no cross-block skew).
    always @(posedge i_Clock) begin
        o_Rx_DV <= 1'b0; // Default assignment
        case (r_SM_Main)
            s_IDLE: begin
                r_Clock_Count <= 0;
                r_Bit_Index   <= 0;
                if (r_Rx_Data == 1'b0)
                    r_SM_Main <= s_RX_START_BIT;
            end

            s_RX_START_BIT: begin
                if (r_Clock_Count == (CLKS_PER_BIT-1)/2) begin
                    if (r_Rx_Data == 1'b0) begin
                        r_Clock_Count <= 0;
                        r_Rx_Byte     <= 8'd0;
                        r_SM_Main     <= s_RX_DATA_BITS;
                    end else begin
                        r_SM_Main <= s_IDLE;
                    end
                end else begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end
            end

            s_RX_DATA_BITS: begin
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    r_Rx_Byte[r_Bit_Index] <= r_Rx_Data; // mid-bit sample

                    if (r_Bit_Index < 7) begin
                        r_Bit_Index <= r_Bit_Index + 1;
                    end else begin
                        r_Bit_Index <= 0;
                        r_SM_Main   <= s_RX_STOP_BIT;
                    end
                end
            end

            s_RX_STOP_BIT: begin
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;

                    case (byte_count)
                        3'd0: o_Rx_Word[7:0]   <= r_Rx_Byte;
                        3'd1: o_Rx_Word[15:8]  <= r_Rx_Byte;
                        3'd2: o_Rx_Word[23:16] <= r_Rx_Byte;
                        3'd3: o_Rx_Word[31:24] <= r_Rx_Byte;
                        3'd4: o_Rx_Word[39:32] <= r_Rx_Byte;
                        3'd5: begin
                            o_Rx_Word[47:40] <= r_Rx_Byte;                          
                        end
                    endcase

                    if (byte_count == 3'd5) begin
                        byte_count <= 0;
                        o_Rx_DV    <= 1'b1; // full 48-bit word ready
                    end else begin
                        byte_count <= byte_count + 1;
                    end
                    r_SM_Main <= s_IDLE;
                end
            end

            default: r_SM_Main <= s_IDLE;
        endcase
    end
endmodule