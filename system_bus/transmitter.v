`timescale 1ns / 1ps
module transmitter #(
    parameter CLKS_PER_BIT = 13021   // must match receiver's baud rate
)(
    input  wire        i_Clock,
    input  wire        i_Tx_DV,
    input  wire [39:0] i_Tx_Word,
    output wire        o_Tx_Active,
    output reg         o_Tx_Serial,
    output wire        o_Tx_Done
);
    localparam [2:0]
        s_IDLE         = 3'b000,
        s_TX_START_BIT = 3'b001,
        s_TX_DATA_BITS = 3'b010,
        s_TX_STOP_BIT  = 3'b011,
        s_NEXT_BYTE    = 3'b100;

    reg [2:0]  r_SM_Main     = s_IDLE;
    reg [20:0] r_Clock_Count = 0;   // widened to match CLKS_PER_BIT range safely
    reg [3:0]  r_Bit_Index   = 0;
    reg [2:0]  r_Byte_Index  = 0;
    reg [7:0]  r_Tx_Byte     = 0;
    reg [39:0] r_Tx_Word     = 0;
    reg        r_Tx_Active   = 0;
    reg        r_Tx_Done     = 0;

    always @(posedge i_Clock) begin
        r_Tx_Done <= 1'b0; // default
        case (r_SM_Main)
            s_IDLE: begin
                o_Tx_Serial   <= 1'b1;
                r_Tx_Active   <= 1'b0;
                r_Clock_Count <= 0;
                r_Bit_Index   <= 0;
                r_Byte_Index  <= 0;
                if (i_Tx_DV) begin
                    r_Tx_Word   <= i_Tx_Word;
                    r_Tx_Byte   <= i_Tx_Word[7:0];
                    r_Tx_Active <= 1'b1;
                    r_SM_Main   <= s_TX_START_BIT;
                end
            end

            s_TX_START_BIT: begin
                o_Tx_Serial <= 1'b0;
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    r_SM_Main     <= s_TX_DATA_BITS;
                end
            end

            s_TX_DATA_BITS: begin
                o_Tx_Serial <= r_Tx_Byte[r_Bit_Index];
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    if (r_Bit_Index < 7) begin
                        r_Bit_Index <= r_Bit_Index + 1;
                    end else begin
                        r_Bit_Index <= 0;
                        r_SM_Main   <= s_TX_STOP_BIT;
                    end
                end
            end

            s_TX_STOP_BIT: begin
                o_Tx_Serial <= 1'b1;
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    r_SM_Main     <= s_NEXT_BYTE;
                end
            end

            s_NEXT_BYTE: begin
                if (r_Byte_Index < 4) begin
                    r_Byte_Index <= r_Byte_Index + 1;
                    r_Tx_Byte    <= r_Tx_Word[(r_Byte_Index+1)*8 +: 8];
                    r_SM_Main    <= s_TX_START_BIT;
                end else begin
                    r_Tx_Active <= 1'b0;
                    r_Tx_Done   <= 1'b1; // full 40-bit word sent
                    r_SM_Main   <= s_IDLE;
                end
            end

            default: r_SM_Main <= s_IDLE;
        endcase
    end

    assign o_Tx_Active = r_Tx_Active;
    assign o_Tx_Done   = r_Tx_Done;
endmodule