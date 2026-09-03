// ============================================================================
// uart_frame_tx.v
// ----------------------------------------------------------------------------
// UART transmitter that sends a WIDTH-bit word as ONE frame: a single start
// bit, WIDTH data bits, a single stop bit. Used by:
//   - bb_slave_core  : WIDTH = 24, the whole {header, addr_lo, wdata} request
//   - bb_master_core : WIDTH = 8,  a single read-reply byte
//
// This replaces the earlier uart_byte_tx, which framed every 8 bits
// separately. Sending 24 bits as one frame costs 26 bit-times instead of 30,
// and -- more usefully -- removes byte alignment from the protocol entirely:
// a frame either arrives whole or is rejected, so the receiver can never end
// up permanently offset by one byte the way a byte-counting receiver can.
//
// The cost is clock-drift tolerance. The receiver only re-centres its
// sampling at the start bit, so error accumulates across the frame as
// (bit position x frequency error). Staying within half a bit at the last bit
// of a 26-bit frame needs the two ends within roughly 2% of each other; the
// 10-bit framing it replaces allowed roughly 5%. Both boards run from crystal
// oscillators (tens of ppm), so this is a non-issue here -- but it would
// matter if either end were ever clocked from an internal RC oscillator.
//
// Bit order: LSB-first (data_i[0] goes out first), matching the convention in
// the project's existing deserializer.v UART.
//
// Baud generation: CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE, computed at
// elaboration. Defaults are the real hardware numbers (125 MHz / 100 kbaud
// -> CLKS_PER_BIT = 1250); override with small values for fast simulation.
//
// Handshake: pulse send_i for one cycle with data_i stable. busy_o stays high
// for the whole frame and send_i is ignored while it is. done_o pulses for one
// cycle once the stop bit has finished.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module uart_frame_tx #(
    parameter CLK_FREQ_HZ = 125000000,  // 125 MHz board clock
    parameter BAUD_RATE   = 100000,     // -> CLKS_PER_BIT = 1250
    parameter WIDTH       = 8           // data bits per frame
)(
    input  wire             clk,
    input  wire             rst,        // active-low

    input  wire [WIDTH-1:0] data_i,     // sampled on send_i
    input  wire             send_i,     // 1-cycle pulse: start sending data_i

    output reg              tx_o,       // serial line out (idles high)
    output reg              busy_o,     // high for the whole frame
    output reg              done_o      // 1-cycle pulse when the frame has been sent
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam         BITCNT_W     = $clog2(CLKS_PER_BIT + 1);
    localparam         IDXW         = (WIDTH <= 1) ? 1 : $clog2(WIDTH);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]          state;
    reg [BITCNT_W-1:0] clk_cnt;
    reg [IDXW-1:0]     bit_idx;
    reg [WIDTH-1:0]    shift_reg;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= {WIDTH{1'b1}};
            tx_o      <= 1'b1;   // idle high
            busy_o    <= 1'b0;
            done_o    <= 1'b0;
        end else begin
            done_o <= 1'b0; // default: 1-cycle pulse

            case (state)
                S_IDLE: begin
                    tx_o <= 1'b1;
                    if (send_i) begin
                        shift_reg <= data_i;
                        busy_o    <= 1'b1;
                        clk_cnt   <= 0;
                        bit_idx   <= 0;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    tx_o <= 1'b0;   // one start bit for the whole frame
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    tx_o <= shift_reg[bit_idx];   // LSB first
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        if (bit_idx == WIDTH-1) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    tx_o <= 1'b1;   // one stop bit for the whole frame
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        busy_o  <= 1'b0;
                        done_o  <= 1'b1;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
