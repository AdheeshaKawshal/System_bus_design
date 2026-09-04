// ============================================================================
// uart_frame_rx.v
// ----------------------------------------------------------------------------
// UART receiver matching uart_frame_tx: one start bit, WIDTH data bits, one
// stop bit, delivered as a single parallel word. Used by:
//   - bb_master_core : WIDTH = 24, the whole {header, addr_lo, wdata} request
//   - bb_slave_core  : WIDTH = 8,  a single read-reply byte
//
// Because a frame is indivisible there is no byte counter and no byte
// alignment to lose -- the failure mode of the byte-framed receiver this
// replaces (one dropped byte leaves it permanently offset, misassembling
// every later packet) cannot occur. A corrupted frame fails the stop-bit
// check and is discarded whole.
//
// Metastability : rx_i is double-registered before it reaches the FSM.
// Start-bit check: after a falling edge, the FSM waits to the midpoint of the
//                  candidate start bit and re-checks it is still low, so a
//                  glitch is rejected rather than treated as a start.
// Sampling      : every data bit and the stop bit are sampled at their centre,
//                  using the counter re-centred by that start-bit check.
// Stop-bit check: the stop bit must read high. If it does not, the frame was
//                  mis-framed (wrong baud, noise, a mid-frame reset on the
//                  far side) and is dropped without pulsing valid_o -- better
//                  than handing a garbage transaction to the bus. See the
//                  drift note in uart_frame_tx.v for how much timing error a
//                  WIDTH-bit frame tolerates before the stop bit lands wrong.
//
// Bit order: LSB-first, matching uart_frame_tx.
//
// Reset: active-low, posedge clk / negedge rst (project convention).
// ============================================================================
module uart_frame_rx #(
    parameter CLK_FREQ_HZ = 125000000,  // 125 MHz board clock
    parameter BAUD_RATE   = 100000,     // -> CLKS_PER_BIT = 1250
    parameter WIDTH       = 8           // data bits per frame
)(
    input  wire              clk,
    input  wire              rst,       // active-low

    input  wire              rx_i,      // asynchronous serial input pin

    output reg  [WIDTH-1:0]  data_o,    // valid the same cycle as valid_o
    output reg               valid_o,   // 1-cycle pulse when a whole frame has arrived
    output reg               frame_err_o // sticky: a frame was dropped on a bad stop bit
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer HALF_BIT     = (CLKS_PER_BIT-1)/2;
    localparam         BITCNT_W     = $clog2(CLKS_PER_BIT + 1);
    localparam         IDXW         = (WIDTH <= 1) ? 1 : $clog2(WIDTH);

    localparam S_IDLE      = 2'd0,
               S_START_BIT = 2'd1,
               S_DATA_BITS = 2'd2,
               S_STOP_BIT  = 2'd3;

    // ---- double-register rx_i to remove metastability ---------------------
    reg rx_meta, rx_sync;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx_i;
            rx_sync <= rx_meta;
        end
    end

    reg [1:0]          state;
    reg [BITCNT_W-1:0] clk_cnt;
    reg [IDXW-1:0]     bit_idx;
    reg [WIDTH-1:0]    shift_reg;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state       <= S_IDLE;
            clk_cnt     <= 0;
            bit_idx     <= 0;
            shift_reg   <= {WIDTH{1'b0}};
            data_o      <= {WIDTH{1'b0}};
            valid_o     <= 1'b0;
            frame_err_o <= 1'b0;
        end else begin
            valid_o <= 1'b0; // default: 1-cycle pulse

            case (state)
                S_IDLE: begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    if (rx_sync == 1'b0) begin
                        // possible start bit -- confirmed at its midpoint below
                        state <= S_START_BIT;
                    end
                end

                S_START_BIT: begin
                    if (clk_cnt == HALF_BIT) begin
                        if (rx_sync == 1'b0) begin
                            // real start bit: re-centre the counter so every
                            // following bit is sampled at ITS midpoint too
                            clk_cnt   <= 0;
                            shift_reg <= {WIDTH{1'b0}};
                            state     <= S_DATA_BITS;
                        end else begin
                            state <= S_IDLE; // was just a glitch
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_DATA_BITS: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        shift_reg[bit_idx] <= rx_sync;   // centre sample, LSB first
                        if (bit_idx == WIDTH-1) begin
                            bit_idx <= 0;
                            state   <= S_STOP_BIT;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_STOP_BIT: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        if (rx_sync == 1'b1) begin
                            data_o  <= shift_reg;
                            valid_o <= 1'b1;      // whole frame received cleanly
                        end else begin
                            frame_err_o <= 1'b1;  // sticky, only cleared by rst
                        end
                        state <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
