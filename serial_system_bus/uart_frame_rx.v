module uart_frame_rx #(
    parameter CLK_FREQ_HZ = 125000000,  // 125 MHz board clock
    parameter BAUD_RATE   = 100000,     // -> CLKS_PER_BIT = 1250
    parameter WIDTH       = 8           // data bits per frame
)(
    input  wire              clk,
    input  wire              rst,       // active-low

    input  wire              rx_i,      // asynchronous serial input pin

    output reg  [WIDTH-1:0]  data_o,    // valid the same cycle as valid_o
    output reg               valid_o  // 1-cycle pulse when a whole frame has arrived
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
