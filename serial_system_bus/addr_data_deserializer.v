// addr_data_deserializer: captures the shared 24-bit serial {addr,we,wdata}
// request frame (the same frame addr_redirect.v forwards verbatim to every
// slave) and presents it in parallel once fully captured. Every slave-side
// module (slave.v, slave_split.v, slave_bridge.v) needs exactly this same
// capture logic gated by its own chip-select, so it lives here once instead
// of being duplicated in each of them.
module addr_data_deserializer #(
    parameter ADDR_W  = 15,
    parameter RW      = 1,
    parameter DATA_W  = 8,
    parameter FRAME_W = ADDR_W + RW + DATA_W  // full bus frame width (24)
)(
    input wire clk,
    input wire rst,   // active-low, matches the slave modules' convention

    input wire cs_i,         // chip select from the bus (gates when capture may start)
    input wire addr_data_i,  // shared serial {addr,we,wdata} request frame, MSB first
    input wire valid_i,      // shared frame-start strobe

    output reg               we_o,
    output reg [ADDR_W-1:0]  addr_o,
    output reg [DATA_W-1:0]  wdata_o,
    output reg               frame_done  // pulses 1 cycle once we_o/addr_o/wdata_o are valid
);

    reg [FRAME_W-1:0]       shift_reg;
    reg [$clog2(FRAME_W):0] bit_count;
    reg                     capturing;

    wire [FRAME_W-1:0] shift_next = {shift_reg[FRAME_W-2:0], addr_data_i};
    wire               sel        = cs_i && valid_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            shift_reg  <= {FRAME_W{1'b0}};
            bit_count  <= 0;
            capturing  <= 1'b0;
            frame_done <= 1'b0;
            we_o       <= 1'b0;
            addr_o     <= {ADDR_W{1'b0}};
            wdata_o    <= {DATA_W{1'b0}};
        end else begin
            frame_done <= 1'b0;

            if (!capturing) begin
                if (sel) begin
                    shift_reg <= shift_next;
                    bit_count <= 1;
                    capturing <= 1'b1;
                end
            end else begin
                shift_reg <= shift_next;
                bit_count <= bit_count + 1'b1;

                // captured frame layout, MSB first: [FRAME_W-1:DATA_W+1]=addr,
                // [DATA_W]=we, [DATA_W-1:0]=wdata - matches addr_serializer.v's
                // packing on the transmit side.
                if ((bit_count + 1'b1) == FRAME_W) begin
                    we_o       <= shift_next[DATA_W];
                    addr_o     <= shift_next[FRAME_W-1:DATA_W+1];
                    wdata_o    <= shift_next[DATA_W-1:0];
                    frame_done <= 1'b1;
                    capturing  <= 1'b0;
                end
            end
        end
    end

endmodule
