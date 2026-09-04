// addr_serializer: the TX-side mirror of addr_redirect.v. Packs one master's
// {addr,we} field and write-data byte into a single FRAME_W-bit frame
// ({addr_i, wdata_i}, MSB first - exactly the layout addr_redirect expects:
// frame[FRAME_W-1:DATA_W] = {addr,we}, frame[DATA_W-1:0] = data) and shifts
// it out one bit per cycle on serial_out.
//
// Unlike Serializer.v's data_valid_out (held high for the whole
// transmission), frame_valid_out here is a single one-cycle start pulse,
// asserted the same cycle serial_out first presents the frame's MSB. That
// match is required because addr_redirect only samples serial_in as bit 0
// of a new frame on the cycle frame_valid_in pulses while idle - it never
// looks at frame_valid_in again until the next frame.
//
// Both outputs are registered (posedge-clocked only, no combinational
// bypass), so the MSB appears one cycle after valid_i rather than the same
// cycle. That extra cycle is harmless: addr_redirect's own OUT_DELAY shift
// register buffers serial_in/frame_valid_in together every cycle before
// forwarding to the slaves, so it tolerates any fixed start-up latency here
// as long as serial_out and frame_valid_out stay aligned to each other.
module addr_serializer #(
    parameter ADDR_W  = 15,
    parameter RW      = 1,
    parameter DATA_W  = 8,
    parameter FRAME_W = ADDR_W + RW + DATA_W   // full serial frame width (24)
)(
    input  wire clk,
    input  wire rst_n,

    input  wire [ADDR_W+RW-1:0] addr_i,    // {addr, we}, LSB carries we
    input  wire [DATA_W-1:0]    wdata_i,
    input  wire                 valid_i,   // pulse: start a new frame (sampled while idle)

    output reg  serial_out,        // serial frame line, MSB first
    output reg  frame_valid_out    // 1-cycle start strobe, same cycle as the MSB
);

    localparam CNT_W = $clog2(FRAME_W);

    wire [FRAME_W-1:0] frame_data = {addr_i, wdata_i};  // continuous view of the current inputs, sampled only at start

    reg [FRAME_W-1:0] frame_in;      // doubles as the shift register once loaded
    reg [CNT_W-1:0]   bit_counter;
    reg               transmitting;

    wire idle  = !transmitting;
    wire start = valid_i && idle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_in        <= {FRAME_W{1'b0}};
            bit_counter     <= {CNT_W{1'b0}};
            transmitting    <= 1'b0;
            frame_valid_out <= 1'b0;
            serial_out      <= 1'b0;
        end else begin
            frame_valid_out <= start; // pulses exactly the cycle serial_out first shows the MSB

            if (start) begin
                serial_out   <= frame_data[FRAME_W-1];
                frame_in     <= {frame_data[FRAME_W-2:0], 1'b0};
                bit_counter  <= {CNT_W{1'b0}};
                transmitting <= 1'b1;
            end else if (transmitting) begin
                serial_out <= frame_in[FRAME_W-1];
                if (bit_counter < FRAME_W-2) begin
                    bit_counter <= bit_counter + 1'b1;
                    frame_in    <= {frame_in[FRAME_W-2:0], 1'b0};
                end else begin
                    transmitting <= 1'b0; // last bit was just presented
                end
            end
        end
    end

endmodule
