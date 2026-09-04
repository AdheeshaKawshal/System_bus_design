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

    output wire serial_out,        // serial frame line, MSB first
    output wire frame_valid_out    // 1-cycle start strobe, same cycle as the MSB
);

    localparam CNT_W = $clog2(FRAME_W);

    wire [FRAME_W-1:0] frame_in = {addr_i, wdata_i};

    reg [FRAME_W-1:0] shift_reg;
    reg [CNT_W-1:0]   bit_counter;
    reg               transmitting;

    wire idle  = !transmitting;
    wire start = valid_i && idle;

    // Bypass the register on the start cycle so the MSB appears on
    // serial_out the same cycle frame_valid_out pulses, before shift_reg
    // has even been loaded.
    assign frame_valid_out = start;
    assign serial_out      = start ? frame_in[FRAME_W-1] : shift_reg[FRAME_W-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg    <= {FRAME_W{1'b0}};
            bit_counter  <= {CNT_W{1'b0}};
            transmitting <= 1'b0;
        end else begin
            if (start) begin
                // Bit 0 (the MSB) already went out combinationally above;
                // load the register pre-shifted by one so next cycle's
                // shift_reg[FRAME_W-1] is bit 1.
                shift_reg    <= {frame_in[FRAME_W-2:0], 1'b0};
                bit_counter  <= {CNT_W{1'b0}};
                transmitting <= 1'b1;
            end else if (transmitting) begin
                if (bit_counter < FRAME_W-2) begin
                    bit_counter <= bit_counter + 1'b1;
                    shift_reg   <= {shift_reg[FRAME_W-2:0], 1'b0};
                end else begin
                    transmitting <= 1'b0; // last bit was just presented
                end
            end
        end
    end

endmodule
