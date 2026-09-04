// addr_decoder: picks a slave from just the 3-bit early tap of a request
// frame (internal-flag + 2 slave-select bits) - everything addr_redirect
// needs to route a frame, since the rest of the frame is now delayed and
// forwarded to the slave verbatim rather than being re-decoded here.
//
// There's no external/off-bus port anymore: an address whose internal-flag
// bit says "external" is routed to slave 3 (S2), which acts as the
// bridge/gateway slave - same slave_sel3 a genuine sel==10 internal access
// would get. ext_redirect is the extra bit that tells slave 3 apart the two
// cases: high only for the external case, so the bridge slave knows to
// forward this particular transaction on rather than service it itself.
module addr_decoder #(
    parameter NUM_SLAVES = 3,
    parameter SEL_W      = 2,
    parameter TAP_BITS   = SEL_W + 1   // internal-flag + slave-select bits
)(
    input  wire [TAP_BITS-1:0] sel_bits,  // {internal_flag, slave_sel[1:0]}, MSB first
    input  wire                valid_i,

    output reg  slave_sel1,    // 1 = sel == 00 (internal)
    output reg  slave_sel2,    // 1 = sel == 01 (internal)
    output reg  slave_sel3,    // 1 = sel == 10 (internal), or internal_flag == 0 (external, routed here)
    output reg  ext_redirect,  // 1 alongside slave_sel3 only for the external case - tells S2 to bridge, not service
    output reg  addr_invalid   // 1 = sel == 11 (internal)
);

    wire             is_internal = sel_bits[TAP_BITS-1];
    wire [SEL_W-1:0] sel         = sel_bits[SEL_W-1:0];

    reg [NUM_SLAVES-1:0] slave_sel;

    always @* begin
        addr_invalid = 1'b0;
        ext_redirect = 1'b0;
        slave_sel    = {NUM_SLAVES{1'b0}};

        if (valid_i) begin
            if (!is_internal) begin
                slave_sel    = 3'b100;  // external -> slave 3 (bridge/gateway)
                ext_redirect = 1'b1;
            end else begin
                case (sel)
                    2'b00:   slave_sel = 3'b001;
                    2'b01:   slave_sel = 3'b010;
                    2'b10:   slave_sel = 3'b100;
                    2'b11:   addr_invalid = 1'b1;
                    default: slave_sel = 3'b000;
                endcase
            end
        end

        slave_sel1 = slave_sel[0];
        slave_sel2 = slave_sel[1];
        slave_sel3 = slave_sel[2];
    end

endmodule
