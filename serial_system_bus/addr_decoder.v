// addr_decoder: picks a slave from just the 4-bit early tap of a request
// frame (external-flag + a 3-bit slave-select field) - everything
// addr_redirect needs to route a frame, since the rest of the frame is now
// delayed and forwarded to the slave verbatim rather than being re-decoded
// here.
//
// The 3-bit select field is matched with don't-cares on its LSB for
// slave1/slave2 (3'b00x / 3'b01x - the extra bit is unused/free), and an
// exact match for slave3 (3'b100):
//   3'b00x -> slave1
//   3'b01x -> slave2
//   3'b100 -> slave3
//   anything else (3'b101/110/111) -> addr_invalid
//
// An address whose external-flag bit (sel_bits' MSB, i.e. addr[14]) is set
// is routed to its own dedicated select, ext_redirect, instead - the 3-bit
// field is not even inspected in that case. ext_redirect is mutually
// exclusive with slave_sel1/2/3 (never asserted alongside any of them), so
// the external bridge slave (bb_slave_core) and the genuine internal
// 3'b100 slave never collide on the same chip select.
module addr_decoder #(
    parameter NUM_SLAVES = 3,
    parameter SEL_W      = 3,
    parameter TAP_BITS   = SEL_W + 1   // external-flag + slave-select bits
)(
    input  wire [TAP_BITS-1:0] sel_bits,  // {external_flag, slave_sel[2:0]}, MSB first
    input  wire                valid_i,

    output reg  slave_sel1,    // 1 = sel == 3'b00x (internal)
    output reg  slave_sel2,    // 1 = sel == 3'b01x (internal)
    output reg  slave_sel3,    // 1 = sel == 3'b100 (internal), or external_flag == 1 (external, routed here)
    output reg  ext_redirect,  // 1 alongside slave_sel3 only for the external case - tells S2 to bridge, not service
    output reg  addr_invalid   // 1 = sel matches none of the above (internal)
);

    wire             is_external = sel_bits[TAP_BITS-1];
    wire [SEL_W-1:0] sel         = sel_bits[SEL_W-1:0];

    reg [NUM_SLAVES-1:0] slave_sel;

    always @* begin
        addr_invalid = 1'b0;
        ext_redirect = 1'b0;
        slave_sel    = {NUM_SLAVES{1'b0}};

        if (valid_i) begin
            if (is_external) begin
                // external (addr[14]=1) -> ext_redirect only; slave_sel
                // stays all-zero so slave_sel1/2/3 never assert here.
                ext_redirect = 1'b1;
            end else begin
                casez (sel)
                    3'b00?:  slave_sel = 3'b001;
                    3'b01?:  slave_sel = 3'b010;
                    3'b100:  slave_sel = 3'b100;
                    default: addr_invalid = 1'b1;
                endcase
            end
        end

        slave_sel1 = slave_sel[0];
        slave_sel2 = slave_sel[1];
        slave_sel3 = slave_sel[2];
    end

endmodule
