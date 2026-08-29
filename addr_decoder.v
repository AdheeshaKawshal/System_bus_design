module addr_decoder #(
    parameter ADDR_W     = 15,
    parameter NUM_SLAVES = 3,
    parameter RW         = 1,
    parameter ADDR_LINE_W        = ADDR_W + RW
)(
    input  wire [ADDR_LINE_W-1:0] addr_i,      // address driven on the bus
    input  wire                   valid_i,     // bus trigger: address is valid this cycle

    output reg                    slave_sel1,  // 1 = sel == 00
    output reg                    slave_sel2,  // 1 = sel == 01
    output reg                    slave_sel3,  // 1 = sel == 10

    output reg  [ADDR_LINE_W-1:0] addr_ext_o,  // forwarded address, msb forced 0
    output reg                    ext_valid_o, // 1 = addr_i is outside the bus
    output reg [ADDR_LINE_W-4:0]  addr_o,           // addr to slave (without sel bits and we bit)
    output reg                      we,          // write enable to
    output reg                    addr_invalid
);

    // addr_i[MSB]      : 1 = inside this bus, 0 = outside (goes off-bus)
    // addr_i[MSB-1:MSB-2]: slave select within the bus (up to 4 slaves)
    localparam SEL_W = 2;

    wire  is_internal            = !addr_i[ADDR_LINE_W-1];
    wire [SEL_W-1:0] sel         = addr_i[ADDR_LINE_W-2 : ADDR_LINE_W-2-SEL_W];

    reg [NUM_SLAVES-1:0] slave_sel;

    always @* begin
        addr_ext_o  = {ADDR_LINE_W{1'b0}};
        ext_valid_o = 1'b0;
        addr_invalid = 1'b0;
        slave_sel   = {NUM_SLAVES{1'b0}};

        if (valid_i) begin
            if (is_internal) begin
                we = addr_i[0]; // LSB of addr_i indicates read/write
                slave_sel1 = slave_sel[0];
                slave_sel2 = slave_sel[1];
                slave_sel3 = slave_sel[2];
                case (sel)
                    2'b00:   slave_sel = 3'b001;
                    2'b01:   slave_sel = 3'b010;
                    2'b10:   slave_sel = 3'b100;
                    2'b11:   addr_invalid = 1'b1; // invalid address
                    default: slave_sel = 3'b000;
                endcase
            end else begin
                addr_ext_o  = {1'b0, addr_i[ADDR_LINE_W-3:0]}; // send out with msb = 0
                ext_valid_o = 1'b1;
            end
        end
    end

endmodule
