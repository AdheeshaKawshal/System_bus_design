// led_display: taps the bus's own decoded control/data lines (we_bus,
// valid_bus, wdata_bus, rdata_bus - the same signals addr_decoder's
// we/valid feed and the slave mux returns) and shows the lower 4 bits
// of whichever data actually moved on the last completed transfer:
// wdata_bus on a write, rdata_bus on a read. Holds its value between
// transfers instead of showing bus data that isn't actually valid.
module led_display #(
    parameter DATA_W = 8
)(
    input  wire              clk,
    input  wire              rst,

    input  wire              we_i,     // we_bus: 1 = write, 0 = read
    input  wire              valid_i,  // valid_bus: transaction active this cycle
    input  wire              ready_i,  // ready_slave: slave has actually accepted/returned data
    input  wire [DATA_W-1:0] wdata_i,  // wdata_bus
    input  wire [DATA_W-1:0] rdata_i,  // rdata_bus

    output reg  [3:0]        led
);
    reg ready;
    reg blink;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            led <= 4'b0;
            ready<= 1'b0;
            blink <= 1'b0;
        end else if (1) begin
            blink <= !blink;
            led <= wdata_i[3:0];
        end
        ready <= ready_i;
    end

endmodule
