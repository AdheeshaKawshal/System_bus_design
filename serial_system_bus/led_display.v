// Taps the shared slave-side request bus (addr_data_bus/valid_bus in
// serial_system_bus.v) with its own addr_data_deserializer, and the
// response bus (rdata_bus_ser/rvalid_bus - same wire both masters see,
// see serial_system_bus.v's rdata_M0_ser/rdata_M1_ser) with its own
// deserializer, so every request that reaches a slave - including one
// addr_redirect is about to bridge across to the other board via slave 3
// - can be watched here.
// Assumes only valid addresses are ever sent (no addr_invalid gating).
//
// led_o[3]   = we (1 = write, 0 = read) of the last captured frame
// led_o[2:0] = that frame's data[2:0]: wdata for a write (known
//              immediately, at frame_done), rdata for a read (known only
//              once the slave's response comes back, so led_o holds the
//              write/read bit alone until then).
//
// Held unchanged until the next frame completes (write) / next response
// arrives (read).
module led_display #(
    parameter ADDR_W = 15,
    parameter RW     = 1,
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,   // active-low

    input wire addr_data_bus,  // serial {addr,we,wdata} frame to the selected slave
    input wire valid_bus,      // frame-start strobe for addr_data_bus

    input wire rdata_bus_ser,  // serial rdata response frame, MSB first
    input wire rvalid_bus,     // response frame-valid (held-high style)

    output reg [3:0] led_o
);

    wire                  we_c;
    wire [ADDR_W-1:0]     addr_c;
    wire [DATA_W-1:0]     wdata_c;
    wire                  frame_done;

    addr_data_deserializer #(
        .ADDR_W (ADDR_W),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_deserializer (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (1'b1),
        .addr_data_i (addr_data_bus),
        .valid_i     (valid_bus),
        .we_o        (we_c),
        .addr_o      (addr_c),
        .wdata_o     (wdata_c),
        .frame_done  (frame_done)
    );

    // Response path: same edge-detect trick master.v uses, since
    // deserializer's data_valid stays high (not a single-cycle pulse)
    // once the byte is captured.
    wire [DATA_W-1:0] rdata_c;
    wire               rvalid_c;
    reg                rvalid_c_d;
    wire               rvalid_c_pulse = rvalid_c && !rvalid_c_d;

    deserializer u_resp_deserializer (
        .clk_in       (clk),
        .rst_n        (rst),
        .serial_in    (rdata_bus_ser),
        .data_valid_in(rvalid_bus),
        .data_out     (rdata_c),
        .data_valid   (rvalid_c)
    );

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            led_o      <= 4'b0;
            rvalid_c_d <= 1'b0;
        end else begin
            rvalid_c_d <= rvalid_c;

            if (frame_done && we_c) begin
                // Write: data is right there in the request frame.
                led_o <= {we_c, wdata_c[2:0]};
            end else if (frame_done && !we_c) begin
                // Read: update the we-bit now, data catches up below once
                // the slave actually responds.
                led_o[3] <= we_c;
            end

            if (rvalid_c_pulse) begin
                led_o[2:0] <= rdata_c[2:0];
            end
        end
    end

endmodule
