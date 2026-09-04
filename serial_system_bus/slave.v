module slave #(
    parameter ADDR_W  = 12,
    parameter DATA_W  = 8,
    parameter RW      = 1
)(
    input wire clk,
    input wire rst,   // active-low, matches this module's pre-existing convention

    input wire cs_i,        // chip select from the bus (slave_selN)
    input wire addr_data_i, // shared serial {addr,we,wdata} request frame, MSB first
    input wire valid_i,     // shared frame-start strobe

    output wire rdata_o_ser, // serial rdata response, MSB first
    output wire rvalid_o     // held-high-during-frame response valid (Serializer's data_valid_out)
);

    reg [DATA_W-1:0] mem [0:15];

    // ---------------------------------------------------------
    // Frame capture: shared addr_data_deserializer does the shift-register
    // work, gated by cs_i (stable ahead of the data per the bus's
    // OUT_DELAY design).
    // ---------------------------------------------------------
    wire              we_c;
    wire [14:0]       addr_c;
    wire [DATA_W-1:0] wdata_c;
    wire              frame_done;

    addr_data_deserializer #(
        .ADDR_W (15),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_deserializer (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (cs_i),
        .addr_data_i (addr_data_i),
        .valid_i     (valid_i),
        .we_o        (we_c),
        .addr_o      (addr_c),
        .wdata_o     (wdata_c),
        .frame_done  (frame_done)
    );

    // ---------------------------------------------------------
    // Mem access + read-response trigger, one cycle after frame_done (i.e.
    // once addr_c/we_c/wdata_c are stable) - same write-on-write/read-into-
    // register behaviour as the original parallel slave.v.
    // ---------------------------------------------------------
    reg [DATA_W-1:0] rdata_reg;
    reg              ser_trigger;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_reg   <= {DATA_W{1'b0}};
            ser_trigger <= 1'b0;
        end else begin
            ser_trigger <= 1'b0;
            if (frame_done) begin
                if (we_c) begin
                    mem[addr_c[3:0]] <= wdata_c;
                end else begin
                    rdata_reg   <= mem[addr_c[3:0]];
                    ser_trigger <= 1'b1;
                end
            end
        end
    end

    // For a write there is nothing to send back: Serializer is simply never
    // triggered, so rdata_o_ser/rvalid_o stay idle (serial_out driven low,
    // data_valid_out low) until the next read.
    Serializer u_serializer (
        .clk_in      (clk),
        .rst_n       (rst),
        .data_in     (rdata_reg),
        .data_valid  (ser_trigger),
        .serial_out  (rdata_o_ser),
        .data_valid_out (rvalid_o),
        .done        ()
    );

endmodule
