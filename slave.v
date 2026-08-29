module slave #(
    parameter ADDR_W = 12,
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,

    input wire                  cs_i,     // chip select from addr_decoder
    input wire                  valid_i,  // master has a valid transaction
    input wire                  we_i,     // 1 = write, 0 = read
    input wire [ADDR_W-1:0]     addr_i,
    input wire [DATA_W-1:0]     wdata_i,

    output reg [DATA_W-1:0]     rdata_o,  // data back to master on a read
    output reg                  ready_o   // pulses when the transaction is done
);

    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    wire sel = cs_i && valid_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_o <= {DATA_W{1'b0}};
            ready_o <= 1'b0;
        end else begin
            ready_o <= 1'b0;

            if (sel) begin
                if (we_i) begin
                    mem[addr_i] <= wdata_i;
                end else begin
                    rdata_o <= mem[addr_i];
                end
                ready_o <= 1'b1;
            end
        end
    end

endmodule
